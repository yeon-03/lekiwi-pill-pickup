#!/usr/bin/env python3
"""abo(반려로봇) 명령 <-> Nav2 중개 노드.

abo 쪽은 문자열 토픽 하나만 알면 된다. 좌표도 Nav2 액션도 몰라도 된다.

  입력  /abo/command   std_msgs/String
          "home"                 -> waypoints.yaml 의 이름으로 이동
          "go home"              -> 접두사 go/가자/이동 은 무시한다
          "goto 1.5 0.58"        -> 좌표 직접 (yaw 는 현재 방향 유지)
          "goto 1.5 0.58 90"     -> 좌표 + 방향(도)
          "stop" / "정지"        -> 진행 중인 목표 취소
          "where" / "어디"       -> 현재 위치를 상태로 발행
          "list"                 -> 등록된 목적지 이름 발행

          "fetch center"         -> 왕복 미션: 이동 -> pick -> **출발 자리로** 복귀
          "fetch center to home" -> 복귀 지점을 웨이포인트로 지정
          "fetch center to start"-> 명시적으로 출발 자리 (기본값)
          "fetch center color:red" -> 약통 색상을 함께 실어 보낸다(선택, 순서 무관 —
                                     "to"/"color:" 토큰을 먼저 걷어내고 남는 게 목적지
                                     이름). A-Bo_project의 dialogue_node가 LLM
                                     tool-calling으로 색상을 판단해 SSH로 이 명령을
                                     보낸다(도메인 77 -> 42 경계를 SSH로 넘는다 — 두
                                     로봇은 ROS_DOMAIN_ID가 달라 ROS2 토픽을 직접
                                     공유하지 못한다).

  출력  /abo/status    std_msgs/String   사람이 읽는 한 줄
        /abo/state     std_msgs/String   기계가 읽는 값:
                                         idle|moving|arrived|picking|returning|
                                         done|failed|rejected|canceled

  pick 인계 (담당자가 다른 사람이라 여기서는 신호만 주고받는다)
        /abo/pick_request  std_msgs/String  ->  "목적지" 또는 색상이 있으면
                                                "목적지:색상"(예: "center:red").
                                                도착 직후 발행
        /abo/pick_done     std_msgs/Bool    <-  true 성공 / false 실패
                                                이걸 받아야 복귀를 시작한다

상태 문자열은 abo 가 그대로 말하게 해도 되도록 한국어로 쓴다.
"""
import math, os, sys, time, threading, subprocess, shlex
import yaml
import rclpy

from color_intent import COLORS, extract_color_token  # 같은 디렉터리에 배포됨
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import SetBool

HERE = os.path.dirname(os.path.abspath(__file__))
PREFIXES = ("go to", "goto", "go", "가자", "이동", "가줘", "가", "move to", "move")


class AboNav(Node):
    def __init__(self, wp_path, pick_timeout=180.0, host_cmd=None):
        super().__init__("abo_nav_bridge")
        self.pick_timeout = pick_timeout
        self.host_cmd = host_cmd          # ZMQ 호스트 실행 명령 (None 이면 안 띄움)
        self.host_proc = None
        # 상태 전이는 한 번에 하나만 -- 미션 상태가 꼬이지 않게 한다.
        self.cbg_state = MutuallyExclusiveCallbackGroup()
        # 액션 피드백/결과와 TF 는 서로 막지 않아야 한다.
        self.cbg_io = ReentrantCallbackGroup()
        self.wp_path = wp_path
        self.reloc_map = None
        self.wp = {}
        self._load()
        self.cli = ActionClient(self, NavigateToPose, "navigate_to_pose",
                                callback_group=self.cbg_io)
        # 베이스 노드의 시리얼 버스 양보/회수
        self.bus_cli = self.create_client(SetBool, "/lekiwi_base/set_bus",
                                          callback_group=self.cbg_io)
        # TF 리스너를 두지 않는다.  /tf 는 56 Hz 로 오는데 이 노드가 자세를
        # 쓰는 곳은 목표 yaw 기본값 정도라 그 비용(측정 ~40% CPU)이 아깝다.
        # /amcl_pose 는 AMCL 이 갱신할 때만 나오므로 훨씬 싸다.
        self._pose = None
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                                 self._on_amcl, 10)
        self.gh = None
        # 미션(왕복) 상태. None 이면 단발 이동.
        self.mission = None          # {"target","return_to","phase"}
        self.pick_deadline = 0.0
        self.pub_pick = self.create_publisher(String, "/abo/pick_request", 10)
        self.create_subscription(Bool, "/abo/pick_done", self.on_pick_done, 10,
                                 callback_group=self.cbg_state)
        self.create_timer(1.0, self.on_tick, callback_group=self.cbg_state)
        self.pub_status = self.create_publisher(String, "/abo/status", 10)
        self.pub_state = self.create_publisher(String, "/abo/state", 10)
        self.create_subscription(String, "/abo/command", self.on_cmd, 10,
                                 callback_group=self.cbg_state)
        self._last_fb = 0.0
        self.say("대기 중입니다.", "idle")
        self.get_logger().info(
            f"명령 토픽 /abo/command, 목적지 {len(self.wp)}개: {', '.join(self.wp) or '(없음)'}")

    # ---------- 유틸 ----------
    def _load(self):
        try:
            with open(self.wp_path) as f:
                d = yaml.safe_load(f) or {}
            self.wp = d.get("points") or {}
            self.wp_map = d.get("map", "?")
        except Exception as e:
            self.get_logger().warn(f"waypoints 를 읽지 못했다: {e}")
            self.wp, self.wp_map = {}, "?"

    def say(self, text, state=None):
        self.pub_status.publish(String(data=text))
        if state:
            self.pub_state.publish(String(data=state))
        self.get_logger().info(text)

    def _on_amcl(self, msg):
        p = msg.pose.pose; q = p.orientation
        self._pose = (p.position.x, p.position.y,
                      math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))

    def pose_now(self):
        return self._pose

    # ---------- 명령 처리 ----------
    def on_cmd(self, msg):
        raw = (msg.data or "").strip()
        low = raw.lower()
        for p in PREFIXES:
            if low.startswith(p + " "):
                raw = raw[len(p):].strip(); low = raw.lower(); break

        if low in ("stop", "정지", "멈춰", "cancel"):
            return self.cancel()
        if low in ("where", "어디", "위치"):
            p = self.pose_now()
            if p is None:
                return self.say("아직 제 위치를 모르겠어요.", "failed")
            return self.say(f"지금 x {p[0]:+.2f}, y {p[1]:+.2f}, 방향 {math.degrees(p[2]):+.0f}도에 있어요.")
        if low in ("list", "목록", "어디로"):
            self._load()
            return self.say("갈 수 있는 곳: " + (", ".join(self.wp) or "없어요") + f" (지도 {self.wp_map})")
        if low in ("reload",):
            self._load()
            return self.say(f"목적지를 다시 읽었어요. {len(self.wp)}개.")

        # 왕복 미션:  fetch <목적지> [to <복귀지>]
        tok0 = raw.split()
        if tok0 and tok0[0].lower() in ("fetch", "가져와", "심부름"):
            rest = tok0[1:]
            # 기본 복귀 지점은 고정 웨이포인트가 아니라 **출발 자리**다.
            # 사람이 로봇을 놓는 자리는 매번 조금씩 다른데, home(0,0) 으로
            # 돌아가면 그 차이만큼 어긋난 곳에 선다(실측 9.2 cm).
            ret = "start"
            rest, color = extract_color_token(rest)
            if color and color not in COLORS:
                return self.say(f"'{color}' 색은 지원하지 않아요. 지원 색상: "
                                + ", ".join(COLORS), "rejected")
            if len(rest) >= 3 and rest[-2].lower() in ("to", "->", "로"):
                ret = rest[-1]; rest = rest[:-2]
            name = " ".join(rest).strip()
            return self.start_mission(name, ret, color)

        tok = raw.split()
        if tok and tok[0].lower() == "goto":
            tok = tok[1:]
        # 좌표 형태인가
        try:
            x, y = float(tok[0]), float(tok[1])
            yaw = math.radians(float(tok[2])) if len(tok) > 2 else None
            return self.go(x, y, yaw, f"({x:+.2f}, {y:+.2f})")
        except (IndexError, ValueError):
            pass
        # 이름인가
        self._load()
        key = raw.strip()
        pt = self.wp.get(key) or self.wp.get(key.lower())
        if pt is None:
            return self.say(f"'{raw}' 는 모르는 곳이에요. 갈 수 있는 곳: "
                            + (", ".join(self.wp) or "없어요"), "rejected")
        return self.go(pt["x"], pt["y"], math.radians(pt.get("yaw", 0.0)), key)

    # ---------- 서보 버스 / ZMQ 호스트 ----------
    def set_bus(self, on, timeout=8.0):
        """베이스 노드가 시리얼 포트를 놓거나(False) 되잡게(True) 한다.
        ZMQ 호스트와 같은 /dev/ttyACM0 를 쓰므로 동시에 열 수 없다."""
        if not self.bus_cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn("/lekiwi_base/set_bus 가 없다. 베이스 노드 확인 필요.")
            return False
        req = SetBool.Request(); req.data = bool(on)
        fut = self.bus_cli.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            time.sleep(0.05)          # MultiThreadedExecutor 가 별도 스레드에서 돈다
        if not fut.done():
            self.get_logger().warn("set_bus 응답 없음"); return False
        r = fut.result()
        self.get_logger().info(f"set_bus({on}) -> {r.message}")
        return r.success

    def host_start(self):
        if not self.host_cmd:
            return True                # 호스트는 외부에서 관리
        if self.host_proc and self.host_proc.poll() is None:
            return True
        self.host_proc = subprocess.Popen(shlex.split(self.host_cmd),
                                          stdout=subprocess.DEVNULL,
                                          stderr=subprocess.DEVNULL)
        self.get_logger().info(f"ZMQ 호스트 기동 (pid {self.host_proc.pid})")
        time.sleep(3.0)                # 포트 잡을 시간
        return self.host_proc.poll() is None

    def host_stop(self):
        if not self.host_proc:
            return
        self.host_proc.terminate()
        try: self.host_proc.wait(timeout=8)
        except subprocess.TimeoutExpired: self.host_proc.kill()
        self.get_logger().info("ZMQ 호스트 종료")
        self.host_proc = None
        time.sleep(1.5)                # 포트가 풀릴 시간

    def _wp(self, name):
        return self.wp.get(name) or self.wp.get(str(name).lower())

    def relocalize(self, at=None, state="returning", moved=False):
        """pick 중 오도메트리가 끊겼으므로 라이다로 위치를 회복한다.
        Nav2 는 내리지 않는다 -- AMCL 은 실행 중에 /initialpose 를 받는다.

        moved=True 는 '기준점을 준 뒤 로봇이 그로부터 움직였다'는 뜻이다.
        pick 이 그렇다 -- align_base 로 게걸음/회전/전진을 하고, 가운데
        병에서 목표 병으로 옆걸음까지 한다. 그 이동은 오도메트리에 전혀
        남지 않으므로(버스를 양보한 동안 바퀴를 못 읽는다) 라이다로만
        회복할 수 있고, 따라서 도착 직후보다 넓게 훑어야 한다."""
        mp = os.path.expanduser(self.reloc_map) if self.reloc_map else ""
        if not mp or not os.path.exists(mp):
            self.get_logger().warn("재정합할 지도를 모르겠다. 건너뛴다.")
            return
        self.say("제 위치를 다시 확인할게요.", state)
        # 기준점을 goal 좌표로 준다.  Nav2 의 도착 허용오차 안에 있으므로
        # 좁은 범위만 훑으면 되고, 흐트러진 AMCL 추정에 끌려가지 않는다.
        if at:
            rng, yaw = ("0.80", "40") if moved else ("0.35", "20")
            extra = ["--at", f"{at['x']:.4f}", f"{at['y']:.4f}",
                     f"{at.get('yaw', 0.0):.2f}", "--range", rng, "--yaw", yaw]
        else:
            extra = ["--range", "0.8", "--yaw", "40"]
        # 재정합은 '있으면 좋은' 단계다. 실패해도 AMCL 은 계속 추적하므로
        # 예외를 밖으로 내보내지 않는다 -- 콜백에서 터지면 노드가 죽는다.
        try:
            r = subprocess.run(
                ["/usr/bin/python3", os.path.expanduser("~/refine_pose.py"),
                 mp, "--publish"] + extra,
                timeout=90, capture_output=True, text=True)
            tail = [l for l in (r.stdout or "").splitlines() if l.strip()][-4:]
            for l in tail:
                self.get_logger().info("  " + l)
            if r.returncode != 0:
                self.get_logger().warn(f"재정합이 {r.returncode} 로 끝났다. 그대로 진행한다.")
        except subprocess.TimeoutExpired:
            self.get_logger().warn("재정합이 90초를 넘겼다. 건너뛰고 진행한다.")
        except Exception as e:
            self.get_logger().warn(f"재정합 실패({e}). 건너뛰고 진행한다.")

    # ---------- 왕복 미션 ----------
    def start_mission(self, name, return_to, color=None):
        self._load()
        pt = self.wp.get(name) or self.wp.get(name.lower())
        if pt is None:
            return self.say(f"'{name}' 는 모르는 곳이에요. 갈 수 있는 곳: "
                            + (", ".join(self.wp) or "없어요"), "rejected")

        # 복귀 지점.  기본값 "start" 는 **지금 서 있는 자리**다.
        #
        # 고정 웨이포인트(home)로 돌아가면, 사람이 로봇을 놓은 자리와 home 이
        # 다른 만큼 어긋난다(실측: 9.2 cm). 지도 원점을 옮겨 맞추는 방법도
        # 있지만 그러면 다른 웨이포인트가 전부 같이 밀리고 점유격자와도
        # 어긋난다 -- 프레임은 두고 복귀 목표만 바꾸는 것이 맞다.
        if str(return_to).lower() in ("start", "here", "제자리", "원래"):
            p = self.pose_now()
            if p is None:
                return self.say("지금 위치를 몰라서 복귀 지점을 정할 수 없어요. "
                                "먼저 위치를 잡아 주세요.", "rejected")
            rt_pt = {"x": p[0], "y": p[1], "yaw": math.degrees(p[2])}
            rt_label = "출발 자리"
        else:
            rt_pt = self.wp.get(return_to) or self.wp.get(str(return_to).lower())
            if rt_pt is None:
                return self.say(f"복귀 지점 '{return_to}' 를 모르겠어요.", "rejected")
            rt_label = return_to

        self.mission = {"target": name, "return_to": rt_label,
                        "return_pt": rt_pt, "phase": "going", "color": color}
        self.say(f"{name} 에 가서 물건을 가져올게요.", "moving")
        self.go(pt["x"], pt["y"], math.radians(pt.get("yaw", 0.0)), name)

    def begin_pick(self):
        self.mission["phase"] = "picking"
        # 서보 버스를 ZMQ 쪽에 넘긴다. 이 시점부터 오도메트리는 멈춘다.
        self.set_bus(False)
        if not self.host_start():
            self.mission = None
            self.set_bus(True)
            return self.say("팔 제어를 시작하지 못했어요.", "failed")
        color = self.mission.get("color")
        payload = f'{self.mission["target"]}:{color}' if color else self.mission["target"]
        self.pub_pick.publish(String(data=payload))
        self.pick_deadline = time.time() + self.pick_timeout
        self.say("도착했어요. 물건을 집는 중이에요.", "picking")

    def on_pick_done(self, msg):
        # 콜백에서 예외가 새어 나가면 실행기가 노드를 내려버린다. 통째로 감싼다.
        try:
            self._on_pick_done(msg)
        except Exception as e:
            self.get_logger().error(f"pick 처리 중 오류: {e}")
            self.mission = None
            self.say("물건을 집은 뒤 처리에 실패했어요.", "failed")

    def _on_pick_done(self, msg):
        if not self.mission or self.mission["phase"] != "picking":
            return
        self.host_stop()
        self.set_bus(True)
        if not msg.data:
            self.mission = None
            return self.say("물건을 집지 못했어요.", "failed")
        # pick 이 베이스를 움직였다. 넓게 훑어야 한다 (moved=True 주석 참고).
        self.relocalize(at=self._wp(self.mission["target"]), moved=True)
        rt = self.mission["return_to"]
        pt = self.mission["return_pt"]
        self.mission["phase"] = "returning"
        self.say(f"집었어요. {rt} 으로 돌아갈게요.", "returning")
        self.go(pt["x"], pt["y"], math.radians(pt.get("yaw", 0.0)), rt)

    def on_tick(self):
        if self.mission and self.mission["phase"] == "picking" \
           and time.time() > self.pick_deadline:
            self.mission = None
            self.host_stop(); self.set_bus(True)
            self.say("물건 집기가 너무 오래 걸려서 그만둘게요.", "failed")

    def cancel(self):
        if self.gh is None:
            return self.say("가고 있지 않아요.", "idle")
        self.gh.cancel_goal_async()
        self.gh = None
        self.mission = None
        self.say("멈췄어요.", "canceled")

    def go(self, x, y, yaw, label):
        if not self.cli.wait_for_server(timeout_sec=5.0):
            return self.say("자율주행이 준비되지 않았어요. Nav2 를 확인해 주세요.", "failed")
        if yaw is None:
            p = self.pose_now()
            yaw = p[2] if p else 0.0
        g = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.orientation.z = math.sin(yaw/2)
        ps.pose.orientation.w = math.cos(yaw/2)
        g.pose = ps
        self.say(f"{label} 으로 갈게요.", "moving")
        self.cli.send_goal_async(g, feedback_callback=self.on_fb).add_done_callback(self.on_accept)

    def on_accept(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.gh = None
            return self.say("거기까지 가는 길을 찾지 못했어요.", "rejected")
        self.gh = gh
        gh.get_result_async().add_done_callback(self.on_result)

    def on_fb(self, fb):
        now = time.time()
        if now - self._last_fb < 3.0:      # 3초에 한 번만 -- 로그 폭주 방지
            return
        self._last_fb = now
        self.say(f"가는 중이에요. {fb.feedback.distance_remaining:.1f} m 남았어요.", "moving")

    def on_result(self, fut):
        st = fut.result().status
        self.gh = None
        if st == 4:      # SUCCEEDED
            if self.mission and self.mission["phase"] == "going":
                # pick 에 넘기기 전에 도착 지점 기준으로 자세를 다듬는다.
                self.relocalize(at=self._wp(self.mission["target"]), state="arrived")
                return self.begin_pick()
            if self.mission and self.mission["phase"] == "returning":
                self.relocalize(at=self.mission.get("return_pt"), state="returning")
                self.mission = None
                return self.say("돌아왔어요. 다 끝났어요.", "done")
            self.say("도착했어요.", "arrived")
        elif st == 5:    # CANCELED
            self.mission = None
            self.say("멈췄어요.", "canceled")
        else:
            self.mission = None
            self.say("도착하지 못했어요. 길이 막혀 있을 수 있어요.", "failed")


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--waypoints", default=os.path.join(HERE, "waypoints.yaml"))
    ap.add_argument("--map", default="", help="pick 후 재정합에 쓸 지도 yaml")
    ap.add_argument("--host-cmd", default="",
                    help="ZMQ 호스트 실행 명령. 비우면 외부에서 관리한다")
    ap.add_argument("--pick-timeout", type=float, default=180.0)
    a = ap.parse_args()
    wp = a.waypoints if os.path.exists(a.waypoints) else os.path.expanduser("~/waypoints.yaml")

    rclpy.init()
    n = AboNav(wp, pick_timeout=a.pick_timeout, host_cmd=a.host_cmd or None)
    n.reloc_map = a.map or None
    # ZMQ 호스트를 기다리는 동안(set_bus 응답 등) ROS 콜백이 멈추면 안 되므로
    # 멀티스레드 실행기를 쓴다. SingleThreadedExecutor 는 콜백을 하나씩만
    # 처리해서, 블로킹 구간에 /scan 구독도 액션 피드백도 전부 멎는다.
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(n)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        n.host_stop()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
