#!/usr/bin/env python3
"""자율주행(로봇)과 집기(노트북)를 잇는 어댑터.

  /abo/pick_request (String)  받음  ->  집기 스크립트 실행 (--script)
  /abo/pick_done    (Bool)    보냄  <-  결과 JSON 판정

  ** 이 노드는 노트북에서 돈다. 로봇(Pi)이 아니다. **

집기 코드가 카메라 2대와 OpenCV 를 쓰므로 노트북에서만 돌 수 있고, 그쪽은
ROS 를 전혀 모른다(requirements.txt 에 rclpy 가 없다). 그래서 ROS 를 아는
쪽은 이 파일 하나뿐이고, 집기 코드는 지금 그대로 두면 된다.

로봇과 같은 도메인이어야 토픽이 보인다:

    export ROS_DOMAIN_ID=42          # 로봇의 lekiwi_profile.sh 와 같은 값
    python3 pick_adapter.py --repo ~/lekiwi-pill-pickup

배포하지 않는다 -- deploy_lekiwi.py 의 SHARED 목록에 넣지 말 것. 로봇에
올라가면 카메라가 없어 실행되지 않는다.

목적지 이름 -> 색상
  자율주행은 기본적으로 웨이포인트 이름("center")만 보내고, 그러면 집기는
  --map 으로 미리 정해둔 고정 색상을 쓴다(현장에 색상별로 다른 자리를 뒀을 때):

    python3 pick_adapter.py --map center=green,left=red,right=blue

  하지만 사용자가 발화로 그때그때 색을 고르는 경우(한 자리에 여러 색이 같이
  있음, abo_nav_bridge.py 의 "fetch center color:red" 형식)에는 요청 자체에
  "이름:색상"으로 색이 실려 온다 -- 그 경우 --map 보다 우선한다. 요청에 색이
  없으면(콜론 없음) 예전처럼 --map 을 그대로 쓴다.

집기 스크립트 (--script)
    python3 pick_adapter.py                        # scripts/pick_cycle.py (기본)
    python3 pick_adapter.py --script yolo_pick     # yolo_and_pick/lekiwi_yolo_pick.py
  색상/결과 파일/--model/--poses-dir/--remote-ip 는 스크립트에 맞는 인자 이름으로
  번역해 넘긴다. 나머지 pick_args 는 번역 없이 그대로 넘어간다.

결과 판정
  두 스크립트 모두 시작하자마자 결과 파일을 지우고, 끝나면 다시 쓴다.
  따라서 "파일 없음"은 성공도 실패도 아닌 '끝나기 전에 죽음'이다 -- 셋 다
  복귀는 해야 하므로 pick_done=false 로 보내되, 로그는 구분해서 남긴다.
"""
import argparse
import json
import os
import queue
import subprocess
import sys
import threading
import time
from pathlib import Path

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String

SKILL = "pill_pickup"
RESULT = f"/tmp/lekiwi_result_{SKILL}.json"

# --script 로 고르는 집기 스크립트. 둘 다 "시작하자마자 결과 파일 삭제, 끝나면
# JSON 기록" 규약은 같고 인자 이름만 다르다 -- 번역은 여기서 한다.
#   pick_cycle: scripts/pick_cycle.py (argparse, 하이픈)
#   yolo_pick : yolo_and_pick/lekiwi_yolo_pick.py (roboseasy/lekiwi 원본, draccus, 점/언더스코어)
SCRIPTS = {
    "pick_cycle": "scripts/pick_cycle.py",
    "yolo_pick": "yolo_and_pick/lekiwi_yolo_pick.py",
}


def color_args(script, color, result):
    if script == "yolo_pick":
        return [f"--target_color={color}", f"--result_file={result}"]
    return ["--color", color, "--result-file", result]


def setup_args(script, model, poses, remote_ip):
    """--model/--poses-dir/--remote-ip 를 스크립트별 인자로 바꾼다. 빈 값은 뺀다."""
    out = []
    if script == "yolo_pick":
        if model:
            out.append(f"--yolo.path={model}")
        if poses:
            files = {"pick.pose_file": "pre_pick", "grasp.grasp_pose_file": "grasp",
                     "grasp.close_pose_file": "grasp_closed", "grasp.ref_file": "grasp_ref"}
            for key, name in files.items():
                path = os.path.join(poses, name + ".json")
                if os.path.isfile(path):
                    out.append(f"--{key}={path}")
        if remote_ip:
            out.append(f"--robot.remote_ip={remote_ip}")
        return out
    if model:
        out += ["--model", model]
    if poses:
        out += ["--poses-dir", poses]
    if remote_ip:
        out += ["--remote-ip", remote_ip]
    return out


class PickAdapter(Node):
    def __init__(self, repo, mapping, timeout, python, extra, script="pick_cycle"):
        super().__init__("pick_adapter")
        self.repo = Path(repo).expanduser()
        self.script = script
        self.mapping = mapping
        self.timeout = timeout
        self.python = python
        self.extra = extra
        self.busy = False
        self.done_q = queue.Queue()

        cbg = MutuallyExclusiveCallbackGroup()
        self.create_subscription(String, "/abo/pick_request",
                                 self.on_request, 10, callback_group=cbg)
        self.pub = self.create_publisher(Bool, "/abo/pick_done", 10)
        # 집기는 수십 초 걸린다. 콜백에서 기다리면 실행기가 멈추므로 별도
        # 스레드에서 돌리고, 결과만 이 타이머가 받아 발행한다.
        self.create_timer(0.5, self.on_tick, callback_group=cbg)

        script = self.repo / SCRIPTS[self.script]
        if not script.is_file():
            self.get_logger().error(f"집기 스크립트를 찾지 못했다: {script}")
            self.get_logger().error("--repo 로 저장소 경로를 지정할 것.")
        self.get_logger().info(
            f"준비됨. repo={self.repo}  스크립트={SCRIPTS[self.script]}  매핑={self.mapping}  "
            f"제한시간={self.timeout:.0f}초  도메인={os.environ.get('ROS_DOMAIN_ID','0')}")

    # --- 요청 ---------------------------------------------------------------
    def on_request(self, msg):
        # 콜백에서 예외가 새어 나가면 실행기가 노드를 내려버린다.
        try:
            self._on_request(msg)
        except Exception as e:
            self.get_logger().error(f"요청 처리 실패: {e}")
            self.done_q.put((False, f"어댑터 오류: {e}"))

    def _on_request(self, msg):
        raw = (msg.data or "").strip()
        # "center:red" 형식이면 발화로 고른 색이 실려온 것 -- --map 보다 우선.
        # 콜론이 없으면 partition 의 세 번째 값이 빈 문자열이라 기존처럼 --map 을 쓴다.
        target, _, explicit_color = raw.partition(":")
        target = target.strip()
        explicit_color = explicit_color.strip().lower()

        if self.busy:
            # 이미 집는 중인데 또 왔다. 무시한다 -- 두 번 실행하면 팔이 엉킨다.
            self.get_logger().warn(f"집는 중이라 '{raw}' 요청 무시")
            return

        color = explicit_color or self.mapping.get(target)
        if color is None:
            self.get_logger().error(
                f"목적지 '{target}' 에 대응하는 색상이 없다. --map 확인. "
                f"현재: {self.mapping}")
            self.done_q.put((False, f"'{target}' 매핑 없음"))
            return
        if color not in ("red", "green", "blue"):
            self.get_logger().error(f"알 수 없는 색상 '{color}' (요청: {raw!r})")
            self.done_q.put((False, f"알 수 없는 색상 '{color}'"))
            return

        self.busy = True
        self.get_logger().info(f"집기 시작: {raw} -> 색상 {color}")
        threading.Thread(target=self.run_pick, args=(color,), daemon=True).start()

    # --- 집기 실행 (별도 스레드) --------------------------------------------
    def run_pick(self, color):
        ok, why = False, ""
        try:
            # 지난 실행 결과가 남아 있으면 이번 것으로 오독된다.
            Path(RESULT).unlink(missing_ok=True)

            cmd = ([self.python, SCRIPTS[self.script]]
                   + color_args(self.script, color, RESULT) + self.extra)
            self.get_logger().info("실행: " + " ".join(cmd))
            t0 = time.time()
            p = subprocess.run(cmd, cwd=self.repo, timeout=self.timeout,
                               capture_output=True, text=True)
            dt = time.time() - t0

            if p.returncode != 0:
                tail = (p.stderr or "").strip().splitlines()[-3:]
                self.get_logger().error(
                    f"{SCRIPTS[self.script]} 종료코드 {p.returncode} ({dt:.0f}초)")
                for line in tail:
                    self.get_logger().error("  " + line)

            ok, why = self.verdict(dt)

        except subprocess.TimeoutExpired:
            ok, why = False, f"제한시간 {self.timeout:.0f}초 초과"
            self.get_logger().error(why)
        except Exception as e:
            ok, why = False, f"실행 실패: {e}"
            self.get_logger().error(why)
        finally:
            self.done_q.put((ok, why))

    def verdict(self, dt):
        """결과 파일을 읽어 성공/실패를 정한다."""
        try:
            raw = Path(RESULT).read_text(encoding="utf-8").strip()
        except OSError:
            # pick_cycle 은 시작하자마자 파일을 지운다. 없다는 건 끝내지
            # 못하고 죽었다는 뜻이다 (성공도 실패도 아니다).
            return False, "결과 파일 없음 -- 집기가 끝나기 전에 죽었다"
        if not raw:
            return False, "결과 파일이 비어 있다"
        try:
            d = json.loads(raw)
        except ValueError:
            return False, "결과 파일이 JSON 이 아니다"
        if not isinstance(d, dict):
            return False, "결과 파일 형식이 다르다"

        ok = bool(d.get("success"))
        why = d.get("verdict_reason") or ("성공" if ok else "실패")
        return ok, f"{why} ({d.get('elapsed_sec', dt):.0f}초)"

    # --- 결과 발행 -----------------------------------------------------------
    def on_tick(self):
        try:
            ok, why = self.done_q.get_nowait()
        except queue.Empty:
            return
        self.busy = False
        self.pub.publish(Bool(data=ok))
        lg = self.get_logger().info if ok else self.get_logger().warn
        lg(f"pick_done={ok}  {why}")


def parse_map(s):
    out = {}
    for pair in s.split(","):
        pair = pair.strip()
        if not pair:
            continue
        if "=" not in pair:
            raise argparse.ArgumentTypeError(f"'{pair}' 는 이름=색상 형식이 아니다")
        k, v = pair.split("=", 1)
        out[k.strip()] = v.strip()
    return out


def main():
    ap = argparse.ArgumentParser(
        description="자율주행(로봇)과 집기(노트북)를 잇는다. 노트북에서 실행할 것.")
    ap.add_argument("--repo", default="~/lekiwi-pill-pickup",
                    help="집기 저장소 경로 (scripts/, yolo_and_pick/ 이 있는 곳)")
    ap.add_argument("--script", choices=sorted(SCRIPTS), default="pick_cycle",
                    help="실행할 집기 스크립트. pick_cycle=scripts/pick_cycle.py, "
                         "yolo_pick=yolo_and_pick/lekiwi_yolo_pick.py")
    ap.add_argument("--map", type=parse_map, default="center=green",
                    help="목적지=색상 대응. 예: center=green,left=red,right=blue")
    ap.add_argument("--timeout", type=float, default=150.0,
                    help="집기 제한시간(초). 로봇 쪽 --pick-timeout(기본 180초)보다 "
                         "짧게 둘 것 -- 그래야 로봇이 먼저 포기하지 않고 "
                         "이쪽이 실패 이유를 보고한다")
    # rclpy 가 있는 파이썬과 집기(numpy/opencv/lerobot)가 있는 파이썬은
    # 대개 다른 환경이다. 같은 걸로 가정하면 import 에서 죽는다.
    ap.add_argument("--python", default=sys.executable,
                    help="pick_cycle.py 를 실행할 인터프리터. 기본값은 이 노드와 "
                         "같은 것이지만, 집기 의존성(numpy/opencv/scipy/lerobot)이 "
                         "다른 venv 에 있다면 그쪽 python 을 지정할 것. "
                         "예: --python ~/lekiwi-pill-pickup/.venv/bin/python")
    # pick_cycle.py 의 필수 인자는 여기서 받아 넘긴다. 예전엔 pick_args 뒤에 손으로
    # 붙여야 했고, 빠뜨리면 pick_cycle 이 인자 오류로 바로 죽어 "결과 파일 없음"
    # 이라는 엉뚱한 이유만 남았다.
    ap.add_argument("--model", default=os.environ.get("PICK_MODEL", ""),
                    help="YOLO 가중치(.pt). 환경변수 PICK_MODEL")
    ap.add_argument("--poses-dir", default=os.environ.get("PICK_POSES_DIR", ""),
                    help="pre_pick/grasp/grasp_closed.json 이 있는 폴더. 환경변수 PICK_POSES_DIR")
    ap.add_argument("--remote-ip", default=os.environ.get("LEKIWI_HOST_IP", ""),
                    help="로봇 주소. 환경변수 LEKIWI_HOST_IP (무선은 DHCP 라 바뀐다)")
    ap.add_argument("pick_args", nargs="*",
                    help="집기 스크립트에 그대로 넘길 인자 (예: -- --grasp-lift 68, "
                         "yolo_pick 이면 -- --display=none)")
    a = ap.parse_args()

    mapping = a.map if isinstance(a.map, dict) else parse_map(a.map)

    # 색상은 pick_cycle.py 가 choices 로 강제한다. 여기서 미리 걸러야
    # 실행 후 argparse 오류로 죽어 "결과 파일 없음"이라는 엉뚱한 이유가 남지 않는다.
    bad = {k: v for k, v in mapping.items() if v not in ("red", "green", "blue")}
    if bad:
        ap.error(f"색상은 red/green/blue 만 된다. 잘못된 항목: {bad}")

    python = os.path.expanduser(a.python)
    if not os.path.isfile(python):
        ap.error(f"인터프리터가 없다: {python}")

    # 줄 수 있는 건 시작할 때 검사한다 -- 집기 도중에 알게 되면 로봇이 이미
    # 목적지까지 간 뒤다.
    model = poses = ""
    if a.model:
        model = os.path.expanduser(a.model)
        if not os.path.isfile(model):
            ap.error(f"YOLO 모델 파일이 없다: {model}")
    if a.poses_dir:
        poses = os.path.expanduser(a.poses_dir)
        if not os.path.isdir(poses):
            ap.error(f"자세 폴더가 없다: {poses}")
        missing = [n for n in ("pre_pick", "grasp", "grasp_closed")
                   if not os.path.isfile(os.path.join(poses, n + ".json"))]
        if missing:
            print(f"경고: 자세 파일 없음 {missing}", file=sys.stderr)
    pick_opts = setup_args(a.script, model, poses, a.remote_ip)
    # lekiwi_yolo_pick.py 는 안 주면 yolo_and_pick/weights/best.pt, poses/*.json 을 쓴다.
    if a.script == "pick_cycle" and not (a.model and a.poses_dir):
        print("경고: --model/--poses-dir 이 없다. 실제 pick_cycle.py 는 둘 다 필수라 "
              "집기가 바로 실패한다 (시험용 가짜 pick_cycle 이면 무시)", file=sys.stderr)

    rclpy.init()
    node = PickAdapter(a.repo, mapping, a.timeout, python,
                       pick_opts + list(a.pick_args), a.script)
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
