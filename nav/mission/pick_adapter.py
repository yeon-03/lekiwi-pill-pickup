#!/usr/bin/env python3
"""자율주행(로봇)과 집기(노트북)를 잇는 어댑터.

  /abo/pick_request (String)  받음  ->  집기 스크립트 실행 (--pick-script)
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

집기 스크립트 (--pick-script, 저장소 기준 경로)
    scripts/pick_cycle.py         기본값. motion 로직을 직접 조립
    scripts/pick_worker_cycle.py  motion 의 PickPlaceHeadlessWorker(웹 시연 UI 의
                                  제어 루프)를 그대로 써서 한 번 집는다
  둘 다 인자(--color/--result-file/--model/--poses-dir/--remote-ip)와 결과 파일
  규약이 같아서, 어댑터는 어느 쪽이든 똑같이 부른다.

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
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String

SKILL = "pill_pickup"
RESULT = f"/tmp/lekiwi_result_{SKILL}.json"

# 집기 스크립트는 ROS 를 모른다(lerobot·opencv 환경). 이 어댑터는 ros2 환경에서 떠서
# PYTHONPATH/LD_LIBRARY_PATH 에 /opt/ros/... 와 colcon 작업공간 경로가 들어 있는데, 그대로
# 물려주면 다른 인터프리터(--python, 예: conda 의 lerobot 환경)에 ROS 의 파이썬 패키지와
# 공유 라이브러리가 섞인다. 2026-09-13 실기기에서 같은 이유로(로봇 쪽 lekiwi_host 가 ROS
# 환경을 물려받음) 카메라를 여는 순간 USB 가 끊기는 일이 있었고, 노트북에서는 PYTHONPATH 를
# 빼는 보조 스크립트를 따로 두고 --python 에 넘겨야 했다. 여기서 ROS 경로 항목만 빼서
# 넘긴다 -- 사용자가 직접 넣은 경로(CUDA 라이브러리 등)는 그대로 둔다.
ROS_PATH_VARS = ("PYTHONPATH", "LD_LIBRARY_PATH")


def child_env(env=None):
    """집기 스크립트에 넘길 환경변수. ROS 설치·작업공간 경로를 PYTHONPATH/LD_LIBRARY_PATH 에서 뺀다."""
    env = dict(os.environ if env is None else env)
    prefixes = ["/opt/ros/"]
    for var in ("AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH"):
        prefixes += [p.rstrip("/") + "/" for p in env.get(var, "").split(os.pathsep) if p]
    for var in ROS_PATH_VARS:
        if var not in env:
            continue
        keep = [p for p in env[var].split(os.pathsep)
                if p and not any(p.startswith(x) or p + "/" == x for x in prefixes)]
        if keep:
            env[var] = os.pathsep.join(keep)
        else:
            del env[var]
    return env


class PickAdapter(Node):
    def __init__(self, repo, mapping, timeout, python, extra, script="scripts/pick_cycle.py",
                 run_log_dir=None):
        super().__init__("pick_adapter")
        # 실행마다 집기 스크립트 출력(stdout+stderr)을 여기에 파일로 실시간 기록한다. None 이면 남기지 않는다.
        self.run_log_dir = Path(run_log_dir).expanduser() if run_log_dir else None
        self.repo = Path(repo).expanduser()
        self.script = script
        self.mapping = mapping
        self.timeout = timeout
        self.python = python
        self.extra = extra
        self.busy = False
        self.last_result = None     # 화면 페이지용: 지난 집기 {ok, why, at}
        self.done_q = queue.Queue()

        cbg = MutuallyExclusiveCallbackGroup()
        self.create_subscription(String, "/abo/pick_request",
                                 self.on_request, 10, callback_group=cbg)
        self.pub = self.create_publisher(Bool, "/abo/pick_done", 10)
        # 집기는 수십 초 걸린다. 콜백에서 기다리면 실행기가 멈추므로 별도
        # 스레드에서 돌리고, 결과만 이 타이머가 받아 발행한다.
        self.create_timer(0.5, self.on_tick, callback_group=cbg)

        script = self.repo / self.script
        if not script.is_file():
            self.get_logger().error(f"집기 스크립트를 찾지 못했다: {script}")
            self.get_logger().error("--repo 로 저장소 경로를 지정할 것.")
        self.get_logger().info(
            f"준비됨. repo={self.repo}  스크립트={self.script}  매핑={self.mapping}  "
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
            self.get_logger().warning(f"집는 중이라 '{raw}' 요청 무시")
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
        self.get_logger().info(f"집기 시작: {raw} -> --color {color}")
        threading.Thread(target=self.run_pick, args=(color,), daemon=True).start()

    # --- 집기 실행 (별도 스레드) --------------------------------------------
    def run_pick(self, color):
        ok, why = False, ""
        try:
            # 지난 실행 결과가 남아 있으면 이번 것으로 오독된다.
            Path(RESULT).unlink(missing_ok=True)

            cmd = [self.python, self.script,
                   "--color", color, "--result-file", RESULT] + self.extra
            # 출력을 모았다가 끝에 버리지 않고 파일로 바로 쓴다 -- 제한시간에 걸리거나 어댑터가
            # 죽어도 집기가 어디까지 갔는지 남는다 (2026-09-14 5분 실패의 원인을 못 봤다).
            if self.run_log_dir is not None:
                self.run_log_dir.mkdir(parents=True, exist_ok=True)
                log_path = self.run_log_dir / f"{time.strftime('%Y%m%d_%H%M%S')}_{color}.log"
            else:
                fd, name = tempfile.mkstemp(prefix="pick_run_", suffix=".log")
                os.close(fd)
                log_path = Path(name)
            self.get_logger().info("실행: " + " ".join(cmd) + f"  (출력 {log_path})")
            env = child_env()                      # ROS 경로를 뺀 환경 (child_env 주석 참고)
            env.setdefault("PYTHONUNBUFFERED", "1")  # 출력이 버퍼에 묶이지 않고 파일에 바로 쓰이게
            t0 = time.time()
            with open(log_path, "w", encoding="utf-8") as out:
                proc = subprocess.Popen(cmd, cwd=self.repo, stdout=out,
                                        stderr=subprocess.STDOUT, env=env)
                try:
                    rc = proc.wait(timeout=self.timeout)
                except subprocess.TimeoutExpired:
                    proc.kill()                    # run() 과 달리 Popen 은 스스로 안 죽인다
                    proc.wait()
                    raise
            dt = time.time() - t0

            if rc != 0:
                tail = log_path.read_text(encoding="utf-8", errors="replace").strip().splitlines()[-3:]
                self.get_logger().error(
                    f"{self.script} 종료코드 {rc} ({dt:.0f}초)")
                for line in tail:
                    self.get_logger().error("  " + line)
            if self.run_log_dir is None:
                log_path.unlink(missing_ok=True)

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
        self.last_result = {"ok": ok, "why": why, "at": time.strftime("%H:%M:%S")}
        self.pub.publish(Bool(data=ok))
        # rclpy 는 로그를 부르는 코드 위치마다 처음 쓴 심각도를 기억하고, 같은 위치에서 다른
        # 심각도가 오면 ValueError 를 낸다. 예전엔 info/warn 을 한 줄에서 골라 불러서, 한
        # 어댑터가 성공(info) 뒤 실패(warn)를 처리하는 순간 예외가 타이머 밖으로 새어 어댑터가
        # 통째로 죽었다 (2026-09-14 실기기: 파랑 성공 뒤 초록 실패). 호출 위치를 나누고, 로그
        # 때문에 노드가 죽지 않게 감싼다 -- pick_done 은 이미 보냈다.
        try:
            if ok:
                self.get_logger().info(f"pick_done={ok}  {why}")
            else:
                self.get_logger().warning(f"pick_done={ok}  {why}")
        except Exception as e:
            print(f"[pick_adapter] 로그 실패({e}): pick_done={ok}  {why}", flush=True)


# ── 집기 카메라 화면 페이지 (항상 떠 있음) ────────────────────────────────────
# 영상·워커 상태는 집는 동안에만 워커(pick_worker_cycle --view-port)가 보낸다. 워커 서버만 두면
# 집기 전후에는 페이지 자체가 안 열려서 새로고침할 수 없었다 (2026-09-14). 그래서 페이지는 항상
# 떠 있는 어댑터가 주고, 영상은 페이지가 워커 포트에서 직접 받는다(이미지는 다른 포트여도 된다).
VIEW_PAGE = """<!doctype html><html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>르키위 집기 화면</title>
<style>body{margin:0;padding:12px;background:#111;color:#eee;font-family:sans-serif}
.row{display:flex;flex-wrap:wrap;gap:8px}.cam{flex:1 1 480px;max-width:100%}
.cam img{width:100%;background:#222;min-height:240px;display:block}#st{margin:8px 0;font-size:15px}</style></head>
<body><div id="st">연결 중…</div><div class="row">
<div class="cam"><div>front</div><img id="front" alt="front"></div>
<div class="cam"><div>wrist</div><img id="wrist" alt="wrist"></div></div>
<script>
const W = location.protocol + '//' + location.hostname + ':__WORKER_PORT__';
const STREAMS = ['front', 'wrist']; const st = document.getElementById('st'); let up = false;
function load(v){ document.getElementById(v).src = W + '/stream/' + v + '?t=' + Date.now(); }
function clear(v){ document.getElementById(v).removeAttribute('src'); }
function fmt(s){ return s.state + ' · 시도 ' + s.pick_attempts + '/' + s.max_pick_attempts
  + ' · 재시도 ' + s.retries + '/' + s.max_retries + (s.target_class ? ' · 목표 ' + s.target_class : ''); }
async function poll(){
  let ok = false;
  try { const r = await fetch(W + '/status', {cache: 'no-store'}); const s = await r.json();
        ok = true; st.textContent = '집는 중: ' + fmt(s); }
  catch (e) {
    try { const a = await (await fetch('/adapter_status', {cache: 'no-store'})).json();
          st.textContent = a.busy ? '집기 준비 중… (워커가 로봇에 연결하는 중)'
            : '대기 중 — 집기가 시작되면 화면이 자동으로 떠요'
              + (a.last ? ' (지난 결과: ' + (a.last.ok ? '성공' : '실패') + ' · ' + a.last.why + ' · ' + a.last.at + ')' : ''); }
    catch (e2) { st.textContent = '어댑터 응답 없음'; }
  }
  if (ok && !up) STREAMS.forEach(load);
  if (!ok && up) STREAMS.forEach(clear);
  up = ok; setTimeout(poll, 1000);
}
poll();
</script></body></html>"""


def start_view_page(node, port, worker_port, host="0.0.0.0"):
    """카메라 화면 페이지와 /adapter_status 를 주는 보기 전용 HTTP 서버. 돌려준 서버는 shutdown() 으로 끈다."""
    page = VIEW_PAGE.replace("__WORKER_PORT__", str(worker_port)).encode("utf-8")

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _send(self, code, ctype, body):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path == "/":
                return self._send(200, "text/html; charset=utf-8", page)
            if path == "/adapter_status":
                body = json.dumps({"busy": node.busy, "last": node.last_result}, ensure_ascii=False)
                return self._send(200, "application/json; charset=utf-8", body.encode("utf-8"))
            return self._send(404, "text/plain; charset=utf-8", "없음".encode("utf-8"))

    srv = ThreadingHTTPServer((host, port), Handler)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


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
                    help="집기 저장소 경로 (scripts/pick_cycle.py 가 있는 곳)")
    ap.add_argument("--pick-script", default="scripts/pick_cycle.py",
                    help="실행할 집기 스크립트 (--repo 기준). "
                         "예: scripts/pick_worker_cycle.py")
    ap.add_argument("--map", type=parse_map, default="center=green",
                    help="목적지=색상 대응. 예: center=green,left=red,right=blue")
    ap.add_argument("--run-log-dir", default="~/pickplace_logs/pick_runs",
                    help="실행마다 집기 스크립트 출력을 <시각>_<색>.log 로 실시간 기록할 폴더. "
                         "빈 값이면 남기지 않는다")
    ap.add_argument("--view-page-port", type=int, default=0,
                    help="0 이 아니면 이 포트에 집기 카메라 화면 페이지를 항상 띄운다. 영상·워커 상태는 "
                         "집는 동안 워커가 --worker-view-port 로 보낸다 (pick_worker_cycle 일 때 자동으로 넘김)")
    ap.add_argument("--worker-view-port", type=int, default=8011,
                    help="집기 워커의 화면 서버 포트 (--view-page-port 를 쓸 때)")
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
                    help="pick_cycle.py 에 그대로 넘길 인자 (예: -- --grasp-lift 68)")
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
    pick_opts = []
    if a.model:
        model = os.path.expanduser(a.model)
        if not os.path.isfile(model):
            ap.error(f"YOLO 모델 파일이 없다: {model}")
        pick_opts += ["--model", model]
    if a.poses_dir:
        poses = os.path.expanduser(a.poses_dir)
        if not os.path.isdir(poses):
            ap.error(f"자세 폴더가 없다: {poses}")
        missing = [n for n in ("pre_pick", "grasp", "grasp_closed")
                   if not os.path.isfile(os.path.join(poses, n + ".json"))]
        if missing:
            print(f"경고: 자세 파일 없음 {missing}", file=sys.stderr)
        pick_opts += ["--poses-dir", poses]
    if a.remote_ip:
        pick_opts += ["--remote-ip", a.remote_ip]
    if not (a.model and a.poses_dir):
        print("경고: --model/--poses-dir 이 없다. 실제 pick_cycle.py 는 둘 다 필수라 "
              "집기가 바로 실패한다 (시험용 가짜 pick_cycle 이면 무시)", file=sys.stderr)

    rclpy.init()
    extra = pick_opts + list(a.pick_args)
    if a.view_page_port and "pick_worker_cycle" in a.pick_script and "--view-port" not in extra:
        extra += ["--view-port", str(a.worker_view_port)]
    node = PickAdapter(a.repo, mapping, a.timeout, python, extra,
                       a.pick_script, run_log_dir=a.run_log_dir or None)
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(node)
    page = None
    if a.view_page_port:
        try:
            page = start_view_page(node, a.view_page_port, a.worker_view_port)
            node.get_logger().info(f"집기 카메라 화면: http://localhost:{a.view_page_port} (항상 열림)")
        except OSError as e:               # 포트가 막히면 화면만 포기
            node.get_logger().warning(f"화면 페이지를 못 띄움 ({e}) -- 집기는 계속")
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if page is not None:
            page.shutdown()
            page.server_close()
        node.destroy_node()
        # SIGTERM/SIGINT 로 끝나면 rclpy 가 이미 컨텍스트를 내렸다 -- 두 번 부르면 RCLError.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
