#!/usr/bin/env python3
"""약통 집기 1회 -- motion 의 PickPlaceHeadlessWorker 를 그대로 써서 한 번 집고 끝낸다.

nav/mission/pick_adapter.py 가 서브프로세스로 부른다 (--pick-script 로 지정):

    python3 nav/mission/pick_adapter.py --pick-script scripts/pick_worker_cycle.py \
        --model ~/YOLO/outputs/runs/green_pill/weights/best.pt \
        --poses-dir ~/.PhysicalLabs/pickplace/lekiwi01/poses

인자와 결과 파일 규약은 scripts/pick_cycle.py 와 같다 -- 어댑터는 어느 쪽을
부르든 똑같이 동작한다.

## 왜 워커를 감싸기만 하나

motion/services/pickplace/headless_worker.py (웹 시연 UI 의 제어 루프)는 실물
약통으로 튜닝·버그 수정이 가장 많이 된 집기 코드다. 여기서는 그 코드를 한 줄도
고치지 않고, 웹 버튼이 하던 일만 대신한다:

  - [시작] 버튼   -> 만들자마자 resume()
  - 사람의 눈     -> status 의 arm_done / gave_up / ERROR 를 보고 끝을 판정
  - [정지] 버튼   -> 끝나면 request_stop() (팔을 시작 자세로 롤아웃, 그리퍼는 그대로)

웹 서버처럼 계속 떠 있지 않고 집을 때마다 새로 실행한다. 워커는 시작할 때 한
번만 로봇 호스트에 붙는데, 호스트는 abo_nav_bridge.py 가 도착한 뒤에야 켜기
때문이다.

## 색 -> 목표 고르기

YOLO 클래스 이름에 색 단어(red/빨강 ...)가 들어 있으면 그 클래스만 목표로 삼는다
(워커의 set_target_class). 모델이 색을 구분하지 않으면(예: 클래스가 bottle 하나)
검출은 그대로 두고 박스 안 HSV 색 비율로 거른다 (pick_cycle.py 의 color_filter).
--class-name 으로 직접 지정할 수도 있다.

## 팔 토크

이 스크립트는 LeKiwiClient(네트워크 클라이언트)라 끝날 때 소켓만 닫는다. 약통을
쥔 채 복귀할 수 있는 건 로봇 쪽 호스트가 disable_torque_on_disconnect=false 로
떠 있기 때문이다 (nav/shell/start_pick_host.sh).
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "motion"))  # services.pickplace
sys.path.insert(0, str(HERE))                    # pick_cycle (색 필터 재사용)

import pick_cycle  # noqa: E402

COLORS = pick_cycle.COLORS
COLOR_WORDS = {
    "red": ("red", "빨강", "빨간"),
    "green": ("green", "초록", "녹색"),
    "blue": ("blue", "파랑", "파란"),
}
#: 어댑터 제한시간(기본 150초)보다 짧아야 한다 -- 그래야 이쪽이 먼저 멈추고
#: 팔을 정리한 뒤 결과를 쓴다. 어댑터가 먼저 죽이면 정리 없이 끝난다.
DEFAULT_MAX_SECONDS = 120.0
#: 집는 중 잡았을 수 있는 상태 (시간초과 때 grasped 추정용)
GRASPED_STATES = ("GRASPED", "GRASP_CHECK", "GRASP_OK", "CARRY_HOME", "DONE")
#: 진행 중에 바뀔 때마다 한 줄씩 출력할 status 키 -- 어댑터가 실행마다 파일로 남긴다.
#: 예전엔 끝날 때 결과 한 줄만 나와서, 5분 제한에 걸린 실패(2026-09-14)의 원인을 볼 수 없었다.
REPORT_KEYS = ("state", "pick_attempts", "retries", "gave_up", "arm_done", "error")


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--color", required=True, choices=COLORS, help="집을 약통 색상")
    ap.add_argument("--result-file", required=True, help="결과 JSON 을 쓸 경로")
    ap.add_argument("--model", default=os.environ.get("PICK_MODEL"),
                    required=not os.environ.get("PICK_MODEL"),
                    help="YOLO 가중치(.pt) (환경변수 PICK_MODEL)")
    ap.add_argument("--poses-dir", default=os.environ.get("PICK_POSES_DIR"),
                    required=not os.environ.get("PICK_POSES_DIR"),
                    help="pre_pick/grasp/grasp_closed(.json), 선택 home.json 이 있는 폴더 "
                         "(환경변수 PICK_POSES_DIR)")
    ap.add_argument("--class-name", default=None,
                    help="목표 YOLO 클래스 이름을 직접 지정. 생략하면 색 단어로 자동 선택")
    ap.add_argument("--remote-ip", default=os.environ.get("LEKIWI_HOST_IP", "192.168.0.201"),
                    help="LeKiwi 호스트 주소 (환경변수 LEKIWI_HOST_IP)")
    ap.add_argument("--robot-id", default="lekiwi01")
    ap.add_argument("--zmq-cmd-port", type=int, default=5555)
    ap.add_argument("--zmq-obs-port", type=int, default=5556)
    ap.add_argument("--conf", type=float, default=0.4, help="YOLO confidence 임계값")
    ap.add_argument("--min-color-ratio", type=float, default=0.25,
                    help="HSV 필터를 쓸 때, 박스 안 목표 색 비율 하한")
    ap.add_argument("--max-seconds", type=float, default=DEFAULT_MAX_SECONDS,
                    help="이 시간 안에 못 끝내면 멈추고 실패로 기록")
    ap.add_argument("--rollout-time", type=float, default=2.0,
                    help="멈출 때 팔을 시작 자세로 되돌리는 시간(초)")
    ap.add_argument("--max-pick-attempts", type=int, default=None,
                    help="접근부터 다시 집는 최대 횟수(첫 시도 포함). 다 쓰면 포기. 기본은 워커 설정(5)")
    ap.add_argument("--x-target-dx", type=int, default=None,
                    help="손목 뷰에서 약통 기준 변(x_anchor, 기본 왼쪽 변)을 맞출 위치 = 화면 중심 x + 이 값(px). "
                         "생략하면 워커 기본값(0). 음수 = 왼쪽")
    ap.add_argument("--y-target-dy", type=int, default=None,
                    help="손목 뷰에서 약통 세로 기준(y_anchor, 기본 박스 중심)을 맞출 위치 = 화면 중심 y + 이 값(px). "
                         "생략하면 워커 기본값(+25). 음수 = 위 -- 박스가 위로 가면 집게가 약통의 더 아랫부분을 문다")
    ap.add_argument("--pan-freeze-on-retry", action="store_true",
                    help="집기 재시도에서는 좌우(shoulder_pan)를 더 움직이지 않고 고정한다. 첫 접근의 정렬은 그대로 "
                         "두고, 재시도마다 왼쪽으로 밀리던 것만 막는다 (2026-09-16 실기기)")
    ap.add_argument("--grasp-check-sides", choices=("both", "left", "right"), default=None,
                    help="집기 성공 판정에 볼 그리퍼 쪽. left = 고정된 왼쪽 손가락만 본다 "
                         "(오른쪽 손가락이 약통에 가려 잘 쥐어도 실패로 볼 때). 생략하면 워커 기본값(both)")
    ap.add_argument("--frames-dir", default="",
                    help="비어 있지 않으면 상태가 바뀔 때마다 워커가 그린 front/wrist 화면(pan/dx 숫자 포함)을 "
                         "<이 폴더>/<시각>_<색>/ 에 jpg 로 남긴다")
    ap.add_argument("--dry-run", action="store_true", help="계산만 하고 실제로 움직이지 않음")
    ap.add_argument("--view-port", type=int, default=0,
                    help="0 이 아니면 집는 동안 워커가 그린 front/wrist 화면(십자선·검출·정렬 숫자)을 "
                         "http://<노트북>:<포트> 로 보여준다 (보기 전용). 로봇 호스트는 클라이언트를 "
                         "하나만 받으므로 웹 시연 UI 를 따로 켤 수 없을 때 쓴다")
    return ap.parse_args(argv)


# ── 순수 로직 (로봇·YOLO 없이 시험) ──────────────────────────────────────────

def resolve_target_class(names, color: str, explicit: str | None = None) -> str | None:
    """목표 YOLO 클래스 이름. None 이면 클래스로 거르지 않는다(HSV 필터를 쓴다).

    names 는 ultralytics 의 model.names (dict id->이름 또는 이름 목록).
    색 단어가 든 클래스가 정확히 하나일 때만 고른다 -- 없거나 여럿이면 애매하다.
    """
    values = list(names.values()) if isinstance(names, dict) else list(names or [])
    if explicit:
        if values and explicit not in values:
            raise ValueError(f"클래스 '{explicit}' 가 모델에 없다. 모델 클래스: {values}")
        return explicit
    words = COLOR_WORDS[color]
    matches = [n for n in values if any(w in str(n).lower() for w in words)]
    return matches[0] if len(matches) == 1 else None


def make_infer(base_infer, spec, min_ratio: float):
    """워커의 infer_fn. spec 이 있으면 박스 안 HSV 색 비율로 검출을 거른다."""
    if spec is None:
        return base_infer

    def infer(model, cfg, frames_bgr):
        dets = base_infer(model, cfg, frames_bgr)
        return {
            v: (pick_cycle.color_filter(d, frames_bgr[v], spec, min_ratio) if v in frames_bgr else d)
            for v, d in dets.items()
        }

    return infer


def outcome(status: dict):
    """워커 status 로 끝났는지 판정. (success, grasped, 이유) 또는 아직이면 None."""
    if status.get("state") == "ERROR":
        return False, False, f"오류: {status.get('error') or '알 수 없음'}"
    if status.get("arm_done"):
        return True, True, "집기 성공"
    if status.get("gave_up"):
        return False, False, "집기 포기 -- 재시도 횟수 소진"
    return None


def apply_grasp_overrides(cfg, args) -> list[str]:
    """명령줄로 받은 집기 설정을 워커 설정에 넣는다. 바꾼 항목을 사람이 읽을 문자열로 돌려준다.

    x_target_dx: 그리퍼는 왼쪽 집게가 고정이고 오른쪽 집게만 움직인다. 기본값 0(약통 왼쪽 변 =
    화면 중심선)이면 닫기 직전 약통이 고정 집게 안쪽(손목 뷰에서 중심보다 약 60px 왼쪽)에서
    떨어져 있어, 오른쪽 집게가 약통을 왼쪽으로 민다. 재시도 때 서보가 밀린 약통을 따라
    shoulder_pan 을 왼쪽으로 돌려 점점 왼쪽으로 갔다 (2026-09-13 프레임, 2026-09-14 실기기).
    """
    changed = []
    if getattr(args, "x_target_dx", None) is not None:
        cfg.grasp.x_target_dx = int(args.x_target_dx)
        changed.append(f"x_target_dx={cfg.grasp.x_target_dx:+d}")
    # y_target_dy: 기본 +25(박스 중심을 화면 중심보다 25px 아래)면 집게가 약통 윗부분(뚜껑 쪽)을
    # 물었다 (2026-09-14 실기기). 음수로 두면 박스가 위로 올라가 중간보다 아랫부분을 문다.
    if getattr(args, "y_target_dy", None) is not None:
        cfg.grasp.y_target_dy = int(args.y_target_dy)
        changed.append(f"y_target_dy={cfg.grasp.y_target_dy:+d}")
    if getattr(args, "pan_freeze_on_retry", False):
        cfg.grasp.pan_freeze_on_retry = True
        changed.append("pan_freeze_on_retry=True")
    if getattr(args, "grasp_check_sides", None) is not None:
        cfg.check.sides = args.grasp_check_sides
        changed.append(f"check.sides={cfg.check.sides}")
    return changed


def save_frames(worker, out_dir, elapsed: float, state: str | None,
                views=("front", "wrist")) -> list[Path]:
    """워커가 지금 그려 둔 화면을 jpg 로 남긴다 (없는 view 는 건너뛴다)."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    saved = []
    for v in views:
        jpg = worker.frames.get(v)
        if not jpg:
            continue
        p = out / f"{elapsed:06.1f}s_{state or 'NONE'}_{v}.jpg"
        p.write_bytes(jpg)
        saved.append(p)
    return saved


def run_cycle(worker, *, max_seconds: float, target_class: str | None = None,
              poll_s: float = 0.2, join_timeout_s: float = 15.0,
              stop_flag: threading.Event | None = None,
              clock=time.monotonic, sleep=time.sleep, report=None, on_change=None) -> dict:
    """워커를 한 번 돌려 결과 dict 를 돌려준다. 어떤 경로로 끝나든 request_stop 한다.

    report: 한 줄 문자열을 받는 함수. status(REPORT_KEYS)가 바뀔 때마다 부른다.
    on_change: (경과초, status 요약 dict) 를 받는 함수. report 와 같은 때 부른다 (화면 저장용).
    """
    if target_class:
        worker.set_target_class(target_class)
    worker.start_background()
    worker.resume()                      # 워커는 일시정지로 시작한다 -- 웹의 [시작] 버튼 역할
    t0 = clock()
    last = {}
    prev_snap = None
    try:
        while True:
            last = worker.status.get() or {}
            snap = {k: last[k] for k in REPORT_KEYS if k in last}
            if snap != prev_snap:
                if report is not None:
                    report(f"{clock() - t0:6.1f}s 상태 {snap}")
                if on_change is not None:
                    try:                         # 화면 저장이 실패해도 집기는 계속한다
                        on_change(clock() - t0, snap)
                    except Exception as exc:
                        if report is not None:
                            report(f"화면 저장 실패: {exc}")
                prev_snap = snap
            r = outcome(last)
            if r is not None:
                success, grasped, why = r
                break
            if stop_flag is not None and stop_flag.is_set():
                success, grasped, why = False, last.get("state") in GRASPED_STATES, "외부 종료 신호"
                break
            thread = getattr(worker, "_thread", None)
            if thread is not None and not thread.is_alive():
                success, grasped, why = False, False, "제어 루프가 결과 없이 끝났다"
                break
            if clock() - t0 >= max_seconds:
                success = False
                grasped = last.get("state") in GRASPED_STATES
                why = f"제한시간 {max_seconds:.0f}초 초과 (마지막 상태 {last.get('state')})"
                break
            sleep(poll_s)
    finally:
        # 정상 정지: 팔을 시작 자세로 롤아웃(그리퍼는 그대로) 후 연결 해제.
        worker.request_stop()
        worker.join(timeout=join_timeout_s)
    return {"success": success, "grasped": grasped, "verdict_reason": why,
            "elapsed_sec": round(clock() - t0, 1), "last_state": last.get("state")}


def write_result(path: str, **fields) -> None:
    Path(path).write_text(json.dumps(fields, ensure_ascii=False, indent=2), encoding="utf-8")


REQUIRED_POSES = ("pre_pick", "grasp", "grasp_closed")


# ── 보기 전용 화면 서버 ───────────────────────────────────────────────────────

VIEW_PAGE = """<!doctype html><html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>르키위 집기 화면</title>
<style>body{margin:0;padding:12px;background:#111;color:#eee;font-family:sans-serif}
.row{display:flex;flex-wrap:wrap;gap:8px}.cam{flex:1 1 480px;max-width:100%}
.cam img{width:100%;background:#222;min-height:240px}#st{margin:8px 0;font-size:15px}</style></head>
<body><div id="st">연결 중…</div><div class="row">
<div class="cam"><div>front</div><img id="front" alt="front"></div>
<div class="cam"><div>wrist</div><img id="wrist" alt="wrist"></div></div>
<script>
// 서버는 집는 동안만 떠 있다. 꺼져 있다가 다시 켜진 순간에만 영상을 다시 붙인다.
const STREAMS=['front','wrist'];let up=false;
function load(v){document.getElementById(v).src='/stream/'+v+'?t='+Date.now();}
async function poll(){let ok=false;try{const r=await fetch('/status');const s=await r.json();ok=true;
 document.getElementById('st').textContent='상태: '+JSON.stringify(s);}
 catch(e){document.getElementById('st').textContent='집기 중이 아님 (다음 집기를 기다리는 중)';}
 if(ok&&!up){STREAMS.forEach(load);}up=ok;setTimeout(poll,1000);}
poll();
</script></body></html>"""


def start_view_server(worker, port: int, host: str = "0.0.0.0", fps: float = 15.0):
    """워커의 frames(워커가 그린 jpg)·status 를 보여주는 보기 전용 HTTP 서버를 띄운다.

    쓰기(시작/정지) 경로가 없다 -- 로봇 조작은 run_cycle 만 한다. 돌려준 서버는 shutdown() 으로 끈다.
    """
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):          # 요청마다 찍히면 실행 로그가 묻힌다
            pass

        def _send(self, code, ctype, body):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            # 항상 떠 있는 어댑터 페이지(다른 포트)가 /status 를 읽을 수 있게 한다.
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path == "/":
                return self._send(200, "text/html; charset=utf-8", VIEW_PAGE.encode("utf-8"))
            if path == "/status":
                body = json.dumps(worker.status.get() or {}, ensure_ascii=False, default=str)
                return self._send(200, "application/json; charset=utf-8", body.encode("utf-8"))
            if path.startswith("/stream/"):
                view = path[len("/stream/"):]
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                try:
                    while not stopping.is_set():
                        jpg = worker.frames.get(view)
                        if jpg:
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                             + f"Content-Length: {len(jpg)}\r\n\r\n".encode() + jpg + b"\r\n")
                        time.sleep(1.0 / fps)
                except (BrokenPipeError, ConnectionResetError):
                    pass
                return
            return self._send(404, "text/plain; charset=utf-8", "없음".encode("utf-8"))

    stopping = threading.Event()
    srv = ThreadingHTTPServer((host, port), Handler)
    srv.daemon_threads = True
    orig_shutdown = srv.shutdown

    def shutdown():
        stopping.set()
        orig_shutdown()
        srv.server_close()

    srv.shutdown = shutdown
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


def load_poses(poses_dir: str) -> dict:
    """pick_cycle 과 같은 세 자세 + 있으면 home.json (워커의 [처음 자세로] 기준)."""
    poses = pick_cycle.load_poses(poses_dir)
    home = Path(os.path.expanduser(poses_dir)) / "home.json"
    if home.is_file():
        from services.pickplace.poses import load_pose
        poses["home"] = load_pose(home)
    return poses


# ── 실행 ────────────────────────────────────────────────────────────────────

def main(argv=None) -> int:
    args = parse_args(argv)
    Path(args.result_file).unlink(missing_ok=True)   # 어댑터 규약: 없음 = 끝나기 전에 죽음
    t_start = time.monotonic()
    info = {"skill": "pill_pickup", "color": args.color, "target_class": None, "filter": None}

    def finish(result: dict) -> int:
        write_result(args.result_file, **info, **result)
        print(f"[pick_worker_cycle] {'성공' if result['success'] else '실패'}: {result['verdict_reason']}")
        return 0

    poses = load_poses(args.poses_dir)
    missing = [n for n in REQUIRED_POSES if n not in poses]
    if missing:
        return finish({"success": False, "grasped": False, "elapsed_sec": 0.0,
                       "verdict_reason": f"자세 파일 없음: {missing} ({args.poses_dir})"})

    stop_flag = threading.Event()
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *_: stop_flag.set())

    try:
        from lerobot.robots.lekiwi import LeKiwiClient
        from services.pickplace.config import PickPlaceConfig
        from services.pickplace.headless_worker import PickPlaceHeadlessWorker
        from services.pickplace.robot_args import LeKiwiRobotArgs
        from services.pickplace.yolo_detect import YoloArgs, infer, load_model

        cfg = PickPlaceConfig()
        cfg.yolo = YoloArgs(path=os.path.expanduser(args.model), conf=args.conf)
        cfg.dry_run = args.dry_run
        if args.max_pick_attempts is not None:
            cfg.grasp.max_pick_attempts = args.max_pick_attempts
        changed = apply_grasp_overrides(cfg, args)
        cfg.validate()
        if changed:
            print(f"[pick_worker_cycle] 집기 설정 변경: {', '.join(changed)}", flush=True)

        model = load_model(cfg.yolo)
        target_class = resolve_target_class(model.names, args.color, args.class_name)
        spec = None
        if target_class is None:
            spec = pick_cycle.ColorSpec(pick_cycle.DEFAULT_HUE_RANGES[args.color],
                                        cfg.check.sat_min, cfg.check.val_min)
        info["target_class"] = target_class
        info["filter"] = "class" if target_class else "hsv"
        print(f"[pick_worker_cycle] 색 {args.color} -> "
              + (f"클래스 '{target_class}'" if target_class else f"HSV 필터 (모델 클래스 {model.names})"))

        robot = LeKiwiClient(LeKiwiRobotArgs(
            remote_ip=args.remote_ip, id=args.robot_id,
            port_zmq_cmd=args.zmq_cmd_port, port_zmq_observations=args.zmq_obs_port,
        ).to_config())
        worker = PickPlaceHeadlessWorker(
            robot, cfg, poses, rollout_time_s=args.rollout_time,
            infer_fn=make_infer(infer, spec, args.min_color_ratio),
            load_model_fn=lambda _cfg: model,   # 이미 올린 모델을 재사용
        )
        view = None
        if args.view_port:
            try:
                view = start_view_server(worker, args.view_port)
                print(f"[pick_worker_cycle] 집기 화면: http://localhost:{args.view_port}", flush=True)
            except OSError as e:               # 포트가 이미 쓰이면 화면만 포기하고 집기는 계속
                print(f"[pick_worker_cycle] 화면 서버를 못 띄움 ({e}) -- 집기는 계속", flush=True)
        on_change = None
        if args.frames_dir:
            frames_out = Path(os.path.expanduser(args.frames_dir)) / f"{time.strftime('%Y%m%d_%H%M%S')}_{args.color}"
            print(f"[pick_worker_cycle] 상태별 화면 저장: {frames_out}", flush=True)
            on_change = lambda elapsed, snap: save_frames(worker, frames_out, elapsed, snap.get("state"))  # noqa: E731
        try:
            result = run_cycle(worker, max_seconds=args.max_seconds, target_class=target_class,
                               join_timeout_s=args.rollout_time + 15.0, stop_flag=stop_flag,
                               report=lambda line: print(f"[pick_worker_cycle] {line}", flush=True),
                               on_change=on_change)
        finally:
            if view is not None:
                view.shutdown()
    except Exception as exc:  # 시작 실패도 결과 파일로 남긴다 -- 어댑터가 이유를 보고한다
        result = {"success": False, "grasped": False,
                  "elapsed_sec": round(time.monotonic() - t_start, 1),
                  "verdict_reason": f"시작 실패: {type(exc).__name__}: {exc}"}
    return finish(result)


if __name__ == "__main__":
    sys.exit(main())
