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
import socket
import sys
import threading
import time
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
    ap.add_argument("--dry-run", action="store_true", help="계산만 하고 실제로 움직이지 않음")
    ap.add_argument("--web-port", type=int, default=None,
                    help="지정하면 집기 동안 카메라 스트림·상태 웹(motion/webui)을 이 포트로 띄운다")
    ap.add_argument("--web-host", default="127.0.0.1",
                    help="웹 바인드 주소. 다른 기기에서 볼 때만 0.0.0.0 (8000 에는 시작·재시작 버튼도 열린다)")
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


def run_cycle(worker, *, max_seconds: float, target_class: str | None = None,
              poll_s: float = 0.2, join_timeout_s: float = 15.0,
              stop_flag: threading.Event | None = None,
              clock=time.monotonic, sleep=time.sleep) -> dict:
    """워커를 한 번 돌려 결과 dict 를 돌려준다. 어떤 경로로 끝나든 request_stop 한다."""
    if target_class:
        worker.set_target_class(target_class)
    worker.start_background()
    worker.resume()                      # 워커는 일시정지로 시작한다 -- 웹의 [시작] 버튼 역할
    t0 = clock()
    last = {}
    try:
        while True:
            last = worker.status.get() or {}
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


def start_web(worker, host: str, port: int, app_factory=None, serve=None) -> bool:
    """카메라 스트림 웹을 데몬 스레드로 띄운다. 실패해도 집기는 계속한다 -- 표시 실패가 미션 실패가 되면 안 된다."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        # uvicorn 과 같은 SO_REUSEADDR -- 없으면 직전 워커 연결이 남긴 TIME_WAIT 에 막혀 웹 없이 돈다.
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            probe.bind((host, port))
        except OSError as exc:
            print(f"[pick_worker_cycle] 경고: 웹 포트 {host}:{port} 사용 불가 ({exc}) -- 카메라 스트림 없이 계속")
            return False
    try:
        if app_factory is None:
            from webui.app import create_app as app_factory
        if serve is None:
            def serve(app, host, port):
                import uvicorn
                uvicorn.Server(uvicorn.Config(app, host=host, port=port, log_level="warning")).run()
        app = app_factory(worker)
        threading.Thread(target=serve, args=(app, host, port), daemon=True, name="pick-web").start()
        print(f"[pick_worker_cycle] 카메라 스트림 http://{host}:{port}/stream/front")
        return True
    except Exception as exc:
        print(f"[pick_worker_cycle] 경고: 웹 시작 실패 ({exc}) -- 카메라 스트림 없이 계속")
        return False


def write_result(path: str, **fields) -> None:
    Path(path).write_text(json.dumps(fields, ensure_ascii=False, indent=2), encoding="utf-8")


REQUIRED_POSES = ("pre_pick", "grasp", "grasp_closed")


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
        cfg.validate()

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
        if args.web_port:
            start_web(worker, args.web_host, args.web_port)
        result = run_cycle(worker, max_seconds=args.max_seconds, target_class=target_class,
                           join_timeout_s=args.rollout_time + 15.0, stop_flag=stop_flag)
    except Exception as exc:  # 시작 실패도 결과 파일로 남긴다 -- 어댑터가 이유를 보고한다
        result = {"success": False, "grasped": False,
                  "elapsed_sec": round(time.monotonic() - t_start, 1),
                  "verdict_reason": f"시작 실패: {type(exc).__name__}: {exc}"}
    return finish(result)


if __name__ == "__main__":
    sys.exit(main())
