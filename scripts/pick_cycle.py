#!/usr/bin/env python3
"""약통 색상 지정 픽업 1회 — nav/mission/pick_adapter.py 가 서브프로세스로 부른다.

motion/services/pickplace(로보시지 Physical Labs에서 가져온, 실기기로 검증된
YOLO 접근+정렬/손목 서보/그립 판정 로직)를 그대로 조립해서 쓴다. motion/README.md
가 밝힌 대로 원본 `pick_worker.py`(QThread+PyQt6)의 조립 부분만 헤드리스로
다시 짠 것이다 — 게인·상태 전이·판별 규칙은 손대지 않는다.

## 색상 선택은 이 파일이 새로 얹는 것

motion/ 쪽 기본 동작은 "화면에서 가장 큰 검출"(largest)을 목표로 삼는다 —
약통이 한 색만 있다면 그걸로 충분하지만, 지금은 **요청받은 색만** 집어야
한다. 그래서 YOLO 검출 목록을 Approacher/ArmSequencer 에 넘기기 전에 색상
HSV 범위로 한 번 걸러낸다. 방식은 grasp_check.py 의 purple_mask 와 같지만
**설정은 따로 둔다** -- cfg.check 의 hue 는 "약통 양옆의 보라색 그리퍼"를 찾는
값이라, 약통 색으로 덮어쓰면 GraspChecker 가 약통 색을 그리퍼로 착각해 집기
성공/실패 판정이 망가진다.

## 자율주행(Nav2)과의 역할 분담

여기서는 바퀴를 approach(접근/정렬)만큼만 움직인다(게걸음/회전/전진, 원본
그대로). 큰 범위 복귀는 abo_nav_bridge.py(Nav2+AMCL)가 맡으므로 이 스크립트는
집고 나서 팔을 안전하게 정리(carry_home)하는 데서 끝낸다 -- base_return.py
(원본의 개루프 되돌아오기)는 쓰지 않는다(motion/README.md 참고).

## 결과 파일

pick_adapter.py 가 규정한 스키마 그대로 쓴다:
    {"skill": "pill_pickup", "color": <str>, "success": <bool>,
     "grasped": <bool>, "elapsed_sec": <float>, "verdict_reason": <str>}
시작하자마자 이전 결과를 지운다 -- 끝나기 전에 죽으면 파일이 없어야
pick_adapter.py 가 "죽었다"와 "실패"를 구분할 수 있다.

사용법:
    python scripts/pick_cycle.py --color red --result-file /tmp/r.json \\
        --model ~/medicine_yolo/best.pt --poses-dir ~/medicine_poses

사전조건(이 스크립트가 대신할 수 없음): YOLO 모델(.pt), pre_pick/grasp/
grasp_closed 자세 파일(JSON, motion/services/pickplace/poses.py 스키마),
로봇 캘리브레이션.

lekiwi_host 는 abo_nav_bridge.py 가 바퀴 노드에게서 서보 버스를 넘겨받은 뒤
nav/shell/start_pick_host.sh 로 띄운다(lekiwi_nav.launch.py 의 host_cmd). 그
호스트는 disable_torque_on_disconnect=false 로 떠서, 집기가 끝나 호스트가
내려가도 팔 토크가 유지된다 -- 약통을 쥔 채 복귀한다. 이 스크립트 자신은
LeKiwiClient(네트워크 클라이언트)라 끝날 때 소켓만 닫고 토크는 건드리지 않는다.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "motion"))  # services.pickplace 를 그대로 import

# 빨강/파랑/초록 HSV 대략 범위 (OpenCV H 0~179). 실물/조명에 맞춰 조정 필요.
# 빨강은 색상환에서 0 과 179 양쪽에 걸쳐 있어 범위가 두 개다 -- 한쪽만 쓰면
# 조명에 따라 빨간 약통의 절반을 놓친다.
DEFAULT_HUE_RANGES = {
    "red": ((0, 10), (170, 179)),
    "blue": ((100, 130),),
    "green": ((40, 80),),
}
COLORS = ("red", "green", "blue")

#: 색으로 못 걸러낸 채(=목표를 아예 못 찾음) 이 시간(초) 넘게 흐르면 실패로 끝낸다.
DEFAULT_MAX_SECONDS = 90.0


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--color", required=True, choices=COLORS, help="집을 약통 색상")
    ap.add_argument("--result-file", required=True, help="결과 JSON을 쓸 경로")
    ap.add_argument("--model", default=os.environ.get("PICK_MODEL"),
                    required=not os.environ.get("PICK_MODEL"),
                    help="YOLO 가중치(.pt) 경로 (환경변수 PICK_MODEL)")
    ap.add_argument("--poses-dir", default=os.environ.get("PICK_POSES_DIR"),
                    required=not os.environ.get("PICK_POSES_DIR"),
                    help="pre_pick.json/grasp.json/grasp_closed.json 이 있는 폴더 (환경변수 PICK_POSES_DIR)")
    ap.add_argument("--class-index", type=int, default=None,
                    help="여러 색을 각각 다른 YOLO 클래스로 학습시켰다면 그 클래스 id. "
                         "생략하면 클래스 무관, 색상 필터만 적용")
    ap.add_argument("--remote-ip", default=os.environ.get("LEKIWI_HOST_IP", "192.168.0.201"),
                    help="LeKiwi host IP (환경변수 LEKIWI_HOST_IP). 무선은 DHCP 라 주소가 바뀐다")
    ap.add_argument("--robot-id", default="lekiwi01", help="LeKiwi 보정 id")
    ap.add_argument("--zmq-cmd-port", type=int, default=5555)
    ap.add_argument("--zmq-obs-port", type=int, default=5556)
    ap.add_argument("--approach-view", default="front")
    ap.add_argument("--wrist-view", default="wrist")
    ap.add_argument("--conf", type=float, default=0.4, help="YOLO confidence 임계값")
    ap.add_argument("--min-color-ratio", type=float, default=0.25,
                    help="검출 박스 안에서 이 비율 이상이 목표 색이어야 후보로 인정")
    ap.add_argument("--hue-min", type=int, default=None,
                    help="색상 HSV hue 하한. --hue-max 와 함께 주면 기본 범위 대신 이 한 범위만 쓴다")
    ap.add_argument("--hue-max", type=int, default=None, help="색상 HSV hue 상한 (--hue-min 참고)")
    ap.add_argument("--max-seconds", type=float, default=DEFAULT_MAX_SECONDS,
                    help="이 시간 안에 못 끝내면 실패로 종료")
    ap.add_argument("--dry-run", action="store_true", help="계산만 하고 실제로 움직이지 않음")
    a = ap.parse_args(argv)
    if (a.hue_min is None) != (a.hue_max is None):
        ap.error("--hue-min 과 --hue-max 는 함께 줘야 한다")
    if a.hue_min is not None:
        a.hue_ranges = ((a.hue_min, a.hue_max),)
    else:
        a.hue_ranges = DEFAULT_HUE_RANGES[a.color]
    return a


def write_result(path: str, **fields) -> None:
    Path(path).write_text(json.dumps(fields, ensure_ascii=False, indent=2), encoding="utf-8")


def build_config(args):
    from services.pickplace.config import PickPlaceConfig
    from services.pickplace.grasp_check import GraspCheckArgs
    from services.pickplace.yolo_detect import YoloArgs

    cfg = PickPlaceConfig()
    classes = [args.class_index] if args.class_index is not None else None
    cfg.yolo = YoloArgs(path=args.model, conf=args.conf, classes=classes)
    cfg.approach.view = args.approach_view
    cfg.grasp.view = args.wrist_view
    cfg.check.front_view = args.approach_view
    cfg.check.wrist_view = args.wrist_view
    # cfg.check.hue_min/hue_max 는 건드리지 않는다 -- 보라색 그리퍼를 찾는 값이다.
    # 약통 색은 ColorSpec 으로 따로 넘긴다 (모듈 docstring 참고).
    cfg.views = [args.approach_view, args.wrist_view]
    cfg.dry_run = args.dry_run
    cfg.validate()
    return cfg


def load_poses(poses_dir: str) -> dict:
    from services.pickplace.poses import load_pose

    out = {}
    for name in ("pre_pick", "grasp", "grasp_closed"):
        path = os.path.join(os.path.expanduser(poses_dir), f"{name}.json")
        if os.path.isfile(path):
            out[name] = load_pose(Path(path))
    return out


@dataclass(frozen=True)
class ColorSpec:
    """집을 약통의 색. 그립 판정용 GraspCheckArgs(보라색 그리퍼)와 **별개**다.

    hue_ranges 는 (하한, 상한) 튜플들 -- 빨강처럼 0/179 양쪽에 걸친 색은 둘.
    sat_min/val_min 은 main() 에서 cfg.check 의 값을 그대로 가져와 실기기로
    검증된 채도·명도 기준을 따른다(기본값은 시험용).
    """
    hue_ranges: tuple
    sat_min: int = 80
    val_min: int = 60


def color_mask(frame_bgr, spec: ColorSpec):
    """spec 의 모든 hue 범위를 합친 마스크 (0/255)."""
    import cv2
    import numpy as np

    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in spec.hue_ranges:
        lo, hi = max(0, int(lo)), min(179, int(hi))
        mask |= cv2.inRange(hsv, (lo, spec.sat_min, spec.val_min), (hi, 255, 255))
    return mask


def color_filter(dets, frame_bgr, spec: ColorSpec, min_ratio: float):
    """검출 목록에서 박스 안 픽셀 중 목표 색 비율이 min_ratio 이상인 것만 남긴다."""
    kept = []
    h, w = frame_bgr.shape[:2]
    for d in dets:
        # YOLO 좌표는 실수일 수 있다 -- 그대로 슬라이싱하면 TypeError 로 죽는다.
        x1, y1, x2, y2 = (int(round(float(v))) for v in d.xyxy)
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)
        if x2 <= x1 or y2 <= y1:
            continue
        roi = frame_bgr[y1:y2, x1:x2]
        if roi.size == 0:
            continue
        ratio = float(color_mask(roi, spec).mean() / 255.0)
        if ratio >= min_ratio:
            kept.append(d)
    return kept


def main(argv=None) -> int:
    args = parse_args(argv)
    Path(args.result_file).unlink(missing_ok=True)  # pick_adapter.py 와의 규약: 시작하자마자 지운다
    t0 = time.time()

    def finish(success: bool, grasped: bool, reason: str) -> int:
        write_result(
            args.result_file, skill="pill_pickup", color=args.color,
            success=success, grasped=grasped,
            elapsed_sec=round(time.time() - t0, 1), verdict_reason=reason,
        )
        print(f"[pick_cycle:{args.color}] {'성공' if success else '실패'}: {reason}")
        return 0 if success else 1

    import cv2
    import numpy as np

    from services.pickplace import PickPlaceError
    from services.pickplace.approach import STOP, Approacher
    from services.pickplace.arm_sequencer import ArmSequencer
    from services.pickplace.grasp_check import GraspChecker
    from services.pickplace.wrist_servo import GRIPPER_JOINT, filter_wrist_dets
    from services.pickplace.yolo_detect import infer, load_model

    try:
        cfg = build_config(args)
    except PickPlaceError as exc:
        return finish(False, False, str(exc))
    spec = ColorSpec(args.hue_ranges, cfg.check.sat_min, cfg.check.val_min)

    try:
        model = load_model(cfg.yolo)
    except PickPlaceError as exc:
        return finish(False, False, str(exc))

    from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig

    robot = LeKiwiClient(LeKiwiClientConfig(
        remote_ip=args.remote_ip, id=args.robot_id,
        port_zmq_cmd=args.zmq_cmd_port, port_zmq_observations=args.zmq_obs_port,
    ))
    try:
        robot.connect()
    except Exception as exc:
        return finish(False, False, f"로봇 연결 실패: {exc}")

    try:
        views = list(cfg.views)
        deadline = t0 + 8.0
        obs: dict = {}
        while True:
            obs = robot.get_observation() or {}
            got = [v for v in views if isinstance(obs.get(v), np.ndarray)]
            if len(got) == len(views):
                break
            if time.time() > deadline:
                have = sorted(k for k, v in obs.items() if isinstance(v, np.ndarray))
                return finish(False, False, f"카메라 {views} 프레임을 못 받음 (받은 것: {have or '없음'})")
            time.sleep(0.1)

        home = {k: float(v) for k, v in obs.items() if isinstance(k, str) and k.endswith(".pos")}
        if not home:
            return finish(False, False, "팔 관절 위치를 읽지 못함")

        poses = load_poses(args.poses_dir)
        pick_pose = {k: float(v) for k, v in (poses.get("pre_pick") or {}).items() if k in home} or None
        grasp_pose = {k: float(v) for k, v in (poses.get("grasp") or {}).items() if k in home} or None
        if cfg.grasp.close_after_ready and cfg.grasp.gripper_close_pct is None:
            closed = poses.get("grasp_closed") or {}
            if GRIPPER_JOINT not in closed:
                return finish(False, False, "grasp_closed 자세에 그리퍼 값이 없음 (자세 파일 확인)")
            cfg.grasp.gripper_close_pct = float(
                np.clip(float(closed[GRIPPER_JOINT]) - cfg.grasp.gripper_close_extra_pct, 0.0, 100.0))

        ap = Approacher(cfg.approach)
        arm = ArmSequencer(home, pick_pose, cfg.pick, cfg.grasp, grasp_pose)
        checker = GraspChecker(cfg.check)
        hold_pose = dict(home)
        interval = 1.0 / max(1, cfg.fps)
        hz = 0.0
        allow_motion = not args.dry_run

        while True:
            loop_start = time.time()
            if loop_start - t0 > args.max_seconds:
                return finish(False, arm.state in ("GRASPED", "CARRY_HOME", "DONE"),
                              f"제한시간 {args.max_seconds:.0f}초 초과")

            obs = robot.get_observation() or {}
            frames_bgr = {v: cv2.cvtColor(obs[v], cv2.COLOR_RGB2BGR)
                          for v in views if isinstance(obs.get(v), np.ndarray)}
            if cfg.approach.view not in frames_bgr:
                robot.send_action({**hold_pose, **STOP})
                time.sleep(0.05)
                continue

            dets_by_view = infer(model, cfg.yolo, frames_bgr)
            # 목표 색만 남긴다 -- motion/ 기본 동작(가장 큰 검출)은 색을 모르므로
            # 여기서 미리 걸러야 엉뚱한 색 약통을 쫓아가지 않는다.
            dets_by_view[cfg.approach.view] = color_filter(
                dets_by_view.get(cfg.approach.view, []), frames_bgr[cfg.approach.view], spec, args.min_color_ratio)
            if cfg.grasp.view in frames_bgr:
                wrist_dets = filter_wrist_dets(
                    dets_by_view.get(cfg.grasp.view, []), frames_bgr[cfg.grasp.view].shape, cfg.grasp)
                dets_by_view[cfg.grasp.view] = color_filter(
                    wrist_dets, frames_bgr[cfg.grasp.view], spec, args.min_color_ratio)
            wrist_frame = frames_bgr.get(cfg.grasp.view)

            cmd = ap.update(dets_by_view[cfg.approach.view], frames_bgr[cfg.approach.view].shape, loop_start)
            tracking_ok = ap.target is not None and ap.size_ok and ap.center_ok
            arm_pose = arm.update(
                ap.done, tracking_ok, allow_motion, loop_start,
                wrist_dets=dets_by_view.get(cfg.grasp.view) if wrist_frame is not None else None,
                wrist_shape=wrist_frame.shape if wrist_frame is not None else None,
                dt=(1.0 / hz) if hz > 0 else interval,
            )

            if cfg.check.enabled and arm.grasp_enabled and arm.state == "GRASPED":
                if checker.state == "IDLE":
                    checker.start(loop_start)
                checker.update(frames_bgr, dets_by_view, loop_start)
            if checker.just_decided:
                if checker.state == "FAIL" and allow_motion and arm.can_retry:
                    arm.retry(loop_start)
                    checker.reset()
                elif checker.state == "FAIL" and allow_motion and arm.can_restart_pick:
                    arm.restart_pick(loop_start)
                    checker.reset()
                elif checker.state == "FAIL":
                    arm.give_up()
            # 그립 판정이 꺼져 있으면(check.enabled=False) checker.state 가 영영
            # IDLE 이라 SUCCESS 가 안 나온다 -- 그 경우 CLOSE_GRIPPER 가 끝난
            # 것만으로 성공으로 본다(색상 검증 없이 그냥 잡았다고 신뢰).
            grasp_confirmed = (
                checker.state == "SUCCESS" if cfg.check.enabled else arm.state == "GRASPED")
            if grasp_confirmed and arm.state == "GRASPED" and cfg.grasp.return_home_when_grasped and allow_motion:
                arm.carry_home(loop_start)
            if arm.state not in ("GRASPED", "CARRY_HOME", "DONE") and checker.state != "IDLE":
                checker.reset()
            hold_pose = dict(arm_pose)

            sent = STOP if (args.dry_run or arm.base_locked) else cmd
            robot.send_action({**arm_pose, **sent})

            if arm.just_done:
                return finish(True, True, "집기 성공")
            if arm.gave_up:
                return finish(False, False, "집기 포기 -- 재시도/재접근 횟수 소진")

            dt = time.time() - loop_start
            if interval - dt > 0:
                time.sleep(interval - dt)
            hz = 1.0 / max(time.time() - loop_start, 1e-6)
    finally:
        try:
            if robot.is_connected:
                try:
                    hold = {k: float(v) for k, v in (robot.get_observation() or {}).items()
                            if isinstance(k, str) and k.endswith(".pos")}
                    if hold:
                        robot.send_action({**hold, **STOP})
                        time.sleep(0.2)
                except Exception:
                    pass
                robot.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
