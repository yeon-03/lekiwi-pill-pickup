"""Physical Labs 앱 없이 터미널/웹에서 pick&place 를 돌리는 헤드리스 제어 루프.

원본 CLI(`lekiwi_yolo_pick.py`, roboseasy/lekiwi.git)의 control_loop/main 과 같은
조립 순서(연결 → 첫 프레임 대기 → 관측 → YOLO 추론 → Approacher/ArmSequencer/
GraspChecker 갱신 → 오버레이 → send_action)를 따르되, PyQt 대신 threading.Event
로 제어하고 cv2 창 대신 LatestFrame/SharedStatus 로 상태를 내보낸다.

정지 시 시작 자세로 롤아웃하는 안전 동작(`RollOutPlayer`)은 원본에 없던 것으로
설계 문서 §"제어 루프"에서 의도적으로 추가한 것이다.
"""
from __future__ import annotations

import logging
import threading
import time
from typing import Any, Callable, Protocol

import cv2
import numpy as np

from services.pickplace.approach import STOP, Approacher, draw_alignment
from services.pickplace.arm_sequencer import ArmSequencer, draw_wrist_servo
from services.pickplace.config import PickPlaceConfig
from services.pickplace.frame_stream import FrameAgeTracker, LatestFrame, SharedStatus
from services.pickplace.grasp_check import GraspChecker, center_purple_ratio, draw_grasp_check
from services.pickplace.roll_out import RollOutPlayer
from services.pickplace.wrist_servo import GRIPPER_JOINT, filter_wrist_dets
from services.pickplace.yolo_detect import draw, infer as yolo_infer, load_model


class Robot(Protocol):
    """PickPlaceHeadlessWorker 가 요구하는 최소 인터페이스 (덕타이핑, lerobot 무관 테스트용)."""

    def connect(self) -> None: ...
    def get_observation(self) -> dict[str, Any]: ...
    def send_action(self, action: dict[str, float]) -> None: ...
    def disconnect(self) -> None: ...

    @property
    def is_connected(self) -> bool: ...


class PickPlaceHeadlessWorker:
    def __init__(
        self,
        robot: Robot,
        cfg: PickPlaceConfig,
        poses: dict[str, dict[str, float]],
        *,
        rollout_time_s: float = 2.0,
        first_obs_timeout_s: float = 8.0,
        infer_fn: Callable = yolo_infer,
        load_model_fn: Callable = load_model,
    ) -> None:
        self.robot = robot
        self.cfg = cfg
        self._poses = {k: dict(v) for k, v in poses.items()}
        self._rollout_time_s = rollout_time_s
        self._first_obs_timeout_s = first_obs_timeout_s
        self._infer = infer_fn
        self._load_model = load_model_fn

        self.frames = LatestFrame()
        self.status = SharedStatus()

        self.paused_event = threading.Event()
        self.paused_event.set()  # 시작은 항상 일시정지 상태 — 조작자가 [시작] 을 눌러야 움직인다
        self.stop_event = threading.Event()
        self.abort_event = threading.Event()
        self.restart_event = threading.Event()
        self.home_event = threading.Event()

        # 실행 중 라이브로 바꿀 수 있는 타겟 클래스 필터 (예: "red_pill_bottle") —
        # None 이면 필터 없이 검출된 모든 클래스가 후보다. 웹 UI 버튼이나(나중에)
        # 외부 에이전트가 set_target_class() 로 세션 재시작 없이 바꾼다
        # (2026-09-13: 색 지정 없이는 초록/빨강 둘 다 유효한 타겟이라 매 프레임 더 큰
        # 쪽으로 흔들리며 왔다갔다하던 문제 — 색을 지정하면 애초에 다른 색은 후보에서
        # 빠지므로 해결된다).
        self._target_lock = threading.Lock()
        self._target_class: str | None = None

        self._thread: threading.Thread | None = None

    def set_target_class(self, name: str | None) -> None:
        with self._target_lock:
            self._target_class = name

    def get_target_class(self) -> str | None:
        with self._target_lock:
            return self._target_class

    # ── 공개 제어 ──
    def start_background(self) -> None:
        self._thread = threading.Thread(target=self.run, daemon=False)
        self._thread.start()

    def join(self, timeout: float | None = None) -> None:
        if self._thread is not None:
            self._thread.join(timeout)

    def resume(self) -> None:
        self.paused_event.clear()

    def pause(self) -> None:
        self.paused_event.set()

    def request_stop(self) -> None:
        self.stop_event.set()

    def request_abort(self) -> None:
        self.abort_event.set()
        self.stop_event.set()

    def go_home(self) -> None:
        """[다시 시도]와 달리, 지금 자세를 그대로 인정하는 게 아니라 연결을 유지한 채
        실제로 저장된 기준 자세로 천천히 되돌린다 (RollOutPlayer 재사용). "home"(단정한
        시작 자세)이 저장돼 있으면 그걸, 없으면 "pre_pick"을 목표로 삼는다(둘 다 없으면
        현재 자세를 그대로 유지). 도착 후 일시정지 상태로 pick 을 다시 준비한다 —
        수동으로 팔을 이리저리 만져본 뒤 "일단 처음 자세로" 되돌리고 싶을 때 쓴다."""
        self.home_event.set()

    def restart(self) -> None:
        """[정지] 처럼 연결을 끊지 않고, 지금 자세를 새 기준으로 잡아 pick 을 처음부터
        다시 준비한다 (일시정지 상태로 — [시작] 을 눌러야 움직인다). Physical Labs
        GUI 앱의 `restart_pick()` UI 액션과 같은 역할: [정지]/[비상정지]는 세션을
        완전히 끝내고 연결을 끊는 것이고(터미널 Ctrl+C 와 같은 급), 이건 연결을 유지한
        채 같은 세션 안에서 다시 시도하는 것이다."""
        self.restart_event.set()

    # ── 내부 헬퍼 ──
    @staticmethod
    def _latch_pose(obs: dict) -> dict[str, float]:
        return {k: float(v) for k, v in (obs or {}).items() if isinstance(k, str) and k.endswith(".pos")}

    def _filter_pose(self, name: str, home: dict[str, float]) -> dict[str, float] | None:
        pose = self._poses.get(name)
        if not pose:
            return None
        return {k: float(v) for k, v in pose.items() if k in home}

    def _resolve_gripper_close_pct(self) -> str:
        """그리퍼 닫기 목표를 poses['grasp_closed'] 에서 정한다. 문제가 있으면 에러 메시지를 돌려준다."""
        g = self.cfg.grasp
        if not (self.cfg.pick.enabled and g.enabled and g.close_after_ready):
            return ""
        if g.gripper_close_pct is not None:
            return ""
        closed = self._poses.get("grasp_closed") or {}
        if GRIPPER_JOINT not in closed:
            return "grasp_closed 자세에 그리퍼 값이 없습니다 (poses['grasp_closed'])."
        base = float(closed[GRIPPER_JOINT])
        g.gripper_close_pct = float(np.clip(base - g.gripper_close_extra_pct, 0.0, 100.0))
        return ""

    def _recover_lost_servo(self, arm: ArmSequencer, allow_motion: bool, now: float) -> None:
        """손목캠이 오래 놓친 채(LOST)면 — 놓친 게 아니라 이번 접근 실패로 보고
        그리퍼를 벌리고 물러났다가 재접근한다 (2026-09-13 발견: 이 트리거가 없으면
        다시 보일 때까지 영원히 그 자리에 멈춰있었다). 재시도 횟수를 다 쓰면 포기한다."""
        g = self.cfg.grasp
        if not (
            allow_motion
            and g.servo_give_up_s > 0
            and arm.state == "SERVO"
            and arm.servo is not None
            and arm.servo.state == "LOST"
            and now - arm.servo.last_seen >= g.servo_give_up_s
        ):
            return
        if arm.pick_attempts < g.max_pick_attempts:
            arm.restart_pick(now)
        else:
            arm.give_up()

    def _wait_for_first_frames(self) -> dict[str, Any] | None:
        deadline = time.perf_counter() + self._first_obs_timeout_s
        while not self.stop_event.is_set():
            obs = self.robot.get_observation() or {}
            got = [v for v in self.cfg.views if isinstance(obs.get(v), np.ndarray)]
            if len(got) == len(self.cfg.views):
                return obs
            if time.perf_counter() > deadline:
                return None
            time.sleep(0.1)
        return None

    # ── 메인 루프 ──
    def run(self) -> None:
        home: dict[str, float] = {}
        clean_exit = False
        interval = 1.0 / max(1, self.cfg.fps)
        try:
            model = self._load_model(self.cfg.yolo)
            self.robot.connect()

            obs = self._wait_for_first_frames()
            if obs is None:
                self.status.set({"state": "ERROR", "error": f"{self._first_obs_timeout_s:.0f}초 안에 카메라 프레임을 받지 못했습니다."})
                return

            home = self._latch_pose(obs)
            if not home:
                self.status.set({"state": "ERROR", "error": "팔 관절 위치를 읽지 못했습니다."})
                return

            problem = self._resolve_gripper_close_pct()
            if problem:
                self.status.set({"state": "ERROR", "error": problem})
                return

            pick_pose = self._filter_pose("pre_pick", home) if self.cfg.pick.enabled else None
            grasp_pose = (
                self._filter_pose("grasp", home)
                if (self.cfg.pick.enabled and self.cfg.grasp.enabled)
                else None
            )

            ap = Approacher(self.cfg.approach)
            arm = ArmSequencer(home, pick_pose, self.cfg.pick, self.cfg.grasp, grasp_pose)
            checker = GraspChecker(self.cfg.check)
            hold_pose = dict(home)
            hz = 0.0
            ages = FrameAgeTracker()

            while not self.stop_event.is_set():
                loop_start = time.perf_counter()
                obs = self.robot.get_observation() or {}
                frames_bgr = {
                    v: cv2.cvtColor(obs[v], cv2.COLOR_RGB2BGR)
                    for v in self.cfg.views
                    if isinstance(obs.get(v), np.ndarray)
                }
                for v, img_bgr in frames_bgr.items():
                    ages.update(v, img_bgr, loop_start)
                if self.cfg.approach.view not in frames_bgr:
                    self.robot.send_action({**hold_pose, **STOP})
                    self.stop_event.wait(0.05)
                    continue

                if self.restart_event.is_set():
                    self.restart_event.clear()
                    home = self._latch_pose(obs) or home
                    pick_pose = self._filter_pose("pre_pick", home) if self.cfg.pick.enabled else None
                    grasp_pose = (
                        self._filter_pose("grasp", home)
                        if (self.cfg.pick.enabled and self.cfg.grasp.enabled)
                        else None
                    )
                    ap = Approacher(self.cfg.approach)
                    arm = ArmSequencer(home, pick_pose, self.cfg.pick, self.cfg.grasp, grasp_pose)
                    checker = GraspChecker(self.cfg.check)
                    hold_pose = dict(home)
                    self.paused_event.set()
                    self.status.set({"state": "RESTART_READY", "dry_run": self.cfg.dry_run, "paused": True})
                    continue

                if self.home_event.is_set():
                    self.home_event.clear()
                    self.paused_event.set()
                    current = self._latch_pose(obs)
                    # "처음 자세" = 세션 연결 시점에 팔이 우연히 있던 자세가 아니라, 저장해 둔
                    # 기준 자세여야 한다 (2026-09-13 실사용 중 발견: 연결 시점 자세를 그대로
                    # home 으로 쓰면, 마지막으로 팔을 어디에 뒀었는지에 따라 "처음 자세로"가
                    # 아무 효과도 없어 보일 수 있었다). "home"(단정한 시작 자세)이 저장돼
                    # 있으면 그걸 쓰고, 없으면 "pre_pick"(약통 찾기 직전 자세)으로, 그것도
                    # 없으면(예: --pick.enabled=false) 세션 시작 자세로 대체한다.
                    home = self._filter_pose("home", current) or self._filter_pose("pre_pick", current) or home
                    if not self.cfg.dry_run and current and home:
                        self.status.set({"state": "GOING_HOME", "dry_run": self.cfg.dry_run, "paused": True})
                        self._run_rollout(current, home)
                    ap = Approacher(self.cfg.approach)
                    arm = ArmSequencer(home, pick_pose, self.cfg.pick, self.cfg.grasp, grasp_pose)
                    checker = GraspChecker(self.cfg.check)
                    hold_pose = dict(home)
                    self.status.set({"state": "HOME_READY", "dry_run": self.cfg.dry_run, "paused": True})
                    continue

                dets_by_view = self._infer(model, self.cfg.yolo, frames_bgr)
                target_class = self.get_target_class()
                if target_class:
                    dets_by_view = {
                        v: [d for d in dets if d.name == target_class] for v, dets in dets_by_view.items()
                    }
                if (
                    self.cfg.grasp.enabled
                    and self.cfg.grasp.view in frames_bgr
                    and self.cfg.grasp.view != self.cfg.approach.view
                ):
                    dets_by_view[self.cfg.grasp.view] = filter_wrist_dets(
                        dets_by_view.get(self.cfg.grasp.view, []),
                        frames_bgr[self.cfg.grasp.view].shape,
                        self.cfg.grasp,
                    )
                wrist_frame = frames_bgr.get(self.cfg.grasp.view)
                paused = self.paused_event.is_set()
                allow_motion = not paused and not self.cfg.dry_run

                cmd = ap.update(
                    dets_by_view[self.cfg.approach.view], frames_bgr[self.cfg.approach.view].shape, loop_start
                )
                tracking_ok = ap.target is not None and ap.size_ok and ap.center_ok

                descend_hint = False
                if self.cfg.grasp.front_hint and arm.state == "SERVO" and self.cfg.approach.view in frames_bgr:
                    ratio = center_purple_ratio(
                        frames_bgr[self.cfg.approach.view], self.cfg.check, self.cfg.grasp.front_hint_win_px
                    )
                    descend_hint = ratio >= self.cfg.grasp.front_hint_min_ratio

                arm_pose = arm.update(
                    ap.done,
                    tracking_ok,
                    allow_motion,
                    loop_start,
                    wrist_dets=dets_by_view.get(self.cfg.grasp.view) if wrist_frame is not None else None,
                    wrist_shape=wrist_frame.shape if wrist_frame is not None else None,
                    dt=interval,
                    descend_hint=descend_hint,
                )

                self._recover_lost_servo(arm, allow_motion, loop_start)

                if self.cfg.check.enabled and arm.grasp_enabled and arm.state == "GRASPED" and allow_motion:
                    if checker.state == "IDLE":
                        checker.start(loop_start)
                    checker.update(frames_bgr, dets_by_view, loop_start)
                check_label = {"CHECKING": "GRASP_CHECK", "SUCCESS": "GRASP_OK", "FAIL": "GRASP_FAIL"}.get(
                    checker.state
                )

                if paused:
                    state_label, sent = "PAUSED", dict(STOP)
                elif self.cfg.dry_run:
                    state_label, sent = "DRY_RUN", dict(STOP)
                elif arm.base_locked:
                    state_label, sent = (check_label or arm.label or ap.state), dict(STOP)
                else:
                    state_label, sent = ap.state, cmd

                if checker.just_decided:
                    if checker.state == "FAIL" and allow_motion and arm.can_retry:
                        arm.retry(loop_start)
                        checker.reset()
                    elif checker.state == "FAIL" and allow_motion and arm.can_restart_pick:
                        arm.restart_pick(loop_start)
                        checker.reset()
                    elif checker.state == "FAIL":
                        arm.give_up()
                if (
                    checker.state == "SUCCESS"
                    and arm.state == "GRASPED"
                    and self.cfg.grasp.return_home_when_grasped
                    and allow_motion
                ):
                    arm.carry_home(loop_start)
                if arm.state not in ("GRASPED", "CARRY_HOME", "DONE") and checker.state != "IDLE":
                    checker.reset()

                hold_pose = dict(arm_pose)
                self.robot.send_action({**arm_pose, **sent})

                for v in self.cfg.views:
                    if v not in frames_bgr:
                        continue
                    img = draw(frames_bgr[v], v, dets_by_view.get(v, []), hz, crosshair=v in self.cfg.crosshair_views)
                    if v == self.cfg.approach.view:
                        img = draw_alignment(img, ap, cmd, state_label)
                    if v == self.cfg.grasp.view and arm.grasp_enabled:
                        img = draw_wrist_servo(img, arm, state_label)
                    if self.cfg.check.enabled and v in (self.cfg.check.front_view, self.cfg.check.wrist_view):
                        img = draw_grasp_check(img, checker, v)
                    ok, buf = cv2.imencode(".jpg", img)
                    if ok:
                        self.frames.set(v, buf.tobytes())

                self.status.set(
                    {
                        "state": state_label,
                        "pick_attempts": arm.pick_attempts,
                        "max_pick_attempts": self.cfg.grasp.max_pick_attempts,
                        "retries": arm.retries,
                        "max_retries": self.cfg.grasp.max_retries,
                        "gave_up": arm.gave_up,
                        "arm_done": arm.done,
                        "dry_run": self.cfg.dry_run,
                        "paused": paused,
                        "hz": round(hz, 1),
                        "target_class": target_class,
                        "purple": {
                            v: {
                                "ratio": round(min(r.left_ratio, r.right_ratio), 3),
                                "thr": self.cfg.check.min_ratio_for(v),
                            }
                            for v, r in checker.results.items()
                        },
                        "retry_depth": round(arm.servo.retry_depth, 3) if arm.servo is not None else 0.0,
                        "frame_age_s": ages.ages(time.perf_counter()),
                    }
                )

                dt = time.perf_counter() - loop_start
                if interval - dt > 0:
                    self.stop_event.wait(interval - dt)
                hz = 1.0 / max(time.perf_counter() - loop_start, 1e-6)

            clean_exit = True
        except Exception as exc:
            logging.exception("PickPlaceHeadlessWorker 오류")
            self.status.set({"state": "ERROR", "error": str(exc)})
        finally:
            self._shutdown(home, clean_exit)

    def _run_rollout(self, current: dict[str, float], home: dict[str, float]) -> None:
        """`RollOutPlayer` 로 current → home 을 천천히 보간해서 실제로 보낸다 (블로킹).
        [정지]의 롤아웃과 [처음 자세로] 버튼이 공유하는 로직."""
        rollout = RollOutPlayer(current, home, self._rollout_time_s)
        while not rollout.done:
            if self.abort_event.is_set():
                # 비상정지는 롤아웃이 끝날 때까지(최대 rollout_time_s 초) 기다리면 안 된다 —
                # 2026-09-13 리뷰에서 지적: _shutdown() 의 정상 종료 롤아웃은 stop_event 가
                # 이미 set 된 채로 진행되므로 여기서는 abort_event 만 확인해야 한다.
                return
            now = time.perf_counter()
            pose = rollout.update(now)
            self.robot.send_action({**pose, **STOP})
            time.sleep(1.0 / max(1, self.cfg.fps))

    def _shutdown(self, home: dict[str, float], clean_exit: bool) -> None:
        try:
            if not self.robot.is_connected:
                return
            if clean_exit and not self.abort_event.is_set() and home:
                current = self._latch_pose(self.robot.get_observation() or {})
                self._run_rollout(current, home)
            else:
                hold = self._latch_pose(self.robot.get_observation() or {})
                self.robot.send_action({**hold, **STOP})
            time.sleep(0.2)
        except Exception:
            logging.exception("정지 시퀀스 중 오류")
        finally:
            if self.robot.is_connected:
                try:
                    self.robot.disconnect()
                except Exception:
                    logging.exception("연결 해제 실패")
