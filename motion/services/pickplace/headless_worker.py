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
from services.pickplace.frame_stream import LatestFrame, SharedStatus
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

        self._thread: threading.Thread | None = None

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
