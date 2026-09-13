import numpy as np
import pytest

from services.pickplace.arm_sequencer import PickArgs
from services.pickplace.config import PickPlaceConfig
from services.pickplace.headless_worker import PickPlaceHeadlessWorker
from services.pickplace.wrist_servo import GRIPPER_JOINT, GraspArgs


class FakeRobot:
    """PickPlaceHeadlessWorker 가 요구하는 최소 인터페이스만 흉내낸다 (덕타이핑)."""

    def __init__(self, frame_shape=(64, 64, 3)):
        self.connected = False
        self.sent_actions: list[dict[str, float]] = []
        self.on_observation = None  # 테스트가 나중에 채운다
        self._obs_calls = 0
        self._frame = np.zeros(frame_shape, dtype=np.uint8)
        self._pose = {
            "arm_shoulder_pan.pos": 0.0,
            "arm_shoulder_lift.pos": 0.0,
            "arm_elbow_flex.pos": 0.0,
            "arm_wrist_flex.pos": 0.0,
            "arm_wrist_roll.pos": 0.0,
            GRIPPER_JOINT: 0.0,
        }

    def connect(self) -> None:
        self.connected = True

    def get_observation(self) -> dict:
        self._obs_calls += 1
        obs = dict(self._pose)
        obs["front"] = self._frame
        obs["wrist"] = self._frame
        if self.on_observation is not None:
            self.on_observation()
        return obs

    def send_action(self, action: dict) -> None:
        self.sent_actions.append(dict(action))
        for k, v in action.items():
            if k in self._pose:
                self._pose[k] = v

    def disconnect(self) -> None:
        self.connected = False

    @property
    def is_connected(self) -> bool:
        return self.connected


def _fake_infer(model, cfg, frames_bgr):
    return {v: [] for v in frames_bgr}


def _fake_load_model(cfg):
    return object()


def _worker(robot, cfg=None, poses=None) -> PickPlaceHeadlessWorker:
    cfg = cfg or PickPlaceConfig(pick=PickArgs(enabled=False))
    w = PickPlaceHeadlessWorker(
        robot, cfg, poses or {}, infer_fn=_fake_infer, load_model_fn=_fake_load_model,
        first_obs_timeout_s=0.5,
    )
    return w


def test_resolve_gripper_close_pct_from_poses():
    robot = FakeRobot()
    cfg = PickPlaceConfig(pick=PickArgs(enabled=True), grasp=GraspArgs(enabled=True, close_after_ready=True))
    w = _worker(robot, cfg, poses={"grasp_closed": {GRIPPER_JOINT: 15.1}})
    problem = w._resolve_gripper_close_pct()
    assert problem == ""
    assert cfg.grasp.gripper_close_pct == pytest.approx(15.1)


def test_resolve_gripper_close_pct_missing_pose_returns_error():
    robot = FakeRobot()
    cfg = PickPlaceConfig(pick=PickArgs(enabled=True), grasp=GraspArgs(enabled=True, close_after_ready=True))
    w = _worker(robot, cfg, poses={})
    problem = w._resolve_gripper_close_pct()
    assert problem != ""
    assert cfg.grasp.gripper_close_pct is None


def test_wait_for_first_frames_times_out_without_cameras():
    robot = FakeRobot()
    robot._frame = None  # front/wrist 를 아예 안 준다
    cfg = PickPlaceConfig(pick=PickArgs(enabled=False))
    w = PickPlaceHeadlessWorker(
        robot, cfg, {}, infer_fn=_fake_infer, load_model_fn=_fake_load_model, first_obs_timeout_s=0.2,
    )

    def get_observation_no_cameras():
        return {}

    robot.get_observation = get_observation_no_cameras
    assert w._wait_for_first_frames() is None
