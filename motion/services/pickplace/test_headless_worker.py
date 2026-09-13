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


def _worker(robot, cfg=None, poses=None, rollout_time_s=0.1) -> PickPlaceHeadlessWorker:
    cfg = cfg or PickPlaceConfig(pick=PickArgs(enabled=False))
    w = PickPlaceHeadlessWorker(
        robot, cfg, poses or {}, infer_fn=_fake_infer, load_model_fn=_fake_load_model,
        first_obs_timeout_s=0.5, rollout_time_s=rollout_time_s,
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


def test_paused_worker_only_sends_stop_and_then_stops_cleanly():
    robot = FakeRobot()
    w = _worker(robot)  # 기본 paused_event.set() 상태, pick.enabled=False

    def stop_after_three():
        if robot._obs_calls >= 3:
            w.stop_event.set()

    robot.on_observation = stop_after_three
    w.run()

    # 첫 프레임 대기(_wait_for_first_frames)가 관측 1회를 이미 소모하므로,
    # obs_calls==3 에 도달할 때까지 메인 루프는 최소 2번 돈다.
    assert len(robot.sent_actions) >= 2
    for action in robot.sent_actions:
        assert action["x.vel"] == 0.0
        assert action["y.vel"] == 0.0
        assert action["theta.vel"] == 0.0
    assert w.status.get()["state"] == "PAUSED"
    assert robot.connected is False  # finally 에서 disconnect 됨


def test_clean_stop_rolls_out_toward_home_before_disconnect():
    """home 은 첫 프레임 대기 때(0.0) 캡처된다. 이후 관측을 가로채 '실제 로봇은
    10.0 에 있다'고 강제로 어긋나게 만들면(pick 이 꺼져 있어 팔이 스스로 움직이지
    않으므로, send_action 이 매번 0.0 을 되돌려보내도 이 오버라이드로 관측값만은
    계속 10.0 으로 보인다) 종료 시 current(10.0) != home(0.0) 이 되어, 롤아웃이
    실제로 0.0 쪽으로 되돌리는 것을 확인할 수 있다."""
    robot = FakeRobot()
    w = _worker(robot)
    w.resume()
    real_get_observation = robot.get_observation

    def get_observation_diverged():
        obs = real_get_observation()
        if robot._obs_calls >= 3:
            obs["arm_shoulder_pan.pos"] = 10.0
            w.stop_event.set()  # abort 아님 → 롤아웃 있어야 함
        return obs

    robot.get_observation = get_observation_diverged
    w.run()

    pan_values = [a["arm_shoulder_pan.pos"] for a in robot.sent_actions if "arm_shoulder_pan.pos" in a]
    assert pan_values[-1] < 10.0  # 롤아웃이 시작 자세(0.0) 쪽으로 움직였다


def test_abort_skips_rollout():
    robot = FakeRobot()
    w = _worker(robot)
    w.resume()
    real_get_observation = robot.get_observation

    def get_observation_diverged():
        obs = real_get_observation()
        if robot._obs_calls >= 3:
            obs["arm_shoulder_pan.pos"] = 10.0
            w.request_abort()
        return obs

    robot.get_observation = get_observation_diverged
    w.run()

    # 마지막 액션의 팔 자세가 그대로 10.0 이어야 한다 (롤아웃 없이 즉시 정지)
    pan_values = [a["arm_shoulder_pan.pos"] for a in robot.sent_actions if "arm_shoulder_pan.pos" in a]
    assert pan_values[-1] == pytest.approx(10.0)
