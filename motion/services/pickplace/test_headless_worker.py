import numpy as np
import pytest

from services.pickplace.arm_sequencer import ArmSequencer, PickArgs
from services.pickplace.config import PickPlaceConfig
from services.pickplace.headless_worker import PickPlaceHeadlessWorker
from services.pickplace.wrist_servo import GRIPPER_JOINT, GraspArgs, WristServo


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


def test_restart_pauses_and_resets_without_disconnecting():
    """restart() 는 [정지]와 달리 연결을 끊지 않고 스레드가 계속 돌아야 한다 —
    2026-09-13 실사용 중: [정지] 뒤 [시작]을 눌러도 스레드가 이미 끝나 있어 아무
    반응이 없던 문제. restart()는 그 대신 쓰는, 연결 유지한 채 다시 준비하는 액션."""
    robot = FakeRobot()
    w = _worker(robot)
    w.resume()

    def on_obs():
        if robot._obs_calls == 3:
            w.restart()
        if robot._obs_calls >= 6:
            w.stop_event.set()  # 테스트를 끝내려고 나중에 정상 정지

    robot.on_observation = on_obs
    w.run()

    assert robot._obs_calls >= 6  # restart 이후에도 스레드가 계속 관측을 읽었다(안 끊김)
    assert w.status.get()["state"] != "ERROR"
    assert robot.connected is False  # 마지막엔 stop_event 로 정상 종료돼 연결 해제됨


def test_go_home_rolls_out_and_resets_without_disconnecting():
    """[처음 자세로] 는 [다시 시도]와 달리 실제로 RollOutPlayer 로 홈까지 이동시킨 뒤
    (연결은 유지한 채) 새 세션을 준비한다."""
    robot = FakeRobot()
    w = _worker(robot)
    w.resume()

    rollout_calls = []
    w._run_rollout = lambda current, home: rollout_calls.append((dict(current), dict(home)))

    def on_obs():
        if robot._obs_calls == 3:
            w.go_home()
        if robot._obs_calls >= 6:
            w.stop_event.set()

    robot.on_observation = on_obs
    w.run()

    # go_home 한 번 + 정상 종료 시 _shutdown 의 롤아웃, 합쳐서 최소 2번 호출돼야 한다
    assert len(rollout_calls) >= 2
    assert w.paused_event.is_set()
    assert robot._obs_calls >= 6  # 연결 안 끊기고 계속 돎
    assert robot.connected is False  # 마지막엔 stop_event 로 정상 종료돼 연결 해제됨


def _worker_and_stuck_arm(*, max_pick_attempts=3, servo_give_up_s=1.0, pick_attempts=1):
    """SERVO 상태에서 손목캠을 오래 놓친(LOST) 상황을 그대로 재현한다 (전체 파이프라인을
    카메라/검출로 몰지 않고, 상태만 직접 강제해서 _recover_lost_servo() 만 초점 테스트).

    _recover_lost_servo() 는 워커의 self.cfg.grasp 에서 게이트 값을 읽으므로, 워커와
    ArmSequencer 가 같은 GraspArgs 인스턴스를 보게 만든다."""
    grasp_cfg = GraspArgs(
        approach_mode="joints", reach_joints={"arm_shoulder_lift.pos": 5.0},
        max_pick_attempts=max_pick_attempts, servo_give_up_s=servo_give_up_s,
    )
    robot = FakeRobot()
    cfg = PickPlaceConfig(pick=PickArgs(enabled=False), grasp=grasp_cfg)
    w = _worker(robot, cfg=cfg)

    home = {"arm_shoulder_pan.pos": 0.0, GRIPPER_JOINT: 0.0}
    arm = ArmSequencer(home, home, PickArgs(), grasp_cfg, home)
    arm.state = "SERVO"
    arm.pick_attempts = pick_attempts
    arm.servo = WristServo(grasp_cfg, home, home)
    arm.servo.state = "LOST"
    arm.servo.last_seen = 0.0
    return w, arm


def test_recover_lost_servo_restarts_pick_when_attempts_remain():
    w, arm = _worker_and_stuck_arm(max_pick_attempts=3, servo_give_up_s=1.0, pick_attempts=1)

    w._recover_lost_servo(arm, allow_motion=True, now=10.0)  # 10s 놓침 >> servo_give_up_s(1.0)

    assert arm.state == "OPEN_GRIPPER"  # restart_pick() 이 호출됐다 (그리퍼 벌리기 → HOME_WAIT → HOME)
    assert not arm.gave_up


def test_recover_lost_servo_gives_up_when_attempts_exhausted():
    w, arm = _worker_and_stuck_arm(max_pick_attempts=3, servo_give_up_s=1.0, pick_attempts=3)

    w._recover_lost_servo(arm, allow_motion=True, now=10.0)

    assert arm.gave_up


def test_recover_lost_servo_does_nothing_before_give_up_timeout():
    w, arm = _worker_and_stuck_arm(servo_give_up_s=3.0)

    w._recover_lost_servo(arm, allow_motion=True, now=1.0)  # 1s 놓침 < servo_give_up_s(3.0)

    assert arm.state == "SERVO"
    assert not arm.gave_up


def test_recover_lost_servo_does_nothing_when_paused():
    w, arm = _worker_and_stuck_arm(servo_give_up_s=1.0)

    w._recover_lost_servo(arm, allow_motion=False, now=10.0)  # 일시정지/dry-run 중

    assert arm.state == "SERVO"
    assert not arm.gave_up
