import numpy as np
import pytest

from services.pickplace.approach import ApproachArgs
from services.pickplace.arm_sequencer import ArmSequencer, PickArgs
from services.pickplace.config import PickPlaceConfig
from services.pickplace.headless_worker import PickPlaceHeadlessWorker
from services.pickplace.wrist_servo import GRIPPER_JOINT, GraspArgs, WristServo
from services.pickplace.yolo_detect import Detection


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


def test_go_home_targets_saved_pre_pick_not_connect_time_pose():
    """2026-09-13 실사용 중 발견: home 을 "세션 연결 시점에 팔이 있던 자세"로 그대로
    쓰면, 마지막으로 팔을 어디에 뒀었는지에 따라 [처음 자세로] 가 아무 효과도 없어
    보일 수 있었다(이미 그 자리라서). 저장된 pre_pick.json 을 목표로 삼아야 한다."""
    robot = FakeRobot()  # 연결 시점 자세: 전부 0.0
    saved_pre_pick = {
        "arm_shoulder_pan.pos": 5.98,
        "arm_shoulder_lift.pos": 58.95,
        "arm_elbow_flex.pos": -54.99,
        "arm_wrist_flex.pos": 47.25,
        "arm_wrist_roll.pos": -16.13,
        GRIPPER_JOINT: 1.36,
    }
    w = _worker(robot, poses={"pre_pick": saved_pre_pick})
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

    go_home_call = rollout_calls[0]
    _current, target_home = go_home_call
    assert target_home["arm_shoulder_pan.pos"] == 5.98
    assert target_home["arm_shoulder_lift.pos"] == 58.95
    assert robot.connected is False  # 마지막엔 stop_event 로 정상 종료돼 연결 해제됨


def test_go_home_prefers_saved_home_over_pre_pick():
    """2026-09-13: pre_pick(약통 찾기 직전 자세)과 home(단정한 시작 자세)은 서로 다른
    자세다 — 둘 다 저장돼 있으면 [처음 자세로] 는 home 을 우선해야 한다."""
    robot = FakeRobot()
    saved_pre_pick = {
        "arm_shoulder_pan.pos": 5.98,
        "arm_shoulder_lift.pos": 58.95,
        "arm_elbow_flex.pos": -54.99,
        "arm_wrist_flex.pos": 47.25,
        "arm_wrist_roll.pos": -16.13,
        GRIPPER_JOINT: 1.36,
    }
    saved_home = {
        "arm_shoulder_pan.pos": -2.73,
        "arm_shoulder_lift.pos": 1.58,
        "arm_elbow_flex.pos": -0.22,
        "arm_wrist_flex.pos": 78.51,
        "arm_wrist_roll.pos": -0.92,
        GRIPPER_JOINT: 4.65,
    }
    w = _worker(robot, poses={"pre_pick": saved_pre_pick, "home": saved_home})
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

    _current, target_home = rollout_calls[0]
    assert target_home["arm_shoulder_lift.pos"] == 1.58
    assert target_home["arm_wrist_flex.pos"] == 78.51


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


def test_get_set_target_class_round_trip():
    w = _worker(FakeRobot())
    assert w.get_target_class() is None
    w.set_target_class("red_pill_bottle")
    assert w.get_target_class() == "red_pill_bottle"
    w.set_target_class(None)
    assert w.get_target_class() is None


def test_target_class_filter_picks_only_matching_color():
    """색 지정 없이는 더 큰 박스(빨강, 오른쪽)를 타겟으로 잡지만, target_class 를
    초록으로 지정하면 빨강은 후보에서 아예 빠지고 초록(왼쪽) 쪽으로 회전한다
    (2026-09-13: 색 지정 없이는 둘 다 유효한 타겟이라 매 프레임 더 큰 쪽으로 흔들리며
    왔다갔다하던 문제 — 색을 지정하면 다른 색은 애초에 후보에서 빠진다)."""
    green = Detection(name="green_pill_bottle", conf=0.9, xyxy=(4, 40, 24, 60), cls=0)  # 폭 20, 왼쪽(cx=14)
    red = Detection(name="red_pill_bottle", conf=0.9, xyxy=(40, 40, 62, 60), cls=1)  # 폭 22, 오른쪽(cx=51) — 더 큼

    def infer(model, cfg, frames_bgr):
        return {v: ([green, red] if v == "front" else []) for v in frames_bgr}

    robot = FakeRobot()
    cfg = PickPlaceConfig(
        pick=PickArgs(enabled=False),
        # backward_when_clipped 는 이 테스트의 작은 가짜 프레임에서 박스가 바닥에
        # 닿은 것처럼 보여 "잘렸다"고 오판, 후진만 하고 회전을 안 하게 만들어서 끈다
        # — 여기서 보려는 건 타겟 선택/회전 방향이지 바닥 클리핑 로직이 아니다.
        approach=ApproachArgs(center_tolerance_px=5, backward_when_clipped=False),
    )
    w = PickPlaceHeadlessWorker(
        robot, cfg, {}, infer_fn=infer, load_model_fn=_fake_load_model, first_obs_timeout_s=0.5,
    )
    w.set_target_class("green_pill_bottle")
    w.paused_event.clear()  # 실제로 접근 명령이 나오는 걸 봐야 하므로 일시정지 해제

    def stop_after_two():
        if robot._obs_calls >= 2:
            w.stop_event.set()

    robot.on_observation = stop_after_two
    w.run()

    thetas = [a["theta.vel"] for a in robot.sent_actions if a["theta.vel"] != 0.0]
    assert thetas, "회전 명령이 하나도 안 나왔음"
    # green center_error = 14-32 = -18(화면 왼쪽) → +theta(좌회전). 필터 없이 더 큰
    # red(center_error=+19)가 골라졌다면 부호가 반대(-theta)로 나왔을 것이다.
    assert all(t > 0 for t in thetas)
    assert w.status.get()["target_class"] == "green_pill_bottle"


def test_status_reports_frame_age_purple_and_retry_depth():
    import time as _time

    robot = FakeRobot()
    w = _worker(robot)
    w.start_background()                      # 워커는 일시정지로 시작 → 루프는 돌며 status 를 쓴다
    deadline = _time.time() + 3.0
    status = {}
    while _time.time() < deadline:
        status = w.status.get()
        if "frame_age_s" in status and status["frame_age_s"].get("front", 0) > 0.2:
            break
        _time.sleep(0.05)
    w.request_stop()
    w.join(timeout=5.0)

    assert set(status["frame_age_s"]) == {"front", "wrist"}
    assert status["frame_age_s"]["front"] > 0.2          # FakeRobot 은 같은 프레임만 준다
    assert set(status["purple"]) == {"front", "wrist"}
    assert status["purple"]["front"] == {"ratio": 0.0, "thr": w.cfg.check.min_ratio_for("front")}
    assert status["retry_depth"] == 0.0
