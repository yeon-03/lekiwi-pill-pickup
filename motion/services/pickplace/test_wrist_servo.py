"""WristServo 회귀 테스트 — 2026-09-13 실사용 중 발견된 버그 두 건.

1) REFINING(크기는 이미 목표 도달, 가로/세로만 다듬는 중)에서 손목캠이 물체를
   놓치면(너무 가까워져 박스가 화면에서 사라짐) LOST 로 멈춰 그리퍼를 영영 닫지
   못했다. 그리퍼만 닫으면 충분히 집을 수 있는 자세였는데도 READY 로 못 넘어간
   사례.
2) 위 수정을 front 카메라 확인(descend_hint) 없이 무조건 적용했더니, 실제로는
   잘 안 맞은 상태에서도 집으려 들어 정확도가 떨어졌다 — front 카메라가 "집게
   사이에 약통이 잘 왔다"고 확인해줄 때(+ 마지막으로 본 가로 정렬도 맞았을 때)만
   박스 소실을 근접으로 보고 READY 로 넘어가야 한다.
"""

from services.pickplace.wrist_servo import GraspArgs, WristServo
from services.pickplace.yolo_detect import Detection


def _servo() -> WristServo:
    cfg = GraspArgs(approach_mode="joints", reach_joints={"arm_shoulder_lift.pos": 5.0}, lost_timeout_s=0.5)
    start = {"arm_shoulder_pan.pos": 0.0, "arm_shoulder_lift.pos": 0.0, "arm_gripper.pos": 100.0}
    return WristServo(cfg, start, None)


def test_refining_box_disappearing_with_front_confirmation_reaches_ready():
    """front 카메라가 정중앙에 그리퍼(보라색)를 확인해준(descend_hint=True) 경우 —
    가로도 마지막 프레임에서 맞았으므로, 박스 소실을 근접으로 보고 READY 로 넘어간다."""
    servo = _servo()
    shape = (480, 640, 3)

    # 프레임 1: 폭 260(목표 도달), x_ok=True, 세로는 아직 안 맞음 → REFINING 진입
    det = Detection(name="pill", conf=0.9, xyxy=(310, 250, 570, 430), cls=0)
    servo.update([det], shape, dt=1 / 30, now=0.0, allow_motion=True)
    assert servo.state == "REFINING"

    # 프레임 2~: 손목캠은 물체를 놓쳤지만, front 카메라가 계속 높이를 확인해준다
    for i in range(1, 10):
        servo.update([], shape, dt=0.1, now=i * 0.1, allow_motion=True, descend_hint=True)

    assert servo.state == "READY"
    assert servo.done


def test_refining_box_disappearing_without_front_confirmation_stays_lost():
    """front 카메라 확인(descend_hint)이 없으면 — 실제로 잘 맞았는지 알 수 없으니
    무리하게 집지 않고 기존대로 LOST 로 멈춰야 한다 (2026-09-13 사용자 피드백:
    확인 없이 무조건 READY 로 넘기면 정확도가 떨어짐)."""
    servo = _servo()
    shape = (480, 640, 3)

    det = Detection(name="pill", conf=0.9, xyxy=(310, 250, 570, 430), cls=0)
    servo.update([det], shape, dt=1 / 30, now=0.0, allow_motion=True)
    assert servo.state == "REFINING"

    for i in range(1, 10):
        servo.update([], shape, dt=0.1, now=i * 0.1, allow_motion=True)  # descend_hint 기본값 False

    assert servo.state == "LOST"
    assert not servo.done


def test_centering_box_disappearing_still_goes_lost():
    """크기도 아직 안 됐는데(CENTERING) 놓친 경우는 기존대로 LOST 여야 한다 —
    이 픽스는 REFINING(크기 도달 후) 한정이다."""
    servo = _servo()
    shape = (480, 640, 3)

    # 폭 100 < 목표(260) → 아직 CENTERING/APPROACHING 단계
    det = Detection(name="pill", conf=0.9, xyxy=(400, 200, 500, 300), cls=0)
    servo.update([det], shape, dt=1 / 30, now=0.0, allow_motion=True)
    assert servo.state != "REFINING"

    for i in range(1, 10):
        servo.update([], shape, dt=0.1, now=i * 0.1, allow_motion=True)

    assert servo.state == "LOST"
    assert not servo.done


def test_y_anchor_bottom_targets_box_bottom_edge_not_center():
    """y_anchor=bottom 이면 박스 중심(center)이나 위 변(top)이 아니라 아래 변(y2)을
    기준으로 세로 오차를 계산해야 한다 (2026-09-13 사용자 피드백: 병 위쪽[뚜껑]을
    잡으면 안 잡히고, 아래쪽[몸통]을 겨냥해야 잘 잡힌다)."""
    cfg = GraspArgs(approach_mode="joints", reach_joints={"arm_shoulder_lift.pos": 5.0}, y_anchor="bottom")
    start = {"arm_shoulder_pan.pos": 0.0, "arm_shoulder_lift.pos": 0.0, "arm_wrist_flex.pos": 0.0, "arm_gripper.pos": 100.0}
    servo = WristServo(cfg, start, None)
    shape = (480, 640, 3)  # cy0 = 240

    det = Detection(name="pill", conf=0.9, xyxy=(200, 100, 400, 300), cls=0)  # top=100 center=200 bottom=300
    servo.update([det], shape, dt=1 / 30, now=0.0, allow_motion=True)

    assert servo.anchor_y == 300  # 아래 변(y2), 중심(200)도 위 변(100)도 아니다
    assert servo.dy == 300 - 240  # y_target_dy 기본값 0


def test_y_anchor_bottom_accepted_by_validate():
    GraspArgs(y_anchor="bottom").validate()
