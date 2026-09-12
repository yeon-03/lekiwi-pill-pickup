"""LeKiwi "YOLO + Pick&Place" 순수 제어 로직 패키지.

원본: `/home/khw/workspace/lekiwi/yolo_and_pick/` (CLI 도구, 2026-09-04 실물 성공 확인).
기획서: `features/workspace_yolo_pick_place.md` §4 (재사용 경계).

검증된 게인·상태 전이·판별 규칙은 원본과 **한 줄도 다르지 않게** 유지한다. 바뀐 것은
CLI 파서·cv2 창·`SystemExit`·상대 경로 기본값·lerobot 로봇 의존 같은 I/O 껍데기뿐이다.
로직(게인·상태기)을 고칠 때는 원본 폴더와 이 패키지 양쪽을 함께 고칠 것.
"""


class PickPlaceError(RuntimeError):
    """원본의 `SystemExit` 대신 쓰는 예외. 메시지는 한국어를 유지한다."""


# 아래 import 들은 PickPlaceError 정의 *뒤*에 와야 한다 — 각 서브모듈이
# `from services.pickplace import PickPlaceError` 로 이 클래스를 되가져오기 때문
# (패키지가 아직 초기화되는 중이라도, 이 시점엔 이미 속성이 존재한다).
from services.pickplace.yolo_detect import (  # noqa: E402
    BOX_COLORS,
    CROSSHAIR_COLOR,
    Detection,
    YoloArgs,
    draw,
    draw_crosshair,
    hstack_views,
    infer,
    load_model,
)
from services.pickplace.approach import (  # noqa: E402
    BAND_COLOR,
    IGNORED_COLOR,
    LINE_COLOR,
    SIZE_REF_COLOR,
    STATE_COLORS,
    STOP,
    TARGET_COLOR,
    ApproachArgs,
    Approacher,
    box_size,
    draw_alignment,
    largest,
    p_speed,
    status_line,
)
from services.pickplace.wrist_servo import (  # noqa: E402
    GRIPPER_JOINT,
    GraspArgs,
    WristServo,
    filter_wrist_dets,
    save_reference,
)
from services.pickplace.arm_sequencer import ArmSequencer, PickArgs, draw_wrist_servo  # noqa: E402
from services.pickplace.grasp_check import (  # noqa: E402
    GraspCheckArgs,
    GraspChecker,
    ViewResult,
    center_purple_ratio,
    draw_grasp_check,
    edge_strips,
    purple_mask,
    strip_ratio,
)
from services.pickplace.poses import load_pose, pose_summary, read_arm_pose, save_pose  # noqa: E402
from services.pickplace.config import PickPlaceConfig  # noqa: E402

__all__ = [
    "PickPlaceError",
    "PickPlaceConfig",
    # yolo_detect
    "YoloArgs",
    "Detection",
    "load_model",
    "infer",
    "draw",
    "draw_crosshair",
    "hstack_views",
    "BOX_COLORS",
    "CROSSHAIR_COLOR",
    # approach
    "ApproachArgs",
    "Approacher",
    "p_speed",
    "box_size",
    "largest",
    "draw_alignment",
    "status_line",
    "STOP",
    "STATE_COLORS",
    "LINE_COLOR",
    "BAND_COLOR",
    "TARGET_COLOR",
    "SIZE_REF_COLOR",
    "IGNORED_COLOR",
    # arm_sequencer
    "PickArgs",
    "ArmSequencer",
    "draw_wrist_servo",
    # wrist_servo
    "GraspArgs",
    "WristServo",
    "filter_wrist_dets",
    "save_reference",
    "GRIPPER_JOINT",
    # grasp_check
    "GraspCheckArgs",
    "GraspChecker",
    "ViewResult",
    "center_purple_ratio",
    "draw_grasp_check",
    "purple_mask",
    "edge_strips",
    "strip_ratio",
    # poses
    "load_pose",
    "read_arm_pose",
    "save_pose",
    "pose_summary",
]
