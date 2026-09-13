import pytest

from services.pickplace import PickPlaceError
from services.pickplace.grasp_check import GraspCheckArgs


def test_front_uses_its_own_ratio_threshold():
    cfg = GraspCheckArgs(min_purple_ratio=0.15, front_min_purple_ratio=0.23)
    assert cfg.min_ratio_for("front") == 0.23
    assert cfg.min_ratio_for("wrist") == 0.15


def test_front_falls_back_to_shared_threshold_when_unset():
    cfg = GraspCheckArgs(min_purple_ratio=0.15)
    assert cfg.min_ratio_for("front") == 0.15


def test_front_ratio_between_shared_and_front_threshold_is_not_ok():
    """front 비율 0.20 은 공통 문턱(0.15)은 넘지만 front 문턱(0.23)은 못 넘으므로 실패여야 한다."""
    import numpy as np

    from services.pickplace.grasp_check import GraspChecker
    from services.pickplace.yolo_detect import Detection

    cfg = GraspCheckArgs(min_purple_ratio=0.15, front_min_purple_ratio=0.23)
    checker = GraspChecker(cfg)
    frame = np.zeros((100, 100, 3), dtype=np.uint8)
    det = Detection(name="pill", conf=0.9, xyxy=(40, 40, 60, 60), cls=0)

    import services.pickplace.grasp_check as gc

    orig = gc.strip_ratio
    gc.strip_ratio = lambda mask, strip: 0.20
    try:
        assert checker._check_view(frame, [det], "front").ok is False
        assert checker._check_view(frame, [det], "wrist").ok is True
    finally:
        gc.strip_ratio = orig


def test_front_min_purple_ratio_validated():
    with pytest.raises(PickPlaceError):
        GraspCheckArgs(front_min_purple_ratio=1.5).validate()
