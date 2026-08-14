from lekiwi_pill_pickup.servo_control import AlignmentError, compute_alignment_error, is_aligned


def test_compute_alignment_error_centered_and_at_target_distance():
    err = compute_alignment_error(
        bbox_center_x=320, bbox_center_y=240, frame_width=640, frame_height=480,
        current_distance_cm=15.0, target_distance_cm=15.0)
    assert err.x_offset_px == 0
    assert err.y_offset_px == 0
    assert err.distance_error_cm == 0


def test_compute_alignment_error_offset_right_and_down():
    err = compute_alignment_error(
        bbox_center_x=420, bbox_center_y=340, frame_width=640, frame_height=480,
        current_distance_cm=15.0, target_distance_cm=15.0)
    assert err.x_offset_px == 100
    assert err.y_offset_px == 100


def test_compute_alignment_error_too_far_gives_positive_distance_error():
    err = compute_alignment_error(
        bbox_center_x=320, bbox_center_y=240, frame_width=640, frame_height=480,
        current_distance_cm=25.0, target_distance_cm=15.0)
    assert err.distance_error_cm == 10.0


def test_is_aligned_true_within_tolerance():
    err = AlignmentError(x_offset_px=5, y_offset_px=-3, distance_error_cm=0.5)
    assert is_aligned(err, center_tolerance_px=10, distance_tolerance_cm=1.0) is True


def test_is_aligned_false_when_x_offset_exceeds_tolerance():
    err = AlignmentError(x_offset_px=50, y_offset_px=0, distance_error_cm=0.0)
    assert is_aligned(err, center_tolerance_px=10, distance_tolerance_cm=1.0) is False


def test_is_aligned_false_when_distance_error_exceeds_tolerance():
    err = AlignmentError(x_offset_px=0, y_offset_px=0, distance_error_cm=5.0)
    assert is_aligned(err, center_tolerance_px=10, distance_tolerance_cm=1.0) is False
