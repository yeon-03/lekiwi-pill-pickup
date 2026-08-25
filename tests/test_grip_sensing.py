from lekiwi_pill_pickup.grip_sensing import grasped_something


def test_grasped_something_true_when_load_exceeds_threshold():
    assert grasped_something(load_reading=500, empty_close_load=100, margin=50) is True


def test_grasped_something_false_when_load_at_empty_baseline():
    assert grasped_something(load_reading=100, empty_close_load=100, margin=50) is False


def test_grasped_something_false_within_margin_of_baseline():
    assert grasped_something(load_reading=120, empty_close_load=100, margin=50) is False


def test_grasped_something_true_just_above_margin():
    assert grasped_something(load_reading=151, empty_close_load=100, margin=50) is True
