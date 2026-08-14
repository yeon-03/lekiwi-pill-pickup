import pytest

from lekiwi_pill_pickup.distance_estimation import estimate_distance_cm


def test_estimate_distance_known_values():
    assert estimate_distance_cm(
        pixel_height=100, real_height_cm=10, focal_length_px=800) == pytest.approx(80.0)


def test_estimate_distance_closer_object_has_larger_pixel_height():
    far = estimate_distance_cm(pixel_height=50, real_height_cm=10, focal_length_px=800)
    near = estimate_distance_cm(pixel_height=200, real_height_cm=10, focal_length_px=800)
    assert near < far


def test_estimate_distance_raises_on_nonpositive_pixel_height():
    with pytest.raises(ValueError):
        estimate_distance_cm(pixel_height=0, real_height_cm=10, focal_length_px=800)
    with pytest.raises(ValueError):
        estimate_distance_cm(pixel_height=-5, real_height_cm=10, focal_length_px=800)
