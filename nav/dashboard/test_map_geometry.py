import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import pytest  # noqa: E402

import map_geometry as mg  # noqa: E402

REPO = Path(__file__).resolve().parents[2]


def _write_map(tmp_path, w=4, h=3):
    pgm = tmp_path / "m.pgm"
    pgm.write_bytes(b"P5\n%d %d\n255\n" % (w, h) + bytes([254] * (w * h)))
    y = tmp_path / "m.yaml"
    y.write_text("image: m.pgm\nresolution: 0.050\norigin: [-0.707, -0.919, 0]\n")
    return y


def test_load_map_info_reads_yaml_and_pgm_size(tmp_path):
    info = mg.load_map_info(_write_map(tmp_path))
    assert (info.width, info.height) == (4, 3)
    assert info.resolution == pytest.approx(0.05)
    assert (info.origin_x, info.origin_y) == (pytest.approx(-0.707), pytest.approx(-0.919))
    assert info.image_path.name == "m.pgm"


def test_real_lekiwi01_map():
    info = mg.load_map_info(REPO / "nav/maps/lekiwi01/map_0912_1654.yaml")
    assert (info.width, info.height) == (53, 48)


def test_map_png_bytes_is_png(tmp_path):
    data = mg.map_png_bytes(mg.load_map_info(_write_map(tmp_path)))
    assert data[:8] == b"\x89PNG\r\n\x1a\n"


def test_world_to_pixel_flips_y(tmp_path):
    info = mg.load_map_info(_write_map(tmp_path))          # origin (-0.707,-0.919), h=3
    px, py = mg.world_to_pixel(info, -0.707, -0.919)       # 원점 = 왼쪽 아래 모서리
    assert (px, py) == (pytest.approx(0.0), pytest.approx(3.0))
    px, py = mg.world_to_pixel(info, -0.707 + 0.10, -0.919 + 0.05)
    assert (px, py) == (pytest.approx(2.0), pytest.approx(2.0))


def test_load_waypoints_real_file():
    wp = mg.load_waypoints(REPO / "nav/mission/waypoints.yaml")
    assert wp["home"] == {"x": 0.0, "y": 0.0, "yaw": 0.0}
    assert wp["center"]["x"] == pytest.approx(1.10)


def test_yaw_from_quat_90deg():
    s = math.sin(math.pi / 4)
    assert mg.yaw_from_quat(0.0, 0.0, s, s) == pytest.approx(math.pi / 2)


def test_compose_rotates_child_offset():
    a = (1.0, 2.0, math.pi / 2)
    b = (1.0, 0.0, 0.1)
    x, y, yaw = mg.compose(a, b)
    assert (x, y, yaw) == (pytest.approx(1.0), pytest.approx(3.0), pytest.approx(math.pi / 2 + 0.1))


def test_scan_points_filters_invalid_and_transforms():
    ranges = [1.0, float("inf"), 0.01, 2.0, float("nan")]
    pts = mg.scan_points(ranges, angle_min=0.0, angle_increment=math.pi / 2,
                         range_min=0.05, range_max=6.0, robot=(1.0, 1.0, math.pi / 2))
    # 빔0: 전방 1 m, 로봇이 +90° → map 에서 +y 로 1 m.  빔3: 각도 270°(=-90°) 2 m → map +x 로 2 m
    assert pts == [[1.0, 2.0], [3.0, 1.0]]


def test_decimate_even_spacing():
    pts = [[i, 0] for i in range(10)]
    assert mg.decimate(pts, 5) == [[0, 0], [2, 0], [4, 0], [6, 0], [8, 0]]
    assert mg.decimate(pts, 20) == pts
