import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import dashboard_node  # noqa: E402  (rclpy 를 import 하지 않아야 한다)


def test_defaults_point_to_repo_files():
    a = dashboard_node.parse_args([])
    assert Path(a.map).is_file() and a.map.endswith("nav/maps/lekiwi01/map_0912_1654.yaml")
    assert Path(a.waypoints).is_file()
    assert (a.host, a.port, a.pick_url, a.no_particles) == ("127.0.0.1", 8001, "http://127.0.0.1:8000", False)


def test_flags():
    a = dashboard_node.parse_args(["--host", "0.0.0.0", "--port", "9001", "--no-particles"])
    assert (a.host, a.port, a.no_particles) == ("0.0.0.0", 9001, True)


def test_module_does_not_import_rclpy_at_top():
    assert "rclpy" not in dashboard_node.__dict__
