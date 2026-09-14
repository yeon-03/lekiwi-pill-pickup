import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import pytest  # noqa: E402

from dashboard_state import DashboardState  # noqa: E402
from map_geometry import MapInfo  # noqa: E402

INFO = MapInfo(0.05, -0.707, -0.919, 53, 48, Path("/nonexistent.pgm"))


def _with_pose(now=0.0):
    s = DashboardState(INFO)
    s.on_tf("map", "odom", (1.0, 0.0, math.pi / 2), now)
    s.on_tf("odom", "base_link", (1.0, 0.0, 0.0), now)
    return s


def test_empty_snapshot_is_json_and_has_all_keys():
    snap = DashboardState(INFO).snapshot(1.0)
    assert set(snap) == {"t", "pose", "scan", "particles", "plan", "trail", "mission", "stop", "pick", "topics", "warnings"}
    assert snap["pose"] is None
    assert snap["scan"] == {"pts": [], "age": None}
    assert snap["particles"] == {"pts": [], "n": 0, "age": None}
    assert snap["pick"] == {"reachable": False, "status": None, "age": None}
    assert list(snap["topics"])[:4] == ["/abo/state", "/abo/status", "/abo/pick_request", "/abo/pick_done"]
    assert snap["topics"]["/abo/command"]["dir"] == "out"
    json.dumps(snap)


def test_pose_is_composed_and_age_uses_older_transform():
    s = DashboardState(INFO)
    s.on_tf("map", "odom", (1.0, 0.0, math.pi / 2), 10.0)
    s.on_tf("odom", "base_link", (1.0, 0.0, 0.0), 12.0)
    s.on_amcl_pose(1.0, 1.0, 0.012, 12.0)
    p = s.snapshot(13.0)["pose"]
    assert (p["x"], p["y"]) == (pytest.approx(1.0), pytest.approx(1.0))
    assert p["yaw"] == pytest.approx(math.pi / 2, abs=1e-3)
    assert p["age"] == 3.0 and p["cov_xy"] == 0.012


def test_scan_uses_pose_at_receive_time():
    s = _with_pose()
    s.on_scan([1.0, float("inf")], 0.0, math.pi / 2, 0.05, 6.0, 0.5)
    snap = s.snapshot(1.0)
    assert snap["scan"] == {"pts": [[1.0, 2.0]], "age": 0.5}
    assert snap["topics"]["/scan"]["last"] == "2빔 · 유효 1"


def test_scan_without_pose_keeps_stats_only():
    s = DashboardState(INFO)
    s.on_scan([1.0], 0.0, 0.1, 0.05, 6.0, 1.0)
    snap = s.snapshot(2.0)
    assert snap["scan"]["pts"] == [] and snap["topics"]["/scan"]["age"] == 1.0


def test_particles_decimated():
    s = DashboardState(INFO, max_particles=3)
    s.on_particles([[i, 0.0] for i in range(9)], 1.0)
    snap = s.snapshot(1.5)["particles"]
    assert snap == {"pts": [[0, 0.0], [3, 0.0], [6, 0.0]], "n": 9, "age": 0.5}


def test_trail_step_and_reset_on_fetch():
    s = _with_pose()
    for i, x in enumerate([0.0, 0.01, 0.2, 0.21, 0.5]):
        s.on_odom((x, 0.0, 0.0), float(i))
    assert len(s.snapshot(5.0)["trail"]) == 3              # 0.0, 0.2, 0.5 (5 cm 미만 건너뜀)
    s.on_command("fetch center color:blue", 6.0)
    assert s.snapshot(6.0)["trail"] == []
    assert s.snapshot(6.0)["mission"]["color"] == "blue"


def test_map_mismatch_warning():
    s = DashboardState(INFO)
    s.on_map_msg(53, 48, 0.05, -0.707, -0.919, 1.0)
    assert s.snapshot(1.0)["warnings"] == []
    s.on_map_msg(60, 48, 0.05, -0.707, -0.919, 2.0)
    assert s.snapshot(2.0)["warnings"] == ["/map 이 파일 지도와 다름 (60x48, 0.050, -0.707,-0.919)"]


def test_stop_latch_flows_through_state():
    s = DashboardState(INFO)
    s.on_command("fetch center", 0.0)
    s.on_bridge_state("moving", 1.0)
    s.on_bridge_state("picking", 5.0)
    assert s.engage_stop(6.0) == ["estop", "stop"]
    s.on_bridge_state("idle", 6.1)                         # stop 응답 — 단계는 그대로
    assert s.snapshot(7.0)["mission"]["stages"][1]["state"] == "active"
    assert s.set_pick(True, {"state": "SEARCH"}, 7.0) == ["estop"]
    s.on_bridge_state("returning", 8.0)
    assert s.stop_tick(8.0) == ["stop"]
    s.note_stop("stop", "stop 발행")
    snap = s.snapshot(8.5)
    assert snap["stop"]["latched"] is True and snap["stop"]["last_result"] == "stop 발행"
    assert snap["pick"] == {"reachable": True, "status": {"state": "SEARCH"}, "age": 1.5}
    s.release_stop()
    assert s.snapshot(9.0)["stop"]["latched"] is False
