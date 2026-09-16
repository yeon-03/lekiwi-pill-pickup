import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import pytest  # noqa: E402

from map_geometry import MapInfo  # noqa: E402
from preview_server import SCENARIOS, build_state  # noqa: E402

INFO = MapInfo(0.05, -0.707, -0.919, 53, 48, Path("/nonexistent.pgm"))
NOW = 1_789_350_000.0


def _stages(state):
    return [st["state"] for st in state.snapshot(NOW)["mission"]["stages"]]


def test_scenarios_list():
    assert SCENARIOS == ("idle", "driving", "picking", "done", "failed")


@pytest.mark.parametrize("scenario, stages, outcome, reachable", [
    ("idle", [], None, False),
    ("driving", ["active", "todo", "todo"], None, False),
    ("picking", ["done", "active", "todo"], None, True),
    ("done", ["done", "done", "done"], "done", False),
    ("failed", ["done", "failed", "done"], "failed", False),
])
def test_build_state_scenarios(scenario, stages, outcome, reachable):
    st = build_state(scenario, NOW, info=INFO)
    snap = st.snapshot(NOW)
    assert _stages(st) == stages
    assert snap["mission"]["outcome"] == outcome
    assert snap["pick"]["reachable"] is reachable
    assert snap["pose"] is not None and snap["scan"]["pts"]
    assert snap["topics"]["/scan"]["age"] is not None


def test_picking_scenario_has_worker_status_and_notes():
    snap = build_state("picking", NOW, info=INFO).snapshot(NOW)
    status = snap["pick"]["status"]
    assert (status["pick_attempts"], status["max_pick_attempts"]) == (2, 5)
    assert status["purple"]["front"] == {"ratio": 0.27, "thr": 0.23}
    m = snap["mission"]
    assert m["color"] == "blue" and m["headline"] == "파란색 약을 집는 중"
    assert m["stages"][0]["note"] and m["stages"][1]["note"]
    assert all(st["start_at"] is not None and st["start_at"] <= NOW for st in m["stages"][:2])


def test_failed_scenario_headline():
    m = build_state("failed", NOW, info=INFO).snapshot(NOW)["mission"]
    assert m["headline"].endswith("약을 가져오지 못했어요")


def test_unknown_scenario_rejected():
    with pytest.raises(ValueError):
        build_state("flying", time.time(), info=INFO)
