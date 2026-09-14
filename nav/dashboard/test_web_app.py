import asyncio
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from fastapi.testclient import TestClient  # noqa: E402

from dashboard_state import DashboardState  # noqa: E402
from map_geometry import MapInfo  # noqa: E402
from web_app import create_app, make_stop_handlers, sse_events  # noqa: E402

INFO = MapInfo(0.05, -0.707, -0.919, 53, 48, Path("/nonexistent.pgm"))
META = {"resolution": 0.05, "origin_x": -0.707, "origin_y": -0.919, "width": 53, "height": 48,
        "waypoints": {"home": {"x": 0.0, "y": 0.0, "yaw": 0.0}}}


class FakePick:
    def __init__(self, estop_text="estop 보냄"):
        self.estops = 0
        self.estop_text = estop_text

    def estop(self):
        self.estops += 1
        return self.estop_text


def _handlers(reachable):
    state = DashboardState(INFO)
    pick = FakePick()
    published = []
    clock = iter([10.0, 10.05, 10.6, 11.0, 12.0]).__next__
    h = make_stop_handlers(state, pick, lambda: published.append("stop"), clock=clock)
    state.set_pick(reachable, {"state": "SEARCH"} if reachable else None, 9.0)
    return state, pick, published, h


def test_stop_calls_estop_then_publishes_when_pick_reachable():
    state, pick, published, (on_stop, on_release, on_tick, on_pick_update) = _handlers(True)
    res = on_stop()
    assert res == {"ok": True, "results": ["estop 보냄", "stop 발행"]}
    assert pick.estops == 1 and published == ["stop"]
    assert state.snapshot(10.0)["stop"]["last_result"] == "estop 보냄 · stop 발행"


def test_stop_skips_estop_when_pick_not_running():
    state, pick, published, (on_stop, *_rest) = _handlers(False)
    assert on_stop()["results"] == ["stop 발행"]
    assert pick.estops == 0 and published == ["stop"]


def test_tick_republishes_during_return_and_release_stops():
    state, pick, published, (on_stop, on_release, on_tick, on_pick_update) = _handlers(False)
    on_stop()                                   # clock 10.0
    state.on_bridge_state("returning", 10.05)
    on_tick()                                   # clock 10.05 → 0.05 s 뒤라 아직 안 보냄
    on_tick()                                   # clock 10.6 → 보냄
    assert published == ["stop", "stop"]
    assert on_release() == {"ok": True}
    on_tick()                                   # clock 11.0 → 해제됨
    assert published == ["stop", "stop"]


def test_pick_update_estops_when_web_comes_up_while_latched():
    state, pick, published, (on_stop, on_release, on_tick, on_pick_update) = _handlers(False)
    on_stop()
    on_pick_update(True, {"state": "SEARCH"})
    assert pick.estops == 1


def test_estop_failure_is_kept_and_stop_still_publishes():
    state = DashboardState(INFO)
    pick = FakePick(estop_text="estop 실패: timed out")
    published = []
    clock = iter([10.0, 10.05]).__next__
    on_stop, *_ = make_stop_handlers(state, pick, lambda: published.append("stop"), clock=clock)
    state.set_pick(True, {"state": "SEARCH"}, 9.0)
    res = on_stop()
    assert res == {"ok": True, "results": ["estop 실패: timed out", "stop 발행"]}
    assert published == ["stop"]
    assert state.snapshot(10.0)["stop"]["last_result"] == "estop 실패: timed out · stop 발행"


def test_sse_events_formats_json_events():
    async def run_test():
        async def is_disconnected():
            return False

        async def async_sleep(s):
            pass

        events = []
        async for event in sse_events(lambda: {"a": 1}, is_disconnected, interval_s=0.0, sleep=async_sleep, max_events=2):
            events.append(event)
        return events

    result = asyncio.run(run_test())
    assert result == ['data: {"a": 1}\n\n', 'data: {"a": 1}\n\n']


def test_sse_events_stops_when_client_disconnects():
    async def run_test():
        call_count = [0]

        async def is_disconnected():
            call_count[0] += 1
            return call_count[0] > 1

        async def async_sleep(s):
            pass

        events = []
        async for event in sse_events(lambda: {"a": 1}, is_disconnected, interval_s=0.0, sleep=async_sleep, max_events=None):
            events.append(event)
        return events

    result = asyncio.run(run_test())
    assert len(result) == 1 and result[0] == 'data: {"a": 1}\n\n'


def test_routes():
    state = DashboardState(INFO)
    client = TestClient(create_app(state, b"\x89PNG-fake", META, lambda: {"ok": True, "results": []},
                                   lambda: {"ok": True}))
    assert client.get("/").status_code == 200
    r = client.get("/map.png")
    assert r.headers["content-type"] == "image/png" and r.content == b"\x89PNG-fake"
    assert client.get("/api/map_info").json() == META
    assert client.post("/api/stop").json() == {"ok": True, "results": []}
    assert client.post("/api/stop/release").json() == {"ok": True}


def test_index_has_dashboard_regions_and_assets():
    client = TestClient(create_app(DashboardState(INFO), b"", META, lambda: {}, lambda: {}))
    html = client.get("/").text
    for element_id in ("conn", "mission-chip", "stop-btn", "latch-banner", "release-btn", "warn-banner",
                       "map", "legend", "stages", "status-text", "cam-front", "cam-wrist", "topics"):
        assert f'id="{element_id}"' in html, element_id
    assert "확실한 비상정지는 로봇 전원 스위치" in html
    assert client.get("/static/style.css").status_code == 200
    js = client.get("/static/app.js")
    assert js.status_code == 200 and "EventSource" in js.text
