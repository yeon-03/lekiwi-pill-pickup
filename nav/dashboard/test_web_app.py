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
    pick = FakePick("estop 보냄" if reachable else "estop 불가 (8000 응답 없음)")
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


def test_stop_always_tries_estop_and_reports_unreachable_pick():
    # 캐시된 reachable 로 estop 을 건너뛰면 8000 이 잠깐 안 보일 때 팔이 멈췄다고 착각한다.
    state, pick, published, (on_stop, *_rest) = _handlers(False)
    assert on_stop()["results"] == ["estop 불가 (8000 응답 없음)", "stop 발행"]
    assert pick.estops == 1 and published == ["stop"]
    assert state.snapshot(10.0)["stop"]["last_result"] == "estop 불가 (8000 응답 없음) · stop 발행"


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
    before = pick.estops
    on_pick_update(True, {"state": "SEARCH"})
    assert pick.estops == before + 1


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


def test_camera_placeholder_hides_img_alt_text():
    # 스트림이 없을 때 <img> 의 alt 글자가 .cam-empty 안내와 겹치지 않게 (자리는 유지).
    client = TestClient(create_app(DashboardState(INFO), b"", META, lambda: {}, lambda: {}))
    css = client.get("/static/style.css").text
    assert ".cam:not(.live) img { visibility: hidden; }" in css


def test_font_size_tokens_used():
    # 멀리서도 읽히게 글자 크기를 토큰으로 모은다 — 테마와 무관하게 bare :root 에 정의.
    import re
    css = (Path(__file__).resolve().parent / "static" / "style.css").read_text(encoding="utf-8")
    root = re.search(r"^:root \{(.*?)^\}", css, re.S | re.M)
    assert root and "--ui-scale: 1;" in root.group(1)
    for name in ("--fs-xs", "--fs-sm", "--fs-md", "--fs-lg", "--fs-xl"):
        decl = re.search(rf"{name}:\s*([^;]+);", root.group(1))
        assert decl, name
        assert "clamp(" in decl.group(1) and "var(--ui-scale)" in decl.group(1), decl.group(1)
    raw = [ln for ln in css.splitlines() if "font-size:" in ln and "var(--fs-" not in ln]
    assert raw == [], raw
    assert not re.search(r"font-size:\s*\d+px", css)
    html = (Path(__file__).resolve().parent / "static" / "index.html").read_text(encoding="utf-8")
    assert 'id="font-up"' in html and 'id="font-down"' in html
    js = (Path(__file__).resolve().parent / "static" / "app.js").read_text(encoding="utf-8")
    assert "lekiwi-dashboard-ui-scale" in js and js.index('$("stop-btn").addEventListener') < js.index('$("font-up")')


def test_stop_and_release_requests_report_failure():
    client = TestClient(create_app(DashboardState(INFO), b"", META, lambda: {}, lambda: {}))
    js = client.get("/static/app.js").text
    assert "정지 요청 실패" in js and "로봇 전원 스위치를 사용하세요" in js
    assert "해제 요청 실패" in js


def test_stop_button_is_wired_before_map_info_fetch():
    client = TestClient(create_app(DashboardState(INFO), b"", META, lambda: {}, lambda: {}))
    js = client.get("/static/app.js").text
    stop_listener_idx = js.index('$("stop-btn").addEventListener')
    map_info_fetch_idx = js.index('fetch("/api/map_info")')
    event_source_idx = js.index('new EventSource("/api/events")')
    assert stop_listener_idx < map_info_fetch_idx, "stop button listener should be wired before map info fetch"
    assert event_source_idx < map_info_fetch_idx, "EventSource should be created before map info fetch"
    assert " s 전" not in js, "Should not have English 's 전' unit in app.js"
    assert "ms 전" not in js, "Should not have English 'ms 전' unit in app.js"


def test_one_screen_layout_and_pipeline_ids():
    import re
    here = Path(__file__).resolve().parent / "static"
    css = (here / "style.css").read_text(encoding="utf-8")
    m = re.search(r"@media \(min-width: 1280px\) \{(.*?)^\}", css, re.S | re.M)
    assert m, "wide-screen block missing"
    assert re.search(r"height:\s*100d?vh", m.group(1)) and "grid-template-rows" in m.group(1)
    html = (here / "index.html").read_text(encoding="utf-8")
    for element_id in ("headline", "topics-summary", "topics-toggle", "stages", "status-text",
                       "mission-chip", "topics", "cam-title-target", "target-badge"):
        assert f'id="{element_id}"' in html, element_id
    js = (here / "app.js").read_text(encoding="utf-8")
    for hexcolor in ("#D93636", "#2E9E4F", "#2F6BD8"):
        assert hexcolor in js, hexcolor
    for word in ("빨간색", "초록색", "파란색", "▼"):
        assert word in js, word
