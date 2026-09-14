"""대시보드 HTTP 계층: 페이지·지도 이미지·SSE 스냅샷·정지 API.

SSE 는 동기 제너레이터를 StreamingResponse 에 넘긴다 — Starlette 가 스레드풀에서 next() 를
부르므로 time.sleep 이 이벤트 루프를 막지 않는다 (motion/webui/app.py 와 같은 방식).
"""
from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Iterator

from fastapi import FastAPI
from fastapi.responses import HTMLResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles

STATIC = Path(__file__).resolve().parent / "static"


def make_stop_handlers(state, pick, publish_stop, clock=time.time):
    def run(actions: list[str], reachable: bool) -> list[str]:
        results = []
        for a in actions:
            if a == "estop":
                if not reachable:
                    continue
                text = pick.estop()
            else:
                publish_stop()
                text = "stop 발행"
            state.note_stop(a, text)
            results.append(text)
        return results

    def on_stop() -> dict:
        now = clock()
        reachable = state.snapshot(now)["pick"]["reachable"]
        return {"ok": True, "results": run(state.engage_stop(now), reachable)}

    def on_release() -> dict:
        state.release_stop()
        return {"ok": True}

    def on_tick() -> None:
        run(state.stop_tick(clock()), reachable=False)

    def on_pick_update(reachable: bool, status: dict | None) -> None:
        run(state.set_pick(reachable, status, clock()), reachable=reachable)

    return on_stop, on_release, on_tick, on_pick_update


def sse_stream(snapshot_fn, interval_s: float = 0.2, sleep=time.sleep, max_events: int | None = None) -> Iterator[str]:
    n = 0
    while max_events is None or n < max_events:
        yield f"data: {json.dumps(snapshot_fn(), ensure_ascii=False)}\n\n"
        n += 1
        sleep(interval_s)


def create_app(state, map_png: bytes, map_meta: dict, on_stop, on_release) -> FastAPI:
    app = FastAPI()
    app.mount("/static", StaticFiles(directory=STATIC), name="static")

    @app.get("/", response_class=HTMLResponse)
    def index() -> str:
        return (STATIC / "index.html").read_text(encoding="utf-8")

    @app.get("/map.png")
    def map_png_route() -> Response:
        return Response(map_png, media_type="image/png")

    @app.get("/api/map_info")
    def map_info() -> dict:
        return map_meta

    @app.get("/api/events")
    def events() -> StreamingResponse:
        return StreamingResponse(sse_stream(lambda: state.snapshot(time.time())), media_type="text/event-stream",
                                 headers={"Cache-Control": "no-cache"})

    @app.post("/api/stop")
    def stop() -> dict:
        return on_stop()

    @app.post("/api/stop/release")
    def release() -> dict:
        return on_release()

    return app
