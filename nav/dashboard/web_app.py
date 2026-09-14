"""대시보드 HTTP 계층: 페이지·지도 이미지·SSE 스냅샷·정지 API.

SSE 는 비동기 제너레이터를 StreamingResponse 에 넘긴다 — 클라이언트 연결 해제를 감지하므로
리소스 누수가 없다.
"""
from __future__ import annotations

import asyncio
import json
import time
from pathlib import Path
from typing import AsyncIterator

from fastapi import FastAPI, Request
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
            results.append(text)
        if results:
            state.note_stop(actions[-1], " · ".join(results))
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


async def sse_events(snapshot_fn, is_disconnected, interval_s: float = 0.2, sleep=asyncio.sleep,
                     max_events: int | None = None) -> AsyncIterator[str]:
    n = 0
    while max_events is None or n < max_events:
        if await is_disconnected():
            return
        yield f"data: {json.dumps(snapshot_fn(), ensure_ascii=False)}\n\n"
        n += 1
        await sleep(interval_s)


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
    async def events(request: Request) -> StreamingResponse:
        return StreamingResponse(sse_events(lambda: state.snapshot(time.time()), request.is_disconnected), media_type="text/event-stream",
                                 headers={"Cache-Control": "no-cache"})

    @app.post("/api/stop")
    def stop() -> dict:
        return on_stop()

    @app.post("/api/stop/release")
    def release() -> dict:
        return on_release()

    return app
