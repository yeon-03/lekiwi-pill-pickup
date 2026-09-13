"""Pick&Place 시연용 FastAPI 앱 — PickPlaceHeadlessWorker 를 감싸는 얇은 HTTP 계층.

`/estop`/`/status` 는 플래그 세팅·dict 복사뿐인 가벼운 동기 함수라 FastAPI 가
자동으로 스레드풀에서 돌린다 (`async def` 를 안 쓰면 이렇게 된다) — 영상 스트리밍
같은 무거운 핸들러에 발목 잡히지 않는다 (설계 문서 §3).
"""
from __future__ import annotations

import time
from pathlib import Path
from typing import Any, Iterator

from fastapi import FastAPI
from fastapi.responses import HTMLResponse, StreamingResponse

_STATIC_DIR = Path(__file__).resolve().parent / "static"
_BOUNDARY = b"frame"


def mjpeg_frames(worker: Any, view: str, poll_interval_s: float = 0.1) -> Iterator[bytes]:
    """무한 MJPEG 프레임 제너레이터 — 모듈 레벨 함수로 둬서 실제 HTTP 왕복 없이
    `next()` 한 번으로 직접 테스트할 수 있게 한다 (TestClient 로 끝까지 스트리밍을
    시키면 스레드풀 워커가 영원히 남아 테스트 프로세스가 종료되지 않는다)."""
    while True:
        frame = worker.frames.get(view)
        if frame is not None:
            yield (
                b"--" + _BOUNDARY + b"\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
            )
        time.sleep(poll_interval_s)


def create_app(worker: Any) -> FastAPI:
    app = FastAPI()

    @app.get("/", response_class=HTMLResponse)
    def index() -> str:
        return (_STATIC_DIR / "index.html").read_text()

    @app.post("/start")
    def start() -> dict[str, bool]:
        worker.resume()
        return {"ok": True}

    @app.post("/pause")
    def pause() -> dict[str, bool]:
        worker.pause()
        return {"ok": True}

    @app.post("/stop")
    def stop() -> dict[str, bool]:
        worker.request_stop()
        return {"ok": True}

    @app.post("/estop")
    def estop() -> dict[str, bool]:
        worker.request_abort()
        return {"ok": True}

    @app.post("/restart")
    def restart() -> dict[str, bool]:
        worker.restart()
        return {"ok": True}

    @app.post("/home")
    def home() -> dict[str, bool]:
        worker.go_home()
        return {"ok": True}

    @app.get("/status")
    def status() -> dict[str, Any]:
        return worker.status.get()

    @app.get("/stream/{view}")
    def stream(view: str) -> StreamingResponse:
        # 동기 제너레이터를 StreamingResponse 에 넘기면 FastAPI/Starlette 가
        # 스레드풀에서 next() 를 호출한다 — time.sleep 이 메인 asyncio 이벤트
        # 루프를 막지 않는다.
        return StreamingResponse(
            mjpeg_frames(worker, view),
            media_type=f"multipart/x-mixed-replace; boundary={_BOUNDARY.decode()}",
        )

    return app
