"""백그라운드 제어 스레드와 FastAPI 요청 핸들러 사이의 스레드-세이프 공유 상태.

락은 참조 교체에만 쓴다 — JPEG 인코딩·dict 조립은 락 밖에서 한다. 그래야 영상
스트리밍 요청이 30Hz 제어 루프를 막지 않는다 (설계 문서 §3, 독립 검토 지적사항 반영).
"""
from __future__ import annotations

import threading
from typing import Any


class LatestFrame:
    """뷰 이름 → 최신 JPEG bytes."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._frames: dict[str, bytes] = {}

    def set(self, view: str, jpeg_bytes: bytes) -> None:
        with self._lock:
            self._frames[view] = jpeg_bytes

    def get(self, view: str) -> bytes | None:
        with self._lock:
            return self._frames.get(view)


class SharedStatus:
    """최신 상태 dict."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._status: dict[str, Any] = {}

    def set(self, status: dict[str, Any]) -> None:
        with self._lock:
            self._status = dict(status)

    def get(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._status)
