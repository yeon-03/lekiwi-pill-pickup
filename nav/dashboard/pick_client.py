"""집기 워커 웹(:8000, pick_worker_cycle --web-port)의 /status 폴링과 /estop 호출.

브라우저가 아니라 대시보드 서버가 부르므로 8000 쪽 CORS 가 필요 없다. 8000 은 집기
중에만 떠 있으니 연결 실패는 정상 상황이다(짧은 타임아웃, 예외 없이 False).
"""
from __future__ import annotations

import json
import threading
import time
import urllib.error
import urllib.request


def _urllib_fetch(method: str, url: str, timeout: float) -> bytes:
    req = urllib.request.Request(url, method=method, data=b"" if method == "POST" else None)
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return resp.read()


def _refused(exc: BaseException) -> bool:
    if isinstance(exc, ConnectionRefusedError):
        return True
    return isinstance(exc, urllib.error.URLError) and isinstance(exc.reason, ConnectionRefusedError)


class PickClient:
    def __init__(self, base_url: str = "http://127.0.0.1:8000", timeout_s: float = 0.3, fetch=None) -> None:
        self.base = base_url.rstrip("/")
        self.timeout = timeout_s
        self.fetch = fetch or _urllib_fetch

    def poll_once(self) -> tuple[bool, dict | None]:
        try:
            return True, json.loads(self.fetch("GET", f"{self.base}/status", self.timeout) or b"{}")
        except Exception:
            return False, None

    def estop(self) -> str:
        try:
            self.fetch("POST", f"{self.base}/estop", self.timeout)
            return "estop 보냄"
        except Exception as exc:
            if _refused(exc):
                return "estop 불가 (8000 응답 없음)"
            return f"estop 실패: {exc}"

    def start(self, on_update, interval_s: float = 0.2) -> threading.Thread:
        def loop():
            while True:
                try:
                    ok, status = self.poll_once()
                    on_update(ok, status)
                except Exception as exc:
                    print(f"[pick_client] 경고: 상태 갱신 오류 {type(exc).__name__}: {exc}", flush=True)
                finally:
                    time.sleep(interval_s)

        th = threading.Thread(target=loop, daemon=True, name="pick-client")
        th.start()
        return th
