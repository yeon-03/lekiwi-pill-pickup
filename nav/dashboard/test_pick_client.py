import json
import sys
import threading
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from pick_client import PickClient  # noqa: E402


def test_poll_ok_and_fail():
    calls = []

    def fetch(method, url, timeout):
        calls.append((method, url, timeout))
        return json.dumps({"state": "SEARCH"}).encode()

    assert PickClient("http://h:8000", 0.3, fetch).poll_once() == (True, {"state": "SEARCH"})
    assert calls == [("GET", "http://h:8000/status", 0.3)]

    def boom(method, url, timeout):
        raise OSError("refused")

    assert PickClient(fetch=boom).poll_once() == (False, None)


def test_estop_result_text():
    seen = []
    assert PickClient("http://h:8000", fetch=lambda m, u, t: seen.append((m, u)) or b"{}").estop() == "estop 보냄"
    assert seen == [("POST", "http://h:8000/estop")]

    def boom(m, u, t):
        raise TimeoutError("timed out")

    assert PickClient(fetch=boom).estop() == "estop 실패: timed out"


def test_estop_connection_refused_is_clear():
    import urllib.error

    def refused(m, u, t):
        raise ConnectionRefusedError(111, "Connection refused")

    def url_refused(m, u, t):
        raise urllib.error.URLError(ConnectionRefusedError(111, "Connection refused"))

    assert PickClient(fetch=refused).estop() == "estop 불가 (8000 응답 없음)"
    assert PickClient(fetch=url_refused).estop() == "estop 불가 (8000 응답 없음)"


def test_estop_real_refused_port():
    import socket
    with socket.socket() as s:                       # 비어 있는 포트 — 아무도 듣지 않는다
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    assert PickClient(f"http://127.0.0.1:{port}").estop() == "estop 불가 (8000 응답 없음)"


def test_start_calls_update_in_background():
    got = threading.Event()
    PickClient(fetch=lambda m, u, t: b'{"state": "X"}').start(
        lambda ok, st: got.set() if (ok, st) == (True, {"state": "X"}) else None, interval_s=0.01)
    assert got.wait(1.0)


def test_start_keeps_polling_after_update_raises():
    calls = []
    got = threading.Event()

    def on_update(ok, st):
        calls.append((ok, st))
        if len(calls) == 1:
            raise RuntimeError("simulated error")
        got.set()

    PickClient(fetch=lambda m, u, t: b'{"state": "X"}').start(on_update, interval_s=0.01)
    assert got.wait(1.0), "Second call should arrive after first raises"
    assert len(calls) >= 2, f"Expected at least 2 calls, got {len(calls)}"
