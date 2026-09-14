"""정지 유지(stop latch) 판단. ROS·HTTP 호출은 하지 않고 '무엇을 보낼지'만 돌려준다.

로봇 브리지(PR #8)는 Nav2 목표가 있을 때만 stop 을 받아들인다. 집기 중에 멈추면
실패로 보고 복귀 주행을 시작하므로, 켜져 있는 동안 주행이 시작될 때마다 stop 을
다시 보낸다. 설계 문서 §정지 참고.
"""
from __future__ import annotations

MOVE = {"moving", "returning"}


class StopLatch:
    def __init__(self, window_s: float = 5.0, interval_s: float = 0.5) -> None:
        self.window_s, self.interval_s = window_s, interval_s
        self.latched = False
        self.since: float | None = None
        self.sent = 0
        self.last_result: str | None = None
        self._state: str | None = None
        self._window_start: float | None = None
        self._last_stop: float | None = None
        self._pick_up = False

    def engage(self, now: float) -> list[str]:
        if not self.latched:
            self.latched, self.since = True, now
        if self._state in MOVE:
            self._window_start = now
        self._last_stop = now
        self.sent += 1
        return ["estop", "stop"]

    def release(self) -> None:
        self.latched = False
        self.since = None
        self._window_start = None

    def on_bridge_state(self, state: str, now: float) -> None:
        if state in MOVE and state != self._state:
            self._window_start = now
        elif state not in MOVE:
            self._window_start = None
        self._state = state

    def on_pick_reachable(self, reachable: bool) -> list[str]:
        rising = reachable and not self._pick_up
        self._pick_up = reachable
        return ["estop"] if (rising and self.latched) else []

    def tick(self, now: float) -> list[str]:
        if not self.latched or self._window_start is None:
            return []
        if now - self._window_start > self.window_s:
            return []
        if self._last_stop is not None and now - self._last_stop < self.interval_s:
            return []
        self._last_stop = now
        self.sent += 1
        return ["stop"]

    def note(self, action: str, result: str) -> None:
        self.last_result = result

    def snapshot(self) -> dict:
        return {"latched": self.latched, "since": self.since, "sent": self.sent, "last_result": self.last_result}
