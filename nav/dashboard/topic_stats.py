"""토픽별 수신 주기·나이·마지막 값 요약. 시각은 노트북이 받은 시각. ROS 없음."""
from __future__ import annotations

from collections import deque


class _Entry:
    def __init__(self, direction: str):
        self.dir = direction
        self.times: deque[float] = deque(maxlen=400)
        self.last = ""


class TopicStats:
    def __init__(self, rate_window_s: float = 5.0) -> None:
        self.window = rate_window_s
        self._topics: dict[str, _Entry] = {}

    def declare(self, topic: str, direction: str) -> None:
        self._topics.setdefault(topic, _Entry(direction))

    def record(self, topic: str, now: float, summary: str) -> None:
        e = self._topics.setdefault(topic, _Entry("in"))
        e.times.append(now)
        e.last = summary

    def snapshot(self, now: float) -> dict[str, dict]:
        out = {}
        for topic, e in self._topics.items():
            recent = sum(1 for t in e.times if now - self.window <= t <= now)
            out[topic] = {
                "dir": e.dir,
                "rate": round(recent / self.window, 1),
                "age": round(now - e.times[-1], 2) if e.times else None,
                "last": e.last,
            }
        return out
