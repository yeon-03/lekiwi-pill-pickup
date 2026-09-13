"""정지 시 현재 자세 → 시작 자세로 천천히 되돌아가는 보간 재생기.

Physical Labs GUI 앱(`pick_worker.py`)의 `_roll_out` 안전 동작을 참고해 이 레포에
독립적으로 새로 구현했다 — 원본 CLI(`lekiwi_yolo_pick.py`)에는 이 동작이 없다
(정지 시 그냥 현재 자세를 유지한 채 바퀴만 세우고 즉시 연결을 끊는다). 그리퍼는
절대 건드리지 않는다 — 물건을 물고 있을 수 있다.
"""
from __future__ import annotations


class RollOutPlayer:
    def __init__(self, current: dict[str, float], home: dict[str, float], duration_s: float):
        self._current = dict(current)
        self._home = dict(home)
        self._joints = [k for k in home if k in current and not k.endswith("gripper.pos")]
        self._duration = max(duration_s, 1e-3)
        self._t0: float | None = None
        self.done = not self._joints
        if self._joints:
            max_delta = max(abs(self._home[k] - self._current[k]) for k in self._joints)
            if max_delta < 0.5:
                self.done = True

    def update(self, now: float) -> dict[str, float]:
        """`팔 목표 자세` 를 돌려준다. `self.done` 이 True 면 이미 시작 자세."""
        if self.done:
            return dict(self._current)
        if self._t0 is None:
            self._t0 = now
        ratio = min(1.0, (now - self._t0) / self._duration)
        pose = dict(self._current)
        for k in self._joints:
            pose[k] = self._current[k] + (self._home[k] - self._current[k]) * ratio
        if ratio >= 1.0:
            self.done = True
        return pose
