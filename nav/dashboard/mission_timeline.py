"""브리지 /abo/state 전이로 미션 단계(주행 → 집기 → 복귀)를 만든다. ROS 없음."""
from __future__ import annotations

END_OK = {"done"}
END_FAIL = {"failed", "rejected", "canceled"}


class _Stage:
    def __init__(self, key: str, label: str):
        self.key, self.label = key, label
        self.state = "todo"
        self.start: float | None = None
        self.end: float | None = None

    def begin(self, now: float) -> None:
        self.state, self.start = "active", now

    def finish(self, now: float, ok: bool) -> None:
        self.state, self.end = ("done" if ok else "failed"), now

    def as_dict(self, now: float) -> dict:
        sec = None if self.start is None else round((self.end if self.end is not None else now) - self.start, 1)
        return {"key": self.key, "label": self.label, "state": self.state, "sec": sec}


class MissionTimeline:
    def __init__(self) -> None:
        self.state: str | None = None
        self.status = ""
        self.command = ""
        self.target: str | None = None
        self.color: str | None = None
        self.started_at: float | None = None
        self.ended_at: float | None = None
        self.stages: list[_Stage] = []

    def on_command(self, text: str, now: float) -> None:
        self.command = text.strip()
        tok = self.command.split()
        if not tok or tok[0].lower() != "fetch" or len(tok) < 2:
            return
        self.target = tok[1]
        self.color = next((t.split(":", 1)[1] for t in tok[2:] if t.lower().startswith("color:")), None)
        ret = tok[tok.index("to") + 1] if "to" in tok[2:-1] else "home"
        self.started_at, self.ended_at = now, None
        self.stages = [
            _Stage("drive", f"Nav2 주행 {ret} → {self.target}"),
            _Stage("pick", "집기"),
            _Stage("return", f"복귀 {self.target} → {ret}"),
        ]

    def is_new_mission_since(self, t: float) -> bool:
        return self.started_at is not None and self.started_at > t

    def on_status(self, text: str) -> None:
        self.status = text

    def _active(self) -> _Stage | None:
        return next((s for s in self.stages if s.state == "active"), None)

    def on_state(self, state: str, now: float, stop_latched: bool = False) -> None:
        self.state = state
        if not self.stages or self.ended_at is not None:
            return
        drive, pick, ret = self.stages
        active = self._active()
        if state == "moving" and all(s.state == "todo" for s in self.stages):
            drive.begin(now)
        elif state == "picking" and pick.state == "todo":
            if drive.state == "active":
                drive.finish(now, True)
            pick.begin(now)
        elif state == "returning" and ret.state == "todo":
            if pick.state == "active":
                pick.finish(now, True)
            ret.begin(now)
        elif state in END_OK | END_FAIL:
            if active is not None:
                active.finish(now, state in END_OK)
            self.ended_at = now

    def snapshot(self, now: float) -> dict:
        elapsed = None
        if self.started_at is not None:
            elapsed = round((self.ended_at if self.ended_at is not None else now) - self.started_at, 1)
        return {
            "state": self.state, "status": self.status, "command": self.command,
            "target": self.target, "color": self.color, "elapsed": elapsed,
            "stages": [s.as_dict(now) for s in self.stages],
        }
