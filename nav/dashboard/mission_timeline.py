"""브리지 /abo/state 전이로 미션 단계(주행 → 집기 → 복귀)를 만든다. ROS 없음."""
from __future__ import annotations

from mission_words import effective_state, headline

END_OK = {"done"}
END_FAIL = {"failed", "rejected", "canceled"}


class _Stage:
    def __init__(self, key: str, label: str):
        self.key, self.label = key, label
        self.state = "todo"
        self.start: float | None = None
        self.end: float | None = None
        self.note = ""                       # 이 단계가 진행 중일 때 마지막으로 받은 /abo/status

    def begin(self, now: float) -> None:
        self.state, self.start = "active", now

    def finish(self, now: float, ok: bool) -> None:
        self.state, self.end = ("done" if ok else "failed"), now

    def as_dict(self, now: float) -> dict:
        sec = None if self.start is None else round((self.end if self.end is not None else now) - self.start, 1)
        return {"key": self.key, "label": self.label, "state": self.state, "sec": sec,
                "start_at": self.start, "note": self.note}


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
        self.picked: bool | None = None
        self.outcome: str | None = None
        self.shown_state: str | None = None   # 문장용 상태 (집기 뒤 moving → returning)

    def on_command(self, text: str, now: float) -> None:
        self.command = text.strip()
        tok = self.command.split()
        if not tok or tok[0].lower() != "fetch" or len(tok) < 2:
            return
        if self.active():
            return        # 브리지는 미션 중 같은 fetch 는 무시, 다른 fetch 는 rejected — 진행 중인 미션을 지우지 않는다
        self.target = tok[1]
        self.color = next((t.split(":", 1)[1] for t in tok[2:] if t.lower().startswith("color:")), None)
        ret = tok[tok.index("to") + 1] if "to" in tok[2:-1] else "home"
        self.started_at, self.ended_at = now, None
        self.picked, self.outcome = None, None
        self.shown_state = None                    # 새 미션 문장은 "전달했어요" 부터
        self.stages = [
            _Stage("drive", f"Nav2 주행 {ret} → {self.target}"),
            _Stage("pick", "집기"),
            _Stage("return", f"복귀 {self.target} → {ret}"),
        ]

    def active(self) -> bool:
        return self.started_at is not None and self.ended_at is None

    def is_new_mission_since(self, t: float) -> bool:
        return self.started_at is not None and self.started_at > t

    def on_status(self, text: str, now: float | None = None) -> None:
        self.status = text
        active = self._active()
        if active is not None:
            active.note = text

    def _active(self) -> _Stage | None:
        return next((s for s in self.stages if s.state == "active"), None)

    def on_pick_done(self, ok: bool, now: float) -> None:
        if not self.stages or self.ended_at is not None:
            return
        pick = self.stages[1]
        if pick.state == "active":
            pick.finish(now, ok)
            self.picked = ok

    def on_state(self, state: str, now: float, stop_latched: bool = False) -> None:
        self.state = state
        mid_reject = state == "rejected" and any(s.state != "todo" for s in self.stages)
        if self.stages and self.ended_at is None and not mid_reject:
            self.shown_state = effective_state(self.shown_state, state)
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
            if pick.state == "active":           # /abo/pick_done 을 못 받았을 때만 성공으로 본다
                pick.finish(now, True)
                self.picked = True
            ret.begin(now)
        elif state == "rejected" and any(s.state != "todo" for s in self.stages):
            return                               # 진행 중 미션에 온 거절은 새 fetch 에 대한 답 — 미션은 계속된다
        elif state in END_OK | END_FAIL:
            ok = state in END_OK
            if active is not None:
                # 집기 실패 후 복귀가 끝나면 브리지는 failed 로 끝낸다 — 복귀 자체는 해냈다.
                came_back = state == "failed" and active is ret and self.picked is False
                active.finish(now, ok or came_back)
            self.outcome = "done" if ok else "failed"
            self.ended_at = now

    def snapshot(self, now: float) -> dict:
        elapsed = None
        if self.started_at is not None:
            elapsed = round((self.ended_at if self.ended_at is not None else now) - self.started_at, 1)
        return {
            "state": self.state, "status": self.status, "command": self.command,
            "target": self.target, "color": self.color, "elapsed": elapsed,
            "stages": [s.as_dict(now) for s in self.stages],
            "outcome": self.outcome,
            "received_at": self.started_at,
            "headline": self._headline(),
        }

    def _headline(self) -> str:
        if not self.stages:
            return headline(None, None)
        return headline(self.shown_state or "sent", self.color)
