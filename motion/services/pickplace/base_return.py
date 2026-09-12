"""바퀴 경로 기록·역재생 — Pick 하러 간 길을 되짚어 출발 위치로 돌아온다 (순수 로직).

## 왜 (2026-09-08 사용자 요청)

Pick&Place 실행은 "접근(바퀴) → Pick(팔) → **되돌아오기**(바퀴) → Place(팔 모션 재생)" 순이다.
LeKiwi 에는 위치 추정(odometry)이 앱에 노출되지 않으므로, 접근 중 **보낸 바퀴 속도 명령**을
시간과 함께 기록해 두었다가 **역순으로 부호를 뒤집어** 같은 시간만큼 재생한다. 개루프라
바닥 마찰·가속 지연만큼 오차가 남지만, 출발 지점 근처로 돌아오는 데는 충분하다
(Place 모션은 팔만 움직이므로 수 cm 오차는 문제가 안 된다).

## 기록 규칙

- `BaseMotionLog.append(now, base)` 를 접근 루프의 **매 tick** 부른다. 실제로 보낸 값
  (`sent`)만 넣는다 — 일시정지·드라이런·팔 동작 중의 정지 명령은 0 이라 자연히 길이 0 구간.
- 연속된 같은 명령은 한 구간으로 합친다. 0 속도 구간은 되돌아올 때 **건너뛴다**(멈춰 있던
  시간을 되풀이할 이유가 없다).
- Pick 재시도로 접근을 다시 하면 이어서 기록된다 — 전체를 뒤집으면 여전히 출발점이다.

## 역재생

`reversed_segments()` = 구간을 역순으로, `x.vel/y.vel/theta.vel` 부호를 뒤집어서.
`BaseReturnPlayer.update(now)` 가 tick 마다 "지금 보낼 바퀴 속도" 와 끝났는지를 돌려준다.
"""
from __future__ import annotations

from dataclasses import dataclass

from services.net import lekiwi_units

_KEYS = lekiwi_units.BASE_KEYS
#: 이 이하는 정지로 본다 (부동소수 잡음).
_EPS = 1e-6


def _is_zero(base: dict) -> bool:
    return all(abs(float(base.get(k, 0.0))) < _EPS for k in _KEYS)


def _same(a: dict, b: dict) -> bool:
    return all(abs(float(a.get(k, 0.0)) - float(b.get(k, 0.0))) < _EPS for k in _KEYS)


@dataclass(frozen=True)
class BaseSegment:
    duration_s: float
    base: dict[str, float]


class BaseMotionLog:
    """보낸 바퀴 명령의 시간열. `(t, base)` 를 tick 마다 넣는다."""

    def __init__(self) -> None:
        self._entries: list[tuple[float, dict[str, float]]] = []
        self._last_t: float | None = None

    def clear(self) -> None:
        self._entries.clear()
        self._last_t = None

    def append(self, now: float, base: dict | None) -> None:
        clean = lekiwi_units.build_base_velocity(base)
        self._entries.append((float(now), clean))
        self._last_t = float(now)

    @property
    def moved(self) -> bool:
        return any(not _is_zero(b) for _t, b in self._entries)

    def segments(self) -> list[BaseSegment]:
        """연속 동일 명령을 합친 구간 목록 (0 속도 구간 포함, 시간순)."""
        out: list[BaseSegment] = []
        n = len(self._entries)
        if n == 0:
            return out
        # 마지막 항목의 길이는 직전 간격으로 추정한다 (tick 주기).
        last_dt = (self._entries[-1][0] - self._entries[-2][0]) if n > 1 else 0.0
        i = 0
        while i < n:
            t0, base = self._entries[i]
            j = i + 1
            while j < n and _same(self._entries[j][1], base):
                j += 1
            t_end = self._entries[j][0] if j < n else (self._entries[-1][0] + last_dt)
            out.append(BaseSegment(max(0.0, t_end - t0), dict(base)))
            i = j
        return out

    def reversed_segments(self) -> list[BaseSegment]:
        """되돌아오기용: 역순 + 부호 반전, 0 속도·길이 0 구간 제외."""
        out: list[BaseSegment] = []
        for seg in reversed(self.segments()):
            if seg.duration_s <= 0.0 or _is_zero(seg.base):
                continue
            out.append(BaseSegment(seg.duration_s, {k: -float(seg.base.get(k, 0.0)) for k in _KEYS}))
        return out

    @property
    def total_return_s(self) -> float:
        return sum(s.duration_s for s in self.reversed_segments())


class BaseReturnPlayer:
    """역재생 구간을 시간 기준으로 돌려준다. 일시정지는 호출자가 `now` 를 멈춰서(진행 안 함) 처리한다."""

    def __init__(self, segments: list[BaseSegment]) -> None:
        self._segments = list(segments)
        self._t0: float | None = None
        self._paused_offset = 0.0
        self.done = not self._segments
        self.index = 0

    @property
    def total(self) -> int:
        return len(self._segments)

    @property
    def total_s(self) -> float:
        return sum(s.duration_s for s in self._segments)

    def update(self, now: float, *, advance: bool = True) -> tuple[dict[str, float], bool]:
        """`(바퀴 속도, 끝났는가)`. `advance=False`(일시정지)면 진행하지 않고 정지를 돌려준다."""
        if self.done:
            return dict(lekiwi_units.BASE_STOP), True
        if self._t0 is None:
            self._t0 = now
            self._last_now = now
        if not advance:
            # 멈춘 시간만큼 시작점을 뒤로 민다 — 재개하면 그 구간부터 이어간다.
            self._t0 += now - self._last_now
            self._last_now = now
            return dict(lekiwi_units.BASE_STOP), False
        self._last_now = now
        elapsed = now - self._t0
        acc = 0.0
        for idx, seg in enumerate(self._segments):
            if elapsed < acc + seg.duration_s:
                self.index = idx
                return dict(seg.base), False
            acc += seg.duration_s
        self.done = True
        self.index = len(self._segments)
        return dict(lekiwi_units.BASE_STOP), True


__all__ = ["BaseMotionLog", "BaseReturnPlayer", "BaseSegment"]
