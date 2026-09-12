"""Place 재생 — 녹화 모션 프레임을 시간 기준으로 팔 목표·바퀴 속도로 바꾼다 (순수 로직).

`PickWorker` 가 Pick 완료 후 **같은 ZMQ 연결**로 이 클래스를 돌린다 (기획서 §6.3 A안).
`TeleopReplayWorker` 처럼 sleep 으로 프레임을 넘기지 않고, 워커 루프의 tick 마다
`update(now)` 를 불러 "지금 시각에 해당하는 프레임" 을 돌려준다 — 카메라 표시와
정지 응답성을 Pick 루프와 똑같이 유지하기 위해서다.

## 규칙 (기획서 §3.5·§6.4)

1. **시작 자세 정합**: 첫 프레임과 현재 자세의 관절별 최대 차이가 `mismatch_tol_deg`
   를 넘으면 `prelude_s` 동안 첫 프레임까지 선형 보간한 뒤 타임라인을 시작한다.
2. **그리퍼 유지**: 큐브를 문 채 출발하므로 그리퍼는 현재(닫힌) 값을 유지하다가,
   모션의 그리퍼가 `hold + gripper_open_margin` 을 넘어 **열리는 시점부터** 따라간다.
3. **바퀴**: 프레임의 `base`/`x.vel…` 를 그대로 돌려준다 (D4). 호출자가 dry-run·
   일시정지면 정지로 바꾼다.
"""
from __future__ import annotations

from services.motion_library import frame_base, frame_joints
from services.net import lekiwi_units

GRIPPER_KEY = "arm_gripper.pos"


def _arm_key(short: str) -> str:
    return f"arm_{short}.pos"


class PlacePlayer:
    def __init__(
        self,
        frames: list[dict],
        start_pose: dict[str, float],
        *,
        hold_gripper: bool = True,
        prelude_s: float = 2.0,
        mismatch_tol_deg: float = 8.0,
        gripper_open_margin: float = 5.0,
    ) -> None:
        self._frames = [f for f in frames if isinstance(f, dict)]
        self._start = dict(start_pose)
        self._hold_gripper = hold_gripper
        self._hold_value = float(start_pose.get(GRIPPER_KEY, 0.0))
        self._released = not hold_gripper
        self._margin = float(gripper_open_margin)
        self._prelude_s = float(prelude_s)
        self._t0: float | None = None
        self._timeline_t0: float | None = None
        self._last_now: float | None = None
        self.done = False
        self.label = "PLACE"
        self.index = 0
        self.last_pose: dict[str, float] = dict(start_pose)

        first = self._pose_from_frame(self._frames[0]) if self._frames else dict(start_pose)
        self._first_pose = first
        worst = 0.0
        for k, v in first.items():
            if k == GRIPPER_KEY or k not in self._start:
                continue
            worst = max(worst, abs(float(v) - float(self._start[k])))
        self.mismatch_deg = worst
        self._needs_prelude = worst > float(mismatch_tol_deg) and self._prelude_s > 0

    # ── 공개 ──
    @property
    def total(self) -> int:
        return len(self._frames)

    def update(self, now: float, *, advance: bool = True) -> tuple[dict[str, float], dict[str, float], bool]:
        """`(팔 목표 {arm_*.pos}, 바퀴 {x.vel…}, 끝났는가)`.

        `advance=False`(일시정지·드라이런)면 타임라인을 멈추고 마지막 자세 + 정지를 돌려준다 —
        멈춘 시간만큼 시작점을 뒤로 밀어 재개하면 그 프레임부터 이어간다.
        """
        if not self._frames:
            self.done = True
            return dict(self._start), dict(lekiwi_units.BASE_STOP), True
        if self._t0 is None:
            self._t0 = now
            self._last_now = now
        if not advance:
            delta = now - (self._last_now if self._last_now is not None else now)
            self._t0 += delta
            if self._timeline_t0 is not None:
                self._timeline_t0 += delta
            self._last_now = now
            return dict(self.last_pose), dict(lekiwi_units.BASE_STOP), False
        self._last_now = now
        if self._needs_prelude:
            a = min(1.0, (now - self._t0) / self._prelude_s)
            pose = {
                k: self._start.get(k, v) + (v - self._start.get(k, v)) * a
                for k, v in self._first_pose.items()
            }
            pose = {**self._start, **pose}
            self.label = "PLACE_PRELUDE"
            self.last_pose = self._apply_gripper_hold(pose)
            if a >= 1.0:
                self._needs_prelude = False
                self._timeline_t0 = now
            return self.last_pose, dict(lekiwi_units.BASE_STOP), False

        if self._timeline_t0 is None:
            self._timeline_t0 = now
        elapsed = now - self._timeline_t0
        idx = self.index
        while idx + 1 < len(self._frames) and self._frame_t(idx + 1) <= elapsed:
            idx += 1
        self.index = idx
        frame = self._frames[idx]
        pose = {**self._start, **self._pose_from_frame(frame)}
        self.last_pose = self._apply_gripper_hold(pose)
        base = frame_base(frame)
        self.label = "PLACE"
        finished = idx >= len(self._frames) - 1 and elapsed >= self._frame_t(idx)
        if finished:
            self.done = True
            self.label = "PLACED"
            return self.last_pose, dict(lekiwi_units.BASE_STOP), True
        return self.last_pose, base, False

    # ── 내부 ──
    def _frame_t(self, idx: int) -> float:
        try:
            return float(self._frames[idx].get("t", 0.0))
        except (TypeError, ValueError):
            return 0.0

    @staticmethod
    def _pose_from_frame(frame: dict) -> dict[str, float]:
        return {_arm_key(k): float(v) for k, v in frame_joints(frame).items()}

    def _apply_gripper_hold(self, pose: dict[str, float]) -> dict[str, float]:
        if not self._hold_gripper:
            return pose
        out = dict(pose)
        motion_gripper = pose.get(GRIPPER_KEY)
        if not self._released and motion_gripper is not None and motion_gripper > self._hold_value + self._margin:
            self._released = True
        if not self._released:
            out[GRIPPER_KEY] = self._hold_value
        return out


def interpolate_pose(src: dict[str, float], dst: dict[str, float], a: float) -> dict[str, float]:
    """관절 dict 선형 보간 (dst 에 없는 관절은 src 유지)."""
    a = max(0.0, min(1.0, a))
    return {k: v + (dst.get(k, v) - v) * a for k, v in src.items()}


__all__ = ["GRIPPER_KEY", "PlacePlayer", "interpolate_pose"]
