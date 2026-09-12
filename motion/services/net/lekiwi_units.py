"""LeKiwi 관측값 단위 변환 — **정규화값 → 도(degree)**.

## 왜 필요한가 (2026-08-06)

같은 텔레옵 화면인데 기종마다 관측 단위가 다르다:

| | `use_degrees` | 팔 5축 정규화 모드 | 관측값 |
|---|---|---|---|
| SO-101 (`SO101FollowerConfig`) | **True** (기본) | `DEGREES` | 도 |
| LeKiwi (`LeKiwiConfig`) | **False** (기본) | `RANGE_M100_100` | −100 ~ 100 |

`ui/teleop_pane._on_motor_positions` 는 값을 **도로 그대로** `Robot3DView` 에 넘긴다.
LeKiwi 값을 변환 없이 흘리면 3D 로봇 자세가 **조용히 틀리게** 그려진다 (예외도 경고도 없다).

**Pi 의 설정은 바꾸지 않는다.** `use_degrees=True` 로 돌리면 리더암(정규화값)과 단위가
어긋나 텔레옵과 기존 데이터셋이 통째로 깨진다 (.agent/LEKIWI.md §8).

## ⚠️ 그리퍼는 변환하지 않는다

`SO101Follower` 와 `LeKiwi` **둘 다** 그리퍼만은 `MotorNormMode.RANGE_0_100` 을 쓴다
(`use_degrees` 와 무관하게 하드코딩). 즉 그리퍼는 이미 두 기종이 같은 단위라
변환하면 오히려 어긋난다. 실측 확인:
- so_follower.py:59 `"gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100)`
- lekiwi.py:65    `"arm_gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100)`

## 식

lerobot `MotorsBus._normalize` 를 뒤집은 것이다.

    RANGE_M100_100:  norm = ((raw - min) / (max - min)) * 200 - 100
    DEGREES:         deg  = (raw - mid) * 360 / max_res,  mid = (min + max) / 2

두 식에서 raw 를 소거하면 **보정 범위만으로** 곧바로 변환된다:

    raw - mid = (max - min) * norm / 200
    deg       = norm * (max - min) / 200 * 360 / max_res

`drive_mode` 가 켜져 있으면 lerobot 이 부호를 뒤집으므로 여기서도 뒤집는다.
"""
from __future__ import annotations

import json
from dataclasses import dataclass

#: sts3215 해상도. lerobot `model_resolution_table` 과 같은 값.
TICKS_PER_TURN = 4096
_MAX_RES = TICKS_PER_TURN - 1

#: 두 기종이 이미 같은 단위(0~100)를 쓰는 관절 — **변환 금지**.
PASSTHROUGH_JOINTS = frozenset({"gripper"})

#: LeKiwi 모터 이름의 팔 접두사. 떼면 SO-101 이름과 같아진다.
_ARM_PREFIX = "arm_"

#: `send_action` 에 **항상** 실어야 하는 바퀴 속도. ⚠️ 빼면 `LeKiwi.send_action` 이
#: `base_goal_vel["x.vel"]` 에서 KeyError 를 내는데 `lekiwi_host` 가 그걸 삼켜
#: **팔이 조용히 안 움직인다** (`build_arm_action` 주석 참조).
BASE_STOP: dict[str, float] = {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}
#: 바퀴 속도 키 — `base` 인자로 받을 수 있는 키의 전부. (x/y: m/s, theta: deg/s)
BASE_KEYS: tuple[str, ...] = ("x.vel", "y.vel", "theta.vel")


@dataclass(frozen=True)
class MotorCal:
    """한 모터의 보정 범위 (raw tick)."""

    range_min: int
    range_max: int
    drive_mode: int = 0

    @property
    def span(self) -> int:
        return self.range_max - self.range_min

    @property
    def usable(self) -> bool:
        """범위가 0 이면 변환식이 성립하지 않는다 (미보정·손상된 파일)."""
        return self.span != 0


def strip_arm_prefix(name: str) -> str:
    """`arm_shoulder_pan` → `shoulder_pan`. 접두사가 없으면 그대로."""
    text = name or ""
    return text[len(_ARM_PREFIX):] if text.startswith(_ARM_PREFIX) else text


def parse_calibration(text: str) -> dict[str, MotorCal]:
    """Pi 의 보정 JSON → `{관절명(접두사 없음): MotorCal}`.

    바퀴(`base_*`)는 버린다 — 관측의 팔 관절만 변환 대상이다.
    형식이 깨졌으면 빈 dict 를 돌려준다 (호출자가 변환을 건너뛰도록).
    """
    try:
        payload = json.loads(text or "")
    except (TypeError, ValueError):
        return {}
    if not isinstance(payload, dict):
        return {}

    out: dict[str, MotorCal] = {}
    for raw_name, entry in payload.items():
        if not isinstance(raw_name, str) or not isinstance(entry, dict):
            continue
        if raw_name.startswith("base_"):
            continue
        try:
            cal = MotorCal(
                range_min=int(entry["range_min"]),
                range_max=int(entry["range_max"]),
                drive_mode=int(entry.get("drive_mode", 0)),
            )
        except (KeyError, TypeError, ValueError):
            continue
        if cal.usable:
            out[strip_arm_prefix(raw_name)] = cal
    return out


def build_arm_action(action: dict, base: dict | None = None) -> dict[str, float]:
    """관절 dict → `LeKiwi.send_action` 페이로드. **송신 페이로드는 여기서만 만든다.**

    입력은 `{"shoulder_pan": 12.0}` / `{"shoulder_pan.pos": 12.0}` /
    `{"arm_shoulder_pan.pos": 12.0}` 중 무엇이어도 된다 — 텔레옵은 리더에서 읽은
    dict 을, Motion Replay 는 녹화 JSON 의 action 을 그대로 넘긴다.

    ⚠️⚠️ **조용한 실패 두 가지가 여기 모여 있다.** 실물에서 만나면 원인 찾기가 매우 어렵다:

    1. **`.pos` 접미사** — `LeKiwi.send_action` 은 `k.endswith(".pos")` 로 거른다.
       접미사가 없으면 하나도 안 걸려 `sync_write("Goal_Position", {})` 가 되고,
       예외도 경고도 없이 **팔이 그냥 안 움직인다.**
    2. **바퀴 속도 키** — 같은 함수가 `base_goal_vel["x.vel"]` 를 **무조건 인덱싱**한다.
       빠지면 KeyError 인데 `lekiwi_host` 가 그 예외를 삼키고 로그만 남긴다 →
       역시 **팔이 조용히 안 움직인다.** (lekiwi.py:388 / lekiwi_host.py:83 실측)

    바퀴 속도는 **`base` 인자로만** 들어간다 (기본 = 정지). `action` 안의 `x.vel` 같은
    키는 무시한다 — 녹화 파일·데이터셋 액션에 바퀴 속도가 섞여 있어도 재생 때 굴러가면
    안 된다. 키보드 주행(`lekiwi_base_drive`)은 텔레옵 워커가 `base` 로 넘긴다.

    Args:
        action: 팔 관절 dict (위 세 모양 중 하나).
        base: `{"x.vel", "y.vel", "theta.vel"}` (일부만 줘도 나머지는 0). None = 정지.
            숫자가 아니거나 모르는 키는 버린다.

    Returns:
        `{"arm_<관절>.pos": 값, "x.vel": …, "y.vel": …, "theta.vel": …}`.
        팔 관절이 하나도 없으면 **빈 dict** — 호출자가 송신 자체를 건너뛰도록.
        ⚠️ 바퀴만 보내는 건 "의미가 없는" 정도가 아니라 **실패한다**: 호스트의
        `sync_write("Goal_Position", {})` 가 `next(iter([]))` 로 예외를 내고 호스트가
        삼킨다 (motors_bus.py:1245, 2026-09-03 확인). 바퀴만 굴리고 싶으면 현재 관측
        자세를 팔 관절로 같이 실어라 (`LekiwiTeleopWorker._pump_base_only`).
    """
    joints: dict[str, float] = {}
    for raw, value in (action or {}).items():
        if not isinstance(raw, str):
            continue
        name = raw[: -len(".pos")] if raw.endswith(".pos") else raw
        if "." in name:
            continue                      # `x.vel` 등 바퀴 속도 — BASE_STOP 이 대신한다
        name = strip_arm_prefix(name)
        if not name:
            continue
        try:
            joints[f"{_ARM_PREFIX}{name}.pos"] = float(value)
        except (TypeError, ValueError):
            continue
    if not joints:
        return {}
    joints.update(build_base_velocity(base))
    return joints


def build_base_velocity(base: dict | None) -> dict[str, float]:
    """바퀴 속도 dict 를 **세 키가 전부 있는** 정상 모양으로 만든다.

    None·빈 dict → `BASE_STOP` 사본. 모르는 키·숫자 아님·NaN/inf 는 버리고 0 으로 둔다
    (호스트가 `_body_to_wheel_raw` 에 그대로 넣으므로 쓰레기를 흘리면 안 된다).
    """
    out = dict(BASE_STOP)
    for key in BASE_KEYS:
        try:
            value = float((base or {}).get(key, 0.0))
        except (TypeError, ValueError):
            continue
        if value != value or value in (float("inf"), float("-inf")):
            continue
        out[key] = value
    return out


def normalized_to_degrees(value: float, cal: MotorCal) -> float:
    """`RANGE_M100_100` 정규화값 → 도."""
    deg = float(value) * cal.span / 200.0 * 360.0 / _MAX_RES
    return -deg if cal.drive_mode else deg


def convert_observation(
    positions: dict[str, float], calibration: dict[str, MotorCal]
) -> dict[str, float]:
    """관측 dict 전체를 SO-101 과 **같은 단위**로 맞춘다.

    Args:
        positions: `{관절명: 값}` — 이름은 이미 `arm_` 접두사가 없어야 한다.
        calibration: `parse_calibration()` 결과. 비어 있으면 **원본을 그대로** 돌려준다
            (보정을 못 읽었을 때 엉뚱한 값을 만들어 내는 것보다 낫다 — 호출자가
            사용자에게 경고한다).

    Returns:
        새 dict. 그리퍼는 손대지 않고, 보정에 없는 관절도 그대로 통과시킨다.
    """
    if not calibration:
        return dict(positions)
    out: dict[str, float] = {}
    for name, value in (positions or {}).items():
        cal = calibration.get(name)
        if cal is None or name in PASSTHROUGH_JOINTS:
            out[name] = value
            continue
        try:
            out[name] = normalized_to_degrees(value, cal)
        except (TypeError, ValueError):
            out[name] = value
    return out
