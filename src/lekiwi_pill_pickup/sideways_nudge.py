"""move_marker_to_bottle 뒤에 붙는 오픈루프 옆걸음 넛지의 계획 로직.

■ 왜 필요한가 (2026-08-27 스파이크로 확인)
고정캠 시각서보(`approach_board.move_marker_to_bottle`)는 가운데 슬롯 병에서는
잘 맞지만, 바깥 슬롯 병에서는 두 가지 이유로 목표에 못 미친 채 끝난다:
  1. 파랄랙스 언더슈트 — 마커(손목)와 병은 깊이가 다른 평면이라, 화면
     가장자리로 갈수록 "마커를 병 픽셀에 맞춤" != "그리퍼가 병 위"가 커진다.
  2. 마커가 화면 끝까지 가면 검출이 안 돼서 서보가 그 전에 멈춘다.
그 상태로 하강하면 그리퍼가 병 옆구리를 쳐서 shoulder_lift가 65~67에서
스톨한다(팔 한계가 아니라 충돌). 서보가 끝난 뒤 목표 슬롯 쪽으로 정해진
횟수만큼 오픈루프 펄스를 더 주면 그리퍼가 병을 안 치고 끝까지 내려가
그립에 성공한다(실측: shoulder_lift 61.2~61.4).

■ 키는 색이 아니라 슬롯('left'/'right')
이 셋업은 걸이 위치가 고정이고 어느 슬롯에 어느 색이 걸리는지는 바뀔 수
있다. 그래서 넛지량은 색이 아니라 슬롯에 묶는다. 슬롯 판정은 목표 병의
화면 x가 기준점(가운데 정렬 마커 위치)보다 왼쪽인지 오른쪽인지로 한다 —
서보가 "도달"로 끝났는지 "놓침"으로 끝났는지와 무관하게 안정적이다.

■ 값은 이 보드 위치 전용
config/sideways_nudge.json 의 left/right 펄스 수는 지금 보드/걸이 배치에서
실측한 값이다. 보드를 옮기거나 걸이 간격이 바뀌면 다시 재야 한다. 파일이
없으면 기본값 0 = 오픈루프 안 함 = 이 기능 도입 전 동작 그대로.
"""
import json
from pathlib import Path

CONFIG_PATH = (Path(__file__).resolve().parent.parent.parent
               / 'config' / 'sideways_nudge.json')

DEFAULTS = {'left': 0, 'right': 0}

# 안전 상한 — config가 stale/오설정이어도 이 이상은 안 민다.
MAX_PULSES = 14

# 목표 병이 기준점에서 이만큼도 안 떨어져 있으면 = 가운데 슬롯 병 = 넛지 안 함.
MIN_CENTER_OFFSET_PX = 40

# 스파이크(2026-08-27)가 검증한 펄스 파라미터. base y 속도(m/s), 각 펄스/간격(초).
# ⚠️ 이 값들을 바꾸면 config의 펄스 수가 뜻하는 실제 변위가 달라진다 — 같이 재보정.
NUDGE_VEL = 0.03
NUDGE_PULSE_SEC = 0.3
NUDGE_GAP_SEC = 0.3


def _clamp_pulses(value) -> int:
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return 0
    return max(0, min(MAX_PULSES, int(round(value))))


def load(path=None) -> dict:
    """저장된 슬롯별 펄스 수를 읽는다. 파일이 없거나 깨졌으면 기본값(0)."""
    path = Path(path) if path is not None else CONFIG_PATH
    try:
        raw = json.loads(path.read_text())
    except (OSError, ValueError):
        return dict(DEFAULTS)
    if not isinstance(raw, dict):
        return dict(DEFAULTS)
    return {slot: _clamp_pulses(raw.get(slot, 0)) for slot in DEFAULTS}


def save(cfg: dict, path=None) -> Path:
    path = Path(path) if path is not None else CONFIG_PATH
    path.parent.mkdir(parents=True, exist_ok=True)
    out = {slot: _clamp_pulses(cfg.get(slot, 0)) for slot in DEFAULTS}
    path.write_text(json.dumps(out, indent=2, ensure_ascii=False) + '\n')
    return path


def open_loop_plan(servo_measured_sign, target_x, center_x, cfg) -> tuple[int, float]:
    """서보가 끝난 뒤 얼마나/어느 방향으로 오픈루프 펄스를 줄지 정한다.

    servo_measured_sign: +vy가 마커 화면 x를 늘리면 +1, 줄이면 -1.
        서보가 방향을 실측하지 못하고 끝났으면 None — 그러면 넛지 안 함
        (방향 모르는 채로 블라인드 주행하지 않는다).
    target_x:  목표 병의 고정캠 화면 x (팔 들기 전에 확정한 값).
    center_x:  기준점 x (가운데 슬롯 병에 정렬됐을 때 마커 x, 없으면 화면중심).
    cfg:       {'left': int, 'right': int} — load()가 준 슬롯별 펄스 수.

    반환: (펄스 수, vy 부호). 펄스 수 0이면 오픈루프 생략.
    """
    if servo_measured_sign is None:
        return (0, 0.0)
    offset = target_x - center_x
    if abs(offset) < MIN_CENTER_OFFSET_PX:
        return (0, 0.0)
    slot = 'left' if offset < 0 else 'right'
    n_pulses = _clamp_pulses(cfg.get(slot, 0))
    if n_pulses == 0:
        return (0, 0.0)
    direction = 1.0 if offset > 0 else -1.0     # 목표가 오른쪽이면 마커 x를 늘려야
    vy_sign = servo_measured_sign * direction
    return (n_pulses, vy_sign)
