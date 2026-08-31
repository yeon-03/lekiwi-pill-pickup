#!/usr/bin/env python
"""고정캠 시차(parallax) 보정 — 화면 가장자리 물체를 못 집던 문제의 해결.

■ 왜 필요한가 (2026-08-26 규명)
약병들은 바닥에 서 있는 게 아니라 **수직 보드에 걸려 있고**, 고정 USB캠은
그 보드를 정면에서 본다. 그리퍼(와 기준점인 흰 스티커)는 보드보다 카메라
쪽으로 d 만큼 튀어나와 있다. 즉 마커와 병은 **깊이가 다른 평면**에 있다.

핀홀 투영에서 깊이 (D-d) 인 점은 배율 f/(D-d), 깊이 D 인 점은 f/D 로
찍히므로, 그리퍼가 병 정면에 실제로 오려면 마커의 화면좌표는 병보다
화면중심에서 **더 바깥쪽**이어야 한다:

    marker_target - c = k * (bottle - c),   k = D / (D - d)  > 1

    D = 카메라 ~ 보드 거리, d = 보드에서 그리퍼가 튀어나온 거리

이 보정량은 화면중심에서 정확히 0이고 가장자리로 갈수록 커진다 — 실제
관측(가운데 병 10/10 성공, 좌/우 병 실패)과 정확히 일치한다. 보정을 안 하면
로봇이 목표보다 **덜 가서** 멈춘다("물체까지 가지 않고 대각선에 위치").

■ 구현
축별 1차식 `target = a * bottle + b` 로 저장한다(a=k, b=c*(1-k)와 동치).
이렇게 두면 배율뿐 아니라 마커~집게끝 사이의 고정 오프셋까지 같은 식이
흡수하므로, 실측 샘플만 있으면 물리량을 몰라도 맞출 수 있다.

**기본값은 항등(a=1, b=0)** — 설정 파일이 없으면 보정을 아예 안 한다.
즉 캘리브레이션 전까지는 지금까지 검증된 동작(가운데 병 10/10)이 그대로다.
"""
import json
from pathlib import Path

CONFIG_PATH = Path(__file__).resolve().parent.parent / 'config' / 'parallax.json'
SAMPLES_PATH = Path(__file__).resolve().parent.parent / 'config' / 'parallax_samples.json'

IDENTITY = {'ax': 1.0, 'bx': 0.0, 'ay': 1.0, 'by': 0.0}

# 축별 샘플이 이 폭 미만으로 몰려 있으면 기울기(=k)를 못 믿는다.
# 두 점이 거의 같은 자리면 회귀 기울기가 잡음에 폭주하기 때문 —
# 그 축은 조용히 항등으로 남긴다(잘못 보정하느니 안 하는 게 낫다).
MIN_SPREAD_PX = 60.0
# 물리적으로 말이 되는 배율 범위. 벗어나면 측정이 잘못된 것으로 본다.
K_MIN, K_MAX = 0.7, 3.0


def load(path: Path | None = None) -> dict:
    """저장된 보정계수를 읽는다. 파일이 없거나 깨졌으면 항등을 돌려준다."""
    path = path or CONFIG_PATH
    try:
        raw = json.loads(path.read_text())
    except (OSError, ValueError):
        return dict(IDENTITY)
    params = dict(IDENTITY)
    for key in IDENTITY:
        value = raw.get(key)
        if isinstance(value, (int, float)):
            params[key] = float(value)
    return params


def save(params: dict, path: Path | None = None) -> Path:
    path = path or CONFIG_PATH
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(params, indent=2, ensure_ascii=False) + '\n')
    return path


def is_identity(params: dict) -> bool:
    return (abs(params['ax'] - 1.0) < 1e-9 and abs(params['bx']) < 1e-9
            and abs(params['ay'] - 1.0) < 1e-9 and abs(params['by']) < 1e-9)


def correct(bottle_px, params: dict) -> tuple[float, float]:
    """병의 화면좌표 -> 마커가 실제로 가야 할 화면좌표."""
    bx, by = bottle_px
    return (params['ax'] * bx + params['bx'], params['ay'] * by + params['by'])


def describe(params: dict) -> str:
    if is_identity(params):
        return '시차보정 없음(항등) — config/parallax.json 미설정'
    parts = []
    for axis, a, b in (('x', params['ax'], params['bx']), ('y', params['ay'], params['by'])):
        if abs(a - 1.0) < 1e-9 and abs(b) < 1e-9:
            parts.append(f'{axis}: 보정없음')
        else:
            center = b / (1.0 - a) if abs(1.0 - a) > 1e-6 else float('nan')
            parts.append(f'{axis}: k={a:.3f} 기준점={center:.0f}px')
    return '시차보정 ' + ', '.join(parts)


def from_distance(camera_to_board: float, gripper_offset: float,
                  anchor_x: float, anchor_y: float) -> dict:
    """자로 잰 두 거리로 계수를 만든다(단위 무관, 둘만 같으면 됨).

    camera_to_board: 카메라 렌즈 ~ 보드 표면 거리 (D)
    gripper_offset:  보드 표면 ~ 흰 스티커 거리, 파지 자세에서 (d)
    anchor_x/y: **보정을 0으로 둘 기준점**. 화면중심이 아니라
        "지금 보정 없이도 잘 잡히는 병"의 좌표를 넣어야 한다.

    ⚠️ 기준점이 왜 화면중심이 아닌가:
    이론상 배율의 중심은 주점(≈화면중심)이지만, 목표좌표에는 마커~집게끝
    사이의 고정 오프셋이 이미 경험적으로 녹아 있다(특히 y는 병 실루엣의
    아래끝 y2를 쓰고 있어 화면중심과 62px 떨어져 있음). 화면중심을 기준으로
    잡으면 그 오프셋을 0으로 가정하는 셈이라, 이미 10/10으로 검증된 가운데
    병의 목표까지 십여 px 밀어버린다(2026-08-26 실측 확인).

    `c + k(b-c) + o` (o = 미지의 고정 오프셋)는 `a + k(b-a)`와 완전히 같은
    식이므로(a = c - o/(k-1)), 잘 되는 점 하나를 기준점으로 쓰면 o를 몰라도
    정확히 흡수된다. 부수효과로 그 위치의 동작은 **정의상 하나도 안 바뀐다**.
    """
    if camera_to_board <= gripper_offset:
        raise ValueError('카메라~보드 거리가 그리퍼 돌출량보다 커야 합니다')
    k = camera_to_board / (camera_to_board - gripper_offset)
    if not (K_MIN <= k <= K_MAX):
        raise ValueError(f'계산된 배율 k={k:.2f}가 상식 범위({K_MIN}~{K_MAX})를 벗어남 '
                         '— 두 거리를 다시 재보세요')
    return {'ax': k, 'bx': anchor_x * (1.0 - k),
            'ay': k, 'by': anchor_y * (1.0 - k)}


def _fit_axis(bottle_vals, marker_vals, axis_name):
    """marker = a*bottle + b 최소제곱. (params, 설명문) 반환.

    못 믿을 상황이면 (None, 이유) 를 돌려줘서 그 축은 항등으로 남긴다.
    """
    n = len(bottle_vals)
    if n < 2:
        return None, f'{axis_name}축: 샘플 {n}개(2개 이상 필요) — 보정 안 함'
    spread = max(bottle_vals) - min(bottle_vals)
    if spread < MIN_SPREAD_PX:
        return None, (f'{axis_name}축: 샘플들이 {spread:.0f}px 안에 몰려 있음'
                      f'(>{MIN_SPREAD_PX:.0f}px 필요) — 보정 안 함')

    mean_b = sum(bottle_vals) / n
    mean_m = sum(marker_vals) / n
    num = sum((b - mean_b) * (m - mean_m) for b, m in zip(bottle_vals, marker_vals))
    den = sum((b - mean_b) ** 2 for b in bottle_vals)
    a = num / den
    b = mean_m - a * mean_b
    if not (K_MIN <= a <= K_MAX):
        return None, (f'{axis_name}축: 기울기 k={a:.2f}가 상식 범위'
                      f'({K_MIN}~{K_MAX})를 벗어남 — 보정 안 함(샘플 확인 필요)')
    resid = [m - (a * bv + b) for bv, m in zip(bottle_vals, marker_vals)]
    worst = max(abs(r) for r in resid)
    return (a, b), (f'{axis_name}축: k={a:.3f} b={b:+.1f} '
                    f'(샘플 {n}개, 폭 {spread:.0f}px, 최대잔차 {worst:.1f}px)')


def fit(samples: list[dict]) -> tuple[dict, list[str]]:
    """record 로 모은 샘플에서 계수를 맞춘다.

    samples: [{'bottle': [x, y], 'marker': [x, y]}, ...]
             marker 는 "집게가 실제로 병에 제대로 맞았을 때"의 마커 좌표여야 한다.
    """
    params = dict(IDENTITY)
    notes = []
    for axis, idx, akey, bkey in (('x', 0, 'ax', 'bx'), ('y', 1, 'ay', 'by')):
        bottle_vals = [float(s['bottle'][idx]) for s in samples]
        marker_vals = [float(s['marker'][idx]) for s in samples]
        fitted, note = _fit_axis(bottle_vals, marker_vals, axis)
        notes.append(note)
        if fitted is not None:
            params[akey], params[bkey] = fitted
    return params, notes
