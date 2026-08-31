"""move_marker_to_bottle 뒤에 붙는 오픈루프 옆걸음 넛지의 순수 계획 로직.

배경: 고정캠 시각서보는 바깥 슬롯 병에서 파랄랙스로 언더슈트하고, 마커를
화면 끝까지 추적하지 못한다. 그 상태로 하강하면 그리퍼가 병 옆구리를 쳐서
shoulder_lift가 65~67에서 스톨한다(스파이크 2026-08-27로 확인). 서보가 끝난
뒤 목표 슬롯 쪽으로 정해진 횟수만큼 오픈루프 펄스를 더 주면 그리퍼가 병을
안 치고 끝까지 내려간다(61.2~61.4, 그립 성공).

키는 색이 아니라 슬롯('left'/'right') — 위치는 고정이고 색만 바뀔 수 있는
셋업이기 때문.
"""
import json

import pytest

from lekiwi_pill_pickup.sideways_nudge import (
    DEFAULTS,
    MAX_PULSES,
    load,
    open_loop_plan,
    save,
)

CFG = {"left": 7, "right": 7}


def test_no_measured_sign_means_no_nudge():
    # 서보가 방향을 실측하지 못하고 끝났으면(sign=None) 블라인드 주행 금지.
    assert open_loop_plan(None, target_x=30, center_x=320, cfg=CFG) == (0, 0.0)


def test_target_near_center_means_no_nudge():
    # 목표가 중앙 근처(=가운데 슬롯 병)면 오픈루프 안 함.
    assert open_loop_plan(1.0, target_x=315, center_x=320, cfg=CFG) == (0, 0.0)


def test_left_slot_positive_camera_sign_drives_vy_negative():
    # 왼쪽 슬롯: 마커 x를 줄여야 함. +vy가 마커 x를 늘리면(sign=+1) vy는 음수.
    assert open_loop_plan(1.0, target_x=30, center_x=320, cfg=CFG) == (7, -1.0)


def test_left_slot_negative_camera_sign_drives_vy_positive():
    # 같은 왼쪽 슬롯인데 카메라 부호가 반대(sign=-1)면 vy 방향도 뒤집힌다.
    assert open_loop_plan(-1.0, target_x=30, center_x=320, cfg=CFG) == (7, 1.0)


def test_right_slot_uses_right_pulse_count_and_opposite_direction():
    cfg = {"left": 7, "right": 5}
    assert open_loop_plan(1.0, target_x=600, center_x=320, cfg=cfg) == (5, 1.0)


def test_pulse_count_is_clamped_to_max():
    cfg = {"left": 99, "right": 7}
    assert open_loop_plan(1.0, target_x=30, center_x=320, cfg=cfg) == (MAX_PULSES, -1.0)


def test_zero_configured_pulses_means_no_nudge():
    cfg = {"left": 0, "right": 7}
    assert open_loop_plan(1.0, target_x=30, center_x=320, cfg=cfg) == (0, 0.0)


def test_load_missing_file_returns_defaults():
    assert load(path="/no/such/sideways_nudge.json") == DEFAULTS


def test_load_clamps_and_fills_missing_keys(tmp_path):
    p = tmp_path / "sideways_nudge.json"
    p.write_text(json.dumps({"left": 99, "right": 3}))
    assert load(path=p) == {"left": MAX_PULSES, "right": 3}


def test_load_ignores_non_numeric_values(tmp_path):
    p = tmp_path / "sideways_nudge.json"
    p.write_text(json.dumps({"left": "seven", "right": 4}))
    assert load(path=p) == {"left": 0, "right": 4}


def test_save_then_load_roundtrips(tmp_path):
    p = tmp_path / "sideways_nudge.json"
    save({"left": 5, "right": 8}, path=p)
    assert load(path=p) == {"left": 5, "right": 8}
