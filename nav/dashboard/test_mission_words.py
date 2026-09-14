import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import pytest  # noqa: E402

from mission_words import NO_MISSION, STATE_TEXT, headline  # noqa: E402


@pytest.mark.parametrize("state, text", [
    ("sent", "파란색 약을 가져오라고 르키위에게 전달했어요"),
    ("moving", "파란색 약을 향해 가는 중"),
    ("arrived", "파란색 약 앞에 도착했어요"),
    ("picking", "파란색 약을 집는 중"),
    ("returning", "파란색 약을 가지고 돌아오는 중"),
    ("done", "파란색 약을 가져왔어요"),
    ("failed", "파란색 약을 가져오지 못했어요"),
    ("rejected", "르키위가 요청을 거절했어요"),
    ("canceled", "르키위가 멈췄어요"),
    ("idle", "르키위 대기 중"),
])
def test_every_state_with_color(state, text):
    assert headline(state, "blue") == text


def test_all_states_covered_and_colors_korean():
    assert set(STATE_TEXT) == {"sent", "moving", "arrived", "picking", "returning", "done", "failed",
                               "rejected", "canceled", "idle"}
    assert headline("moving", "red") == "빨간색 약을 향해 가는 중"
    assert headline("done", "green") == "초록색 약을 가져왔어요"


def test_without_color_drops_color_word():
    assert headline("moving", None) == "약을 향해 가는 중"
    assert headline("arrived", None) == "약 앞에 도착했어요"
    assert headline("failed", "purple") == "약을 가져오지 못했어요"     # 모르는 색도 색 없이


def test_moving_after_pick_is_returning():
    assert headline("moving", "blue", prev_state="picking") == "파란색 약을 가지고 돌아오는 중"
    assert headline("moving", "blue", prev_state="returning") == "파란색 약을 가지고 돌아오는 중"
    assert headline("moving", "blue", prev_state="arrived") == "파란색 약을 향해 가는 중"


def test_no_mission_and_unknown_state():
    assert headline(None, None) == NO_MISSION == "미션 없음 — fetch 명령을 기다리는 중"
    assert headline("warping", "blue") == "warping"
