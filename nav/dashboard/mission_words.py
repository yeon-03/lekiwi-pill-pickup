"""미션 상태(/abo/state) → 사람이 읽는 한 문장 ("파란색 약을 향해 가는 중"). ROS 없음.

출처: 에이보 브랜치 origin/feature/abo-web-html 의 nav/mission/mission_text.py (STATE_TEXT,
effective_state) 와 nav/mission/color_intent.py (COLOR_KR). 에이보 웹앱과 이 대시보드가
같은 문장을 보여줘야 하므로 **문구를 바꾸면 양쪽을 같이** 고친다.
"""
from __future__ import annotations

COLOR_KR = {"red": "빨간색", "blue": "파란색", "green": "초록색"}

STATE_TEXT = {
    "sent": "{c} 약을 가져오라고 르키위에게 전달했어요",
    "moving": "{c} 약을 향해 가는 중",
    "arrived": "{c} 약 앞에 도착했어요",
    "picking": "{c} 약을 집는 중",
    "returning": "{c} 약을 가지고 돌아오는 중",
    "done": "{c} 약을 가져왔어요",
    "failed": "{c} 약을 가져오지 못했어요",
    "rejected": "르키위가 요청을 거절했어요",
    "canceled": "르키위가 멈췄어요",
    "idle": "르키위 대기 중",
}

NO_MISSION = "미션 없음 — fetch 명령을 기다리는 중"

# 브리지는 복귀 주행 중에도 state="moving" 을 보낸다 — 집기 뒤의 moving 은 복귀다.
AFTER_PICK = ("picking", "returning")


def effective_state(prev_state: str | None, state: str | None) -> str | None:
    if state == "moving" and prev_state in AFTER_PICK:
        return "returning"
    return state


def headline(state: str | None, color: str | None, prev_state: str | None = None) -> str:
    """state 가 None 이면 미션 없음. 모르는 state 는 그대로. 모르는 색은 색 없이."""
    if state is None:
        return NO_MISSION
    state = effective_state(prev_state, state)
    tmpl = STATE_TEXT.get(state)
    if tmpl is None:
        return str(state)
    c = COLOR_KR.get(color) if color else None
    return tmpl.format(c=c) if c else tmpl.format(c="").replace(" 약", "약").strip()
