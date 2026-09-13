#!/usr/bin/env python3
"""르키위 심부름 상태(/abo/state) -> 사람이 읽는 한 줄 ("빨간색 약을 향해 가는 중").

medicine_relay.py 가 이 문장을 만들어 에이보 쪽(도메인 77) /lekiwi/mission_state 로
돌려보낸다. 에이보 동반 웹앱과 터미널 뷰어(nav/tools/abo_console.py)는 받은 문장을
그대로 보여주기만 한다 -- 문구를 바꾸려면 여기 한 곳만 고치면 된다.

/abo/state 값은 abo_nav_bridge.py 가 정한다:
    idle | moving | arrived | picking | returning | done | failed | rejected | canceled
여기에 중계기 자신이 명령을 넘긴 순간의 "sent" 를 더한다.
"""
import json
import time

from color_intent import COLOR_KR

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


# 집기 뒤의 "moving" 은 출발 자리로 돌아가는 주행이다 -- 브리지는 복귀할 때도 go() 가
# state="moving" 을 보낸다. 그대로 쓰면 "약을 가지고 돌아오는 중" 다음에 "약을 향해 가는 중"이
# 다시 떠서 사용자를 헷갈리게 한다 (2026-09-13 가짜 르키위 시험에서 발견).
AFTER_PICK = ("picking", "returning")


def effective_state(prev_state, state):
    """직전 상태를 보고 표시용 상태를 고른다. 집기 뒤 moving -> returning."""
    if state == "moving" and prev_state in AFTER_PICK:
        return "returning"
    return state


def action_text(state, color=None, status=""):
    """상태와 색으로 문장을 만든다. 모르는 상태면 브리지가 보낸 status 를 그대로 쓴다."""
    tmpl = STATE_TEXT.get(state)
    if tmpl is None:
        return status or str(state)
    c = COLOR_KR.get(color) if color else None
    text = tmpl.format(c=c) if c else tmpl.format(c="").replace(" 약", "약").strip()
    return text


def mission_state_json(state, status="", color=None, target="", dry_run=False, now=None,
                       received=None, command=""):
    """/lekiwi/mission_state 에 실어 보낼 JSON 문자열.

    received: 노트북 중계기가 **실제로 받은** 토픽 {"topic", "data", "at"} -- 에이보 웹앱이
              "보낸 토픽"과 나란히 보여줘 수신을 눈으로 확인할 수 있게 한다.
    command : 그 토픽으로 만든 르키위 명령 ("fetch center color:red").
    """
    received = received or {}
    return json.dumps({
        "received_topic": received.get("topic"),
        "received_data": received.get("data"),
        "received_at": received.get("at"),
        "command": command,
        "state": state,
        "action_text": action_text(state, color, status),
        "status": status,
        "color": color,
        "target": target,
        "dry_run": bool(dry_run),
        "ts": round(now if now is not None else time.time(), 3),
    }, ensure_ascii=False)
