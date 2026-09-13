"""르키위 심부름 진행 상황을 에이보가 말로 알려주는 문장 결정 — LLM 없이 고정 문구 (ROS 무관 순수 로직).

입력은 노트북 lekiwi-pill-pickup `medicine_relay.py` 가 도메인 77 로 되돌려주는
`/lekiwi/mission_state` JSON 이다(lekiwi_status.py 와 같은 토픽). 쓰는 필드:
  - state       : sent | moving | arrived | picking | returning | done | failed
                  | rejected | canceled | idle
  - received_at : 노트북이 이번 요청 토픽을 받은 시각 — 미션 하나를 구분하는 열쇠
  - picked      : 집기 결과 true/false, 아직 모르면 null (노트북이 /abo/pick_done 을 받아 채움)

말하는 순간 (2026-09-13 실기기 흐름 기준: 주행 시작 0초 → 도착 29초 → 집기 끝 99초 → 복귀 145초)
  moving(집기 전) → "주행 시작할게요."
  arrived         → "도착했어요. 약을 집을게요."
  picked=true     → "약 집기 성공했어요. 가져다 드릴게요."
  picked=false    → "약 집기에 실패했어요. 제자리로 돌아갈게요."
  done            → "약을 가져왔어요."
  failed          → 집기 실패 뒤면 "돌아왔지만 약은 가져오지 못했어요.", 아니면 "르키위가 약을 가져오지 못했어요."
  rejected / canceled → 거절 / 멈춤 안내

요청 순간의 "네, 알겠어요! 초록색 약을 가져올게요." 는 pickup_ack.py 가 따로 말한다.

상태 문장을 LLM 에 맡기지 않는 이유는 pickup_ack.py 와 같다 — 실제로 일어난 일만 말해야 한다.
"""
from __future__ import annotations

import json

# 이벤트 -> 문장. 색은 요청 순간의 대답(pickup_ack.py, "네, 알겠어요! 초록색 약을 가져올게요.")에만 넣는다.
ANNOUNCE_TEXT = {
    'moving': '주행 시작할게요.',
    'arrived': '도착했어요. 약을 집을게요.',
    'picked_ok': '약 집기 성공했어요. 가져다 드릴게요.',
    'picked_fail': '약 집기에 실패했어요. 제자리로 돌아갈게요.',
    'done': '약을 가져왔어요.',
    'failed_after_pick': '돌아왔지만 약은 가져오지 못했어요.',
    'failed': '르키위가 약을 가져오지 못했어요.',
    'rejected': '르키위가 지금은 요청을 받을 수 없어요.',
    'canceled': '르키위가 멈췄어요.',
}
# 이 이벤트들은 미션의 끝(또는 결과)이라 대화 중에 밀려도 버리지 않는다.
# 나머지(moving/arrived)는 진행 안내라, 말하기 전에 다음 안내가 오면 버린다.
RESULT_EVENTS = ('picked_ok', 'picked_fail', 'done', 'failed_after_pick', 'failed',
                 'rejected', 'canceled')


class LekiwiAnnouncer:
    """mission_state 를 받아 말할 문장을 대기열에 쌓는다.

    dialogue_node 는 말해도 될 때 next_text() 로 하나씩 꺼내 _speak() 한다.
    """

    def __init__(self):
        self._mission_key = None
        self._announced: set[str] = set()   # 이번 미션에서 이미 대기열에 넣은 이벤트
        self._picked = None                 # 이번 미션의 집기 결과
        self._pick_started = False          # picking 이후인지 (복귀 주행의 moving 을 거르기 위해)
        self._queue: list[str] = []         # 이벤트 이름

    def on_mission_state(self, raw: str) -> bool:
        """JSON 을 반영한다. 새로 말할 것이 생기면 True. 형식이 틀리면 무시하고 False."""
        try:
            d = json.loads(raw)
        except (TypeError, ValueError):
            return False
        if not isinstance(d, dict):
            return False
        key = d.get('received_at')
        state = d.get('state')
        if key != self._mission_key:
            # 새 미션 — 이전 미션에서 못 한 안내는 버린다(이미 지난 일).
            self._mission_key = key
            self._announced = set()
            self._picked = None
            self._pick_started = False
            self._queue = []

        events = []
        if state in ('picking', 'returning'):
            self._pick_started = True
        picked = d.get('picked')
        if picked is not None and self._picked is None:
            self._picked = bool(picked)
            self._pick_started = True
            events.append('picked_ok' if self._picked else 'picked_fail')
        if state == 'moving' and not self._pick_started:
            events.append('moving')
        elif state == 'arrived' and not self._pick_started:
            events.append('arrived')
        elif state == 'done':
            events.append('done')
        elif state == 'failed':
            events.append('failed_after_pick' if self._picked is False else 'failed')
        elif state in ('rejected', 'canceled'):
            events.append(state)

        added = False
        for ev in events:
            if ev in self._announced:
                continue
            self._announced.add(ev)
            # 진행 안내는 최신 것만 — 말하기 전에 쌓인 이전 진행 안내를 버린다.
            self._queue = [q for q in self._queue if q in RESULT_EVENTS]
            self._queue.append(ev)
            added = True
        return added

    def has_pending(self) -> bool:
        return bool(self._queue)

    def next_text(self) -> str | None:
        """다음에 말할 문장을 대기열에서 꺼낸다. 없으면 None."""
        if not self._queue:
            return None
        ev = self._queue.pop(0)
        return ANNOUNCE_TEXT[ev]

    def peek_text(self) -> str | None:
        if not self._queue:
            return None
        return ANNOUNCE_TEXT[self._queue[0]]
