"""르키위(별도 로봇) 심부름 상태를 동반 웹앱에 보여주기 위한 순수 로직 — ROS 무관.

두 입력을 모아 웹앱 한 칸에 보여줄 스냅샷을 만든다.
- `pickup/medicine/<색>` (data=색). 두 경로로 생긴다:
    · 음성 — dialogue_node 가 사용자 발화에서 LLM 도구(pickup_medicine)로 판단해 발행
    · 버튼 — 웹앱 "르키위" 탭의 물약 버튼 (companion_bridge_node 의 POST /lekiwi/pickup/{색})
  어느 쪽이든 같은 토픽이라, 그 뒤에서 색을 담는 곳(노트북 medicine_relay 의 color,
  르키위 abo_nav_bridge 의 mission["color"], pick_adapter 의 색)은 똑같이 바뀐다.
- `/lekiwi/mission_state` (노트북의 lekiwi-pill-pickup `medicine_relay.py` 가 르키위의
  /abo/state 를 받아 되돌려주는 JSON — `action_text` 에 "빨간색 약을 향해 가는 중" 같은
  문장이 이미 들어 있다. 문구는 그쪽 `mission_text.py` 한 곳에서 정한다)

중계기가 안 떠 있으면 토픽만 보이고 동작 칸은 "요청을 받았어요"로 남는다.
"""
from __future__ import annotations

import json
import time

COLOR_KR = {'red': '빨간색', 'blue': '파란색', 'green': '초록색'}
MEDICINE_COLORS = tuple(COLOR_KR)
# 이 상태에서 새 요청이 오면 이전 동작 표시를 지운다 (예전 미션의 "가져왔어요"가 새 요청에 붙어 보이지 않게)
TERMINAL_STATES = ('done', 'failed', 'rejected', 'canceled', 'idle')
# 같은 색 버튼을 이 시간(초) 안에 또 누르면 다시 발행하지 않는다 — 두 번 누름이 르키위에
# 명령을 두 번 보내지 않게. 자기가 발행한 토픽이 구독으로 되돌아오는 것도 이 창으로 알아본다.
REPEAT_WINDOW_S = 2.0
SOURCE_KR = {'button': '웹 버튼으로 요청', 'topic': '음성(토픽)으로 요청'}


class LekiwiMissionView:
    def __init__(self):
        self.pickup: dict | None = None
        self.mission: dict | None = None

    def is_recent_request(self, color: str, now: float | None = None,
                          window_s: float = REPEAT_WINDOW_S) -> bool:
        """같은 색 요청이 window_s 초 안에 이미 있었나."""
        now = time.time() if now is None else now
        p = self.pickup
        return p is not None and p['color'] == color and now - p['at'] < window_s

    def on_pickup(self, color: str, data: str, now: float | None = None,
                  source: str = 'topic') -> None:
        now = time.time() if now is None else now
        same = self.pickup is not None and self.pickup['color'] == color
        # 버튼이 발행한 토픽이 이 노드의 구독으로 곧바로 되돌아온 것 — 출처는 '버튼'으로 둔다.
        if (source == 'topic' and same and self.pickup.get('source') == 'button'
                and self.is_recent_request(color, now)):
            source = 'button'
        self.pickup = {'topic': f'/pickup/medicine/{color}', 'data': data, 'color': color,
                       'color_kr': COLOR_KR.get(color, color), 'at': now, 'source': source}
        # 같은 요청이 중계 과정에서 두 번 들어와도 진행 중인 동작 표시는 유지한다.
        if not same or (self.mission and self.mission['state'] in TERMINAL_STATES):
            self.mission = None

    def on_mission_state(self, raw: str, now: float | None = None) -> bool:
        """JSON 문자열을 받아 반영한다. 형식이 틀리면 무시하고 False."""
        try:
            d = json.loads(raw)
        except (TypeError, ValueError):
            return False
        if not isinstance(d, dict) or not d.get('state'):
            return False
        received = None
        if d.get('received_topic'):
            received = {'topic': str(d['received_topic']), 'data': d.get('received_data'),
                        'at': d.get('received_at'), 'command': str(d.get('command') or '')}
        self.mission = {
            'received': received,
            'state': str(d['state']),
            'action_text': str(d.get('action_text') or d.get('status') or d['state']),
            'status': str(d.get('status') or ''),
            'color': d.get('color'),
            'dry_run': bool(d.get('dry_run')),
            'at': time.time() if now is None else now,
        }
        return True

    def snapshot(self) -> dict | None:
        """웹앱 상태 JSON 의 'lekiwi' 값. 아무 일도 없었으면 None (카드 숨김)."""
        if self.pickup is None and self.mission is None:
            return None
        p, m = self.pickup, self.mission
        if m is not None:
            action = m['action_text']
        else:
            action = f"{p['color_kr']} 약 요청을 받았어요"
        # 노트북 중계기가 실제로 받은 토픽 — 보낸 토픽과 같은지 웹앱에서 눈으로 확인하게 한다.
        laptop = dict(m['received']) if m and m.get('received') else None
        if laptop is not None:
            laptop['match'] = bool(p and laptop['topic'] == p['topic'] and laptop['data'] == p['data'])
        return {
            'topic': p['topic'] if p else None,
            'data': p['data'] if p else None,
            'color': (p['color'] if p else None) or (m['color'] if m else None),
            'topic_at': p['at'] if p else None,
            'source': p['source'] if p else None,
            'source_text': SOURCE_KR.get(p['source'], '') if p else '',
            'state': m['state'] if m else 'requested',
            'action_text': action,
            'status': m['status'] if m else '',
            'dry_run': m['dry_run'] if m else False,
            'laptop': laptop,
            'updated_at': max(x['at'] for x in (p, m) if x),
        }
