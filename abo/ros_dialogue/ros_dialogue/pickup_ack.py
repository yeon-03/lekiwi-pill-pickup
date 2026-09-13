"""약통 픽업 요청에 대한 에이보의 첫 대답 — LLM 없이 코드가 정한 문장 (ROS 무관 순수 로직).

음성과 웹 버튼, 두 경로가 같은 대답을 쓴다(2026-09-13).

- 음성: LLM 은 pickup_medicine(color) 도구 호출로 **어떤 색인지만** 고른다. dialogue_node 가
  색을 검사해 pickup/medicine/<색> 을 발행하고, 대답은 여기서 만든 고정 문구로 말한 뒤 LLM
  대답 생성 없이 턴을 끝낸다 — "가져왔어요"처럼 사실과 다른 말을 LLM 이 지어낼 여지를 없앤다
  (도구 결과로 "가져왔다고 말하지 마세요"라고 부탁하는 것만으로는 문장을 LLM 이 만들기 때문에
  보장이 안 된다).
- 웹 버튼: companion_bridge_node 가 같은 토픽을 바로 발행한다. dialogue_node 는 그 토픽을
  구독해, 자기가 방금 발행한 것(음성 경로에서 보낸 토픽이 구독으로 되돌아온 것)이 아니면 같은
  문장을 말한다. LLM 은 아예 부르지 않는다.

문장에 들어갈 수 있는 색은 COLOR_KR 의 세 가지뿐이다 — 도구 인자는 Literal 로 제한되고,
여기서 한 번 더 막는다.
"""
from __future__ import annotations

import time

from .lekiwi_status import COLOR_KR

PICKUP_ACK_TEMPLATE = '네, 알겠어요! {color_kr} 약을 가져올게요.'
# 이 노드가 발행한 토픽이 구독으로 되돌아오는 데 걸리는 여유 시간(같은 프로세스라 보통 수 ms)
OWN_PUBLISH_WINDOW_S = 2.0
# 대화 중이라 바로 못 말한 버튼 대답을 이 시간(초)까지만 기다린다 — 너무 늦은 "가져올게요"는
# 이미 르키위가 출발한 뒤라 오히려 헷갈린다.
BUTTON_ACK_MAX_WAIT_S = 30.0


def pickup_ack_text(color: str) -> str:
    """색 코드(red/blue/green) -> 고정 대답 문장. 모르는 색이면 ValueError."""
    if color not in COLOR_KR:
        raise ValueError(f'알 수 없는 약통 색상: {color!r}')
    return PICKUP_ACK_TEMPLATE.format(color_kr=COLOR_KR[color])


class PickupAckGate:
    """pickup/medicine/<색> 구독으로 들어온 요청 중 '웹 버튼 요청'만 골라 대답 차례를 기다린다.

    - 음성 경로는 발행 직전에 mark_own_publish() 를 부른다. 그 토픽이 구독으로 되돌아오면
      on_topic() 이 False 를 돌려준다(이미 음성 턴에서 대답함).
    - 버튼 대답은 한 칸만 들고 있는다. 대화 중에 버튼을 여러 번 누르면 마지막 색만 남는다.
    """

    def __init__(self, own_window_s: float = OWN_PUBLISH_WINDOW_S,
                 max_wait_s: float = BUTTON_ACK_MAX_WAIT_S):
        self._own_window_s = own_window_s
        self._max_wait_s = max_wait_s
        self._own: dict[str, float] = {}
        self._pending: tuple[str, float] | None = None

    def mark_own_publish(self, color: str, now: float | None = None) -> None:
        """음성 경로가 토픽을 발행하기 직전에 호출한다.

        그 턴에서 바로 대답하므로 기다리던 버튼 대답은 버린다(같은 요청을 두 번 말하지 않게).
        """
        self._own[color] = time.monotonic() if now is None else now
        self._pending = None

    def on_topic(self, color: str, now: float | None = None) -> bool:
        """구독으로 받은 요청. 버튼 요청이라 대답해야 하면 대기 칸에 넣고 True."""
        now = time.monotonic() if now is None else now
        if color not in COLOR_KR:
            return False
        own_at = self._own.pop(color, None)
        if own_at is not None and now - own_at < self._own_window_s:
            return False
        self._pending = (color, now)
        return True

    def pending_text(self, now: float | None = None) -> str | None:
        """지금 말할 버튼 대답을 돌려준다(아직 칸에서 빼지 않음).

        없거나 너무 오래 기다렸으면 None — 오래된 것은 이때 버린다.
        """
        if self._pending is None:
            return None
        now = time.monotonic() if now is None else now
        color, at = self._pending
        if now - at > self._max_wait_s:
            self._pending = None
            return None
        return pickup_ack_text(color)

    def clear_pending(self) -> None:
        self._pending = None
