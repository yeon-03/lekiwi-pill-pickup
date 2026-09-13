import pytest

from ros_dialogue.pickup_ack import (
    BUTTON_ACK_MAX_WAIT_S, OWN_PUBLISH_WINDOW_S, pickup_ack_text, PickupAckGate)


@pytest.mark.parametrize('color, kr', [('red', '빨간색'), ('blue', '파란색'), ('green', '초록색')])
def test_ack_text_puts_color_into_fixed_sentence(color, kr):
    assert pickup_ack_text(color) == f'네, 알겠어요! {kr} 약을 가져올게요.'


@pytest.mark.parametrize('bad', ['yellow', '파란색', '', None])
def test_ack_text_rejects_unknown_color(bad):
    with pytest.raises(ValueError):
        pickup_ack_text(bad)


def test_button_request_waits_for_ack():
    gate = PickupAckGate()
    assert gate.on_topic('blue', now=10.0) is True
    assert gate.pending_text(now=10.5) == '네, 알겠어요! 파란색 약을 가져올게요.'


def test_own_voice_publish_echo_is_not_answered_again():
    gate = PickupAckGate()
    gate.mark_own_publish('green', now=10.0)
    assert gate.on_topic('green', now=10.01) is False
    assert gate.pending_text(now=10.02) is None


def test_own_mark_is_used_once_then_same_color_button_is_answered():
    gate = PickupAckGate()
    gate.mark_own_publish('red', now=10.0)
    assert gate.on_topic('red', now=10.01) is False      # 음성 경로 되돌아옴
    assert gate.on_topic('red', now=10.5) is True        # 곧이어 누른 버튼


def test_own_mark_expires_after_window():
    gate = PickupAckGate()
    gate.mark_own_publish('red', now=10.0)
    assert gate.on_topic('red', now=10.0 + OWN_PUBLISH_WINDOW_S + 0.1) is True


def test_own_mark_for_other_color_does_not_hide_button():
    gate = PickupAckGate()
    gate.mark_own_publish('red', now=10.0)
    assert gate.on_topic('blue', now=10.01) is True


def test_voice_request_drops_waiting_button_ack():
    gate = PickupAckGate()
    gate.on_topic('blue', now=10.0)
    gate.mark_own_publish('blue', now=11.0)
    assert gate.pending_text(now=11.1) is None


def test_latest_button_press_replaces_waiting_one():
    gate = PickupAckGate()
    gate.on_topic('red', now=10.0)
    gate.on_topic('green', now=12.0)
    assert gate.pending_text(now=12.5) == '네, 알겠어요! 초록색 약을 가져올게요.'


def test_stale_button_ack_is_dropped():
    gate = PickupAckGate()
    gate.on_topic('red', now=10.0)
    assert gate.pending_text(now=10.0 + BUTTON_ACK_MAX_WAIT_S + 1) is None
    assert gate.pending_text(now=10.0) is None           # 한 번 버리면 다시 안 나온다


def test_pending_text_does_not_consume_until_cleared():
    gate = PickupAckGate()
    gate.on_topic('red', now=10.0)
    assert gate.pending_text(now=11.0) is not None
    assert gate.pending_text(now=12.0) is not None       # 말할 수 없어 재시도하는 동안 유지
    gate.clear_pending()
    assert gate.pending_text(now=12.0) is None


def test_unknown_color_topic_is_ignored():
    gate = PickupAckGate()
    assert gate.on_topic('yellow', now=10.0) is False
    assert gate.pending_text(now=10.0) is None
