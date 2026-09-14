import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from stop_latch import StopLatch  # noqa: E402


def test_not_latched_never_sends():
    s = StopLatch()
    s.on_bridge_state("moving", 1.0)
    assert s.tick(1.0) == [] and s.tick(2.0) == []
    assert s.on_pick_reachable(True) == []


def test_engage_sends_estop_and_stop_once():
    s = StopLatch()
    assert s.engage(0.0) == ["estop", "stop"]
    assert s.snapshot() == {"latched": True, "since": 0.0, "sent": 1, "last_result": None}
    assert s.tick(0.1) == []


def test_no_resend_while_picking():
    s = StopLatch()
    s.on_bridge_state("picking", 0.0)
    s.engage(1.0)
    assert [s.tick(t) for t in (2.0, 3.0, 4.0)] == [[], [], []]


def test_return_after_pick_resends_every_half_second_for_five_seconds():
    s = StopLatch()
    s.on_bridge_state("picking", 0.0)
    s.engage(1.0)
    s.on_bridge_state("returning", 10.0)
    assert s.tick(10.0) == ["stop"]
    assert s.tick(10.2) == []
    assert s.tick(10.5) == ["stop"]
    assert s.tick(15.0) == ["stop"]          # 창 끝(10+5) 포함
    assert s.tick(15.6) == []                # 창 지남
    s.on_bridge_state("moving", 20.0)        # 위치 다듬기 뒤 실제 복귀 주행 시작 → 창 다시 열림
    assert s.tick(20.0) == ["stop"]
    s.on_bridge_state("moving", 21.0)        # 피드백은 창을 다시 열지 않음
    assert s.tick(25.1) == []
    assert s.snapshot()["sent"] == 1 + 3 + 1


def test_canceled_closes_window():
    s = StopLatch()
    s.engage(0.0)
    s.on_bridge_state("moving", 30.0)
    assert s.tick(30.0) == ["stop"]
    s.on_bridge_state("canceled", 30.3)
    assert s.tick(30.8) == []


def test_engage_while_already_moving_opens_window():
    s = StopLatch()
    s.on_bridge_state("moving", 0.0)
    assert s.engage(3.0) == ["estop", "stop"]
    assert s.tick(3.2) == []
    assert s.tick(3.5) == ["stop"]


def test_idle_reply_then_moving_feedback_reopens():
    s = StopLatch()
    s.engage(0.0)
    s.on_bridge_state("moving", 1.0)          # 목표 수락 전에 stop 이 도착 → 브리지는 idle 응답
    s.on_bridge_state("idle", 1.1)
    assert s.tick(1.6) == []
    s.on_bridge_state("moving", 4.0)          # 3 초 뒤 주행 피드백
    assert s.tick(4.0) == ["stop"]


def test_pick_web_coming_up_while_latched_requests_estop():
    s = StopLatch()
    assert s.on_pick_reachable(False) == []
    s.engage(0.0)
    assert s.on_pick_reachable(True) == ["estop"]
    assert s.on_pick_reachable(True) == []


def test_release_stops_everything_and_note_records_result():
    s = StopLatch()
    s.engage(0.0)
    s.note("estop", "estop 실패: timed out")
    assert s.snapshot()["last_result"] == "estop 실패: timed out"
    s.release()
    s.on_bridge_state("moving", 5.0)
    assert s.tick(5.0) == []
    assert s.snapshot()["latched"] is False
