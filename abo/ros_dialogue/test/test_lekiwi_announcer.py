import json

from ros_dialogue.lekiwi_announcer import ANNOUNCE_TEXT, LekiwiAnnouncer


def ms(state, key=100.0, color='green', picked=None, status=''):
    return json.dumps({'state': state, 'color': color, 'received_at': key, 'picked': picked,
                       'status': status}, ensure_ascii=False)


def drain(a):
    out = []
    while a.has_pending():
        out.append(a.next_text())
    return out


def test_real_success_flow_speaks_each_step_once():
    """2026-09-13 실기기 성공 흐름 그대로 (브리지가 3초마다 같은 상태를 다시 보냄)."""
    a = LekiwiAnnouncer()
    said = []
    for raw in [ms('sent'), ms('moving'), ms('moving', status='1.4 m 남았어요'), ms('moving'),
                ms('arrived'), ms('picking'), ms('picking'),
                ms('returning', picked=True),            # /abo/pick_done true 뒤 재정합
                ms('returning', picked=True), ms('returning', picked=True),
                ms('done', picked=True)]:
        a.on_mission_state(raw)
        said += drain(a)
    assert said == ['주행 시작할게요.',
                    '도착했어요. 약을 집을게요.',
                    '약 집기 성공했어요. 가져다 드릴게요.',
                    '약을 가져왔어요.']


def test_pick_failure_flow():
    a = LekiwiAnnouncer()
    said = []
    for raw in [ms('sent', color='blue'), ms('moving', color='blue'), ms('arrived', color='blue'),
                ms('picking', color='blue'), ms('returning', color='blue', picked=False),
                ms('failed', color='blue', picked=False)]:
        a.on_mission_state(raw)
        said += drain(a)
    assert said[-2:] == ['약 집기에 실패했어요. 제자리로 돌아갈게요.',
                         '돌아왔지만 약은 가져오지 못했어요.']


def test_return_drive_moving_is_not_announced_as_start():
    """복귀 주행도 브리지는 moving 을 보낸다 — 주행 시작을 다시 말하면 안 된다."""
    a = LekiwiAnnouncer()
    for raw in [ms('moving'), ms('arrived'), ms('picking')]:
        a.on_mission_state(raw)
    drain(a)
    a.on_mission_state(ms('moving'))
    assert drain(a) == []


def test_picked_result_even_if_state_name_is_same():
    a = LekiwiAnnouncer()
    a.on_mission_state(ms('picking'))
    drain(a)
    assert a.on_mission_state(ms('picking', picked=True)) is True
    assert drain(a) == ['약 집기 성공했어요. 가져다 드릴게요.']


def test_progress_announcements_are_merged_when_busy_but_results_kept():
    """대화 중이라 못 말하는 사이 여러 상태가 오면 진행 안내는 최신만, 결과는 전부."""
    a = LekiwiAnnouncer()
    for raw in [ms('moving'), ms('arrived'), ms('picking'), ms('returning', picked=True),
                ms('done', picked=True)]:
        a.on_mission_state(raw)
    assert drain(a) == ['약 집기 성공했어요. 가져다 드릴게요.', '약을 가져왔어요.']


def test_new_mission_resets_and_drops_old_pending():
    a = LekiwiAnnouncer()
    a.on_mission_state(ms('moving', key=1.0))
    a.on_mission_state(ms('sent', key=2.0, color='red'))
    a.on_mission_state(ms('moving', key=2.0, color='red'))
    assert drain(a) == ['주행 시작할게요.']


def test_same_event_in_new_mission_is_spoken_again():
    a = LekiwiAnnouncer()
    a.on_mission_state(ms('done', key=1.0))
    drain(a)
    a.on_mission_state(ms('done', key=2.0))
    assert drain(a) == ['약을 가져왔어요.']


def test_failed_without_pick_uses_generic_text():
    a = LekiwiAnnouncer()
    a.on_mission_state(ms('moving'))
    a.on_mission_state(ms('failed'))
    assert drain(a) == ['르키위가 약을 가져오지 못했어요.']


def test_rejected_and_canceled():
    a = LekiwiAnnouncer()
    a.on_mission_state(ms('rejected', key=5.0))
    assert drain(a) == [ANNOUNCE_TEXT['rejected']]
    a.on_mission_state(ms('canceled', key=6.0))
    assert drain(a) == [ANNOUNCE_TEXT['canceled']]


def test_sent_idle_picking_returning_alone_say_nothing():
    a = LekiwiAnnouncer()
    for s in ('sent', 'idle', 'picking', 'returning'):
        assert a.on_mission_state(ms(s)) is False
    assert drain(a) == []


def test_progress_texts_have_no_color_word():
    a = LekiwiAnnouncer()
    a.on_mission_state(ms('arrived', color='red'))
    assert drain(a) == ['도착했어요. 약을 집을게요.']


def test_bad_json_ignored():
    a = LekiwiAnnouncer()
    assert a.on_mission_state('not json') is False
    assert a.on_mission_state('[1, 2]') is False
    assert a.on_mission_state(None) is False


def test_peek_does_not_consume():
    a = LekiwiAnnouncer()
    a.on_mission_state(ms('moving'))
    assert a.peek_text() == '주행 시작할게요.'
    assert a.has_pending()
    assert a.next_text() == '주행 시작할게요.'
    assert not a.has_pending()
