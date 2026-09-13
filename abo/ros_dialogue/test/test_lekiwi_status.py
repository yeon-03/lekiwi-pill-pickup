from ros_dialogue.lekiwi_status import LekiwiMissionView
import json


def state(s, text='', color='red', dry=False):
    return json.dumps({'state': s, 'action_text': text, 'status': '', 'color': color, 'dry_run': dry},
                      ensure_ascii=False)


def test_empty_view_hides_card():
    assert LekiwiMissionView().snapshot() is None


def test_pickup_topic_only():
    v = LekiwiMissionView()
    v.on_pickup('red', 'red', now=10.0)
    s = v.snapshot()
    assert s['topic'] == '/pickup/medicine/red'
    assert s['data'] == 'red'
    assert s['state'] == 'requested'
    assert s['action_text'] == '빨간색 약 요청을 받았어요'
    assert s['source'] == 'topic'


def test_mission_state_shows_relay_text():
    v = LekiwiMissionView()
    v.on_pickup('red', 'red', now=10.0)
    assert v.on_mission_state(state('moving', '빨간색 약을 향해 가는 중'), now=11.0)
    s = v.snapshot()
    assert s['action_text'] == '빨간색 약을 향해 가는 중'
    assert s['topic'] == '/pickup/medicine/red'
    assert s['updated_at'] == 11.0
    v.on_mission_state(state('picking', '빨간색 약을 집는 중'), now=12.0)
    assert v.snapshot()['action_text'] == '빨간색 약을 집는 중'


def test_duplicate_pickup_keeps_progress():
    v = LekiwiMissionView()
    v.on_pickup('red', 'red')
    v.on_mission_state(state('moving', '빨간색 약을 향해 가는 중'))
    v.on_pickup('red', 'red')  # 중계 과정의 중복 수신
    assert v.snapshot()['state'] == 'moving'


def test_new_color_or_finished_mission_resets_action():
    v = LekiwiMissionView()
    v.on_pickup('red', 'red')
    v.on_mission_state(state('moving', '빨간색 약을 향해 가는 중'))
    v.on_pickup('green', 'green')
    assert v.snapshot()['action_text'] == '초록색 약 요청을 받았어요'
    v.on_mission_state(state('done', '초록색 약을 가져왔어요', color='green'))
    v.on_pickup('green', 'green')
    assert v.snapshot()['state'] == 'requested'


def test_bad_json_is_ignored():
    v = LekiwiMissionView()
    assert not v.on_mission_state('not json')
    assert not v.on_mission_state(json.dumps({'no_state': 1}))
    assert v.snapshot() is None


def test_state_without_pickup_topic():
    v = LekiwiMissionView()
    v.on_mission_state(state('returning', '빨간색 약을 가지고 돌아오는 중', dry=True))
    s = v.snapshot()
    assert s['topic'] is None
    assert s['action_text'] == '빨간색 약을 가지고 돌아오는 중'
    assert s['dry_run'] is True


# ── 르키위 탭 물약 버튼(2026-09-13) ──

def test_button_request_source_and_color():
    v = LekiwiMissionView()
    v.on_pickup('blue', 'blue', now=100.0, source='button')
    s = v.snapshot()
    assert s['topic'] == '/pickup/medicine/blue'
    assert s['color'] == 'blue'
    assert s['source'] == 'button'
    assert s['source_text'] == '웹 버튼으로 요청'
    assert s['action_text'] == '파란색 약 요청을 받았어요'


def test_button_echo_through_own_subscription_keeps_button_source():
    v = LekiwiMissionView()
    v.on_pickup('blue', 'blue', now=100.0, source='button')
    v.on_pickup('blue', 'blue', now=100.05)          # 자기가 발행한 토픽이 구독으로 되돌아옴
    assert v.snapshot()['source'] == 'button'
    v.on_pickup('blue', 'blue', now=110.0)            # 한참 뒤 음성으로 같은 색
    assert v.snapshot()['source'] == 'topic'


def test_is_recent_request_window():
    v = LekiwiMissionView()
    assert not v.is_recent_request('red', now=0.0)
    v.on_pickup('red', 'red', now=10.0, source='button')
    assert v.is_recent_request('red', now=11.0)
    assert not v.is_recent_request('red', now=12.5)
    assert not v.is_recent_request('green', now=10.5)


# ── 노트북 수신 확인(2026-09-13) ──

def sent_state(topic='/pickup/medicine/red', data='red', command='fetch center color:red'):
    return json.dumps({'state': 'sent', 'action_text': '빨간색 약을 가져오라고 르키위에게 전달했어요',
                       'color': 'red', 'dry_run': True, 'received_topic': topic,
                       'received_data': data, 'received_at': 50.2, 'command': command},
                      ensure_ascii=False)


def test_laptop_received_topic_matches_sent():
    v = LekiwiMissionView()
    v.on_pickup('red', 'red', now=50.0, source='button')
    v.on_mission_state(sent_state(), now=50.3)
    lap = v.snapshot()['laptop']
    assert lap['topic'] == '/pickup/medicine/red'
    assert lap['data'] == 'red'
    assert lap['command'] == 'fetch center color:red'
    assert lap['at'] == 50.2
    assert lap['match'] is True


def test_laptop_received_mismatch_is_flagged():
    v = LekiwiMissionView()
    v.on_pickup('red', 'red', now=50.0)
    v.on_mission_state(sent_state(topic='/pickup/medicine/blue', data='blue'), now=50.3)
    assert v.snapshot()['laptop']['match'] is False


def test_no_laptop_info_before_relay_answers():
    v = LekiwiMissionView()
    v.on_pickup('green', 'green')
    assert v.snapshot()['laptop'] is None
