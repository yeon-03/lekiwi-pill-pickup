from ros_dialogue.lekiwi_control import (
    ActiveJob,
    MIN_MOVE_DURATION_SEC,
    PICK_MEDICINE_SKILLS,
    SKILL_MAP,
    build_alive_check_command,
    build_launch_command,
    build_stop_command,
    can_start_skill,
    is_known_skill,
    parse_command,
    parse_pid,
    resolve_duration_sec,
    resolve_timeout_sec,
)
import pytest


def test_skill_map_has_move_skills_and_pick_medicine_skills():
    assert set(SKILL_MAP.keys()) == {
        'forward', 'backward', 'left', 'right',
        'pick_red', 'pick_blue', 'pick_green',
    }


def test_move_skills_target_lekiwi_pi_host():
    for skill in ('forward', 'backward', 'left', 'right'):
        assert SKILL_MAP[skill]['host_param'] == 'lekiwi_pi_host'


def test_pick_medicine_skills_target_lekiwi_pi_host():
    # abo_nav_bridge.py(그리고 그걸 통해 SSH로 넘어가는 /abo/command)는
    # lekiwi01 자체에서 도는 것이라 노트북(laptop_ssh_host)이 아니라
    # 로봇(lekiwi_pi_host)으로 SSH해야 한다.
    for skill in PICK_MEDICINE_SKILLS:
        assert SKILL_MAP[skill]['host_param'] == 'lekiwi_pi_host'


def test_pick_medicine_skills_run_pick_trigger_with_matching_color():
    for skill, color in zip(PICK_MEDICINE_SKILLS, ('red', 'blue', 'green')):
        assert SKILL_MAP[skill]['command_template'] == f'~/pick_trigger.sh {color}'


def test_pick_medicine_skill_is_known():
    for skill in PICK_MEDICINE_SKILLS:
        assert is_known_skill(skill) is True


def test_pick_medicine_skill_duration_ignores_requested_value():
    # MOVE_SKILLS가 아니므로 요청값과 무관하게 SKILL_MAP 고정값(0.0)을 쓴다.
    assert resolve_duration_sec('pick_red', 999.0) == 0.0


def test_pick_medicine_skill_timeout_matches_skill_map_fixed_value():
    assert resolve_timeout_sec('pick_red', 0.0) == SKILL_MAP['pick_red']['timeout_sec']


def test_build_launch_command_for_pick_skill_has_no_duration_placeholder_error():
    # command_template에 {duration} 자리가 없어도 .format(duration=...)이
    # 에러 없이 지나가야 한다(사용하지 않는 kwarg는 무시됨).
    cmd = build_launch_command('pick_red')
    assert 'pick_trigger.sh red' in cmd
    assert 'timeout 15s' in cmd


def test_is_known_skill_true_for_mapped_skill():
    assert is_known_skill('forward') is True


def test_is_known_skill_true_for_stop():
    assert is_known_skill('stop') is True


def test_is_known_skill_false_for_unknown():
    assert is_known_skill('fly') is False


def test_can_start_skill_true_when_idle():
    assert can_start_skill(None) is True


def test_can_start_skill_false_when_active():
    job = ActiveJob(skill='forward', host='pi@1.2.3.4', started_at=0.0)
    assert can_start_skill(job) is False


def test_build_launch_command_includes_timeout_and_pid_capture():
    cmd = build_launch_command('forward')
    assert 'timeout 12s' in cmd
    assert cmd.strip().endswith('echo $!')


def test_build_launch_command_move_skill_duration_differs_from_outer_timeout():
    # 연결 오버헤드(~9초 실측)를 감안해 바깥쪽 timeout은 스크립트에 넘기는
    # 실제 이동시간(duration)보다 커야 한다 — 같으면 연결이 끝나기 전에
    # 강제종료되어 로봇이 전혀 안 움직이는 버그가 재발한다(2026-08-11 실측).
    cmd = build_launch_command('forward')
    assert 'move_skill.py forward 2' in cmd
    assert 'timeout 12s' in cmd


def test_build_launch_command_survives_ssh_disconnect():
    cmd = build_launch_command('forward')
    assert cmd.startswith('setsid nohup')


def test_parse_pid_valid():
    assert parse_pid('12345\n') == 12345


def test_parse_pid_ignores_leading_noise_lines():
    assert parse_pid('Warning: something\n67890\n') == 67890


def test_parse_pid_raises_on_empty_output():
    with pytest.raises(ValueError):
        parse_pid('')


def test_parse_pid_raises_on_non_numeric_output():
    with pytest.raises(ValueError):
        parse_pid('command not found\n')


def test_build_alive_check_command():
    cmd = build_alive_check_command(42)
    assert 'kill -0 42' in cmd
    assert cmd.strip().endswith('|| echo DEAD')


def test_build_stop_command_sends_int_then_grace_then_kill():
    cmd = build_stop_command(42, grace_sec=3.0)
    assert 'kill -INT -42' in cmd
    assert 'sleep 3.0' in cmd
    assert 'kill -9 -42' in cmd


def test_build_stop_command_targets_process_group_not_single_pid():
    # setsid로 launch했으므로 pid==pgid — 프로세스 그룹 전체에 신호를 보내야
    # timeout이 fork한 자식까지 종료된다(단일 PID만 죽이면 자식이 고아로 남는
    # 실측 확인된 버그의 재발 방지 테스트).
    cmd = build_stop_command(42, grace_sec=3.0)
    assert cmd == 'kill -INT -42 2>/dev/null ; sleep 3.0 ; kill -9 -42 2>/dev/null ; true'


def test_build_stop_command_always_exits_zero():
    # SIGINT로 이미 정상 종료된 경우(대부분) 뒤이은 kill -9는 "No such process"로
    # 실패하는 게 정상 흐름 — 호출부가 이 결과를 성공/실패 판정에 쓰지 않으므로
    # 커맨드 자체의 종료코드가 항상 0이 되도록 강제해야 한다.
    cmd = build_stop_command(42, grace_sec=3.0)
    assert cmd.strip().endswith('; true')


# ── 사용자 요청 지속시간(2026-08-14) ──

def test_resolve_duration_sec_uses_default_when_none_requested():
    assert resolve_duration_sec('forward', None) == SKILL_MAP['forward']['duration_sec']


def test_resolve_duration_sec_honors_request_within_range():
    assert resolve_duration_sec('forward', 5.0) == 5.0


def test_resolve_duration_sec_has_no_upper_bound():
    # 2026-08-14 사용자 요청으로 상한 제거 — 큰 값도 그대로 통과해야 한다.
    assert resolve_duration_sec('forward', 999.0) == 999.0


def test_resolve_duration_sec_clamps_to_min():
    assert resolve_duration_sec('forward', 0.01) == MIN_MOVE_DURATION_SEC


def test_resolve_timeout_sec_matches_skill_map_at_default_duration():
    # 기본 duration을 그대로 넣으면 SKILL_MAP에 미리 계산돼 있는 고정 timeout_sec와
    # 정확히 같아야 한다(회귀 방지 — 여기 별도 마진을 더 얹으면 안 됨).
    default_duration = SKILL_MAP['forward']['duration_sec']
    assert resolve_timeout_sec('forward', default_duration) == SKILL_MAP['forward']['timeout_sec']


def test_resolve_timeout_sec_scales_with_longer_duration():
    short = resolve_timeout_sec('forward', 2.0)
    long = resolve_timeout_sec('forward', 8.0)
    assert long > short
    assert long - short == 6.0


def test_build_launch_command_with_custom_duration_overrides_default():
    cmd = build_launch_command('forward', duration_sec=7.0)
    assert 'move_skill.py forward 7' in cmd


def test_build_launch_command_with_large_custom_duration_is_not_clamped():
    cmd = build_launch_command('forward', duration_sec=999.0)
    assert 'move_skill.py forward 999' in cmd


def test_build_launch_command_without_duration_matches_original_default():
    # duration_sec을 안 넘긴 기존 호출부(다른 곳에서 아직 안 바꼈을 수 있음)가
    # 예전과 똑같이 동작해야 한다 — 하위호환 확인.
    assert build_launch_command('forward') == build_launch_command('forward', None)


def test_parse_command_skill_only():
    assert parse_command('forward') == ('forward', None)


def test_parse_command_with_duration():
    assert parse_command('forward:7.5') == ('forward', 7.5)


def test_parse_command_strips_whitespace():
    assert parse_command('  forward  ') == ('forward', None)


def test_parse_command_ignores_non_numeric_duration():
    # 방어적 파싱 — 콜론 뒤가 숫자가 아니면 지속시간 없이 스킬만 요청한 것으로
    # 취급한다(예외를 던져 노드 콜백이 죽는 것보다 안전).
    assert parse_command('forward:abc') == ('forward', None)
