"""LeKiwi 스킬 트리거의 순수 로직(설정 데이터, 원격 커맨드 조립, PID 파싱, 동시성 판정)
— ROS 의존성 없는 순수 파이썬 모듈. session_manager.py/profile_store.py와 같은 패턴.

skill 이름 -> 실행 host/커맨드 매핑(SKILL_MAP)의 forward/backward/left/right는
2026-08-10 LeKiwi 실물(lekiwi01, 192.168.0.201)로 직접 검증 완료. LeRobot 공식
CLI(`lerobot-teleoperate`)엔 "커맨드라인으로 그냥 앞으로 가라" 같은 기능 자체가
없음을 소스코드 확인으로 발견(teleoperate는 사람이 리더암+키보드로 실시간
조종하는 대화형 스크립트) — 그래서 LeKiwi Pi의 ~/move_skill.py(신규 작성,
LeKiwiClient로 localhost의 lekiwi_host.py에 ZMQ 접속해 x.vel/y.vel/theta.vel을
워치독(500ms)보다 빠른 주기로 반복 전송)를 대신 트리거하는 방식으로 확정.
⚠️ move_skill.py는 매 전송마다 현재 팔 위치를 "그대로 유지"로 같이 보내야 함
— 안 보내면 호스트의 LeKiwi.send_action()이 빈 팔 딕셔너리에서 StopIteration으로
죽어(str(e)가 빈 문자열이라 로그에 "Message fetching failed: "로만 찍힘) 바퀴
명령까지 통째로 실행이 안 되는 버그를 실측으로 발견함(move_skill.py 자체 docstring
참고). 4방향 전부 실물로 방향(전/후/좌/우 부호 포함) 검증 완료.

⚠️ 2026-08-11 실물 재검증 중 발견 — `LeKiwiClient.connect()` 자체가 실측 8~9초
걸리는데(카메라 스트림 초기화 포함으로 추정), 예전엔 이 연결 오버헤드를 감안하지
않고 `timeout_sec`를 곧 이동시간으로 취급해 3.0초로 잡아뒀었음 — SSH 트리거
경로(`build_launch_command`의 바깥쪽 `timeout {timeout}s`)로 실행하면 연결이
끝나기도 전에 강제종료되어 로봇이 전혀 안 움직이는 버그였음(move_skill.py를
로봇에 직접 SSH 접속해 20초짜리로 수동 실행했을 때만 우연히 성공했던 것).
그래서 이동 커맨드는 `duration_sec`(move_skill.py에 실제로 넘기는 이동 시간)과
`timeout_sec`(바깥쪽 안전장치, 연결 오버헤드+이동시간+여유를 합친 값)를
분리했다 — `command_template`은 이제 `{duration}`을 채운다.
"""
from dataclasses import dataclass

MOVE_SKILLS = ('forward', 'backward', 'left', 'right')
STOP_SKILL = 'stop'

# LeKiwiClient.connect() 실측 오버헤드(약 8.8초, 2026-08-11) + 여유
_CONNECT_OVERHEAD_SEC = 10.0

SKILL_MAP: dict[str, dict] = {
    'forward': {
        'host_param': 'lekiwi_pi_host',
        'command_template': '~/lerobot_venv/bin/python ~/move_skill.py forward {duration}',
        'duration_sec': 2.0,
        'timeout_sec': _CONNECT_OVERHEAD_SEC + 2.0,
    },
    'backward': {
        'host_param': 'lekiwi_pi_host',
        'command_template': '~/lerobot_venv/bin/python ~/move_skill.py backward {duration}',
        'duration_sec': 2.0,
        'timeout_sec': _CONNECT_OVERHEAD_SEC + 2.0,
    },
    'left': {
        'host_param': 'lekiwi_pi_host',
        'command_template': '~/lerobot_venv/bin/python ~/move_skill.py left {duration}',
        'duration_sec': 2.0,
        'timeout_sec': _CONNECT_OVERHEAD_SEC + 2.0,
    },
    'right': {
        'host_param': 'lekiwi_pi_host',
        'command_template': '~/lerobot_venv/bin/python ~/move_skill.py right {duration}',
        'duration_sec': 2.0,
        'timeout_sec': _CONNECT_OVERHEAD_SEC + 2.0,
    },
    # 약통 픽업(색상별). lekiwi01 자체에 SSH로 접속해 ~/pick_trigger.sh(저장소
    # lekiwi-pill-pickup의 nav/shell/pick_trigger.sh)를 실행한다 -- 이 스크립트가
    # /abo/command 에 "fetch center color:<색>"을 1회 발행하면, 그 로봇(도메인
    # 42)에서 이미 돌고 있는 abo_nav_bridge.py가 왕복 미션(이동->픽->복귀)을
    # 이어서 처리한다. 에이보(도메인 77)와 lekiwi01(도메인 42)은 ROS_DOMAIN_ID가
    # 달라 ROS2 토픽을 직접 공유하지 못해서, 이동 스킬과 동일하게 SSH로 경계를
    # 넘는다. duration_sec은 이 스킬에 의미 없어 0(리터럴 -- command_template에
    # {duration} 자리가 없어 어차피 안 쓰임), timeout_sec은 SSH+ros2 CLI 콜드스타트
    # 여유를 넉넉히 잡음(실측 전 추정치).
    'pick_red': {
        'host_param': 'lekiwi_pi_host',
        'command_template': '~/pick_trigger.sh red',
        'duration_sec': 0.0,
        'timeout_sec': 15.0,
    },
    'pick_blue': {
        'host_param': 'lekiwi_pi_host',
        'command_template': '~/pick_trigger.sh blue',
        'duration_sec': 0.0,
        'timeout_sec': 15.0,
    },
    'pick_green': {
        'host_param': 'lekiwi_pi_host',
        'command_template': '~/pick_trigger.sh green',
        'duration_sec': 0.0,
        'timeout_sec': 15.0,
    },
}

PICK_MEDICINE_SKILLS = ('pick_red', 'pick_blue', 'pick_green')


def is_known_skill(skill: str) -> bool:
    """skill이 SKILL_MAP에 있거나 'stop'이면 True."""
    return skill in SKILL_MAP or skill == STOP_SKILL


@dataclass
class ActiveJob:
    skill: str
    host: str
    started_at: float
    pid: int | None = None
    pending_stop: bool = False
    # 이번 작업에 실제로 쓰인(클램프까지 끝난) 지속시간/바깥쪽 timeout(초) —
    # 사용자가 지속시간을 요청한 경우 SKILL_MAP의 고정값과 달라질 수 있어서
    # (resolve_duration_sec/resolve_timeout_sec 참고), _handle_start의
    # build_launch_command 호출과 _check_expiry의 만료 판정이 매번 다시 계산하지
    # 않고 이 값을 그대로 쓴다. None이면(레거시 경로) 호출부가 SKILL_MAP을 대신 쓴다.
    duration_sec: float | None = None
    timeout_sec: float | None = None


def can_start_skill(active_job: 'ActiveJob | None') -> bool:
    """실행 중인 작업이 없어야 새 skill(stop 제외)을 시작할 수 있다."""
    return active_job is None


# 이동 스킬은 장애물 회피가 전혀 없는 단순 dead-reckoning이다(move_skill.py 참고).
# 2026-08-14 최초 도입 시 상한(MAX_MOVE_DURATION_SEC=10.0)을 뒀었으나, 사용자가
# "일단은 시간 제한 없이" 요청해 제거함 — 감독 없이 로봇이 오래 움직일 위험은
# 남아있으니, 나중에 다시 필요해지면 이 함수에 상한만 추가하면 된다(구조는
# 그대로 재사용 가능하게 남겨둠). 0 이하 요청만 최소한으로 막는다.
MIN_MOVE_DURATION_SEC = 0.5


def resolve_duration_sec(skill: str, requested_duration_sec: float | None) -> float:
    """사용자가 요청한 지속시간을 반영할지 판단한다. 이동 스킬(MOVE_SKILLS)에만
    적용되고 상한 없이 그대로 쓰인다(2026-08-14, 사용자 요청으로 상한 제거) — 0
    이하 요청만 MIN_MOVE_DURATION_SEC로 막는다. 이동 외 스킬(나중에 다시
    추가될 경우)은 요청값을 무시하고 SKILL_MAP 고정값을 그대로 쓴다."""
    default = SKILL_MAP[skill]['duration_sec']
    if skill not in MOVE_SKILLS or requested_duration_sec is None:
        return default
    return max(MIN_MOVE_DURATION_SEC, requested_duration_sec)


def resolve_timeout_sec(skill: str, duration_sec: float) -> float:
    """실제로 쓰일 duration_sec(위에서 클램프된 값)에 맞춰 바깥쪽 안전장치 timeout을
    다시 계산한다 — SKILL_MAP의 고정 timeout_sec는 고정 duration_sec 기준이라,
    duration이 달라지면 timeout도 같이 늘어나야 한다(안 그러면 늘어난 duration이
    끝나기도 전에 outer timeout이 먼저 죽여버림). SKILL_MAP이 애초에 여유 없이
    "연결 오버헤드 + duration"으로만 정의돼 있어(그 자체가 이미 충분한 여유,
    2026-08-11 실측) 별도 마진을 더 얹지 않는다 — 기본 duration을 넣으면 SKILL_MAP의
    고정 timeout_sec와 정확히 같은 값이 나와야 한다(회귀 테스트로 확인됨)."""
    if skill not in MOVE_SKILLS:
        return SKILL_MAP[skill]['timeout_sec']
    return _CONNECT_OVERHEAD_SEC + duration_sec


def parse_command(raw: str) -> tuple[str, float | None]:
    """'/lekiwi_command' 페이로드를 스킬 이름과 선택적 지속시간(초)으로 나눈다.
    'forward' 또는 'forward:7.5' 형식을 받는다 — 콜론 뒤가 숫자로 파싱 안 되면
    지속시간 없이 스킬만 요청한 것으로 방어적으로 취급한다."""
    skill, _, duration_part = raw.strip().partition(':')
    if not duration_part:
        return skill, None
    try:
        return skill, float(duration_part)
    except ValueError:
        return skill, None


def build_launch_command(skill: str, duration_sec: float | None = None) -> str:
    """SKILL_MAP을 참고해 원격에서 백그라운드로 실행하고 PID를 표준출력에 남기는
    커맨드 문자열을 만든다. setsid+nohup으로 SSH 세션이 끝나도 원격 프로세스가
    살아남게 하고(companion_bridge_node.py에서 실전 검증된 패턴), timeout으로
    스크립트 내부 안전장치 유무와 무관하게 상한을 강제한다. 로그는 /tmp에 남긴다.

    `duration`(스크립트에 넘기는 실제 이동 시간)과 `timeout`(바깥쪽 안전장치,
    연결 오버헤드를 포함해 duration보다 커야 함)을 분리해서 채운다 — 합치면
    연결이 끝나기 전에 outer timeout이 먼저 죽여버리는 버그가 남(위 모듈
    docstring 2026-08-11 항목 참고).

    `duration_sec`을 넘기면(사용자가 지속시간을 직접 요청한 경우) resolve_duration_sec로
    클램프해 반영한다 — 안 넘기면(기본값 None) 예전과 동일하게 SKILL_MAP 고정값을 쓴다."""
    entry = SKILL_MAP[skill]
    resolved_duration = resolve_duration_sec(skill, duration_sec)
    duration = int(resolved_duration)
    timeout = int(resolve_timeout_sec(skill, resolved_duration))
    inner = entry['command_template'].format(duration=duration)
    log_path = f'/tmp/lekiwi_skill_{skill}.log'
    return f'setsid nohup timeout {timeout}s {inner} >{log_path} 2>&1 </dev/null & echo $!'


def parse_pid(ssh_stdout: str) -> int:
    """SSH stdout의 마지막 줄(echo $!가 출력한 PID)을 파싱한다. 실패 시 ValueError."""
    lines = [line for line in ssh_stdout.strip().splitlines() if line.strip()]
    if not lines or not lines[-1].strip().isdigit():
        raise ValueError(f'PID를 파싱할 수 없음: {ssh_stdout!r}')
    return int(lines[-1].strip())


def build_alive_check_command(pid: int) -> str:
    """생존 여부를 stdout으로 명시적으로 보고한다(ALIVE/DEAD) — SSH 자체가 실패한
    경우(None 반환)와 "진짜 죽음"을 구분하기 위해서다. exit code만 보면 SSH 연결
    실패(네트워크 순단 등)도 "죽었음"으로 오판해 active_job을 잘못 풀어버릴 수 있다."""
    return f'kill -0 {pid} 2>/dev/null && echo ALIVE || echo DEAD'


def build_stop_command(pid: int, grace_sec: float) -> str:
    """SIGINT로 먼저 정중히 종료 요청 후 grace_sec 대기, 그래도 살아있으면 SIGKILL —
    companion_bridge_node.py의 _stop_laptop_nodes와 동일 원리(그쪽은 이름 기반
    pkill, 여기는 실행 직후 캡처한 정확한 PID 기반이라 동명 프로세스 오살 위험이 없다).
    setsid로 launch했으므로 pid==pgid==sid — 반드시 프로세스 그룹 전체(-pid)에
    신호를 보내야 한다: timeout이 실제 작업을 위해 fork한 자식 프로세스는 timeout
    자신과 다른 PID를 가지므로, 단일 PID에만 신호를 보내면 timeout 래퍼만 죽고
    실제 작업 프로세스는 고아로 남아 계속 실행된다(실측으로 확인된 버그).
    SIGINT로 이미 정상 종료된 경우(대부분)엔 뒤이은 kill -9가 "No such process"로
    실패하는 게 정상 흐름이라 stderr를 억제하고, 호출부(lekiwi_command_node.py)가
    이 결과를 성공/실패 판정에 쓰지 않는 fire-and-forget 전제를 커맨드 자체의
    종료코드에도 정직하게 반영해 마지막에 true를 붙여 항상 0으로 끝나게 한다."""
    return (f'kill -INT -{pid} 2>/dev/null ; sleep {grace_sec} ; '
            f'kill -9 -{pid} 2>/dev/null ; true')
