"""르로봇(LeKiwi) 스킬 원격 트리거 노드 — /lekiwi_command를 구독해 SSH로 LeKiwi 자체
Pi에 이미 있는 이동 스크립트(forward/backward/left/right)를 실행시킨다.
에이보는 조종기가 아니라 방아쇠 역할만 한다(설계 배경:
docs/superpowers/specs/2026-08-10-lekiwi-command-node-design.md).

SSH 호출(subprocess.run)이 ROS2 콜백을 블로킹하면 안 된다 — companion_bridge_node.py는
FastAPI/uvicorn의 asyncio 이벤트루프 위에서 asyncio.to_thread를 쓰지만, 이 노드는
평범한 rclpy 노드(이벤트루프 없음)라 concurrent.futures.ThreadPoolExecutor로 같은
효과(콜백 즉시 리턴, SSH는 백그라운드 스레드)를 낸다.

동시성: self.active_job 읽기/쓰기는 전부 self._job_lock 안에서만 한다. 특히 "새 skill을
시작해도 되는지" 판정과 "그 자리를 예약"하는 것은 반드시 원자적으로(같은 락 안에서)
해야 한다 — 판정과 예약 사이에 SSH 왕복(수 초)이 끼면 그 사이 다른 요청이 같은 판정
결과를 보고 동시에 시작을 시도할 수 있다(설계 중 발견한 레이스, 반드시 지킬 것).
"""
import os
import subprocess
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import rclpy
from dotenv import load_dotenv
from rclpy.node import Node
from std_msgs.msg import String

from .lekiwi_control import (
    ActiveJob,
    MOVE_SKILLS,
    SKILL_MAP,
    STOP_SKILL,
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

load_dotenv()

# companion_bridge_node.py의 LAPTOP_SSH_HOST_DEFAULT와 동일 패턴 — .env의
# LEKIWI_PI_HOST=lekiwi(사용자 ~/.ssh/config의 Host alias, HostName/User/IdentityFile을
# 그 파일이 이미 들고 있어 ssh_key_path 파라미터는 따로 안 채워도 됨)를 기본값으로 쓴다.
LEKIWI_PI_HOST_DEFAULT = os.environ.get('LEKIWI_PI_HOST', '')

SSH_CONNECT_TIMEOUT_SEC = 5
SSH_SUBPROCESS_TIMEOUT_SEC = 20   # stop 커맨드 안에 grace sleep이 포함되므로 여유있게
ALIVE_CHECK_DELAY_SEC = 1.5      # 착수 직후 생존 확인까지 대기(즉시 크래시 탐지용)
EXPIRY_CHECK_INTERVAL_SEC = 1.0
EXPIRY_MARGIN_SEC = 5.0   # SSH 왕복+alive-check 지연을 감안한 여유
# 이동 스킬끼리는 순서대로 이어서 실행할 수 있다(2026-08-14) — "뒤로 갔다가
# 회전해줘"처럼 한 turn에 LLM이 run_lekiwi_skill을 여러 번 부르면, 예전엔 두
# 번째 호출이 "이미 실행 중"으로 거절돼 LLM이 말로만 약속하고 실제로는 하나만
# 실행되는 문제가 있었다(실기기 테스트로 발견). 무한정 쌓이는 걸 막는 상한.
MAX_MOVE_QUEUE_LEN = 5


def _ssh_run(host: str, key_path: str, remote_cmd: str) -> subprocess.CompletedProcess | None:
    """companion_bridge_node.py의 _ssh_run()과 동일한 3중 방어(BatchMode/ConnectTimeout/
    subprocess timeout) — 실패 시 None을 반환하고 예외를 던지지 않는다."""
    args = ['ssh', '-o', 'BatchMode=yes',
            '-o', f'ConnectTimeout={SSH_CONNECT_TIMEOUT_SEC}',
            '-o', 'StrictHostKeyChecking=accept-new']
    if key_path:
        args += ['-i', key_path]
    args += [host, remote_cmd]
    try:
        result = subprocess.run(
            args, capture_output=True, text=True, timeout=SSH_SUBPROCESS_TIMEOUT_SEC)
    except (subprocess.TimeoutExpired, OSError):
        return None
    return result if result.returncode == 0 else None


class LekiwiCommandNode(Node):
    def __init__(self):
        super().__init__('lekiwi_command_node')
        self.declare_parameter('lekiwi_pi_host', LEKIWI_PI_HOST_DEFAULT)
        self.declare_parameter('laptop_ssh_host', '')
        self.declare_parameter('ssh_key_path', '')
        self.declare_parameter('stop_grace_sec', 3.0)
        self.lekiwi_pi_host = self.get_parameter(
            'lekiwi_pi_host').get_parameter_value().string_value
        self.laptop_ssh_host = self.get_parameter(
            'laptop_ssh_host').get_parameter_value().string_value
        self.ssh_key_path = self.get_parameter(
            'ssh_key_path').get_parameter_value().string_value
        self.stop_grace_sec = self.get_parameter(
            'stop_grace_sec').get_parameter_value().double_value

        self.active_job: ActiveJob | None = None
        # 이동 스킬 전용 대기열 — (skill, requested_duration_sec) 튜플의 FIFO.
        # active_job이 있을 때만 채워지고, _clear_if_current가 매번 비우고 나면
        # 자동으로 다음 항목을 시작한다(불변조건: active_job이 None이면 이 큐도
        # 항상 비어있다).
        self._move_queue: list[tuple[str, float | None]] = []
        self._job_lock = threading.Lock()
        self._executor = ThreadPoolExecutor(max_workers=2)

        self.create_subscription(String, '/lekiwi_command', self._on_command, 10)
        self.create_timer(EXPIRY_CHECK_INTERVAL_SEC, self._check_expiry)
        self.get_logger().info('lekiwi_command_node 시작')

    def _host_for(self, skill: str) -> str:
        host_param = SKILL_MAP[skill]['host_param']
        return self.lekiwi_pi_host if host_param == 'lekiwi_pi_host' else self.laptop_ssh_host

    def _on_command(self, msg: String) -> None:
        # 'forward' 또는 'forward:7.5'(사용자가 지속시간을 요청한 경우, 2026-08-14)
        # 형식을 받는다 — dialogue_node가 인코딩하고 parse_command가 대칭으로 푼다.
        skill, requested_duration = parse_command(msg.data)
        if not is_known_skill(skill):
            self.get_logger().warning(f'알 수 없는 skill 무시: {skill!r}')
            return

        with self._job_lock:
            if skill == STOP_SKILL:
                self._move_queue.clear()   # 대기 중이던 다음 동작들도 전부 취소
                job = self.active_job
                if job is None:
                    self.get_logger().info('정지 요청 — 실행 중인 작업 없음(no-op)')
                    return
                if job.pid is None:
                    job.pending_stop = True
                    self.get_logger().info(f'{job.skill} 착수 중이라 착수 완료 후 정지 예정')
                    return
                self._executor.submit(self._handle_stop, job)
                return

            if not can_start_skill(self.active_job):
                # 이동 스킬끼리는 거절하지 않고 대기열에 넣어 순서대로 이어서
                # 실행한다(2026-08-14) — 이동 외 스킬이 섞이면(나중에 다시 추가될
                # 경우) 순서 보장이 애매해지므로 그 경우만 기존처럼 거절한다.
                if skill in MOVE_SKILLS and self.active_job.skill in MOVE_SKILLS:
                    if len(self._move_queue) >= MAX_MOVE_QUEUE_LEN:
                        self.get_logger().warning(
                            f'대기열이 가득 차({MAX_MOVE_QUEUE_LEN}개) {skill} 요청을 거절함')
                        return
                    if not self._host_for(skill):
                        self.get_logger().error(
                            f"{SKILL_MAP[skill]['host_param']} 파라미터가 비어있음")
                        return
                    self._move_queue.append((skill, requested_duration))
                    self.get_logger().info(f'{skill} 대기열에 추가(대기 {len(self._move_queue)}개)')
                    return
                self.get_logger().warning(
                    f'이미 {self.active_job.skill} 실행 중이라 {skill} 요청을 거절함')
                return
            job = self._build_job(skill, requested_duration)
            if job is None:
                return
            self.active_job = job   # SSH 왕복 전에 즉시 예약 — 레이스 방지(위 클래스 docstring 참고)

        self._executor.submit(self._handle_start, job)

    def _build_job(self, skill: str, requested_duration: float | None) -> 'ActiveJob | None':
        """host 파라미터가 비어있으면 None을 반환한다 — 호출부가 이미 self._job_lock
        안에 있어야 한다. 클램프까지 끝난 실제 duration/timeout을 여기서 미리 확정해
        ActiveJob에 담아둔다 — _handle_start(SSH 왕복 이후)와 _check_expiry가 매번
        다시 계산하지 않고 이 값을 그대로 참조한다."""
        host = self._host_for(skill)
        if not host:
            self.get_logger().error(f"{SKILL_MAP[skill]['host_param']} 파라미터가 비어있음")
            return None
        duration_sec = resolve_duration_sec(skill, requested_duration)
        timeout_sec = resolve_timeout_sec(skill, duration_sec)
        return ActiveJob(skill=skill, host=host, started_at=time.time(),
                          duration_sec=duration_sec, timeout_sec=timeout_sec)

    def _handle_start(self, job: ActiveJob) -> None:
        launch_cmd = build_launch_command(job.skill, job.duration_sec)
        result = _ssh_run(job.host, self.ssh_key_path, launch_cmd)
        if result is None:
            self.get_logger().error(f'{job.skill} 실행 SSH 실패 (host={job.host})')
            self._clear_if_current(job)
            return
        try:
            pid = parse_pid(result.stdout)
        except ValueError as e:
            self.get_logger().error(f'{job.skill} PID 파싱 실패: {e}')
            self._clear_if_current(job)
            return

        with self._job_lock:
            job.pid = pid
            pending_stop = job.pending_stop
        self.get_logger().info(f'{job.skill} 시작 (host={job.host}, pid={pid})')

        if pending_stop:
            self._handle_stop(job)
            return

        time.sleep(ALIVE_CHECK_DELAY_SEC)
        alive = _ssh_run(job.host, self.ssh_key_path, build_alive_check_command(pid))
        if alive is not None and 'DEAD' in alive.stdout:
            self.get_logger().warning(f'{job.skill}(pid={pid})이 시작 직후 종료된 것으로 보임')
            self._clear_if_current(job)
        elif alive is None:
            self.get_logger().warning(
                f'{job.skill}(pid={pid}) 생존 확인 SSH 실패(네트워크 문제 가능) — '
                f'실제로 죽었는지 불확실하니 active_job은 유지함')

    def _handle_stop(self, job: ActiveJob) -> None:
        _ssh_run(job.host, self.ssh_key_path, build_stop_command(job.pid, self.stop_grace_sec))
        self.get_logger().info(f'{job.skill}(pid={job.pid}) 정지 완료')
        self._clear_if_current(job)

    def _clear_if_current(self, job: ActiveJob) -> None:
        """작업을 해제하고, 대기열에 다음 이동 스킬이 있으면 이어서 시작한다
        (2026-08-14) — 여기 한 곳에서만 처리해야 _handle_start/_handle_stop/
        _check_expiry의 모든 종료 경로가 빠짐없이 대기열을 소비한다."""
        next_job: ActiveJob | None = None
        with self._job_lock:
            if self.active_job is not job:
                return
            self.active_job = None
            while self._move_queue:
                next_skill, next_duration = self._move_queue.pop(0)
                next_job = self._build_job(next_skill, next_duration)
                if next_job is not None:
                    self.active_job = next_job
                    break
                # host 파라미터가 그 사이 비워진 것 같은 드문 경우 — 이 항목만
                # 버리고 대기열의 다음 항목으로 계속 시도한다.
        if next_job is not None:
            self.get_logger().info(f'대기열에서 {next_job.skill} 시작')
            self._executor.submit(self._handle_start, next_job)

    def _check_expiry(self) -> None:
        """원격 커맨드는 이미 timeout {t}s로 자체 종료되지만, 이 노드는 그 사실을 스스로
        알 방법이 없다 — 그래서 여유(EXPIRY_MARGIN_SEC)를 두고 주기적으로 확인해 active_job을
        해제한다. 확인 안 하면 성공적으로 끝난 스킬 뒤로 모든 새 명령이 영구 거절된다."""
        with self._job_lock:
            job = self.active_job
            if job is None or job.pid is None:
                return
            timeout_sec = (job.timeout_sec if job.timeout_sec is not None
                           else SKILL_MAP[job.skill]['timeout_sec'])
            deadline = job.started_at + timeout_sec + EXPIRY_MARGIN_SEC
            if time.time() < deadline:
                return
            expired = job
        self.get_logger().info(f'{expired.skill}(pid={expired.pid}) 자체 timeout 경과로 만료 처리')
        self._clear_if_current(expired)


def main(args=None):
    rclpy.init(args=args)
    node = LekiwiCommandNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
