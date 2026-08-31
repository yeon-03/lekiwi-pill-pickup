#!/usr/bin/env python
"""LeKiwi 연결을 혼자 계속 들고 있으면서, 다른 스크립트들에게는 로컬 소켓으로
명령 창구를 열어주는 상시 서버.

배경: LeKiwiClient는 연결을 하나만 허용해서, 지금까지는 베이스캠(front)을
보는 스크립트와 팔/베이스를 움직이는 스크립트를 동시에 못 띄웠다. 이 서버가
연결을 계속 물고 있으면서 ①front/wrist 카메라를 계속 받아 공유 파일로
갱신해두고(그래서 아무 스크립트나 그 파일만 읽으면 됨) ②다른 스크립트가
robot_link.py(RobotLink)를 통해 보내는 "이 액션 좀 대신 보내줘" 요청을
그때그때 처리해준다.

실행: python scripts/robot_bridge.py --lekiwi-host 192.168.0.201
      (터미널 하나에 계속 띄워두고, base_nudge.py/align_and_grasp.py 등
      나머지 스크립트는 평소처럼 그대로 실행하면 됨 — 알아서 이 브릿지를
      찾아 쓴다)
종료: Ctrl+C — 정지 명령을 보내고 연결 해제까지 하고 끝난다."""
import argparse
import json
import os
import socketserver
import sys
import threading
import time
from pathlib import Path

import cv2

from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig

SOCK_PATH = '/tmp/lekiwi_bridge.sock'
FRAME_DIR = Path('/dev/shm/lekiwi_cam')
POLL_INTERVAL_SEC = 0.05  # 기존 base_nudge 등의 제어 주기(20Hz)와 동일

ARM_KEYS = [
    'arm_shoulder_pan.pos', 'arm_shoulder_lift.pos', 'arm_elbow_flex.pos',
    'arm_wrist_flex.pos', 'arm_wrist_roll.pos', 'arm_gripper.pos',
]

_client = None
# get_observation/send_action은 poll_loop 스레드와 소켓 핸들러 스레드가 동시에
# 부를 수 있어서, LeKiwiClient 하나를 여러 스레드가 동시에 건드리지 않게 잠근다.
_lock = threading.Lock()


def _atomic_write_bytes(path: Path, data: bytes) -> None:
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_bytes(data)
    os.replace(tmp, path)


def _atomic_write_json(path: Path, obj: dict) -> None:
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(obj))
    os.replace(tmp, path)


# 베이스캠은 로봇에 거꾸로 달려 있다(2026-08-27 실측: 판과 병이 화면 위쪽에
# 매달린 것처럼 찍힘). 여기서 한 번만 바로잡아야 뷰어·검출·제어가 전부 같은
# 방향을 본다. 안 그러면 화면의 왼쪽이 로봇의 오른쪽이 되어 좌우/회전 부호가
# 전부 반대로 동작한다.
CAMERA_ROTATE_180 = {'front': True, 'wrist': False}


def _publish_frame(name: str, rgb_array) -> None:
    bgr = cv2.cvtColor(rgb_array, cv2.COLOR_RGB2BGR)
    if CAMERA_ROTATE_180.get(name):
        bgr = cv2.rotate(bgr, cv2.ROTATE_180)
    ok, buf = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 85])
    if ok:
        _atomic_write_bytes(FRAME_DIR / f'{name}.jpg', buf.tobytes())


def poll_loop(stop_event: threading.Event) -> None:
    """카메라 프레임 + 관절 상태를 계속 받아서 공유 파일로 갱신만 한다
    (베이스/팔을 직접 움직이지는 않음 — 그건 오직 소켓 핸들러의 send_action
    요청을 통해서만 일어난다)."""
    while not stop_event.is_set():
        with _lock:
            obs = _client.get_observation()
        positions = {k: float(v) for k, v in obs.items() if k.endswith('.pos')}
        for cam in ('front', 'wrist'):
            if cam in obs:
                _publish_frame(cam, obs[cam])
        _atomic_write_json(FRAME_DIR / 'bridge_state.json',
                            {'ts': time.time(), 'positions': positions})
        time.sleep(POLL_INTERVAL_SEC)


class Handler(socketserver.BaseRequestHandler):
    def handle(self) -> None:
        data = b''
        while not data.endswith(b'\n'):
            chunk = self.request.recv(65536)
            if not chunk:
                break
            data += chunk
        try:
            req = json.loads(data.decode())
            reply = self._dispatch(req)
        except Exception as e:
            reply = {'ok': False, 'error': f'{type(e).__name__}: {e}'}
        self.request.sendall((json.dumps(reply) + '\n').encode())

    def _dispatch(self, req: dict) -> dict:
        cmd = req.get('cmd')
        if cmd == 'get_observation':
            with _lock:
                obs = _client.get_observation()
            positions = {k: float(v) for k, v in obs.items() if k.endswith('.pos')}
            return {'ok': True, 'obs': positions}
        if cmd == 'send_action':
            with _lock:
                _client.send_action(req['action'])
            return {'ok': True}
        return {'ok': False, 'error': f'알 수 없는 명령: {cmd}'}


class UnixServer(socketserver.ThreadingMixIn, socketserver.UnixStreamServer):
    daemon_threads = True


def stop_robot() -> None:
    try:
        with _lock:
            obs = _client.get_observation()
            action = {k: obs[k] for k in ARM_KEYS}
            action['x.vel'] = 0.0
            action['y.vel'] = 0.0
            action['theta.vel'] = 0.0
            _client.send_action(action)
    except Exception as e:
        print(f'정지 명령 실패(무시): {e}', file=sys.stderr)


def main() -> None:
    global _client
    p = argparse.ArgumentParser()
    p.add_argument('--lekiwi-host', default='192.168.0.201')
    args = p.parse_args()

    FRAME_DIR.mkdir(parents=True, exist_ok=True)
    if os.path.exists(SOCK_PATH):
        os.remove(SOCK_PATH)

    print(f'{args.lekiwi_host} 연결 중...')
    _client = LeKiwiClient(LeKiwiClientConfig(remote_ip=args.lekiwi_host))
    _client.connect()
    print('연결됨 — 이제 base_nudge.py/align_and_grasp.py 등 나머지 스크립트를 '
          '평소처럼 그대로 실행하면 이 브릿지를 통해 동작합니다')

    stop_event = threading.Event()
    poll_thread = threading.Thread(target=poll_loop, args=(stop_event,), daemon=True)
    poll_thread.start()

    server = UnixServer(SOCK_PATH, Handler)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    print(f'소켓 서버 시작: {SOCK_PATH} (Ctrl+C로 종료)')

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print('\n종료 중...')
    finally:
        stop_event.set()
        poll_thread.join(timeout=2.0)
        stop_robot()
        server.shutdown()
        _client.disconnect()
        try:
            os.remove(SOCK_PATH)
        except OSError:
            pass
        print('정리 완료')


if __name__ == '__main__':
    main()
