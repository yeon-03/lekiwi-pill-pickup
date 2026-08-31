#!/usr/bin/env python
"""LeKiwiClient를 직접 여는 대신, robot_bridge.py가 떠 있으면 그 브릿지에
소켓으로 명령만 넘기는 얇은 창구.

문제: LeKiwiClient는 로봇 하나당 연결을 딱 하나만 허용한다(2026-08-25
세션에서 실측 확인). 그래서 지금까지는 카메라를 보는 스크립트와 팔/베이스를
움직이는 스크립트를 절대 동시에 못 띄웠다 — 정렬(align_and_grasp.py)을
돌릴 때마다 보고 있던 화면을 꺼야 했다.

해결: robot_bridge.py가 연결을 혼자 계속 들고 있고, 이 파일(RobotLink)이
LeKiwiClient와 완전히 똑같은 4개 메서드(connect/get_observation/
send_action/disconnect)로 보이는 창구 역할을 한다. 그래서 기존 스크립트는
"LeKiwiClient(LeKiwiClientConfig(remote_ip=...))"를 "RobotLink(...)"로
바꾸기만 하면 나머지 코드는 한 줄도 안 고쳐도 된다. 브릿지가 안 떠 있으면
지금까지처럼 직접 연결로 자동 폴백하므로 브릿지 없이 단독 실행도 그대로 된다."""
import json
import os
import socket
import time
from pathlib import Path

import cv2

BRIDGE_SOCK_PATH = '/tmp/lekiwi_bridge.sock'
FRAME_DIR = Path('/dev/shm/lekiwi_cam')
# 이보다 오래된 하트비트(bridge_state.json)면 브릿지가 죽은 걸로 보고
# 직접연결로 폴백한다 — 비정상 종료로 소켓 파일만 남아있는 경우 대비.
BRIDGE_STALE_SEC = 2.0


def bridge_available() -> bool:
    if not os.path.exists(BRIDGE_SOCK_PATH):
        return False
    heartbeat = FRAME_DIR / 'bridge_state.json'
    try:
        return (time.time() - heartbeat.stat().st_mtime) < BRIDGE_STALE_SEC
    except OSError:
        return False


def _rpc(request: dict, timeout: float = 5.0) -> dict:
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.settimeout(timeout)
    try:
        sock.connect(BRIDGE_SOCK_PATH)
        sock.sendall((json.dumps(request) + '\n').encode())
        chunks = []
        while True:
            chunk = sock.recv(65536)
            if not chunk:
                break
            chunks.append(chunk)
            if chunks[-1].endswith(b'\n'):
                break
        reply = json.loads(b''.join(chunks).decode())
    finally:
        sock.close()
    if not reply.get('ok', False):
        raise RuntimeError(f"robot_bridge 오류: {reply.get('error')}")
    return reply


def _read_frame(name: str):
    """브릿지가 써둔 카메라 프레임을 읽어 RGB로 반환(obs['front']/obs['wrist']와
    같은 형식) — 원본 스크립트들이 cv2.cvtColor(obs[cam], COLOR_RGB2BGR)로
    다시 변환해서 쓰므로 맞춰줘야 한다. 파일이 없으면(예: 손목캠 미장착) None."""
    path = FRAME_DIR / f'{name}.jpg'
    if not path.exists():
        return None
    img_bgr = cv2.imread(str(path))
    if img_bgr is None:
        return None
    return cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)


class RobotLink:
    """LeKiwiClient와 같은 인터페이스(connect/get_observation/send_action/
    disconnect). 브릿지가 떠 있으면 소켓 경유, 없으면 지금까지처럼 직접연결."""

    def __init__(self, lekiwi_host: str):
        self._host = lekiwi_host
        self._direct = None
        self._use_bridge = False

    def connect(self) -> None:
        if bridge_available():
            self._use_bridge = True
            print('[robot_link] robot_bridge.py 감지 — 소켓으로 명령만 전달 '
                  '(연결은 브릿지가 계속 유지)')
            return
        from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig
        print('[robot_link] robot_bridge.py 없음 — 직접 연결')
        self._direct = LeKiwiClient(LeKiwiClientConfig(remote_ip=self._host))
        self._direct.connect()

    def get_observation(self) -> dict:
        if self._use_bridge:
            reply = _rpc({'cmd': 'get_observation'})
            obs = reply['obs']
            for cam in ('front', 'wrist'):
                frame = _read_frame(cam)
                if frame is not None:
                    obs[cam] = frame
            return obs
        return self._direct.get_observation()

    def send_action(self, action: dict) -> None:
        if self._use_bridge:
            _rpc({'cmd': 'send_action', 'action': action})
            return
        self._direct.send_action(action)

    def disconnect(self) -> None:
        if self._use_bridge:
            return  # 연결은 브릿지 소유 — 여기서 끊지 않는다
        if self._direct is not None:
            self._direct.disconnect()


if __name__ == '__main__':
    print('브릿지 감지:', '있음 (소켓 경유로 동작함)' if bridge_available()
          else '없음 (모든 스크립트가 직접연결로 폴백함)')
