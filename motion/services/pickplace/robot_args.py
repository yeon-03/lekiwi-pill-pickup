"""라즈베리파이의 lekiwi_host 에 붙기 위한 연결 설정.

필드명은 원본 CLI(`lekiwi_yolo_view.py`)의 `LeKiwiRobotArgs` 와 맞춰
`--robot.remote_ip=... --robot.id=...` 인자 스타일이 그대로 통하게 한다.
`cameras` 선언은 두지 않는다 — Physical Labs GUI 앱(`pick_worker.py`)도
`LeKiwiClientConfig` 생성 시 넘기지 않고 동작을 확인했다(관측은 ZMQ 페이로드의
키 이름으로 그대로 들어온다). `lerobot` import 는 `to_config()` 안에서만 해서,
이 파일의 나머지 부분은 `lerobot` 없이도 테스트할 수 있게 한다.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class LeKiwiRobotArgs:
    remote_ip: str = "192.168.0.201"
    id: str = "lekiwi01"
    port_zmq_cmd: int = 5555
    port_zmq_observations: int = 5556
    connect_timeout_s: int = 5

    def to_config(self):
        from lerobot.robots.lekiwi import LeKiwiClientConfig

        return LeKiwiClientConfig(
            remote_ip=self.remote_ip,
            id=self.id,
            port_zmq_cmd=self.port_zmq_cmd,
            port_zmq_observations=self.port_zmq_observations,
            connect_timeout_s=self.connect_timeout_s,
        )
