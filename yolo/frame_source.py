"""카메라 프레임을 어디서 받을지 한 군데서 결정한다.

local   노트북에 꽂은 USB 웹캠 (cv2.VideoCapture(index))
lekiwi  라즈베리파이의 lekiwi_host 가 ZMQ 로 뿌리는 관측 스트림

둘 다 cv2.VideoCapture 와 같은 인터페이스(read/release/isOpened)를 노출하므로
스크립트 쪽은 소스 종류를 신경 쓰지 않아도 된다.
"""
import base64
import json
import time

import numpy as np
import cv2

DEFAULT_REMOTE_IP = "10.42.0.61"
DEFAULT_PORT_OBSERVATIONS = 5556

# 특정 카메라를 고르지 않고, host 가 발행하는 아무거나 받고 싶을 때 cam_name 으로 전달.
ANY_CAMERA = "*"

HOST_CMD = (
    "ssh roboseasy@10.42.0.61\n"
    "    ./start_lekiwi_host.sh"
)


class FrameSourceError(RuntimeError):
    """소스를 열지 못했을 때. 메시지는 그대로 사용자에게 보여준다."""


class LocalCamera:
    """로컬 장치 인덱스의 cv2.VideoCapture."""

    def __init__(self, index, width=None, height=None):
        self.index = int(index)
        self.cap = cv2.VideoCapture(self.index)
        if not self.cap.isOpened():
            raise FrameSourceError(
                f"카메라를 열 수 없습니다 (index={self.index}).\n"
                "  `ls /dev/video*` 로 장치를 확인하세요."
            )
        if width:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
        if height:
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))

    def read(self):
        return self.cap.read()

    def isOpened(self):
        return self.cap.isOpened()

    def release(self):
        self.cap.release()

    def describe(self):
        return f"local camera index={self.index}"

    @property
    def fps(self):
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        return fps if fps and fps > 1e-2 else None


class LeKiwiStream:
    """LeKiwi host 의 ZMQ 관측 스트림에서 프레임(과 팔 상태)을 꺼낸다.

    lekiwi_host 는 제어 주기마다 JSON 한 덩어리를 PUSH 소켓으로 보낸다.
    float 값은 팔/베이스 상태이고, 문자열 값은 카메라 이름("front", "wrist")을
    키로 하는 base64 JPEG 이다.

    PUSH/PULL + CONFLATE 라서 host 가 붙어 있는 소비자들에게 프레임을 번갈아
    나눠준다. 한 host 에는 클라이언트를 하나만 붙일 것.

    색은 기본적으로 채널이 뒤집혀서 온다. lerobot 의 OpenCVCameraConfig 기본값이
    color_mode=RGB 라 host 가 RGB 배열을 그대로 cv2.imencode(BGR 가정)에 넣기
    때문이다. host_color_mode 로 되돌린다 (보라가 분홍으로, 노랑이 파랑으로
    보이면 이 값이 틀린 것).
    """

    def __init__(self, remote_ip=DEFAULT_REMOTE_IP, cam_name="front",
                 port=DEFAULT_PORT_OBSERVATIONS, connect_timeout_s=10.0,
                 read_timeout_s=2.0, host_color_mode="rgb"):
        self.host_color_mode = str(host_color_mode).lower()
        if self.host_color_mode not in ("rgb", "bgr"):
            raise FrameSourceError(
                f"host_color_mode 는 'rgb' 또는 'bgr' 이어야 합니다: {host_color_mode!r}")
        try:
            import zmq
        except ImportError as e:
            raise FrameSourceError(
                "lekiwi 소스에는 pyzmq 가 필요합니다.\n  pip install pyzmq"
            ) from e

        self._zmq = zmq
        self.cam_name = str(cam_name)
        self.address = f"tcp://{remote_ip}:{int(port)}"
        self.read_timeout_s = float(read_timeout_s)
        self.state = {}          # 이미지가 아닌 값들 (팔 관절, 베이스 속도)
        self.camera_names = []

        self._ctx = zmq.Context()
        self._sock = self._ctx.socket(zmq.PULL)
        self._sock.setsockopt(zmq.CONFLATE, 1)
        self._sock.connect(self.address)
        self._poller = zmq.Poller()
        self._poller.register(self._sock, zmq.POLLIN)

        obs = self._recv(float(connect_timeout_s) * 1000.0)
        if obs is None:
            self.release()
            raise FrameSourceError(
                f"{self.address} 의 LeKiwi host 에서 {connect_timeout_s:g}초 안에 "
                "관측이 오지 않았습니다.\n"
                "  로봇에 접속해 host 를 띄우고 그대로 두세요:\n"
                f"    {HOST_CMD}\n"
                "  다른 클라이언트(teleop, record, 다른 스크립트)가 같은 스트림을\n"
                "  이미 빨아가고 있지는 않은지도 확인하세요 — 프레임을 나눠 가집니다."
            )

        self.camera_names = sorted(k for k, v in obs.items() if isinstance(v, str))
        if not self.camera_names:
            self.release()
            raise FrameSourceError(
                f"{self.address} 의 host 가 카메라를 발행하지 않습니다.\n"
                "  --robot.cameras=... 로 띄웠는지, 파이가 /dev/video* 를 열 수 있는지 확인하세요."
            )
        if self.cam_name == ANY_CAMERA:
            self.cam_name = self.camera_names[0]
        elif self.cam_name not in self.camera_names:
            available = self.camera_names
            self.release()
            raise FrameSourceError(
                f"'{self.cam_name}' 카메라가 스트림에 없습니다.\n"
                f"  사용 가능: {available}"
            )

    def use_camera(self, cam_name):
        """다른 카메라로 바꾼다. 관측 하나에 모든 카메라가 실려 오므로 재접속은 없다."""
        name = str(cam_name)
        if name not in self.camera_names:
            raise FrameSourceError(
                f"'{name}' 카메라가 스트림에 없습니다. 사용 가능: {self.camera_names}")
        self.cam_name = name

    def _recv(self, timeout_ms):
        """가장 최신 관측 dict, 아무것도 안 오면 None."""
        try:
            socks = dict(self._poller.poll(timeout_ms))
        except self._zmq.ZMQError:
            return None
        if self._sock not in socks:
            return None

        latest = None
        while True:
            try:
                latest = self._sock.recv_string(self._zmq.NOBLOCK)
            except self._zmq.Again:
                break
        if latest is None:
            return None
        try:
            return json.loads(latest)
        except json.JSONDecodeError:
            return None

    def read(self):
        """read_timeout_s 까지 새 프레임을 기다린다. cv2 처럼 (ok, frame)."""
        deadline = time.time() + self.read_timeout_s
        while True:
            remaining_ms = max((deadline - time.time()) * 1000.0, 0.0)
            obs = self._recv(remaining_ms)
            if obs is not None:
                self.state = {k: v for k, v in obs.items() if not isinstance(v, str)}
                encoded = obs.get(self.cam_name)
                if encoded:
                    buf = np.frombuffer(base64.b64decode(encoded), dtype=np.uint8)
                    frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
                    if frame is not None:
                        if self.host_color_mode == "rgb":
                            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                        return True, frame
            if time.time() >= deadline:
                return False, None

    def isOpened(self):
        return True

    def release(self):
        try:
            self._sock.close(linger=0)
            self._ctx.term()
        except Exception:
            pass

    def describe(self):
        return f"lekiwi '{self.cam_name}' @ {self.address}"

    @property
    def fps(self):
        return None          # host 가 알려주지 않는다. 필요하면 직접 측정할 것.
