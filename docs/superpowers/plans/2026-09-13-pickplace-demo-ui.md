# Pick&Place 시연용 웹 UI Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 터미널 CLI 플래그 대신 브라우저에서 클릭으로 YOLO pick&place 를 시작/일시정지/정지/비상정지하고, front/wrist 카메라 영상을 볼 수 있는 로컬 웹 UI를 만든다.

**Architecture:** 기존에 실물 검증된 순수 로직(`motion/services/pickplace/`: Approacher/ArmSequencer/WristServo/GraspChecker/YOLO)을 그대로 재사용하고, 그 위에 (1) PyQt 없이 도는 헤드리스 제어 루프, (2) FastAPI+uvicorn 백엔드, (3) 정적 HTML/JS 프론트엔드, (4) CLI 진입점을 새로 얹는다. Physical Labs GUI 앱(`/opt/physical-labs`)은 읽기 참고만 하고 절대 수정·의존하지 않는다.

**Tech Stack:** Python 3, `lerobot`(LeKiwiClient), `ultralytics`(YOLO, 실행 시에만 필요), `fastapi`+`uvicorn`, `opencv-python`, `numpy`, 순정 HTML/CSS/JS(빌드 없음), `pytest`.

**Spec:** `docs/superpowers/specs/2026-09-13-pickplace-demo-ui-design.md`

## Global Constraints

- Physical Labs 앱(`/opt/physical-labs`)의 파일을 수정하거나 import 하거나 런타임에 영향을 주는 어떤 것도 하지 않는다 — 읽기 참고만.
- 새 코드는 전부 이 레포(`motion/` 아래)에만 만든다.
- 모든 새 파일은 `motion/` 을 작업 디렉터리로 실행한다고 가정한다 (`from services.pickplace...`, `from webui...` 형태의 절대 import — 기존 `test_wrist_servo.py` 관례와 동일).
- 정지의 유일한 책임자는 제어 루프 자신의 `try/finally` 다 — 웹 서버 상태와 무관하게 동작해야 한다.
- 정상 정지(`stop`)는 시작 자세로 천천히 롤아웃 후 연결 해제, 비상정지(`abort`)/예외 종료는 롤아웃 없이 즉시 정지 후 연결 해제.
- 원본 CLI(`lekiwi_yolo_pick.py`, `roboseasy/lekiwi.git`)와 같은 순서로 상태기를 조립한다 — 이미 이 레포에 포팅된 `services.pickplace.*` 를 그대로 쓴다.
- 테스트는 `lerobot` conda 환경에서 실행한다 (`conda activate lerobot`) — `fastapi`/`lerobot`/`ultralytics` 가 그 환경에만 있음.

---

## Task 0: 테스트 환경에 pytest 설치

**Files:** 없음 (환경 설정만)

- [ ] **Step 1: `lerobot` conda 환경에 pytest 설치**

```bash
source /home/roboseasy/miniforge3/etc/profile.d/conda.sh && conda activate lerobot
pip install pytest
```

- [ ] **Step 2: 확인**

```bash
python -c "import pytest; print(pytest.__version__)"
```

Expected: 버전 번호가 출력됨 (에러 없음).

이후 모든 `pytest` 실행 커맨드는 `conda activate lerobot` 이 된 셸에서, 그리고 `cd motion` 한 상태에서 실행한다 (`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` 을 앞에 붙여 무관한 ROS2 pytest 플러그인 자동로드 에러를 피한다 — 이 레포에서 이미 쓰던 관례).

---

## Task 1: RollOutPlayer — 정지 시 시작 자세로 천천히 되돌아가는 보간기

**Files:**
- Create: `motion/services/pickplace/roll_out.py`
- Test: `motion/services/pickplace/test_roll_out.py`

**Interfaces:**
- Produces: `RollOutPlayer(current: dict[str, float], home: dict[str, float], duration_s: float)`, method `.update(now: float) -> dict[str, float]`, attribute `.done: bool`.

- [ ] **Step 1: 실패하는 테스트 작성**

```python
# motion/services/pickplace/test_roll_out.py
from services.pickplace.roll_out import RollOutPlayer


def test_interpolates_toward_home_over_duration():
    current = {"arm_shoulder_pan.pos": 0.0, "arm_gripper.pos": 40.0}
    home = {"arm_shoulder_pan.pos": 10.0, "arm_gripper.pos": 0.0}
    player = RollOutPlayer(current, home, duration_s=2.0)

    pose = player.update(now=0.0)
    assert pose["arm_shoulder_pan.pos"] == 0.0
    assert not player.done

    pose = player.update(now=1.0)  # 절반 지남
    assert pose["arm_shoulder_pan.pos"] == 5.0
    assert not player.done

    pose = player.update(now=2.0)  # 다 지남
    assert pose["arm_shoulder_pan.pos"] == 10.0
    assert player.done


def test_gripper_is_never_touched():
    current = {"arm_shoulder_pan.pos": 0.0, "arm_gripper.pos": 40.0}
    home = {"arm_shoulder_pan.pos": 10.0, "arm_gripper.pos": 0.0}
    player = RollOutPlayer(current, home, duration_s=1.0)
    pose = player.update(now=1.0)
    assert pose["arm_gripper.pos"] == 40.0  # home 의 0.0 이 아니라 current 값 유지


def test_already_close_to_home_finishes_immediately():
    current = {"arm_shoulder_pan.pos": 10.1}
    home = {"arm_shoulder_pan.pos": 10.0}
    player = RollOutPlayer(current, home, duration_s=2.0)
    assert player.done
```

- [ ] **Step 2: 테스트 실행해서 실패 확인**

```bash
cd motion && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest services/pickplace/test_roll_out.py -v
```

Expected: `ModuleNotFoundError: No module named 'services.pickplace.roll_out'` 로 FAIL.

- [ ] **Step 3: 구현 작성**

```python
# motion/services/pickplace/roll_out.py
"""정지 시 현재 자세 → 시작 자세로 천천히 되돌아가는 보간 재생기.

Physical Labs GUI 앱(`pick_worker.py`)의 `_roll_out` 안전 동작을 참고해 이 레포에
독립적으로 새로 구현했다 — 원본 CLI(`lekiwi_yolo_pick.py`)에는 이 동작이 없다
(정지 시 그냥 현재 자세를 유지한 채 바퀴만 세우고 즉시 연결을 끊는다). 그리퍼는
절대 건드리지 않는다 — 물건을 물고 있을 수 있다.
"""
from __future__ import annotations


class RollOutPlayer:
    def __init__(self, current: dict[str, float], home: dict[str, float], duration_s: float):
        self._current = dict(current)
        self._home = dict(home)
        self._joints = [k for k in home if k in current and not k.endswith("gripper.pos")]
        self._duration = max(duration_s, 1e-3)
        self._t0: float | None = None
        self.done = not self._joints
        if self._joints:
            max_delta = max(abs(self._home[k] - self._current[k]) for k in self._joints)
            if max_delta < 0.5:
                self.done = True

    def update(self, now: float) -> dict[str, float]:
        """`팔 목표 자세` 를 돌려준다. `self.done` 이 True 면 이미 시작 자세."""
        if self.done:
            return dict(self._current)
        if self._t0 is None:
            self._t0 = now
        ratio = min(1.0, (now - self._t0) / self._duration)
        pose = dict(self._current)
        for k in self._joints:
            pose[k] = self._current[k] + (self._home[k] - self._current[k]) * ratio
        if ratio >= 1.0:
            self.done = True
        return pose
```

- [ ] **Step 4: 테스트 실행해서 통과 확인**

```bash
cd motion && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest services/pickplace/test_roll_out.py -v
```

Expected: 3개 PASS.

- [ ] **Step 5: 커밋**

```bash
git add motion/services/pickplace/roll_out.py motion/services/pickplace/test_roll_out.py
git commit -m "Add: 정지 시 시작 자세로 되돌아가는 RollOutPlayer"
```

---

## Task 2: LatestFrame / SharedStatus — 스레드 간 공유 상태

**Files:**
- Create: `motion/services/pickplace/frame_stream.py`
- Test: `motion/services/pickplace/test_frame_stream.py`

**Interfaces:**
- Produces: `LatestFrame` (`.set(view: str, jpeg_bytes: bytes)`, `.get(view: str) -> bytes | None`), `SharedStatus` (`.set(status: dict)`, `.get() -> dict`).

- [ ] **Step 1: 실패하는 테스트 작성**

```python
# motion/services/pickplace/test_frame_stream.py
from services.pickplace.frame_stream import LatestFrame, SharedStatus


def test_latest_frame_returns_none_when_unset():
    frames = LatestFrame()
    assert frames.get("front") is None


def test_latest_frame_round_trip():
    frames = LatestFrame()
    frames.set("front", b"jpegbytes")
    assert frames.get("front") == b"jpegbytes"
    assert frames.get("wrist") is None


def test_shared_status_returns_empty_dict_by_default():
    status = SharedStatus()
    assert status.get() == {}


def test_shared_status_round_trip_returns_copy():
    status = SharedStatus()
    status.set({"state": "PICK"})
    got = status.get()
    assert got == {"state": "PICK"}
    got["state"] = "MUTATED"
    assert status.get() == {"state": "PICK"}  # 내부 dict 가 외부 변경에 영향받지 않음
```

- [ ] **Step 2: 테스트 실행해서 실패 확인**

```bash
cd motion && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest services/pickplace/test_frame_stream.py -v
```

Expected: `ModuleNotFoundError` 로 FAIL.

- [ ] **Step 3: 구현 작성**

```python
# motion/services/pickplace/frame_stream.py
"""백그라운드 제어 스레드와 FastAPI 요청 핸들러 사이의 스레드-세이프 공유 상태.

락은 참조 교체에만 쓴다 — JPEG 인코딩·dict 조립은 락 밖에서 한다. 그래야 영상
스트리밍 요청이 30Hz 제어 루프를 막지 않는다 (설계 문서 §3, 독립 검토 지적사항 반영).
"""
from __future__ import annotations

import threading
from typing import Any


class LatestFrame:
    """뷰 이름 → 최신 JPEG bytes."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._frames: dict[str, bytes] = {}

    def set(self, view: str, jpeg_bytes: bytes) -> None:
        with self._lock:
            self._frames[view] = jpeg_bytes

    def get(self, view: str) -> bytes | None:
        with self._lock:
            return self._frames.get(view)


class SharedStatus:
    """최신 상태 dict."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._status: dict[str, Any] = {}

    def set(self, status: dict[str, Any]) -> None:
        with self._lock:
            self._status = dict(status)

    def get(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._status)
```

- [ ] **Step 4: 테스트 실행해서 통과 확인**

```bash
cd motion && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest services/pickplace/test_frame_stream.py -v
```

Expected: 4개 PASS.

- [ ] **Step 5: 커밋**

```bash
git add motion/services/pickplace/frame_stream.py motion/services/pickplace/test_frame_stream.py
git commit -m "Add: 제어 스레드-FastAPI 간 공유 상태(LatestFrame/SharedStatus)"
```

---

## Task 3: LeKiwiRobotArgs — 로봇 연결 설정

**Files:**
- Create: `motion/services/pickplace/robot_args.py`
- Test: `motion/services/pickplace/test_robot_args.py`

**Interfaces:**
- Produces: `LeKiwiRobotArgs` dataclass (`remote_ip`, `id`, `port_zmq_cmd`, `port_zmq_observations`, `connect_timeout_s`), 메서드 `.to_config()` → `lerobot.robots.lekiwi.LeKiwiClientConfig`.

- [ ] **Step 1: 실패하는 테스트 작성**

```python
# motion/services/pickplace/test_robot_args.py
from services.pickplace.robot_args import LeKiwiRobotArgs


def test_defaults():
    args = LeKiwiRobotArgs()
    assert args.remote_ip == "192.168.0.201"
    assert args.id == "lekiwi01"
    assert args.port_zmq_cmd == 5555
    assert args.port_zmq_observations == 5556


def test_to_config_builds_lekiwi_client_config():
    args = LeKiwiRobotArgs(
        remote_ip="10.42.0.141", id="lekiwi01", port_zmq_cmd=5555, port_zmq_observations=5556,
    )
    cfg = args.to_config()
    assert cfg.remote_ip == "10.42.0.141"
    assert cfg.id == "lekiwi01"
    assert cfg.port_zmq_cmd == 5555
    assert cfg.port_zmq_observations == 5556
```

- [ ] **Step 2: 테스트 실행해서 실패 확인**

```bash
source /home/roboseasy/miniforge3/etc/profile.d/conda.sh && conda activate lerobot
cd motion && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest services/pickplace/test_robot_args.py -v
```

Expected: `ModuleNotFoundError` 로 FAIL.

- [ ] **Step 3: 구현 작성**

```python
# motion/services/pickplace/robot_args.py
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
```

- [ ] **Step 4: 테스트 실행해서 통과 확인**

```bash
cd motion && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest services/pickplace/test_robot_args.py -v
```

Expected: 2개 PASS.

- [ ] **Step 5: 커밋**

```bash
git add motion/services/pickplace/robot_args.py motion/services/pickplace/test_robot_args.py
git commit -m "Add: LeKiwiRobotArgs 연결 설정"
```

---

## Task 4: PickPlaceHeadlessWorker — 헤드리스 제어 루프

**Files:**
- Create: `motion/services/pickplace/headless_worker.py`
- Test: `motion/services/pickplace/test_headless_worker.py`

**Interfaces:**
- Consumes: `RollOutPlayer` (Task 1), `LatestFrame`/`SharedStatus` (Task 2), `services.pickplace.approach.{STOP, Approacher, draw_alignment}`, `services.pickplace.arm_sequencer.{ArmSequencer, draw_wrist_servo}`, `services.pickplace.config.PickPlaceConfig`, `services.pickplace.grasp_check.{GraspChecker, center_purple_ratio, draw_grasp_check}`, `services.pickplace.wrist_servo.{GRIPPER_JOINT, filter_wrist_dets}`, `services.pickplace.yolo_detect.{draw, infer, load_model}`.
- Produces: `PickPlaceHeadlessWorker(robot, cfg, poses, *, rollout_time_s=2.0, first_obs_timeout_s=8.0, infer_fn=infer, load_model_fn=load_model)`. 공개 API: `.start_background()`, `.join(timeout=None)`, `.resume()`, `.pause()`, `.request_stop()`, `.request_abort()`, `.run()`(동기 실행, 테스트/디버깅용), 속성 `.paused_event`, `.stop_event`, `.abort_event`, `.frames: LatestFrame`, `.status: SharedStatus`.

- [ ] **Step 1: 헬퍼 메서드용 실패하는 테스트 작성**

```python
# motion/services/pickplace/test_headless_worker.py
import numpy as np
import pytest

from services.pickplace.arm_sequencer import PickArgs
from services.pickplace.config import PickPlaceConfig
from services.pickplace.headless_worker import PickPlaceHeadlessWorker
from services.pickplace.wrist_servo import GRIPPER_JOINT, GraspArgs


class FakeRobot:
    """PickPlaceHeadlessWorker 가 요구하는 최소 인터페이스만 흉내낸다 (덕타이핑)."""

    def __init__(self, frame_shape=(64, 64, 3)):
        self.connected = False
        self.sent_actions: list[dict[str, float]] = []
        self.on_observation = None  # 테스트가 나중에 채운다
        self._obs_calls = 0
        self._frame = np.zeros(frame_shape, dtype=np.uint8)
        self._pose = {
            "arm_shoulder_pan.pos": 0.0,
            "arm_shoulder_lift.pos": 0.0,
            "arm_elbow_flex.pos": 0.0,
            "arm_wrist_flex.pos": 0.0,
            "arm_wrist_roll.pos": 0.0,
            GRIPPER_JOINT: 0.0,
        }

    def connect(self) -> None:
        self.connected = True

    def get_observation(self) -> dict:
        self._obs_calls += 1
        obs = dict(self._pose)
        obs["front"] = self._frame
        obs["wrist"] = self._frame
        if self.on_observation is not None:
            self.on_observation()
        return obs

    def send_action(self, action: dict) -> None:
        self.sent_actions.append(dict(action))
        for k, v in action.items():
            if k in self._pose:
                self._pose[k] = v

    def disconnect(self) -> None:
        self.connected = False

    @property
    def is_connected(self) -> bool:
        return self.connected


def _fake_infer(model, cfg, frames_bgr):
    return {v: [] for v in frames_bgr}


def _fake_load_model(cfg):
    return object()


def _worker(robot, cfg=None, poses=None) -> PickPlaceHeadlessWorker:
    cfg = cfg or PickPlaceConfig(pick=PickArgs(enabled=False))
    w = PickPlaceHeadlessWorker(
        robot, cfg, poses or {}, infer_fn=_fake_infer, load_model_fn=_fake_load_model,
        first_obs_timeout_s=0.5,
    )
    return w


def test_resolve_gripper_close_pct_from_poses():
    robot = FakeRobot()
    cfg = PickPlaceConfig(pick=PickArgs(enabled=True), grasp=GraspArgs(enabled=True, close_after_ready=True))
    w = _worker(robot, cfg, poses={"grasp_closed": {GRIPPER_JOINT: 15.1}})
    problem = w._resolve_gripper_close_pct()
    assert problem == ""
    assert cfg.grasp.gripper_close_pct == pytest.approx(15.1)


def test_resolve_gripper_close_pct_missing_pose_returns_error():
    robot = FakeRobot()
    cfg = PickPlaceConfig(pick=PickArgs(enabled=True), grasp=GraspArgs(enabled=True, close_after_ready=True))
    w = _worker(robot, cfg, poses={})
    problem = w._resolve_gripper_close_pct()
    assert problem != ""
    assert cfg.grasp.gripper_close_pct is None


def test_wait_for_first_frames_times_out_without_cameras():
    robot = FakeRobot()
    robot._frame = None  # front/wrist 를 아예 안 준다
    cfg = PickPlaceConfig(pick=PickArgs(enabled=False))
    w = PickPlaceHeadlessWorker(
        robot, cfg, {}, infer_fn=_fake_infer, load_model_fn=_fake_load_model, first_obs_timeout_s=0.2,
    )

    def get_observation_no_cameras():
        return {}

    robot.get_observation = get_observation_no_cameras
    assert w._wait_for_first_frames() is None
```

- [ ] **Step 2: 테스트 실행해서 실패 확인**

```bash
cd motion && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest services/pickplace/test_headless_worker.py -v
```

Expected: `ModuleNotFoundError` 로 FAIL.

- [ ] **Step 3: 구현 작성 (헬퍼 + 생성자)**

```python
# motion/services/pickplace/headless_worker.py
"""Physical Labs 앱 없이 터미널/웹에서 pick&place 를 돌리는 헤드리스 제어 루프.

원본 CLI(`lekiwi_yolo_pick.py`, roboseasy/lekiwi.git)의 control_loop/main 과 같은
조립 순서(연결 → 첫 프레임 대기 → 관측 → YOLO 추론 → Approacher/ArmSequencer/
GraspChecker 갱신 → 오버레이 → send_action)를 따르되, PyQt 대신 threading.Event
로 제어하고 cv2 창 대신 LatestFrame/SharedStatus 로 상태를 내보낸다.

정지 시 시작 자세로 롤아웃하는 안전 동작(`RollOutPlayer`)은 원본에 없던 것으로
설계 문서 §"제어 루프"에서 의도적으로 추가한 것이다.
"""
from __future__ import annotations

import logging
import threading
import time
from typing import Any, Callable, Protocol

import cv2
import numpy as np

from services.pickplace.approach import STOP, Approacher, draw_alignment
from services.pickplace.arm_sequencer import ArmSequencer, draw_wrist_servo
from services.pickplace.config import PickPlaceConfig
from services.pickplace.frame_stream import LatestFrame, SharedStatus
from services.pickplace.grasp_check import GraspChecker, center_purple_ratio, draw_grasp_check
from services.pickplace.roll_out import RollOutPlayer
from services.pickplace.wrist_servo import GRIPPER_JOINT, filter_wrist_dets
from services.pickplace.yolo_detect import draw, infer as yolo_infer, load_model


class Robot(Protocol):
    """PickPlaceHeadlessWorker 가 요구하는 최소 인터페이스 (덕타이핑, lerobot 무관 테스트용)."""

    def connect(self) -> None: ...
    def get_observation(self) -> dict[str, Any]: ...
    def send_action(self, action: dict[str, float]) -> None: ...
    def disconnect(self) -> None: ...

    @property
    def is_connected(self) -> bool: ...


class PickPlaceHeadlessWorker:
    def __init__(
        self,
        robot: Robot,
        cfg: PickPlaceConfig,
        poses: dict[str, dict[str, float]],
        *,
        rollout_time_s: float = 2.0,
        first_obs_timeout_s: float = 8.0,
        infer_fn: Callable = yolo_infer,
        load_model_fn: Callable = load_model,
    ) -> None:
        self.robot = robot
        self.cfg = cfg
        self._poses = {k: dict(v) for k, v in poses.items()}
        self._rollout_time_s = rollout_time_s
        self._first_obs_timeout_s = first_obs_timeout_s
        self._infer = infer_fn
        self._load_model = load_model_fn

        self.frames = LatestFrame()
        self.status = SharedStatus()

        self.paused_event = threading.Event()
        self.paused_event.set()  # 시작은 항상 일시정지 상태 — 조작자가 [시작] 을 눌러야 움직인다
        self.stop_event = threading.Event()
        self.abort_event = threading.Event()

        self._thread: threading.Thread | None = None

    # ── 공개 제어 ──
    def start_background(self) -> None:
        self._thread = threading.Thread(target=self.run, daemon=False)
        self._thread.start()

    def join(self, timeout: float | None = None) -> None:
        if self._thread is not None:
            self._thread.join(timeout)

    def resume(self) -> None:
        self.paused_event.clear()

    def pause(self) -> None:
        self.paused_event.set()

    def request_stop(self) -> None:
        self.stop_event.set()

    def request_abort(self) -> None:
        self.abort_event.set()
        self.stop_event.set()

    # ── 내부 헬퍼 ──
    @staticmethod
    def _latch_pose(obs: dict) -> dict[str, float]:
        return {k: float(v) for k, v in (obs or {}).items() if isinstance(k, str) and k.endswith(".pos")}

    def _filter_pose(self, name: str, home: dict[str, float]) -> dict[str, float] | None:
        pose = self._poses.get(name)
        if not pose:
            return None
        return {k: float(v) for k, v in pose.items() if k in home}

    def _resolve_gripper_close_pct(self) -> str:
        """그리퍼 닫기 목표를 poses['grasp_closed'] 에서 정한다. 문제가 있으면 에러 메시지를 돌려준다."""
        g = self.cfg.grasp
        if not (self.cfg.pick.enabled and g.enabled and g.close_after_ready):
            return ""
        if g.gripper_close_pct is not None:
            return ""
        closed = self._poses.get("grasp_closed") or {}
        if GRIPPER_JOINT not in closed:
            return "grasp_closed 자세에 그리퍼 값이 없습니다 (poses['grasp_closed'])."
        base = float(closed[GRIPPER_JOINT])
        g.gripper_close_pct = float(np.clip(base - g.gripper_close_extra_pct, 0.0, 100.0))
        return ""

    def _wait_for_first_frames(self) -> dict[str, Any] | None:
        deadline = time.perf_counter() + self._first_obs_timeout_s
        while not self.stop_event.is_set():
            obs = self.robot.get_observation() or {}
            got = [v for v in self.cfg.views if isinstance(obs.get(v), np.ndarray)]
            if len(got) == len(self.cfg.views):
                return obs
            if time.perf_counter() > deadline:
                return None
            time.sleep(0.1)
        return None
```

- [ ] **Step 4: 테스트 실행해서 통과 확인**

```bash
cd motion && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest services/pickplace/test_headless_worker.py -v
```

Expected: 3개 PASS.

- [ ] **Step 5: 커밋**

```bash
git add motion/services/pickplace/headless_worker.py motion/services/pickplace/test_headless_worker.py
git commit -m "Add: PickPlaceHeadlessWorker 헬퍼 메서드 (그리퍼 닫기 목표·첫 프레임 대기)"
```

- [ ] **Step 6: 루프/정지 동작용 실패하는 테스트 추가**

```python
# motion/services/pickplace/test_headless_worker.py 에 추가

def test_paused_worker_only_sends_stop_and_then_stops_cleanly():
    robot = FakeRobot()
    w = _worker(robot)  # 기본 paused_event.set() 상태, pick.enabled=False

    def stop_after_three():
        if robot._obs_calls >= 3:
            w.stop_event.set()

    robot.on_observation = stop_after_three
    w.run()

    assert len(robot.sent_actions) >= 3
    for action in robot.sent_actions:
        assert action["x.vel"] == 0.0
        assert action["y.vel"] == 0.0
        assert action["theta.vel"] == 0.0
    assert w.status.get()["state"] == "PAUSED"
    assert robot.connected is False  # finally 에서 disconnect 됨


def test_clean_stop_rolls_out_toward_home_before_disconnect():
    robot = FakeRobot()
    w = _worker(robot)
    w.resume()

    def stop_after_three():
        if robot._obs_calls >= 3:
            w.stop_event.set()  # abort 아님 → 롤아웃 있어야 함

    robot.on_observation = stop_after_three
    robot._pose["arm_shoulder_pan.pos"] = 10.0  # home(0.0) 과 차이를 만들어 롤아웃이 보이게

    w.run()

    # 마지막에서 두 번째 이후 액션들 중 팔이 0.0 쪽으로 움직이는 시도가 있어야 한다
    pan_values = [a["arm_shoulder_pan.pos"] for a in robot.sent_actions if "arm_shoulder_pan.pos" in a]
    assert pan_values[-1] < 10.0


def test_abort_skips_rollout():
    robot = FakeRobot()
    w = _worker(robot)
    w.resume()
    robot._pose["arm_shoulder_pan.pos"] = 10.0

    def abort_after_three():
        if robot._obs_calls >= 3:
            w.request_abort()

    robot.on_observation = abort_after_three
    w.run()

    # 마지막 액션의 팔 자세가 그대로 10.0 이어야 한다 (롤아웃 없이 즉시 정지)
    pan_values = [a["arm_shoulder_pan.pos"] for a in robot.sent_actions if "arm_shoulder_pan.pos" in a]
    assert pan_values[-1] == pytest.approx(10.0)
```

- [ ] **Step 7: 테스트 실행해서 실패 확인**

```bash
cd motion && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest services/pickplace/test_headless_worker.py -v
```

Expected: 새 3개 테스트가 `AttributeError: 'PickPlaceHeadlessWorker' object has no attribute 'run'` 로 FAIL (나머지 3개는 여전히 PASS).

- [ ] **Step 8: `run()`/`_shutdown()` 구현 (파일 끝에 추가)**

```python
# motion/services/pickplace/headless_worker.py 에 이어서 (클래스 안, _wait_for_first_frames 다음)

    def run(self) -> None:
        home: dict[str, float] = {}
        clean_exit = False
        interval = 1.0 / max(1, self.cfg.fps)
        try:
            model = self._load_model(self.cfg.yolo)
            self.robot.connect()

            obs = self._wait_for_first_frames()
            if obs is None:
                self.status.set({"state": "ERROR", "error": f"{self._first_obs_timeout_s:.0f}초 안에 카메라 프레임을 받지 못했습니다."})
                return

            home = self._latch_pose(obs)
            if not home:
                self.status.set({"state": "ERROR", "error": "팔 관절 위치를 읽지 못했습니다."})
                return

            problem = self._resolve_gripper_close_pct()
            if problem:
                self.status.set({"state": "ERROR", "error": problem})
                return

            pick_pose = self._filter_pose("pre_pick", home) if self.cfg.pick.enabled else None
            grasp_pose = (
                self._filter_pose("grasp", home)
                if (self.cfg.pick.enabled and self.cfg.grasp.enabled)
                else None
            )

            ap = Approacher(self.cfg.approach)
            arm = ArmSequencer(home, pick_pose, self.cfg.pick, self.cfg.grasp, grasp_pose)
            checker = GraspChecker(self.cfg.check)
            hold_pose = dict(home)
            hz = 0.0

            while not self.stop_event.is_set():
                loop_start = time.perf_counter()
                obs = self.robot.get_observation() or {}
                frames_bgr = {
                    v: cv2.cvtColor(obs[v], cv2.COLOR_RGB2BGR)
                    for v in self.cfg.views
                    if isinstance(obs.get(v), np.ndarray)
                }
                if self.cfg.approach.view not in frames_bgr:
                    self.robot.send_action({**hold_pose, **STOP})
                    self.stop_event.wait(0.05)
                    continue

                dets_by_view = self._infer(model, self.cfg.yolo, frames_bgr)
                if (
                    self.cfg.grasp.enabled
                    and self.cfg.grasp.view in frames_bgr
                    and self.cfg.grasp.view != self.cfg.approach.view
                ):
                    dets_by_view[self.cfg.grasp.view] = filter_wrist_dets(
                        dets_by_view.get(self.cfg.grasp.view, []),
                        frames_bgr[self.cfg.grasp.view].shape,
                        self.cfg.grasp,
                    )
                wrist_frame = frames_bgr.get(self.cfg.grasp.view)
                paused = self.paused_event.is_set()
                allow_motion = not paused and not self.cfg.dry_run

                cmd = ap.update(
                    dets_by_view[self.cfg.approach.view], frames_bgr[self.cfg.approach.view].shape, loop_start
                )
                tracking_ok = ap.target is not None and ap.size_ok and ap.center_ok

                descend_hint = False
                if self.cfg.grasp.front_hint and arm.state == "SERVO" and self.cfg.approach.view in frames_bgr:
                    ratio = center_purple_ratio(
                        frames_bgr[self.cfg.approach.view], self.cfg.check, self.cfg.grasp.front_hint_win_px
                    )
                    descend_hint = ratio >= self.cfg.grasp.front_hint_min_ratio

                arm_pose = arm.update(
                    ap.done,
                    tracking_ok,
                    allow_motion,
                    loop_start,
                    wrist_dets=dets_by_view.get(self.cfg.grasp.view) if wrist_frame is not None else None,
                    wrist_shape=wrist_frame.shape if wrist_frame is not None else None,
                    dt=interval,
                    descend_hint=descend_hint,
                )

                if self.cfg.check.enabled and arm.grasp_enabled and arm.state == "GRASPED":
                    if checker.state == "IDLE":
                        checker.start(loop_start)
                    checker.update(frames_bgr, dets_by_view, loop_start)
                check_label = {"CHECKING": "GRASP_CHECK", "SUCCESS": "GRASP_OK", "FAIL": "GRASP_FAIL"}.get(
                    checker.state
                )

                if paused:
                    state_label, sent = "PAUSED", dict(STOP)
                elif self.cfg.dry_run:
                    state_label, sent = "DRY_RUN", dict(STOP)
                elif arm.base_locked:
                    state_label, sent = (check_label or arm.label or ap.state), dict(STOP)
                else:
                    state_label, sent = ap.state, cmd

                if checker.just_decided:
                    if checker.state == "FAIL" and allow_motion and arm.can_retry:
                        arm.retry(loop_start)
                        checker.reset()
                    elif checker.state == "FAIL" and allow_motion and arm.can_restart_pick:
                        arm.restart_pick(loop_start)
                        checker.reset()
                    elif checker.state == "FAIL":
                        arm.give_up()
                if (
                    checker.state == "SUCCESS"
                    and arm.state == "GRASPED"
                    and self.cfg.grasp.return_home_when_grasped
                    and allow_motion
                ):
                    arm.carry_home(loop_start)
                if arm.state not in ("GRASPED", "CARRY_HOME", "DONE") and checker.state != "IDLE":
                    checker.reset()

                hold_pose = dict(arm_pose)
                self.robot.send_action({**arm_pose, **sent})

                for v in self.cfg.views:
                    if v not in frames_bgr:
                        continue
                    img = draw(frames_bgr[v], v, dets_by_view.get(v, []), hz, crosshair=v in self.cfg.crosshair_views)
                    if v == self.cfg.approach.view:
                        img = draw_alignment(img, ap, cmd, state_label)
                    if v == self.cfg.grasp.view and arm.grasp_enabled:
                        img = draw_wrist_servo(img, arm, state_label)
                    if self.cfg.check.enabled and v in (self.cfg.check.front_view, self.cfg.check.wrist_view):
                        img = draw_grasp_check(img, checker, v)
                    ok, buf = cv2.imencode(".jpg", img)
                    if ok:
                        self.frames.set(v, buf.tobytes())

                self.status.set(
                    {
                        "state": state_label,
                        "pick_attempts": arm.pick_attempts,
                        "max_pick_attempts": self.cfg.grasp.max_pick_attempts,
                        "retries": arm.retries,
                        "max_retries": self.cfg.grasp.max_retries,
                        "gave_up": arm.gave_up,
                        "arm_done": arm.done,
                        "dry_run": self.cfg.dry_run,
                        "paused": paused,
                        "hz": round(hz, 1),
                    }
                )

                dt = time.perf_counter() - loop_start
                if interval - dt > 0:
                    self.stop_event.wait(interval - dt)
                hz = 1.0 / max(time.perf_counter() - loop_start, 1e-6)

            clean_exit = True
        except Exception as exc:
            logging.exception("PickPlaceHeadlessWorker 오류")
            self.status.set({"state": "ERROR", "error": str(exc)})
        finally:
            self._shutdown(home, clean_exit)

    def _shutdown(self, home: dict[str, float], clean_exit: bool) -> None:
        try:
            if not self.robot.is_connected:
                return
            if clean_exit and not self.abort_event.is_set() and home:
                current = self._latch_pose(self.robot.get_observation() or {})
                rollout = RollOutPlayer(current, home, self._rollout_time_s)
                while not rollout.done:
                    now = time.perf_counter()
                    pose = rollout.update(now)
                    self.robot.send_action({**pose, **STOP})
                    time.sleep(1.0 / max(1, self.cfg.fps))
            else:
                hold = self._latch_pose(self.robot.get_observation() or {})
                self.robot.send_action({**hold, **STOP})
            time.sleep(0.2)
        except Exception:
            logging.exception("정지 시퀀스 중 오류")
        finally:
            try:
                self.robot.disconnect()
            except Exception:
                logging.exception("연결 해제 실패")
```

- [ ] **Step 9: 테스트 실행해서 통과 확인**

```bash
cd motion && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest services/pickplace/test_headless_worker.py -v
```

Expected: 6개 모두 PASS.

- [ ] **Step 10: 커밋**

```bash
git add motion/services/pickplace/headless_worker.py motion/services/pickplace/test_headless_worker.py
git commit -m "Add: PickPlaceHeadlessWorker 제어 루프 (정지/비상정지/롤아웃)"
```

---

## Task 5: FastAPI 백엔드

**Files:**
- Create: `motion/webui/__init__.py` (빈 파일)
- Create: `motion/webui/app.py`
- Test: `motion/webui/test_app.py`

**Interfaces:**
- Consumes: `PickPlaceHeadlessWorker` 와 같은 인터페이스를 가진 아무 객체 (`.resume()`, `.pause()`, `.request_stop()`, `.request_abort()`, `.status: SharedStatus`, `.frames: LatestFrame`) — Task 4 의 클래스이거나 테스트용 페이크.
- Produces: `create_app(worker) -> fastapi.FastAPI`.

- [ ] **Step 1: 빈 패키지 파일 생성**

```bash
touch motion/webui/__init__.py
```

- [ ] **Step 2: 실패하는 테스트 작성**

```python
# motion/webui/test_app.py
from fastapi.testclient import TestClient

from services.pickplace.frame_stream import LatestFrame, SharedStatus
from webui.app import create_app


class FakeWorker:
    def __init__(self):
        self.status = SharedStatus()
        self.frames = LatestFrame()
        self.resumed = False
        self.paused = False
        self.stopped = False
        self.aborted = False

    def resume(self):
        self.resumed = True

    def pause(self):
        self.paused = True

    def request_stop(self):
        self.stopped = True

    def request_abort(self):
        self.aborted = True


def test_start_resumes_worker():
    worker = FakeWorker()
    client = TestClient(create_app(worker))
    resp = client.post("/start")
    assert resp.status_code == 200
    assert worker.resumed is True


def test_pause_pauses_worker():
    worker = FakeWorker()
    client = TestClient(create_app(worker))
    resp = client.post("/pause")
    assert resp.status_code == 200
    assert worker.paused is True


def test_stop_requests_stop():
    worker = FakeWorker()
    client = TestClient(create_app(worker))
    resp = client.post("/stop")
    assert resp.status_code == 200
    assert worker.stopped is True
    assert worker.aborted is False


def test_estop_requests_abort():
    worker = FakeWorker()
    client = TestClient(create_app(worker))
    resp = client.post("/estop")
    assert resp.status_code == 200
    assert worker.aborted is True


def test_status_returns_worker_status():
    worker = FakeWorker()
    worker.status.set({"state": "PICK", "hz": 29.5})
    client = TestClient(create_app(worker))
    resp = client.get("/status")
    assert resp.json() == {"state": "PICK", "hz": 29.5}


def test_stream_serves_multipart_jpeg():
    worker = FakeWorker()
    worker.frames.set("front", b"\xff\xd8\xff\xfake-jpeg")
    client = TestClient(create_app(worker))
    with client.stream("GET", "/stream/front") as resp:
        assert resp.status_code == 200
        chunk = next(resp.iter_bytes())
        assert b"fake-jpeg" in chunk
```

- [ ] **Step 3: 테스트 실행해서 실패 확인**

```bash
cd motion && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest webui/test_app.py -v
```

Expected: `ModuleNotFoundError: No module named 'webui.app'` 로 FAIL.

- [ ] **Step 4: 구현 작성**

```python
# motion/webui/app.py
"""Pick&Place 시연용 FastAPI 앱 — PickPlaceHeadlessWorker 를 감싸는 얇은 HTTP 계층.

`/estop`/`/status` 는 플래그 세팅·dict 복사뿐인 가벼운 동기 함수라 FastAPI 가
자동으로 스레드풀에서 돌린다 (`async def` 를 안 쓰면 이렇게 된다) — 영상 스트리밍
같은 무거운 핸들러에 발목 잡히지 않는다 (설계 문서 §3).
"""
from __future__ import annotations

import time
from pathlib import Path
from typing import Any, Iterator

from fastapi import FastAPI
from fastapi.responses import HTMLResponse, StreamingResponse

_STATIC_DIR = Path(__file__).resolve().parent / "static"
_BOUNDARY = b"frame"


def create_app(worker: Any) -> FastAPI:
    app = FastAPI()

    @app.get("/", response_class=HTMLResponse)
    def index() -> str:
        return (_STATIC_DIR / "index.html").read_text()

    @app.post("/start")
    def start() -> dict[str, bool]:
        worker.resume()
        return {"ok": True}

    @app.post("/pause")
    def pause() -> dict[str, bool]:
        worker.pause()
        return {"ok": True}

    @app.post("/stop")
    def stop() -> dict[str, bool]:
        worker.request_stop()
        return {"ok": True}

    @app.post("/estop")
    def estop() -> dict[str, bool]:
        worker.request_abort()
        return {"ok": True}

    @app.get("/status")
    def status() -> dict[str, Any]:
        return worker.status.get()

    def _mjpeg(view: str) -> Iterator[bytes]:
        while True:
            frame = worker.frames.get(view)
            if frame is not None:
                yield (
                    b"--" + _BOUNDARY + b"\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
                )
            time.sleep(0.1)

    @app.get("/stream/{view}")
    def stream(view: str) -> StreamingResponse:
        return StreamingResponse(
            _mjpeg(view),
            media_type=f"multipart/x-mixed-replace; boundary={_BOUNDARY.decode()}",
        )

    return app
```

`_STATIC_DIR / "index.html"` 은 Task 6 에서 만든다 — 이 Task 의 테스트는 `index.html` 을 건드리지 않으므로 먼저 진행해도 된다.

- [ ] **Step 5: 테스트 실행해서 통과 확인**

```bash
cd motion && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest webui/test_app.py -v
```

Expected: 6개 모두 PASS.

- [ ] **Step 6: 커밋**

```bash
git add motion/webui/__init__.py motion/webui/app.py motion/webui/test_app.py
git commit -m "Add: pick&place 제어용 FastAPI 백엔드"
```

---

## Task 6: 프론트엔드 정적 페이지

**Files:**
- Create: `motion/webui/static/index.html`

**Interfaces:**
- Consumes: Task 5 의 `/start`, `/pause`, `/stop`, `/estop`, `/status`, `/stream/front`, `/stream/wrist` 엔드포인트.

- [ ] **Step 1: 페이지 작성**

```html
<!-- motion/webui/static/index.html -->
<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="utf-8">
<title>LeKiwi Pick&Place</title>
<style>
  body { font-family: sans-serif; background: #111; color: #eee; margin: 0; padding: 16px; }
  .views { display: flex; gap: 12px; flex-wrap: wrap; }
  .views img { width: 480px; max-width: 100%; border: 2px solid #444; border-radius: 4px; }
  .controls { margin-top: 16px; display: flex; gap: 10px; align-items: center; flex-wrap: wrap; }
  button { font-size: 18px; padding: 10px 18px; border-radius: 6px; border: none; cursor: pointer; }
  #start { background: #2ecc71; color: #000; }
  #pause { background: #f1c40f; color: #000; }
  #stop { background: #7f8c8d; color: #fff; }
  #estop { background: #e74c3c; color: #fff; font-weight: bold; font-size: 22px; padding: 14px 24px; }
  #status { margin-top: 16px; font-family: monospace; white-space: pre-wrap; background: #222; padding: 10px; border-radius: 4px; }
</style>
</head>
<body>
  <h1>LeKiwi Pick&Place</h1>
  <div class="views">
    <img src="/stream/front" alt="front">
    <img src="/stream/wrist" alt="wrist">
  </div>
  <div class="controls">
    <button id="start">▶ 시작</button>
    <button id="pause">❚❚ 일시정지</button>
    <button id="stop">■ 정지</button>
    <button id="estop">⚠ 비상정지</button>
  </div>
  <pre id="status">연결 중...</pre>

<script>
  async function post(path) {
    await fetch(path, { method: "POST" });
  }
  document.getElementById("start").onclick = () => post("/start");
  document.getElementById("pause").onclick = () => post("/pause");
  document.getElementById("stop").onclick = () => post("/stop");
  document.getElementById("estop").onclick = () => post("/estop");

  async function pollStatus() {
    try {
      const resp = await fetch("/status");
      const data = await resp.json();
      document.getElementById("status").textContent = JSON.stringify(data, null, 2);
    } catch (e) {
      document.getElementById("status").textContent = "상태 조회 실패: " + e;
    }
  }
  setInterval(pollStatus, 300);
  pollStatus();
</script>
</body>
</html>
```

- [ ] **Step 2: 수동 확인** (자동 테스트 없음 — 정적 파일)

```bash
cd motion && python -c "from pathlib import Path; Path('webui/static/index.html').read_text()" && echo "파일 읽힘 OK"
```

- [ ] **Step 3: 커밋**

```bash
git add motion/webui/static/index.html
git commit -m "Add: 시연용 프론트엔드 페이지 (버튼 + 카메라 스트림 + 상태)"
```

---

## Task 7: CLI 진입점

**Files:**
- Create: `motion/webui/run_pickplace_ui.py`

**Interfaces:**
- Consumes: `LeKiwiRobotArgs`(Task 3), `PickPlaceHeadlessWorker`(Task 4), `create_app`(Task 5), `services.pickplace.poses.load_pose`, `services.pickplace.config.PickPlaceConfig`, `lerobot.configs.parser`, `lerobot.robots.lekiwi.LeKiwiClient`.

- [ ] **Step 1: 구현 작성**

```python
# motion/webui/run_pickplace_ui.py
"""Pick&Place 시연용 웹 서버 진입점.

사용: python webui/run_pickplace_ui.py --robot.remote_ip=10.42.0.141 --robot.id=lekiwi01 ...
인자 스타일은 원본 CLI(lekiwi_yolo_pick.py) 와 동일하다 (중첩 dataclass,
`--그룹.필드=값`, `lerobot.configs.parser` 사용).

SIGINT/SIGTERM 을 별도로 잡지 않는다 — uvicorn 이 그 신호들을 받아 서버를 정리하고
`uvicorn.run()` 이 정상적으로 리턴하면, 아래 `finally` 가 항상 `worker.request_stop()`
을 부르므로 결과적으로 같은 정지 경로를 탄다. (단, kill -9 처럼 프로세스를 강제
종료하는 신호는 어떤 파이썬 코드로도 막을 수 없다 — 그건 물리적 비상정지로 대응한다.)
"""
from __future__ import annotations

import sys
from dataclasses import dataclass, field
from pathlib import Path

import uvicorn
from lerobot.configs import parser
from lerobot.robots.lekiwi import LeKiwiClient
from lerobot.utils.utils import init_logging

_MOTION_DIR = Path(__file__).resolve().parent.parent
if str(_MOTION_DIR) not in sys.path:
    sys.path.insert(0, str(_MOTION_DIR))

from services.pickplace.approach import ApproachArgs  # noqa: E402
from services.pickplace.arm_sequencer import PickArgs  # noqa: E402
from services.pickplace.config import PickPlaceConfig  # noqa: E402
from services.pickplace.grasp_check import GraspCheckArgs  # noqa: E402
from services.pickplace.headless_worker import PickPlaceHeadlessWorker  # noqa: E402
from services.pickplace.poses import load_pose  # noqa: E402
from services.pickplace.robot_args import LeKiwiRobotArgs  # noqa: E402
from services.pickplace.wrist_servo import GraspArgs  # noqa: E402
from services.pickplace.yolo_detect import YoloArgs  # noqa: E402

from webui.app import create_app  # noqa: E402

_DEFAULT_POSES_DIR = Path("~/.PhysicalLabs/pickplace/lekiwi01/poses").expanduser()
_DEFAULT_MODEL = "/home/roboseasy/YOLO/outputs/runs/green_pill/weights/best.pt"


@dataclass
class RunConfig:
    robot: LeKiwiRobotArgs = field(
        default_factory=lambda: LeKiwiRobotArgs(remote_ip="10.42.0.141", id="lekiwi01")
    )
    yolo: YoloArgs = field(default_factory=lambda: YoloArgs(path=_DEFAULT_MODEL))
    approach: ApproachArgs = field(default_factory=ApproachArgs)
    pick: PickArgs = field(
        default_factory=lambda: PickArgs(pose_file=str(_DEFAULT_POSES_DIR / "pre_pick.json"))
    )
    grasp: GraspArgs = field(
        default_factory=lambda: GraspArgs(
            grasp_pose_file=str(_DEFAULT_POSES_DIR / "grasp.json"),
            close_pose_file=str(_DEFAULT_POSES_DIR / "grasp_closed.json"),
        )
    )
    check: GraspCheckArgs = field(default_factory=GraspCheckArgs)
    views: list[str] = field(default_factory=lambda: ["front", "wrist"])
    fps: int = 30
    dry_run: bool = False
    crosshair_views: list[str] = field(default_factory=lambda: ["front", "wrist"])
    host: str = "0.0.0.0"
    port: int = 8000
    rollout_time_s: float = 2.0

    def to_pickplace_config(self) -> PickPlaceConfig:
        return PickPlaceConfig(
            yolo=self.yolo,
            approach=self.approach,
            pick=self.pick,
            grasp=self.grasp,
            check=self.check,
            views=list(self.views),
            fps=self.fps,
            dry_run=self.dry_run,
            crosshair_views=list(self.crosshair_views),
        )


def _load_poses(cfg: RunConfig) -> dict[str, dict[str, float]]:
    poses: dict[str, dict[str, float]] = {}
    for name, path in (
        ("pre_pick", cfg.pick.pose_file),
        ("grasp", cfg.grasp.grasp_pose_file),
        ("grasp_closed", cfg.grasp.close_pose_file),
    ):
        if path:
            poses[name] = load_pose(Path(path).expanduser())
    return poses


@parser.wrap()
def main(cfg: RunConfig) -> None:
    init_logging()
    pp_cfg = cfg.to_pickplace_config()
    pp_cfg.validate()
    poses = _load_poses(cfg)

    robot = LeKiwiClient(cfg.robot.to_config())
    worker = PickPlaceHeadlessWorker(robot, pp_cfg, poses, rollout_time_s=cfg.rollout_time_s)
    worker.start_background()

    app = create_app(worker)
    try:
        uvicorn.run(app, host=cfg.host, port=cfg.port)
    finally:
        worker.request_stop()
        worker.join(timeout=30.0)


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: `--help` 로 인자 파싱 확인 (로봇 연결 없이)**

```bash
source /home/roboseasy/miniforge3/etc/profile.d/conda.sh && conda activate lerobot
cd motion && python webui/run_pickplace_ui.py --help
```

Expected: `--robot.remote_ip`, `--yolo.path`, `--pick.pose_file`, `--grasp.grasp_pose_file` 등 인자 목록이 에러 없이 출력됨.

- [ ] **Step 3: dry-run 스모크 테스트 (로봇 호스트가 켜져 있을 때만 가능 — 지금 당장은 실패해도 됨, 나중에 로봇 켜지면 확인)**

```bash
cd motion && python webui/run_pickplace_ui.py \
  --robot.remote_ip=10.42.0.141 --robot.id=lekiwi01 \
  --dry_run=true --pick.enabled=false
```

Expected (로봇 호스트가 켜져 있으면): `Uvicorn running on http://0.0.0.0:8000` 로그가 뜨고, 브라우저로 `http://<이 PC IP>:8000` 접속 시 영상 두 개와 버튼이 보임. `Ctrl+C` 로 종료하면 로그에 정지 시퀀스가 찍히고 프로세스가 끝남.
(로봇 호스트가 꺼져 있으면): 연결 에러가 나며 `/status` 에 `"state": "ERROR"` 가 뜸 — 이것도 정상 동작(에러 처리가 됐다는 뜻).

- [ ] **Step 4: 커밋**

```bash
git add motion/webui/run_pickplace_ui.py
git commit -m "Add: pick&place 웹 UI CLI 진입점"
```

---

## Self-Review 결과 (계획 작성자가 직접 확인)

- **스펙 커버리지**: 목표(시작/일시정지/정지/비상정지, 카메라 스트림, 상태 표시, 기존 pose/robots.json 재사용) 전부 Task 5·6·7 에서 다룸. 안전 설계(정지=롤아웃, 비상정지=즉시)는 Task 1·4 에서. FastAPI 스레드풀/락 스코프는 Task 5 구현에 반영. "명시적으로 제외" 항목(이중 클라이언트 가드, 스레드 생존 여부 표시)은 의도적으로 어떤 Task 에도 없음 — 일치.
- **Placeholder 스캔**: "TODO"/"나중에" 문구 없음. 모든 스텝에 실제 코드 포함.
- **타입 일관성**: `PickPlaceHeadlessWorker` 생성자 인자명(`robot, cfg, poses, rollout_time_s, first_obs_timeout_s, infer_fn, load_model_fn`)과 속성명(`paused_event, stop_event, abort_event, frames, status`)이 Task 4 정의와 Task 5·7 사용처에서 동일하게 쓰임. `create_app(worker)` 시그니처가 Task 5 정의·테스트·Task 7 사용처에서 일치.
