# LeKiwi 관제 대시보드 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 노트북 브라우저 한 화면에서 Nav2 지도(로봇 자세·스캔·파티클·경로·궤적)와 집기 카메라, 미션 단계, 토픽 입출력, 정지 버튼을 보여주는 대시보드를 만든다.

**Architecture:** `nav/dashboard/` 의 ROS 노드(`/usr/bin/python3` + rclpy + FastAPI, :8001)가 토픽을 구독해 메모리 상태를 만들고 SSE 로 5 Hz 스냅샷을 보낸다. 브라우저가 canvas 에 지도와 레이어를 그린다. 카메라는 집기 스크립트가 `--web-port 8000` 으로 띄우는 기존 `create_app(worker)` 의 MJPEG 를 `<img>` 로 받는다. 순수 로직(좌표·단계·토픽 통계·정지 유지)은 ROS 없이 테스트한다.

**Tech Stack:** Python 3.12, rclpy (ROS 2 Jazzy), FastAPI + uvicorn, numpy, PyYAML, Pillow(pgm→png), pytest 7 (시스템 python), lerobot conda env(집기 워커), 순수 HTML/CSS/JS(canvas, EventSource).

**Spec:** `docs/superpowers/specs/2026-09-14-nav-pick-dashboard-design.md`

## Global Constraints

- 로봇에 아무것도 배포하지 않는다. 로봇 쪽 파일·설정 변경 금지.
- 브랜치 `feature/nav-dashboard` (main 기반, 이미 생성·설계 문서 커밋됨).
- 대시보드 코드는 `/usr/bin/python3` 로 실행·테스트: `/usr/bin/python3 -m pytest nav/dashboard -q` (ROS 가 필요한 `ros_listener.py`/`dashboard_node.py` 는 단위 테스트 대상 아님).
- motion·scripts 테스트는 lerobot env: `PYTHONNOUSERSITE=1 env -u PYTHONPATH /home/roboseasy/miniforge3/envs/lerobot/bin/python -m pytest <경로> -q` (저장소 루트에서).
- 대시보드는 `/abo/command "stop"` 외에 아무것도 발행하지 않는다.
- 나이(age)는 노트북 수신 시각 기준 초, 한 번도 안 받았으면 `None`(JSON `null`).
- QoS: `/scan`·`/particle_cloud` best_effort(sensor data), `/map`·`/amcl_pose`·`/tf_static` reliable+transient_local, 나머지 reliable depth 10 (`/tf` depth 100).
- 지도 파일: `nav/maps/lekiwi01/map_0912_1654.yaml` (53×48, res 0.05, origin [-0.707, -0.919]). 웨이포인트: `nav/mission/waypoints.yaml` (`points:` 아래 `{x, y, yaw(도)}`).
- SSE 5 Hz, 파티클 최대 800점, 궤적 최대 600점·5 cm 미만 이동 건너뜀.
- 정지 유지: 켜면 워커 estop + stop 1회. `moving`/`returning` 이 된 시각부터 5 s 동안 0.5 s 간격 재전송, `canceled` 받으면 창 닫음, `picking`/`arrived` 중엔 안 보냄, 8000 이 새로 응답하면 estop 재호출, 해제는 `/api/stop/release` 로만.
- `--web-port` 기본 바인드 `127.0.0.1`, 포트 사용 중이면 경고만 하고 집기는 계속.
- 문구는 한국어, 버튼 이름은 "정지"(비상정지 아님), 옆에 "확실한 비상정지는 로봇 전원 스위치".
- 커밋 메시지 끝에 두 줄:
  `Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>`
  `Claude-Session: https://claude.ai/code/session_011xipca4hVhjGeqXSUCLhrD`

## File Structure

| 파일 | 책임 |
|---|---|
| `scripts/pick_worker_cycle.py`, `scripts/test_pick_worker_cycle.py` | PR #8 원본 그대로(Task 1) + `--web-port` (Task 3) |
| `motion/services/pickplace/frame_stream.py` | + `FrameAgeTracker` (Task 2) |
| `motion/services/pickplace/headless_worker.py` | `/status` 에 `purple`·`retry_depth`·`frame_age_s` 추가 (Task 2) |
| `nav/dashboard/__init__.py` | 빈 파일(패키지) |
| `nav/dashboard/map_geometry.py` | 지도 정보·좌표 변환·스캔/파티클 점 (Task 4) |
| `nav/dashboard/mission_timeline.py` | `/abo/state` → 단계 (Task 5) |
| `nav/dashboard/topic_stats.py` | 토픽별 주기·나이·마지막 값 (Task 6) |
| `nav/dashboard/stop_latch.py` | 정지 유지 판단 (Task 7) |
| `nav/dashboard/dashboard_state.py` | 스레드 안전 상태 + 스냅샷 (Task 8) |
| `nav/dashboard/pick_client.py` | :8000 `/status` 폴링 캐시 + `/estop` 호출 (Task 9) |
| `nav/dashboard/web_app.py` | FastAPI 라우트 + SSE (Task 9) |
| `nav/dashboard/static/index.html`, `app.js`, `style.css` | 화면 (Task 10) |
| `nav/dashboard/ros_listener.py`, `dashboard_node.py`, `run_dashboard.sh` | ROS 연결·진입점 (Task 11) |
| `nav/dashboard/test_*.py` | 각 순수 모듈 테스트 |

---

### Task 1: PR #8 집기 스크립트를 원본 그대로 가져오기

**Files:**
- Create: `scripts/pick_worker_cycle.py`, `scripts/test_pick_worker_cycle.py` (작업 트리에 추적 안 된 파일로 이미 있음 — 원본과 같은지 확인 후 커밋)

**Interfaces:**
- Produces: `pick_worker_cycle.parse_args(argv)`, `run_cycle(worker, ...)`, `main(argv) -> int` (PR #8 그대로)

- [ ] **Step 1: 원본과 바이트 비교**

```bash
for f in scripts/pick_worker_cycle.py scripts/test_pick_worker_cycle.py; do
  git show origin/feature/nav-pick-worker:$f | cmp - $f && echo "same $f"
done
```
Expected: `same scripts/pick_worker_cycle.py`, `same scripts/test_pick_worker_cycle.py`. 다르면 `git show origin/feature/nav-pick-worker:$f > $f` 로 덮어쓴 뒤 다시 비교.

- [ ] **Step 2: 테스트 통과 확인**

Run: `PYTHONNOUSERSITE=1 env -u PYTHONPATH /home/roboseasy/miniforge3/envs/lerobot/bin/python -m pytest scripts/test_pick_worker_cycle.py -q`
Expected: 모두 PASS

- [ ] **Step 3: Commit**

```bash
git add scripts/pick_worker_cycle.py scripts/test_pick_worker_cycle.py
git commit -m "Add: PR #8 의 pick_worker_cycle.py 와 테스트를 원본 그대로 가져옴 (대시보드 카메라 스트림 기반)

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_011xipca4hVhjGeqXSUCLhrD"
```

---

### Task 2: 워커 상태에 보라색 비율·재시도 깊이·카메라 프레임 나이 추가

**Files:**
- Modify: `motion/services/pickplace/frame_stream.py` (끝에 클래스 추가)
- Modify: `motion/services/pickplace/headless_worker.py` (`run()` 루프 안 `self.status.set({...})` 부분, 약 372–386행, 그리고 루프 시작 전 초기화)
- Test: `motion/services/pickplace/test_frame_stream.py`, `motion/services/pickplace/test_headless_worker.py`

**Interfaces:**
- Produces:
  - `FrameAgeTracker().update(view: str, frame: np.ndarray, now: float) -> None`
  - `FrameAgeTracker().ages(now: float) -> dict[str, float]` (뷰별 "내용이 마지막으로 바뀐 뒤" 초, 소수 2자리)
  - 워커 `/status` 새 키: `"purple": {"front": {"ratio": float, "thr": float}, "wrist": {...}}`, `"retry_depth": float`, `"frame_age_s": {"front": float, "wrist": float}`
  - `ratio` 는 `max(left_ratio, right_ratio)` 가 아니라 **`min(left_ratio, right_ratio)`** (양쪽 다 넘어야 OK 인 기본 설정과 같은 기준)

- [ ] **Step 1: FrameAgeTracker 실패 테스트 작성** — `motion/services/pickplace/test_frame_stream.py` 끝에 추가

```python
import numpy as np

from services.pickplace.frame_stream import FrameAgeTracker


def test_frame_age_grows_while_frame_is_identical():
    t = FrameAgeTracker()
    f = np.zeros((48, 64, 3), dtype=np.uint8)
    t.update("front", f, now=10.0)
    t.update("front", f.copy(), now=12.5)
    assert t.ages(now=13.0) == {"front": 3.0}


def test_frame_age_resets_when_content_changes():
    t = FrameAgeTracker()
    a = np.zeros((48, 64, 3), dtype=np.uint8)
    b = a.copy()
    b[::4, ::4] = 200
    t.update("wrist", a, now=1.0)
    t.update("wrist", b, now=4.0)
    assert t.ages(now=4.5) == {"wrist": 0.5}


def test_frame_age_is_per_view():
    t = FrameAgeTracker()
    f = np.zeros((8, 8, 3), dtype=np.uint8)
    t.update("front", f, now=0.0)
    t.update("wrist", f, now=2.0)
    assert t.ages(now=3.0) == {"front": 3.0, "wrist": 1.0}
```

- [ ] **Step 2: 실패 확인**

Run: `PYTHONNOUSERSITE=1 env -u PYTHONPATH /home/roboseasy/miniforge3/envs/lerobot/bin/python -m pytest motion/services/pickplace/test_frame_stream.py -q`
Expected: FAIL — `ImportError: cannot import name 'FrameAgeTracker'`

- [ ] **Step 3: 구현** — `frame_stream.py` 끝에 추가 (파일 맨 위 import 에 `import hashlib` 추가)

```python
class FrameAgeTracker:
    """뷰별로 카메라 이미지 '내용'이 마지막으로 바뀐 시각을 기억한다.

    hz 는 멀쩡한데 같은 프레임이 반복되던 카메라 멈춤(2026-09-13 실기기)을 잡기 위한 것.
    축소(16칸 간격) 이미지의 해시만 비교해서 매 루프 비용을 작게 유지한다.
    """

    def __init__(self) -> None:
        self._hash: dict[str, bytes] = {}
        self._changed_at: dict[str, float] = {}

    def update(self, view: str, frame: "np.ndarray", now: float) -> None:
        digest = hashlib.blake2b(frame[::16, ::16].tobytes(), digest_size=8).digest()
        if self._hash.get(view) != digest:
            self._hash[view] = digest
            self._changed_at[view] = now

    def ages(self, now: float) -> dict[str, float]:
        return {v: round(now - t, 2) for v, t in self._changed_at.items()}
```

- [ ] **Step 4: 통과 확인**

Run: 같은 명령. Expected: PASS

- [ ] **Step 5: 워커 상태 필드 실패 테스트 작성** — `test_headless_worker.py` 끝에 추가

```python
def test_status_reports_frame_age_purple_and_retry_depth():
    import time as _time

    robot = FakeRobot()
    w = _worker(robot)
    w.start_background()                      # 워커는 일시정지로 시작 → 루프는 돌며 status 를 쓴다
    deadline = _time.time() + 3.0
    status = {}
    while _time.time() < deadline:
        status = w.status.get()
        if "frame_age_s" in status and status["frame_age_s"].get("front", 0) > 0.2:
            break
        _time.sleep(0.05)
    w.request_stop()
    w.join(timeout=5.0)

    assert set(status["frame_age_s"]) == {"front", "wrist"}
    assert status["frame_age_s"]["front"] > 0.2          # FakeRobot 은 같은 프레임만 준다
    assert set(status["purple"]) == {"front", "wrist"}
    assert status["purple"]["front"] == {"ratio": 0.0, "thr": w.cfg.check.min_ratio_for("front")}
    assert status["retry_depth"] == 0.0
```

- [ ] **Step 6: 실패 확인**

Run: `PYTHONNOUSERSITE=1 env -u PYTHONPATH /home/roboseasy/miniforge3/envs/lerobot/bin/python -m pytest motion/services/pickplace/test_headless_worker.py -q -k status_reports`
Expected: FAIL — `KeyError: 'frame_age_s'`

- [ ] **Step 7: 구현** — `headless_worker.py`
  1. import 줄 `from services.pickplace.frame_stream import ...` 에 `FrameAgeTracker` 추가.
  2. `run()` 에서 `hz = 0.0` 바로 아래에 `ages = FrameAgeTracker()` 추가.
  3. `frames_bgr = {...}` 블록 바로 아래에:

```python
                for v, img_bgr in frames_bgr.items():
                    ages.update(v, img_bgr, loop_start)
```
  4. `self.status.set({...})` 의 dict 에 `"target_class": target_class,` 다음 줄로 추가:

```python
                        "purple": {
                            v: {
                                "ratio": round(min(r.left_ratio, r.right_ratio), 3),
                                "thr": self.cfg.check.min_ratio_for(v),
                            }
                            for v, r in checker.results.items()
                        },
                        "retry_depth": round(arm.servo.retry_depth, 3) if arm.servo is not None else 0.0,
                        "frame_age_s": ages.ages(time.perf_counter()),
```

- [ ] **Step 8: 통과 + 기존 테스트 확인**

Run: `PYTHONNOUSERSITE=1 env -u PYTHONPATH /home/roboseasy/miniforge3/envs/lerobot/bin/python -m pytest motion scripts/test_pick_worker_cycle.py -q`
Expected: 전부 PASS (기존 웹 UI `motion/webui/test_app.py` 포함)

- [ ] **Step 9: Commit**

```bash
git add motion/services/pickplace/frame_stream.py motion/services/pickplace/headless_worker.py motion/services/pickplace/test_frame_stream.py motion/services/pickplace/test_headless_worker.py
git commit -m "Add: 워커 상태에 보라색 비율·재시도 깊이·카메라 프레임 나이 추가

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_011xipca4hVhjGeqXSUCLhrD"
```

---

### Task 3: `pick_worker_cycle.py --web-port` 카메라 스트림

**Files:**
- Modify: `scripts/pick_worker_cycle.py` (`parse_args`, 새 함수 `start_web`, `main` 의 워커 생성 직후)
- Test: `scripts/test_pick_worker_cycle.py` (새 클래스 `TestWeb`)

**Interfaces:**
- Consumes: `motion/webui/app.py` 의 `create_app(worker) -> FastAPI`
- Produces:
  - 인자 `--web-port INT` (기본 `None`), `--web-host STR` (기본 `"127.0.0.1"`)
  - `start_web(worker, host: str, port: int, app_factory=None, serve=None) -> bool` — 포트가 이미 쓰이면 경고 출력 후 `False`, 아니면 데몬 스레드로 서버 시작 후 `True`. `app_factory` 기본은 `webui.app.create_app`, `serve(app, host, port)` 기본은 `uvicorn.Server(uvicorn.Config(app, host=host, port=port, log_level="warning")).run()`.

- [ ] **Step 1: 실패 테스트 작성** — `scripts/test_pick_worker_cycle.py` 의 `if __name__ == "__main__":` 위에 추가

```python
class TestWeb(unittest.TestCase):
    def test_args_default_off_and_local_bind(self):
        a = pwc.parse_args(REQ)
        self.assertIsNone(a.web_port)
        self.assertEqual(a.web_host, "127.0.0.1")
        b = pwc.parse_args(REQ + ["--web-port", "8000", "--web-host", "0.0.0.0"])
        self.assertEqual((b.web_port, b.web_host), (8000, "0.0.0.0"))

    def test_start_web_runs_server_in_background_thread(self):
        import socket
        called = threading.Event()
        seen = {}

        def serve(app, host, port):
            seen.update(app=app, host=host, port=port)
            called.set()

        with socket.socket() as s:                       # 비어 있는 포트 하나 고르기
            s.bind(("127.0.0.1", 0))
            port = s.getsockname()[1]
        ok = pwc.start_web("W", "127.0.0.1", port, app_factory=lambda w: ("app", w), serve=serve)
        self.assertTrue(ok)
        self.assertTrue(called.wait(2.0))
        self.assertEqual(seen, {"app": ("app", "W"), "host": "127.0.0.1", "port": port})

    def test_start_web_returns_false_when_port_busy(self):
        import socket
        with socket.socket() as busy:
            busy.bind(("127.0.0.1", 0))
            busy.listen(1)
            port = busy.getsockname()[1]
            served = []
            ok = pwc.start_web("W", "127.0.0.1", port, app_factory=lambda w: w,
                               serve=lambda *a: served.append(a))
        self.assertFalse(ok)
        self.assertEqual(served, [])
```

- [ ] **Step 2: 실패 확인**

Run: `PYTHONNOUSERSITE=1 env -u PYTHONPATH /home/roboseasy/miniforge3/envs/lerobot/bin/python -m pytest scripts/test_pick_worker_cycle.py -q -k TestWeb`
Expected: FAIL — `AttributeError: 'Namespace' object has no attribute 'web_port'`

- [ ] **Step 3: 구현**
  1. `parse_args` 의 `--dry-run` 줄 아래에:

```python
    ap.add_argument("--web-port", type=int, default=None,
                    help="지정하면 집기 동안 카메라 스트림·상태 웹(motion/webui)을 이 포트로 띄운다")
    ap.add_argument("--web-host", default="127.0.0.1",
                    help="웹 바인드 주소. 다른 기기에서 볼 때만 0.0.0.0 (8000 에는 시작·재시작 버튼도 열린다)")
```
  2. `write_result` 함수 위에 추가 (`import socket` 을 파일 상단 import 에 추가):

```python
def start_web(worker, host: str, port: int, app_factory=None, serve=None) -> bool:
    """카메라 스트림 웹을 데몬 스레드로 띄운다. 실패해도 집기는 계속한다 -- 표시 실패가 미션 실패가 되면 안 된다."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        try:
            probe.bind((host, port))
        except OSError as exc:
            print(f"[pick_worker_cycle] 경고: 웹 포트 {host}:{port} 사용 불가 ({exc}) -- 카메라 스트림 없이 계속")
            return False
    if app_factory is None:
        from webui.app import create_app as app_factory
    if serve is None:
        def serve(app, host, port):
            import uvicorn
            uvicorn.Server(uvicorn.Config(app, host=host, port=port, log_level="warning")).run()
    app = app_factory(worker)
    threading.Thread(target=serve, args=(app, host, port), daemon=True, name="pick-web").start()
    print(f"[pick_worker_cycle] 카메라 스트림 http://{host}:{port}/stream/front")
    return True
```
  3. `main` 에서 `worker = PickPlaceHeadlessWorker(...)` 문장이 끝난 바로 다음 줄에:

```python
        if args.web_port:
            start_web(worker, args.web_host, args.web_port)
```

- [ ] **Step 4: 통과 확인**

Run: `PYTHONNOUSERSITE=1 env -u PYTHONPATH /home/roboseasy/miniforge3/envs/lerobot/bin/python -m pytest scripts/test_pick_worker_cycle.py -q`
Expected: 전부 PASS

- [ ] **Step 5: Commit**

```bash
git add scripts/pick_worker_cycle.py scripts/test_pick_worker_cycle.py
git commit -m "Add: pick_worker_cycle --web-port 로 집기 중 카메라 스트림 웹 띄우기 (기본 127.0.0.1)

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_011xipca4hVhjGeqXSUCLhrD"
```

---

### Task 4: 지도 정보·좌표 변환 (`map_geometry.py`)

**Files:**
- Create: `nav/dashboard/__init__.py` (빈 파일), `nav/dashboard/map_geometry.py`
- Test: `nav/dashboard/test_map_geometry.py`

**Interfaces:**
- Produces (모두 ROS 없음):
  - `Pose2D = tuple[float, float, float]` — (x m, y m, yaw rad)
  - `@dataclass(frozen=True) MapInfo(resolution: float, origin_x: float, origin_y: float, width: int, height: int, image_path: Path)`
  - `load_map_info(yaml_path: str | Path) -> MapInfo`
  - `map_png_bytes(info: MapInfo) -> bytes`
  - `load_waypoints(path: str | Path) -> dict[str, dict[str, float]]` — `{"home": {"x": 0.0, "y": 0.0, "yaw": 0.0}}` (yaw 는 **도**, 파일 그대로)
  - `world_to_pixel(info: MapInfo, x: float, y: float) -> tuple[float, float]`
  - `yaw_from_quat(x: float, y: float, z: float, w: float) -> float`
  - `compose(a: Pose2D, b: Pose2D) -> Pose2D` — a 프레임에서 본 b 를 a 의 부모 프레임으로
  - `scan_points(ranges, angle_min, angle_increment, range_min, range_max, robot: Pose2D, lidar: Pose2D = (0.0, 0.0, 0.0)) -> list[list[float]]` — map 좌표 `[x, y]` (소수 3자리), 유효빔만
  - `decimate(points: list, max_n: int) -> list` — 균등 간격으로 최대 `max_n` 개

- [ ] **Step 1: 실패 테스트 작성** — `nav/dashboard/test_map_geometry.py`

```python
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import pytest  # noqa: E402

import map_geometry as mg  # noqa: E402

REPO = Path(__file__).resolve().parents[2]


def _write_map(tmp_path, w=4, h=3):
    pgm = tmp_path / "m.pgm"
    pgm.write_bytes(b"P5\n%d %d\n255\n" % (w, h) + bytes([254] * (w * h)))
    y = tmp_path / "m.yaml"
    y.write_text("image: m.pgm\nresolution: 0.050\norigin: [-0.707, -0.919, 0]\n")
    return y


def test_load_map_info_reads_yaml_and_pgm_size(tmp_path):
    info = mg.load_map_info(_write_map(tmp_path))
    assert (info.width, info.height) == (4, 3)
    assert info.resolution == pytest.approx(0.05)
    assert (info.origin_x, info.origin_y) == (pytest.approx(-0.707), pytest.approx(-0.919))
    assert info.image_path.name == "m.pgm"


def test_real_lekiwi01_map():
    info = mg.load_map_info(REPO / "nav/maps/lekiwi01/map_0912_1654.yaml")
    assert (info.width, info.height) == (53, 48)


def test_map_png_bytes_is_png(tmp_path):
    data = mg.map_png_bytes(mg.load_map_info(_write_map(tmp_path)))
    assert data[:8] == b"\x89PNG\r\n\x1a\n"


def test_world_to_pixel_flips_y(tmp_path):
    info = mg.load_map_info(_write_map(tmp_path))          # origin (-0.707,-0.919), h=3
    px, py = mg.world_to_pixel(info, -0.707, -0.919)       # 원점 = 왼쪽 아래 모서리
    assert (px, py) == (pytest.approx(0.0), pytest.approx(3.0))
    px, py = mg.world_to_pixel(info, -0.707 + 0.10, -0.919 + 0.05)
    assert (px, py) == (pytest.approx(2.0), pytest.approx(2.0))


def test_load_waypoints_real_file():
    wp = mg.load_waypoints(REPO / "nav/mission/waypoints.yaml")
    assert wp["home"] == {"x": 0.0, "y": 0.0, "yaw": 0.0}
    assert wp["center"]["x"] == pytest.approx(1.10)


def test_yaw_from_quat_90deg():
    s = math.sin(math.pi / 4)
    assert mg.yaw_from_quat(0.0, 0.0, s, s) == pytest.approx(math.pi / 2)


def test_compose_rotates_child_offset():
    a = (1.0, 2.0, math.pi / 2)
    b = (1.0, 0.0, 0.1)
    x, y, yaw = mg.compose(a, b)
    assert (x, y, yaw) == (pytest.approx(1.0), pytest.approx(3.0), pytest.approx(math.pi / 2 + 0.1))


def test_scan_points_filters_invalid_and_transforms():
    ranges = [1.0, float("inf"), 0.01, 2.0, float("nan")]
    pts = mg.scan_points(ranges, angle_min=0.0, angle_increment=math.pi / 2,
                         range_min=0.05, range_max=6.0, robot=(1.0, 1.0, math.pi / 2))
    # 빔0: 전방 1 m, 로봇이 +90° → map 에서 +y 로 1 m.  빔3: 각도 270°(=-90°) 2 m → map +x 로 2 m
    assert pts == [[1.0, 2.0], [3.0, 1.0]]


def test_decimate_even_spacing():
    pts = [[i, 0] for i in range(10)]
    assert mg.decimate(pts, 5) == [[0, 0], [2, 0], [4, 0], [6, 0], [8, 0]]
    assert mg.decimate(pts, 20) == pts
```

- [ ] **Step 2: 실패 확인**

Run: `/usr/bin/python3 -m pytest nav/dashboard/test_map_geometry.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'map_geometry'`

- [ ] **Step 3: 구현** — `nav/dashboard/map_geometry.py` (그리고 빈 `nav/dashboard/__init__.py`)

```python
"""지도 파일·좌표 변환. ROS 없이 테스트한다.

좌표 규약 (ROS map_server 와 같음)
  - yaml origin 은 pgm 왼쪽 아래 모서리의 map 좌표
  - pgm 첫 행이 y 최대 → 픽셀 py = height - (y - origin_y) / res
"""
from __future__ import annotations

import io
import math
from dataclasses import dataclass
from pathlib import Path

import yaml
from PIL import Image

Pose2D = tuple[float, float, float]


@dataclass(frozen=True)
class MapInfo:
    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int
    image_path: Path


def load_map_info(yaml_path) -> MapInfo:
    yaml_path = Path(yaml_path)
    y = yaml.safe_load(yaml_path.read_text())
    image = (yaml_path.parent / y["image"]).resolve()
    with Image.open(image) as im:
        width, height = im.size
    ox, oy = float(y["origin"][0]), float(y["origin"][1])
    return MapInfo(float(y["resolution"]), ox, oy, width, height, image)


def map_png_bytes(info: MapInfo) -> bytes:
    buf = io.BytesIO()
    with Image.open(info.image_path) as im:
        im.convert("L").save(buf, format="PNG")
    return buf.getvalue()


def load_waypoints(path) -> dict[str, dict[str, float]]:
    data = yaml.safe_load(Path(path).read_text()) or {}
    return {
        name: {"x": float(p.get("x", 0.0)), "y": float(p.get("y", 0.0)), "yaw": float(p.get("yaw", 0.0))}
        for name, p in (data.get("points") or {}).items()
    }


def world_to_pixel(info: MapInfo, x: float, y: float) -> tuple[float, float]:
    return (x - info.origin_x) / info.resolution, info.height - (y - info.origin_y) / info.resolution


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def compose(a: Pose2D, b: Pose2D) -> Pose2D:
    ax, ay, at = a
    bx, by, bt = b
    c, s = math.cos(at), math.sin(at)
    return ax + c * bx - s * by, ay + s * bx + c * by, at + bt


def scan_points(ranges, angle_min, angle_increment, range_min, range_max,
                robot: Pose2D, lidar: Pose2D = (0.0, 0.0, 0.0)) -> list[list[float]]:
    sensor = compose(robot, lidar)
    out = []
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r < range_min or r > range_max:
            continue
        th = angle_min + i * angle_increment
        x, y, _ = compose(sensor, (r * math.cos(th), r * math.sin(th), 0.0))
        out.append([round(x, 3), round(y, 3)])
    return out


def decimate(points: list, max_n: int) -> list:
    if len(points) <= max_n:
        return list(points)
    step = len(points) / max_n
    return [points[int(i * step)] for i in range(max_n)]
```

- [ ] **Step 4: 통과 확인**

Run: `/usr/bin/python3 -m pytest nav/dashboard/test_map_geometry.py -q`
Expected: 9 passed

- [ ] **Step 5: Commit**

```bash
git add nav/dashboard/__init__.py nav/dashboard/map_geometry.py nav/dashboard/test_map_geometry.py
git commit -m "Add: 대시보드 지도 정보·좌표 변환 (map_geometry)

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_011xipca4hVhjGeqXSUCLhrD"
```

---

### Task 5: 미션 단계 (`mission_timeline.py`)

**Files:**
- Create: `nav/dashboard/mission_timeline.py`
- Test: `nav/dashboard/test_mission_timeline.py`

**Interfaces:**
- Produces:
  - `MissionTimeline()`
  - `.on_command(text: str, now: float) -> None` — `fetch <목적지> [color:<색>] [to <복귀지>]` 이면 새 미션 시작(궤적 초기화 신호로도 쓰임). 그 외 명령은 `command` 만 기록.
  - `.on_state(state: str, now: float, stop_latched: bool = False) -> None`
  - `.on_status(text: str) -> None`
  - `.is_new_mission_since(t: float) -> bool` — 마지막 fetch 시각이 `t` 이후면 True (궤적 비우기 판단용)
  - `.snapshot(now: float) -> dict` — `{"state", "status", "command", "target", "color", "elapsed", "stages": [{"key", "label", "state", "sec"}]}`; 단계 `state` 는 `"todo" | "active" | "done" | "failed"`; `sec` 은 todo 면 `None`; 미션이 없으면 `elapsed` 는 `None`, `stages` 는 `[]`
- 판정 규칙 (브리지 실제 동작 기준):
  - `moving`: 아직 아무 단계도 시작 안 했으면 drive 시작. **복귀 중 피드백도 `moving` 으로 오므로** 그 외에는 무시.
  - `picking`: drive 가 active 면 done, pick active.
  - `returning`: pick 이 active 면 done, return active (이미 active 면 무시 — 위치 다듬기 중 여러 번 옴).
  - `done`: active 단계 done, 미션 종료.
  - `failed` / `rejected` / `canceled`: active 단계 failed, 미션 종료.
  - `idle`: `stop_latched=True` 면 무시. 아니면 무시(브리지 "가고 있지 않아요" 응답일 뿐) — 단, `state` 필드는 갱신한다.

- [ ] **Step 1: 실패 테스트 작성** — `nav/dashboard/test_mission_timeline.py`

```python
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from mission_timeline import MissionTimeline  # noqa: E402


def _stages(tl, now):
    return [(s["key"], s["state"], s["sec"]) for s in tl.snapshot(now)["stages"]]


def test_no_mission_snapshot():
    snap = MissionTimeline().snapshot(5.0)
    assert snap["stages"] == [] and snap["elapsed"] is None and snap["target"] is None


def test_success_flow_with_moving_feedback_during_return():
    tl = MissionTimeline()
    tl.on_command("fetch center color:blue", 100.0)
    tl.on_state("moving", 100.5)
    tl.on_state("moving", 110.0)          # 주행 피드백
    tl.on_state("arrived", 129.0)
    tl.on_state("picking", 140.0)
    tl.on_state("returning", 212.0)
    tl.on_state("returning", 222.0)       # 위치 다듬기 후 한 번 더
    tl.on_state("moving", 230.0)          # 복귀 주행 피드백 — drive 를 다시 시작하면 안 됨
    assert _stages(tl, 240.0) == [("drive", "done", 39.5), ("pick", "done", 72.0), ("return", "active", 28.0)]
    tl.on_state("done", 255.0)
    snap = tl.snapshot(300.0)
    assert _stages(tl, 300.0) == [("drive", "done", 39.5), ("pick", "done", 72.0), ("return", "done", 43.0)]
    assert snap["elapsed"] == 155.0 and snap["target"] == "center" and snap["color"] == "blue"
    assert snap["stages"][0]["label"] == "Nav2 주행 home → center"
    assert snap["stages"][2]["label"] == "복귀 center → home"


def test_failure_marks_active_stage():
    tl = MissionTimeline()
    tl.on_command("fetch center color:green", 0.0)
    tl.on_state("moving", 1.0)
    tl.on_state("failed", 9.0)
    assert _stages(tl, 20.0) == [("drive", "failed", 8.0), ("pick", "todo", None), ("return", "todo", None)]
    assert tl.snapshot(20.0)["elapsed"] == 9.0


def test_idle_reply_to_stop_does_not_change_stages():
    tl = MissionTimeline()
    tl.on_command("fetch center", 0.0)
    tl.on_state("moving", 1.0)
    tl.on_state("picking", 5.0)
    tl.on_state("idle", 6.0, stop_latched=True)
    assert _stages(tl, 7.0)[1] == ("pick", "active", 2.0)
    assert tl.snapshot(7.0)["state"] == "idle"


def test_new_fetch_resets_and_return_target_parsed():
    tl = MissionTimeline()
    tl.on_command("fetch center", 0.0)
    tl.on_state("moving", 1.0)
    tl.on_state("canceled", 2.0)
    tl.on_command("fetch center color:red to home2", 50.0)
    snap = tl.snapshot(51.0)
    assert [s["state"] for s in snap["stages"]] == ["todo", "todo", "todo"]
    assert snap["stages"][2]["label"] == "복귀 center → home2"
    assert tl.is_new_mission_since(10.0) and not tl.is_new_mission_since(60.0)


def test_non_fetch_command_only_recorded():
    tl = MissionTimeline()
    tl.on_command("stop", 3.0)
    snap = tl.snapshot(4.0)
    assert snap["command"] == "stop" and snap["stages"] == []
```

- [ ] **Step 2: 실패 확인**

Run: `/usr/bin/python3 -m pytest nav/dashboard/test_mission_timeline.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'mission_timeline'`

- [ ] **Step 3: 구현** — `nav/dashboard/mission_timeline.py`

```python
"""브리지 /abo/state 전이로 미션 단계(주행 → 집기 → 복귀)를 만든다. ROS 없음."""
from __future__ import annotations

END_OK = {"done"}
END_FAIL = {"failed", "rejected", "canceled"}


class _Stage:
    def __init__(self, key: str, label: str):
        self.key, self.label = key, label
        self.state = "todo"
        self.start: float | None = None
        self.end: float | None = None

    def begin(self, now: float) -> None:
        self.state, self.start = "active", now

    def finish(self, now: float, ok: bool) -> None:
        self.state, self.end = ("done" if ok else "failed"), now

    def as_dict(self, now: float) -> dict:
        sec = None if self.start is None else round((self.end if self.end is not None else now) - self.start, 1)
        return {"key": self.key, "label": self.label, "state": self.state, "sec": sec}


class MissionTimeline:
    def __init__(self) -> None:
        self.state: str | None = None
        self.status = ""
        self.command = ""
        self.target: str | None = None
        self.color: str | None = None
        self.started_at: float | None = None
        self.ended_at: float | None = None
        self.stages: list[_Stage] = []

    def on_command(self, text: str, now: float) -> None:
        self.command = text.strip()
        tok = self.command.split()
        if not tok or tok[0].lower() != "fetch" or len(tok) < 2:
            return
        self.target = tok[1]
        self.color = next((t.split(":", 1)[1] for t in tok[2:] if t.lower().startswith("color:")), None)
        ret = tok[tok.index("to") + 1] if "to" in tok[2:-1] else "home"
        self.started_at, self.ended_at = now, None
        self.stages = [
            _Stage("drive", f"Nav2 주행 {ret} → {self.target}"),
            _Stage("pick", "집기"),
            _Stage("return", f"복귀 {self.target} → {ret}"),
        ]

    def is_new_mission_since(self, t: float) -> bool:
        return self.started_at is not None and self.started_at > t

    def on_status(self, text: str) -> None:
        self.status = text

    def _active(self) -> _Stage | None:
        return next((s for s in self.stages if s.state == "active"), None)

    def on_state(self, state: str, now: float, stop_latched: bool = False) -> None:
        self.state = state
        if not self.stages or self.ended_at is not None:
            return
        drive, pick, ret = self.stages
        active = self._active()
        if state == "moving" and all(s.state == "todo" for s in self.stages):
            drive.begin(now)
        elif state == "picking" and pick.state == "todo":
            if drive.state == "active":
                drive.finish(now, True)
            pick.begin(now)
        elif state == "returning" and ret.state == "todo":
            if pick.state == "active":
                pick.finish(now, True)
            ret.begin(now)
        elif state in END_OK | END_FAIL:
            if active is not None:
                active.finish(now, state in END_OK)
            self.ended_at = now

    def snapshot(self, now: float) -> dict:
        elapsed = None
        if self.started_at is not None:
            elapsed = round((self.ended_at if self.ended_at is not None else now) - self.started_at, 1)
        return {
            "state": self.state, "status": self.status, "command": self.command,
            "target": self.target, "color": self.color, "elapsed": elapsed,
            "stages": [s.as_dict(now) for s in self.stages],
        }
```

- [ ] **Step 4: 통과 확인**

Run: `/usr/bin/python3 -m pytest nav/dashboard/test_mission_timeline.py -q`
Expected: 6 passed

- [ ] **Step 5: Commit**

```bash
git add nav/dashboard/mission_timeline.py nav/dashboard/test_mission_timeline.py
git commit -m "Add: 대시보드 미션 단계 판정 (mission_timeline)

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_011xipca4hVhjGeqXSUCLhrD"
```

---

### Task 6: 토픽 주기·나이·마지막 값 (`topic_stats.py`)

**Files:**
- Create: `nav/dashboard/topic_stats.py`
- Test: `nav/dashboard/test_topic_stats.py`

**Interfaces:**
- Produces:
  - `TopicStats(rate_window_s: float = 5.0)`
  - `.declare(topic: str, direction: str) -> None` — `direction` 은 `"in"` 또는 `"out"`. 한 번도 안 받은 토픽도 패널에 뜨게 한다.
  - `.record(topic: str, now: float, summary: str) -> None` — `summary` 는 사람이 읽을 짧은 문자열 (예: `"720빔 · 유효 407"`)
  - `.snapshot(now: float) -> dict[str, dict]` — `{topic: {"dir": str, "rate": float, "age": float | None, "last": str}}`, 선언 순서 유지
- 규칙: `rate` = 최근 `rate_window_s` 초 안에 받은 개수 / `rate_window_s` (소수 1자리). 끊기면 자연히 0 으로 떨어진다. `age` 는 소수 2자리.

- [ ] **Step 1: 실패 테스트 작성** — `nav/dashboard/test_topic_stats.py`

```python
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from topic_stats import TopicStats  # noqa: E402


def test_declared_but_never_received():
    ts = TopicStats()
    ts.declare("/scan", "in")
    assert ts.snapshot(10.0) == {"/scan": {"dir": "in", "rate": 0.0, "age": None, "last": ""}}


def test_rate_and_age_over_window():
    ts = TopicStats(rate_window_s=5.0)
    ts.declare("/odom", "in")
    for i in range(30):                      # 6 Hz 로 5 초
        ts.record("/odom", 100.0 + i / 6.0, f"n={i}")
    snap = ts.snapshot(105.0)["/odom"]
    assert snap["rate"] == 6.0
    assert snap["age"] == round(105.0 - (100.0 + 29 / 6.0), 2)
    assert snap["last"] == "n=29"


def test_rate_drops_to_zero_when_stale():
    ts = TopicStats(rate_window_s=5.0)
    ts.declare("/scan", "in")
    ts.record("/scan", 1.0, "x")
    assert ts.snapshot(20.0)["/scan"]["rate"] == 0.0
    assert ts.snapshot(20.0)["/scan"]["age"] == 19.0


def test_undeclared_topic_is_added_as_in_and_order_kept():
    ts = TopicStats()
    ts.declare("/abo/command", "out")
    ts.record("/tf", 1.0, "map→odom")
    assert list(ts.snapshot(2.0)) == ["/abo/command", "/tf"]
    assert ts.snapshot(2.0)["/tf"]["dir"] == "in"
```

- [ ] **Step 2: 실패 확인**

Run: `/usr/bin/python3 -m pytest nav/dashboard/test_topic_stats.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'topic_stats'`

- [ ] **Step 3: 구현** — `nav/dashboard/topic_stats.py`

```python
"""토픽별 수신 주기·나이·마지막 값 요약. 시각은 노트북이 받은 시각. ROS 없음."""
from __future__ import annotations

from collections import deque


class _Entry:
    def __init__(self, direction: str):
        self.dir = direction
        self.times: deque[float] = deque(maxlen=400)
        self.last = ""


class TopicStats:
    def __init__(self, rate_window_s: float = 5.0) -> None:
        self.window = rate_window_s
        self._topics: dict[str, _Entry] = {}

    def declare(self, topic: str, direction: str) -> None:
        self._topics.setdefault(topic, _Entry(direction))

    def record(self, topic: str, now: float, summary: str) -> None:
        e = self._topics.setdefault(topic, _Entry("in"))
        e.times.append(now)
        e.last = summary

    def snapshot(self, now: float) -> dict[str, dict]:
        out = {}
        for topic, e in self._topics.items():
            recent = sum(1 for t in e.times if now - self.window <= t <= now)
            out[topic] = {
                "dir": e.dir,
                "rate": round(recent / self.window, 1),
                "age": round(now - e.times[-1], 2) if e.times else None,
                "last": e.last,
            }
        return out
```

- [ ] **Step 4: 통과 확인**

Run: `/usr/bin/python3 -m pytest nav/dashboard/test_topic_stats.py -q`
Expected: 4 passed

- [ ] **Step 5: Commit**

```bash
git add nav/dashboard/topic_stats.py nav/dashboard/test_topic_stats.py
git commit -m "Add: 대시보드 토픽 주기·나이 통계 (topic_stats)

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_011xipca4hVhjGeqXSUCLhrD"
```

---

### Task 7: 정지 유지 판단 (`stop_latch.py`)

**Files:**
- Create: `nav/dashboard/stop_latch.py`
- Test: `nav/dashboard/test_stop_latch.py`

**Interfaces:**
- Produces:
  - `StopLatch(window_s: float = 5.0, interval_s: float = 0.5)`
  - `.engage(now: float) -> list[str]` — 이미 켜져 있어도 호출 가능. 항상 `["estop", "stop"]` 반환 (호출 측이 8000 이 안 떠 있으면 estop 은 건너뜀)
  - `.release() -> None`
  - `.on_bridge_state(state: str, now: float) -> None` — 켜짐 여부와 무관하게 상태를 추적한다
  - `.on_pick_reachable(reachable: bool) -> list[str]` — 켜진 상태에서 False→True 로 바뀌면 `["estop"]`, 아니면 `[]`
  - `.tick(now: float) -> list[str]` — 지금 `stop` 을 보내야 하면 `["stop"]`
  - `.note(action: str, result: str) -> None` — 마지막 결과 문자열 기록 (예: `"stop 발행"`, `"estop 실패: timed out"`)
  - `.snapshot() -> dict` — `{"latched": bool, "since": float | None, "sent": int, "last_result": str | None}`
- 규칙:
  - 창(window)은 켜져 있는 동안 브리지 상태가 **다른 상태에서** `moving` 또는 `returning` 으로 바뀐 시각에 새로 열린다 (`moving`→`moving` 피드백은 다시 열지 않음, `returning`→`moving` 은 다시 엶 — 복귀는 `returning` 후 위치 다듬기 약 10 초 뒤에 `moving` 이 오기 때문).
  - 켤 때 브리지 상태가 이미 `moving`/`returning` 이면 그 시각에 창을 연다.
  - 창이 열린 뒤 `window_s` 초 이내에서, 마지막 `stop` 이후 `interval_s` 이상 지났으면 `["stop"]`.
  - `canceled`, `picking`, `arrived`, `done`, `failed`, `idle`, `rejected` 를 받으면 창을 닫는다.
  - `engage` 가 보낸 `stop` 도 `sent` 와 "마지막 stop 시각"에 포함한다.

- [ ] **Step 1: 실패 테스트 작성** — `nav/dashboard/test_stop_latch.py`

```python
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from stop_latch import StopLatch  # noqa: E402


def test_not_latched_never_sends():
    s = StopLatch()
    s.on_bridge_state("moving", 1.0)
    assert s.tick(1.0) == [] and s.tick(2.0) == []
    assert s.on_pick_reachable(True) == []


def test_engage_sends_estop_and_stop_once():
    s = StopLatch()
    assert s.engage(0.0) == ["estop", "stop"]
    assert s.snapshot() == {"latched": True, "since": 0.0, "sent": 1, "last_result": None}
    assert s.tick(0.1) == []


def test_no_resend_while_picking():
    s = StopLatch()
    s.on_bridge_state("picking", 0.0)
    s.engage(1.0)
    assert [s.tick(t) for t in (2.0, 3.0, 4.0)] == [[], [], []]


def test_return_after_pick_resends_every_half_second_for_five_seconds():
    s = StopLatch()
    s.on_bridge_state("picking", 0.0)
    s.engage(1.0)
    s.on_bridge_state("returning", 10.0)
    assert s.tick(10.0) == ["stop"]
    assert s.tick(10.2) == []
    assert s.tick(10.5) == ["stop"]
    assert s.tick(15.0) == ["stop"]          # 창 끝(10+5) 포함
    assert s.tick(15.6) == []                # 창 지남
    s.on_bridge_state("moving", 20.0)        # 위치 다듬기 뒤 실제 복귀 주행 시작 → 창 다시 열림
    assert s.tick(20.0) == ["stop"]
    s.on_bridge_state("moving", 21.0)        # 피드백은 창을 다시 열지 않음
    assert s.tick(25.1) == []
    assert s.snapshot()["sent"] == 1 + 3 + 1


def test_canceled_closes_window():
    s = StopLatch()
    s.engage(0.0)
    s.on_bridge_state("moving", 30.0)
    assert s.tick(30.0) == ["stop"]
    s.on_bridge_state("canceled", 30.3)
    assert s.tick(30.8) == []


def test_engage_while_already_moving_opens_window():
    s = StopLatch()
    s.on_bridge_state("moving", 0.0)
    assert s.engage(3.0) == ["estop", "stop"]
    assert s.tick(3.2) == []
    assert s.tick(3.5) == ["stop"]


def test_idle_reply_then_moving_feedback_reopens():
    s = StopLatch()
    s.engage(0.0)
    s.on_bridge_state("moving", 1.0)          # 목표 수락 전에 stop 이 도착 → 브리지는 idle 응답
    s.on_bridge_state("idle", 1.1)
    assert s.tick(1.6) == []
    s.on_bridge_state("moving", 4.0)          # 3 초 뒤 주행 피드백
    assert s.tick(4.0) == ["stop"]


def test_pick_web_coming_up_while_latched_requests_estop():
    s = StopLatch()
    assert s.on_pick_reachable(False) == []
    s.engage(0.0)
    assert s.on_pick_reachable(True) == ["estop"]
    assert s.on_pick_reachable(True) == []


def test_release_stops_everything_and_note_records_result():
    s = StopLatch()
    s.engage(0.0)
    s.note("estop", "estop 실패: timed out")
    assert s.snapshot()["last_result"] == "estop 실패: timed out"
    s.release()
    s.on_bridge_state("moving", 5.0)
    assert s.tick(5.0) == []
    assert s.snapshot()["latched"] is False
```

- [ ] **Step 2: 실패 확인**

Run: `/usr/bin/python3 -m pytest nav/dashboard/test_stop_latch.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'stop_latch'`

- [ ] **Step 3: 구현** — `nav/dashboard/stop_latch.py`

```python
"""정지 유지(stop latch) 판단. ROS·HTTP 호출은 하지 않고 '무엇을 보낼지'만 돌려준다.

로봇 브리지(PR #8)는 Nav2 목표가 있을 때만 stop 을 받아들인다. 집기 중에 멈추면
실패로 보고 복귀 주행을 시작하므로, 켜져 있는 동안 주행이 시작될 때마다 stop 을
다시 보낸다. 설계 문서 §정지 참고.
"""
from __future__ import annotations

MOVE = {"moving", "returning"}


class StopLatch:
    def __init__(self, window_s: float = 5.0, interval_s: float = 0.5) -> None:
        self.window_s, self.interval_s = window_s, interval_s
        self.latched = False
        self.since: float | None = None
        self.sent = 0
        self.last_result: str | None = None
        self._state: str | None = None
        self._window_start: float | None = None
        self._last_stop: float | None = None
        self._pick_up = False

    def engage(self, now: float) -> list[str]:
        if not self.latched:
            self.latched, self.since = True, now
        if self._state in MOVE:
            self._window_start = now
        self._last_stop = now
        self.sent += 1
        return ["estop", "stop"]

    def release(self) -> None:
        self.latched = False
        self.since = None
        self._window_start = None

    def on_bridge_state(self, state: str, now: float) -> None:
        if state in MOVE and state != self._state:
            self._window_start = now
        elif state not in MOVE:
            self._window_start = None
        self._state = state

    def on_pick_reachable(self, reachable: bool) -> list[str]:
        rising = reachable and not self._pick_up
        self._pick_up = reachable
        return ["estop"] if (rising and self.latched) else []

    def tick(self, now: float) -> list[str]:
        if not self.latched or self._window_start is None:
            return []
        if now - self._window_start > self.window_s:
            return []
        if self._last_stop is not None and now - self._last_stop < self.interval_s:
            return []
        self._last_stop = now
        self.sent += 1
        return ["stop"]

    def note(self, action: str, result: str) -> None:
        self.last_result = result

    def snapshot(self) -> dict:
        return {"latched": self.latched, "since": self.since, "sent": self.sent, "last_result": self.last_result}
```

- [ ] **Step 4: 통과 확인**

Run: `/usr/bin/python3 -m pytest nav/dashboard/test_stop_latch.py -q`
Expected: 9 passed

- [ ] **Step 5: Commit**

```bash
git add nav/dashboard/stop_latch.py nav/dashboard/test_stop_latch.py
git commit -m "Add: 대시보드 정지 유지 판단 (stop_latch)

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_011xipca4hVhjGeqXSUCLhrD"
```

---

### Task 8: 스레드 안전 상태와 스냅샷 (`dashboard_state.py`)

**Files:**
- Create: `nav/dashboard/dashboard_state.py`
- Test: `nav/dashboard/test_dashboard_state.py`

**Interfaces:**
- Consumes: `map_geometry.MapInfo`, `compose`, `scan_points`, `decimate` (Task 4) · `MissionTimeline` (Task 5) · `TopicStats` (Task 6) · `StopLatch` (Task 7)
- Produces: `DashboardState(info: MapInfo, lidar: Pose2D = (0.0, 0.0, 0.0), max_particles: int = 800, trail_max: int = 600, trail_step_m: float = 0.05)` — 모든 메서드는 내부 락을 잡는다. ROS 메시지를 직접 받지 않고 **이미 꺼낸 값**을 받는다(ROS 없이 테스트).
  - `on_tf(parent: str, child: str, pose: Pose2D, now: float) -> None` — `("map","odom")`, `("odom","base_link")` 만 저장, 그 외는 토픽 통계만
  - `on_odom(pose_in_odom: Pose2D, now: float) -> None`
  - `on_scan(ranges, angle_min, angle_increment, range_min, range_max, now: float) -> None`
  - `on_particles(xy: list[list[float]], now: float) -> None`
  - `on_amcl_pose(x: float, y: float, cov_xy: float, now: float) -> None`
  - `on_plan(xy: list[list[float]], now: float) -> None`
  - `on_map_msg(width: int, height: int, resolution: float, origin_x: float, origin_y: float, now: float) -> None`
  - `on_bridge_state(state: str, now: float)`, `on_bridge_status(text: str, now: float)`, `on_command(text: str, now: float)`, `on_pick_request(text: str, now: float)`, `on_pick_done(ok: bool, now: float)`
  - `set_pick(reachable: bool, status: dict | None, now: float) -> list[str]` — 정지 유지 중 8000 이 새로 뜨면 `["estop"]`
  - `engage_stop(now: float) -> list[str]`, `release_stop() -> None`, `stop_tick(now: float) -> list[str]`, `note_stop(action: str, result: str) -> None`
  - `snapshot(now: float) -> dict` — 설계 문서 §스냅샷 JSON 의 키 전부: `t, pose, scan, particles, plan, trail, mission, stop, pick, topics, warnings`
- 스냅샷 규칙:
  - `pose`: map→odom 과 odom→base_link 둘 다 있으면 `{"x","y","yaw","age","cov_xy"}` (소수 3자리, `age` = 둘 중 **더 오래된** 수신 기준), 아니면 `None`
  - `scan`/`particles`/`plan`: 받은 적 없으면 `{"pts": [], "age": None}` (`particles` 는 `"n": 0` 추가)
  - 스캔은 받은 순간의 로봇 자세로 변환해 저장한다. 자세가 없으면 점은 비우고 토픽 통계만 남긴다.
  - 궤적: odom 수신마다 map 좌표로 변환, 마지막 점과 `trail_step_m` 미만이면 건너뜀, 최대 `trail_max`. `fetch` 명령을 받으면 비운다.
  - `/map` 메시지의 크기·해상도·원점(허용 1 mm)이 파일과 다르면 `warnings` 에 `"/map 이 파일 지도와 다름 (WxH, res, origin)"` 한 줄
  - 토픽 선언 순서: in = `/abo/state, /abo/status, /abo/pick_request, /abo/pick_done, /tf, /odom, /scan, /particle_cloud, /amcl_pose, /plan, /map`; out = `/abo/command`

- [ ] **Step 1: 실패 테스트 작성** — `nav/dashboard/test_dashboard_state.py`

```python
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import pytest  # noqa: E402

from dashboard_state import DashboardState  # noqa: E402
from map_geometry import MapInfo  # noqa: E402

INFO = MapInfo(0.05, -0.707, -0.919, 53, 48, Path("/nonexistent.pgm"))


def _with_pose(now=0.0):
    s = DashboardState(INFO)
    s.on_tf("map", "odom", (1.0, 0.0, math.pi / 2), now)
    s.on_tf("odom", "base_link", (1.0, 0.0, 0.0), now)
    return s


def test_empty_snapshot_is_json_and_has_all_keys():
    snap = DashboardState(INFO).snapshot(1.0)
    assert set(snap) == {"t", "pose", "scan", "particles", "plan", "trail", "mission", "stop", "pick", "topics", "warnings"}
    assert snap["pose"] is None
    assert snap["scan"] == {"pts": [], "age": None}
    assert snap["particles"] == {"pts": [], "n": 0, "age": None}
    assert snap["pick"] == {"reachable": False, "status": None, "age": None}
    assert list(snap["topics"])[:4] == ["/abo/state", "/abo/status", "/abo/pick_request", "/abo/pick_done"]
    assert snap["topics"]["/abo/command"]["dir"] == "out"
    json.dumps(snap)


def test_pose_is_composed_and_age_uses_older_transform():
    s = DashboardState(INFO)
    s.on_tf("map", "odom", (1.0, 0.0, math.pi / 2), 10.0)
    s.on_tf("odom", "base_link", (1.0, 0.0, 0.0), 12.0)
    s.on_amcl_pose(1.0, 1.0, 0.012, 12.0)
    p = s.snapshot(13.0)["pose"]
    assert (p["x"], p["y"]) == (pytest.approx(1.0), pytest.approx(1.0))
    assert p["yaw"] == pytest.approx(math.pi / 2, abs=1e-3)
    assert p["age"] == 3.0 and p["cov_xy"] == 0.012


def test_scan_uses_pose_at_receive_time():
    s = _with_pose()
    s.on_scan([1.0, float("inf")], 0.0, math.pi / 2, 0.05, 6.0, 0.5)
    snap = s.snapshot(1.0)
    assert snap["scan"] == {"pts": [[1.0, 2.0]], "age": 0.5}
    assert snap["topics"]["/scan"]["last"] == "2빔 · 유효 1"


def test_scan_without_pose_keeps_stats_only():
    s = DashboardState(INFO)
    s.on_scan([1.0], 0.0, 0.1, 0.05, 6.0, 1.0)
    snap = s.snapshot(2.0)
    assert snap["scan"]["pts"] == [] and snap["topics"]["/scan"]["age"] == 1.0


def test_particles_decimated():
    s = DashboardState(INFO, max_particles=3)
    s.on_particles([[i, 0.0] for i in range(9)], 1.0)
    snap = s.snapshot(1.5)["particles"]
    assert snap == {"pts": [[0, 0.0], [3, 0.0], [6, 0.0]], "n": 9, "age": 0.5}


def test_trail_step_and_reset_on_fetch():
    s = _with_pose()
    for i, x in enumerate([0.0, 0.01, 0.2, 0.21, 0.5]):
        s.on_odom((x, 0.0, 0.0), float(i))
    assert len(s.snapshot(5.0)["trail"]) == 3              # 0.0, 0.2, 0.5 (5 cm 미만 건너뜀)
    s.on_command("fetch center color:blue", 6.0)
    assert s.snapshot(6.0)["trail"] == []
    assert s.snapshot(6.0)["mission"]["color"] == "blue"


def test_map_mismatch_warning():
    s = DashboardState(INFO)
    s.on_map_msg(53, 48, 0.05, -0.707, -0.919, 1.0)
    assert s.snapshot(1.0)["warnings"] == []
    s.on_map_msg(60, 48, 0.05, -0.707, -0.919, 2.0)
    assert s.snapshot(2.0)["warnings"] == ["/map 이 파일 지도와 다름 (60x48, 0.050, -0.707,-0.919)"]


def test_stop_latch_flows_through_state():
    s = DashboardState(INFO)
    s.on_command("fetch center", 0.0)
    s.on_bridge_state("moving", 1.0)
    s.on_bridge_state("picking", 5.0)
    assert s.engage_stop(6.0) == ["estop", "stop"]
    s.on_bridge_state("idle", 6.1)                         # stop 응답 — 단계는 그대로
    assert s.snapshot(7.0)["mission"]["stages"][1]["state"] == "active"
    assert s.set_pick(True, {"state": "SEARCH"}, 7.0) == ["estop"]
    s.on_bridge_state("returning", 8.0)
    assert s.stop_tick(8.0) == ["stop"]
    s.note_stop("stop", "stop 발행")
    snap = s.snapshot(8.5)
    assert snap["stop"]["latched"] is True and snap["stop"]["last_result"] == "stop 발행"
    assert snap["pick"] == {"reachable": True, "status": {"state": "SEARCH"}, "age": 1.5}
    s.release_stop()
    assert s.snapshot(9.0)["stop"]["latched"] is False
```

- [ ] **Step 2: 실패 확인**

Run: `/usr/bin/python3 -m pytest nav/dashboard/test_dashboard_state.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'dashboard_state'`

- [ ] **Step 3: 구현** — `nav/dashboard/dashboard_state.py`

```python
"""ROS 콜백 스레드·웹 스레드가 함께 쓰는 대시보드 상태. ROS 메시지 대신 꺼낸 값만 받는다."""
from __future__ import annotations

import math
import threading

from map_geometry import MapInfo, Pose2D, compose, decimate, scan_points
from mission_timeline import MissionTimeline
from stop_latch import StopLatch
from topic_stats import TopicStats

IN_TOPICS = ["/abo/state", "/abo/status", "/abo/pick_request", "/abo/pick_done", "/tf", "/odom",
             "/scan", "/particle_cloud", "/amcl_pose", "/plan", "/map"]
OUT_TOPICS = ["/abo/command"]


def _age(now: float, t: float | None) -> float | None:
    return None if t is None else round(now - t, 2)


class DashboardState:
    def __init__(self, info: MapInfo, lidar: Pose2D = (0.0, 0.0, 0.0), max_particles: int = 800,
                 trail_max: int = 600, trail_step_m: float = 0.05) -> None:
        self.info, self.lidar = info, lidar
        self.max_particles, self.trail_max, self.trail_step = max_particles, trail_max, trail_step_m
        self._lock = threading.Lock()
        self.timeline = MissionTimeline()
        self.latch = StopLatch()
        self.topics = TopicStats()
        for t in IN_TOPICS:
            self.topics.declare(t, "in")
        for t in OUT_TOPICS:
            self.topics.declare(t, "out")
        self._map_odom: tuple[Pose2D, float] | None = None
        self._odom_base: tuple[Pose2D, float] | None = None
        self._cov_xy: float | None = None
        self._scan: tuple[list, float] | None = None
        self._particles: tuple[list, int, float] | None = None
        self._plan: tuple[list, float] | None = None
        self._trail: list[list[float]] = []
        self._warnings: list[str] = []
        self._pick: tuple[bool, dict | None, float | None] = (False, None, None)

    # ── 내부 ──
    def _robot(self) -> Pose2D | None:
        if self._map_odom is None or self._odom_base is None:
            return None
        return compose(self._map_odom[0], self._odom_base[0])

    # ── 입력 ──
    def on_tf(self, parent: str, child: str, pose: Pose2D, now: float) -> None:
        with self._lock:
            if (parent, child) == ("map", "odom"):
                self._map_odom = (pose, now)
            elif (parent, child) == ("odom", "base_link"):
                self._odom_base = (pose, now)
            self.topics.record("/tf", now, f"{parent}→{child}")

    def on_odom(self, pose_in_odom: Pose2D, now: float) -> None:
        with self._lock:
            self.topics.record("/odom", now, f"x {pose_in_odom[0]:+.2f} y {pose_in_odom[1]:+.2f}")
            if self._map_odom is None:
                return
            x, y, _ = compose(self._map_odom[0], pose_in_odom)
            if self._trail and math.hypot(x - self._trail[-1][0], y - self._trail[-1][1]) < self.trail_step:
                return
            self._trail.append([round(x, 3), round(y, 3)])
            del self._trail[:-self.trail_max]

    def on_scan(self, ranges, angle_min, angle_increment, range_min, range_max, now: float) -> None:
        with self._lock:
            robot = self._robot()
            pts = [] if robot is None else scan_points(ranges, angle_min, angle_increment,
                                                       range_min, range_max, robot, self.lidar)
            valid = sum(1 for r in ranges if math.isfinite(r) and range_min <= r <= range_max)
            self.topics.record("/scan", now, f"{len(ranges)}빔 · 유효 {valid}")
            if robot is not None:
                self._scan = (pts, now)

    def on_particles(self, xy: list[list[float]], now: float) -> None:
        with self._lock:
            self._particles = (decimate(xy, self.max_particles), len(xy), now)
            self.topics.record("/particle_cloud", now, f"{len(xy)}개")

    def on_amcl_pose(self, x: float, y: float, cov_xy: float, now: float) -> None:
        with self._lock:
            self._cov_xy = cov_xy
            self.topics.record("/amcl_pose", now, f"x {x:+.2f} y {y:+.2f} σ² {cov_xy:.3f}")

    def on_plan(self, xy: list[list[float]], now: float) -> None:
        with self._lock:
            self._plan = (xy, now)
            self.topics.record("/plan", now, f"{len(xy)}점")

    def on_map_msg(self, width, height, resolution, origin_x, origin_y, now: float) -> None:
        with self._lock:
            i = self.info
            same = (width, height) == (i.width, i.height) and abs(resolution - i.resolution) < 1e-3 \
                and abs(origin_x - i.origin_x) < 1e-3 and abs(origin_y - i.origin_y) < 1e-3
            self._warnings = [] if same else [
                f"/map 이 파일 지도와 다름 ({width}x{height}, {resolution:.3f}, {origin_x:.3f},{origin_y:.3f})"]
            self.topics.record("/map", now, f"{width}x{height}")

    def on_bridge_state(self, state: str, now: float) -> None:
        with self._lock:
            self.timeline.on_state(state, now, stop_latched=self.latch.latched)
            self.latch.on_bridge_state(state, now)
            self.topics.record("/abo/state", now, state)

    def on_bridge_status(self, text: str, now: float) -> None:
        with self._lock:
            self.timeline.on_status(text)
            self.topics.record("/abo/status", now, text)

    def on_command(self, text: str, now: float) -> None:
        with self._lock:
            self.timeline.on_command(text, now)
            if self.timeline.is_new_mission_since(now - 1e-9):
                self._trail = []
            self.topics.record("/abo/command", now, f'"{text}"')

    def on_pick_request(self, text: str, now: float) -> None:
        with self._lock:
            self.topics.record("/abo/pick_request", now, f'"{text}"')

    def on_pick_done(self, ok: bool, now: float) -> None:
        with self._lock:
            self.topics.record("/abo/pick_done", now, "true" if ok else "false")

    def set_pick(self, reachable: bool, status: dict | None, now: float) -> list[str]:
        with self._lock:
            prev_t = self._pick[2]
            self._pick = (reachable, status if reachable else None, now if reachable else prev_t)
            return self.latch.on_pick_reachable(reachable)

    # ── 정지 ──
    def engage_stop(self, now: float) -> list[str]:
        with self._lock:
            return self.latch.engage(now)

    def release_stop(self) -> None:
        with self._lock:
            self.latch.release()

    def stop_tick(self, now: float) -> list[str]:
        with self._lock:
            return self.latch.tick(now)

    def note_stop(self, action: str, result: str) -> None:
        with self._lock:
            self.latch.note(action, result)

    # ── 출력 ──
    def snapshot(self, now: float) -> dict:
        with self._lock:
            robot = self._robot()
            pose = None
            if robot is not None:
                older = min(self._map_odom[1], self._odom_base[1])
                pose = {"x": round(robot[0], 3), "y": round(robot[1], 3), "yaw": round(robot[2], 3),
                        "age": _age(now, older), "cov_xy": self._cov_xy}
            reachable, status, pick_t = self._pick
            return {
                "t": now,
                "pose": pose,
                "scan": {"pts": self._scan[0], "age": _age(now, self._scan[1])} if self._scan
                        else {"pts": [], "age": None},
                "particles": {"pts": self._particles[0], "n": self._particles[1],
                              "age": _age(now, self._particles[2])} if self._particles
                             else {"pts": [], "n": 0, "age": None},
                "plan": {"pts": self._plan[0], "age": _age(now, self._plan[1])} if self._plan
                        else {"pts": [], "age": None},
                "trail": list(self._trail),
                "mission": self.timeline.snapshot(now),
                "stop": self.latch.snapshot(),
                "pick": {"reachable": reachable, "status": status, "age": _age(now, pick_t) if reachable else None},
                "topics": self.topics.snapshot(now),
                "warnings": list(self._warnings),
            }
```

- [ ] **Step 4: 통과 확인**

Run: `/usr/bin/python3 -m pytest nav/dashboard -q`
Expected: 지금까지 전부 PASS (Task 4–8)

- [ ] **Step 5: Commit**

```bash
git add nav/dashboard/dashboard_state.py nav/dashboard/test_dashboard_state.py
git commit -m "Add: 대시보드 스레드 안전 상태와 스냅샷 (dashboard_state)

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_011xipca4hVhjGeqXSUCLhrD"
```

---

### Task 9: 집기 워커 클라이언트와 웹 앱 (`pick_client.py`, `web_app.py`)

**Files:**
- Create: `nav/dashboard/pick_client.py`, `nav/dashboard/web_app.py`, `nav/dashboard/static/index.html` (이 태스크에선 최소 자리표시 — Task 10 에서 교체)
- Test: `nav/dashboard/test_pick_client.py`, `nav/dashboard/test_web_app.py`

**Interfaces:**
- Consumes: `DashboardState` (Task 8)
- Produces:
  - `PickClient(base_url: str = "http://127.0.0.1:8000", timeout_s: float = 0.3, fetch=None)` — `fetch(method: str, url: str, timeout: float) -> bytes`, 기본은 `urllib.request`
    - `.poll_once() -> tuple[bool, dict | None]` — `GET /status` 성공이면 `(True, dict)`, 실패면 `(False, None)`
    - `.estop() -> str` — `POST /estop` 결과 문장: `"estop 보냄"` 또는 `"estop 실패: <예외>"`
    - `.start(on_update, interval_s: float = 0.2) -> threading.Thread` — 데몬 스레드가 `on_update(reachable, status)` 반복 호출
  - `make_stop_handlers(state: DashboardState, pick: PickClient, publish_stop, clock=time.time) -> tuple[on_stop, on_release, on_tick, on_pick_update]`
    - `publish_stop()` 는 `/abo/command "stop"` 을 발행하는 함수(노드가 넘김)
    - `on_stop() -> dict` — `{"ok": True, "results": [...]}`; `state.engage_stop` 의 액션 순서대로 수행, `estop` 은 `snapshot()["pick"]["reachable"]` 일 때만
    - `on_release() -> dict` — `{"ok": True}`
    - `on_tick() -> None` — 노드 타이머(0.1 s)가 부름: `state.stop_tick` 결과대로 `publish_stop`
    - `on_pick_update(reachable: bool, status: dict | None) -> None` — `PickClient.start` 에 넘김: `state.set_pick` 이 `["estop"]` 을 주면 `pick.estop()`
  - `sse_stream(snapshot_fn, interval_s: float = 0.2, sleep=time.sleep, max_events: int | None = None) -> Iterator[str]` — `"data: <json>\n\n"`
  - `create_app(state: DashboardState, map_png: bytes, map_meta: dict, on_stop, on_release) -> FastAPI`
    - `GET /` → `static/index.html`, `/static/*` → 정적 파일, `GET /map.png`, `GET /api/map_info` → `map_meta`, `GET /api/events` → SSE, `POST /api/stop`, `POST /api/stop/release`
  - `map_meta` 형식: `{"resolution", "origin_x", "origin_y", "width", "height", "waypoints": {name: {"x","y","yaw"}}}`

- [ ] **Step 1: pick_client 실패 테스트 작성** — `nav/dashboard/test_pick_client.py`

```python
import json
import sys
import threading
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from pick_client import PickClient  # noqa: E402


def test_poll_ok_and_fail():
    calls = []

    def fetch(method, url, timeout):
        calls.append((method, url, timeout))
        return json.dumps({"state": "SEARCH"}).encode()

    assert PickClient("http://h:8000", 0.3, fetch).poll_once() == (True, {"state": "SEARCH"})
    assert calls == [("GET", "http://h:8000/status", 0.3)]

    def boom(method, url, timeout):
        raise OSError("refused")

    assert PickClient(fetch=boom).poll_once() == (False, None)


def test_estop_result_text():
    seen = []
    assert PickClient("http://h:8000", fetch=lambda m, u, t: seen.append((m, u)) or b"{}").estop() == "estop 보냄"
    assert seen == [("POST", "http://h:8000/estop")]

    def boom(m, u, t):
        raise TimeoutError("timed out")

    assert PickClient(fetch=boom).estop() == "estop 실패: timed out"


def test_start_calls_update_in_background():
    got = threading.Event()
    PickClient(fetch=lambda m, u, t: b'{"state": "X"}').start(
        lambda ok, st: got.set() if (ok, st) == (True, {"state": "X"}) else None, interval_s=0.01)
    assert got.wait(1.0)
```

- [ ] **Step 2: 실패 확인**

Run: `/usr/bin/python3 -m pytest nav/dashboard/test_pick_client.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'pick_client'`

- [ ] **Step 3: 구현** — `nav/dashboard/pick_client.py`

```python
"""집기 워커 웹(:8000, pick_worker_cycle --web-port)의 /status 폴링과 /estop 호출.

브라우저가 아니라 대시보드 서버가 부르므로 8000 쪽 CORS 가 필요 없다. 8000 은 집기
중에만 떠 있으니 연결 실패는 정상 상황이다(짧은 타임아웃, 예외 없이 False).
"""
from __future__ import annotations

import json
import threading
import time
import urllib.request


def _urllib_fetch(method: str, url: str, timeout: float) -> bytes:
    req = urllib.request.Request(url, method=method, data=b"" if method == "POST" else None)
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return resp.read()


class PickClient:
    def __init__(self, base_url: str = "http://127.0.0.1:8000", timeout_s: float = 0.3, fetch=None) -> None:
        self.base = base_url.rstrip("/")
        self.timeout = timeout_s
        self.fetch = fetch or _urllib_fetch

    def poll_once(self) -> tuple[bool, dict | None]:
        try:
            return True, json.loads(self.fetch("GET", f"{self.base}/status", self.timeout) or b"{}")
        except Exception:
            return False, None

    def estop(self) -> str:
        try:
            self.fetch("POST", f"{self.base}/estop", self.timeout)
            return "estop 보냄"
        except Exception as exc:
            return f"estop 실패: {exc}"

    def start(self, on_update, interval_s: float = 0.2) -> threading.Thread:
        def loop():
            while True:
                ok, status = self.poll_once()
                on_update(ok, status)
                time.sleep(interval_s)

        th = threading.Thread(target=loop, daemon=True, name="pick-client")
        th.start()
        return th
```

- [ ] **Step 4: 통과 확인**

Run: `/usr/bin/python3 -m pytest nav/dashboard/test_pick_client.py -q`
Expected: 3 passed

- [ ] **Step 5: web_app 실패 테스트 작성** — `nav/dashboard/test_web_app.py`

```python
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from fastapi.testclient import TestClient  # noqa: E402

from dashboard_state import DashboardState  # noqa: E402
from map_geometry import MapInfo  # noqa: E402
from web_app import create_app, make_stop_handlers, sse_stream  # noqa: E402

INFO = MapInfo(0.05, -0.707, -0.919, 53, 48, Path("/nonexistent.pgm"))
META = {"resolution": 0.05, "origin_x": -0.707, "origin_y": -0.919, "width": 53, "height": 48,
        "waypoints": {"home": {"x": 0.0, "y": 0.0, "yaw": 0.0}}}


class FakePick:
    def __init__(self):
        self.estops = 0

    def estop(self):
        self.estops += 1
        return "estop 보냄"


def _handlers(reachable):
    state = DashboardState(INFO)
    pick = FakePick()
    published = []
    clock = iter([10.0, 10.05, 10.6, 11.0, 12.0]).__next__
    h = make_stop_handlers(state, pick, lambda: published.append("stop"), clock=clock)
    state.set_pick(reachable, {"state": "SEARCH"} if reachable else None, 9.0)
    return state, pick, published, h


def test_stop_calls_estop_then_publishes_when_pick_reachable():
    state, pick, published, (on_stop, on_release, on_tick, on_pick_update) = _handlers(True)
    res = on_stop()
    assert res == {"ok": True, "results": ["estop 보냄", "stop 발행"]}
    assert pick.estops == 1 and published == ["stop"]
    assert state.snapshot(10.0)["stop"]["last_result"] == "stop 발행"


def test_stop_skips_estop_when_pick_not_running():
    state, pick, published, (on_stop, *_rest) = _handlers(False)
    assert on_stop()["results"] == ["stop 발행"]
    assert pick.estops == 0 and published == ["stop"]


def test_tick_republishes_during_return_and_release_stops():
    state, pick, published, (on_stop, on_release, on_tick, on_pick_update) = _handlers(False)
    on_stop()                                   # clock 10.0
    state.on_bridge_state("returning", 10.05)
    on_tick()                                   # clock 10.05 → 0.05 s 뒤라 아직 안 보냄
    on_tick()                                   # clock 10.6 → 보냄
    assert published == ["stop", "stop"]
    assert on_release() == {"ok": True}
    on_tick()                                   # clock 11.0 → 해제됨
    assert published == ["stop", "stop"]


def test_pick_update_estops_when_web_comes_up_while_latched():
    state, pick, published, (on_stop, on_release, on_tick, on_pick_update) = _handlers(False)
    on_stop()
    on_pick_update(True, {"state": "SEARCH"})
    assert pick.estops == 1


def test_sse_stream_formats_json_events():
    it = sse_stream(lambda: {"a": 1}, interval_s=0.0, sleep=lambda s: None, max_events=2)
    assert list(it) == ['data: {"a": 1}\n\n', 'data: {"a": 1}\n\n']


def test_routes():
    state = DashboardState(INFO)
    client = TestClient(create_app(state, b"\x89PNG-fake", META, lambda: {"ok": True, "results": []},
                                   lambda: {"ok": True}))
    assert client.get("/").status_code == 200
    r = client.get("/map.png")
    assert r.headers["content-type"] == "image/png" and r.content == b"\x89PNG-fake"
    assert client.get("/api/map_info").json() == META
    assert client.post("/api/stop").json() == {"ok": True, "results": []}
    assert client.post("/api/stop/release").json() == {"ok": True}
```

- [ ] **Step 6: 실패 확인**

Run: `/usr/bin/python3 -m pytest nav/dashboard/test_web_app.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'web_app'`

- [ ] **Step 7: 구현** — `nav/dashboard/web_app.py`, 그리고 자리표시 `nav/dashboard/static/index.html`

```python
"""대시보드 HTTP 계층: 페이지·지도 이미지·SSE 스냅샷·정지 API.

SSE 는 동기 제너레이터를 StreamingResponse 에 넘긴다 — Starlette 가 스레드풀에서 next() 를
부르므로 time.sleep 이 이벤트 루프를 막지 않는다 (motion/webui/app.py 와 같은 방식).
"""
from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Iterator

from fastapi import FastAPI
from fastapi.responses import HTMLResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles

STATIC = Path(__file__).resolve().parent / "static"


def make_stop_handlers(state, pick, publish_stop, clock=time.time):
    def run(actions: list[str], reachable: bool) -> list[str]:
        results = []
        for a in actions:
            if a == "estop":
                if not reachable:
                    continue
                text = pick.estop()
            else:
                publish_stop()
                text = "stop 발행"
            state.note_stop(a, text)
            results.append(text)
        return results

    def on_stop() -> dict:
        now = clock()
        reachable = state.snapshot(now)["pick"]["reachable"]
        return {"ok": True, "results": run(state.engage_stop(now), reachable)}

    def on_release() -> dict:
        state.release_stop()
        return {"ok": True}

    def on_tick() -> None:
        run(state.stop_tick(clock()), reachable=False)

    def on_pick_update(reachable: bool, status: dict | None) -> None:
        run(state.set_pick(reachable, status, clock()), reachable=reachable)

    return on_stop, on_release, on_tick, on_pick_update


def sse_stream(snapshot_fn, interval_s: float = 0.2, sleep=time.sleep, max_events: int | None = None) -> Iterator[str]:
    n = 0
    while max_events is None or n < max_events:
        yield f"data: {json.dumps(snapshot_fn(), ensure_ascii=False)}\n\n"
        n += 1
        sleep(interval_s)


def create_app(state, map_png: bytes, map_meta: dict, on_stop, on_release) -> FastAPI:
    app = FastAPI()
    app.mount("/static", StaticFiles(directory=STATIC), name="static")

    @app.get("/", response_class=HTMLResponse)
    def index() -> str:
        return (STATIC / "index.html").read_text(encoding="utf-8")

    @app.get("/map.png")
    def map_png_route() -> Response:
        return Response(map_png, media_type="image/png")

    @app.get("/api/map_info")
    def map_info() -> dict:
        return map_meta

    @app.get("/api/events")
    def events() -> StreamingResponse:
        return StreamingResponse(sse_stream(lambda: state.snapshot(time.time())), media_type="text/event-stream",
                                 headers={"Cache-Control": "no-cache"})

    @app.post("/api/stop")
    def stop() -> dict:
        return on_stop()

    @app.post("/api/stop/release")
    def release() -> dict:
        return on_release()

    return app
```

`nav/dashboard/static/index.html` (Task 10 에서 교체):

```html
<!DOCTYPE html>
<html lang="ko"><head><meta charset="utf-8"><title>LeKiwi 관제</title></head>
<body><p>LeKiwi 관제 대시보드</p></body></html>
```

- [ ] **Step 8: 통과 확인**

Run: `/usr/bin/python3 -m pytest nav/dashboard -q`
Expected: 전부 PASS

- [ ] **Step 9: Commit**

```bash
git add nav/dashboard/pick_client.py nav/dashboard/web_app.py nav/dashboard/static/index.html nav/dashboard/test_pick_client.py nav/dashboard/test_web_app.py
git commit -m "Add: 대시보드 집기 워커 클라이언트·웹 앱·정지 핸들러

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_011xipca4hVhjGeqXSUCLhrD"
```

---

### Task 10: 화면 (시안 A) — `index.html`, `style.css`, `app.js`

**Files:**
- Modify: `nav/dashboard/static/index.html` (Task 9 자리표시 교체)
- Create: `nav/dashboard/static/style.css`, `nav/dashboard/static/app.js`
- Test: `nav/dashboard/test_web_app.py` (테스트 1개 추가)

**Interfaces:**
- Consumes: `GET /api/map_info`, `GET /map.png`, `GET /api/events` (스냅샷 JSON, Task 8 형식), `POST /api/stop`, `POST /api/stop/release`, 카메라 `http://<location.hostname>:8000/stream/front|wrist`
- Produces: DOM id — `conn`, `mission-chip`, `target`, `elapsed`, `stop-btn`, `latch-banner`, `latch-info`, `release-btn`, `warn-banner`, `map`, `legend`, `pose-info`, `stages`, `status-text`, `cam-front`, `cam-wrist`, `topics`
- 디자인 기준: 승인된 시안 `.superpowers/brainstorm/console-options.html` 의 "A안 수정본" (색 토큰·지도:카메라 폭 1.25fr:1fr·카메라 세로 2단·토픽 2열). 외부 폰트/CDN 을 쓰지 않는다(노트북만으로 동작).

- [ ] **Step 1: 실패 테스트 추가** — `nav/dashboard/test_web_app.py` 끝에

```python
def test_index_has_dashboard_regions_and_assets():
    client = TestClient(create_app(DashboardState(INFO), b"", META, lambda: {}, lambda: {}))
    html = client.get("/").text
    for element_id in ("conn", "mission-chip", "stop-btn", "latch-banner", "release-btn", "warn-banner",
                       "map", "legend", "stages", "status-text", "cam-front", "cam-wrist", "topics"):
        assert f'id="{element_id}"' in html, element_id
    assert "확실한 비상정지는 로봇 전원 스위치" in html
    assert client.get("/static/style.css").status_code == 200
    js = client.get("/static/app.js")
    assert js.status_code == 200 and "EventSource" in js.text
```

- [ ] **Step 2: 실패 확인**

Run: `/usr/bin/python3 -m pytest nav/dashboard/test_web_app.py -q -k regions`
Expected: FAIL — `AssertionError: conn`

- [ ] **Step 3: `index.html` 작성** (전체 교체)

```html
<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>LeKiwi 관제</title>
<link rel="stylesheet" href="/static/style.css">
</head>
<body>
<div class="console">
  <header class="strip">
    <span class="brand">LeKiwi 01</span>
    <span class="chip" id="conn"><i class="dot"></i><span>연결 중…</span></span>
    <span class="chip state" id="mission-chip">대기</span>
    <span class="chip" id="target">목표 없음</span>
    <span class="chip mono" id="elapsed">–</span>
    <span class="spacer"></span>
    <span class="estop-note">확실한 비상정지는 로봇 전원 스위치</span>
    <button class="estop" id="stop-btn" type="button">정지</button>
  </header>

  <div class="banner danger" id="latch-banner" hidden>
    <span>정지 유지 중 — 로봇이 움직이려 하면 계속 멈춥니다</span>
    <span class="mono" id="latch-info"></span>
    <button class="btn" id="release-btn" type="button">해제</button>
  </div>
  <div class="banner warn" id="warn-banner" hidden></div>

  <main class="body">
    <section class="col">
      <div class="pane">
        <div class="pane-title"><span>지도</span><span class="mono" id="pose-info">위치 없음</span></div>
        <div class="map-box"><canvas id="map" aria-label="Nav2 지도"></canvas></div>
        <div class="legend" id="legend"></div>
      </div>
      <div class="pane">
        <div class="pane-title"><span>미션 단계</span></div>
        <ol class="steps" id="stages"></ol>
        <p class="status-text" id="status-text"></p>
      </div>
    </section>

    <section class="col">
      <div class="pane">
        <div class="pane-title"><span>YOLO 카메라</span><span class="mono">front · wrist</span></div>
        <figure class="cam" id="cam-front" data-view="front">
          <img alt="front 카메라">
          <figcaption class="cam-label"><span>front</span><span class="cam-meta"></span></figcaption>
          <div class="cam-bar"></div>
          <div class="cam-empty">집기 단계에서 켜져요</div>
        </figure>
        <figure class="cam" id="cam-wrist" data-view="wrist">
          <img alt="wrist 카메라">
          <figcaption class="cam-label"><span>wrist</span><span class="cam-meta"></span></figcaption>
          <div class="cam-bar"></div>
          <div class="cam-empty">집기 단계에서 켜져요</div>
        </figure>
      </div>
    </section>
  </main>

  <section class="pane">
    <div class="pane-title"><span>토픽 · 입력 / 출력</span><span class="mono">노트북이 받은 시각 기준</span></div>
    <div class="io" id="topics"></div>
  </section>
</div>
<script src="/static/app.js"></script>
</body>
</html>
```

- [ ] **Step 4: `style.css` 작성**

```css
:root {
  --bg: #EDF0F4; --panel: #FFFFFF; --panel-2: #F4F5F9; --ink: #1A2130; --muted: #5F6878; --line: #D9DDE6;
  --accent: #5B45C9; --accent-soft: #ECE8FB; --ok: #1F8A5B; --ok-soft: #E2F3EA; --warn: #A86E12; --warn-soft: #FBF1DE;
  --danger: #C8372D; --danger-soft: #FBE6E4; --scan: #D9622B; --plan: #1F8A5B; --trail: #5B45C9;
  --map-filter: none;
  --shadow: 0 1px 2px rgba(26, 33, 48, 0.06), 0 8px 24px rgba(26, 33, 48, 0.06);
  --sans: "IBM Plex Sans KR", "Apple SD Gothic Neo", "Noto Sans KR", system-ui, sans-serif;
  --mono: "IBM Plex Mono", ui-monospace, "SFMono-Regular", Menlo, monospace;
}
@media (prefers-color-scheme: dark) {
  :root:not([data-theme="light"]) {
    --bg: #0E1219; --panel: #151B25; --panel-2: #1B2230; --ink: #E3E8F0; --muted: #8C96A8; --line: #273041;
    --accent: #9D8BFF; --accent-soft: #252046; --ok: #45C08A; --ok-soft: #173326; --warn: #E0A94A; --warn-soft: #3A2E17;
    --danger: #F0625A; --danger-soft: #3A1B1A; --scan: #F08A52; --plan: #45C08A; --trail: #9D8BFF;
    --map-filter: invert(0.86) hue-rotate(180deg);
    --shadow: 0 1px 2px rgba(0, 0, 0, 0.3), 0 8px 24px rgba(0, 0, 0, 0.25);
  }
}
:root[data-theme="dark"] {
  --bg: #0E1219; --panel: #151B25; --panel-2: #1B2230; --ink: #E3E8F0; --muted: #8C96A8; --line: #273041;
  --accent: #9D8BFF; --accent-soft: #252046; --ok: #45C08A; --ok-soft: #173326; --warn: #E0A94A; --warn-soft: #3A2E17;
  --danger: #F0625A; --danger-soft: #3A1B1A; --scan: #F08A52; --plan: #45C08A; --trail: #9D8BFF;
  --map-filter: invert(0.86) hue-rotate(180deg);
  --shadow: 0 1px 2px rgba(0, 0, 0, 0.3), 0 8px 24px rgba(0, 0, 0, 0.25);
}

* { box-sizing: border-box; }
body { margin: 0; background: var(--bg); color: var(--ink); font-family: var(--sans); font-size: 14px; line-height: 1.5; }
.console { max-width: 1600px; margin: 0 auto; padding-inline: 16px; padding-block: 12px 24px; display: grid; gap: 10px; }
.mono { font-family: var(--mono); font-variant-numeric: tabular-nums; }
[hidden] { display: none !important; }

.strip { display: flex; flex-wrap: wrap; align-items: center; gap: 8px; }
.brand { font-weight: 700; font-size: 15px; margin-right: 4px; }
.chip { display: inline-flex; align-items: center; gap: 6px; border: 1px solid var(--line); background: var(--panel); border-radius: 999px; padding: 2px 10px; white-space: nowrap; font-size: 13px; }
.chip.state { background: var(--accent-soft); border-color: transparent; color: var(--accent); font-weight: 600; }
.chip.state.ok { background: var(--ok-soft); color: var(--ok); }
.chip.state.bad { background: var(--danger-soft); color: var(--danger); }
.dot { width: 8px; height: 8px; border-radius: 50%; background: var(--muted); display: inline-block; }
.dot.ok { background: var(--ok); }
.dot.bad { background: var(--danger); }
.pill { width: 10px; height: 10px; border-radius: 50%; display: inline-block; }
.pill.green { background: #2FA35B; } .pill.red { background: #D8413A; } .pill.blue { background: #3A6FD8; }
.spacer { flex: 1; }
.estop-note { font-size: 12px; color: var(--muted); }
.estop { font: inherit; font-weight: 700; font-size: 14px; color: #fff; background: var(--danger); border: 0; border-radius: 999px; padding: 8px 20px; box-shadow: 0 0 0 3px var(--danger-soft); cursor: pointer; }
.estop:focus-visible, .btn:focus-visible { outline: 2px solid var(--accent); outline-offset: 2px; }
.btn { font: inherit; font-size: 13px; font-weight: 600; border-radius: 7px; padding: 4px 12px; border: 1px solid var(--line); background: var(--panel); color: var(--ink); cursor: pointer; }

.banner { display: flex; flex-wrap: wrap; align-items: center; gap: 12px; border-radius: 10px; padding: 8px 12px; font-weight: 600; }
.banner.danger { background: var(--danger-soft); color: var(--danger); }
.banner.warn { background: var(--warn-soft); color: var(--warn); }

.body { display: grid; grid-template-columns: minmax(0, 1.25fr) minmax(0, 1fr); gap: 10px; align-items: start; }
@media (max-width: 900px) { .body { grid-template-columns: 1fr; } }
.col { display: grid; gap: 10px; align-content: start; min-width: 0; }
.pane { background: var(--panel); border: 1px solid var(--line); border-radius: 10px; padding: 10px; display: grid; gap: 8px; align-content: start; min-width: 0; box-shadow: var(--shadow); }
.pane-title { display: flex; justify-content: space-between; align-items: center; gap: 8px; font-size: 11px; letter-spacing: 0.06em; text-transform: uppercase; color: var(--muted); font-weight: 600; }
.pane-title .mono { letter-spacing: 0; text-transform: none; font-weight: 500; }

.map-box { position: relative; width: 100%; }
#map { display: block; width: 100%; height: auto; border-radius: 8px; background: var(--panel-2); }
.legend { display: flex; flex-wrap: wrap; gap: 4px 14px; color: var(--muted); font-size: 12px; }
.legend label { display: inline-flex; align-items: center; gap: 5px; cursor: pointer; }
.legend i { display: inline-block; width: 12px; height: 4px; border-radius: 2px; }
.legend .age { font-family: var(--mono); font-size: 11px; }
.legend .stale { color: var(--warn); }

.steps { list-style: none; margin: 0; padding: 0; display: grid; gap: 2px; }
.steps li { display: grid; grid-template-columns: 14px 1fr auto; align-items: center; gap: 8px; padding: 5px 6px; border-radius: 6px; }
.steps .ic { width: 10px; height: 10px; border-radius: 50%; border: 2px solid var(--line); }
.steps .done .ic { background: var(--ok); border-color: var(--ok); }
.steps .failed .ic { background: var(--danger); border-color: var(--danger); }
.steps .active { background: var(--accent-soft); }
.steps .active .ic { border-color: var(--accent); background: var(--panel); }
.steps .active .label { color: var(--accent); font-weight: 600; }
.steps .todo { color: var(--muted); }
.steps .t { font-family: var(--mono); color: var(--muted); font-size: 12px; }
.status-text { margin: 0; color: var(--muted); font-size: 13px; min-height: 1.5em; }

.cam { position: relative; margin: 0; aspect-ratio: 4 / 3; max-width: 100%; border-radius: 8px; overflow: hidden; background: var(--panel-2); border: 2px solid transparent; }
.cam img { width: 100%; height: 100%; object-fit: cover; display: block; }
.cam.frozen { border-color: var(--danger); }
.cam-label { position: absolute; top: 0; left: 0; right: 0; display: flex; justify-content: space-between; padding: 3px 8px; font-family: var(--mono); font-size: 11px; color: #f2f4f7; background: rgba(12, 16, 22, 0.62); }
.cam-bar { position: absolute; bottom: 0; left: 0; right: 0; padding: 3px 8px; font-family: var(--mono); font-size: 11px; color: #d2c8ff; background: rgba(12, 16, 22, 0.72); }
.cam-bar:empty { display: none; }
.cam-empty { position: absolute; inset: 0; display: grid; place-items: center; color: var(--muted); font-size: 14px; }
.cam.live .cam-empty { display: none; }

.io { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); column-gap: 16px; }
@media (max-width: 900px) { .io { grid-template-columns: 1fr; } }
.io-row { display: grid; grid-template-columns: 34px minmax(0, 12em) minmax(0, 1fr) auto; gap: 8px; align-items: baseline; padding: 4px 2px; border-bottom: 1px solid var(--line); }
.dir { font-family: var(--mono); font-size: 11px; font-weight: 600; color: var(--accent); }
.dir.out { color: var(--ok); }
.io-row code { font-family: var(--mono); font-size: 12px; overflow-wrap: anywhere; }
.io-row .last { color: var(--muted); font-size: 12px; overflow-wrap: anywhere; }
.io-row .age { font-family: var(--mono); font-size: 11px; color: var(--muted); white-space: nowrap; }
.io-row.stale .age { color: var(--warn); }
.io-row.never code { color: var(--muted); }

@media (prefers-reduced-motion: no-preference) {
  .steps .active .ic { animation: pulse 2.4s ease-in-out infinite; }
  @keyframes pulse { 50% { box-shadow: 0 0 0 4px var(--accent-soft); } }
}
```

- [ ] **Step 5: `app.js` 작성**

```javascript
"use strict";

const $ = (id) => document.getElementById(id);
const esc = (t) => String(t ?? "").replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
const css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();

const STATE_TEXT = { idle: "대기", moving: "주행 중", arrived: "도착", picking: "집는 중", returning: "복귀 중",
  done: "완료", failed: "실패", canceled: "멈춤", rejected: "거절" };
const COLOR_TEXT = { green: "초록", red: "빨강", blue: "파랑" };
const LAYERS = [
  { key: "scan", label: "/scan", color: "--scan" },
  { key: "particles", label: "파티클", color: "--accent" },
  { key: "plan", label: "Nav2 경로", color: "--plan" },
  { key: "trail", label: "odom 궤적", color: "--trail" },
];
const visible = { scan: true, particles: true, plan: true, trail: true };

let meta = null;
let mapImg = null;
let last = null;
let lastEventAt = 0;

function fmtAge(a) {
  if (a == null) return "받은 적 없음";
  return a < 1 ? `${Math.round(a * 1000)} ms 전` : `${a.toFixed(1)} s 전`;
}

function fmtSec(s) {
  if (s == null) return "–";
  return `${Math.floor(s / 60)}:${String(Math.floor(s % 60)).padStart(2, "0")}`;
}

async function init() {
  meta = await (await fetch("/api/map_info")).json();
  mapImg = new Image();
  mapImg.onload = () => draw();
  mapImg.src = "/map.png";
  buildLegend();
  new ResizeObserver(() => draw()).observe($("map").parentElement);

  const es = new EventSource("/api/events");
  es.onmessage = (ev) => {
    last = JSON.parse(ev.data);
    lastEventAt = performance.now();
    render(last);
  };
  setInterval(checkConnection, 1000);

  $("stop-btn").addEventListener("click", () => fetch("/api/stop", { method: "POST" }));
  $("release-btn").addEventListener("click", () => fetch("/api/stop/release", { method: "POST" }));
}

function checkConnection() {
  const ok = performance.now() - lastEventAt < 3000;
  const c = $("conn");
  c.querySelector(".dot").className = "dot " + (ok ? "ok" : "bad");
  c.querySelector("span").textContent = ok ? "대시보드 연결됨" : "대시보드 서버 연결 끊김";
}

function buildLegend() {
  $("legend").innerHTML = LAYERS.map((l) =>
    `<label><input type="checkbox" id="layer-${l.key}" checked><i style="background:var(${l.color})"></i>` +
    `${l.label} <span class="age" data-age="${l.key}"></span></label>`).join("") +
    `<span><i style="background:var(--ink)"></i>로봇</span>`;
  for (const l of LAYERS) {
    $(`layer-${l.key}`).addEventListener("change", (e) => { visible[l.key] = e.target.checked; draw(); });
  }
}

function render(s) {
  const m = s.mission;
  const chip = $("mission-chip");
  chip.textContent = m.state ? (STATE_TEXT[m.state] || m.state) : "대기";
  chip.className = "chip state" + (m.state === "done" ? " ok"
    : ["failed", "canceled", "rejected"].includes(m.state) ? " bad" : "");
  $("target").innerHTML = m.target
    ? `${m.color ? `<i class="pill ${esc(m.color)}"></i>` : ""}${esc(m.target)} ${esc(COLOR_TEXT[m.color] || "")}`
    : "목표 없음";
  $("elapsed").textContent = fmtSec(m.elapsed);
  $("status-text").textContent = m.status || "";
  $("stages").innerHTML = m.stages.length
    ? m.stages.map((st) => `<li class="${st.state}"><span class="ic"></span><span class="label">${esc(st.label)}</span>` +
        `<span class="t">${st.sec == null ? "" : fmtSec(st.sec)}</span></li>`).join("")
    : `<li class="todo"><span class="ic"></span><span class="label">미션 없음 — fetch 명령을 기다리는 중</span><span></span></li>`;

  $("latch-banner").hidden = !s.stop.latched;
  $("latch-info").textContent = s.stop.latched ? `stop ${s.stop.sent}회 · ${s.stop.last_result || ""}` : "";
  $("warn-banner").hidden = s.warnings.length === 0;
  $("warn-banner").textContent = s.warnings.join(" · ");

  const p = s.pose;
  $("pose-info").textContent = p
    ? `x ${p.x.toFixed(2)} · y ${p.y.toFixed(2)} · ${(p.yaw * 180 / Math.PI).toFixed(0)}°` +
      (p.age > 1 ? ` · TF 끊김 ${p.age.toFixed(1)}s` : "")
    : "위치 없음 (TF 없음)";

  for (const l of LAYERS) {
    const el = document.querySelector(`[data-age="${l.key}"]`);
    if (l.key === "trail") { el.textContent = `${s.trail.length}점`; continue; }
    const age = s[l.key].age;
    if (l.key === "particles" && age != null && age > 2) {
      el.textContent = `마지막 갱신 ${age.toFixed(0)}s 전 (정지 중엔 정상)`;
      el.classList.remove("stale");
    } else {
      el.textContent = fmtAge(age);
      el.classList.toggle("stale", age != null && age > 2);
    }
  }

  renderCams(s.pick);
  renderTopics(s.topics);
  draw();
}

function renderCams(pick) {
  for (const view of ["front", "wrist"]) {
    const fig = $(`cam-${view}`);
    const img = fig.querySelector("img");
    if (!pick.reachable) {
      if (img.dataset.src) { img.removeAttribute("src"); delete img.dataset.src; }   // MJPEG 연결을 끊는다
      fig.classList.remove("live", "frozen");
      fig.querySelector(".cam-meta").textContent = "";
      fig.querySelector(".cam-bar").textContent = "";
      continue;
    }
    const url = `http://${location.hostname}:8000/stream/${view}`;
    if (img.dataset.src !== url) { img.src = url; img.dataset.src = url; }
    fig.classList.add("live");
    const st = pick.status || {};
    const age = (st.frame_age_s || {})[view];
    const frozen = age != null && age > 2;
    fig.classList.toggle("frozen", frozen);
    fig.querySelector(".cam-meta").textContent =
      `${st.state || ""} · ${st.hz ?? "–"} Hz` + (frozen ? ` · 카메라 멈춤 ${age.toFixed(1)}s` : "");
    const bits = [];
    const pu = (st.purple || {})[view];
    if (pu) bits.push(`보라 ${pu.ratio.toFixed(2)} / ${pu.thr.toFixed(2)}${pu.ratio >= pu.thr ? " OK" : ""}`);
    if (view === "front" && st.pick_attempts != null) bits.push(`시도 ${st.pick_attempts}/${st.max_pick_attempts}`);
    if (view === "wrist" && st.retry_depth) bits.push(`재시도 깊이 ${st.retry_depth.toFixed(2)}`);
    fig.querySelector(".cam-bar").textContent = bits.join(" · ");
  }
}

function renderTopics(topics) {
  $("topics").innerHTML = Object.entries(topics).map(([name, t]) => {
    const cls = t.age == null ? "never" : (t.rate > 0 && t.age > 2 ? "stale" : "");
    return `<div class="io-row ${cls}"><span class="dir ${t.dir}">${t.dir === "out" ? "OUT" : "IN"}</span>` +
      `<code>${esc(name)}</code><span class="last">${esc(t.last)}</span>` +
      `<span class="age">${t.rate ? `${t.rate.toFixed(1)} Hz · ` : ""}${fmtAge(t.age)}</span></div>`;
  }).join("");
}

function draw() {
  const canvas = $("map");
  if (!meta || !mapImg || !mapImg.complete || !mapImg.naturalWidth) return;
  const dpr = window.devicePixelRatio || 1;
  const cssW = canvas.parentElement.clientWidth;
  const cssH = cssW * meta.height / meta.width;
  if (canvas.width !== Math.round(cssW * dpr)) {
    canvas.width = Math.round(cssW * dpr);
    canvas.height = Math.round(cssH * dpr);
    canvas.style.height = `${cssH}px`;
  }
  const ctx = canvas.getContext("2d");
  const k = canvas.width / meta.width;                                     // 지도 한 칸 = k 캔버스 픽셀
  const P = (x, y) => [(x - meta.origin_x) / meta.resolution * k,
                       (meta.height - (y - meta.origin_y) / meta.resolution) * k];

  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.imageSmoothingEnabled = false;
  ctx.filter = css("--map-filter") || "none";
  ctx.drawImage(mapImg, 0, 0, canvas.width, canvas.height);
  ctx.filter = "none";

  ctx.font = `${Math.round(12 * dpr)}px ${css("--sans")}`;
  for (const [name, w] of Object.entries(meta.waypoints)) {
    const [px, py] = P(w.x, w.y);
    ctx.strokeStyle = css("--muted");
    ctx.fillStyle = css("--muted");
    ctx.lineWidth = dpr;
    ctx.beginPath(); ctx.arc(px, py, 6 * dpr, 0, Math.PI * 2); ctx.stroke();
    ctx.fillText(name, px + 8 * dpr, py - 8 * dpr);
  }

  const s = last;
  if (!s) return;
  const line = (pts, color, width, alpha) => {
    if (pts.length < 2) return;
    ctx.globalAlpha = alpha; ctx.strokeStyle = color; ctx.lineWidth = width * dpr;
    ctx.beginPath();
    pts.forEach(([x, y], i) => { const [px, py] = P(x, y); if (i) ctx.lineTo(px, py); else ctx.moveTo(px, py); });
    ctx.stroke(); ctx.globalAlpha = 1;
  };
  const dots = (pts, color, size, alpha) => {
    ctx.globalAlpha = alpha; ctx.fillStyle = color;
    for (const [x, y] of pts) { const [px, py] = P(x, y); ctx.fillRect(px - size / 2, py - size / 2, size, size); }
    ctx.globalAlpha = 1;
  };

  if (visible.trail) line(s.trail, css("--trail"), 2, 0.5);
  const driving = s.mission.stages.some((st) => st.state === "active" && (st.key === "drive" || st.key === "return"));
  if (visible.plan && driving) line(s.plan.pts, css("--plan"), 2.5, 0.9);
  if (visible.particles) dots(s.particles.pts, css("--accent"), 2 * dpr, s.particles.age > 2 ? 0.25 : 0.6);
  if (visible.scan && s.scan.age != null && s.scan.age <= 2) dots(s.scan.pts, css("--scan"), 3 * dpr, 0.9);

  if (s.pose) {
    const [px, py] = P(s.pose.x, s.pose.y);
    const R = Math.max(7 * dpr, 0.12 / meta.resolution * k);              // 로봇 반경 약 12 cm
    ctx.save();
    ctx.translate(px, py);
    ctx.rotate(-s.pose.yaw);                                               // 캔버스는 y 가 아래 → 부호 반대
    ctx.fillStyle = s.pose.age > 1 ? css("--muted") : css("--ink");
    ctx.beginPath(); ctx.moveTo(R, 0); ctx.lineTo(-R * 0.7, R * 0.6); ctx.lineTo(-R * 0.7, -R * 0.6); ctx.closePath();
    ctx.fill();
    ctx.restore();
  }
}

init().catch((err) => {
  $("warn-banner").hidden = false;
  $("warn-banner").textContent = `지도 정보를 불러오지 못했어요: ${err}`;
});
```

- [ ] **Step 6: 통과 확인**

Run: `/usr/bin/python3 -m pytest nav/dashboard -q`
Expected: 전부 PASS

- [ ] **Step 7: Commit**

```bash
git add nav/dashboard/static/index.html nav/dashboard/static/style.css nav/dashboard/static/app.js nav/dashboard/test_web_app.py
git commit -m "Add: 대시보드 화면 (시안 A — 지도·미션 단계·카메라·토픽·정지)

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_011xipca4hVhjGeqXSUCLhrD"
```

---

### Task 11: ROS 연결과 실행 진입점 (`ros_listener.py`, `dashboard_node.py`, `run_dashboard.sh`)

**Files:**
- Create: `nav/dashboard/ros_listener.py`, `nav/dashboard/dashboard_node.py`, `nav/dashboard/run_dashboard.sh`
- Test: `nav/dashboard/test_dashboard_node.py` (인자·경로만 — rclpy 는 `main()` 안에서만 import)

**Interfaces:**
- Consumes: `DashboardState` (Task 8), `PickClient`·`make_stop_handlers`·`create_app` (Task 9), `load_map_info`·`map_png_bytes`·`load_waypoints`·`yaw_from_quat` (Task 4)
- Produces:
  - `DashboardListener(state: DashboardState, particles: bool = True)` — rclpy `Node("lekiwi_dashboard")`, `.publish_stop() -> None`
  - `dashboard_node.parse_args(argv) -> Namespace` — `--map`(기본 `<repo>/nav/maps/lekiwi01/map_0912_1654.yaml`), `--waypoints`(기본 `<repo>/nav/mission/waypoints.yaml`), `--host`(기본 `127.0.0.1`), `--port`(기본 `8001`), `--pick-url`(기본 `http://127.0.0.1:8000`), `--no-particles`
  - `run_dashboard.sh [--robot-ip IP] [dashboard_node 인자...]`

- [ ] **Step 1: 실패 테스트 작성** — `nav/dashboard/test_dashboard_node.py`

```python
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import dashboard_node  # noqa: E402  (rclpy 를 import 하지 않아야 한다)


def test_defaults_point_to_repo_files():
    a = dashboard_node.parse_args([])
    assert Path(a.map).is_file() and a.map.endswith("nav/maps/lekiwi01/map_0912_1654.yaml")
    assert Path(a.waypoints).is_file()
    assert (a.host, a.port, a.pick_url, a.no_particles) == ("127.0.0.1", 8001, "http://127.0.0.1:8000", False)


def test_flags():
    a = dashboard_node.parse_args(["--host", "0.0.0.0", "--port", "9001", "--no-particles"])
    assert (a.host, a.port, a.no_particles) == ("0.0.0.0", 9001, True)


def test_module_does_not_import_rclpy_at_top():
    assert "rclpy" not in dashboard_node.__dict__
```

- [ ] **Step 2: 실패 확인**

Run: `/usr/bin/python3 -m pytest nav/dashboard/test_dashboard_node.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'dashboard_node'`

- [ ] **Step 3: `ros_listener.py` 구현**

```python
"""rclpy 구독 → DashboardState. 메시지에서 값만 꺼내 넘긴다(상태는 ROS 를 모른다).

QoS 는 발행 측에 맞춘다 (2026-09-14 `ros2 topic info -v` 로 확인):
  /scan, /particle_cloud  best_effort   /map, /amcl_pose, /tf_static  reliable + transient_local
"""
from __future__ import annotations

import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.msg import ParticleCloud
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage

from map_geometry import yaw_from_quat

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
RELIABLE = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
TF_QOS = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)


def _pose_from_transform(tr):
    t, q = tr.transform.translation, tr.transform.rotation
    return (t.x, t.y, yaw_from_quat(q.x, q.y, q.z, q.w))


def _pose_from_msg_pose(p):
    q = p.orientation
    return (p.position.x, p.position.y, yaw_from_quat(q.x, q.y, q.z, q.w))


class DashboardListener(Node):
    def __init__(self, state, particles: bool = True) -> None:
        super().__init__("lekiwi_dashboard")
        self.state = state
        sub = self.create_subscription
        sub(TFMessage, "/tf", self._safe(self._on_tf), TF_QOS)
        sub(TFMessage, "/tf_static", self._safe(self._on_tf_static), LATCHED)
        sub(Odometry, "/odom", self._safe(lambda m: state.on_odom(_pose_from_msg_pose(m.pose.pose), time.time())), RELIABLE)
        sub(LaserScan, "/scan", self._safe(self._on_scan), qos_profile_sensor_data)
        if particles:
            sub(ParticleCloud, "/particle_cloud", self._safe(lambda m: state.on_particles(
                [[round(p.pose.position.x, 3), round(p.pose.position.y, 3)] for p in m.particles], time.time())),
                qos_profile_sensor_data)
        sub(PoseWithCovarianceStamped, "/amcl_pose", self._safe(self._on_amcl), LATCHED)
        sub(Path, "/plan", self._safe(lambda m: state.on_plan(
            [[round(ps.pose.position.x, 3), round(ps.pose.position.y, 3)] for ps in m.poses], time.time())), RELIABLE)
        sub(OccupancyGrid, "/map", self._safe(lambda m: state.on_map_msg(
            m.info.width, m.info.height, m.info.resolution,
            m.info.origin.position.x, m.info.origin.position.y, time.time())), LATCHED)
        sub(String, "/abo/state", self._safe(lambda m: state.on_bridge_state(m.data, time.time())), RELIABLE)
        sub(String, "/abo/status", self._safe(lambda m: state.on_bridge_status(m.data, time.time())), RELIABLE)
        sub(String, "/abo/command", self._safe(lambda m: state.on_command(m.data, time.time())), RELIABLE)
        sub(String, "/abo/pick_request", self._safe(lambda m: state.on_pick_request(m.data, time.time())), RELIABLE)
        sub(Bool, "/abo/pick_done", self._safe(lambda m: state.on_pick_done(bool(m.data), time.time())), RELIABLE)
        self._cmd_pub = self.create_publisher(String, "/abo/command", 10)

    def _safe(self, fn):
        def wrapped(msg):
            try:
                fn(msg)
            except Exception as exc:        # 콜백 예외로 노드가 죽지 않게
                self.get_logger().error(f"콜백 오류: {type(exc).__name__}: {exc}", throttle_duration_sec=5.0)
        return wrapped

    def _on_tf(self, msg) -> None:
        now = time.time()
        for tr in msg.transforms:
            self.state.on_tf(tr.header.frame_id.lstrip("/"), tr.child_frame_id.lstrip("/"), _pose_from_transform(tr), now)

    def _on_tf_static(self, msg) -> None:
        for tr in msg.transforms:
            if (tr.header.frame_id.lstrip("/"), tr.child_frame_id.lstrip("/")) == ("base_link", "lidar_link"):
                self.state.lidar = _pose_from_transform(tr)

    def _on_scan(self, m) -> None:
        self.state.on_scan(list(m.ranges), m.angle_min, m.angle_increment, m.range_min, m.range_max, time.time())

    def _on_amcl(self, m) -> None:
        c = m.pose.covariance
        self.state.on_amcl_pose(m.pose.pose.position.x, m.pose.pose.position.y, (c[0] + c[7]) / 2.0, time.time())

    def publish_stop(self) -> None:
        self._cmd_pub.publish(String(data="stop"))
```

- [ ] **Step 4: `dashboard_node.py` 구현**

```python
#!/usr/bin/env python3
"""LeKiwi 관제 대시보드 진입점 (노트북 전용, 로봇에 배포하지 않는다).

    nav/dashboard/run_dashboard.sh --robot-ip 223.194.139.15      # 환경변수까지 맞춰 실행
    브라우저: http://localhost:8001

집기 카메라는 pick_worker_cycle.py 를 -- --web-port 8000 으로 띄웠을 때만 나온다.
"""
from __future__ import annotations

import argparse
import sys
import threading
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--map", default=str(REPO / "nav/maps/lekiwi01/map_0912_1654.yaml"))
    ap.add_argument("--waypoints", default=str(REPO / "nav/mission/waypoints.yaml"))
    ap.add_argument("--host", default="127.0.0.1", help="다른 기기에서 볼 때만 0.0.0.0")
    ap.add_argument("--port", type=int, default=8001)
    ap.add_argument("--pick-url", default="http://127.0.0.1:8000")
    ap.add_argument("--no-particles", action="store_true", help="파티클 구독을 끈다 (로봇 와이파이 송신량 절약)")
    return ap.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    import rclpy
    import uvicorn

    from dashboard_state import DashboardState
    from map_geometry import load_map_info, load_waypoints, map_png_bytes
    from pick_client import PickClient
    from ros_listener import DashboardListener
    from web_app import create_app, make_stop_handlers

    info = load_map_info(args.map)
    meta = {"resolution": info.resolution, "origin_x": info.origin_x, "origin_y": info.origin_y,
            "width": info.width, "height": info.height, "waypoints": load_waypoints(args.waypoints)}
    state = DashboardState(info)
    pick = PickClient(args.pick_url)

    rclpy.init()
    node = DashboardListener(state, particles=not args.no_particles)
    on_stop, on_release, on_tick, on_pick_update = make_stop_handlers(state, pick, node.publish_stop)
    node.create_timer(0.1, on_tick)
    pick.start(on_pick_update)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True, name="ros-spin").start()

    app = create_app(state, map_png_bytes(info), meta, on_stop, on_release)
    print(f"[dashboard] http://{args.host}:{args.port}  지도 {Path(args.map).name}  집기 {args.pick_url}", flush=True)
    try:
        uvicorn.run(app, host=args.host, port=args.port, log_level="warning", timeout_graceful_shutdown=2)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 5: `run_dashboard.sh` 작성 후 실행 권한**

```bash
#!/usr/bin/env bash
# 노트북에서 대시보드 실행. 셸 기본 ROS_DOMAIN_ID(77, 에이보)와 무관하게 로봇 도메인 42 로 고정한다.
#   nav/dashboard/run_dashboard.sh [--robot-ip 223.194.139.15] [--port 8001] [--no-particles] ...
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
ROBOT_IP="223.194.139.15"
ARGS=()
while [ $# -gt 0 ]; do
    case "$1" in
        --robot-ip) ROBOT_IP="$2"; shift 2 ;;
        *) ARGS+=("$1"); shift ;;
    esac
done
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET ROS_STATIC_PEERS="$ROBOT_IP"
exec /usr/bin/python3 -u "$HERE/dashboard_node.py" "${ARGS[@]}"
```

Run: `chmod +x nav/dashboard/run_dashboard.sh`

- [ ] **Step 6: 테스트 통과 확인**

Run: `/usr/bin/python3 -m pytest nav/dashboard -q`
Expected: 전부 PASS

- [ ] **Step 7: 로봇 없이 기동 확인 (로봇 스택 꺼진 상태에서도 떠야 한다)**

```bash
nav/dashboard/run_dashboard.sh --port 8001 > /tmp/claude-dashboard-smoke.log 2>&1 &
DPID=$!
until curl -s -o /dev/null http://127.0.0.1:8001/api/map_info; do sleep 1; done
curl -s http://127.0.0.1:8001/api/map_info
curl -s -N http://127.0.0.1:8001/api/events | head -c 400; echo
curl -s -o /dev/null -w "%{http_code} %{content_type}\n" http://127.0.0.1:8001/map.png
kill $DPID
```
Expected: `map_info` 에 `"width":53,"height":48` 와 `home`/`center`, `events` 가 `data: {"t": ...` 로 시작하고 `"pose": null`, `map.png` 는 `200 image/png`. 로그에 Traceback 없음.

- [ ] **Step 8: 브라우저 화면 확인 (사람이 본다)**

같은 명령으로 띄운 뒤 `http://localhost:8001` 을 연다. 확인:
- 지도(53×48)가 선명하게 왼쪽 큰 칸에 보이고 `home`·`center` 표시가 있다
- 연결 칩이 "대시보드 연결됨", 위치는 "위치 없음 (TF 없음)", 카메라 두 칸은 "집기 단계에서 켜져요"
- 토픽 패널에 12개 토픽이 "받은 적 없음"으로 나온다
- 브라우저 폭을 400px 로 줄이면 한 열로 쌓이고 가로 스크롤이 없다
- 대시보드를 끄면 3초 안에 "대시보드 서버 연결 끊김"

- [ ] **Step 9: Commit**

```bash
git add nav/dashboard/ros_listener.py nav/dashboard/dashboard_node.py nav/dashboard/run_dashboard.sh nav/dashboard/test_dashboard_node.py
git commit -m "Add: 대시보드 ROS 구독 노드와 실행 스크립트

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_011xipca4hVhjGeqXSUCLhrD"
```

---

### Task 12: 실기기 통합 확인 (수동, 코드 변경 없음)

**전제:** 로봇을 쓰는 팀원과 시간을 맞춘다. 로봇은 home 에 두고 center 에 약통을 둔다. 집기 중 [정지] 시험은 로봇을 손으로 잡을 수 있을 때만 한다.

- [ ] **Step 1: 로봇 Nav2 스택 기동 (노트북을 연결 대상에 추가, 로봇 파일은 수정하지 않음)**

```bash
ssh roboseasy@223.194.139.15 'source /opt/ros/jazzy/setup.bash; source ~/lekiwi_profile.sh;
  export ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET ROS_STATIC_PEERS="223.194.157.118";
  setsid nohup ros2 launch ~/launch/lekiwi_nav.launch.py map:=/home/roboseasy/maps/map_0912_1654.yaml pick_timeout:=360.0 \
    > ~/nav_dashboard_test.log 2>&1 < /dev/null &'
```
Expected: 약 60 초 뒤 `grep -a "abo_nav_bridge\]: 대기 중" ~/nav_dashboard_test.log` 가 나온다.

- [ ] **Step 2: 다른 기기의 집기 어댑터가 없는지 확인**

로봇에서 `ros2 topic info -v /abo/pick_request` 의 SUBSCRIPTION 이 노트북 어댑터 하나뿐인지 본다(엔드포인트 GID 3·4번째 바이트가 호스트, 노트북 `6b.1e`). 다른 호스트가 있으면 그 기기의 어댑터를 먼저 끈다.

- [ ] **Step 3: 노트북에서 대시보드와 어댑터 기동**

```bash
nav/dashboard/run_dashboard.sh &
# 어댑터: 기존 명령 끝의 -- 뒤에 --web-port 8000 추가
```
Expected: 브라우저에서 지도 위 로봇 삼각형·스캔 점이 보이고 `/scan` 약 6 Hz, `/tf`·`/odom` 20 Hz 이상. 파티클은 "마지막 갱신 n초 전 (정지 중엔 정상)".

- [ ] **Step 4: 미션 1회 — `fetch center color:blue`**

Expected 순서: 미션 칩 "주행 중" → 경로(초록 선)·궤적·파티클 갱신 → "집는 중" 에서 카메라 두 칸이 켜지고 보라 비율·시도 횟수 표시 → "복귀 중" → "완료", 단계 3개가 모두 done 이고 시간이 로그와 맞는다.

- [ ] **Step 5: 주행 중 [정지] 시험**

다시 fetch 후 주행 중 [정지]. Expected: 1 초 안에 로봇 정지, "정지 유지 중" 띠, 미션 칩 "멈춤". [해제] 후 띠가 사라진다.

- [ ] **Step 6: 정리**

대시보드·어댑터 종료, 로봇 스택 종료(launch 부모 SIGKILL 먼저 → 자식 PID 종료 → `fuser /dev/ttyUSB0 /dev/ttyACM0` 비었는지 확인). 결과(단계 시간, 문제점)를 PR 설명에 적는다.
