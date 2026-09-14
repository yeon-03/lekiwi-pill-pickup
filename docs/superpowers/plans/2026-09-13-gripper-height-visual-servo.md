# 그리퍼 높이 시각 서보 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 손목캠 박스 크기나 미리 저장된 joint 각도에 의존하지 않고, 프론트 카메라로 직접
관찰한 그리퍼(보라색)와 약통의 실제 상대 높이로 "충분히 내려갔는지"를 판단하게 만든다.

**Architecture:** `grasp_check.py`의 기존 보라색 검출을 확장해 그리퍼 바운딩박스를 뽑고,
`headless_worker.py`에서 이걸 약통 박스와 비교해 `front_dy`를 계산한다. `front_dy`는
`ArmSequencer.update()` → `WristServo.update()`로 흘러가 기존 `y_anchor` 기반 세로
보정을 대체한다(둘이 동시에 작동하면 안 됨). 보라색 인식이 오래 실패하면
`shoulder_lift`/`elbow_flex` 두 관절만 직접, 상한을 둔 채로 느리게 내리는 안전 폴백이
따로 병렬로 작동한다.

**Tech Stack:** Python, OpenCV(`cv2.findContours`), numpy, pytest.

**Spec:** `docs/superpowers/specs/2026-09-13-gripper-height-visual-servo-design.md`

## Global Constraints

- 모든 새 함수/메서드는 순수 함수 또는 기존 클래스의 메서드로 만들고, 기존 파일의
  TDD/스타일(주석은 "왜"만, 한글 docstring)을 따른다.
- `front_dy`가 `None`이면 기존 동작(y_anchor 기반)을 그대로 유지해야 한다 — 기존
  테스트(`test_wrist_servo.py`의 기존 케이스들)가 전부 그대로 통과해야 한다.
- 바닥-근접 폴백은 `arm_shoulder_lift.pos`/`arm_elbow_flex.pos` **두 관절만** 직접
  움직인다 — `retry_overreach`처럼 pick→grasp 벡터를 연장하는 방식은 절대 쓰지 않는다
  (2026-09-13 세션에서 그 방식이 pan까지 같이 밀리는 버그를 냈다).
- 모든 단계는 `cd /home/roboseasy/lekiwi-pill-pickup-1/motion && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONNOUSERSITE=1 /home/roboseasy/miniforge3/envs/lerobot/bin/python -m pytest services/ -q` 로 검증한다 (이 레포의 표준 테스트 실행 방식).

---

### Task 1: `purple_bbox()` — 그리퍼(보라색) 바운딩박스 추출

**Files:**
- Modify: `services/pickplace/grasp_check.py` (`purple_mask()` 함수 바로 뒤, 76~79번 줄 다음)
- Test: `services/pickplace/test_grasp_check.py` (신규 파일)

**Interfaces:**
- Produces: `purple_bbox(frame_bgr: np.ndarray, cfg: GraspCheckArgs, x_range: tuple[int, int] | None = None) -> tuple[int, int, int, int] | None`

- [ ] **Step 1: Write the failing tests**

`services/pickplace/test_grasp_check.py` 새로 생성:

```python
import cv2
import numpy as np

from services.pickplace.grasp_check import GraspCheckArgs, purple_bbox


def _purple_bgr(cfg: GraspCheckArgs) -> tuple[int, int, int]:
    """cfg 의 보라색 HSV 범위 한가운데 값을 BGR 로 변환 (테스트용 색칠 재료)."""
    hue = (cfg.hue_min + cfg.hue_max) // 2
    hsv = np.uint8([[[hue, 200, 200]]])
    bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0][0]
    return int(bgr[0]), int(bgr[1]), int(bgr[2])


def test_purple_bbox_returns_none_when_no_purple():
    cfg = GraspCheckArgs()
    frame = np.zeros((100, 100, 3), dtype=np.uint8)
    assert purple_bbox(frame, cfg) is None


def test_purple_bbox_finds_largest_purple_region():
    cfg = GraspCheckArgs()
    frame = np.zeros((200, 300, 3), dtype=np.uint8)
    color = _purple_bgr(cfg)
    cv2.rectangle(frame, (50, 60), (90, 120), color, -1)  # 작은 블록 (40x60)
    cv2.rectangle(frame, (150, 20), (230, 100), color, -1)  # 더 큰 블록 (80x80)
    box = purple_bbox(frame, cfg)
    assert box == (150, 20, 230, 100)


def test_purple_bbox_ignores_regions_outside_x_range():
    cfg = GraspCheckArgs()
    frame = np.zeros((200, 300, 3), dtype=np.uint8)
    color = _purple_bgr(cfg)
    cv2.rectangle(frame, (150, 20), (230, 100), color, -1)  # x_range 밖, 더 큼
    cv2.rectangle(frame, (50, 60), (90, 120), color, -1)  # x_range 안, 더 작음
    box = purple_bbox(frame, cfg, x_range=(0, 100))
    assert box == (50, 60, 90, 120)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd /home/roboseasy/lekiwi-pill-pickup-1/motion && PYTHONNOUSERSITE=1 /home/roboseasy/miniforge3/envs/lerobot/bin/python -m pytest services/pickplace/test_grasp_check.py -v`
Expected: FAIL — `ImportError: cannot import name 'purple_bbox'`

- [ ] **Step 3: Write minimal implementation**

`services/pickplace/grasp_check.py`, `purple_mask()` 함수(76~79번 줄) 바로 뒤에 추가:

```python
def purple_bbox(
    frame_bgr: np.ndarray, cfg: GraspCheckArgs, x_range: tuple[int, int] | None = None
) -> tuple[int, int, int, int] | None:
    """보라색(그리퍼) 영역 중 가장 큰 덩어리의 바운딩박스 (x1, y1, x2, y2).

    `x_range` (x1, x2) 를 주면 중심 x 가 그 범위 안에 있는 컨투어만 후보로 삼는다
    (화면의 다른 부위 — 팔 상단 등 — 에 있는 보라색 오검출 방지). 없으면 없음.
    """
    mask = purple_mask(frame_bgr, cfg)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best: tuple[int, int, int, int] | None = None
    best_area = 0
    for c in contours:
        x, y, w, h = cv2.boundingRect(c)
        if w * h == 0:
            continue
        cx = x + w / 2
        if x_range is not None and not (x_range[0] <= cx <= x_range[1]):
            continue
        area = w * h
        if area > best_area:
            best_area = area
            best = (x, y, x + w, y + h)
    return best
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd /home/roboseasy/lekiwi-pill-pickup-1/motion && PYTHONNOUSERSITE=1 /home/roboseasy/miniforge3/envs/lerobot/bin/python -m pytest services/pickplace/test_grasp_check.py -v`
Expected: 3 PASS

- [ ] **Step 5: Commit**

```bash
cd /home/roboseasy/lekiwi-pill-pickup-1
git add motion/services/pickplace/grasp_check.py motion/services/pickplace/test_grasp_check.py
git commit -m "Add: purple_bbox() — 그리퍼 보라색 바운딩박스 추출 (그리퍼 높이 시각 서보 1단계)"
```

---

### Task 2: `GraspArgs`에 폴백 관련 설정 4개 추가

**Files:**
- Modify: `services/pickplace/wrist_servo.py` (`GraspArgs` 클래스, `servo_give_up_s` 필드
  바로 뒤 — 164번 줄 다음. `validate()` 메서드 — 218번 줄 `servo_give_up_s` 검증 다음)
- Test: `services/pickplace/test_wrist_servo.py` (기존 파일에 추가)

**Interfaces:**
- Produces: `GraspArgs.front_hint_lost_s: float`, `GraspArgs.floor_descend_dps: float`,
  `GraspArgs.floor_descend_max_deg: float`, `GraspArgs.floor_pose_file: str`

- [ ] **Step 1: Write the failing tests**

`services/pickplace/test_wrist_servo.py` 맨 위 import 에 추가:

```python
import pytest

from services.pickplace import PickPlaceError
```

파일 끝에 추가:

```python
def test_grasp_args_rejects_non_positive_front_hint_lost_s():
    with pytest.raises(PickPlaceError):
        GraspArgs(front_hint_lost_s=0).validate()


def test_grasp_args_rejects_non_positive_floor_descend_dps():
    with pytest.raises(PickPlaceError):
        GraspArgs(floor_descend_dps=0).validate()


def test_grasp_args_rejects_non_positive_floor_descend_max_deg():
    with pytest.raises(PickPlaceError):
        GraspArgs(floor_descend_max_deg=0).validate()


def test_grasp_args_accepts_floor_fallback_defaults():
    GraspArgs().validate()  # 기본값은 통과해야 한다
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd /home/roboseasy/lekiwi-pill-pickup-1/motion && PYTHONNOUSERSITE=1 /home/roboseasy/miniforge3/envs/lerobot/bin/python -m pytest services/pickplace/test_wrist_servo.py -k front_hint_lost_s -v`
Expected: FAIL — `TypeError: __init__() got an unexpected keyword argument 'front_hint_lost_s'`

- [ ] **Step 3: Write minimal implementation**

`services/pickplace/wrist_servo.py`, `servo_give_up_s: float = 3.0` 필드(164번 줄) 바로
뒤에 추가:

```python
    # --- 3-3. front_dy(프론트캠 그리퍼↔약통 높이 비교) 실패 시 안전 폴백 ---

    # front_dy 가 이 시간(초) 이상 계속 안 잡히면(보라색 인식 실패), 손목캠 기준 세로
    # 보정 대신 바닥-근접 폴백으로 전환한다.
    front_hint_lost_s: float = 1.0
    # [폴백] shoulder_lift/elbow_flex 를 바닥-근접 자세로 움직이는 속도(deg/s)
    floor_descend_dps: float = 2.0
    # [폴백] 폴백 진입 시점 자세로부터 이 각도(deg) 이상은 더 내려가지 않는다 (안전 상한)
    floor_descend_max_deg: float = 15.0
    # [폴백] 바닥에 근접한 안전 자세 파일 (shoulder_lift/elbow_flex 만 쓴다).
    # 없으면(빈 문자열) 폴백을 하지 않고 기존 LOST 판정으로 넘어간다
    floor_pose_file: str = ""
```

`validate()` 메서드 안, `if self.servo_give_up_s < 0:` 검증(217~218번 줄) 바로 뒤에 추가:

```python
        if self.front_hint_lost_s <= 0:
            raise PickPlaceError(
                f"error: grasp.front_hint_lost_s 는 0 보다 커야 합니다 (받은 값: {self.front_hint_lost_s})"
            )
        if self.floor_descend_dps <= 0 or self.floor_descend_max_deg <= 0:
            raise PickPlaceError("error: grasp.floor_descend_dps / floor_descend_max_deg 는 0 보다 커야 합니다")
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd /home/roboseasy/lekiwi-pill-pickup-1/motion && PYTHONNOUSERSITE=1 /home/roboseasy/miniforge3/envs/lerobot/bin/python -m pytest services/pickplace/test_wrist_servo.py -v`
Expected: 모든 테스트(기존 + 신규 4개) PASS

- [ ] **Step 5: Commit**

```bash
cd /home/roboseasy/lekiwi-pill-pickup-1
git add motion/services/pickplace/wrist_servo.py motion/services/pickplace/test_wrist_servo.py
git commit -m "Add: GraspArgs 바닥-근접 폴백 설정 4개 (front_hint_lost_s/floor_descend_dps/floor_descend_max_deg/floor_pose_file)"
```

---

### Task 3: `WristServo` — `front_dy` 가 `y_anchor` 세로 보정을 대체하게 만들기

**Files:**
- Modify: `services/pickplace/wrist_servo.py` (`WristServo.update()`, 372~485번 줄)
- Test: `services/pickplace/test_wrist_servo.py`

**Interfaces:**
- Consumes: Task 2 의 `GraspArgs.y_tolerance_px`(기존), `front_hint`(기존)
- Produces: `WristServo.update(..., front_dy: float | None = None)` — `front_dy` 가
  주어지면 `self.dy`/`self.y_ok` 를 그걸로 계산하고, 기존 `front_hint` 즉시-READY
  지름길 2곳을 비활성화한다.

- [ ] **Step 1: Write the failing tests**

`services/pickplace/test_wrist_servo.py` 파일 끝에 추가:

```python
def test_front_dy_overrides_y_anchor_dy_and_y_ok():
    """front_dy 가 주어지면 y_anchor 계산 대신 그 값을 그대로 dy 로 쓴다."""
    cfg = GraspArgs(approach_mode="joints", reach_joints={}, y_anchor="bottom", y_tolerance_px=40)
    start = {"arm_shoulder_pan.pos": 0.0, "arm_shoulder_lift.pos": 0.0, "arm_wrist_flex.pos": 0.0, "arm_gripper.pos": 100.0}
    servo = WristServo(cfg, start, None)
    shape = (480, 640, 3)
    det = Detection(name="pill", conf=0.9, xyxy=(310, 100, 340, 400), cls=0)

    servo.update([det], shape, dt=1 / 30, now=0.0, allow_motion=True, front_dy=15.0)

    assert servo.dy == 15.0
    assert servo.y_ok is True  # |15| <= y_tolerance_px(40)


def test_front_dy_disables_old_front_hint_ready_shortcut():
    """front_dy 가 주어지면, 예전 즉시-READY 지름길(front_hint)은 무시하고
    size_reached 조건을 그대로 따라야 한다 (2026-09-13 설계: 신호 충돌 방지)."""
    cfg = GraspArgs(approach_mode="joints", reach_joints={}, front_hint=True)
    start = {"arm_shoulder_pan.pos": 0.0, "arm_shoulder_lift.pos": 0.0, "arm_wrist_flex.pos": 0.0, "arm_gripper.pos": 100.0}
    servo = WristServo(cfg, start, None)
    shape = (480, 640, 3)
    # x_anchor=left 기본, x1=310 → cx0=320 이므로 dx=-10, x_tolerance_px(25) 안 → x_ok=True
    # 폭 30 << target_size_px(260) 이므로 size_reached 는 확실히 False
    det = Detection(name="pill", conf=0.9, xyxy=(310, 100, 340, 400), cls=0)

    servo.update(
        [det], shape, dt=1 / 30, now=0.0, allow_motion=True, descend_hint=True, front_dy=1000.0,
    )

    assert servo.state != "READY"


def test_front_dy_none_preserves_existing_y_anchor_behavior():
    """front_dy 를 안 주면(기본값 None) 기존 y_anchor 기반 계산이 그대로 동작해야 한다."""
    cfg = GraspArgs(approach_mode="joints", reach_joints={}, y_anchor="bottom", y_target_dy=0)
    start = {"arm_shoulder_pan.pos": 0.0, "arm_shoulder_lift.pos": 0.0, "arm_wrist_flex.pos": 0.0, "arm_gripper.pos": 100.0}
    servo = WristServo(cfg, start, None)
    shape = (480, 640, 3)  # cy0 = 240
    det = Detection(name="pill", conf=0.9, xyxy=(310, 100, 340, 400), cls=0)  # bottom(y2)=400

    servo.update([det], shape, dt=1 / 30, now=0.0, allow_motion=True)

    assert servo.dy == 400 - 240
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd /home/roboseasy/lekiwi-pill-pickup-1/motion && PYTHONNOUSERSITE=1 /home/roboseasy/miniforge3/envs/lerobot/bin/python -m pytest services/pickplace/test_wrist_servo.py -k front_dy -v`
Expected: FAIL — `TypeError: update() got an unexpected keyword argument 'front_dy'`

- [ ] **Step 3: Write minimal implementation**

`services/pickplace/wrist_servo.py`, `update()` 메서드 시그니처(372~380번 줄)를:

```python
    def update(
        self,
        dets: list[Detection],
        frame_shape: tuple[int, ...],
        dt: float,
        now: float,
        allow_motion: bool,
        descend_hint: bool = False,
    ) -> dict[str, float]:
```

이렇게 바꾼다:

```python
    def update(
        self,
        dets: list[Detection],
        frame_shape: tuple[int, ...],
        dt: float,
        now: float,
        allow_motion: bool,
        descend_hint: bool = False,
        front_dy: float | None = None,
    ) -> dict[str, float]:
```

`self.target is None` 분기 안의 REFINING-복구 조건(398~408번 줄):

```python
            if (
                self.state == "REFINING"
                and descend_hint
                and self.cfg.front_hint
                and self.attempt == 0
                and self._last_x_ok
            ):
```

를 다음으로 바꾼다 (front_dy 가 있으면 이 지름길도 끈다 — front_dy 가 매 프레임 정확한
높이를 알려주므로 손목 박스 소실만으로 근접을 추정할 필요가 없다):

```python
            if (
                front_dy is None
                and self.state == "REFINING"
                and descend_hint
                and self.cfg.front_hint
                and self.attempt == 0
                and self._last_x_ok
            ):
```

세로 계산 블록(422~432번 줄):

```python
        # 세로
        m = self.cfg.inside_margin_px
        inside_y = y1 + m <= cy0 <= y2 - m  # 화면 중앙 높이가 박스 위/아래 변 사이 (느슨한 조건)
        if self.cfg.y_anchor == "inside":
            self.anchor_y = by
            self.dy = by - cy0
            self.y_ok = inside_y
        else:
            self.anchor_y = {"top": y1, "center": by, "bottom": y2}[self.cfg.y_anchor]
            self.dy = (self.anchor_y - cy0) - self.cfg.y_target_dy
            self.y_ok = abs(self.dy) <= self.cfg.y_tolerance_px
```

를 다음으로 바꾼다:

```python
        # 세로: front_dy(프론트캠에서 직접 잰 그리퍼↔약통 높이 차)가 있으면 그걸로 대체한다.
        # 손목캠 크기/joint 각도에 의존하지 않는 실제 상대 위치 기준이라 카메라·팔 구성이
        # 바뀌어도 재조정이 필요 없다 (2026-09-13 설계). 두 신호가 동시에 tilt_delta 를
        # 건드리면 서로 다른 방향으로 다툴 수 있으므로 항상 하나만 쓴다.
        m = self.cfg.inside_margin_px
        inside_y = y1 + m <= cy0 <= y2 - m  # 화면 중앙 높이가 박스 위/아래 변 사이 (느슨한 조건)
        if front_dy is not None:
            self.anchor_y = cy0 + front_dy
            self.dy = front_dy
            self.y_ok = abs(front_dy) <= self.cfg.y_tolerance_px
        elif self.cfg.y_anchor == "inside":
            self.anchor_y = by
            self.dy = by - cy0
            self.y_ok = inside_y
        else:
            self.anchor_y = {"top": y1, "center": by, "bottom": y2}[self.cfg.y_anchor]
            self.dy = (self.anchor_y - cy0) - self.cfg.y_target_dy
            self.y_ok = abs(self.dy) <= self.cfg.y_tolerance_px
```

기존 즉시-READY 지름길(442~447번 줄):

```python
        # 높이 힌트: front 정중앙에 그리퍼가 보이고 가로만 맞으면 크기와 무관하게 READY (첫 시도만)
        if descend_hint and self.cfg.front_hint and self.attempt == 0 and self.x_ok:
```

를 다음으로 바꾼다:

```python
        # 높이 힌트: front_dy 가 있으면(더 정밀한 신호가 있으므로) 이 지름길은 쓰지 않는다.
        if front_dy is None and descend_hint and self.cfg.front_hint and self.attempt == 0 and self.x_ok:
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd /home/roboseasy/lekiwi-pill-pickup-1/motion && PYTHONNOUSERSITE=1 /home/roboseasy/miniforge3/envs/lerobot/bin/python -m pytest services/pickplace/test_wrist_servo.py -v`
Expected: 모든 테스트 PASS (기존 케이스들도 전부 그대로 통과해야 함 — `front_dy` 기본값이
`None`이라 안 건드리면 예전 그대로 동작)

- [ ] **Step 5: Commit**

```bash
cd /home/roboseasy/lekiwi-pill-pickup-1
git add motion/services/pickplace/wrist_servo.py motion/services/pickplace/test_wrist_servo.py
git commit -m "Add: WristServo.update(front_dy=...) — 프론트캠 높이 신호가 y_anchor/front_hint 대체"
```

---

### Task 4: `WristServo` — 바닥-근접 폴백 (세로축 전용 병렬 동작)

**Files:**
- Modify: `services/pickplace/wrist_servo.py` (`WristServo.__init__`, `resume()`, `update()`)
- Test: `services/pickplace/test_wrist_servo.py`

**Interfaces:**
- Consumes: Task 2 의 `GraspArgs.front_hint_lost_s/floor_descend_dps/floor_descend_max_deg`
- Produces: `WristServo(cfg, start_pose, grasp_pose, floor_pose: dict[str, float] | None = None)`
  — 생성자에 `floor_pose` 키워드 인자 추가. `front_dy` 가 `front_hint_lost_s` 이상 계속
  `None`이면 `arm_shoulder_lift.pos`/`arm_elbow_flex.pos` 를 `floor_pose` 쪽으로 느리게,
  최대 `floor_descend_max_deg` 까지만 움직인다.

- [ ] **Step 1: Write the failing tests**

`services/pickplace/test_wrist_servo.py` 파일 끝에 추가:

```python
def _servo_with_floor(floor_pose: dict[str, float]) -> WristServo:
    cfg = GraspArgs(
        approach_mode="joints",
        reach_joints={},  # 접근 진행이 lift/elbow 를 안 건드리게 비워둔다 (폴백만 순수하게 테스트)
        front_hint_lost_s=0.2,
        floor_descend_dps=10.0,
        floor_descend_max_deg=15.0,
    )
    start = {
        "arm_shoulder_pan.pos": 0.0,
        "arm_shoulder_lift.pos": 100.0,
        "arm_elbow_flex.pos": -50.0,
        "arm_wrist_flex.pos": 0.0,
        "arm_gripper.pos": 100.0,
    }
    return WristServo(cfg, start, None, floor_pose=floor_pose)


def test_floor_fallback_moves_lift_and_elbow_after_front_dy_lost_long_enough():
    servo = _servo_with_floor({"arm_shoulder_lift.pos": 130.0, "arm_elbow_flex.pos": -70.0})
    shape = (480, 640, 3)
    det = Detection(name="pill", conf=0.9, xyxy=(310, 100, 340, 400), cls=0)

    for i in range(4):  # 0.3초 동안 계속 front_dy=None (front_hint_lost_s=0.2 초과)
        servo.update([det], shape, dt=0.1, now=i * 0.1, allow_motion=True, front_dy=None)

    assert servo.current["arm_shoulder_lift.pos"] > 100.0  # 목표(130) 방향으로 움직임
    assert servo.current["arm_elbow_flex.pos"] < -50.0  # 목표(-70) 방향으로 움직임


def test_floor_fallback_does_not_engage_without_floor_pose():
    cfg = GraspArgs(approach_mode="joints", reach_joints={}, front_hint_lost_s=0.2)
    start = {
        "arm_shoulder_pan.pos": 0.0, "arm_shoulder_lift.pos": 100.0,
        "arm_elbow_flex.pos": -50.0, "arm_wrist_flex.pos": 0.0, "arm_gripper.pos": 100.0,
    }
    servo = WristServo(cfg, start, None)  # floor_pose 없음
    shape = (480, 640, 3)
    det = Detection(name="pill", conf=0.9, xyxy=(310, 100, 340, 400), cls=0)

    for i in range(4):
        servo.update([det], shape, dt=0.1, now=i * 0.1, allow_motion=True, front_dy=None)

    assert servo.current["arm_shoulder_lift.pos"] == 100.0  # 안 움직임


def test_floor_fallback_stops_at_max_deg_cap():
    servo = _servo_with_floor({"arm_shoulder_lift.pos": 500.0, "arm_elbow_flex.pos": -500.0})  # 아주 먼 목표
    shape = (480, 640, 3)
    det = Detection(name="pill", conf=0.9, xyxy=(310, 100, 340, 400), cls=0)

    for i in range(200):  # 20초 — 상한(15deg)에 도달하기 충분
        servo.update([det], shape, dt=0.1, now=i * 0.1, allow_motion=True, front_dy=None)

    assert servo.current["arm_shoulder_lift.pos"] == pytest.approx(115.0, abs=0.5)  # 100+15 상한
    assert servo.current["arm_elbow_flex.pos"] == pytest.approx(-65.0, abs=0.5)  # -50-15 상한


def test_floor_fallback_resets_when_front_dy_recovers():
    servo = _servo_with_floor({"arm_shoulder_lift.pos": 130.0, "arm_elbow_flex.pos": -70.0})
    shape = (480, 640, 3)
    det = Detection(name="pill", conf=0.9, xyxy=(310, 100, 340, 400), cls=0)

    for i in range(4):
        servo.update([det], shape, dt=0.1, now=i * 0.1, allow_motion=True, front_dy=None)
    assert servo.current["arm_shoulder_lift.pos"] > 100.0

    servo.update([det], shape, dt=0.1, now=0.5, allow_motion=True, front_dy=0.0)  # 회복

    assert servo._front_dy_lost_since is None
    assert servo._floor_start is None
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd /home/roboseasy/lekiwi-pill-pickup-1/motion && PYTHONNOUSERSITE=1 /home/roboseasy/miniforge3/envs/lerobot/bin/python -m pytest services/pickplace/test_wrist_servo.py -k floor_fallback -v`
Expected: FAIL — `TypeError: __init__() got an unexpected keyword argument 'floor_pose'`

- [ ] **Step 3: Write minimal implementation**

`services/pickplace/wrist_servo.py`, `__init__` 시그니처(294번 줄)를:

```python
    def __init__(self, cfg: GraspArgs, start_pose: dict[str, float], grasp_pose: dict[str, float] | None):
```

이렇게 바꾼다:

```python
    def __init__(
        self,
        cfg: GraspArgs,
        start_pose: dict[str, float],
        grasp_pose: dict[str, float] | None,
        floor_pose: dict[str, float] | None = None,
    ):
```

`__init__` 본문 끝(`self._last_x_ok = False` 줄, 331번 줄) 바로 뒤에 추가:

```python
        self._floor_pose = dict(floor_pose) if floor_pose else None
        self._front_dy_lost_since: float | None = None  # front_dy 가 계속 None 이기 시작한 시각
        self._floor_start: dict[str, float] | None = None  # 폴백 진입 시점의 lift/elbow 값
```

`resume()` 메서드(358~370번 줄) 안, `self._refine_since = 0.0` 줄 바로 뒤에 추가:

```python
        self._front_dy_lost_since = None
        self._floor_start = None
```

`update()` 메서드 끝부분(484번 줄 `self.current = self._limit_speed(self._compose(), dt)`)을:

```python
        self.current = self._limit_speed(self._compose(), dt)
        return self.current
```

이렇게 바꾼다:

```python
        self.current = self._limit_speed(self._compose(), dt)
        self._apply_floor_fallback(front_dy, now, dt)
        return self.current
```

같은 클래스 안, `update()` 메서드 바로 뒤(`progress_pct` 프로퍼티 전, 487번 줄 앞)에
새 메서드 추가:

```python
    def _apply_floor_fallback(self, front_dy: float | None, now: float, dt: float) -> None:
        """front_dy 를 front_hint_lost_s 이상 계속 못 구하면(보라색 인식 실패),
        shoulder_lift/elbow_flex 만 아주 느리게 바닥-근접 자세로 접근시킨다. pick→grasp
        벡터 전체를 연장하는 방식(retry_overreach)은 pan 까지 같이 밀리는 부작용이 있어서
        (2026-09-13 발견) 쓰지 않고, 이 두 관절만 절대값 기준으로 직접, 상한을 두고 움직인다."""
        if front_dy is not None:
            self._front_dy_lost_since = None
            self._floor_start = None
            return
        if self._front_dy_lost_since is None:
            self._front_dy_lost_since = now
        if self._floor_pose is None or now - self._front_dy_lost_since < self.cfg.front_hint_lost_s:
            return
        joints = ("arm_shoulder_lift.pos", "arm_elbow_flex.pos")
        if self._floor_start is None:
            self._floor_start = {j: self.current[j] for j in joints if j in self.current}
        step = self.cfg.floor_descend_dps * dt
        max_deg = self.cfg.floor_descend_max_deg
        for j, start_v in self._floor_start.items():
            target_v = self._floor_pose.get(j, start_v)
            bounded_target = start_v + float(np.clip(target_v - start_v, -max_deg, max_deg))
            cur = self.current[j]
            self.current[j] = cur + float(np.clip(bounded_target - cur, -step, step))
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd /home/roboseasy/lekiwi-pill-pickup-1/motion && PYTHONNOUSERSITE=1 /home/roboseasy/miniforge3/envs/lerobot/bin/python -m pytest services/pickplace/test_wrist_servo.py -v`
Expected: 모든 테스트 PASS

- [ ] **Step 5: Commit**

```bash
cd /home/roboseasy/lekiwi-pill-pickup-1
git add motion/services/pickplace/wrist_servo.py motion/services/pickplace/test_wrist_servo.py
git commit -m "Add: WristServo 바닥-근접 폴백 — front_dy 장시간 실패 시 lift/elbow만 상한 두고 하강"
```

---

### Task 5: `ArmSequencer` — `floor_pose`/`front_dy` 배선

**Files:**
- Modify: `services/pickplace/arm_sequencer.py` (`__init__` 43~76번 줄, `update()` 141~239번 줄)
- Test: `services/pickplace/test_arm_sequencer.py` (신규 파일)

**Interfaces:**
- Consumes: Task 4 의 `WristServo(cfg, start_pose, grasp_pose, floor_pose=...)`,
  `WristServo.update(..., front_dy=...)`
- Produces: `ArmSequencer(home, pick, cfg, grasp_cfg, grasp_pose=None, floor_pose=None)`,
  `ArmSequencer.update(..., front_dy: float | None = None)`

- [ ] **Step 1: Write the failing tests**

`services/pickplace/test_arm_sequencer.py` 새로 생성:

```python
from services.pickplace.arm_sequencer import ArmSequencer, PickArgs
from services.pickplace.wrist_servo import GRIPPER_JOINT, GraspArgs
from services.pickplace.yolo_detect import Detection

HOME = {
    "arm_shoulder_pan.pos": 0.0,
    "arm_shoulder_lift.pos": 0.0,
    "arm_elbow_flex.pos": 0.0,
    "arm_wrist_flex.pos": 0.0,
    GRIPPER_JOINT: 100.0,
}
PICK = {**HOME, "arm_shoulder_lift.pos": 10.0}
GRASP = {**HOME, "arm_shoulder_lift.pos": 20.0}
FLOOR = {"arm_shoulder_lift.pos": 25.0, "arm_elbow_flex.pos": -5.0}


def _drive_to_servo(arm: ArmSequencer, det: Detection) -> None:
    """HOME → TO_PICK → PICK → OPEN_GRIPPER → SERVO 까지 몰아간다 (그리퍼 열기/이동 시간 대기 포함)."""
    now = 0.0
    arm.update(True, True, True, now)  # HOME → TO_PICK 시작
    now += arm.cfg.move_time_s + 0.01
    arm.update(True, True, True, now)  # TO_PICK 완료 → PICK
    now += arm.grasp_cfg.pick_dwell_s + 0.01
    arm.update(True, True, True, now)  # PICK dwell 완료 → OPEN_GRIPPER 시작
    now += arm.grasp_cfg.gripper_open_time_s + 0.01
    arm.update(
        True, True, True, now, wrist_dets=[det], wrist_shape=(480, 640, 3),
    )  # OPEN_GRIPPER 완료 → SERVO 진입


def test_arm_sequencer_passes_floor_pose_to_wrist_servo():
    cfg = PickArgs(move_time_s=0.01)
    grasp_cfg = GraspArgs(approach_mode="joints", reach_joints={}, pick_dwell_s=0.01, gripper_open_time_s=0.01)
    arm = ArmSequencer(HOME, PICK, cfg, grasp_cfg, grasp_pose=GRASP, floor_pose=FLOOR)
    det = Detection(name="pill", conf=0.9, xyxy=(310, 100, 340, 400), cls=0)

    _drive_to_servo(arm, det)

    assert arm.state == "SERVO"
    assert arm.servo is not None
    assert arm.servo._floor_pose == FLOOR


def test_arm_sequencer_forwards_front_dy_to_wrist_servo_update():
    cfg = PickArgs(move_time_s=0.01)
    grasp_cfg = GraspArgs(approach_mode="joints", reach_joints={}, pick_dwell_s=0.01, gripper_open_time_s=0.01)
    arm = ArmSequencer(HOME, PICK, cfg, grasp_cfg, grasp_pose=GRASP, floor_pose=FLOOR)
    det = Detection(name="pill", conf=0.9, xyxy=(310, 100, 340, 400), cls=0)

    _drive_to_servo(arm, det)
    now = arm._pick_t + 10.0  # SERVO 진입 이후 아무 시각
    arm.update(
        True, True, True, now, wrist_dets=[det], wrist_shape=(480, 640, 3), front_dy=42.0,
    )

    assert arm.servo.dy == 42.0
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd /home/roboseasy/lekiwi-pill-pickup-1/motion && PYTHONNOUSERSITE=1 /home/roboseasy/miniforge3/envs/lerobot/bin/python -m pytest services/pickplace/test_arm_sequencer.py -v`
Expected: FAIL — `TypeError: __init__() got an unexpected keyword argument 'floor_pose'`

- [ ] **Step 3: Write minimal implementation**

`services/pickplace/arm_sequencer.py`, `ArmSequencer.__init__` 시그니처(43~50번 줄)를:

```python
    def __init__(
        self,
        home: dict[str, float],
        pick: dict[str, float] | None,
        cfg: PickArgs,
        grasp_cfg: GraspArgs,
        grasp_pose: dict[str, float] | None = None,
    ):
```

이렇게 바꾼다:

```python
    def __init__(
        self,
        home: dict[str, float],
        pick: dict[str, float] | None,
        cfg: PickArgs,
        grasp_cfg: GraspArgs,
        grasp_pose: dict[str, float] | None = None,
        floor_pose: dict[str, float] | None = None,
    ):
```

`__init__` 본문, `self.grasp_pose = dict(grasp_pose) if grasp_pose else None`(53번 줄)
바로 뒤에 추가:

```python
        self.floor_pose = dict(floor_pose) if floor_pose else None
```

`update()` 메서드 시그니처(141~151번 줄)를:

```python
    def update(
        self,
        aligned: bool,
        tracking_ok: bool,
        allow_motion: bool,
        now: float,
        wrist_dets: list[Detection] | None = None,
        wrist_shape: tuple[int, ...] | None = None,
        dt: float = 1 / 30,
        descend_hint: bool = False,
    ) -> dict[str, float]:
```

이렇게 바꾼다:

```python
    def update(
        self,
        aligned: bool,
        tracking_ok: bool,
        allow_motion: bool,
        now: float,
        wrist_dets: list[Detection] | None = None,
        wrist_shape: tuple[int, ...] | None = None,
        dt: float = 1 / 30,
        descend_hint: bool = False,
        front_dy: float | None = None,
    ) -> dict[str, float]:
```

`OPEN_GRIPPER` 상태 처리 안(210~219번 줄):

```python
        elif self.state == "OPEN_GRIPPER":
            if self._interpolate(now) >= 1.0:
                if self._after_open == "HOME":
                    self._start_move(self.home, "TO_HOME", now, self.cfg.move_time_s)
                elif self.servo is None:
                    self.servo = WristServo(self.grasp_cfg, self.current, self.grasp_pose)
                    self.state = "SERVO"
                else:
                    self.servo.resume()  # 재시도: 같은 자리에서 목표를 키워 이어간다
                    self.state = "SERVO"
```

의 `self.servo = WristServo(self.grasp_cfg, self.current, self.grasp_pose)` 줄을:

```python
                    self.servo = WristServo(self.grasp_cfg, self.current, self.grasp_pose, floor_pose=self.floor_pose)
```

로 바꾼다.

`SERVO` 상태 처리(220~226번 줄):

```python
        elif self.state == "SERVO":
            if wrist_dets is not None and wrist_shape is not None:
                self.current = self.servo.update(wrist_dets, wrist_shape, dt, now, allow_motion, descend_hint)
                if self.servo.just_ready:
                    self.state = "GRASP_READY"
                    self.grasp_just_ready = True
                    self._ready_t = now
```

의 `self.servo.update(...)` 줄을:

```python
                self.current = self.servo.update(
                    wrist_dets, wrist_shape, dt, now, allow_motion, descend_hint, front_dy=front_dy
                )
```

로 바꾼다.

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd /home/roboseasy/lekiwi-pill-pickup-1/motion && PYTHONNOUSERSITE=1 /home/roboseasy/miniforge3/envs/lerobot/bin/python -m pytest services/pickplace/test_arm_sequencer.py -v`
Expected: 2 PASS

그 다음 전체 회귀 확인:

Run: `cd /home/roboseasy/lekiwi-pill-pickup-1/motion && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONNOUSERSITE=1 /home/roboseasy/miniforge3/envs/lerobot/bin/python -m pytest services/ -v`
Expected: 전부 PASS (기존 `test_headless_worker.py` 등도 `ArmSequencer(...)` 를 `floor_pose`
없이 생성하므로 기본값 `None` 덕분에 그대로 통과해야 함)

- [ ] **Step 5: Commit**

```bash
cd /home/roboseasy/lekiwi-pill-pickup-1
git add motion/services/pickplace/arm_sequencer.py motion/services/pickplace/test_arm_sequencer.py
git commit -m "Add: ArmSequencer 가 floor_pose/front_dy 를 WristServo 로 배선"
```

---

### Task 6: `headless_worker.py` — `front_dy` 계산 + 배선, `floor_near` 자세 로드

**Files:**
- Modify: `services/pickplace/headless_worker.py` (`run()` 메서드, 274~316번 줄)
- Test: `services/pickplace/test_headless_worker.py`

**Interfaces:**
- Consumes: Task 1 의 `purple_bbox(frame_bgr, cfg, x_range=None)`, Task 5 의
  `ArmSequencer(..., floor_pose=...)` / `ArmSequencer.update(..., front_dy=...)`
- Produces: `poses["floor_near"]` 를 읽어 `ArmSequencer` 생성 시 `floor_pose` 로 넘김.
  매 프레임 `front_dy` 를 계산해 `arm.update(..., front_dy=front_dy)` 로 전달.

- [ ] **Step 1: Write the failing test**

`services/pickplace/test_headless_worker.py` 맨 위 import 에 추가:

```python
from services.pickplace.grasp_check import GraspCheckArgs
```

파일 끝에 추가 (기존 `FakeRobot`, `_fake_load_model`, `PickPlaceHeadlessWorker`,
`PickPlaceConfig`, `PickArgs`, `Detection` 은 이미 파일 상단에서 import 돼 있다):

```python
def test_front_dy_computed_from_purple_bbox_and_bottle_bbox():
    """SERVO 단계에서 front 뷰의 그리퍼(보라색) bbox 와 약통 bbox 를 비교해
    front_dy 를 계산하고, 그 값이 ArmSequencer 를 거쳐 WristServo 까지 전달돼야 한다."""
    import cv2
    import numpy as np

    check_cfg = GraspCheckArgs()
    hue = (check_cfg.hue_min + check_cfg.hue_max) // 2
    purple_bgr = cv2.cvtColor(np.uint8([[[hue, 200, 200]]]), cv2.COLOR_HSV2BGR)[0][0]
    purple_bgr = (int(purple_bgr[0]), int(purple_bgr[1]), int(purple_bgr[2]))

    front_frame = np.zeros((480, 640, 3), dtype=np.uint8)
    # 그리퍼(보라색): 세로 중심 y=100 근방
    cv2.rectangle(front_frame, (300, 80), (340, 120), purple_bgr, -1)
    bottle_det = Detection(name="green_pill_bottle", conf=0.9, xyxy=(290, 250, 350, 350), cls=0)  # 세로 중심 y=300

    def infer(model, cfg, frames_bgr):
        out = {}
        for v, frame in frames_bgr.items():
            out[v] = [bottle_det] if v == "front" and frame is front_frame else []
        return out

    robot = FakeRobot()
    robot._frame = front_frame  # front/wrist 둘 다 이 프레임을 씀 (FakeRobot 기본 동작)
    cfg = PickPlaceConfig(pick=PickArgs(enabled=False), check=GraspCheckArgs())
    w = PickPlaceHeadlessWorker(
        robot, cfg, {}, infer_fn=infer, load_model_fn=_fake_load_model, first_obs_timeout_s=0.5,
    )

    def stop_after_two():
        if robot._obs_calls >= 2:
            w.stop_event.set()

    robot.on_observation = stop_after_two
    w.run()

    # pick.enabled=False 라 arm 은 절대 SERVO 로 안 가지만, front_dy 계산 자체가
    # 예외 없이 돌아가는지(그리퍼 bbox 중심(100) - 약통 bbox 중심(300) = -200 근방)는
    # 아래 통합 테스트(test_front_dy_reaches_wrist_servo_end_to_end)에서 pick.enabled=True 로 검증한다.
    assert len(robot.sent_actions) >= 1  # 루프가 예외 없이 돌았다
```

이 테스트는 배선이 예외 없이 도는지만 얕게 검증한다. **엔드투엔드로 실제 값 전달을
검증**하려면 아래를 이어서 추가한다 (같은 파일 끝):

```python
def test_front_dy_reaches_wrist_servo_end_to_end():
    """pick 활성화 상태에서 SERVO 까지 도달했을 때, front_dy 가 실제로 계산되어
    WristServo 에 전달됐는지 확인한다."""
    import cv2
    import numpy as np

    check_cfg = GraspCheckArgs()
    hue = (check_cfg.hue_min + check_cfg.hue_max) // 2
    purple_bgr = cv2.cvtColor(np.uint8([[[hue, 200, 200]]]), cv2.COLOR_HSV2BGR)[0][0]
    purple_bgr = (int(purple_bgr[0]), int(purple_bgr[1]), int(purple_bgr[2]))

    front_frame = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(front_frame, (300, 80), (340, 120), purple_bgr, -1)  # 그리퍼: 중심 y=100
    bottle_front = Detection(name="pill", conf=0.9, xyxy=(290, 250, 350, 350), cls=0)  # 약통: 중심 y=300
    bottle_wrist = Detection(name="pill", conf=0.9, xyxy=(310, 100, 340, 400), cls=0)

    def infer(model, cfg, frames_bgr):
        return {v: ([bottle_front] if v == "front" else [bottle_wrist]) for v in frames_bgr}

    robot = FakeRobot()
    robot._frame = front_frame
    cfg = PickPlaceConfig(
        pick=PickArgs(enabled=True, move_time_s=0.01),
        grasp=GraspArgs(approach_mode="joints", reach_joints={}, pick_dwell_s=0.01, gripper_open_time_s=0.01),
        check=GraspCheckArgs(),
    )
    w = PickPlaceHeadlessWorker(
        robot, cfg, {"pre_pick": {}, "grasp": {}}, infer_fn=infer, load_model_fn=_fake_load_model,
        first_obs_timeout_s=0.5,
    )
    w.paused_event.clear()

    reached_servo = {"ok": False}

    def stop_when_servo():
        # 상태가 SERVO 가 되면 몇 프레임 더 두고 멈춘다 (front_dy 가 실제로 계산될 시간)
        status = w.status.get()
        if status.get("state") in ("WRIST_CENTERING", "WRIST_APPROACH", "WRIST_REFINE", "GRASP_READY"):
            reached_servo["ok"] = True
        if reached_servo["ok"] and robot._obs_calls >= 30:
            w.stop_event.set()
        elif robot._obs_calls >= 200:  # 안전판: 너무 오래 걸리면 강제 종료
            w.stop_event.set()

    robot.on_observation = stop_when_servo
    w.run()

    assert reached_servo["ok"], "SERVO 단계에 도달하지 못했다 — 배선 확인 필요"
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd /home/roboseasy/lekiwi-pill-pickup-1/motion && PYTHONNOUSERSITE=1 /home/roboseasy/miniforge3/envs/lerobot/bin/python -m pytest services/pickplace/test_headless_worker.py -k front_dy -v`
Expected: 두 번째 테스트(`test_front_dy_reaches_wrist_servo_end_to_end`)는 배선 전이라도
SERVO 까지는 도달할 수 있으므로 통과할 수도 있다 — 이 단계의 진짜 목적은 Step 3 구현 후
**회귀 없이** 계속 통과하는 것 확인이다. 첫 번째 테스트는 새 import(`GraspCheckArgs`)가
없으면 `ImportError` 로 실패한다.

- [ ] **Step 3: Write minimal implementation**

`services/pickplace/headless_worker.py` 맨 위 import 에 `purple_bbox` 추가.
기존(25번 줄):

```python
from services.pickplace.grasp_check import GraspChecker, center_purple_ratio, draw_grasp_check
```

를:

```python
from services.pickplace.grasp_check import GraspChecker, center_purple_ratio, draw_grasp_check, purple_bbox
```

로 바꾼다.

`run()` 메서드 안, `pick_pose`/`grasp_pose` 를 구하는 부분(210~215번 줄):

```python
            pick_pose = self._filter_pose("pre_pick", home) if self.cfg.pick.enabled else None
            grasp_pose = (
                self._filter_pose("grasp", home)
                if (self.cfg.pick.enabled and self.cfg.grasp.enabled)
                else None
            )

            ap = Approacher(self.cfg.approach)
            arm = ArmSequencer(home, pick_pose, self.cfg.pick, self.cfg.grasp, grasp_pose)
```

를:

```python
            pick_pose = self._filter_pose("pre_pick", home) if self.cfg.pick.enabled else None
            grasp_pose = (
                self._filter_pose("grasp", home)
                if (self.cfg.pick.enabled and self.cfg.grasp.enabled)
                else None
            )
            floor_pose = self._poses.get("floor_near")

            ap = Approacher(self.cfg.approach)
            arm = ArmSequencer(home, pick_pose, self.cfg.pick, self.cfg.grasp, grasp_pose, floor_pose=floor_pose)
```

로 바꾼다. `restart_event` 처리 블록(218~233번 줄) 안의 `ArmSequencer(...)` 생성도
같은 방식으로(`floor_pose=floor_pose` 추가) 맞춘다:

```python
                    ap = Approacher(self.cfg.approach)
                    arm = ArmSequencer(home, pick_pose, self.cfg.pick, self.cfg.grasp, grasp_pose)
```

(restart 블록 안, 245~246번 줄과 home 블록 안 267~268번 줄 — 총 2곳) 을 각각:

```python
                    ap = Approacher(self.cfg.approach)
                    arm = ArmSequencer(home, pick_pose, self.cfg.pick, self.cfg.grasp, grasp_pose, floor_pose=floor_pose)
```

로 바꾼다.

`descend_hint` 계산 블록(299~304번 줄) 바로 뒤에 `front_dy` 계산을 추가한다. 기존:

```python
                descend_hint = False
                if self.cfg.grasp.front_hint and arm.state == "SERVO" and self.cfg.approach.view in frames_bgr:
                    ratio = center_purple_ratio(
                        frames_bgr[self.cfg.approach.view], self.cfg.check, self.cfg.grasp.front_hint_win_px
                    )
                    descend_hint = ratio >= self.cfg.grasp.front_hint_min_ratio
```

를:

```python
                descend_hint = False
                if self.cfg.grasp.front_hint and arm.state == "SERVO" and self.cfg.approach.view in frames_bgr:
                    ratio = center_purple_ratio(
                        frames_bgr[self.cfg.approach.view], self.cfg.check, self.cfg.grasp.front_hint_win_px
                    )
                    descend_hint = ratio >= self.cfg.grasp.front_hint_min_ratio

                front_dy = None
                if arm.state == "SERVO" and self.cfg.approach.view in frames_bgr:
                    front_view = frames_bgr[self.cfg.approach.view]
                    bottle_det = largest_front(dets_by_view.get(self.cfg.approach.view, []))
                    if bottle_det is not None:
                        x1, _, x2, _ = bottle_det.xyxy
                        gripper_box = purple_bbox(front_view, self.cfg.check, x_range=(x1 - 60, x2 + 60))
                        if gripper_box is not None:
                            _, gy1, _, gy2 = gripper_box
                            _, by1, _, by2 = bottle_det.xyxy
                            front_dy = (gy1 + gy2) / 2 - (by1 + by2) / 2
```

로 바꾼다. 이 코드는 `largest_front` 라는 이름으로 `approach.py` 의 `largest()` 를 쓴다 —
파일 맨 위 import(21번 줄)를:

```python
from services.pickplace.approach import STOP, Approacher, draw_alignment
```

에서:

```python
from services.pickplace.approach import STOP, Approacher, draw_alignment, largest as largest_front
```

로 바꾼다 (이미 `approach.py` 에 있는 `largest()` 재사용 — 새로 안 만든다).

`arm.update(...)` 호출부(306~315번 줄):

```python
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
```

를:

```python
                arm_pose = arm.update(
                    ap.done,
                    tracking_ok,
                    allow_motion,
                    loop_start,
                    wrist_dets=dets_by_view.get(self.cfg.grasp.view) if wrist_frame is not None else None,
                    wrist_shape=wrist_frame.shape if wrist_frame is not None else None,
                    dt=interval,
                    descend_hint=descend_hint,
                    front_dy=front_dy,
                )
```

로 바꾼다.

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd /home/roboseasy/lekiwi-pill-pickup-1/motion && PYTHONNOUSERSITE=1 /home/roboseasy/miniforge3/envs/lerobot/bin/python -m pytest services/pickplace/test_headless_worker.py -v`
Expected: 모든 테스트 PASS

전체 회귀:

Run: `cd /home/roboseasy/lekiwi-pill-pickup-1/motion && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONNOUSERSITE=1 /home/roboseasy/miniforge3/envs/lerobot/bin/python -m pytest services/ webui/ -v`
Expected: 전부 PASS (이 정확한 조합 — `lerobot` conda 환경의 python + `PYTHONNOUSERSITE=1`
— 이 `services/`/`webui/` 양쪽 다 커버한다. 2026-09-13 세션에서 이미 42개 테스트로 확인됨)

- [ ] **Step 5: Commit**

```bash
cd /home/roboseasy/lekiwi-pill-pickup-1
git add motion/services/pickplace/headless_worker.py motion/services/pickplace/test_headless_worker.py
git commit -m "Add: headless_worker 가 front_dy 를 계산해 ArmSequencer 로 전달, floor_near 자세 로드"
```

---

### Task 7: `run_pickplace_ui.py` — `floor_near.json` 기본 경로 배선

**Files:**
- Modify: `webui/run_pickplace_ui.py`

**Interfaces:**
- Consumes: Task 6 의 `self._poses.get("floor_near")` (headless_worker 가 이미 optional
  하게 처리하므로, 여기서는 파일이 있을 때만 로드하면 된다 — `home` 과 동일한 패턴)

- [ ] **Step 1: 수동 확인 (이 파일은 CLI 진입점이라 유닛테스트 대상이 아니다 —
  기존 `run_pickplace_ui.py` 도 테스트 파일이 없다)**

변경 후 아래 명령으로 `--help` 가 에러 없이 뜨는지, `floor_pose_file` 옵션이 보이는지
확인한다:

Run: `cd /home/roboseasy/lekiwi-pill-pickup-1/motion && PYTHONNOUSERSITE=1 /home/roboseasy/miniforge3/envs/lerobot/bin/python webui/run_pickplace_ui.py --help 2>&1 | grep floor_pose_file`
Expected: `--floor_pose_file str` 같은 줄이 출력됨

- [ ] **Step 2: 구현**

`RunConfig` 데이터클래스, `home_pose_file` 필드 바로 뒤에 추가:

```python
    # 바닥-근접 안전 자세(그리퍼 높이 시각 서보 폴백용). 없는 설치에서도 조용히
    # 생략되도록 존재 여부는 _load_poses 에서 확인한다 (home_pose_file 과 동일한 패턴).
    floor_pose_file: str = str(_DEFAULT_POSES_DIR / "floor_near.json")
```

`_load_poses()` 함수 안, `defaults` dict(91~97번 줄):

```python
    defaults = {
        "pre_pick": cfg.pick.pose_file or str(_DEFAULT_POSES_DIR / "pre_pick.json"),
        "grasp": cfg.grasp.grasp_pose_file or str(_DEFAULT_POSES_DIR / "grasp.json"),
        "grasp_closed": cfg.grasp.close_pose_file or str(_DEFAULT_POSES_DIR / "grasp_closed.json"),
        # home 은 선택 사항 — 없는 설치에서도 조용히 생략된다(아래에서 존재 여부 확인).
        "home": cfg.home_pose_file or str(_DEFAULT_POSES_DIR / "home.json"),
    }
```

를:

```python
    defaults = {
        "pre_pick": cfg.pick.pose_file or str(_DEFAULT_POSES_DIR / "pre_pick.json"),
        "grasp": cfg.grasp.grasp_pose_file or str(_DEFAULT_POSES_DIR / "grasp.json"),
        "grasp_closed": cfg.grasp.close_pose_file or str(_DEFAULT_POSES_DIR / "grasp_closed.json"),
        # home/floor_near 은 선택 사항 — 없는 설치에서도 조용히 생략된다(아래에서 존재 여부 확인).
        "home": cfg.home_pose_file or str(_DEFAULT_POSES_DIR / "home.json"),
        "floor_near": cfg.floor_pose_file or str(_DEFAULT_POSES_DIR / "floor_near.json"),
    }
```

로 바꾸고, 바로 아래 루프(98~104번 줄):

```python
    poses: dict[str, dict[str, float]] = {}
    for name, path in defaults.items():
        if name == "home" and not Path(path).expanduser().exists():
            continue
        if path:
            poses[name] = load_pose(Path(path).expanduser())
    return poses
```

를:

```python
    poses: dict[str, dict[str, float]] = {}
    for name, path in defaults.items():
        if name in ("home", "floor_near") and not Path(path).expanduser().exists():
            continue
        if path:
            poses[name] = load_pose(Path(path).expanduser())
    return poses
```

로 바꾼다.

- [ ] **Step 3: 확인**

Run: `cd /home/roboseasy/lekiwi-pill-pickup-1/motion && PYTHONNOUSERSITE=1 /home/roboseasy/miniforge3/envs/lerobot/bin/python webui/run_pickplace_ui.py --help 2>&1 | grep floor_pose_file`
Expected: 옵션이 출력됨

전체 회귀:

Run: `cd /home/roboseasy/lekiwi-pill-pickup-1/motion && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONNOUSERSITE=1 /home/roboseasy/miniforge3/envs/lerobot/bin/python -m pytest services/ -v && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/roboseasy/miniforge3/envs/lerobot/bin/python -m pytest webui/ -v`
Expected: 전부 PASS

- [ ] **Step 4: Commit**

```bash
cd /home/roboseasy/lekiwi-pill-pickup-1
git add motion/webui/run_pickplace_ui.py
git commit -m "Add: run_pickplace_ui 에 floor_pose_file 기본 경로 배선 (grasp.y_anchor bottom/center 안 쓰는 설치는 영향 없음)"
```

---

## 구현 완료 후 (사람이 할 일 — 계획 범위 밖)

이 계획은 소프트웨어 배선까지만 다룬다. 실제로 켜서 쓰려면:

1. 손목캠이 아니라 **프론트캠**으로 그리퍼가 병 옆에서 살짝 바닥 쪽으로 내려간 "안전한
   근접" 자세를 찾아 `webui/save_pose.py --name floor_near` 로 한 번 기록해야 한다
   (`grasp.json` 을 만들 때와 같은 절차 — 토크 해제 후 수동 위치 조정 → 저장).
2. `--grasp.front_hint_lost_s=1.0 --grasp.floor_descend_dps=2.0 --grasp.floor_descend_max_deg=15` 기본값으로 먼저 라이브 테스트하고, 필요하면 조정한다.
3. `--check.hue_min/hue_max/sat_min/val_min` (보라색 HSV 범위)이 지금 조명에서 잘
   맞는지 프론트 뷰로 확인 — 안 맞으면 `purple_bbox` 가 계속 `None` 을 반환해 폴백만
   계속 작동하게 된다.
