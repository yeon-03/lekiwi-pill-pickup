# LeKiwi 약통 픽업 데모 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 음성 명령("약 좀 찾아서 갖다줘")을 받으면 LeKiwi가 책상 위 눈앞의(크기/자세가
고정된) 약통을 손목 카메라 YOLO 인식으로 찾아 정밀하게 집어드는 파이프라인을 만든다.

**Architecture:** 순수 로직(거리추정/정렬판정/그립성공판정/상태머신)과 하드웨어 I/O
(카메라 프레임, LeKiwiClient, YOLO 모델, 서보 버스)를 분리한다. 순수 로직은 하드웨어
없이 TDD로 개발·검증하고, 통합 스크립트(`pick_pill_bottle.py`)가 이들을 실제 하드웨어와
연결한다. `robot_ws`(별도 저장소) 쪽은 기존 SSH 트리거 인프라(`lekiwi_command_node.py`)에
새 스킬 항목 하나만 추가하는 최소 변경.

**Tech Stack:** Python 3.12, `lerobot`(LeKiwiClient), `ultralytics`(YOLO), `opencv-python`,
`pytest`. 노트북(RTX 3070 8GB)에 독립 venv로 설치(시스템/robot_ws 의존성과 격리 — 이
프로젝트가 CosyVoice/DeepFilterNet 등에서 반복 겪은 numpy/torch 버전 충돌 방지 관례).

## 구현 현황 (2026-08-14)

- ✅ **완료** (하드웨어 불필요, 코드+테스트로 검증됨): Task 2(스캐폴딩), Task 6~10
  (거리추정/정렬오차/그립판정/YOLO파싱/상태머신, pytest 28/28 통과), Task 13(통합
  스크립트, 실제 import 확인 완료 — 관절 게인은 자리표시), Task 14~15(robot_ws —
  `feature/lekiwi-pill-bottle` 브랜치에 커밋, colcon build 통과, pytest 23/23 통과)
- ⏳ **미착수 — 사람이 실기기로 직접 진행 필요**: Task 1(시연 장소 확인), Task 3
  (그리퍼 부하 실측), Task 4(카메라 캘리브레이션), Task 5(SSH 인프라), Task 11
  (YOLO 사전학습 1차 시험), Task 12(파인튜닝, Task 11 결과에 따라 조건부), Task 16
  (실기기 통합 리허설)

## Global Constraints

- 설계 문서: `docs/superpowers/specs/2026-08-12-lekiwi-pill-pickup-design.md` (이 저장소)
- 타임라인: 1차 영상 제출 마감 2주 후, 2차 라이브 시연은 그 직후(며칠 안) — 같은 시스템이
  라이브에서도 그대로 동작해야 함(1차용 별도 임시방편 금지, 특히 결과 조작 절대 금지)
- 약통은 종류/크기 고정, 놓는 자세 규칙화(예: 라벨이 카메라를 향하게 눕혀서)를 전제로
  설계한다 — 임의 자세/임의 물체 대응은 이번 스코프 밖
- 바퀴(베이스) 이동은 픽업 로직에 관여하지 않는다 — 팔+손목 카메라만 사용
- 순수 로직 모듈은 하드웨어/네트워크 의존성이 없어야 한다(테스트가 실제 로봇 없이 돌아야 함)
- 실패해도 되는 이번 스코프 밖 항목(설계 문서 "제외" 절 참고): SLAM, 바닥 전체 탐색,
  임의 자세 대응, 아이작심 실시간 연동, 그립 재시도 로직
- 폴백 플랜: 이 파이프라인이 시간 안에 안정화 안 되면 이미 검증된 음성→LeKiwi 이동
  트리거(`robot_ws`의 `move_skill.py` 경로)만으로 라이브 시연 축소 가능 — 이 계획의
  Task 13/14(robot_ws 변경)는 새 스킬을 "추가"하는 것뿐이라 기존 forward/backward/
  left/right는 전혀 건드리지 않으므로 폴백은 언제나 자동으로 살아있다

---

## Task 1: 확인 — 2차 라이브 시연 장소가 실제 책상인지

**이 태스크는 코드가 아니라 사람이 확인해야 하는 사실 확인 항목이다.** 이후 모든
설계(카메라 거리/캘리브레이션 값, 팔 접근 범위)가 "고정 높이 책상, 좁은 범위"라는
전제 위에 서 있어서, 이게 틀리면 Task 2~12 상당수를 다시 설계해야 한다.

- [ ] **Step 1: 대회 담당자/장소 안내에 실제 시연 공간이 책상(고정 높이 테이블)인지,
  아니면 바닥/다른 형태인지 확인**
- [ ] **Step 2: 확인된 내용을 설계 문서에 반영**

```bash
cd /home/roboseasy/lekiwi-pill-pickup
# 확인 결과에 따라 docs/superpowers/specs/2026-08-12-lekiwi-pill-pickup-design.md의
# "열린 질문" 섹션 첫 항목을 체크하고, 실제 결과를 한 줄로 기록
```

- [ ] **Step 3: 커밋**

```bash
git add docs/superpowers/specs/2026-08-12-lekiwi-pill-pickup-design.md
git commit -m "Docs: 라이브 시연 장소 확인 결과 반영"
```

⚠️ **만약 책상이 아닌 것으로 확인되면, 이 계획의 Task 3(카메라 캘리브레이션)과
Task 12(통합 스크립트)의 가정을 재검토해야 한다 — 코드를 계속 진행하기 전에 먼저
설계 문서를 갱신할 것.**

---

## Task 2: 저장소 스캐폴딩 (venv, 패키지 구조, 테스트 설정)

**Files:**
- Create: `/home/roboseasy/lekiwi-pill-pickup/pyproject.toml`
- Create: `/home/roboseasy/lekiwi-pill-pickup/src/lekiwi_pill_pickup/__init__.py`
- Create: `/home/roboseasy/lekiwi-pill-pickup/tests/__init__.py`
- Create: `/home/roboseasy/lekiwi-pill-pickup/.gitignore`

**Interfaces:**
- Produces: `src/lekiwi_pill_pickup/` 패키지 디렉터리(이후 모든 모듈이 여기 위치),
  `pytest`가 `src/`를 자동으로 import 경로에 포함(pythonpath 설정)

- [ ] **Step 1: 독립 venv 생성 및 기본 의존성 설치**

```bash
cd /home/roboseasy/lekiwi-pill-pickup
python3.12 -m venv venv
source venv/bin/activate
pip install --upgrade pip
# 이 노트북의 CUDA 드라이버(13.x)와 호환되는 torch 휠 — CosyVoice 서브프로젝트에서
# 이미 검증된 조합(torch==2.4.1+cu121)을 재사용. ultralytics/lerobot이 다른 torch
# 버전을 요구하면 이 값은 조정 필요(설치 중 에러 메시지로 확인).
pip install torch==2.4.1 --index-url https://download.pytorch.org/whl/cu121
pip install ultralytics opencv-python-headless lerobot pytest
pip freeze > requirements-lock.txt
```

- [ ] **Step 2: 패키지/테스트 디렉터리 생성**

```bash
mkdir -p src/lekiwi_pill_pickup tests
touch src/lekiwi_pill_pickup/__init__.py tests/__init__.py
```

- [ ] **Step 3: `pyproject.toml` 작성**

```toml
[tool.pytest.ini_options]
pythonpath = ["src"]
testpaths = ["tests"]
```

- [ ] **Step 4: `.gitignore` 작성**

```
venv/
__pycache__/
*.pyc
.pytest_cache/
requirements-lock.txt
*.pt
*.onnx
data/
```

(모델 가중치·학습 데이터는 용량이 커서 git 추적 대상에서 제외 — `requirements-lock.txt`도
설치 환경마다 달라질 수 있어 제외하되, 아래 Step 1의 정확한 설치 커맨드는 이 계획
문서 자체에 남아있으므로 재현 가능하다)

- [ ] **Step 5: pytest가 빈 상태로 정상 인식되는지 확인**

Run: `cd /home/roboseasy/lekiwi-pill-pickup && source venv/bin/activate && pytest -v`
Expected: `no tests ran` (에러 없이 종료)

- [ ] **Step 6: 커밋**

```bash
git add pyproject.toml .gitignore src/ tests/
git commit -m "Add: 저장소 스캐폴딩(venv 설정, 패키지 구조, pytest 설정)"
```

---

## Task 3: [하드웨어 검증] 그리퍼 서보 Present_Load/Current 실측

**목적**: 설계 문서에서 "그리퍼가 서보 부하값으로 그립 성공 여부를 판정할 수 있다"고
가정했는데, 레지스터 존재만 확인했을 뿐(2026-08-12) 실제 판독값은 측정 안 함. Task 8
(`grip_sensing.py`)의 임계값(`empty_close_load`, `margin`)을 채우려면 실측이 필요하다.

**이 태스크는 실제 LeKiwi 하드웨어 접근이 필요해 사람이 직접 실행해야 한다.**

**Files:**
- Create: `/home/roboseasy/lekiwi-pill-pickup/scripts/measure_gripper_load.py`
- Create: `/home/roboseasy/lekiwi-pill-pickup/data/gripper_load_measurements.md` (측정 결과 기록)

- [ ] **Step 1: 측정 스크립트 작성**

```python
#!/usr/bin/env python
"""그리퍼 서보의 Present_Load/Present_Current를 여러 상황(빈손으로 닫음/약통을
잡고 닫음)에서 반복 측정해 콘솔에 출력한다. lerobot의 FeetechMotorsBus를 재사용."""
import sys
import time

from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.motors import Motor, MotorNormMode

GRIPPER_MOTOR_NAME = "gripper"  # SO-101 표준 명칭 — 실제 config_lekiwi.py/캘리브레이션
                                  # 파일로 재확인 필요, 다르면 이 상수만 수정
GRIPPER_MOTOR_ID = 6            # 위와 동일하게 재확인 필요


def main(port: str, label: str) -> None:
    bus = FeetechMotorsBus(
        port=port,
        motors={GRIPPER_MOTOR_NAME: Motor(GRIPPER_MOTOR_ID, "sts3215", MotorNormMode.RANGE_0_100)},
    )
    bus.connect()
    print(f"=== {label} — 5회 측정 ===")
    for i in range(5):
        load = bus.read("Present_Load", GRIPPER_MOTOR_NAME)
        current = bus.read("Present_Current", GRIPPER_MOTOR_NAME)
        print(f"[{i}] load={load} current={current}")
        time.sleep(0.5)
    bus.disconnect()


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print(f"사용법: {sys.argv[0]} <serial_port> <label>")
        sys.exit(1)
    main(sys.argv[1], sys.argv[2])
```

- [ ] **Step 2: LeKiwi에 SSH로 직접 접속해(원격 트리거 아니라 대화형 세션) 실행**

```bash
ssh -i ~/.ssh/id_ed25519_lekiwi roboseasy@192.168.0.201
# LeKiwi Pi 안에서:
~/lerobot_venv/bin/python measure_gripper_load.py /dev/ttyACM0 "빈손으로 완전히 닫음"
# (측정 스크립트를 미리 scp로 옮겨두거나 직접 붙여넣어 실행)
```

세 가지 상황을 각각 측정: **①빈손으로 완전히 닫음 ②약통을 잡고 닫음(정상 그립)
③빈손으로 살짝만 닫음(그립 실패를 흉내)**. `GRIPPER_MOTOR_NAME`/`GRIPPER_MOTOR_ID`가
실제와 다르면 `scan_motors.py`(이미 LeKiwi Pi에 있음, `~/scan_motors.py`)로 먼저
재확인.

- [ ] **Step 3: 측정값을 표로 기록**

```markdown
# 그리퍼 부하 측정 결과 (2026-08-XX)

| 상황 | load 평균 | current 평균 |
|---|---|---|
| 빈손 완전히 닫음 | (측정값 기입) | (측정값 기입) |
| 약통 잡고 닫음 | (측정값 기입) | (측정값 기입) |
| 빈손 살짝만 닫음 | (측정값 기입) | (측정값 기입) |

결론: empty_close_load = (빈손 완전히 닫음의 평균), margin = (약통 잡았을 때와의 차이의
안전 마진, 예: 차이의 절반)
```

- [ ] **Step 4: 커밋**

```bash
cd /home/roboseasy/lekiwi-pill-pickup
git add scripts/measure_gripper_load.py data/gripper_load_measurements.md
git commit -m "Add: 그리퍼 부하 실측 스크립트 및 결과"
```

이 결과값(`empty_close_load`, `margin`)은 Task 8에서 그대로 사용한다.

---

## Task 4: [하드웨어 검증] 카메라 초점거리 캘리브레이션

**목적**: `distance_estimation.py`(Task 6)가 필요로 하는 `focal_length_px` 상수를
실측으로 구한다. 핀홀 카메라 모델: `focal_length_px = (pixel_height * 실측_거리_cm) / 실제_물체_높이_cm`

**이 태스크도 실기기 필요, 사람이 직접 실행.**

**Files:**
- Create: `/home/roboseasy/lekiwi-pill-pickup/scripts/calibrate_camera.py`
- Create: `/home/roboseasy/lekiwi-pill-pickup/data/camera_calibration.md`

- [ ] **Step 1: 캘리브레이션 스크립트 작성** (손목 카메라로 사진 찍고 픽셀 높이를 수동
  측정할 수 있게 이미지를 저장만 하는 단순 스크립트)

```python
#!/usr/bin/env python
"""LeKiwiClient로 손목 카메라 프레임 1장을 받아 파일로 저장한다. 캘리브레이션은
알려진 거리(예: 20cm)에 물체(약통)를 두고 촬영 → 저장된 이미지에서 물체의 픽셀
높이를 이미지 뷰어로 직접 측정 → focal_length_px 계산은 사람이 수동으로 한다."""
import sys

import cv2

from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig


def main(lekiwi_host: str, out_path: str) -> None:
    config = LeKiwiClientConfig(remote_ip=lekiwi_host)
    client = LeKiwiClient(config)
    client.connect()
    obs = client.get_observation()
    frame = obs["wrist"]  # config_lekiwi.py의 카메라 키 이름 — 실제 키와 다르면 수정
    cv2.imwrite(out_path, frame)
    print(f"저장됨: {out_path}")
    client.disconnect()


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print(f"사용법: {sys.argv[0]} <lekiwi_host_ip> <저장경로.jpg>")
        sys.exit(1)
    main(sys.argv[1], sys.argv[2])
```

- [ ] **Step 2: 노트북에서 실행 — 약통을 정확히 20cm(또는 임의의 알려진 거리) 앞에
  놓고 촬영**

```bash
cd /home/roboseasy/lekiwi-pill-pickup && source venv/bin/activate
python scripts/calibrate_camera.py 192.168.0.201 data/calib_20cm.jpg
```

- [ ] **Step 3: 저장된 이미지에서 약통의 픽셀 높이 측정** (이미지 뷰어의 좌표 표시
  기능 또는 아래 보조 스크립트 사용)

```bash
python -c "
import cv2
img = cv2.imread('data/calib_20cm.jpg')
cv2.imshow('클릭해서 위/아래 끝 좌표 확인 (q로 종료)', img)
cv2.waitKey(0)
"
# 또는 그냥 이미지 뷰어로 열어서 좌표 확인
```

- [ ] **Step 4: focal_length_px 계산 및 기록**

```markdown
# 카메라 캘리브레이션 결과 (2026-08-XX)

- 촬영 거리: 20.0 cm
- 약통 실제 높이: (실측, cm)
- 약통 픽셀 높이(이미지 상): (측정값)
- focal_length_px = (픽셀높이 * 20.0) / 실제높이 = (계산값)
```

- [ ] **Step 5: 커밋**

```bash
git add scripts/calibrate_camera.py data/camera_calibration.md
git commit -m "Add: 카메라 캘리브레이션 스크립트 및 결과"
```

(`data/calib_20cm.jpg` 자체는 `.gitignore`의 `data/` 규칙에 걸려 커밋 안 됨 — 계산에
쓴 숫자만 마크다운에 남기면 충분)

---

## Task 5: [인프라] 노트북 SSH 서버 활성화 + laptop_ssh_host 연결 확인

**목적**: 지금 이 노트북(`192.168.0.108`)에서 `dialogue_node`/`lekiwi_command_node`가
이미 실행 중임을 확인했다(2026-08-12) — 즉 `laptop_ssh_host`는 **이 노트북이 자기
자신에게 SSH로 접속**하는 형태가 된다(원래 설계는 별도 라즈베리파이가 노트북에 접속하는
걸 상정했지만, 지금 배포 형태에선 self-SSH가 됨). SSH 서버가 현재 `inactive/disabled`
상태임을 확인했다(2026-08-12) — 활성화 필요.

- [ ] **Step 1: SSH 서버 설치/활성화**

```bash
sudo apt install -y openssh-server   # 이미 설치돼있으면 스킵됨
sudo systemctl enable --now ssh
systemctl is-active ssh   # 'active' 나와야 함
```

- [ ] **Step 2: 자기 자신에게 키 기반 SSH 접속 가능하도록 설정**

```bash
# 이미 존재하는 키(id_ed25519, 2026-07-23 생성)를 재사용
cat ~/.ssh/id_ed25519.pub >> ~/.ssh/authorized_keys
chmod 600 ~/.ssh/authorized_keys
ssh -o BatchMode=yes -i ~/.ssh/id_ed25519 roboseasy@localhost "echo SSH_SELF_OK"
```

Expected: `SSH_SELF_OK` 출력, 비밀번호 프롬프트 없이 바로 성공

- [ ] **Step 3: `lekiwi_command_node`의 `laptop_ssh_host` 파라미터를 반영해 재기동**

```bash
# 기존 실행 중인 lekiwi_command_node 프로세스 종료 후
ros2 run ros_dialogue lekiwi_command --ros-args \
  -p lekiwi_pi_host:=roboseasy@192.168.0.201 \
  -p laptop_ssh_host:=roboseasy@localhost \
  -p ssh_key_path:=/home/roboseasy/.ssh/id_ed25519
```

- [ ] **Step 4: 결과를 설계 문서 "열린 질문"에 반영**

```bash
cd /home/roboseasy/lekiwi-pill-pickup
# docs/superpowers/specs/2026-08-12-lekiwi-pill-pickup-design.md의
# "노트북 SSH 서버 활성화" 항목 체크, laptop_ssh_host 실제 값(roboseasy@localhost) 기록
git add docs/superpowers/specs/2026-08-12-lekiwi-pill-pickup-design.md
git commit -m "Docs: SSH 인프라 확인 결과 및 laptop_ssh_host 값 반영"
```

---

## Task 6: `distance_estimation.py` (TDD)

**Files:**
- Create: `/home/roboseasy/lekiwi-pill-pickup/src/lekiwi_pill_pickup/distance_estimation.py`
- Test: `/home/roboseasy/lekiwi-pill-pickup/tests/test_distance_estimation.py`

**Interfaces:**
- Produces: `estimate_distance_cm(pixel_height, real_height_cm, focal_length_px) -> float`
  — Task 10(`pick_state_machine`)과 Task 12(통합 스크립트)가 사용

- [ ] **Step 1: 실패하는 테스트 작성**

```python
import pytest

from lekiwi_pill_pickup.distance_estimation import estimate_distance_cm


def test_estimate_distance_known_values():
    assert estimate_distance_cm(
        pixel_height=100, real_height_cm=10, focal_length_px=800) == pytest.approx(80.0)


def test_estimate_distance_closer_object_has_larger_pixel_height():
    far = estimate_distance_cm(pixel_height=50, real_height_cm=10, focal_length_px=800)
    near = estimate_distance_cm(pixel_height=200, real_height_cm=10, focal_length_px=800)
    assert near < far


def test_estimate_distance_raises_on_nonpositive_pixel_height():
    with pytest.raises(ValueError):
        estimate_distance_cm(pixel_height=0, real_height_cm=10, focal_length_px=800)
    with pytest.raises(ValueError):
        estimate_distance_cm(pixel_height=-5, real_height_cm=10, focal_length_px=800)
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `pytest tests/test_distance_estimation.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'lekiwi_pill_pickup.distance_estimation'`

- [ ] **Step 3: 구현**

```python
"""단안 카메라 기준, 알려진 물체 실제 크기로 거리를 역산하는 순수 함수.

핀홀 카메라 모델: distance = (real_size * focal_length_px) / pixel_size.
focal_length_px는 Task 4(카메라 캘리브레이션)의 실측값을 호출부에서 넘겨준다 —
이 모듈은 캘리브레이션 자체를 하지 않는다(하드웨어 의존성 없는 순수 계산만)."""


def estimate_distance_cm(pixel_height: float, real_height_cm: float,
                          focal_length_px: float) -> float:
    if pixel_height <= 0:
        raise ValueError(f'pixel_height는 양수여야 함: {pixel_height}')
    return (real_height_cm * focal_length_px) / pixel_height
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `pytest tests/test_distance_estimation.py -v`
Expected: PASS (3 passed)

- [ ] **Step 5: 커밋**

```bash
git add src/lekiwi_pill_pickup/distance_estimation.py tests/test_distance_estimation.py
git commit -m "Add: 단안 카메라 거리 추정 모듈"
```

---

## Task 7: `servo_control.py` — 정렬 오차 계산 (TDD)

**Files:**
- Create: `/home/roboseasy/lekiwi-pill-pickup/src/lekiwi_pill_pickup/servo_control.py`
- Test: `/home/roboseasy/lekiwi-pill-pickup/tests/test_servo_control.py`

**Interfaces:**
- Consumes: 없음(순수 계산)
- Produces: `AlignmentError`(dataclass), `compute_alignment_error(...) -> AlignmentError`,
  `is_aligned(error, center_tolerance_px, distance_tolerance_cm) -> bool` — Task 10/12가 사용

- [ ] **Step 1: 실패하는 테스트 작성**

```python
from lekiwi_pill_pickup.servo_control import AlignmentError, compute_alignment_error, is_aligned


def test_compute_alignment_error_centered_and_at_target_distance():
    err = compute_alignment_error(
        bbox_center_x=320, bbox_center_y=240, frame_width=640, frame_height=480,
        current_distance_cm=15.0, target_distance_cm=15.0)
    assert err.x_offset_px == 0
    assert err.y_offset_px == 0
    assert err.distance_error_cm == 0


def test_compute_alignment_error_offset_right_and_down():
    err = compute_alignment_error(
        bbox_center_x=420, bbox_center_y=340, frame_width=640, frame_height=480,
        current_distance_cm=15.0, target_distance_cm=15.0)
    assert err.x_offset_px == 100
    assert err.y_offset_px == 100


def test_compute_alignment_error_too_far_gives_positive_distance_error():
    err = compute_alignment_error(
        bbox_center_x=320, bbox_center_y=240, frame_width=640, frame_height=480,
        current_distance_cm=25.0, target_distance_cm=15.0)
    assert err.distance_error_cm == 10.0


def test_is_aligned_true_within_tolerance():
    err = AlignmentError(x_offset_px=5, y_offset_px=-3, distance_error_cm=0.5)
    assert is_aligned(err, center_tolerance_px=10, distance_tolerance_cm=1.0) is True


def test_is_aligned_false_when_x_offset_exceeds_tolerance():
    err = AlignmentError(x_offset_px=50, y_offset_px=0, distance_error_cm=0.0)
    assert is_aligned(err, center_tolerance_px=10, distance_tolerance_cm=1.0) is False


def test_is_aligned_false_when_distance_error_exceeds_tolerance():
    err = AlignmentError(x_offset_px=0, y_offset_px=0, distance_error_cm=5.0)
    assert is_aligned(err, center_tolerance_px=10, distance_tolerance_cm=1.0) is False
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `pytest tests/test_servo_control.py -v`
Expected: FAIL — `ModuleNotFoundError`

- [ ] **Step 3: 구현**

```python
"""bbox 관측값으로부터 정렬 여부와 오차를 계산하는 순수 함수.

⚠️ 실제 관절 각도로 변환하는 게인(픽셀 오차 1당 몇 도 움직일지)은 이 모듈이 아니라
pick_pill_bottle.py(Task 12)에서 실기기 튜닝으로 결정한다 — 카메라-관절 기구학적
관계는 실측 없이 계산할 수 없어 여기 넣지 않았다."""
from dataclasses import dataclass


@dataclass
class AlignmentError:
    x_offset_px: float       # 양수=화면 중앙보다 오른쪽
    y_offset_px: float       # 양수=화면 중앙보다 아래
    distance_error_cm: float  # 양수=목표거리보다 더 멀리 있음(더 다가가야 함)


def compute_alignment_error(
    bbox_center_x: float, bbox_center_y: float,
    frame_width: int, frame_height: int,
    current_distance_cm: float, target_distance_cm: float,
) -> AlignmentError:
    return AlignmentError(
        x_offset_px=bbox_center_x - frame_width / 2,
        y_offset_px=bbox_center_y - frame_height / 2,
        distance_error_cm=current_distance_cm - target_distance_cm,
    )


def is_aligned(error: AlignmentError, center_tolerance_px: float,
               distance_tolerance_cm: float) -> bool:
    return (abs(error.x_offset_px) <= center_tolerance_px
            and abs(error.y_offset_px) <= center_tolerance_px
            and abs(error.distance_error_cm) <= distance_tolerance_cm)
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `pytest tests/test_servo_control.py -v`
Expected: PASS (6 passed)

- [ ] **Step 5: 커밋**

```bash
git add src/lekiwi_pill_pickup/servo_control.py tests/test_servo_control.py
git commit -m "Add: 정렬 오차 계산 모듈"
```

---

## Task 8: `grip_sensing.py` — 그립 성공 판정 (TDD)

**Files:**
- Create: `/home/roboseasy/lekiwi-pill-pickup/src/lekiwi_pill_pickup/grip_sensing.py`
- Test: `/home/roboseasy/lekiwi-pill-pickup/tests/test_grip_sensing.py`

**Interfaces:**
- Produces: `grasped_something(load_reading, empty_close_load, margin) -> bool` —
  Task 10/12가 사용. 실제 `empty_close_load`/`margin` 값은 Task 3 실측 결과를 사용

- [ ] **Step 1: 실패하는 테스트 작성** (Task 3에서 아직 실측 전이므로 임의값으로 테스트,
  실측 완료 후 통합 스크립트에서 실제값으로 교체)

```python
from lekiwi_pill_pickup.grip_sensing import grasped_something


def test_grasped_something_true_when_load_exceeds_threshold():
    assert grasped_something(load_reading=500, empty_close_load=100, margin=50) is True


def test_grasped_something_false_when_load_at_empty_baseline():
    assert grasped_something(load_reading=100, empty_close_load=100, margin=50) is False


def test_grasped_something_false_within_margin_of_baseline():
    assert grasped_something(load_reading=120, empty_close_load=100, margin=50) is False


def test_grasped_something_true_just_above_margin():
    assert grasped_something(load_reading=151, empty_close_load=100, margin=50) is True
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `pytest tests/test_grip_sensing.py -v`
Expected: FAIL — `ModuleNotFoundError`

- [ ] **Step 3: 구현**

```python
"""그리퍼 서보의 Present_Load 값으로 "확실히 뭔가 잡았는지" 판정하는 순수 함수.

실제 레지스터 판독(하드웨어 I/O)은 pick_pill_bottle.py(Task 12)가 lerobot의
FeetechMotorsBus로 수행하고, 이 함수는 판독된 숫자만 받아 판정한다.
empty_close_load/margin 값은 Task 3(실기기 측정)의 결과를 사용할 것 — 이 모듈
자체엔 기본값을 두지 않는다(잘못된 기본값으로 조용히 오판정하는 것을 방지)."""


def grasped_something(load_reading: int, empty_close_load: int, margin: int) -> bool:
    """load_reading이 '빈손으로 닫았을 때' 기준(empty_close_load)보다 margin 이상
    크면 뭔가에 걸려 완전히 안 닫힌 것으로 보고 성공으로 추정한다."""
    return load_reading > empty_close_load + margin
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `pytest tests/test_grip_sensing.py -v`
Expected: PASS (4 passed)

- [ ] **Step 5: 커밋**

```bash
git add src/lekiwi_pill_pickup/grip_sensing.py tests/test_grip_sensing.py
git commit -m "Add: 그립 성공 판정 모듈"
```

---

## Task 9: `detector.py` — YOLO 결과 파싱 (TDD)

**Files:**
- Create: `/home/roboseasy/lekiwi-pill-pickup/src/lekiwi_pill_pickup/detector.py`
- Test: `/home/roboseasy/lekiwi-pill-pickup/tests/test_detector.py`

**Interfaces:**
- Produces: `Detection`(dataclass, `.center_x`/`.center_y`/`.height_px` 프로퍼티),
  `parse_detections(boxes_xyxy, confidences, class_names, class_ids) -> list[Detection]`,
  `best_detection(detections, target_label, min_confidence) -> Detection | None`,
  `load_model_and_detect(model_path, frame, target_label, min_confidence) -> Detection | None`
  (하드웨어/모델 의존 부분, 아래 참고) — Task 10/12가 사용

- [ ] **Step 1: 실패하는 테스트 작성** (순수 파싱 로직만 — 실제 YOLO 모델 없이 테스트)

```python
from lekiwi_pill_pickup.detector import Detection, best_detection, parse_detections


def test_parse_detections_builds_detection_list():
    dets = parse_detections(
        boxes_xyxy=[(10, 20, 30, 60)], confidences=[0.9],
        class_names=['bottle'], class_ids=[0])
    assert len(dets) == 1
    assert dets[0].label == 'bottle'
    assert dets[0].confidence == 0.9


def test_detection_center_and_height_properties():
    d = Detection(label='bottle', confidence=0.9, x1=10, y1=20, x2=30, y2=60)
    assert d.center_x == 20
    assert d.center_y == 40
    assert d.height_px == 40


def test_best_detection_filters_by_label_and_confidence():
    dets = [
        Detection(label='bottle', confidence=0.9, x1=0, y1=0, x2=10, y2=10),
        Detection(label='cup', confidence=0.95, x1=0, y1=0, x2=10, y2=10),
        Detection(label='bottle', confidence=0.3, x1=0, y1=0, x2=10, y2=10),
    ]
    best = best_detection(dets, target_label='bottle', min_confidence=0.5)
    assert best is not None
    assert best.confidence == 0.9


def test_best_detection_returns_none_when_no_match():
    dets = [Detection(label='cup', confidence=0.9, x1=0, y1=0, x2=10, y2=10)]
    assert best_detection(dets, target_label='bottle', min_confidence=0.5) is None


def test_best_detection_picks_highest_confidence_among_multiple():
    dets = [
        Detection(label='bottle', confidence=0.6, x1=0, y1=0, x2=10, y2=10),
        Detection(label='bottle', confidence=0.85, x1=0, y1=0, x2=10, y2=10),
    ]
    best = best_detection(dets, target_label='bottle', min_confidence=0.5)
    assert best.confidence == 0.85
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `pytest tests/test_detector.py -v`
Expected: FAIL — `ModuleNotFoundError`

- [ ] **Step 3: 구현**

```python
"""Ultralytics YOLO 추론 래퍼.

원시 ultralytics Results 파싱(순수 로직, `parse_detections`/`best_detection`/
`Detection`)과 실제 모델 로딩+추론(하드웨어 의존, `load_model_and_detect`)을 분리한다
— 전자는 실제 모델 파일 없이 테스트 가능, 후자는 Task 11(YOLO 1차 시험)/Task 12
(통합 스크립트)에서 실기기로 검증한다."""
from dataclasses import dataclass


@dataclass
class Detection:
    label: str
    confidence: float
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def center_x(self) -> float:
        return (self.x1 + self.x2) / 2

    @property
    def center_y(self) -> float:
        return (self.y1 + self.y2) / 2

    @property
    def height_px(self) -> float:
        return self.y2 - self.y1


def parse_detections(boxes_xyxy: list[tuple[float, float, float, float]],
                      confidences: list[float], class_names: list[str],
                      class_ids: list[int]) -> list['Detection']:
    detections = []
    for (x1, y1, x2, y2), conf, cls_id in zip(boxes_xyxy, confidences, class_ids):
        detections.append(Detection(
            label=class_names[cls_id], confidence=conf, x1=x1, y1=y1, x2=x2, y2=y2))
    return detections


def best_detection(detections: list['Detection'], target_label: str,
                    min_confidence: float) -> 'Detection | None':
    candidates = [d for d in detections
                  if d.label == target_label and d.confidence >= min_confidence]
    if not candidates:
        return None
    return max(candidates, key=lambda d: d.confidence)


def load_model_and_detect(model_path: str, frame, target_label: str,
                           min_confidence: float) -> 'Detection | None':
    """실제 YOLO 모델을 로드해 frame(numpy BGR 이미지)에 추론 후 최적 검출 하나를
    반환한다. ultralytics 의존 — 이 함수 자체는 단위테스트하지 않고(모델 파일+무거운
    의존성 필요), Task 11/12의 실기기 검증으로 대신한다."""
    from ultralytics import YOLO
    model = load_model_and_detect._cached_model
    if model is None or load_model_and_detect._cached_path != model_path:
        model = YOLO(model_path)
        load_model_and_detect._cached_model = model
        load_model_and_detect._cached_path = model_path
    results = model(frame, verbose=False)[0]
    boxes_xyxy = results.boxes.xyxy.tolist()
    confidences = results.boxes.conf.tolist()
    class_ids = [int(c) for c in results.boxes.cls.tolist()]
    class_names = [results.names[i] for i in range(len(results.names))]
    detections = parse_detections(boxes_xyxy, confidences, class_names, class_ids)
    return best_detection(detections, target_label, min_confidence)


load_model_and_detect._cached_model = None
load_model_and_detect._cached_path = None
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `pytest tests/test_detector.py -v`
Expected: PASS (5 passed) — `load_model_and_detect`는 테스트 대상이 아니므로 ultralytics
미설치 상태에서도 이 5개 테스트는 통과해야 함(함수 내부에서만 import하므로)

- [ ] **Step 5: 커밋**

```bash
git add src/lekiwi_pill_pickup/detector.py tests/test_detector.py
git commit -m "Add: YOLO 검출 결과 파싱 모듈"
```

---

## Task 10: `pick_state_machine.py` — 픽업 상태머신 (TDD)

**Files:**
- Create: `/home/roboseasy/lekiwi-pill-pickup/src/lekiwi_pill_pickup/pick_state_machine.py`
- Test: `/home/roboseasy/lekiwi-pill-pickup/tests/test_pick_state_machine.py`

**Interfaces:**
- Consumes: 없음(순수 로직, 다른 모듈에 의존하지 않음 — 호출부가 `Observation`을
  채울 때 Task 6~9의 함수 결과를 조합해서 넣는다)
- Produces: `PickState`(Enum), `ActionType`(Enum), `Action`(dataclass), `Observation`
  (dataclass), `tick(state, obs, elapsed_in_state_sec, search_timeout_sec) -> tuple[PickState, Action]`
  — Task 12(통합 스크립트)가 메인 루프에서 사용

이 모듈은 `session_manager.py`(robot_ws 저장소, 세션 상태머신)와 동일한 설계
철학 — 하드웨어/ROS 의존성 없는 순수 상태머신, 실행은 호출부가 담당.

- [ ] **Step 1: 실패하는 테스트 작성**

```python
from lekiwi_pill_pickup.pick_state_machine import (
    Action, ActionType, Observation, PickState, tick,
)


def test_searching_transitions_to_approaching_when_detected():
    state, action = tick(
        PickState.SEARCHING, Observation(detected=True),
        elapsed_in_state_sec=1.0, search_timeout_sec=8.0)
    assert state == PickState.APPROACHING
    assert action.type == ActionType.NONE


def test_searching_sweeps_when_not_detected_and_not_timed_out():
    state, action = tick(
        PickState.SEARCHING, Observation(detected=False),
        elapsed_in_state_sec=1.0, search_timeout_sec=8.0)
    assert state == PickState.SEARCHING
    assert action.type == ActionType.SWEEP


def test_searching_fails_on_timeout():
    state, action = tick(
        PickState.SEARCHING, Observation(detected=False),
        elapsed_in_state_sec=8.0, search_timeout_sec=8.0)
    assert state == PickState.FAILED


def test_approaching_nudges_when_not_aligned():
    state, action = tick(
        PickState.APPROACHING, Observation(detected=True, aligned=False),
        elapsed_in_state_sec=0.5, search_timeout_sec=8.0)
    assert state == PickState.APPROACHING
    assert action.type == ActionType.NUDGE


def test_approaching_transitions_to_grasping_when_aligned():
    state, action = tick(
        PickState.APPROACHING, Observation(detected=True, aligned=True),
        elapsed_in_state_sec=0.5, search_timeout_sec=8.0)
    assert state == PickState.GRASPING
    assert action.type == ActionType.DESCEND_AND_GRIP


def test_approaching_returns_to_searching_if_target_lost():
    state, action = tick(
        PickState.APPROACHING, Observation(detected=False),
        elapsed_in_state_sec=0.5, search_timeout_sec=8.0)
    assert state == PickState.SEARCHING


def test_grasping_succeeds_when_grasped():
    state, action = tick(
        PickState.GRASPING, Observation(detected=True, grasped=True),
        elapsed_in_state_sec=0.1, search_timeout_sec=8.0)
    assert state == PickState.LIFTING
    assert action.type == ActionType.LIFT


def test_grasping_fails_when_not_grasped():
    state, action = tick(
        PickState.GRASPING, Observation(detected=True, grasped=False),
        elapsed_in_state_sec=0.1, search_timeout_sec=8.0)
    assert state == PickState.FAILED


def test_lifting_transitions_to_succeeded():
    state, action = tick(
        PickState.LIFTING, Observation(detected=True),
        elapsed_in_state_sec=0.1, search_timeout_sec=8.0)
    assert state == PickState.SUCCEEDED


def test_terminal_states_stay_terminal_and_take_no_action():
    for terminal in (PickState.SUCCEEDED, PickState.FAILED):
        state, action = tick(
            terminal, Observation(detected=True),
            elapsed_in_state_sec=0.1, search_timeout_sec=8.0)
        assert state == terminal
        assert action.type == ActionType.NONE
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `pytest tests/test_pick_state_machine.py -v`
Expected: FAIL — `ModuleNotFoundError`

- [ ] **Step 3: 구현**

```python
"""약통 픽업 상태머신 — 순수 로직. 관측값(비전 감지결과/정렬여부/그립성공여부)을
입력받아 다음 상태와 이번 틱에 수행할 행동(Action)을 결정만 한다. 실제 하드웨어
I/O(카메라 프레임 읽기, 팔 관절 명령 전송)는 pick_pill_bottle.py(Task 12)가 담당하고
이 모듈은 아무것도 실행하지 않는다 — robot_ws의 session_manager.py와 동일 패턴.

상태 전이 시 그 상태에서 처음 해야 할 행동을 함께 반환한다(예: APPROACHING에서
GRASPING으로 전이하는 순간 DESCEND_AND_GRIP 행동을 반환) — 호출부는 그 행동을
실행한 뒤 결과를 다음 tick() 호출의 Observation에 담아 다시 전달한다."""
from dataclasses import dataclass
from enum import Enum, auto


class PickState(Enum):
    SEARCHING = auto()
    APPROACHING = auto()
    GRASPING = auto()
    LIFTING = auto()
    SUCCEEDED = auto()
    FAILED = auto()


class ActionType(Enum):
    SWEEP = auto()             # 탐색: 팔을 좌우로 소폭 스윕
    NUDGE = auto()              # 접근/정렬: 오차 방향으로 팔 미세 조정
    DESCEND_AND_GRIP = auto()   # 하강+그리퍼 닫기 고정 시퀀스 실행
    LIFT = auto()                # 들어올리기+retract
    NONE = auto()                # 이번 틱은 할 일 없음


@dataclass
class Action:
    type: ActionType


@dataclass
class Observation:
    detected: bool
    aligned: bool = False
    grasped: bool = False


def tick(state: PickState, obs: Observation, elapsed_in_state_sec: float,
         search_timeout_sec: float) -> tuple[PickState, Action]:
    if state == PickState.SEARCHING:
        if obs.detected:
            return PickState.APPROACHING, Action(ActionType.NONE)
        if elapsed_in_state_sec >= search_timeout_sec:
            return PickState.FAILED, Action(ActionType.NONE)
        return PickState.SEARCHING, Action(ActionType.SWEEP)

    if state == PickState.APPROACHING:
        if not obs.detected:
            return PickState.SEARCHING, Action(ActionType.NONE)
        if obs.aligned:
            return PickState.GRASPING, Action(ActionType.DESCEND_AND_GRIP)
        return PickState.APPROACHING, Action(ActionType.NUDGE)

    if state == PickState.GRASPING:
        # 이 상태로 진입한 턴에 이미 DESCEND_AND_GRIP을 호출부가 실행했다는 전제 —
        # 그 결과(그리퍼 부하값 기반 판정)를 obs.grasped에 담아 다시 tick()을 호출한다
        if obs.grasped:
            return PickState.LIFTING, Action(ActionType.LIFT)
        return PickState.FAILED, Action(ActionType.NONE)

    if state == PickState.LIFTING:
        return PickState.SUCCEEDED, Action(ActionType.NONE)

    # SUCCEEDED/FAILED는 종단 상태
    return state, Action(ActionType.NONE)
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `pytest tests/test_pick_state_machine.py -v`
Expected: PASS (10 passed)

- [ ] **Step 5: 커밋**

```bash
git add src/lekiwi_pill_pickup/pick_state_machine.py tests/test_pick_state_machine.py
git commit -m "Add: 약통 픽업 상태머신"
```

- [ ] **Step 6: 전체 테스트 스위트 통과 확인** (Task 6~10 전체)

Run: `cd /home/roboseasy/lekiwi-pill-pickup && source venv/bin/activate && pytest -v`
Expected: 모든 테스트 PASS (25개 이상)

---

## Task 11: [하드웨어 검증] YOLO 사전학습 "bottle" 클래스 1차 시험

**목적**: 설계 문서의 인식 계획 1단계 — 커스텀 학습 없이 COCO 사전학습 모델로 우리
약통이 인식되는지 먼저 확인한다. 성공하면 Task 12(파인튜닝 데이터 수집)를 건너뛸 수
있다.

**Files:**
- Create: `/home/roboseasy/lekiwi-pill-pickup/scripts/test_pretrained_bottle.py`
- Create: `/home/roboseasy/lekiwi-pill-pickup/data/pretrained_test_results.md`

- [ ] **Step 1: 시험 스크립트 작성**

```python
#!/usr/bin/env python
"""사전학습 YOLO(yolov8n.pt, COCO)로 저장된 사진들에서 'bottle' 클래스가 얼마나
정확히/자신있게 검출되는지 확인한다. Task 4에서 찍은 캘리브레이션 사진 등 기존
사진을 재사용해도 되고, 여러 거리/각도로 새로 찍어도 된다."""
import sys
from pathlib import Path

from ultralytics import YOLO


def main(image_dir: str) -> None:
    model = YOLO('yolov8n.pt')  # 최초 실행 시 자동 다운로드
    for img_path in sorted(Path(image_dir).glob('*.jpg')):
        results = model(str(img_path), verbose=False)[0]
        bottle_dets = [
            (results.names[int(c)], conf)
            for c, conf in zip(results.boxes.cls, results.boxes.conf)
            if results.names[int(c)] == 'bottle'
        ]
        print(f'{img_path.name}: {bottle_dets if bottle_dets else "미검출"}')


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print(f'사용법: {sys.argv[0]} <이미지_디렉터리>')
        sys.exit(1)
    main(sys.argv[1])
```

- [ ] **Step 2: 다양한 거리/각도로 약통 사진 10~15장 촬영** (Task 4의
  `calibrate_camera.py`를 여러 번 실행해 `data/pretrained_test/` 아래 저장, 또는
  휴대폰으로 찍어 옮겨도 무방)

```bash
mkdir -p data/pretrained_test
# calibrate_camera.py를 파일명 바꿔가며 여러 번 실행하거나, 휴대폰 사진을 옮겨서 채움
```

- [ ] **Step 3: 실행 및 결과 기록**

```bash
cd /home/roboseasy/lekiwi-pill-pickup && source venv/bin/activate
python scripts/test_pretrained_bottle.py data/pretrained_test
```

```markdown
# 사전학습 YOLO "bottle" 클래스 시험 결과 (2026-08-XX)

- 총 N장 중 M장에서 검출됨
- 평균 confidence: (기입)
- 결론: [사전학습 모델로 충분함 / 파인튜닝 필요]
```

- [ ] **Step 4: 커밋**

```bash
git add scripts/test_pretrained_bottle.py data/pretrained_test_results.md
git commit -m "Add: YOLO 사전학습 bottle 클래스 1차 시험 결과"
```

⚠️ **결론이 "파인튜닝 필요"면 Task 12로 진행. "충분함"이면 Task 12를 건너뛰고
Task 13에서 `model_path='yolov8n.pt'`, `target_label='bottle'`을 그대로 사용한다.**

---

## Task 12: [조건부, Task 11 결과가 "파인튜닝 필요"일 때만] YOLO 커스텀 파인튜닝

**이 태스크는 사람이 직접 촬영·라벨링을 수행해야 하는 대량의 수작업을 포함한다.**

**Files:**
- Create: `/home/roboseasy/lekiwi-pill-pickup/scripts/capture_training_images.py`
- Create: `/home/roboseasy/lekiwi-pill-pickup/scripts/train_yolo.py`

- [ ] **Step 1: 촬영 스크립트 작성** (스페이스바로 캡처하는 간단한 루프)

```python
#!/usr/bin/env python
"""LeKiwiClient로 손목 카메라 스트림을 띄우고, 스페이스바를 누를 때마다 프레임을
저장한다. 약통을 고정된 자세로 유지한 채 거리/각도를 바꿔가며 150~300장 촬영할 것."""
import sys
import time
from pathlib import Path

import cv2

from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig


def main(lekiwi_host: str, out_dir: str) -> None:
    out_path = Path(out_dir)
    out_path.mkdir(parents=True, exist_ok=True)
    config = LeKiwiClientConfig(remote_ip=lekiwi_host)
    client = LeKiwiClient(config)
    client.connect()
    count = 0
    print('스페이스바: 캡처, q: 종료')
    while True:
        obs = client.get_observation()
        frame = obs['wrist']
        cv2.imshow('wrist camera (space=capture, q=quit)', frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '):
            fname = out_path / f'{int(time.time() * 1000)}.jpg'
            cv2.imwrite(str(fname), frame)
            count += 1
            print(f'저장됨 ({count}장): {fname.name}')
        elif key == ord('q'):
            break
    client.disconnect()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    if len(sys.argv) != 3:
        print(f'사용법: {sys.argv[0]} <lekiwi_host_ip> <저장디렉터리>')
        sys.exit(1)
    main(sys.argv[1], sys.argv[2])
```

- [ ] **Step 2: 150~300장 촬영** (거리/각도 다양하게, 자세는 고정 규칙 유지)

```bash
cd /home/roboseasy/lekiwi-pill-pickup && source venv/bin/activate
python scripts/capture_training_images.py 192.168.0.201 data/training_images
```

- [ ] **Step 3: 라벨링** (Roboflow 또는 LabelImg 등 외부 도구 사용 — 단일 클래스라
  빠름. Roboflow 사용 시 프로젝트를 만들고 이미지 업로드 후 바운딩박스 그리기,
  `YOLOv8` 포맷으로 export)

```bash
# Roboflow에서 export한 zip을 data/dataset/ 아래 압축 해제
# (train/images, train/labels, valid/images, valid/labels, data.yaml 구조)
unzip ~/Downloads/roboflow_export.zip -d data/dataset
```

- [ ] **Step 4: 파인튜닝 스크립트 작성**

```python
#!/usr/bin/env python
"""COCO 사전학습 YOLOv8n을 우리 약통 단일 클래스 데이터셋으로 파인튜닝."""
import sys

from ultralytics import YOLO


def main(data_yaml: str, epochs: int) -> None:
    model = YOLO('yolov8n.pt')
    model.train(data=data_yaml, epochs=epochs, imgsz=640, project='runs', name='pill_bottle')


if __name__ == '__main__':
    data_yaml = sys.argv[1] if len(sys.argv) > 1 else 'data/dataset/data.yaml'
    epochs = int(sys.argv[2]) if len(sys.argv) > 2 else 60
    main(data_yaml, epochs)
```

- [ ] **Step 5: 학습 실행**

```bash
python scripts/train_yolo.py data/dataset/data.yaml 60
```

Expected: `runs/pill_bottle/weights/best.pt` 생성

- [ ] **Step 6: Task 11의 시험 스크립트를 재사용해 파인튜닝 모델로 재검증**

```bash
# test_pretrained_bottle.py의 model = YOLO('yolov8n.pt')를
# model = YOLO('runs/pill_bottle/weights/best.pt')로, target label을 실제 클래스명으로
# 바꿔서 재실행(혹은 인자로 받게 소폭 수정)
```

- [ ] **Step 7: 커밋**

```bash
git add scripts/capture_training_images.py scripts/train_yolo.py
git commit -m "Add: YOLO 커스텀 파인튜닝 스크립트"
# best.pt는 .gitignore(*.pt)에 걸려 커밋 안 됨 — Task 13에서 실제 배포 경로에 직접 배치
```

---

## Task 13: `pick_pill_bottle.py` — 메인 통합 스크립트

**이 태스크는 Task 6~10(순수 로직)을 실제 하드웨어와 연결한다. 자동 테스트로 완전히
검증할 수 없고(실제 LeKiwi 필요), 사람이 실기기로 반복 실행하며 게인 값을 튜닝해야
한다 — 아래 Step들은 코드 작성까지고, Task 15에서 실기기 튜닝을 진행한다.**

**Files:**
- Create: `/home/roboseasy/lekiwi-pill-pickup/src/lekiwi_pill_pickup/pick_pill_bottle.py`

**Interfaces:**
- Consumes: `distance_estimation.estimate_distance_cm`, `servo_control.compute_alignment_error`,
  `servo_control.is_aligned`, `grip_sensing.grasped_something`, `detector.load_model_and_detect`,
  `pick_state_machine.tick`/`PickState`/`ActionType`/`Observation`
- Produces: CLI 진입점(`__main__`), 종료 시 stdout에 `SUCCESS` 또는 `FAILED: <이유>` 한 줄
  출력(robot_ws의 `lekiwi_command_node.py`가 SSH stdout으로 결과를 받을 수 있게)

- [ ] **Step 1: 스크립트 작성**

```python
#!/usr/bin/env python
"""약통 픽업 메인 스크립트 — LeKiwiClient로 연결해 손목 카메라+YOLO로 약통을 찾아
정렬 후 집는다. robot_ws의 lekiwi_command_node.py가 SSH로 이 스크립트를 실행하고,
표준출력 마지막 줄(SUCCESS/FAILED)로 결과를 판정한다.

⚠️ 아래 상수 중 GAIN_DEG_PER_PX/SWEEP_STEP_DEG/GRIPPER_MOTOR_NAME 등은 실기기
튜닝값이 채워지기 전까지 자리표시 값이다 — Task 15(실기기 튜닝)에서 실측 후 갱신할 것.
"""
import argparse
import sys
import time

from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig
from lerobot.motors.feetech import FeetechMotorsBus

from lekiwi_pill_pickup.detector import load_model_and_detect
from lekiwi_pill_pickup.distance_estimation import estimate_distance_cm
from lekiwi_pill_pickup.grip_sensing import grasped_something
from lekiwi_pill_pickup.pick_state_machine import (
    Action, ActionType, Observation, PickState, tick,
)
from lekiwi_pill_pickup.servo_control import compute_alignment_error, is_aligned

# --- Task 3/4/11 실측 결과로 채울 상수 (현재 자리표시 값) ---
FOCAL_LENGTH_PX = 800.0          # Task 4 캘리브레이션 결과로 교체
TARGET_HEIGHT_CM = 10.0          # 약통 실제 높이(cm) — 실측해서 교체
TARGET_DISTANCE_CM = 15.0        # 그립을 시도할 목표 거리 — 실기기 튜닝
EMPTY_CLOSE_LOAD = 100           # Task 3 실측 결과로 교체
GRIP_LOAD_MARGIN = 50            # Task 3 실측 결과로 교체
MODEL_PATH = 'yolov8n.pt'        # Task 11 결과에 따라 파인튜닝 모델 경로로 교체 가능
TARGET_LABEL = 'bottle'          # Task 12 진행 시 커스텀 클래스명으로 교체
MIN_CONFIDENCE = 0.5
CENTER_TOLERANCE_PX = 15.0
DISTANCE_TOLERANCE_CM = 1.5
SEARCH_TIMEOUT_SEC = 8.0
GRIPPER_MOTOR_NAME = 'gripper'   # Task 3에서 확인한 실제 명칭으로 교체
GRIPPER_MOTOR_ID = 6             # Task 3에서 확인한 실제 ID로 교체
LOOP_INTERVAL_SEC = 0.1


def _read_gripper_load(bus: FeetechMotorsBus) -> int:
    return bus.read('Present_Load', GRIPPER_MOTOR_NAME)


def _apply_action(client: LeKiwiClient, action: Action) -> None:
    """상태머신이 요청한 Action을 실제 로봇 명령으로 변환해 전송한다.
    실제 관절 델타 값(게인)은 실기기 튜닝 전까지 자리표시 — Task 15에서 확정."""
    if action.type == ActionType.NONE:
        return
    if action.type == ActionType.SWEEP:
        pass  # TODO(Task 15): 팔을 좌우로 소폭 스윕하는 관절 명령
    elif action.type == ActionType.NUDGE:
        pass  # TODO(Task 15): 마지막 정렬 오차 방향으로 관절 미세 조정
    elif action.type == ActionType.DESCEND_AND_GRIP:
        pass  # TODO(Task 15): 고정 시퀀스(내려가기 → 그리퍼 닫기)
    elif action.type == ActionType.LIFT:
        pass  # TODO(Task 15): 들어올리기 + retract


def run(lekiwi_host: str) -> bool:
    client = LeKiwiClient(LeKiwiClientConfig(remote_ip=lekiwi_host))
    client.connect()
    gripper_bus = FeetechMotorsBus(port='/dev/ttyACM0', motors={})  # Task 15에서 실제 구성

    state = PickState.SEARCHING
    state_entered_at = time.time()
    last_alignment_error = None

    try:
        while state not in (PickState.SUCCEEDED, PickState.FAILED):
            obs_frame = client.get_observation()
            frame = obs_frame['wrist']
            detection = load_model_and_detect(MODEL_PATH, frame, TARGET_LABEL, MIN_CONFIDENCE)

            aligned = False
            grasped = False
            if detection is not None:
                distance_cm = estimate_distance_cm(
                    detection.height_px, TARGET_HEIGHT_CM, FOCAL_LENGTH_PX)
                last_alignment_error = compute_alignment_error(
                    detection.center_x, detection.center_y,
                    frame.shape[1], frame.shape[0], distance_cm, TARGET_DISTANCE_CM)
                aligned = is_aligned(
                    last_alignment_error, CENTER_TOLERANCE_PX, DISTANCE_TOLERANCE_CM)

            elapsed = time.time() - state_entered_at
            observation = Observation(detected=detection is not None, aligned=aligned)
            new_state, action = tick(state, observation, elapsed, SEARCH_TIMEOUT_SEC)

            _apply_action(client, action)

            if action.type == ActionType.DESCEND_AND_GRIP:
                load = _read_gripper_load(gripper_bus)
                grasped = grasped_something(load, EMPTY_CLOSE_LOAD, GRIP_LOAD_MARGIN)
                observation = Observation(detected=True, grasped=grasped)
                new_state, action = tick(new_state, observation, 0.0, SEARCH_TIMEOUT_SEC)
                _apply_action(client, action)

            if new_state != state:
                state_entered_at = time.time()
            state = new_state
            time.sleep(LOOP_INTERVAL_SEC)
    finally:
        client.disconnect()

    return state == PickState.SUCCEEDED


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--lekiwi-host', default='192.168.0.201')
    args = parser.parse_args()

    success = run(args.lekiwi_host)
    if success:
        print('SUCCESS')
        sys.exit(0)
    else:
        print('FAILED: 정해진 시간 안에 약통을 찾거나 집지 못함')
        sys.exit(1)


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: 구문 오류 없이 임포트되는지 확인** (실제 실행은 하드웨어 필요하므로
  여기선 정적 검사만)

Run: `cd /home/roboseasy/lekiwi-pill-pickup && source venv/bin/activate && python -c "import ast; ast.parse(open('src/lekiwi_pill_pickup/pick_pill_bottle.py').read())"`
Expected: 에러 없이 종료

- [ ] **Step 3: 커밋**

```bash
git add src/lekiwi_pill_pickup/pick_pill_bottle.py
git commit -m "Add: 약통 픽업 메인 통합 스크립트 (실기기 튜닝값은 자리표시)"
```

---

## Task 14: robot_ws — `lekiwi_tool.py`에 신규 스킬 선언 추가 + 대시보드 도구설명 등록

**이 태스크는 `robot_ws`(별도 저장소, 이 저장소가 아님) 안에서, `feature/lekiwi-pill-bottle`
브랜치(기존 `develop`은 건드리지 않음, 2026-08-14 확정)에서 진행한다.**

**Files:**
- Modify: `/home/roboseasy/robot_ws/src/ros_dialogue/ros_dialogue/lekiwi_tool.py:17,23-24`
- Modify: `/home/roboseasy/robot_ws/src/ros_dialogue/ros_dialogue/education_bridge_node.py`
  (도구 설명 import 목록 — 2026-08-14 코드 확인 결과 `lekiwi_tool`이 아예 없어서
  대시보드가 이 스킬 호출을 설명 없이 표시할 것으로 확정됨, design.md 리스크 표 참고)

**Interfaces:**
- Consumes: 없음(LLM 도구 선언, 실제 실행 안 됨 — 기존 패턴과 동일)

- [ ] **Step 1: `Literal` 목록과 docstring에 `pick_pill_bottle` 추가**

```python
@tool(RUN_LEKIWI_SKILL_TOOL_NAME)
def run_lekiwi_skill(
    skill: Literal['forward', 'backward', 'left', 'right', 'stop', 'pick_cube',
                    'pick_pill_bottle'],
) -> str:
    """LeKiwi 로봇(팔+바퀴)의 동작을 실행시킨다.

    사용자가 LeKiwi를 움직이거나 특정 동작을 시키고 싶어하는 의도를 보이면 사용한다.
    skill 값: forward(앞으로 가/전진)/backward(뒤로 가/후진)/left(좌회전/왼쪽으로 돌아)/
    right(우회전/오른쪽으로 돌아)/stop(멈춰/정지)/pick_cube(큐브 집어줘/큐브를 박스에
    넣어줘 — 카메라로 큐브를 찾아 집어서 박스에 넣는 동작)/pick_pill_bottle(약 좀
    찾아서 갖다줘/약통 집어줘 — 카메라로 책상 위 약통을 찾아 집는 동작).
    """
    raise RuntimeError('run_lekiwi_skill은 dialogue_node가 직접 처리해야 합니다')
```

- [ ] **Step 2: `education_bridge_node.py`에 `run_lekiwi_skill` 도구 설명 등록**
  (대시보드가 이 스킬 호출을 "약 좀 찾아서 갖다줘" 문구 없이 알 수 없는 도구로 표시하는
  것 방지)

```python
# import 목록(파일 상단)에 추가:
from .lekiwi_tool import run_lekiwi_skill

# TOOL_INFO 딕셔너리 리스트에 추가:
        (run_lekiwi_skill, 'LeKiwi 로봇 조종'),
```

- [ ] **Step 3: 빌드 확인**

```bash
cd ~/robot_ws
colcon build --packages-select ros_dialogue --symlink-install
source install/setup.bash
python3 -c "from ros_dialogue.lekiwi_tool import run_lekiwi_skill; print('OK')"
python3 -c "from ros_dialogue.education_bridge_node import TOOL_INFO; assert 'run_lekiwi_skill' in TOOL_INFO; print('OK')"
```

Expected: 둘 다 `OK` 출력, 빌드 에러 없음

- [ ] **Step 4: 커밋** (`robot_ws` 저장소, 이 저장소와 별개, `feature/lekiwi-pill-bottle`
  브랜치)

```bash
cd ~/robot_ws
git add src/ros_dialogue/ros_dialogue/lekiwi_tool.py src/ros_dialogue/ros_dialogue/education_bridge_node.py
git commit -m "Add: pick_pill_bottle LLM 도구 선언 추가 + 대시보드 설명 등록"
```

---

## Task 15: robot_ws — `lekiwi_control.py` SKILL_MAP에 항목 추가 + 테스트

**Task 14와 동일하게 `feature/lekiwi-pill-bottle` 브랜치에서 진행(`develop` 미변경).**

**Files:**
- Modify: `/home/roboseasy/robot_ws/src/ros_dialogue/ros_dialogue/lekiwi_control.py:61-67`
- Modify: `/home/roboseasy/robot_ws/src/ros_dialogue/test/test_lekiwi_control.py`

**Interfaces:**
- Consumes: `SKILL_MAP`(기존, 이 태스크에서 항목 추가), `build_launch_command`(기존,
  변경 없음 — 새 항목도 기존 함수로 그대로 처리됨)

- [ ] **Step 1: 실패하는 테스트 먼저 추가** (`test_lekiwi_control.py`)

```python
def test_skill_map_has_pick_pill_bottle():
    assert 'pick_pill_bottle' in SKILL_MAP


def test_pick_pill_bottle_targets_laptop_host():
    assert SKILL_MAP['pick_pill_bottle']['host_param'] == 'laptop_ssh_host'


def test_build_launch_command_pick_pill_bottle_uses_venv_python():
    cmd = build_launch_command('pick_pill_bottle')
    assert 'lekiwi-pill-pickup/venv/bin/python' in cmd
    assert 'pick_pill_bottle.py' in cmd
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `cd ~/robot_ws && python3 -m pytest src/ros_dialogue/test/test_lekiwi_control.py -v`
Expected: FAIL — `KeyError: 'pick_pill_bottle'`

- [ ] **Step 3: `SKILL_MAP`에 항목 추가**

```python
    'pick_pill_bottle': {
        'host_param': 'laptop_ssh_host',
        'command_template': (
            '/home/roboseasy/lekiwi-pill-pickup/venv/bin/python '
            '/home/roboseasy/lekiwi-pill-pickup/src/lekiwi_pill_pickup/pick_pill_bottle.py '
            '--lekiwi-host=192.168.0.201'
        ),
        'duration_sec': 45.0,
        'timeout_sec': 60.0,
    },
```

(기존 `pick_cube` 항목 바로 아래 추가. `duration_sec`은 이 스킬의 `command_template`이
`{duration}`을 안 써서 실질적으로 안 쓰이지만, `build_launch_command`가
`entry['duration_sec']`을 항상 읽으므로 키 자체는 있어야 함 — `timeout_sec`이 실제
SSH 바깥쪽 안전장치: 연결 오버헤드(~10초)+탐색(8초)+접근/정렬 여유+그립+들어올리기를
합쳐 60초로 설정)

- [ ] **Step 4: 테스트 통과 확인**

Run: `cd ~/robot_ws && python3 -m pytest src/ros_dialogue/test/test_lekiwi_control.py -v`
Expected: PASS (기존 테스트 포함 전체)

- [ ] **Step 5: 빌드 확인**

```bash
cd ~/robot_ws && colcon build --packages-select ros_dialogue --symlink-install
```

- [ ] **Step 6: 커밋**

```bash
cd ~/robot_ws
git add src/ros_dialogue/ros_dialogue/lekiwi_control.py src/ros_dialogue/test/test_lekiwi_control.py
git commit -m "Add: pick_pill_bottle SKILL_MAP 항목 및 테스트"
```

---

## Task 16: [실기기 검증] 통합 리허설 + 반복 시행 성공률 기록

**이 태스크는 실기기 필요, 전 과정 사람이 직접 수행. Task 13의 `TODO`로 남겨둔
`_apply_action`의 실제 관절 명령(게인 값 포함)을 이 과정에서 채워야 한다.**

- [ ] **Step 1: `_apply_action`의 각 분기(SWEEP/NUDGE/DESCEND_AND_GRIP/LIFT)를 실제
  LeKiwi 관절 명령으로 채우기** — LeKiwiClient의 액션 스페이스(관절 이름 등)를
  `lerobot_venv`의 `config_lekiwi.py`/캘리브레이션 파일로 재확인한 뒤 구현. 처음엔
  보수적인 소폭 이동값으로 시작해 반복 실행하며 튜닝.

- [ ] **Step 2: 에이보→SSH 트리거 전체 경로로 1회 실행**

```bash
# 에이보 dialogue_node가 실행 중인 상태에서:
ros2 topic pub -1 /lekiwi_command std_msgs/msg/String "{data: pick_pill_bottle}"
```

Expected: 노트북에서 `pick_pill_bottle.py`가 실행되고, LeKiwi가 실제로 움직여 약통에
접근·집기 시도

- [ ] **Step 3: 책상 위 정해진 범위 내 여러 지점(예: 5곳) × 각 3회 반복 시행**,
  성공/실패와 소요시간을 기록

```bash
mkdir -p /home/roboseasy/lekiwi-pill-pickup/data
cat >> /home/roboseasy/lekiwi-pill-pickup/data/trial_results.md << 'EOF'
# 반복 시행 결과

| 지점 | 시도 | 성공여부 | 소요시간(초) | 비고 |
|---|---|---|---|---|
EOF
# 매 시행마다 표에 한 줄씩 추가
```

- [ ] **Step 4: 음성 트리거 전체 경로("에이보야, 약 좀 찾아서 갖다줘")로 최소 3회
  end-to-end 확인** (LLM tool-calling 판단 → `/lekiwi_command` 발행 → SSH 실행 →
  결과에 따른 음성 응답까지)

- [ ] **Step 5: 결과를 설계 문서/발표자료용으로 정리**

```bash
cd /home/roboseasy/lekiwi-pill-pickup
git add data/trial_results.md
git commit -m "Add: 통합 리허설 반복 시행 결과"
```

---

## Self-Review 체크리스트 (계획 작성자용, 참고)

- **스펙 커버리지**: 설계 문서의 아키텍처/제어알고리즘/인식계획/에러처리/테스트계획/
  열린질문 전부 태스크로 반영됨(Task 1/3/4/5/11/12가 "열린 질문" 해소, Task 6~10이
  제어알고리즘의 각 요소, Task 13이 통합, Task 16이 테스트계획)
- **자리표시 스캔**: Task 13의 `_apply_action` TODO 4개는 의도적 — 실기기 게인 튜닝은
  코드로 미리 쓸 수 없는 값이라 Task 16에서 채우도록 명시적으로 분리함(모호한 "적절히
  처리" 식이 아니라 정확히 무엇을 채워야 하는지 주석에 명시)
- **타입/시그니처 일관성**: `Observation`/`Action`/`PickState`/`ActionType`이 Task 10
  정의 그대로 Task 13에서 동일하게 쓰임, `estimate_distance_cm`/`compute_alignment_error`/
  `is_aligned`/`grasped_something`/`load_model_and_detect` 시그니처도 Task 6~9 정의와
  Task 13 사용처가 일치함 확인
