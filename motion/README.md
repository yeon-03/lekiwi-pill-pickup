# 집기 모션 제어 (Physical Labs `services/pickplace`)

약통을 실제로 집는 제어 로직이다. **로보시지 Physical Labs 앱**
(`physical-labs 0.6.0-0.0.12`, `/opt/physical-labs`)에서 GUI 와 무관한
부분만 가져왔다.

원본은 CLI 도구 `yolo_and_pick/` 이고, **2026-09-04 실물 성공이 확인된**
코드다. Physical Labs 가 그것을 앱으로 이식하면서 제어 로직과 화면을
분리해 두었기에, 화면 쪽만 빼면 그대로 쓸 수 있다.

## 한 줄도 고치지 않았다

원본 패키지 구조(`services/pickplace/…`)를 그대로 유지했다. import 문을
손대지 않으려는 것이다. 게인·상태 전이·판별 규칙은 실기기로 검증된
값이므로 건드리면 안 된다 — 원본 패키지 주석에도 그렇게 적혀 있다.

> 검증된 게인·상태 전이·판별 규칙은 원본과 **한 줄도 다르지 않게** 유지한다.
> 바뀐 것은 CLI 파서·cv2 창·`SystemExit`·lerobot 로봇 의존 같은 I/O 껍데기뿐이다.

고칠 일이 생기면 `/opt/physical-labs` 원본과 이쪽을 **함께** 고칠 것.

## 무엇이 들어 있나

| 파일 | 줄 | 역할 |
|---|---|---|
| `pickplace/wrist_servo.py` | 498 | 손목 카메라 비주얼 서보 — 집기 직전 미세 접근 |
| `pickplace/approach.py` | 358 | 검출 → 바퀴 속도 P 제어, 좌우 정렬 (4단계 우선순위) |
| `pickplace/arm_sequencer.py` | 323 | 접근 완료 후 팔 자세 전환 상태기 |
| `pickplace/grasp_check.py` | 239 | 집기 성공 판별 (front·wrist 양쪽에서 그리퍼 확인) |
| `pickplace/yolo_detect.py` | 181 | YOLO 추론 + 오버레이 |
| `pickplace/place_player.py` | 152 | 녹화 모션 재생해서 놓기 |
| `pickplace/base_return.py` | 145 | 개루프 복귀 — **우리는 안 쓴다** (아래 참고) |
| `pickplace/config.py` | 88 | 설정 묶음 (`PickPlaceConfig`) |
| `pickplace/poses.py` | 82 | 자세 파일 입출력 |
| `pickplace/__init__.py` | 119 | 공개 API |
| `services/net/lekiwi_units.py` | 222 | 서보 단위 변환 (`place_player` 의존) |
| `services/motion_library.py` | 297 | 녹화 모션 읽기 (`place_player` 의존) |
| `config/app_identity.py` | 141 | 캐시 경로 (`motion_library` 의존) |

## 가져오지 않은 것 — GUI 전용

| 파일 | 줄 | 왜 |
|---|---|---|
| `pick_worker.py` | 650 | 전체 조립. QThread·pyqtSignal 에 묶여 있다 |
| `view_worker.py` | 357 | 화면 확인용 뷰어 |
| `motion_play_worker.py` | 199 | 재생 버튼용 |

`pick_worker.py` 가 하던 **조립**은 우리 쪽에서 헤드리스로 다시 쓴다.
로직이 아니라 껍데기만 옮기는 일이다.

## 쓰는 법

```python
import sys
sys.path.insert(0, "motion")          # 이 폴더

from services.pickplace import (
    load_model, infer,            # YOLO 추론
    Approacher, ApproachArgs,     # 바퀴 접근
    ArmSequencer, PickArgs,       # 팔 자세 전환
    WristServo, GraspArgs,        # 손목 서보
    GraspChecker,                 # 성공 판별
    PickPlaceConfig,
)
```

**노트북에서 실행한다.** 카메라 2대와 OpenCV·YOLO 가 필요해 라즈베리파이에서는
못 돈다. 로봇 쪽은 `lekiwi_host` 가 ZMQ 로 영상을 보내는 역할만 한다.

필요한 것: `numpy`, `opencv-python`, `ultralytics` (+ 실기기 연결 시 `lerobot`).

## `pick_worker.py` 가 하던 단계 (헤드리스로 다시 쓸 때 참고)

원본 docstring 의 단계표:

| 단계 | 팔 | 바퀴 |
|---|---|---|
| `PICK` | `ArmSequencer` | `Approacher` (보낸 명령을 `BaseMotionLog` 에 기록) |
| `RETURN_BASE` | 시작 자세 유지 (물건 문 채) | 기록의 **역재생** (`BaseReturnPlayer`) |
| `PLACE` | `PlacePlayer` (녹화 모션) | 프레임의 바퀴 속도 |

제어 루프는 이렇게 돈다.

```python
obs    = robot.get_observation()          # front/wrist 프레임
dets   = infer(model, cfg.yolo, frames)   # YOLO
base_v = ap.update(dets)                  # 바퀴 속도
pose   = arm.update(dets, obs)            # 팔 자세
robot.send_action({**pose, **base_v})
done   = checker.update(dets)             # 성공 판정
```

## ⚠️ `base_return.py` 는 우리 Nav2 로 대체한다

원본 주석:

> LeKiwi 에는 **위치 추정이 앱에 노출되지 않으므로**, 접근 중 보낸 바퀴 속도
> 명령을 기록해 두었다가 **역순으로 부호를 뒤집어** 재생한다. **개루프라 바닥
> 마찰·가속 지연만큼 오차가 남지만** 출발 지점 근처로 돌아오는 데는 충분하다.

| | Physical Labs | 이 저장소의 `nav/` |
|---|---|---|
| 복귀 | 바퀴 명령 역재생 (개루프) | Nav2 + AMCL (폐루프) |
| 오차 | 바닥 마찰만큼 누적 | 왕복 실측 9.1 cm |
| 장애물 | 모름 | costmap 회피 |

우리는 위치추정을 갖고 있으므로 `base_return.py` 대신
[`nav/mission/abo_nav_bridge.py`](../nav/mission/abo_nav_bridge.py) 가 복귀한다.
파일은 참고용으로 남겨 두었다.

## 자율주행과 어떻게 만나나

ROS2 토픽 두 개로만 붙는다. 자세한 것은
[`nav/README.md`](../nav/README.md) 의 "집기와 잇기".

```
nav (로봇 Pi)  ──/abo/pick_request──▶  집기 (노트북)
               ◀──/abo/pick_done────
```

집기 직전 `/lekiwi_base/set_bus` 로 서보 버스를 양보한다 — 반이중이라 두
프로그램이 동시에 열면 통신이 깨진다.

**집기 중에는 로봇이 움직인다.** `approach.py` 가 게걸음·회전·전진을 하고
대상 사이를 옆걸음한다. 그동안 오도메트리는 멈춰 있으므로(버스를 놓은 동안
바퀴를 못 읽는다), 집기 후 라이다 재정합 범위를 0.80 m 로 넓혀 두었다.

## 출처

로보시지 Physical Labs 앱의 코드다. 사내 프로젝트 간 재사용이며, 외부 공개
시에는 별도 확인이 필요하다.
