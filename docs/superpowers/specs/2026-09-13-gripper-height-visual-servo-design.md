# 그리퍼 높이 시각 서보 (프론트캠 기준 정렬 + 바닥 근접 폴백)

## 배경 / 문제

`WristServo`는 지금까지 두 가지 방식으로 "충분히 내려갔는지"를 판단해왔다:

1. 손목캠 박스 크기(`target_size_px`) 도달 여부
2. `front_hint`: 프론트캠 정중앙 작은 창에 보라색(그리퍼) 비율이 문턱을 넘는지

두 방식 모두 **카메라 교체나 팔 재조립(서보 혼 재장착 등) 같은 물리적 변화에 취약**하다 —
실제로 2026-09-13 손목캠을 교체한 뒤, `grasp.json`에 저장된 목표 자세(joint 각도)와
`target_size_px` 문턱값이 더 이상 맞지 않게 되어, 수동으로 `arm_shoulder_lift.pos`를
몇 도씩 조정해가며 맞추는 작업을 반복해야 했다(느리고, 서버를 내릴 때마다 안전 롤아웃이
자세를 홈으로 되돌려버려 매번 처음부터 다시 재보는 등 비효율적이었다).

또한 재시도 시 팔을 "더 뻗게" 만드는 `retry_overreach`가 pick→grasp 벡터 전체를
연장하는 방식이라, 그 벡터 안에 포함된 pan(좌우) 차이까지 같이 늘어나 재시도할 때마다
왼쪽으로 밀리는 부작용이 있었다(이번 세션에 발견, `retry_overreach=0`으로 우회 중).

## 목표

손목캠 박스 크기나 미리 기록해 둔 joint 각도에 의존하지 않고, **매 프레임 눈에 보이는
그리퍼와 약통의 실제 상대적 높이**로 "충분히 내려갔는지"를 판단한다. 카메라나 팔
구성이 바뀌어도 재조정 없이 그대로 동작하는 것이 목표다.

## 아키텍처

### 1. 그리퍼 bbox 추출 (`grasp_check.py` 확장)

기존 `purple_mask()`(HSV 보라색 마스크, `--check.*` 설정 재사용)에 OpenCV
`findContours`를 적용해 가장 큰 보라색 덩어리의 바운딩박스를 뽑는 순수 함수
`purple_bbox(frame_bgr, cfg) -> tuple[int,int,int,int] | None`를 추가한다.

**오검출 방지**: 화면 전체에서 가장 큰 보라색 컨투어만 쓰는 게 아니라, 이번 프레임의
약통 YOLO 박스와 같은 화면의 **가운데 세로선 기준 좌우 인접 범위** 안에 있는 컨투어만
후보로 삼는다(팔 상단부 등 관계없는 보라색 부위 배제). 후보가 없으면 `None`.

### 2. 높이 정렬 (headless_worker.py, WristServo 대체 신호)

`arm.state == "SERVO"`이고 `front_view`가 있을 때, 기존 `descend_hint` 계산 자리
옆에 다음을 계산한다:

```python
gripper_box = purple_bbox(frames_bgr[front_view], cfg.check)
bottle_box = largest(dets_by_view[front_view])  # 이미 있는 헬퍼 재사용
front_dy = None
if gripper_box and bottle_box:
    front_dy = gripper_box_center_y - bottle_box_center_y
```

`front_dy`를 `WristServo.update()`에 새 인자로 전달한다. **`front_dy`가 주어지면
기존 `y_anchor` 기반 세로 보정(`dy`/`y_ok` 계산)을 완전히 건너뛰고 `front_dy`로
대체한다** — 두 신호가 동시에 `tilt_delta`를 건드리면 서로 다른 방향으로 다툴 수
있으므로, 항상 둘 중 하나만 매 프레임 단독으로 작동한다. `front_dy`가 `None`이면
(보라색 인식 실패) 기존 `y_anchor` 로직으로 자동 폴백한다.

### 2-1. 기존 `front_hint`(즉시 READY 지름길)와의 우선순위 정리

기존 코드(`wrist_servo.py:443`)는 `descend_hint and front_hint and attempt==0 and x_ok`
이면 크기·세로 정렬과 무관하게 **즉시 READY**로 넘겨버리는 지름길이다. 이건 "손목캠
크기 기준이 카메라마다 안 맞을 수 있으니, 대신 프론트에서 높이만 확인되면 믿고
집는다"는 취지였는데, `front_dy`가 들어오면 **이 지름길을 끈다** — 이제 높이는
`front_dy`가 매 프레임 연속적으로 정확히 알려주므로, 굳이 "1회성 확인 후 바로
집기"로 도박할 필요가 없다. 대신 READY 조건은 `size_reached and front_dy 문턱
이내`로 통일한다 (기존 `attempt==0` 제한도 함께 사라진다 — `front_dy`는 재시도
여부와 무관하게 매 프레임 유효한 신호이기 때문).

### 2-2. 바닥-폴백은 "상태"가 아니라 "세로축 전용 병렬 동작"

처음 설계에서 바닥-근접 폴백을 `FLOOR_DESCEND`라는 별도 최상위 상태로 넣으려 했는데,
이러면 CENTERING/APPROACHING/REFINING 같은 기존 가로/깊이 진행 상태와 **서로 배타적**이
되어 버려 우선순위 충돌이 생긴다(폴백 중엔 가로 정렬이나 뻗기가 멈춰야 하는지 불명확).
대신 폴백은 **세로(높이) 축에만 관여하는 병렬 동작**으로 만든다 — 가로 정렬(pan)과
뻗기(progress)는 기존 상태 로직대로 계속 진행하고, 세로 보정(tilt_delta)만 상황에 따라
"front_dy 기반" 또는 "바닥-근접 하강" 둘 중 하나로 계산되는 구조다. 즉 `WristServo.state`
값 자체는 그대로(CENTERING/APPROACHING/REFINING/READY/LOST)이고, 그 안에서 세로
보정치만 소스가 바뀐다.

### 3. 바닥 근접 폴백 (안전장치)

`front_dy`가 `front_hint_lost_s`(신규 설정, 기본 1.0초) 이상 계속 `None`이면(보라색을
안정적으로 못 잡음), 2-2에서 정한 대로 **세로 보정치의 소스만** 아래 방식으로
바뀐다(가로 정렬·뻗기는 그대로 진행):

- **오직 `arm_shoulder_lift.pos`, `arm_elbow_flex.pos` 두 관절만** 아주 느린
  속도(신규 설정 `floor_descend_dps`, 기본 2 deg/s)로 미리 기록해 둔 "바닥 근접"
  자세(신규 파일 `floor_near.json`, `save_pose.py`로 사람이 한 번 재기록)까지
  내려간다. **pick→grasp 벡터 연장 방식은 쓰지 않는다** — 재시도 좌우 드리프트
  버그를 재현하지 않기 위해 두 관절만 직접, 절대값 기준으로 목표에 접근시킨다.
- 최대 하강량 상한(`floor_descend_max_deg`, 기본 15deg)을 둬서, 인식이 계속
  실패해도 무한정 내려가지 않는다.
- 보라색이 다시 잡히면(front_dy != None) 즉시 1번 방식으로 복귀한다.

### 4. 파일/설정 변경 요약

- `services/pickplace/grasp_check.py`: `purple_bbox()` 추가.
- `services/pickplace/wrist_servo.py`: `GraspArgs`에 `front_hint_lost_s`,
  `floor_descend_dps`, `floor_descend_max_deg`, `floor_pose_file` 필드 추가.
  `WristServo.update()`가 `front_dy: float | None = None` 인자를 받도록 확장,
  바닥-근접 폴백 상태(`FLOOR_DESCEND`) 추가.
- `services/pickplace/headless_worker.py`: `front_dy` 계산 및 `arm.update()`로
  전달하는 배선 추가.
- `webui/run_pickplace_ui.py`: `floor_pose_file` 기본 경로 추가.
- 새 자세 파일: `~/.PhysicalLabs/pickplace/lekiwi01/poses/floor_near.json`
  (구현 완료 후 `save_pose.py --name floor_near`로 한 번 수동 기록 필요).

## 에러 처리

- 그리퍼도 약통도 안 보이는 완전 소실 상태는 기존 `LOST` 로직(변경 없음)을 그대로 탄다.
- `floor_near.json`이 없으면 폴백 모드 진입 시 에러 없이 그냥 폴백을 건너뛰고 기존
  `LOST` 판정으로 넘어간다(설치 초기에 파일이 없어도 앱이 죽지 않게).

## 테스트

- `purple_bbox()`: 합성 이미지(보라색 사각형 그려서)로 좌표 추출 정확성 테스트.
- `WristServo.update(front_dy=...)`: front_dy 우선 사용, None일 때 y_anchor 폴백,
  `front_hint_lost_s` 경과 후 `FLOOR_DESCEND` 전환, 관절 두 개만 움직이는지,
  `floor_descend_max_deg` 상한 검증 — 기존 `test_wrist_servo.py` 패턴대로 순수
  단위테스트.
- 실제 하드웨어 확인은 라이브 테스트로 진행(자동화 불가 영역).
