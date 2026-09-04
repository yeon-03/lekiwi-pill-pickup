# LeKiwi06 SLAM & Nav2 기술 명세서

작성일 2026-08-27 · 대상 기체 `lekiwi06` (192.168.0.206)

이 문서는 **완성된 시스템의 사양과 운용 절차**를 기술한다. 구축 과정에서
겪은 문제와 그 해결 이력은 여기 담지 않는다.

---

## 1. 시스템 구성

### 1.1 하드웨어

| 항목 | 사양 |
|---|---|
| 연산 | Raspberry Pi 4 Model B Rev 1.5, 4코어 @ 1.8 GHz |
| OS / 미들웨어 | Ubuntu + ROS 2 Jazzy (Docker), `ROS_DOMAIN_ID=42` |
| 라이다 | YDLidar Tmini Plus, `/dev/ttyUSB0`, 약 6.1 Hz (164 ms/회전) |
| IMU | BMI160 (I2C), 6축 |
| 구동 | 옴니 3륜, Feetech 서보 ID 7·8·9 |
| 팔 | 서보 ID 1~6 (동일 버스) |
| 서보 버스 | `/dev/ttyACM0`, 1 Mbps, **반이중 — 동시에 두 프로세스가 쓸 수 없다** |

### 1.2 기구학 상수

`~/lekiwi_profile.sh` 한 파일에 모여 있다. **다른 르키위로 이식할 때 고치는
파일은 이것 하나뿐이다.**

| 상수 | 값 | 검증 |
|---|---|---|
| `LEKIWI_BASE_R` | 0.13647 m | 실측 (3바퀴 제자리 회전). lerobot 기본값 0.125는 9% 틀림 |
| `LEKIWI_WHEEL_R` | 0.05 m | 설계값 |
| `LEKIWI_WHEEL_SIGN` | `1 1 1` | 배선에 따라 뒤집힘 |
| `LEKIWI_LIDAR_YAW` | 0.0° | 직진 주행으로 검증 |
| `LEKIWI_WHEEL_BLIND` | `179.5:38.5, 61.0:35.5, -59.3:34.0` | 실측 (`deadbeam.py`) |

> **이식 시 필수 재검증**: 라이다 장착각(`LEKIWI_LIDAR_YAW`)은 기체마다 다르다.
> 옴니휠 자기반사 3개는 120° 간격이라 각도만으로는 어느 바퀴인지 확정할 수
> 없다. 반드시 **직진 0.5 m 주행 1회**로 확정할 것.

### 1.3 TF 트리

```
map
 └─ odom            ← AMCL (주행 시) 또는 Cartographer (SLAM 시).  6.1 Hz
     └─ base_link   ← lekiwi_base_node.py (휠 오도메트리).  25 Hz
         ├─ base_footprint   xyz  0     0      -0.055
         ├─ lidar_link       xyz  0     0      -0.053   rpy 0 0 0
         └─ imu_link         xyz  0    -0.055   0.020
```

`map→odom` 발행자는 **동시에 하나만** 존재해야 한다. Cartographer와 AMCL을
같이 띄우면 TF가 충돌한다 (`start_all.sh` → `run_nav2.sh` 순서에서
`stop_cartographer.sh`가 반드시 선행).

### 1.4 주요 토픽

| 토픽 | 타입 | 주기 | 발행자 |
|---|---|---|---|
| `/scan` | LaserScan | 6.1 Hz | `ydlidar_node.py` |
| `/odom` | Odometry | 25 Hz | `lekiwi_base_node.py` |
| `/imu` | Imu | — | `bmi160_node.py` |
| `/tf` | TFMessage | 31 Hz 합계 | 베이스 25 + AMCL 6 |
| `/cmd_vel_smoothed` | Twist | 5 Hz | velocity_smoother |
| `/cmd_vel` | Twist | 5 Hz | **collision_monitor** (최종 관문) |
| `/initialpose` | PoseWithCovarianceStamped | 이벤트 | `refine_pose.py`, RViz |
| `/amcl_pose` | PoseWithCovarianceStamped | 이벤트 | AMCL |

---

## 2. 라이다 전처리 (`ydlidar_node.py`)

원시 시리얼 스트림에서 `/scan`까지 6단계를 거친다. 순서가 중요하다.

| # | 단계 | 설정 | 목적 |
|---|---|---|---|
| 1 | 시리얼 파싱 | 3바이트/샘플 | — |
| 2 | 자기 반경 제거 | `--self-radius` | 로봇 몸체 반사 |
| 3 | **강도 필터** | `--min-intensity 25` | 약한 반사 제거. **방 한가운데 가짜 벽의 주원인** |
| 4 | **바퀴 마스킹** | `--mask-wheels --wheel-margin 2.0` | 옴니휠 3개가 시야의 **30.4%**를 가림. 그 방향은 장애물도 자유공간도 아님 |
| 5 | **디스큐** | `--deskew` | 회전 중 왜곡 보정. 회전당 odom yaw 변화량 사용 |
| 6 | 장착각 변환 | `LEKIWI_LIDAR_YAW` | `lidar_link` → `base_link` |

선택 사항:

- `--fov-center C --fov-width W` — 시야 제한 (실험용. 이 시야로는 정합 불가)
- `--no-return-inf` — 미탐지를 `inf`로. 바퀴 마스킹과 함께 써야 안전

**디스큐 구현 주의**: 회전 주기 `T`를 `wz × T`로 계산하면 안 된다. `poll()`이
`read(4096)`에서 블록되어 `T`가 실제 0.156 s 대신 0.30 s로 읽힌다. 회전당
odom yaw 델타를 직접 쓴다.

---

## 3. SLAM — Cartographer 2D

설정: `~/lekiwi06_cartographer.lua`

### 3.1 프레임 구성

```lua
tracking_frame     = "imu_link"    -- 이 기체는 실제 IMU 가 있다
published_frame    = "odom"
odom_frame         = "odom"
provide_odom_frame = false         -- odom 은 lekiwi_base_node.py 가 발행
use_odometry       = true
use_imu_data       = true
```

> **IMU의 역할**: Cartographer 2D는 IMU를 **중력 정렬과 자이로**에만 쓴다.
> 선형 가속도는 위치 적분에 절대 쓰이지 않는다
> (`optimization_problem_2d.cc`: *"IMU data is not used in 2D optimization"*).
> 가속도계로 odom 오차를 줄이려는 시도는 이 구조에서 효과가 없다.

### 3.2 거리 설정

```lua
min_range               = 0.05
max_range               = 4.5
missing_data_ray_length = 3.0
```

**동작 규칙**: `min_range`~`max_range` 안의 값은 **모두 히트(장애물)**가 된다.
자유공간으로 비워지는 것은 `max_range`를 **넘은** 반환뿐이다. `max_range`를
9.0으로 두면 노이즈 반환이 먼 거리의 가짜 벽이 된다 — 그래서 4.5로 낮췄다.

### 3.3 스캔 매칭 / 최적화

```lua
use_online_correlative_scan_matching = true
ceres_scan_matcher.occupied_space_weight = 20.0
ceres_scan_matcher.translation_weight    = 10.0
ceres_scan_matcher.rotation_weight       = 40.0
submaps.num_range_data                   = 45
motion_filter.max_angle_radians   = 1.0°
motion_filter.max_distance_meters = 0.1
motion_filter.max_time_seconds    = 2.0
POSE_GRAPH.constraint_builder.min_score                  = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 4
```

**알려진 개선 여지**: 현재 `min_score 0.65`에서 약 15%의 허위 루프 클로저가
발생한다. `min_score 0.78`, `linear_search_window 2.0`,
`max_constraint_distance 4.0` 으로 재매핑하면 개선될 것으로 보인다 (미적용).

### 3.4 후처리 — `despeckle_map.py`

SLAM 결과에는 이웃 없는 고립 점유 셀(허위 반환 흔적)이 남는다. 이 셀들이
팽창하면 **로봇 자신이 서 있는 칸을 막아** 경로 계획이 시작조차 못 한다.

```bash
python3 ~/leisaac/scripts/despeckle_map.py <in.yaml> <out_이름> [최소이웃수]   # 기본 3
```

현재 운용 지도 `map_0827_1534_clean` = `map_0827_1534`에서 고립 셀 12개 제거.

---

## 4. 자율주행 — Nav2

설정: `~/nav2_lekiwi06.yaml`

### 4.1 AMCL

```yaml
robot_model_type: "nav2_amcl::OmniMotionModel"   # 옴니 전용. 기본값이면 vy 무시
alpha1~alpha5:    0.2
laser_max_range:  6.0
max_particles:    5000
min_particles:    1000
update_min_d:     0.10      # 이 거리만큼 움직여야 갱신
update_min_a:     0.15
```

> **정지 중에는 갱신하지 않는다.** `update_min_d`/`update_min_a` 때문에
> 로봇이 멈춰 있으면 `/amcl_pose`도 `/particle_cloud`도 나오지 않는다.
> 이는 고장이 아니다.
>
> **재시작 없이 재측위하는 서비스**: `/set_initial_pose`,
> `/reinitialize_global_localization`, `/request_nomotion_update`

### 4.2 플래너 / 컨트롤러

| 항목 | 값 |
|---|---|
| 플래너 | `NavfnPlanner`, `allow_unknown: true` |
| 컨트롤러 | `MPPIController`, `motion_model: "Omni"` |
| 제어 주기 | **5.0 Hz** |
| `batch_size` | 400 |
| `time_steps` × `model_dt` | 35 × 0.2 s = **7.0 s 예측 구간** |
| 속도 상한 | vx ±0.10, vy 0.08, wz 0.25 m/s·rad/s |
| 도착 허용오차 | 0.10 m / 0.20 rad |

> `motion_model`을 기본값 `DiffDrive`로 두면 **`vy`가 통째로 무시되어**
> 옴니 로봇이 옆으로 못 간다. 반드시 `Omni`.

**제어 주기 5 Hz의 근거**: Pi 4에서 10 Hz는 실측 7.7~8.7 Hz밖에 나오지 않았고,
그 미달이 BT의 `compute_path_to_pose` 응답 대기 타임아웃 → `Goal failed`를
유발했다. 5 Hz는 안정적으로 유지되며 (`Control loop missed` 경고 0건),
0.10 m/s에서 2 cm마다 재계획한다.

### 4.3 코스트맵

| | local | global |
|---|---|---|
| 크기 | 3 × 3 m (rolling) | 지도 전체 |
| 해상도 | 0.05 m | 0.05 m |
| 갱신 / 발행 | 5 Hz / 2 Hz | 1 Hz / 1 Hz |
| `robot_radius` | 0.18 m | 0.18 m |
| `inflation_radius` | 0.22 m | 0.22 m |
| `cost_scaling_factor` | 3.0 | 3.0 |

레이어: `StaticLayer` → `ObstacleLayer`(`/scan`) → `InflationLayer`

> **`costmap_2d` 값 인코딩** (디버깅 시 필요):
> 253(inscribed)→**99**, 254(lethal)→**100**, 255(unknown)→**−1**.
> 치명 영역을 결정하는 것은 `inflation_radius`가 아니라 **`robot_radius`**다.
>
> **알려진 제약**: `robot_radius 0.18`에서는 오른쪽 방에 도달하지 못한다.
> 0.16으로 낮추면 열릴 것으로 보인다 (바퀴 최외곽 실측 0.158 m). 미적용.

### 4.4 속도 평활 / 충돌 감시

```
MPPI → /cmd_vel_smoothed → collision_monitor → /cmd_vel → 베이스
```

```yaml
velocity_smoother:
  max_velocity: [ 0.08,  0.06,  0.25]
  min_velocity: [-0.08, -0.06, -0.25]
  max_accel:    [ 0.3,   0.3,   0.6 ]

collision_monitor:
  cmd_vel_in_topic:  "cmd_vel_smoothed"
  cmd_vel_out_topic: "cmd_vel"
  source_timeout:    1.0
  polygons: ["FootprintApproach"]
```

> `collision_monitor`가 **마지막 관문**이다. 라이다가 없거나 스캔이
> `source_timeout`(1.0초)보다 오래되면 **모든 명령을 조용히 버린다** —
> 키보드 텔레옵이 아무 반응 없는 것처럼 보이는 원인.
> 이를 우회하려고 `/cmd_vel`에 직접 발행하면 안전장치가 통째로 사라진다.

---

## 5. 위치 정합 — `refine_pose.py`

AMCL 자세를 지도에 맞춰 정밀 보정하고 `/initialpose`로 발행한다.

### 5.1 지표

**스캔 끝점에서 가장 가까운 벽까지의 거리**(ICP 잔차와 동일 개념)를 최소화한다.

```
D = 지도 점유셀에 대한 거리 변환(BFS)
cost = mean( min(D[끝점], 0.30) )      # 빔 최대 220개
```

> **점유셀 적중률을 쓰면 안 된다.** 그 지표는 지도에 점이 많은 구역을
> 편애해서, 실제로 **엉뚱한 방을 92.8%로 고르고 정답을 67%로 매긴** 적이 있다.
> 판별력은 꼬리에 있다 — 끝점의 몇 %가 벽에서 20 cm 이상 떨어져 있는가.

### 5.2 3단계 성긴→정밀 탐색

| 단계 | 범위 | 위치 간격 | yaw 간격 |
|---|---|---|---|
| 1 | `--range` / `--yaw` 전체 | 8 cm | 3.0° |
| 2 | ±0.10 m / ±3° | 3 cm | 1.0° |
| 3 | ±0.03 m / ±1° | 1 cm | 0.5° |

전수 조사는 평가 횟수가 범위의 세제곱으로 늘어 `--range 0.8 --yaw 40`에서
**531,441회 (>180초)**가 된다. 단계별 축소로 **12,495회 / 6.2초**.

### 5.3 사용법

```bash
# 현재 AMCL 자세 주변을 다듬는다
bash ~/relocalize.sh                                   # 기본 ±0.4 m / ±15°
bash ~/relocalize.sh <map.yaml> --range 0.6 --yaw 180  # 방향을 완전히 잃었을 때

# goal 좌표를 기준점으로 (도착 직후에만 유효)
python3 ~/refine_pose.py <map.yaml> --publish --at 1.50 0.58 0 --range 0.35 --yaw 20
```

**`--at` 의 의미**: pick 중에는 서보 버스를 넘겨 오도메트리가 끊기므로 AMCL
추정이 흐트러진다. 반면 Nav2가 향한 **goal 좌표는 우리가 아는 값**이고 도착
허용오차(0.10 m) 안에 있다. 흐트러진 추정보다 나은 출발점이므로 좁은 범위만
훑으면 된다.

**발행 가드**: 보정 결과가 현재 AMCL보다 나쁘면 발행하지 않는다. 로봇이
실제로는 그 goal에 도달하지 못했을 때(주행 실패, 큰 회전 오차) 좁은 탐색창이
엉뚱한 국소 최적에 빠지는데, 그것을 AMCL에 밀어넣으면 위치를 완전히 잃는다.

---

## 6. abo 연동 — `abo_nav_bridge.py`

abo(반려로봇)는 **문자열 토픽 하나만** 알면 된다. 좌표도 Nav2 액션도 모른다.

### 6.1 인터페이스

| 방향 | 토픽 | 타입 | 내용 |
|---|---|---|---|
| 입력 | `/abo/command` | String | 아래 명령어 표 |
| 출력 | `/abo/status` | String | 사람이 읽는 한국어 한 줄 |
| 출력 | `/abo/state` | String | `idle`·`moving`·`arrived`·`picking`·`returning`·`done`·`failed`·`rejected`·`canceled` |
| 출력 | `/abo/pick_request` | String | 목적지 이름. **도착 직후** 발행 |
| 입력 | `/abo/pick_done` | Bool | `true` 성공 → 복귀 시작 |

**명령어**

| 명령 | 동작 |
|---|---|
| `home` | `waypoints.yaml`의 이름으로 이동 |
| `go home` / `가자 home` | 접두사 `go`/`가자`/`이동`은 무시 |
| `goto 1.5 0.58` | 좌표 직접 (yaw는 현재 방향 유지) |
| `goto 1.5 0.58 90` | 좌표 + 방향(도) |
| `stop` / `정지` | 진행 중인 목표 취소 |
| `where` / `어디` | 현재 위치 발행 |
| `list` | 등록된 목적지 이름 발행 |
| `fetch center` | **왕복 미션**: 이동 → pick → 복귀 |
| `fetch center to home` | 복귀 지점 지정 |

### 6.2 왕복 미션 상태 기계

```
going ──[Goal succeeded]──▶ refine(--at 목적지)
                                    │
                         set_bus(False)  ← 서보 버스를 ZMQ 쪽에 양보
                                    │
                         /abo/pick_request 발행
                                    │
      picking ◀───────────────────┘
         │
    [/abo/pick_done true]
         │
    set_bus(True)  ← 버스 회수. odom 기준을 현재 위치로 재설정
         │
    refine(--at 목적지)  ← 끊긴 오도메트리를 라이다로 회복
         │
    returning ──[Goal succeeded]──▶ refine(--at 복귀지) ──▶ done
```

### 6.3 ZMQ ↔ ROS 2 공존

pick은 다른 담당자가 **ZMQ**로 구현한다. 핵심 제약 두 가지:

1. **서보 버스 배타성** — Linux는 시리얼 포트 배타 개방을 강제하지 **않는다**.
   두 프로세스가 동시에 열 수 있고, 반이중 버스에서 충돌한다(실측: 60회 읽기 중
   4회 실패, `SerialException: multiple access on port?`). 그래서 브리지가
   `/lekiwi_base/set_bus` 서비스로 **명시적으로 양보/회수**한다.
   양보 중에는 `drive()`/`tick()`/`stop()`이 모두 즉시 반환한다.

2. **비동기 처리** — rclpy의 `SingleThreadedExecutor`는 콜백을 하나씩 처리하므로
   블로킹 호출 하나가 모든 ROS 콜백을 얼린다. 브리지는
   `MultiThreadedExecutor(num_threads=2)` + 상태는 `MutuallyExclusiveCallbackGroup`,
   I/O는 `ReentrantCallbackGroup`으로 분리한다.

**버스 회수 시 주의**: 양보 중 바퀴가 얼마나 돌았는지 알 수 없다. `set_bus(True)`는
현재 위치를 새 기준으로 삼아 odom이 튀지 않게 하고, 잃어버린 이동은 **반드시
`refine_pose.py`로 회복**해야 한다.

---

## 7. 운용 절차

전제: 로봇과 데스크톱이 같은 서브넷, `ROS_DOMAIN_ID=42`.

```bash
ssh roboseasy@192.168.0.206          # 비밀번호는 팀에 문의 (문서에 적지 않는다)
```

### 7.1 A. 지도 만들기 (SLAM)

**로봇에서**

```bash
bash ~/start_all.sh
```

순서가 고정되어 있다 — `rsp(TF)` → 라이다 → IMU(바이어스 추정 8초, **건드리지 말 것**)
→ 베이스 → Cartographer. 카토그래퍼가 먼저 뜨면 TF가 없어 실패하고, 센서보다
먼저 뜨면 빈 데이터로 첫 서브맵을 만들어 지도가 두 겹이 된다.

**데스크톱에서 — 화면 보기**

```bash
bash ~/leisaac/scripts/run_rviz_lekiwi06.sh     # 실시간 /map
```

**주행** — 키보드 텔레옵

```bash
bash ~/run_teleop.sh
```

**데스크톱에서 — 지도 저장**

```bash
bash ~/leisaac/scripts/save_map.sh [이름]        # 기본: map_MMDD_HHMM
```

**후처리 + 로봇으로 배포**

```bash
python3 ~/leisaac/scripts/despeckle_map.py \
    ~/leisaac/maps/lekiwi06/<이름>.yaml <이름>_clean

~/miniconda3/envs/leisaac/bin/python ~/leisaac/scripts/deploy_map.py <이름>_clean
```

**정지**

```bash
bash ~/stop_all.sh      # 바퀴 속도 0 + 토크 해제까지 확실히 한다
```

### 7.2 B. 자율주행 (Nav2)

**로봇에서**

```bash
bash ~/stop_cartographer.sh          # 필수! AMCL 과 map->odom TF 가 충돌한다
bash ~/run_nav2.sh ~/maps/map_0827_1534_clean.yaml
```

지도를 생략하면 `~/maps`에서 가장 최근 `.yaml`을 쓴다.

**지도 재발행** (AMCL이 `Waiting for map....`에서 멈추면)

```bash
ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap \
  "{map_url: /home/roboseasy/maps/map_0827_1534_clean.yaml}"
```

`map_server`는 `/map`을 TRANSIENT_LOCAL로 한 번만 발행한다. AMCL이 늦게
구독하면 놓칠 수 있다.

**위치 초기화** — 둘 중 하나

```bash
# ① RViz 의 2D Pose Estimate 로 대충 찍고 다듬기
bash ~/relocalize.sh

# ② 방향을 완전히 잃었을 때
bash ~/relocalize.sh ~/maps/map_0827_1534_clean.yaml --range 0.6 --yaw 180
```

판정 기준: **평균 거리 0.02 m 이하 / 5 cm 이내 빔 80% 이상 / 30 cm 초과 0%**

**데스크톱에서 — RViz**

```bash
bash ~/leisaac/scripts/run_rviz_nav2.sh
```

`lekiwi06_nav2.rviz`에는 SetInitialPose / SetGoal 도구가 들어 있다.
지도는 데스크톱이 직접 읽어 `/map_view`로 발행한다 (로봇의 TRANSIENT_LOCAL
`/map`이 WiFi 너머 늦은 구독자에게 전달되지 않는 문제 회피).

**정지**

```bash
bash ~/stop_nav2.sh     # 컨테이너까지 확실히 죽이고 /cmd_vel 0 을 보낸다
```

### 7.3 C. 왕복 미션 (abo 파이프라인)

**전제**: 7.2 B가 완료되어 Nav2가 살아 있고 위치가 정합된 상태.

**로봇에서 — 중개 노드 기동**

```bash
bash ~/run_abo.sh
# ZMQ 호스트까지 브리지가 관리하게 하려면:
bash ~/run_abo.sh --host-cmd "bash /home/roboseasy/start_lekiwi_host06.sh"
```

**미션 실행**

```bash
# 왕복: center 로 가서 pick 하고 home 으로 복귀
ros2 topic pub --once /abo/command std_msgs/String "{data: 'fetch center'}"

# 상태 지켜보기
ros2 topic echo /abo/status
```

**pick 담당자 쪽** — `/abo/pick_request`를 받으면 작업하고, 끝나면:

```bash
ros2 topic pub --once /abo/pick_done std_msgs/Bool "{data: true}"
```

**단발 이동**

```bash
ros2 topic pub --once /abo/command std_msgs/String "{data: 'home'}"
ros2 topic pub --once /abo/command std_msgs/String "{data: 'goto 1.5 0.58 90'}"
ros2 topic pub --once /abo/command std_msgs/String "{data: '정지'}"
```

### 7.4 D. 런치 파일 (shell 대신)

```bash
# 센서만.  layer 로 TF 체인을 한 층씩 올릴 수 있다
ros2 launch ~/launch/lekiwi_sensors.launch.py            # layer:=3 (기본)
ros2 launch ~/launch/lekiwi_sensors.launch.py layer:=1   # rsp 만 (정적 TF)
ros2 launch ~/launch/lekiwi_sensors.launch.py layer:=2   # + 라이다 + IMU

# 센서 + SLAM (카토그래퍼는 14초 뒤 기동)
ros2 launch ~/launch/lekiwi_slam.launch.py

# 센서 + AMCL + map_server (주행 기능 없음 -- 위치추정만)
ros2 launch ~/launch/lekiwi_amcl.launch.py

# 센서 + Nav2 + 자동 재정합 + abo 브리지
ros2 launch ~/launch/lekiwi_nav.launch.py
```

층별 브링업 절차는 `bringup-guide.md` 를 볼 것.

`lekiwi_nav.launch.py`의 시각표:

| 시각 | 동작 |
|---|---|
| 0 s | 센서 (IMU 제외) |
| 6 s | Nav2 (`delay` 인자) |
| 45 s | `/map_server/load_map` 재발행 |
| 52 s | 자동 재정합 (`auto_relocalize`) |
| 58 s | abo 브리지 (`abo`) |

모든 센서 노드는 `respawn=True`.

### 7.5 목적지 등록 — `~/waypoints.yaml`

```yaml
map: map_0827_1534_clean

points:
  home:    {x: 0.00, y: 0.00, yaw:  0.0}
  center:  {x: 1.50, y: 0.58, yaw:  0.0}
  upper:   {x: 1.10, y: 1.40, yaw: 90.0}
```

> **좌표는 지도마다 다르다.** 지도의 (0,0)은 그 SLAM을 시작한 자리다.
> 지도를 새로 따면 여기 값도 전부 다시 잡아야 한다.

---

## 8. 성능

### 8.1 CPU 예산 (Pi 4, 4코어 = 400%)

유휴 상태 측정값:

| 프로세스 | CPU |
|---|---|
| Nav2 컨테이너 | 152% |
| `lekiwi_base_node.py` | 23% |
| `ydlidar_node.py` | 8.5% |
| `abo_nav_bridge.py` | 2.1% |
| **합계** | **~186%** |
| **MPPI 가용 여유** | **~214%** |

적용된 최적화:

| 항목 | 이전 → 이후 | 근거 |
|---|---|---|
| 브리지 TF 리스너 → `/amcl_pose` 구독 | 46% → **2.1%** | `/tf` 56 Hz를 파이썬에서 파싱하는 비용. 브리지는 자세를 목표 yaw 기본값에만 쓴다 |
| `/odom` 발행 25 Hz (기존 50) | 36% → **23%** | `/tf` 구독자 **전원**이 메시지당 비용을 낸다 |
| 제어 주기 5 Hz (기존 10) | — | 10 Hz는 실측 7.7 Hz로 미달 → BT 타임아웃 |
| 라이다 nice −10, 베이스 nice −5 | — | 스캔 지연 2.4초(증가 중) → 0.30초(감소 중) |

**남은 최적화 여지** (미적용):

| 방법 | 예상 효과 | 위험 |
|---|---|---|
| 미사용 Nav2 노드 제거 (`docking_server`, `route_server`, `waypoint_follower`, `smoother_server`) | ~30% | 커스텀 런치 필요 |
| AMCL `max_particles` 5000 → 2000 | ~10% | 작은 방이라 손실 거의 없음 |
| 코스트맵 `publish_frequency` ↓ | ~5% | RViz 갱신만 느려짐 |
| MPPI → Regulated Pure Pursuit | ~60% | 동적 장애물 회피 성능 저하 |
| Nav2를 데스크톱으로 이전 | 152% 전부 | WiFi 지연, `/map` TRANSIENT_LOCAL 문제 |

빈 rclpy 노드의 유휴 비용은 executor 종류와 무관하게 약 6%다
(single 6.0% / multi(2) 6.4% / multi(4) 6.8%). 그 이상은 우리 코드다.

### 8.2 측정된 주행 성능

2026-08-27 왕복 미션 (`fetch center`):

| 구간 | 결과 |
|---|---|
| home → center | `Goal succeeded` |
| 도착 시 goal 기준 재정합 | 0.033 m → **0.006 m** (5 cm 이내 90.0%) |
| pick 인계 | 포트 양보/회수 정상 |
| center → home | `Goal succeeded` |
| 도착 시 goal 기준 재정합 | 0.057 m → **0.013 m** (5 cm 이내 78.6%) |
| 최종 위치 오차 | home 기준 **12.6 cm** |
| `Control loop missed` 경고 | **0건** |

재정합 성능:

| 탐색 범위 | 평가 횟수 | 소요 시간 |
|---|---|---|
| ±0.35 m / ±20° | 1,722 | 1.4 s |
| ±0.8 m / ±40° | 12,495 | 6.2 s |
| ±0.6 m / ±180° | 30,135 | 16.5 s |

---

## 9. 알려진 제약

| 항목 | 내용 |
|---|---|
| 오른쪽 방 도달 불가 | `robot_radius 0.18` → 0.16 필요 (실측 최외곽 0.158 m) |
| 허위 루프 클로저 ~15% | `min_score 0.65` → 0.78, `linear_search_window 2.0` 로 재매핑 필요 |
| 전원 불안정 | 자발적 전원 손실 4회 이상, I2C 버스 드롭 2회. 전원부 점검 필요 |
| 발열 | 72.5 °C (스로틀 임계 80 °C). 냉각 보강 권장 — **오버클럭 금지** |
| IMU 취약 | I2C Errno 5로 죽으면 카토그래퍼가 조용히 멈춘다. 3회 재시도 + 샘플 건너뛰기 적용됨 |

---

## 10. 이식 절차 (다른 르키위)

```bash
~/miniconda3/envs/leisaac/bin/python ~/leisaac/scripts/deploy_lekiwi.py <대상IP>
```

**주소 규칙**: `lekiwi0N` = `192.168.0.20N` (고정 IP). Wi-Fi가 죽으면 기체가
네트워크에서 완전히 사라진다.

이식 후 **반드시** 할 것:

1. `~/lekiwi_profile.sh`의 `LEKIWI_BASE_R`을 3바퀴 제자리 회전으로 재측정
2. `LEKIWI_LIDAR_YAW`를 **직진 0.5 m 주행**으로 확정
   (자기반사만으로는 −30 / +90 / −150 중 어느 것인지 정해지지 않는다)
3. `LEKIWI_WHEEL_BLIND`를 `deadbeam.py`로 재측정 (`LIDAR_YAW`가 바뀌면 같이 바뀜)
4. `LEKIWI_WHEEL_SIGN` 확인 (배선에 따라 뒤집힘)
5. 지도를 새로 따고 `waypoints.yaml` 좌표를 다시 잡을 것

**코드는 전 기체 공통이다.** `*.py`, `run_*.sh`, `*.lua`, `nav2_*.yaml` 은
고치지 않는다. 기체 고유값은 `lekiwi_profile.sh` 하나에만 있다.

---

## 부록. 문제 진단 빠른 참조

| 증상 | 확인할 것 |
|---|---|
| 텔레옵이 아무 반응 없음 | `collision_monitor` — 라이다 없거나 스캔 1초 이상 오래됨 |
| AMCL `Waiting for map....` | `/map_server/load_map` 호출 (TRANSIENT_LOCAL 경합) |
| `/amcl_pose` 안 나옴 | 정상. `update_min_d 0.10` — 움직여야 갱신됨 |
| 방 한가운데 가짜 벽 | `--min-intensity`, `max_range 4.5` 확인 |
| 경로 계획이 시작조차 안 함 | 고립 점유 셀이 팽창해 로봇 칸을 덮음 → `despeckle_map.py` |
| `Goal failed` + `Control loop missed` | CPU 포화. §8.1 참조 |
| `SerialException: multiple access on port?` | 두 프로세스가 `/dev/ttyACM0` 동시 사용 → `set_bus` 확인 |
| `ModuleNotFoundError: rclpy` | `python3`가 conda `(base)`로 잡힘 → `/usr/bin/python3` 명시 |
| SSH가 끊김 (exit 144/255) | `pkill -f` 가 자기 셸을 죽임 → PID 지정 |
| 라이다 스캔이 계속 밀림 | CPU 부하. `renice -n -10` 으로 라이다 우선순위 상향 |
