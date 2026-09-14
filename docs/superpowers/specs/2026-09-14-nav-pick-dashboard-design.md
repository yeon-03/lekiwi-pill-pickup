# LeKiwi 관제 대시보드 (Nav2 지도 + YOLO 카메라) 설계

날짜: 2026-09-14
목적: 약통 심부름 미션(주행 → 집기 → 복귀)을 브라우저 한 화면에서 본다. 저장소에 있는
지도 위에 로봇 위치(TF)·odom 궤적·LaserScan·AMCL 파티클·Nav2 경로를 겹쳐 그리고, 그 옆에
집기 중인 front/wrist 카메라(오버레이 포함)와 미션 단계·토픽 입출력을 띄운다.
관련 문서: [`2026-09-13-pickplace-demo-ui-design.md`](./2026-09-13-pickplace-demo-ui-design.md)
(기존 집기 웹 UI — 이 대시보드는 그 `/stream`·`/status` 를 재사용한다)
화면 시안: "A안 수정본" (지도 + 미션 단계 왼쪽, 카메라 두 대 오른쪽 세로, 폭 약 56:44)
→ 2026-09-14 큰 벽 모니터용 한 화면 3열(지도 · 세로 파이프라인 미션 · 카메라)로 바꿈 — 아래 "화면 배치" 절.

## 배경

- 미션 흐름은 팀원 실기기 기록(2026-09-13, 2회 성공) 그대로다: `/abo/command "fetch center color:green"`
  → 로봇 `abo_nav_bridge` 가 Nav2 주행·위치 다듬기·서보 버스 넘김·ZMQ 호스트 기동 →
  `/abo/pick_request "center:green"` → 노트북 `pick_adapter` 가 `scripts/pick_worker_cycle.py`
  (PickPlaceHeadlessWorker) 실행 → `/abo/pick_done` → 호스트 끄기 → 복귀.
- 로봇에는 PR #8 버전이 배포돼 있고(파일 md5 로 확인) **이 작업은 로봇에 아무것도 배포하지 않는다.**
  대시보드는 노트북에서만 돈다.
- **노트북 수신 검증 (2026-09-14)**: 노트북(eduroam 223.194.157.x)과 로봇(223.194.139.x)이
  다른 서브넷이어도 큰 메시지가 온다. rclpy 로 20초 세어 `/scan` 6.0 Hz(720빔),
  `/particle_cloud` 1001개, `/odom` 24 Hz, `/tf`(map→odom→base_link), `/map`, `/amcl_pose` 확인.
  실제 UDP 1~60 KB 전송도 전부 도착. 구독 중 노트북 수신량 약 290 KB/s.
- **AMCL 은 멈춰 있으면 파티클·`/amcl_pose` 를 내지 않는다** (update_min_d 0.10 m /
  update_min_a 0.15 rad). 정지 중 빈 파티클은 고장이 아니다 — 화면은 "마지막 갱신 n초 전"으로 보여준다.
- `lidar_link` 는 `base_link` 대비 회전 0, xy 0 (`nav/config/lekiwi.urdf`, 로봇 사본과 동일).
  `/scan` 각도는 이미 base 방위다.
- 지도 파일 `nav/maps/lekiwi01/map_0912_1654.pgm/.yaml` 은 로봇 `~/maps` 의 것과 md5 동일 (53×48칸, 0.05 m).

## 목표 / 비목표

**목표**
- 지도 위에 로봇 자세·odom 궤적·스캔 점·파티클·Nav2 경로·웨이포인트(home/center) 표시, 주행 중 실시간 갱신
- 집기 단계에서 front/wrist 카메라(기존 오버레이: 박스·십자선·보라색 비율) 표시
- 미션 단계 타임라인(주행 → 집기 → 복귀)과 경과 시간, 브리지 문장(`/abo/status`)
- 토픽 입출력 패널: 토픽별 마지막 값·주기·나이
- 오래된 데이터는 흐리게/표시해서 "멈춘 화면"을 "정상"으로 착각하지 않게
- **정지 버튼 하나** (정지 유지 방식 — 아래 §정지)
- `--no-particles` 로 파티클 구독 자체를 끌 수 있음 (로봇 와이파이 송신량 절약)

**비목표 (v1)**
- 미션 시작 버튼 (에이보 연결 전이라 시작은 기존처럼 명령 토픽으로 한다)
- 에이보 중계기(`medicine_relay.py`) 연동
- `keep_particles.py` 자동 실행 (주행 중 켜면 파티클 고갈 — 표시 때문에 위치추정을 건드리지 않는다)
- 로봇 쪽 코드·설정 변경, RViz 대체 수준의 범용 뷰어(레이어 추가 UI, 좌표 측정 등)
- 지도 편집·목표 지점 클릭 주행

## 구성

```
[로봇 lekiwi01, 도메인 42]  /tf /odom /scan /particle_cloud /amcl_pose /plan /abo/*
        │ DDS (eduroam)
        ▼
[노트북] nav/dashboard/dashboard_node.py   ← /usr/bin/python3 + rclpy + FastAPI, :8001
        │  - ROS 구독 → 메모리 상태(스레드 안전)
        │  - GET  /                      페이지
        │  - GET  /map.png               지도 이미지 (pgm → png, 시작 시 1회)
        │  - GET  /api/map_info          해상도·원점·크기·웨이포인트
        │  - GET  /api/events            SSE, 5 Hz 스냅샷 JSON
        │  - POST /api/stop              정지 (정지 유지 켜기)
        │  - POST /api/stop/release      정지 유지 풀기
        │  - 집기 워커 :8000 의 /status 를 서버에서 읽어 스냅샷에 합친다
        ▼
[브라우저]  canvas 에 지도·레이어를 그림  +  <img src="http://<노트북>:8000/stream/front|wrist">

[노트북] pick_adapter → scripts/pick_worker_cycle.py --web-port 8000
        └─ 집기하는 동안만 기존 create_app(worker) 를 띄움 (/stream/{view}, /status, /estop)
```

### 이전 합의에서 바꾼 점 — 지도는 MJPEG 가 아니라 브라우저 canvas

처음엔 지도 노드가 OpenCV 로 그린 이미지를 MJPEG 로 보내기로 했다. 대신 **데이터(JSON)를 보내고
브라우저가 그린다.**
- 지도가 53×48 칸(2.65 × 2.4 m)이라 이미지로 크게 키우면 흐려진다. canvas 는 화면 크기에 맞춰 선명하게 그린다.
- 노트북이 초당 여러 장을 JPEG 로 인코딩할 필요가 없다 (집기 YOLO 와 CPU 를 나눠 쓴다).
- 레이어별 "오래됨" 표시(흐리게/숨김)가 그리기 코드 한 줄이다.
- 크기: 스캔 유효점 ~420 + 파티클 최대 800점(초과 시 균등 추림) + 궤적 ≤600점 ≈ 5 Hz 에 수십 KB/s, 로컬 브라우저에 충분.

### CORS 가 필요 없는 이유

카메라는 `<img>` 로 교차 출처 로드(허용됨). `/status` 와 정지는 대시보드 **서버**가 127.0.0.1:8000
으로 대신 호출한다. 브라우저가 8000 에 fetch 하지 않으므로 8000 쪽 CORS 설정은 두지 않는다.

## 구성요소

`nav/dashboard/` (노트북 전용, 로봇 배포 목록에 넣지 않는다 — `pick_adapter.py` 와 같은 원칙)

| 파일 | 역할 | 의존 |
|---|---|---|
| `map_geometry.py` | 지도 yaml 읽기, 세계(m)↔픽셀 변환, 자세 합성(map→odom→base_link), 스캔→지도 점, 점 추림 | numpy, yaml (ROS 없음) |
| `mission_timeline.py` | `/abo/state` 전이 + 시각 → 단계 목록·경과 시간, 단계별 시작 시각·마지막 `/abo/status` 문장 | `mission_words` |
| `mission_words.py` | 상태 → 지금 동작 한 문장 `headline(state, color, prev_state)` (에이보 `mission_text.STATE_TEXT` 복사 — 문구를 바꾸면 양쪽을 같이) | 없음 |
| `topic_stats.py` | 토픽별 마지막 값 요약·주기(이동 평균)·나이 | 없음 |
| `stop_latch.py` | 정지 유지 판단: 지금 `stop` 을 보낼지·워커 estop 을 부를지 (시각·브리지 상태 입력) | 없음 |
| `dashboard_state.py` | 위 넷을 묶은 스레드 안전 상태, `snapshot(now)` → JSON dict | 위 넷 |
| `ros_listener.py` | rclpy 노드: 구독 → `dashboard_state` 갱신, 0.1 s 타이머로 `stop_latch` 결과대로 `/abo/command "stop"` 발행 | rclpy, 메시지 타입 |
| `web_app.py` | FastAPI 앱 팩토리 `create_app(state, map_png, map_meta, on_stop, on_release)`, 정지 핸들러 `make_stop_handlers` | fastapi |
| `pick_client.py` | 집기 워커(:8000) `/status` 폴링 스레드와 `/estop` 호출 (거부면 "estop 불가 (8000 응답 없음)") | 표준 라이브러리 |
| `dashboard_node.py` | 진입점: 인자 파싱, rclpy 스핀 스레드 + uvicorn | 위 전부 |
| `static/index.html`, `static/app.js`, `static/style.css` | A안 화면, canvas 그리기, SSE 수신 | 없음 (CDN 불사용) |
| `run_dashboard.sh` | 환경변수(ROS_DOMAIN_ID=42 등) 고정 후 실행 | |
| `preview_server.py` | ROS 없는 미리보기 `--scenario {idle,driving,picking,done,failed} [--port 8011]` — `build_state(scenario, now)` 가 가짜 이벤트를 넣은 상태를 만들고 정지는 로그만 | 위 순수 모듈, fastapi |

순수 로직(`map_geometry`/`mission_timeline`/`topic_stats`/`stop_latch`/`dashboard_state`)은 ROS 없이 테스트한다.

### ROS 구독 (QoS 는 발행 측에 맞춤 — `ros2 topic info -v` 로 확인한 값)

| 토픽 | 타입 | QoS | 쓰임 |
|---|---|---|---|
| `/tf` | tf2_msgs/TFMessage | reliable, depth 100 | map→odom, odom→base_link 최신값 |
| `/tf_static` | tf2_msgs/TFMessage | reliable, transient_local | base_link→lidar_link (없으면 URDF 값 0 사용) |
| `/odom` | nav_msgs/Odometry | reliable | 궤적 점 (map 으로 변환해 저장) |
| `/scan` | sensor_msgs/LaserScan | **best_effort** (sensor data) | 스캔 점 |
| `/particle_cloud` | nav2_msgs/ParticleCloud | **best_effort** | 파티클 (`--no-particles` 면 구독 안 함) |
| `/amcl_pose` | geometry_msgs/PoseWithCovarianceStamped | reliable, **transient_local** | 공분산(위치 신뢰도 표시) |
| `/plan` | nav_msgs/Path | reliable | 주행 경로 (map 프레임) |
| `/map` | nav_msgs/OccupancyGrid | reliable, **transient_local** | 파일 지도와 크기·원점이 다르면 경고만 표시 |
| `/abo/state`, `/abo/status`, `/abo/command`, `/abo/pick_request` | std_msgs/String | reliable | 미션·입출력 |
| `/abo/pick_done` | std_msgs/Bool | reliable | 입출력 |

### 좌표 계산

- 로봇 자세(map) = `map→odom` ∘ `odom→base_link`. 둘 중 하나라도 없으면 자세 없음.
- 스캔 점: `range_min ≤ r ≤ range_max` 이고 유한한 빔만. `θ = angle_min + i·angle_increment`,
  base 좌표 `(r cosθ, r sinθ)` → lidar_link 오프셋 적용 → 스캔 수신 시점의 로봇 자세로 map 변환.
- 픽셀: `px = (x − origin_x)/res`, `py = height − (y − origin_y)/res` (pgm 첫 행이 y 최대).
- odom 궤적: `/odom` 을 받을 때마다 현재 `map→odom` 으로 변환해 저장(최대 600점, 5 cm 미만 이동은 건너뜀).
  새 미션(`/abo/command` 에 fetch 수신) 시작 시 비운다.

## 스냅샷 JSON (`/api/events`, 5 Hz)

```json
{
  "t": 1789350302.9,
  "pose":      {"x": 0.22, "y": 0.32, "yaw": -0.30, "age": 0.04, "cov_xy": 0.012},
  "scan":      {"pts": [[x, y], ...], "age": 0.15},
  "particles": {"pts": [[x, y], ...], "n": 1001, "age": 12.3},
  "plan":      {"pts": [[x, y], ...], "age": 3.1},
  "trail":     [[x, y], ...],
  "mission":   {"state": "moving", "status": "가는 중이에요. 0.8 m 남았어요.",
                "command": "fetch center color:green", "elapsed": 31.0,
                "stages": [{"key": "drive", "label": "Nav2 주행 home → center", "state": "active", "sec": 31.0,
                            "start_at": 1789350271.9, "note": "가는 중이에요. 0.8 m 남았어요."}, ...],
                "outcome": null, "received_at": 1789350270.8, "headline": "초록색 약을 향해 가는 중"},
  "stop":      {"latched": false, "since": null, "sent": 0, "last_result": null},
  "pick":      {"reachable": true, "status": {...워커 /status 그대로...}, "age": 0.2},
  "topics":    {"/scan": {"dir": "in", "rate": 6.0, "age": 0.15, "last": "720빔 · 유효 407"}, ...},
  "warnings":  ["/map 크기가 파일 지도와 다름"]
}
```
나이(`age`)는 노트북이 **받은 시각** 기준(로봇·노트북 시계 차이에 영향받지 않게). 한 번도 안 받았으면 `null`.

## 미션 단계 (브리지 `/abo/state` 로 판정)

| 단계 | 시작 조건 | 끝 조건 |
|---|---|---|
| Nav2 주행 home → 목적지 | fetch 명령 후 첫 `moving` | `arrived` 또는 `picking` |
| 집기 (호스트 기동 포함) | `picking` | `/abo/pick_done` (true 면 완료, false 면 실패), 못 받았으면 `returning`(완료로 간주) 또는 `failed` |
| 복귀 목적지 → home | `returning` | `done` / `failed` |

브리지는 집기 실패·집기 시간초과에도 `returning` 으로 복귀한 뒤 `failed`("돌아왔어요. 물건은 집지 못했어요.")로 끝낸다.
그래서 집기가 실패(`pick_done false`)한 미션의 복귀 중 `failed` 는 **복귀 완료** + 미션 결과(`outcome`) `failed` 로 표시한다.
`done` 은 진행 중 단계 완료 + `outcome` `done`. 그 밖의 `rejected`/`canceled`/`failed` 는 진행 중 단계를 실패로 표시하고
`outcome` `failed`, `/abo/status` 문장을 그대로 보여준다.
미션 진행 중(시작됐고 안 끝남) 받은 fetch 는 단계·목표를 초기화하지 않고 `command` 만 기록한다(브리지는 같은 fetch 는 무시,
다른 fetch 는 `rejected` 로 답한다). 이미 단계가 시작된 미션에 온 `rejected` 는 그 새 fetch 에 대한 답이므로 미션을 끝내지 않는다.
브리지는 위치 다듬기를 별도 state 로 내지 않으므로 단계로 나누지 않는다(문장에만 나온다).
정지 유지 중 브리지가 내는 `idle`("가고 있지 않아요")은 대시보드가 보낸 `stop` 의 응답이므로 단계를 바꾸지 않는다.

단계마다 `start_at`(시작 epoch)과 `note`(그 단계가 진행 중일 때 마지막으로 받은 `/abo/status`)를 남긴다 — 단계가 바뀌어도
앞 단계의 문장은 그 단계에 남는다. 미션에는 `received_at`(fetch 받은 시각)과 `headline` 을 싣는다.
`headline` 은 에이보 웹앱과 같은 문장이다(`mission_words.py`): 미션 없음 "미션 없음 — fetch 명령을 기다리는 중",
fetch 후 상태 전 "{c} 약을 가져오라고 르키위에게 전달했어요", moving "{c} 약을 향해 가는 중", arrived "{c} 약 앞에 도착했어요",
picking "{c} 약을 집는 중", returning "{c} 약을 가지고 돌아오는 중", done "{c} 약을 가져왔어요", failed "{c} 약을 가져오지 못했어요",
rejected "르키위가 요청을 거절했어요", canceled "르키위가 멈췄어요", idle "르키위 대기 중" (`{c}` 빨간색/파란색/초록색, 색 없으면 "약").
집기(또는 returning) 뒤의 `moving` 은 복귀 주행이므로 returning 문장으로 본다. 진행 중 미션에 온 `rejected` 는 문장을 바꾸지 않는다.

## 화면 배치 (큰 벽 모니터, 스크롤 없이 한 화면)

화면 폭 1280px 이상: 뷰포트 높이를 꽉 채우는 grid — 헤더(auto) / 본문(1fr) / 토픽 요약 한 줄(auto).

- **헤더**: 연결 · 상태 칩 · **목표 배지**(약통 색으로 꽉 찬 배지, 흰 굵은 글씨 `--fs-xl`, `● 파란색 약 · center`;
  색은 에이보 웹앱과 같게 red `#D93636` / green `#2E9E4F` / blue `#2F6BD8`, 색 이름 글자를 늘 같이 씀, 목표 없으면 회색 "목표 없음") ·
  경과 시간 · 가−/가+ · 정지.
- **본문 3열** 지도(1.15fr) · 미션 진행(0.85fr) · 카메라(1fr). 각 열은 본문 높이 안에서만 커진다.
  - 지도 canvas 는 칸의 폭·높이 둘 다에 맞춰 53:48 비율로 가운데 그린다. 범례는 아래.
  - 미션 진행: 왼쪽에 목표 색 굵은 띠. 맨 위 `headline`(큰 글씨) + 마지막 `/abo/status`, 그 아래 세로 파이프라인
    `명령 받음 ▼ Nav2 주행 ▼ 집기 ▼ 복귀 ▼ 결과`. 단계마다 아이콘(○ 대기 / ◉ 진행 중 / ✔ 완료 / ✖ 실패)·상태 배지·걸린 시간(m:ss),
    둘째 줄에 시작 시각(HH:MM:SS)과 그 단계의 `note`. 집기 진행 중엔 `시도 n/m · 보라 ratio/thr`(워커 `/status`).
    진행 중 단계는 크게 강조(깜빡임은 reduced-motion 존중). ▼ 연결선은 끝난 구간 초록, 진행 구간 accent, 남은 구간 회색.
    결과: done "✔ 완료 · 전체 m:ss"(초록), failed "✖ 물건은 집지 못했어요 · 전체 m:ss"(집기 실패) 또는 "✖ 미션 실패 · 전체 m:ss"(빨강).
  - 카메라: front / wrist 가 열 높이를 반씩(4:3 contain). 집기 중이면 제목에 `타겟 ● 파란색`. 스트림 `<img>` 가 오류면
    "영상 연결 안 됨 — 다시 시도 중" 자리표시 후 3 s 뒤 재시도.
- **토픽 요약**: `/scan` `/tf` `/odom` `/particle_cloud` `/abo/state` `/abo/pick_done` 칩(주기 또는 나이; `/scan`·`/tf`·`/odom` 은
  2 s 넘으면 경고색). [토픽 자세히] 가 기존 전체 표(`#topics`)를 화면 위 서랍으로 연다(Esc·바깥 클릭·닫기로 닫힘).

1280px 미만은 한 열로 쌓고 스크롤한다(420px 에서 가로 스크롤 없음). 글자 크기는 모두 `--fs-*` 토큰(× `--ui-scale`).
미리보기: `/usr/bin/python3 nav/dashboard/preview_server.py --scenario picking` → http://127.0.0.1:8011 (카메라는 자리표시).

## 오래된 데이터 표시

| 대상 | 기준 | 표시 |
|---|---|---|
| 로봇 자세 | `/tf` 나이 > 1 s | 로봇 아이콘 회색 + "TF 끊김 n초" |
| 스캔 | 나이 > 2 s | 점 숨김, 범례에 "n초 전" |
| 파티클 | 나이 > 2 s | 점은 흐리게 유지, "마지막 갱신 n초 전 (정지 중엔 정상)" |
| 경로 | 미션 단계가 주행이 아니면 | 숨김 |
| 카메라 | 8000 응답 없음 | "집기 단계에서 켜져요" 자리표시 |
| 카메라 | `/status` 의 `frame_age_s` > 2 s | 붉은 테두리 + "카메라 멈춤 n초" (hz 는 멀쩡한데 프레임이 멈추던 문제 대비) |
| SSE 끊김 | 브라우저가 3 s 동안 이벤트 못 받음 | 화면 전체 상단 배너 "대시보드 서버 연결 끊김" |

## 정지

### 확인된 브리지 동작 (PR #8 `abo_nav_bridge.py`, 로봇 배포본과 동일)

- `stop`/`정지`/`멈춰`/`cancel` → `cancel()`. **Nav2 목표 핸들(`gh`)이 있을 때만** 목표를 취소하고
  미션을 지운다("멈췄어요", `canceled`). 없으면 "가고 있지 않아요"(`idle`)만 말하고 **미션을 그대로 둔다.**
- `gh` 가 없는 구간: 집기 중 전체, 위치 다듬기, 그리고 `go()` 가 `moving` 을 말한 뒤 Nav2 가 목표를
  수락(`on_accept`)하기 전까지의 짧은 틈.
- 집기 중 팔을 멈추면 집기 스크립트가 실패로 끝나 어댑터가 `/abo/pick_done false` 를 보내고,
  브리지는 `return_after_pick` → 위치 다듬기 → **home 으로 주행**한다. `pick_done` 이 안 와도
  `pick_timeout`(360 s) 뒤 같은 복귀를 한다.
- 워커 `/estop`(request_abort) 은 현재 팔 자세 유지 + 바퀴 정지 후 연결을 끊는다. 호스트가
  `disable_torque_on_disconnect=false` 라 토크가 남아 **약통은 떨어뜨리지 않는다.**

→ 로봇 코드를 바꾸지 않는 한 **집기 중 한 번의 `stop` 으로는 미션이 끝나지 않는다.** 그래서 "정지 유지" 방식으로 한다.
근본 해결은 브리지 수정이며 팀원에게 요청한다(§관련 이슈).

### 정지 유지 (stop latch)

`POST /api/stop` → 정지 유지 켜짐. `POST /api/stop/release` 로만 풀린다 (새 fetch 명령을 받아도 자동으로 풀지 않는다 — 풀린 줄 모르고 로봇이 움직이면 안 되므로).

켜지는 순간:
1. `POST 127.0.0.1:8000/estop` 을 **늘 시도**한다(캐시된 응답 여부로 건너뛰지 않음; 거부는 즉시, 타임아웃 0.3 s) — 팔 즉시 고정, 바퀴 정지.
   실패하면 "estop 불가 (8000 응답 없음)" / "estop 실패: …" 가 결과 문구에 남는다.
2. `/abo/command "stop"` 1회 — 주행 중이면 여기서 바로 멈춘다

켜져 있는 동안 (0.1 s 타이머, `stop_latch.py` 가 판단):
- 브리지 state 가 `moving`/`returning` 이 **된 시각부터 5 s 동안** 0.5 s 마다 `stop` 재전송.
  `canceled` 를 받으면 그 창을 닫는다. (목표 수락 전에 도착한 `stop` 은 무시되므로 수락될 때까지 반복한다.)
- `picking`/`arrived` 동안은 보내지 않는다 — 보내봐야 `idle` 만 찍혀 상태 표시가 흐려진다.
- 8000 이 다시 응답하기 시작하면(새 집기 시작) 즉시 estop 을 한 번 더 부른다.

한계 (화면·문서에 그대로 적는다): 복귀 주행이 **시작된 뒤에** 취소되므로 수락 지연 + 최대 0.5 s 만큼
로봇이 움직일 수 있다. 그래서 버튼 이름은 "비상정지"가 아니라 **"정지"**, 옆에 "확실한 비상정지는 로봇 전원 스위치"를 적는다.

추가 한계:
- 재전송 창은 브리지 `on_fb` 가 `idle` 응답 뒤에도 주행 피드백으로 `moving` 을 다시 발행해 주는 데 기대어 다시 열린다.
  브리지를 고치다 피드백 state 발행을 없애면 재정지가 깨진다.
- 직전 피드백 뒤 약 3 s 안에 다시 fetch 하면(피드백 state 가 바뀌지 않아 창이 늦게 열려) 재정지가 최대 3 s 늦을 수 있다.
- 정지 유지 중 집기가 시작되면 워커 웹이 뜨고 8000 응답 상승 시점의 estop 이 닿을 때까지 팔이 움직일 수 있다.

화면: 정지 유지 중엔 상단 붉은 띠 "정지 유지 중 — 로봇이 움직이려 하면 계속 멈춥니다 [해제]", 보낸 `stop` 횟수와 마지막 브리지 응답 표시.
그 외 제어 버튼(시작·일시정지·처음 자세로)은 v1 에 두지 않는다 — 미션 중 수동 조작이 브리지 상태와 어긋나기 때문.

## 집기 워커 변경 (카메라 스트림 + 상태 추가)

1. **`scripts/pick_worker_cycle.py --web-port N [--web-host H]`** (기본: 끔 = 지금과 동일)
   - 지정하면 워커 생성 직후 `motion/webui/app.py` 의 `create_app(worker)` 를 uvicorn 데몬 스레드로 띄운다.
   - **기본 바인드 `127.0.0.1`.** 8000 에는 기존 웹 UI 의 시작·재시작 엔드포인트도 같이 열리므로, 미션 중에
     다른 기기에서 누르지 못하게 한다. 다른 기기로 대시보드를 볼 때만 `--web-host 0.0.0.0`.
   - 스크립트가 끝나면 프로세스와 함께 사라진다(별도 정리 없음). 포트가 이미 쓰이면 경고만 하고 집기는 계속한다 —
     카메라 표시 실패가 미션 실패가 되면 안 된다.
   - 어댑터는 이미 `--` 뒤 인자를 그대로 넘기므로 어댑터 수정 없이 `-- --web-port 8000` 으로 켠다.
2. **워커 `/status` 추가 필드** (`motion/services/pickplace/headless_worker.py`, 기존 필드는 그대로)
   - `purple`: `{"front": {"ratio": 0.27, "thr": 0.23}, "wrist": {...}}` — 잡기 확인 때 계산한 값
   - `retry_depth`: 현재 높이 재시도 깊이(0~0.06)
   - `frame_age_s`: `{"front": 0.03, "wrist": 0.03}` — 카메라 이미지 내용이 마지막으로 **바뀐** 뒤 지난 시간
     (축소 이미지 해시 비교, 뷰당 매 루프 1회)

### 브랜치

PR #8 이 아직 머지 전이라 `scripts/pick_worker_cycle.py` 가 main 에 없다.
작업 브랜치 `feature/nav-dashboard` 를 main 에서 만들고, **첫 커밋에 PR #8 의 `pick_worker_cycle.py` 와
테스트를 바이트 그대로** 넣는다. `--web-port` 는 그 위의 별도 커밋. 로봇 배포는 하지 않는다.
주의: 팀원이 머지 전에 PR #8 의 이 파일을 고치면 충돌한다 — 착수 전에 팀원에게 알린다.

## 실행

```bash
# 노트북 (셸 기본 ROS_DOMAIN_ID 가 77 이라 스크립트가 42 로 고정한다)
nav/dashboard/run_dashboard.sh --robot-ip 223.194.139.15 --port 8001 [--no-particles]
# 브라우저: http://localhost:8001
# 집기 어댑터는 기존 명령 끝에  -- ... --web-port 8000  추가
```

## 오류 처리

- ROS 데이터가 한 번도 없으면 해당 레이어/토픽을 "데이터 없음"으로 표시하고 계속 돈다.
- 콜백 예외는 잡아서 로그(같은 오류는 5초에 한 번), 노드는 죽지 않는다. 화면 `warnings` 는 `/map` 불일치에만 쓴다.
- 8000 조회는 별도 스레드가 5 Hz, 타임아웃 0.3 s 로 읽어 캐시한다 — SSE 주기를 막지 않는다.
- 정지 요청 중 8000 estop 실패는 응답·화면에 표시하되 `stop` 발행은 계속한다.
- 대시보드는 `/abo/command "stop"` 외에 아무것도 발행하지 않는다.

## 테스트

- `map_geometry`: 원점·해상도로 픽셀 변환, y 뒤집기, 자세 합성(90° 회전 사례), 스캔 유효빔 거르기·변환, 파티클 추림 개수
- `mission_timeline`: 성공 흐름(moving→picking→returning→done) 단계 시간, 실패/거절/취소, 새 fetch 에서 초기화, 정지 유지 중 `idle` 무시
- `topic_stats`: 주기 계산, 나이, 한 번도 안 받음 → null
- `stop_latch`: 켜는 순간 estop+stop 1회 / `picking` 중 재전송 없음 / `returning` 이 되면 0.5 s 간격 재전송, 5 s 뒤 멈춤 /
  `canceled` 받으면 즉시 멈춤 / 해제 전엔 새 fetch 에도 유지 / 8000 이 새로 응답하면 estop 재호출
- `dashboard_state.snapshot`: 오래됨 판정 경계값, JSON 직렬화 가능, `/abo/status` 가 진행 중 단계 `note` 로
- `mission_words.headline`: 모든 상태, 색 없음, 집기 뒤 moving → 복귀 문장
- `mission_timeline` 추가: 단계별 `start_at`/`note` 가 단계 전환 뒤에도 남음, `received_at`, `headline`
- `preview_server.build_state`: 시나리오마다 기대 단계 상태·집기 워커 상태
- 정적 검사: 1280px 이상에서 뷰포트 높이 grid, 새 id(`headline` `target-badge` `topics-summary` `topics-toggle`), 3색 hex, 글자 토큰, 정지 버튼 연결 순서
- `web_app`: TestClient 로 `/api/map_info`, `/map.png`, `/api/stop`·`/api/stop/release`(가짜 pick_client 호출 순서), SSE 제너레이터 `next()` 1회
- `pick_worker_cycle --web-port`: 가짜 워커로 앱 팩토리 호출, 기본 바인드 127.0.0.1, 포트 사용 중일 때 집기 계속
- `headless_worker` 상태 필드: `purple`/`retry_depth`/`frame_age_s` (같은 프레임 반복 시 나이 증가), 기존 단독 웹 UI 테스트 통과
- 실기기 확인(수동): 정지 상태에서 지도·스캔 표시 → 주행 명령 후 경로·궤적·파티클 갱신 → 집기 단계 카메라 → 복귀 완료 단계 표시
  → 주행 중 [정지] 로 멈춤 확인. 집기 중 [정지] 는 로봇을 손으로 잡을 준비를 한 상태에서만 시험한다.

## 관련 이슈 (이 설계 범위 밖, 기록만)

- **브리지 정지 동작 (팀원 요청)**: 집기 중·위치 다듬기 중·목표 수락 전에도 `stop` 이 미션을 끝내도록
  (`host_stop` → `set_bus(True)` → 미션 지움 → `canceled`, 복귀 안 함, 이후 늦게 온 `pick_done` 무시).
  반영·배포되면 대시보드의 정지 유지 재전송은 그대로 둬도 무해하다(보낼 일이 없어진다).
- PR #8 `pick_worker_cycle.py` 는 `PickPlaceConfig()` 기본값을 써서 웹 UI 에서 맞춘 값(retry_overreach 0,
  target 290px, front 보라색 0.23 등)이 빠진다. 지금은 노트북 임시 래퍼로 넣고 있다 — PR #8/#9 의
  집기 스크립트 옵션(`--pick-script` vs `--script`) 통합 때 설정 전달 방식도 같이 정해야 한다.
