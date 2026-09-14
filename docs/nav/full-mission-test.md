# 르키위 약통 심부름 전체 테스트

에이보에게 **말(또는 웹 버튼)로 약통을 요청**하면 르키위가 **자율주행 → YOLO 로 요청한 색 약통 집기 → 출발 자리로 복귀**까지 하고, 에이보가 단계마다 말로 알려주는 전체 흐름과 실행 방법.

- 실기기 기록: 2026-09-13 lekiwi01, 지도 `map_0912_1654`, 목적지 `center` — 웹 버튼으로 **2회 성공** (집기 49초 / 80초, 전체 약 2분 25초)
- 음성 안내·3색 모델은 그 뒤에 붙인 것이다 (아래 "코드 위치", "남은 일" 참고)

---

## 0. 코드 위치

> ⚠️ 이 문서의 흐름에 필요한 코드는 **아직 `main` 에 없다.** 아래 PR 이 머지돼야 한다.

| 무엇 | 저장소 / PR |
|---|---|
| 에이보 색상 토픽 → 노트북 중계 → 르키위 (`nav/mission/medicine_relay.py` 기본) | 이 저장소 #7 |
| 중계기가 르키위 상태·집기 결과(`picked`)를 에이보로 되돌림, `abo/ros_dialogue/` 발화 로직 파일 | 이 저장소 #9 (#7 위) |
| 진짜 YOLO 집기 실행기 `scripts/pick_worker_cycle.py`, 브리지 실패 시 복귀·호스트 재시도, `pick_timeout`, 집기 호스트 환경 정리, 어댑터 `--pick-script` | 이 저장소 #8 |
| 에이보가 하는 말(요청 대답·진행 안내)을 LLM 없이 고정 문장으로, 웹 르키위 탭·상태 카드, `voice_input_enabled` / `stt:=false` | roboseasy-members/A-Bo_project #31 (#30 위) |

머지 순서: **#7 → #9**, #8 은 따로 (**#7(#9) 과 #8 은 `nav/mission/pick_adapter.py` 에서 충돌** — `--script` 와 `--pick-script`), 에이보 **#30 → #31**.
로봇에는 #8 버전이 배포돼 있다 — 머지 전까지 `main` 으로 로봇에 재배포하지 말 것.

## 1. 한눈에 보기

```
사용자 "하이 에이보" … "초록색 약 가져다줘"          (또는 에이보 웹 🧪 르키위 탭의 초록 물약 버튼)
   │ 에이보 파이 stt_node → 노트북 whisper_transcribe → /user_input
   ▼
[노트북 dialogue_node]  LLM 은 pickup_medicine(color="green") 으로 색만 고른다
   │  pickup/medicine/green 발행 + 🔊 "네, 알겠어요! 초록색 약을 가져올게요."   (고정 문장)
   ▼                                                               (에이보 도메인 77)
[노트북 medicine_relay]  77 → 42 로 옮김
   │  /abo/command "fetch center color:green"                     (르키위 도메인 42)
   ▼
[르키위 abo_nav_bridge]
  ① 지금 자리를 출발 자리로 기억 → Nav2 로 center 까지         🔊 "주행 시작할게요."
  ② 도착 → 라이다로 위치 다듬기                                 🔊 "도착했어요. 약을 집을게요."
  ③ 바퀴 노드에게 서보 버스 양보받기 (set_bus false)
  ④ ZMQ 호스트 켜기 (카메라 2대 + 팔/바퀴 모터)
   │  /abo/pick_request "center:green"
   ▼
[노트북 pick_adapter] → scripts/pick_worker_cycle.py (motion 의 PickPlaceHeadlessWorker 그대로)
  ⑤ ZMQ 로 로봇에 붙어 YOLO 목표 클래스 green_pill_bottle 로 접근·집기
  ⑥ 잡기 확인(GRASP_OK) → 결과 파일 → /abo/pick_done true      🔊 "약 집기 성공했어요. 가져다 드릴게요."
   ▼
[르키위 abo_nav_bridge]
  ⑦ ZMQ 호스트 끄기 (팔 토크 유지 → 약통 안 떨어뜨림)
  ⑧ 서보 버스 돌려주기 (set_bus true) → 라이다로 위치 다시 맞추기
  ⑨ 출발 자리로 복귀                                            🔊 "약을 가져왔어요."
```

🔊 문장은 전부 에이보 `dialogue_node` 가 코드로 정한 고정 문장이다. 진행 안내는 중계기가 되돌려주는
`/lekiwi/mission_state` (`state`, `picked`) 를 보고 미션마다 단계별로 한 번씩 말한다.
실패하면 "약 집기에 실패했어요. 제자리로 돌아갈게요." → "돌아왔지만 약은 가져오지 못했어요." — 집기 실패·시간 초과·호스트 기동 실패 모두 출발 자리로 돌아온다.

## 2. 실측 시간 (2026-09-13 18:52, 웹 버튼)

| 단계 | 걸린 시간 |
|---|---|
| Nav2 주행 (출발 자리 → center) | 29초 |
| 위치 다듬기 (7.4 cm 보정) | 5초 |
| ZMQ 호스트 기동 (카메라·모터 연결) | 10초 |
| **YOLO 집기** → `GRASP_OK` | 49초 |
| 호스트 끄기 + 버스 돌려주기 | 3초 |
| 위치 다시 맞추기 (4.3 cm, 9.6°) | 10초 |
| 출발 자리로 복귀 + 위치 다듬기 (2.2 cm) | 33초 |
| **전체** | **약 2분 25초** |

## 3. 왜 이렇게 나눴나

- **서보 버스(`/dev/ttyACM0`)는 한 번에 한 프로그램만.** 주행은 바퀴 노드, 집기는 ZMQ 호스트. 도착하면 넘기고 끝나면 돌려받는다.
- **ZMQ 호스트는 집을 때만 켠다.** 주행 중에 켜져 있으면 버스를 뺏어 Nav2 가 멈춘다.
- **팔 토크 유지** (`--robot.disable_torque_on_disconnect=false`) — 호스트를 꺼도 약통을 쥔 채 복귀.
- **위치를 여러 번 다시 맞춘다.** 집는 동안 오도메트리가 끊기고 워커가 바퀴로 움직인 만큼을 라이다로 되찾는다.
- **YOLO 는 노트북에서.** 로봇(Pi)은 카메라 영상만 보내고 추론·판단은 노트북이 한다.
- **에이보의 말은 LLM 이 만들지 않는다.** LLM 은 음성에서 색만 고르고, 사실을 알리는 문장은 실제 상태를 받은 코드가 말한다 (사실과 다른 "가져왔어요" 방지).

## 4. 준비

| 무엇 | 어디 |
|---|---|
| 이 저장소 | `~/lekiwi-pill-pickup` (`pick_adapter.py` 의 `--repo` 기본값) |
| YOLO 모델 (3색) | `~/YOLO/best_3color.pt` — 클래스 `green_pill_bottle`, `red_pill_bottle`, `blue_pill_bottle` (2026-09-13 16:44 학습, `dataset_rgb`). 가중치는 저장소에 넣지 않는다 — 팀 공유본을 이 경로에 둔다 |
| 집기 자세 | `~/.PhysicalLabs/pickplace/lekiwi01/poses/` (`pre_pick`, `grasp`, `grasp_closed`, `home`) |
| lerobot 환경 | 예: `~/miniconda3/envs/lerobot/bin/python` (lerobot, ultralytics) |
| 에이보 | 노트북 `dialogue_node`·`whisper_transcribe`, 파이 `stt`·`tts`·`face_*`·`arm` (에이보 웹 관리자의 "전체 시작") |
| 르키위 | 로봇 홈에 배포된 `start_nav_pick.sh`, `start_pick_host.sh`, `abo_nav_bridge.py`, 지도·`waypoints.yaml` |

**약통을 둘 곳:** 르키위는 `waypoints.yaml` 의 `center` (지도 x 1.10, y −0.10, 정면 0°) 까지 가서 선다. 약통은 거기서 **정면 30~60 cm, 앞 카메라에 보이는 범위**에 있어야 한다 — 방 안을 찾아다니는 기능은 없다.

**르키위 위치:** 주행 스택을 켤 때 자동 위치 맞춤은 지도 원점(시작 표시) 근처 ±40 cm·±15° 만 찾는다. 다른 곳에서 켰으면 `refine_pose.py` 로 넓게 확인한다 (6장). 미션의 복귀 지점은 **요청을 받은 그 자리**다.

## 5. 실행

### 르키위 (로봇, 도메인 42)
```bash
~/start_nav_pick.sh      # 로그 ~/nav_pick.log
# = ros2 launch ~/launch/lekiwi_nav.launch.py map:=/home/roboseasy/maps/map_0912_1654.yaml pick_timeout:=360.0
```
센서 → Nav2(20초) → 지도 재발행(45초) → 자동 위치 맞춤(52초) → 브리지(58초). 로그에 `명령 토픽 /abo/command` 가 뜨면 준비 완료.

### 노트북
```bash
cd ~/lekiwi-pill-pickup
source /opt/ros/jazzy/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

# 중계기 (에이보 77 ↔ 르키위 42) -- conda 의 python3 에는 yaml 이 없어 /usr/bin/python3
ROS_STATIC_PEERS="<에이보 IP>;<르키위 IP>" \
  /usr/bin/python3 -u nav/mission/medicine_relay.py --abo-domain 77 --lekiwi-domain 42

# 집기 어댑터 (진짜 YOLO 집기)
ROS_DOMAIN_ID=42 ROS_STATIC_PEERS="<르키위 IP>" \
  /usr/bin/python3 -u nav/mission/pick_adapter.py \
    --pick-script scripts/pick_worker_cycle.py \
    --python ~/miniconda3/envs/lerobot/bin/python \
    --timeout 330 --map center=green \
    --model ~/YOLO/best_3color.pt \
    --poses-dir ~/.PhysicalLabs/pickplace/lekiwi01/poses \
    --remote-ip <르키위 IP> \
    -- --max-seconds 300 --max-pick-attempts 5

# 기록기 (권장) -- 에이보 말·르키위 상태·집기 결과를 ~/pickplace_logs/<날짜_시각>/topics.log 로
ROS_STATIC_PEERS="<에이보 IP>;<르키위 IP>" /usr/bin/python3 -u nav/tools/mission_topic_logger.py
```
- `pick_adapter.py` 는 집기 스크립트를 실행할 때 ROS 경로(`/opt/ros/...`, colcon 작업공간)를 `PYTHONPATH`·`LD_LIBRARY_PATH` 에서 빼고 넘긴다(#8). 그래서 `--python` 에 lerobot 환경 파이썬을 바로 줘도 된다.
- `--map center=green` 은 요청에 색이 없을 때만 쓰인다. 음성·버튼 요청은 `center:<색>` 으로 색이 실려 온다.

### 에이보
- 에이보 웹 `http://<에이보 IP>:8000` → 관리자 → **전체 시작** (파이 노드). 노트북 `dialogue_node`·`whisper_transcribe` 는 에이보 저장소 워크스페이스에서 도메인 77 로 켠다.
- 얼굴이 등록되지 않은 채 시험하면 에이보가 이름부터 묻는다 — 시험 중에는 `ros2 topic pub -r 1 /user_id std_msgs/msg/String "{data: tester}"` 로 건너뛸 수 있다.
- 시끄러운 날: 파이 `pi_bringup.launch.py stt:=false`, 노트북 `laptop_bringup.launch.py voice_input_enabled:=false` → 웹 버튼과 진행 안내만 동작하고 LLM 은 불리지 않는다.

### 요청
- 음성: **"하이 에이보"** 를 한 번 또렷하게 → 인사 뒤 **"초록색 약 가져다줘"**. 웨이크워드 2차 확인은 노트북 Whisper 가 건당 약 2초 걸려서, 소음으로 1차 감지가 몰리면 확인 대기열이 차 진짜 호출이 버려질 수 있다.
- 버튼: 에이보 웹 🧪 르키위 탭 → 물약 버튼.

### 제한시간 (안쪽이 먼저 멈춰야 팔을 정리하고 실패를 보고한다)

| 어디 | 값 |
|---|---|
| 집기 워커 `--max-seconds` | 300초 |
| 노트북 어댑터 `--timeout` | 330초 |
| 로봇 브리지 `pick_timeout` | 360초 |
| 집기 시도 `--max-pick-attempts` | 5번 (각 시도 안에서 다시 잡기 최대 5번) |

## 6. 문제와 조치

| 증상 | 원인 | 조치 |
|---|---|---|
| "팔 제어를 시작하지 못했어요" — 호스트가 앞 카메라를 여는 순간 USB 가 끊김 | 브리지(ros2 launch)가 띄운 호스트만 ROS `PYTHONPATH`/`LD_LIBRARY_PATH` 를 물려받음 (전원 저하 기록 없음) | `start_pick_host.sh` 가 두 변수를 지우고 실행, 브리지는 호스트 기동 실패 시 3초 뒤 1회 재시도 (#8) |
| 주행 스택을 다시 켰는데 옛 바퀴 노드·브리지가 남아 새 스택과 겹침 | 백그라운드 실행이라 SIGINT 를 무시 | PID 를 확인해 SIGTERM, 남으면 SIGKILL. 서보 버스·라이다 포트를 아무도 안 잡은 걸 확인하고 켠다 |
| 라이다 노드가 반복해서 죽음 ("device reports readiness to read but returned no data") | 라이다 USB 가 다시 잡힘 / 포트 이중 사용 | 스택을 완전히 내리고 `/dev/ttyUSB0` 사용 프로세스가 없을 때 다시 켠다 |
| 켤 때 자동 위치 맞춤 보정이 수십 cm·수십 도 | 시작 표시가 아닌 곳에서 스택을 켬 | 로봇을 움직이지 말고 `refine_pose.py <map.yaml> --range 0.8 --yaw 40` (적용 없이) 로 품질 확인. 좋으면 `--publish`, 시작 표시 위라면 `--at 0 0 0 --yaw 30 --publish` |
| 호스트를 켤 때 팔이 튈 수 있음 | 서보에 예전 목표 위치가 남음 | 수동 시험에서는 호스트 켜기 전 팔 서보 Goal_Position 을 Present_Position 으로 맞춘다 |
| 에이보가 요청 전에 이름을 물음 | 카메라 앞에 등록된 얼굴이 없음(guest 온보딩) | 등록된 얼굴로 서거나 시험용 `/user_id tester` 발행 |

## 7. 남은 일

- [ ] **3색 모델 실제 카메라 확인.** 저장 사진 시험에서 파랑 0.55 로 구분했지만 **초록이 0.26** — 집기 기준값 `--conf` 기본 0.4 보다 낮아 초록을 못 찾을 수 있다. 실제 앞 카메라로 색별 신뢰도를 보고 기준값을 정할 것
- [ ] **"집기 실패 → 출발 자리 복귀" 경로 실기기 검증** (지금까지는 성공만 해서 안 돌았다)
- [ ] #7(#9) 과 #8 의 `pick_adapter.py` 충돌 정리 후 머지
- [ ] 약통이 `center` 에서 안 보이는 곳에 있을 때 찾아가는 기능 없음
