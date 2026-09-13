# 에이보 발화 → 색상 토픽 → 노트북 → LeKiwi

## 흐름

```
에이보 파이  stt_node (마이크)          ──/audio_final──▶  노트북  whisper_transcribe → /user_input
                                                          노트북  dialogue_node (LLM pickup_medicine 도구)
                                                                   └▶ pickup/medicine/<red|blue|green>  data="<색>"   [도메인 77]
                                                          노트북  nav/mission/medicine_relay.py
                                                                   └▶ /abo/command "fetch center color:<색>"          [도메인 42]
LeKiwi       abo_nav_bridge.py → 이동 → /abo/pick_request ──▶ 노트북 pick_adapter.py
```

- 에이보 코드: `abo/ros_dialogue/` (원본 `roboseasy-members/A-Bo_project` `feature/pickup-medicine-color`)
- 에이보와 LeKiwi 는 도메인이 달라(77/42) 노트북의 `medicine_relay.py` 가 한 프로세스에서 둘 다 참여해 넘긴다.

## 1단계 시험: 발화 인식 → 색상 → 노트북 도착

에이보(파이 `pi_bringup`, 노트북 `laptop_bringup`)를 띄운 상태에서 노트북에서:

```bash
bash nav/shell/abo_color_test.sh                    # 마이크로 "빨간약 가져다줘"
bash nav/shell/abo_color_test.sh "빨간약 가져다줘"   # 마이크 없이 문장만
```

`수신: pickup/medicine/red` 가 찍히면 성공. LeKiwi 로는 보내지 않는다.

안 찍힐 때 어디서 끊겼는지:

| 확인 (도메인 77) | 안 되면 |
|---|---|
| `ros2 topic echo /user_input` 에 말한 문장이 나오나 | 파이 마이크/stt 또는 파이↔노트북 통신 (아래 네트워크) |
| dialogue_node 로그에 `약통 픽업 색상 발행` 이 나오나 | LLM 이 도구를 안 부름 — 색을 분명히 말했는지, 세션이 열렸는지(첫 대면이면 이름부터 물음) |
| `ros2 topic echo /pickup/medicine/red` 에 나오나 | 나오는데 relay 가 못 받으면 relay 쪽 도메인 확인 |

## 2단계: LeKiwi 까지

노트북에서 `python3 nav/mission/medicine_relay.py` (`--dry-run` 없이), LeKiwi 에서 `ros2 topic echo /abo/command` (도메인 42).

## 서로 다른 와이파이일 때

`nav/shell/ros_peers.sh` 의 세 IP 를 채워 커밋한다. LeKiwi 는 `lekiwi_profile.sh` 가 자동으로 불러오고
(배포 목록에 포함), 노트북은 `abo_color_test.sh` 가 불러온다. 에이보 파이와 노트북의 에이보 노드들은
`~/.bashrc` 에 `source <저장소>/nav/shell/ros_peers.sh` 한 줄이 필요하다.

- 서로 ping 이 안 되면 피어 설정으로도 안 된다(NAT/AP 격리) → Tailscale 등 VPN IP 사용.
- 설정을 바꾼 뒤엔 `ros2 daemon stop`.

## 에이보 대답을 터미널 글자로 보기 (스피커가 안 나올 때)

노트북에서 (에이보 도메인 77, 받기만 한다):

```bash
source /opt/ros/jazzy/setup.bash
source nav/shell/ros_peers.sh          # 같은 와이파이가 아니면
python3 nav/tools/abo_console.py
```

```
[16:54:31] 🗣  나     : 빨간약 가져다줘
[16:54:31] 🤖 에이보 : 네, 빨간 약 가지러 갈게요.
                    잠시만 기다려 주세요.
[16:54:32] 📦 토픽   : /pickup/medicine/red  (data: 'red')
[16:54:34] 🚗 르키위 : 빨간색 약을 향해 가는 중
```

`🤖 에이보` 줄이 TTS 가 읽었어야 할 문장이다 (`/llm_response`).

## 르키위가 지금 무엇을 하는지 — 에이보 동반 웹앱 카드

`medicine_relay.py` 는 명령을 넘기는 것과 반대 방향으로, 르키위의 `/abo/state`·`/abo/status`
(도메인 42)를 받아 에이보 쪽 `/lekiwi/mission_state` (도메인 77, JSON)로 돌려보낸다.
에이보 동반 웹앱(`http://<에이보 IP>:8000/`)의 **🚗 르키위 심부름** 카드와 위 터미널 뷰어가
이걸 보여준다.

| `/abo/state` | 카드 문장 (빨강 예) |
|---|---|
| (중계기가 명령을 넘긴 순간) `sent` | 빨간색 약을 가져오라고 르키위에게 전달했어요 |
| `moving` | 빨간색 약을 향해 가는 중 |
| `arrived` | 빨간색 약 앞에 도착했어요 |
| `picking` | 빨간색 약을 집는 중 |
| `returning` | 빨간색 약을 가지고 돌아오는 중 |
| `done` | 빨간색 약을 가져왔어요 |
| `failed` / `rejected` / `canceled` | 빨간색 약을 가져오지 못했어요 / 르키위가 요청을 거절했어요 / 르키위가 멈췄어요 |

- 문구는 `nav/mission/mission_text.py` 한 곳에서 바꾼다 (웹앱·뷰어는 받은 문장을 그대로 표시).
- 웹앱 쪽 코드는 A-Bo_project 의 `companion_bridge_node.py` + `lekiwi_status.py` + `web/companion/index.html`.
  에이보 파이에 반영하려면 그 브랜치를 파이의 `robot_ws` 에 올리고 `colcon build` 후 `companion_bridge` 재시작.
- 중계기가 안 떠 있으면 카드에는 토픽과 "빨간색 약 요청을 받았어요"까지만 보인다.
