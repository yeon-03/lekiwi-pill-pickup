# LeKiwi 약통 픽업 데모

공학경진대회 제출용 데모 — 음성 명령으로 LeKiwi가 책상 위 약통을 찾아 집는 파이프라인.

에이보(별도 저장소 `robot_ws`)와 SSH로 연동된다. 설계 문서:
`docs/superpowers/specs/2026-08-12-lekiwi-pill-pickup-design.md`

## 설치

```bash
python3 -m venv venv && source venv/bin/activate
pip install -r requirements.txt
```

`requirements.txt`의 코어(numpy/opencv/scipy)만 있으면 테스트와 정지영상 분석이 된다.
실제 로봇을 붙이려면 `lerobot`이 추가로 필요하다(LeKiwi 라즈베리파이의 `~/lerobot_venv`와
같은 버전). YOLO 실험 스크립트는 `ultralytics`가 있어야 한다. 둘 다 `requirements.txt`에
주석으로 표시돼 있다.

## 구성

집기 파이프라인(색상 검출 기반, YOLO 아님):

- `scripts/pick_cycle.py` — 집기 1사이클 엔트리(`--color`, 결과 JSON `--result-file`)
- `scripts/color_detect.py` — HSV 임계값 + 형태학 연산 + 윤곽/연결요소 분석 기반 색상 블롭 검출
- `scripts/approach_board.py` — 2단계 시각 서보 접근(베이스캠 색상 → 고정캠 흰 마커) + 프레임
  가장자리에서 실패 시 개루프 넛지(`approach_state()` 포함)
- `scripts/align_and_grasp.py` — 정렬 후 파지
- `scripts/robot_bridge.py` / `scripts/robot_link.py` — LeKiwi ZMQ 단일 연결 관리 + Unix 소켓
  멀티플렉싱(`RobotLink`)
- `scripts/fixed_cam_server.py` — 고정캠(상단 부감) 프레임 제공
- `scripts/parallax.py`, `src/lekiwi_pill_pickup/pixel_to_base.py`,
  `src/lekiwi_pill_pickup/sideways_nudge.py`, `src/lekiwi_pill_pickup/undistort.py` —
  시차 보정·픽셀↔베이스 좌표 변환·개루프 넛지 계획·왜곡 보정
- `src/lekiwi_pill_pickup/pick_result.py` — 집기 판정 결과 파일 읽기/쓰기(에이보 연동용)
- `config/*.json` — 캘리브레이션 값(시차 계수, 넛지 펄스 수, 베이스 부호)
- `scripts/calibrate_*.py`, `scripts/teach_pose.py`, `scripts/goto_pose.py` — 캘리브레이션·포즈 도구
- `docs/HANDOFF_2026-08-27.md`, `docs/grasp_plan_white_marker.md` — 인수인계·파지 계획(실측 결과 포함)

그 외 `scripts/`의 `*_step1.py`, `*_loop.py`, `spike_*.sh`, `live_view_*` 등은 개발 중
실험/디버그 스크립트다.

## 테스트

```bash
pytest -q
```

## 카메라 미리보기 켜기/끄기

카메라가 두 종류다 — 헷갈리지 않게 구분할 것:
- **노트북 USB 웹캠**(`camera0`) — 노트북에 직접 꽂혀있는 별도 카메라, 로봇 위/앞에 물리적으로만 놓아둠
- **르키위 자체 카메라**(`front`=베이스캠, `wrist`=손목캠) — 로봇에 내장, 라즈베리파이(`lekiwi_host`)를 거쳐야 함

### 1) 노트북 USB 웹캠 (camera0)

```bash
cd ~/lekiwi-pill-pickup

# 색상 검출로 켜기 (기본)
./scripts/run_camera.sh

# YOLO(bottle 검출) + 색상판정으로 켜기
./scripts/run_camera.sh --yolo

# 카메라 위치를 바꿔서 노란 박스(ROI)를 다시 잡아야 할 때
./scripts/run_camera.sh --grid   # 격자 보고 좌표 읽기
./scripts/run_camera.sh --roi <x> <y> <가로> <세로>   # 읽은 좌표로 재실행
```
**끄기**: 창을 클릭해서 포커스 준 뒤 `q` 또는 `ESC`. 안 꺼지면(백그라운드로 띄운 경우):
```bash
pkill -x live_view_usb_local.py 2>/dev/null || ps aux | grep live_view_usb_local | grep -v grep
# 위 ps 결과에서 PID 확인 후:  kill -9 <PID>
```
화면에서 조절 가능한 키: `[`/`]` 화이트밸런스, `9`/`0` 노출(글레어), `w/a/s/d`(+대문자) ROI 이동, `-`/`=` ROI 폭, `,`/`.` ROI 높이, `p` 현재 ROI 값 출력.

### 2) 르키위 자체 카메라 (front / wrist)

먼저 라즈베리파이에서 `lekiwi_host`가 떠 있어야 한다:
```bash
ssh roboseasy@192.168.0.201
nohup ~/start_lekiwi_host.sh lekiwi01 3600 > ~/lekiwi_host_now.log 2>&1 < /dev/null &
disown
exit
```
그다음 노트북에서:
```bash
cd ~/lekiwi-pill-pickup
env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH \
    ./venv/bin/python scripts/live_view_color.py --camera front   # 베이스캠
# 또는 --camera wrist                                              # 손목캠
```
**끄기**: 창에서 `q`/`ESC`. 안 꺼지면:
```bash
ps aux | grep live_view_color | grep -v grep   # PID 확인
kill -9 <PID>
```
**파이 쪽 `lekiwi_host`까지 완전히 끄려면**(다른 스크립트가 카메라/모터를 쓰기 전 반드시 먼저 끌 것):
```bash
ssh roboseasy@192.168.0.201 "pgrep -af robots.lekiwi.lekiwi_host"   # PID 확인(패턴검색은 죽이지 말고 확인만)
ssh roboseasy@192.168.0.201 "kill -9 <PID>"                          # 확인한 PID로 직접 종료
```
⚠️ `pkill -f`처럼 패턴으로 바로 죽이면 SSH 명령 자기 자신이 매칭돼 연결이 끊길 수 있다 — 반드시 PID 확인 후 `kill <PID>`로 지정할 것.
