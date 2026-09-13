# Pick&Place 시연용 웹 UI 설계

날짜: 2026-09-13
목적: 공학경진대회 라이브 시연에서 터미널 CLI 플래그 대신 클릭 몇 번으로 YOLO
pick&place 를 실행·관찰할 수 있는 최소한의 로컬 웹 UI.
수명: 짧음 — 시연 전 튜닝 3일 안팎 + 시연 당일 1일. 그 이후 유지보수 계획 없음.
관련 문서: [`2026-08-12-lekiwi-pill-pickup-design.md`](./2026-08-12-lekiwi-pill-pickup-design.md)
(같은 공학경진대회 시연의 상위 기획서 — 이 문서는 그중 "노트북에서 실행되는 비전+집기"
부분에만 웹 UI 한 겹을 얹는다).

## 배경

`motion/services/pickplace/` 에 이미 실물 검증된 순수 로직(Approacher/ArmSequencer/
WristServo/GraspChecker/YOLO, 원본: `roboseasy/lekiwi.git` 의 `yolo_and_pick/
lekiwi_yolo_pick.py`, 2026-09-04 실물 성공 확인)이 있고, 여기서 발견된 버그(REFINING
중 검출 소실 시 그리퍼를 못 닫던 문제)도 이미 고쳤다(커밋 `95cc30c`).

이 레포에는 로봇에 실제로 명령을 보내며 이 로직을 돌리는 진입점이 아직 없다 — 원본
CLI 스크립트(`lekiwi_yolo_pick.py`)가 그 역할이지만 별도 GitHub 저장소에만 있고, 같은
로직을 GUI로 감싼 `pick_worker.py` 는 타사(Physical Labs) 소유 앱 안에만 있다.

**절대 제약**: Physical Labs 앱(`/opt/physical-labs`)은 학생 팀 소유가 아닌 회사
자산이다. 이번 작업에서 그 앱의 파일을 수정하거나, 그 앱의 코드를 import/의존하거나,
런타임에 영향을 줄 수 있는 어떤 것도 하지 않는다. 참고(읽기)만 하고 전부 이 레포
안에 독립적으로 새로 만든다.

**로봇 호스트 제약**: LeKiwi 호스트는 ZMQ 클라이언트를 하나만 허용한다. 이 새 도구와
Physical Labs 앱을 동시에 같은 로봇에 연결할 수 없다 (동시 실행 금지는 팀 운영
수칙으로 관리 — 이번 스코프에서 소프트웨어 가드는 만들지 않는다).

## 목표 / 비목표

**목표**:
- 브라우저에서 시작/일시정지/정지/비상정지 버튼으로 pick&place 를 제어
- front/wrist 카메라 영상(오버레이 포함)을 화면에 띄워 시연 관람객이 볼 수 있게
- 현재 상태(상태기 라벨, Hz, pick 시도 횟수 등) 텍스트로 표시
- 기존에 이미 저장된 자세 파일(`~/.PhysicalLabs/pickplace/lekiwi01/poses/*.json`)과
  로봇 연결 정보(`~/.config/PhysicalLabs/robots.json` 의 lekiwi01 항목)를 그대로 활용

**비목표** (짧은 수명 + 코드 리뷰에서 의도적으로 제외하기로 한 것):
- 인증, 영속성/DB, 다중 사용자 대응 — 로컬/같은 부스 네트워크에서만 접속 전제
- 이중 클라이언트 접속을 막는 소프트웨어 가드 — 팀 운영 수칙으로 대체
- 진짜 MJPEG 스트리밍의 세밀한 최적화 — 아래 "카메라 스트림" 절 참고
- Physical Labs 앱과의 어떤 형태의 연동/공유 코드도 만들지 않음

## 아키텍처

```
[브라우저] --HTTP(폴링/버튼)--> [FastAPI + uvicorn] --읽기/쓰기--> [공유 상태 객체]
                                                                        ^
                                                                        |
                                                              [백그라운드 스레드]
                                                          (제어 루프, ~30Hz)
                                                                        |
                                                                        v
                                                          [LeKiwiClient] <-ZMQ-> [로봇 호스트]
```

### 1. 제어 루프 (신규, 이 레포 안에만)

**정확히 무엇을 "포팅"하는지** (검증 과정에서 바로잡음): 원본 CLI(`lekiwi_yolo_pick.py`,
`roboseasy/lekiwi.git`)를 파일째로 이 레포에 들여오지 않는다. 그 스크립트가 의존하는
형제 파일(`lekiwi_yolo_view.py`의 `LeKiwiRobotArgs`/`YoloArgs`/`infer`/`draw` 등)은
이미 이 레포의 `motion/services/pickplace/yolo_detect.py` 등으로 "한 줄도 다르지
않게" 포팅되어 있다 — 즉 **원본의 로직 자체는 이미 이 레포 안에 있다.** 새로 만드는
건 그 조각들을 원본 `control_loop()`/`main()`과 같은 **순서·판정 기준**으로 조립하는
얇은 헤드리스 실행기뿐이다: 연결 → `wait_for_frames`류 첫 프레임 대기 → 관측 → YOLO
배치 추론 → `Approacher`/`ArmSequencer`/`GraspChecker` 갱신 → 오버레이 → `send_action`.
이 조립 순서는 Physical Labs 쪽 소스에서 다시 유추하지 않고 원본 CLI 스크립트를
직접 대조해 확인한다.

**원본에 없어서 새로 추가해야 하는 것 두 가지** (둘 다 원본이 cv2 창 기반 키보드
입력에만 의존했기 때문에 헤드리스/웹 조작에는 원래 없던 것):
- **이벤트 기반 제어**: 원본은 `--display=cv2`일 때만 `cv2.waitKey`로 SPACE(일시정지)/
  Q(종료)를 받는다 — `--display=none`(헤드리스)이면 실행 중 제어 자체가 불가능하다.
  웹 버튼으로 제어하려면 `threading.Event`(시작/일시정지/정지/비상정지) 주입 지점을
  새로 만들어야 한다.
- **정지 시 롤아웃**: 원본의 종료 처리는 사실 "현재 자세 유지 + 바퀴 정지 + 즉시
  연결 해제"뿐이다(정상 종료·Ctrl+C·예외 구분 없이 동일). "시작 자세로 천천히
  롤아웃"은 원본에는 없고 Physical Labs GUI 앱이 나중에(2026-09-08) 추가한 안전
  기능이다. 이번 설계의 정지/비상정지 구분(아래 절)을 위해 **이 롤아웃 동작만은
  의도적으로 가져온다** — GUI 앱 코드 전체를 참고하지 않고, 이 롤아웃 함수 하나만
  (자세 보간 + 진행률 기반 반복, 그 자체로 단순하고 검증하기 쉬운 코드) 대조해
  이 레포 안에 독립적으로 다시 구현한다.

파일: `motion/services/pickplace/headless_worker.py` (가칭). 클래스
`PickPlaceHeadlessWorker`가 `threading.Thread` 서브클래스 또는 `run()`을 갖는
plain 객체 + 별도 스레드 기동 방식 중 구현 시 더 자연스러운 쪽으로.

### 2. 안전 설계 — 정지의 유일한 책임자는 제어 루프 자신

- 루프 본체는 `try: ... finally: 정지 시퀀스` 로 감싼다. `finally` 가 하는 일:
  1. 바퀴 정지 명령 전송
  2. **정상 종료(stop) 인 경우만**: 현재 자세 → 시작 자세로 천천히 롤아웃 (위
     "제어 루프" 절에서 설명한, Physical Labs GUI 앱의 `_roll_out` 패턴을 참고해
     이 레포에 독립적으로 새로 구현하는 부분 — 그리퍼는 건드리지 않음, 물건을
     물고 있을 수 있음)
  3. **비상정지(abort) 또는 예외 종료인 경우**: 롤아웃 없이 그 자리에서 바로
  4. 로봇 연결 해제
- 이 `finally` 는 웹 서버 상태와 무관하게 실행돼야 한다. 프로세스가 `SIGINT`/
  `SIGTERM` 을 받아도(터미널에서 Ctrl+C, 프로세스 kill) 같은 경로를 타도록 신호
  핸들러를 등록한다. FastAPI 가 죽어도 이 스레드의 `finally` 자체는 별도로 동작해야
  하므로, 최상위 실행 스크립트에서 이 스레드가 메인 스레드와 함께 정리되도록 한다
  (데몬 스레드로 두지 않고, 종료 시 join 대기).
- 두 종류의 이벤트를 둔다: `stop_event`(정상 정지, 롤아웃 포함) / `abort_event`
  (비상정지, 롤아웃 생략). 예외가 루프 안에서 발생해도 `finally` 는 `abort_event`
  가 세팅된 것처럼 롤아웃 없이 정지한다(안전 우선).
- 루프는 매 반복 시작 시 두 이벤트를 확인한다. 반영 지연은 최대 한 틱(1/30초)로
  시연 목적엔 충분하다.

### 3. FastAPI + uvicorn 백엔드

이미 `lerobot` conda 환경에 설치돼 있어 새 의존성 불필요.

엔드포인트:
- `POST /start` — `paused_event.clear()` (첫 실행 포함, 일시정지 해제)
- `POST /pause` — `paused_event.set()`
- `POST /stop` — `stop_event` 세팅 (롤아웃 후 종료)
- `POST /estop` — `abort_event` 세팅 (즉시 정지, 롤아웃 생략) — 플래그 하나만
  세팅하는 아주 가벼운 핸들러라 다른 처리와 절대 안 겹친다
- `GET /status` — 상태 라벨, Hz, pick 시도/재시도 횟수, dry-run 여부 등을 JSON 으로.
  프론트가 주기적으로(예: 3~5Hz) 폴링. (스레드 생존 여부 구분 표시는 "명시적으로
  제외" 절 참고 — 이번 스코프에서는 넣지 않는다)
- `GET /stream/front`, `GET /stream/wrist` — 오버레이 포함 최신 프레임을 JPEG 로
  서빙 (스트리밍 방식은 구현 단계에서 MJPEG multipart 로 하되, 인코딩은 락 밖에서
  수행하고 공유 프레임 참조 교체만 락 안에서 짧게 한다 — 제어 루프를 절대 막지
  않는다)
- 블로킹 작업(프레임 인코딩 등)이 들어가는 핸들러는 `async def` 가 아닌 일반
  `def` 로 작성해 FastAPI 스레드풀에서 돌게 한다 — `/estop`/`/status` 같은 가벼운
  핸들러가 무거운 스트리밍 핸들러에 발목 잡히지 않도록

### 4. 프론트엔드

빌드 과정 없는 단일 정적 HTML/JS 페이지:
- `<img src="/stream/front">`, `<img src="/stream/wrist">`
- 상태 텍스트 영역 (`/status` 폴링)
- 버튼: [▶ 시작] [❚❚ 일시정지] [■ 정지] + 눈에 띄게 분리된 큰 빨간 **[⚠ 비상정지]**
- dry-run 토글

### 5. 진입점

`python run_pickplace_ui.py --robot.remote_ip=10.42.0.141 --robot.id=lekiwi01
--yolo.path=/home/roboseasy/YOLO/outputs/runs/green_pill/weights/best.pt
--pick.pose_file=~/.PhysicalLabs/pickplace/lekiwi01/poses/pre_pick.json
--grasp.grasp_pose_file=~/.PhysicalLabs/pickplace/lekiwi01/poses/grasp.json
--grasp.close_pose_file=~/.PhysicalLabs/pickplace/lekiwi01/poses/grasp_closed.json`
형태로, 원본 CLI 와 같은 인자 스타일을 유지하면서 uvicorn 서버를 띄운다. 이 값들
전부(로봇 IP `10.42.0.141`, ZMQ 포트 5555/5556, YOLO 가중치 경로, 자세 파일 경로
3개)를 스크립트의 기본값으로 넣어 매번 길게 안 쳐도 되게 한다.

## 데이터 흐름

1. 백그라운드 스레드가 매 루프 반복 끝에 최신 프레임(오버레이 포함, BGR→JPEG
   인코딩된 bytes)과 상태 dict 를 공유 객체에 기록 (락으로 참조 교체만 보호)
2. FastAPI 스트리밍 핸들러는 공유 객체에서 최신 JPEG 를 꺼내 스트림에 밀어넣음
3. `/status` 는 상태 dict 를 그대로 JSON 직렬화
4. 버튼 클릭 → `fetch(POST)` → 해당 `threading.Event.set()` → 다음 루프 틱에 반영

## 테스트 계획

- `motion/services/pickplace/` 의 기존 순수 로직 단위 테스트(이번에 추가한
  `test_wrist_servo.py` 포함)는 그대로 유지 — 이 UI 작업은 그 로직을 건드리지 않음
- 새 헤드리스 워커/FastAPI 레이어는 로봇 없이 확인 가능한 범위로:
  - `--dry_run=true` 로 실제 로봇 연결 없이(또는 연결 실패를 허용하는 모드로)
    상태 전이·정지/비상정지 이벤트 처리 로직만 단위 테스트
  - 실제 로봇 연결·영상 스트리밍은 로봇 호스트가 켜졌을 때 수동으로 확인
    (이 환경에는 자동화된 하드웨어 테스트가 없음 — README 관례와 동일)

## 명시적으로 제외 (독립 검토에서 나왔지만 이번엔 반영 안 함)

- 이중 클라이언트 접속 소프트웨어 가드 (락파일 등) — 팀 운영 수칙으로 대체
- 진짜 MJPEG 대신 스냅샷 폴링으로 바꾸는 대안 — 이번엔 MJPEG 유지 (락 스코프만
  올바르게)
- 백그라운드 스레드 예외를 `/status` 에 "스레드 살아있음"으로 명시적으로 구분해
  보여주는 것 — 최소 구현에서는 생략, 필요하면 나중에 추가
