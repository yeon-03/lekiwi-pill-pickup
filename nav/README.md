# LeKiwi 자율주행 스택 (SLAM + Nav2)

LeKiwi(전방향 3바퀴 베이스)가 지도를 만들고, 그 지도 위에서 자기 위치를 찾고,
목표 지점까지 스스로 가서 돌아오는 데까지의 ROS2 Jazzy 구성이다.
약통 집기 파이프라인의 "이동" 부분을 담당한다 — 집기 자체는 다루지 않는다.

실제로 검증된 것: lekiwi06 에서 만들고, lekiwi01 로 이식해 **왕복 미션 성공
(출발 지점 기준 오차 9.1 cm)**.

## 이 스택이 가정하는 하드웨어

| | |
|---|---|
| 베이스 | 옴니휠 3개 + Feetech STS3215 서보 (반이중 버스) |
| 라이다 | YDLidar Tmini Plus (`/dev/ttyUSB0`) |
| IMU | BMI160 (I2C) — **없어도 된다**, 아래 참고 |
| 컴퓨터 | Raspberry Pi 4, Ubuntu + ROS2 Jazzy |
| 네트워크 | `lekiwi0N` = `192.168.0.20N` 고정 IP |

**IMU 가 없는 기체**라면 `config/lekiwi_cartographer.lua` 에서
`tracking_frame="base_link"`, `use_imu_data=false` 로 고쳐야 한다.
안 고치면 Cartographer 가 `Queue waiting for data: (0, imu)` 로 **조용히 멈춘다**.
`tools/check_config.py` 가 `/imu` 없음을 경고로 알려준다.

## 노트북에서 준비 (클론 직후)

로봇에 파일을 보내는 데 `paramiko` 하나만 필요하다.

```bash
git clone <이 저장소> && cd lekiwi-pill-pickup
python3 -m venv .venv && source .venv/bin/activate
pip install -r nav/requirements.txt
```

> **주의**: conda 를 쓰는 환경이라면 `python3` 가 `(base)` 를 가리켜
> `ModuleNotFoundError: No module named 'paramiko'` 가 난다. 위처럼 venv 를
> 만들거나, `paramiko` 가 설치된 인터프리터를 명시할 것.
> (같은 함정이 로봇에서도 반복됐다 — 로봇에서는 `/usr/bin/python3` 를 쓸 것.
> 런치 파일들이 그렇게 하도록 되어 있다.)

## 로봇에 배포

```bash
export LEKIWI_SSH_PASS='<로봇 비밀번호>'
python nav/deploy/deploy_lekiwi.py lekiwi07 --dry-run   # 뭘 보낼지 먼저 확인
python nav/deploy/deploy_lekiwi.py lekiwi07
```

보내는 것: 노드·도구·셸·설정 49개 + `launch/` 8개. 기존 파일은 로봇의
`~/lekiwi_backup_<시각>/` 에 백업되고, 전송 후 md5 로 검증한다.

**교정값은 덮어쓰지 않는다.** 로봇에 이미 `~/lekiwi_profile.sh` 가 있으면
그대로 둔다 (코드를 고쳐 재배포할 때마다 교정이 날아가면 안 된다).
새로 만들려면 `--reset-profile`. 그 기체의 교정값이 `nav/robots/<이름>/` 에
기록돼 있으면 그것을 보내고, 없으면 **전 항목 미검증** 템플릿을 만든다.

## 로봇에서 — 한 층씩 올린다

아래층이 틀리면 위층이 **전부 조용히** 틀어진다. 증상은 언제나 위층에서
보이므로, 순서대로 확인하며 올라가는 것이 유일하게 시간을 아끼는 방법이다.

| 층 | TF / 토픽 | 명령 |
|---|---|---|
| 0 | 전원·장치 | `ls /dev/ttyUSB0 /dev/ttyACM0` |
| 1 | `base_link → lidar_link` | `ros2 launch ~/launch/lekiwi_setup.launch.py layer:=1` |
| 2 | `/scan`, `/imu` | `… layer:=2` |
| 3 | `odom → base_link` | `… layer:=3` |
| 4 | `map → odom` (지도 만들기) | `ros2 launch ~/launch/lekiwi_slam.launch.py` |
| 5 | `map → odom` (지도 쓰기) | `ros2 launch ~/launch/lekiwi_amcl.launch.py` |
| 6 | Nav2 자율주행 | `ros2 launch ~/launch/lekiwi_nav.launch.py` |

`lekiwi_setup.launch.py` 는 해당 층까지 띄운 뒤 **점검 결과만 출력하고 끝난다.**
전부 통과하면 다음 층으로. 자세한 절차·실패 대처는
[`docs/nav/bringup-guide.md`](../docs/nav/bringup-guide.md).

## 집기와 잇기

로봇은 도착하면 `/abo/pick_request` 를 발행하고 `/abo/pick_done` 을 받아야
복귀합니다. 그 사이를 `mission/pick_adapter.py` 가 잇습니다.

**이 어댑터는 노트북에서 돕니다** — 집기 코드가 카메라 2대와 OpenCV 를 쓰고,
ROS 를 전혀 모르기 때문입니다(집기 쪽 의존성에 `rclpy` 가 없다). 그래서 ROS 를
아는 쪽은 이 파일 하나뿐이고, 집기 코드는 그대로 두면 됩니다.
로봇에는 배포되지 않습니다(`deploy_lekiwi.py` 의 `SHARED` 에 없음).

```bash
# 노트북에서. 로봇과 같은 도메인이어야 토픽이 보인다.
export ROS_DOMAIN_ID=42
python3 nav/mission/pick_adapter.py \
    --repo ~/lekiwi-pill-pickup \
    --map center=green,left=red,right=blue \
    --python ~/lekiwi-pill-pickup/.venv/bin/python
```

`--python` 은 집기를 실행할 인터프리터입니다. `rclpy` 가 있는 파이썬과
집기 의존성(numpy/opencv/scipy/lerobot)이 있는 파이썬은 대개 다른
환경이므로, 다르면 반드시 지정해야 합니다.

제한시간은 로봇 쪽(`--pick-timeout`, 기본 180초)보다 짧게 두어야 합니다.
그래야 로봇이 먼저 포기하지 않고 어댑터가 실패 이유를 보고합니다.

### 로봇 없이 먼저 확인

로봇을 잡기 전에 노트북에서만 검증할 수 있습니다. 가짜 `pick_cycle.py` 로
성공·실패·중간에 죽음·제한시간 초과·중복 요청까지 돌려 봅니다.

```bash
python3 nav/tools/test_pick_adapter.py
```

로봇이 켜져 있어도 안전합니다 — 로봇(도메인 42)과 겹치지 않는 전용 도메인
77 에서 돕니다. 여기까지 통과하면 ROS2 설치와 어댑터는 정상이고, 남은 것은
두 가지뿐입니다.

1. 노트북과 로봇이 서로 토픽을 보는가 — 양쪽 `ROS_DOMAIN_ID=42` 로 맞추고
   `ros2 topic list` 에 `/abo/state` 가 뜨는지
2. 진짜 `pick_cycle.py` 가 `--result-file` 에 결과를 쓰는가

## 새 기체에서 반드시 할 교정 3가지

배포 직후 프로파일의 기계 상수는 전부 **미검증**(lekiwi06 값 복사본)이다.
그대로 쓰면 위치추정이 틀린다.

```bash
# ① 회전 — 제자리라 책상 위에서도 안전
ros2 launch ~/launch/lekiwi_odom_test.launch.py
ros2 launch ~/launch/lekiwi_odom_test.launch.py measured_deg:=1050   # 실측 입력

# ② 직진 — 앞을 비우고
ros2 launch ~/launch/lekiwi_odom_test.launch.py test:=line dist:=0.3
ros2 launch ~/launch/lekiwi_odom_test.launch.py test:=line measured:=0.28

# ③ 라이다 장착각 — 앞을 비우고
ros2 launch ~/launch/lekiwi_lidar_test.launch.py
```

③이 특히 중요하다. 옴니휠 3개가 120도 간격이라 **자기 반사만 보면
`-30 / +90 / -150` 도가 똑같이 그럴듯해 보인다.** 회전으로는 구분되지
않는다(라이다가 회전 중심에 있다). 장착각을 관측할 수 있는 것은 **직진뿐**이라
이 시험이 로봇을 조금 앞으로 움직인다.

결과는 전부 `~/lekiwi_profile.sh` 한 곳에 적는다. 기체마다 다른 값이 사는
곳은 여기 하나뿐이며, 확정되면 `nav/robots/<이름>/` 에 기록해 둘 것.

## 디렉터리

| 경로 | 내용 |
|---|---|
| `launch/` | 런치 8개. **이것이 정본 진입점**이다 |
| `nodes/` | 라이다·IMU·베이스 드라이버 |
| `mission/` | 위치 재정합(`refine_pose.py`), 왕복 미션 브리지(`abo_nav_bridge.py`), 집기 어댑터(`pick_adapter.py` — **노트북에서 실행**) |
| `tools/` | 점검·교정·진단 (`check_config`, `odom_test`, `lidar_test`, `deadbeam` …) |
| `config/` | URDF, Cartographer, Nav2 파라미터 |
| `robots/<이름>/` | 그 기체에서 **실측한** 교정값 |
| `shell/` | 런치로 대체된 셸. 텔레옵 등 일부는 아직 쓴다 |
| `deploy/` | 다른 기체로 이식 |
| `docker/` | 의존성 고정용 이미지 |
| `install/` | ROS2 Jazzy · Docker 설치 |
| `maps/` | 검증된 지도 (`.pbstream` 은 재생성 가능해서 제외) |
| `rviz/` | 데스크톱 시각화 설정 |

## 파일명에 기체 번호를 넣지 않는 이유

SD카드 하나에 로봇 하나이므로 번호가 정보를 더해주지 않는다. 대신 모든
소비자(런치·점검 도구)가 파일명을 추측해야 해서 **"다른 기체 파일을 가리키는"
버그가 반복됐다.** 기체 이름은 프로파일 안에만 있다.

## 더 읽을 것

| 문서 | 언제 |
|---|---|
| [`bringup-guide.md`](../docs/nav/bringup-guide.md) | 처음 세팅하거나 새 기체로 옮길 때 |
| [`slam-nav2-spec.md`](../docs/nav/slam-nav2-spec.md) | 구조와 파라미터 근거가 궁금할 때 |
| [`troubleshooting.md`](../docs/nav/troubleshooting.md) | 안 될 때. 겪은 문제와 원인을 원인별로 정리 |
| [`wifi-eduroam-setup.md`](../docs/nav/wifi-eduroam-setup.md) | 로봇을 학교 Wi-Fi 에 붙일 때 (랜선 → 무선 전환) |
