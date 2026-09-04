# LeKiwi 브링업 가이드 — TF 체인을 따라 한 층씩

처음 하는 사람이 순서대로 따라갈 수 있게 쓴 문서다. 기준 기체는 `lekiwi06`
(192.168.0.206). 완성된 시스템의 사양은 `slam-nav2-spec.md` 를 볼 것.

---

## 왜 TF 체인 순서인가

이 로봇의 모든 것은 하나의 좌표 사슬 위에 얹혀 있다.

```
map                    ← 지도. SLAM 이 만들고 AMCL 이 그 위에서 위치를 찾는다
 └─ odom               ← 오도메트리 원점. 전원을 켠 자리
     └─ base_link      ← 로봇 몸통 중심
         ├─ base_footprint    바닥 투영
         ├─ lidar_link        라이다
         └─ imu_link          IMU
```

**사슬은 아래에서 위로 만들어진다.** `base_link → lidar_link` 가 틀리면 그
위에 쌓는 오도메트리도 SLAM 도 전부 틀어진다. 그런데 증상은 맨 위(“지도가
이상해요”)에서만 보이기 때문에, 아래층부터 하나씩 확인하지 않으면 원인을
찾는 데 며칠이 걸린다.

그래서 이 문서는 **한 층 올리고 → 그 층만 확인하고 → 다음 층**으로 간다.
각 층은 “무엇을 만드는가 / 실행 / 확인 / 실패하면” 네 부분이다.

| 층 | 만드는 것 | 담당 노드 |
|---|---|---|
| 0 | (TF 이전) 전원·장치 | — |
| 1 | `base_link → lidar_link`·`imu_link`·`base_footprint` | `robot_state_publisher` |
| 2 | `/scan`, `/imu` — 1층 프레임에 실린다 | `ydlidar_node`, `bmi160_node` |
| 3 | `odom → base_link` | `lekiwi_base_node` |
| 4 | `map → odom` (지도 만들기) | `cartographer` |
| 5 | `map → odom` (지도 쓰기) | `amcl` |
| 6 | 자율주행 | Nav2 |
| 7 | 왕복 미션 | `abo_nav_bridge` |

> **4층과 5층은 동시에 띄우면 안 된다.** 둘 다 `map → odom` 을 발행해서
> 충돌한다. 지도를 만들 때는 4층, 그 지도로 주행할 때는 5층이다.

### 공통 준비

모든 명령은 로봇에서 실행한다. 별도 표기가 있으면 데스크톱.

```bash
ssh roboseasy@192.168.0.206          # 비밀번호는 팀에 문의 (문서에 적지 않는다)

source /opt/ros/jazzy/setup.bash     # 새 터미널마다
export ROS_DOMAIN_ID=42              # 이게 없으면 토픽이 아예 안 보인다
```

### 런치 파일로 띄운다

이 문서는 **런치 파일**을 기준으로 쓴다. 셸 스크립트(`run_*.sh`)도 그대로
남아 있고 하는 일은 같지만, 런치가 나은 이유가 있다.

- **`respawn`** — 노드가 죽어도 되살아난다. IMU 가 I2C 오류로 죽었을 때
  카토그래퍼가 `Queue waiting for data: (0, imu)` 로 조용히 멈춰버린 사고가
  있었다.
- **인자를 빠뜨릴 수 없다** — `run_base.sh` 는 `--tf` 를 손으로 붙여야 하는데,
  빠뜨리면 `/odom` 은 나오는데 TF 가 없어서 위층이 전부 실패한다. 런치는 항상
  붙여준다.
- **Ctrl+C 한 번으로 전부 정리된다.**

런치 파일은 `~/launch/` 에 있고 ROS 환경·기체 상수를 스스로 설정한다.
확인 명령(`ros2 topic ...`)을 직접 칠 때만 위의 `source` 두 줄이 필요하다.

| 런치 | 층 | 하는 일 |
|---|---|---|
| `lekiwi_sensors.launch.py` | 1~3 | 스택을 올린다 (`layer:=1/2/3`) |
| **`lekiwi_setup.launch.py`** | 0~3 | 올리고 **설정을 점검**한다 |
| **`lekiwi_odom_test.launch.py`** | 3 | **오도메트리 정확도를 잰다** |
| **`lekiwi_lidar_test.launch.py`** | 1 | **라이다 장착각을 잰다** |
| `lekiwi_slam.launch.py` | 4 | 지도 만들기 |
| `lekiwi_amcl.launch.py` | 5 | 위치추정만 |
| `lekiwi_nav.launch.py` | 6~7 | 자율주행 + abo |

굵게 표시한 셋이 **시험 런치**다. 아래 각 층에서 손으로 확인하는 대신
이걸 돌리면 로봇이 스스로 움직이고 숫자를 내준다. 처음 세팅하거나 다른
기체로 옮긴 뒤에는 이 순서로 돌리는 것이 가장 빠르다.

```bash
ros2 launch ~/launch/lekiwi_setup.launch.py layer:=1     # 설정·정적 TF
ros2 launch ~/launch/lekiwi_setup.launch.py layer:=3     # 장치·토픽까지
ros2 launch ~/launch/lekiwi_odom_test.launch.py          # 오도메트리
ros2 launch ~/launch/lekiwi_lidar_test.launch.py         # 라이다 장착각
```

시험이 끝나면 런치가 스스로 내려간다. 순서가 중요하다 — 라이다 시험은
이동량을 오도메트리에서 읽으므로 **오도메트리가 먼저 맞아야 한다.**

**런치를 끄는 법**: 실행한 터미널에서 `Ctrl+C`. 백그라운드로 띄웠다면
런치 파일명으로 찾아 죽인다 (`pkill -f` 는 자기 셸을 죽이므로 PID 지정).

```bash
ps -eo pid,cmd | grep "lekiwi_.*\.launch" | grep -v grep
kill <PID>
```

> **`python3` 대신 `/usr/bin/python3` 을 쓸 것.** 그냥 `python3` 은 conda
> `(base)` 로 잡히는데 거기엔 `rclpy` 가 없어서 `ModuleNotFoundError` 가 난다.

---

## 0층 — 전원과 장치

TF 이전 단계지만, 여기가 어긋나면 위층이 전부 조용히 실패한다.

### 확인

```bash
/usr/bin/python3 ~/procchk.py        # 뭐가 떠 있나
ls -l /dev/ttyUSB* /dev/ttyACM*      # 장치가 보이나
/usr/bin/python3 ~/scan_servo.py     # 서보가 응답하나
```

**정상 출력**

```
  ○ 라이다   꺼짐          ← 처음이니 다 꺼져 있는 게 맞다
  ○ 베이스   꺼짐
  ...
  crw-rw---- ... /dev/ttyUSB0        ← 라이다
  crw-rw---- ... /dev/ttyACM0        ← 서보 (바퀴 7,8,9 + 팔 1~6)

  /dev/ttyACM0   응답 1,2,3,4,5,6,7,8,9   무응답 10,11,12
```

### 실패하면

| 증상 | 원인 |
|---|---|
| `/dev/ttyUSB0` 없음 | 라이다 USB 가 빠졌다. `lsusb` 에도 안 보이면 물리적으로 분리된 것 |
| 서보 전부 무응답 (포트는 열림) | **본체 전원이 꺼져 있다.** USB 시리얼 칩은 USB 전원으로 살아 있어서 포트는 보이지만 서보는 배터리를 쓴다 |
| 서보 일부만 무응답 | 그 서보의 배선/커넥터 |
| `열 수 없음` | 다른 프로세스가 이미 잡았다. `/usr/bin/python3 ~/procchk.py` 로 확인 |

> **`pgrep -f` 로 노드를 확인하지 말 것.** 검사 명령 문자열 자체에 노드
> 이름이 들어 있어서 자기 자신을 잡는다. 다 꺼져 있는데 다 켜져 있다고
> 나온다. `procchk.py` 는 이걸 걸러낸다.

---

## 1층 — `base_link → lidar_link` (URDF)

**만드는 것**: 몸통 기준으로 라이다·IMU·바닥이 어디에 어떤 각도로 붙어
있는지. 정적 TF(`/tf_static`)라 한 번만 발행된다.

### 실행

```bash
ros2 launch ~/launch/lekiwi_sensors.launch.py layer:=1
```

`layer:=1` 은 `robot_state_publisher` 만 띄운다. 라이다도 베이스도 아직 없다.

### 확인

```bash
ros2 run tf2_ros tf2_echo base_link lidar_link
```

**정상 출력** (lekiwi06 기준)

```
- Translation: [0.000, 0.000, -0.053]
- Rotation: in RPY (degree) [0.000, -0.000, 0.000]
```

`imu_link` 는 `[0.000, -0.055, 0.020]`, `base_footprint` 는 `[0, 0, -0.055]`.

### 실패하면

- **아무것도 안 나온다** → 런치 출력을 볼 것.
  URDF 를 `-p robot_description:="$(cat ...)"` 로 넘기면 여러 줄 XML 이 ROS
  인자 파서를 깨서 즉시 abort 한다. **파일 경로로 넘겨야 한다** (런치는 이미
  그렇게 되어 있다).
- **값이 다르다** → `~/lekiwi06.urdf` 를 고친 뒤 런치 재시작. URDF 를 바꾸면
  `lekiwi_profile.sh` 의 `LEKIWI_LIDAR_YAW` 와 런치의 `DEFAULTS["lidar_yaw"]`
  도 **같이** 고쳐야 한다. 셋은 항상 같은 값이어야 한다.

### ⚠ 이 층에서 가장 중요한 것 — 라이다 장착각

`lidar_link` 의 **yaw** 가 이 시스템에서 제일 틀리기 쉽고, 틀렸을 때 가장
찾기 어려운 값이다. 이 값이 30° 틀어진 채로 지도를 만들면 지도 자체가
못 쓰게 된다.

**옴니휠 자기반사로는 확정할 수 없다.** 라이다는 자기 바퀴 세 개를 보는데,
그 세 개가 120° 간격이라 `-30 / +90 / -150` 세 후보가 데이터와 **똑같이**
맞는다. 각도는 1° 안에서 나오지만 “어느 바퀴인가”는 정해지지 않는다.

**확정하는 방법 — 직진 시험**

```bash
ros2 launch ~/launch/lekiwi_lidar_test.launch.py
```

로봇이 앞으로 0.30 m 이동하면서 출발/도착 스캔을 찍고, 오도메트리 이동량만큼
겹쳐 본다. 장착각이 맞을 때만 두 스캔의 벽이 포개지므로, 후보 각도를 훑어
잔차가 최소인 값을 찾는다. 설정값과 다르면 고쳐야 할 파일을 알려준다.

```
--- 결과 ---
  측정된 장착각   +0.00도   (잔차 0.0243 m)
  현재 설정값     +0.00도   (잔차 0.0243 m)
  설정값과 0.00도 차이 -- 맞다.
```

합성 데이터로 검증했을 때 −30 / +90 / −150 / +17.5 를 모두 0.2° 안에서
되찾았다. **자기반사로 구분할 수 없는 세 후보를 이 방법은 구분한다.**

**왜 회전이 아니라 직진인가**: 라이다는 회전 중심(`base_link` 원점) 바로
위에 있다. 제자리 회전만 시키면 라이다도 같은 각도로 돌 뿐이라 장착각이
얼마든 스캔이 똑같이 겹친다 — **회전으로는 장착각이 관측되지 않는다.**

눈으로 확인하려면: RViz 를 켜고 텔레옵으로 직진시킨 뒤, `base_link` 축이
**실제 로봇이 나아간 방향**과 같은지 본다.

값을 고칠 때는 **세 곳을 전부** 고쳐야 한다. 하나만 고치면 조용히 어긋난다
(`lekiwi_setup.launch.py` 가 이 셋의 일치를 점검한다).

> 실제로 이 기체는 URDF 에 `-30°` 가 들어 있었고, 그 상태로 만든 지도를
> 통째로 버려야 했다. 지금 값은 `0.0°` 이며 직진 시험으로 검증됐다.
>
> **바퀴 사각지대 위치로 역산하지 말 것.** 옴니휠의 `ANGLES` 는 바퀴가
> 굴러가는 방향이지 바퀴가 달린 방위가 아니다. 둘은 90° 차이가 난다.

---

## 2층 — 센서 데이터 `/scan`, `/imu`

**만드는 것**: TF 는 아니지만 1층에서 정의한 `lidar_link`·`imu_link`
프레임 위에 실려 오는 데이터.

### 실행

```bash
ros2 launch ~/launch/lekiwi_sensors.launch.py layer:=2
```

1층에 라이다와 IMU 가 더해진다. IMU 는 시작 후 **8초간 바이어스를 추정**하니
그동안 로봇을 건드리지 말 것.

주행(5~7층)만 할 거라 IMU 가 필요 없으면:

```bash
ros2 launch ~/launch/lekiwi_sensors.launch.py layer:=2 use_imu:=false
```

### 확인

```bash
ros2 topic hz /scan
ros2 topic echo /scan --once | head -20
```

**정상**: 약 **6.1 Hz**, `frame_id: lidar_link`, 빔 약 450개.

```bash
ros2 topic hz /imu                   # IMU 를 쓸 때만
```

### 라이다 전처리가 켜져 있는지

런치는 기본으로 세 가지를 적용한다. 셋 다 실측으로 필요성이 확인된 것이라
끄지 말 것.

| 전처리 | 기본값 | 없으면 |
|---|---|---|
| 강도 필터 | `--min-intensity 25` | **방 한가운데 가짜 벽**이 생긴다 |
| 바퀴 마스킹 | `--mask-wheels` | 옴니휠 3개가 가리는 시야 30.4% 가 장애물로 찍힌다 |
| 디스큐 | `--deskew` | 회전 중 스캔이 휘어 정합이 흔들린다 |

### 실패하면

- **`could not open port /dev/ttyUSB0: No such file or directory`** → 0층으로.
  라이다가 USB 에 없다. 대개 **본체 전원**이다.
- **`device reports readiness to read but returned no data`** → 라이다 노드가
  두 개 떠서 같은 포트를 잡았다. 런치를 두 번 띄우지 않았는지 `procchk.py` 로
  확인할 것.
- **IMU `I2C Errno 5`** → I2C 버스가 죽었다. 노드에 3회 재시도가 들어 있지만
  계속 나면 **전원 재인가**로 복구된다 (칩 ID `0xd1` 이 읽히면 정상).
  IMU 가 죽으면 **카토그래퍼가 조용히 멈춘다** — 에러 없이 지도만 안 늘어난다.
- **IMU 없이 진행해도 되나** → 주행(5~7층)은 IMU 를 안 쓴다. SLAM(4층)만
  쓴다. 단 Cartographer 2D 는 IMU 를 **중력 정렬과 자이로에만** 쓰고
  가속도는 위치 적분에 절대 쓰지 않는다.

---

## 3층 — `odom → base_link` (휠 오도메트리)

**만드는 것**: 전원을 켠 자리를 원점으로 한 상대 위치. 엔코더 적분이라
시간이 지나면 어긋나지만(드리프트), 짧은 구간에서는 정확하다.

### 실행

```bash
ros2 launch ~/launch/lekiwi_sensors.launch.py        # layer:=3 이 기본값
```

모터를 돌리지 않고 오도메트리만 보려면:

```bash
ros2 launch ~/launch/lekiwi_sensors.launch.py drive:=false
```

> 셸 스크립트(`run_base.sh`)로 띄울 때는 **`--tf` 를 반드시 붙여야 한다.**
> 빠뜨리면 `/odom` 토픽은 나오는데 `odom → base_link` TF 가 없다. 그러면
> 위층이 전부 “TF 를 못 찾겠다”로 실패하는데 정작 `/odom` 은 잘 보이니
> 원인을 찾기 어렵다. **런치는 항상 붙여준다** — 런치를 쓰는 이유 중 하나다.

### 확인

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic hz /odom                          # 약 25 Hz
/usr/bin/python3 ~/tf_chain.py               # 어느 링크가 흐르는지
```

**정상**

```
  odom -> base_link           25.0 Hz
```

### 정확도 시험 — 여기서 반드시 할 것

오도메트리가 틀리면 SLAM 도 주행도 전부 틀린다. 두 가지를 잰다.

**① 회전 (제자리라 안전, 먼저 할 것)**

```bash
ros2 launch ~/launch/lekiwi_odom_test.launch.py
```

로봇이 오도메트리 기준으로 +3바퀴 돌고 −3바퀴 되돌아온다. 두 가지를 본다.

- **왕복 닫힘** — 되돌아온 뒤 yaw 잔차. 5° 미만이면 좋다. 크면 바퀴
  미끄러짐이나 부호/전원 문제다. 이건 자동으로 판정된다.
- **축척** — 왕복하면 `BASE_R` 이 틀려도 0 이 되므로 **축척은 검증되지
  않는다.** 실제 회전량은 사람이 재야 한다. 바닥 표시를 기준으로 몇 도를
  돌았는지 재서 다시 실행하면 보정값을 계산해 준다:

```bash
ros2 launch ~/launch/lekiwi_odom_test.launch.py measured_deg:=1050
```

```
  LEKIWI_BASE_R  0.13647 -> 0.14037  (+2.9%)
```

> lekiwi06 은 **0.13647 m**. lerobot 기본값 `0.125` 는 **9% 틀리다.**
> 다른 기체에서도 반드시 다시 재야 한다.

**② 직진**

```bash
ros2 launch ~/launch/lekiwi_odom_test.launch.py test:=line dist:=0.30
```

오도메트리 기준으로 정확히 0.30 m 가고 멈춘다. 자를 대고 실제 거리를 재서:

```bash
ros2 launch ~/launch/lekiwi_odom_test.launch.py test:=line dist:=0.30 measured:=0.28
```

```
  LEKIWI_WHEEL_R  0.05000 -> 0.04667  (-6.7%)
```

> 로봇이 스스로 움직인다. `collision_monitor` 가 없으니 앞을 비우고 지켜볼 것.
> 오도메트리가 4초간 변하지 않으면 시험이 스스로 중단한다.

### 실패하면

- **`ID 7 읽기 실패` 후 즉시 종료** → 서보가 응답하지 않는다. 0층으로
  돌아가 `scan_servo.py` 를 돌린다. 대개 **본체 전원**이다.
- **`SerialException: ... multiple access on port?`** → 두 프로세스가
  `/dev/ttyACM0` 을 동시에 쓴다. Linux 는 시리얼 포트 배타 개방을 강제하지
  않아서 **둘 다 열리고 충돌만 난다.** ZMQ 로 팔을 제어할 때는
  `/lekiwi_base/set_bus` 서비스로 버스를 넘겨야 한다(7층 참조).
- **바퀴가 반대로 돈다** → `LEKIWI_WHEEL_SIGN`.

---

## 4층 — `map → odom` : 지도 만들기 (Cartographer)

**만드는 것**: 지도, 그리고 그 지도 안에서의 위치.

> **5층(AMCL)이 떠 있으면 먼저 끌 것.** 둘 다 `map → odom` 을 발행한다.

### 실행

```bash
ros2 launch ~/launch/lekiwi_slam.launch.py
```

1~3층(센서 전체, IMU 포함)을 띄우고 **14초 뒤** 카토그래퍼를 올린다. 이
지연이 중요하다 — 카토그래퍼가 먼저 뜨면 TF 가 없어 실패하고, 센서보다 먼저
뜨면 빈 데이터로 첫 서브맵을 만들어 **지도가 두 겹**이 된다. IMU 바이어스
추정 8초도 이 지연 안에 들어간다.

### 확인

```bash
/usr/bin/python3 ~/tf_chain.py
```

**정상**

```
  map -> odom                  6.1 Hz        ← 이게 생기면 SLAM 이 돌고 있다
  odom -> base_link           25.0 Hz
```

데스크톱에서 실시간으로 보기:

```bash
bash ~/leisaac/scripts/run_rviz_lekiwi06.sh
```

### 지도를 잘 만드는 요령

- **천천히**. 회전 0.25 rad/s 를 넘기지 말 것. 0.60 rad/s 에서는 연속 스캔의
  21% 가 어긋나고, 0.15 rad/s 에서는 2.5% 다.
- **같은 곳을 두 번 지날 것**. 루프 클로저가 걸려야 누적 오차가 접힌다.
- 벽을 한 바퀴 돌아 **경계를 닫을 것**.

### 저장 (데스크톱에서)

```bash
bash ~/leisaac/scripts/save_map.sh [이름]     # 기본 map_MMDD_HHMM
```

### 후처리 — 반드시 할 것

SLAM 결과에는 이웃 없는 고립 점유 셀(허위 반환 흔적)이 남는다. 이게 팽창하면
**로봇 자신이 서 있는 칸을 막아** 경로 계획이 시작조차 못 한다.

```bash
python3 ~/leisaac/scripts/despeckle_map.py <지도.yaml> <출력이름> [최소이웃수]
```

`최소이웃수` 기본 3. 벽이 얇게 잡힌 지도에서는 3이 **진짜 벽까지 지우므로**
먼저 2로 해보고 결과를 눈으로 비교할 것.

### 로봇으로 배포

```bash
~/miniconda3/envs/leisaac/bin/python ~/leisaac/scripts/deploy_map.py <이름>
```

### 정지

런치를 띄운 터미널에서 `Ctrl+C`. 바퀴 토크까지 확실히 풀려면:

```bash
bash ~/stop_all.sh           # 바퀴 속도 0 + 토크 해제
```

---

## 5층 — `map → odom` : 지도 쓰기 (AMCL)

**만드는 것**: 이미 있는 지도 안에서 “지금 어디인가”.

> **카토그래퍼를 반드시 먼저 끌 것.** 4층 런치를 `Ctrl+C` 로 내리거나
> `bash ~/stop_cartographer.sh`.

### 실행

```bash
ros2 launch ~/launch/lekiwi_amcl.launch.py
ros2 launch ~/launch/lekiwi_amcl.launch.py map:=/home/roboseasy/maps/map_0827_1534_clean.yaml
```

센서(IMU 제외) + `map_server` + AMCL 이 뜬다. 지도를 생략하면 `*_clean` 중
가장 최근 것을 쓴다. 25초 뒤 `/map` 을 한 번 강제로 다시 발행한다(아래
`Waiting for map....` 참조).

주행 기능(플래너·컨트롤러)은 없으므로 목표를 줄 수 없고 텔레옵으로만
움직인다. **위치추정만 보고 싶을 때 이 층에서 멈춘다.**

### 확인

```bash
ros2 service call /amcl/get_state lifecycle_msgs/srv/GetState      # active
/usr/bin/python3 ~/tf_chain.py                                     # map->odom 6 Hz
```

### 위치 잡기 — 순서가 중요하다

**① 실제 위치를 알면 직접 박는다** (가장 확실)

```bash
/usr/bin/python3 ~/set_pose.py 0 0 0        # SLAM 시작 지점 = 지도 원점
```

> **lekiwi06 의 시작 위치는 항상 지도 원점 `(0, 0, 0°)`** — 바닥에 표시해 둔
> 자리이자 카토그래퍼를 시작한 자리다. 로봇을 손으로 옮긴 뒤에는 전역 탐색을
> 돌리지 말고 이걸로 박은 다음 ②로 다듬는다.

**② 미세 조정**

```bash
bash ~/relocalize.sh                                # 현재 위치 주변 ±0.4 m / ±15°
```

**③ 정지 중이면 파티클을 강제로 띄운다**

```bash
ros2 service call /request_nomotion_update std_srvs/srv/Empty
```

### 정합 판정 기준

`relocalize.sh` 출력에서:

| 항목 | 기준 |
|---|---|
| 평균거리 | **0.02 m 이하** |
| `<5cm` | 80% 이상이면 좋음 |
| **`>=30cm`** | **0% — 이게 가장 중요하다** |

> `<5cm` 가 낮아도 `>=30cm` 가 0% 면 위치는 맞다. 반대로 `<5cm` 가 높은데
> `>=30cm` 가 5% 있으면 **엉뚱한 곳일 가능성이 크다.** 스캔 끝점이 허공에
> 떠 있다는 뜻이라 물리적으로 불가능하기 때문이다.

### 실패하면

- **`Waiting for map....`** → `map_server` 는 `/map` 을 TRANSIENT_LOCAL 로
  **한 번만** 발행한다. AMCL 이 늦게 구독하면 놓친다. 강제로 다시 발행:

  ```bash
  ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap \
    "{map_url: /home/roboseasy/maps/map_0827_1534_clean.yaml}"
  ```

- **`/amcl_pose` 도 파티클도 안 나온다** → **정상이다.** AMCL 은
  `update_min_d 0.10` / `update_min_a 0.15` 를 넘게 움직여야 한 주기를 돈다.
  멈춰 있으면 새 정보가 없어서 갱신하지 않는다(같은 정보로 재추출을 반복하면
  파티클이 인위적으로 수렴해 필터가 망가진다). 화면에 띄우려면 위 ③.

- **RViz 에 파티클이 안 보인다** → `ParticleCloud` 는 **BEST_EFFORT** 로
  발행된다. RViz 디스플레이의 Reliability 를 `Best Effort` 로 맞춰야 한다
  (`lekiwi06_nav2.rviz` 는 이미 맞춰져 있다).

- **노란 화살표가 옛 자리에 남아 있다** → 그 화살표는 `/odom` 을 보는
  Odometry 디스플레이이고 `Keep: 30` 이라 과거 30개가 화면에 남는다.
  재정합으로 `map→odom` 이 바뀌어도 이미 그린 것은 안 지워진다. **디스플레이
  체크를 껐다 켜면** 지워진다.

- **재정합이 엉뚱한 곳을 고른다** → 지표는 “잘 매핑된 넓은 구역”을 편애한다.
  지도 가장자리에 있는 시작 지점은 불리하다. **사람이 아는 위치가 지표보다
  우선한다.** `set_pose.py` 로 박을 것.

---

## 6층 — Nav2 자율주행

**만드는 것**: 목표 좌표를 주면 알아서 가는 기능.

### 실행

```bash
ros2 launch ~/launch/lekiwi_nav.launch.py abo:=false
ros2 launch ~/launch/lekiwi_nav.launch.py abo:=false map:=/home/roboseasy/maps/map_0827_1534_clean.yaml
```

센서 → Nav2(6초) → 지도 재발행(45초) → 자동 재정합(52초) 순으로 올라간다.
5층(AMCL)을 따로 띄울 필요 없다 — Nav2 에 포함되어 있다. 카토그래퍼가 떠
있으면 **반드시 먼저 끌 것.**

자동 재정합이 엉뚱한 곳을 고를 수 있으니 끄고 직접 잡으려면
`auto_relocalize:=false`.

### 확인

```bash
for n in map_server amcl controller_server planner_server bt_navigator; do
  echo -n "$n: "
  ros2 service call /$n/get_state lifecycle_msgs/srv/GetState 2>/dev/null \
    | grep -oP "label=.\K[a-z]+" | head -1
done
```

전부 `active` 여야 한다. 그다음 5층과 똑같이 위치를 잡는다
(`set_pose.py` → `relocalize.sh`).

### 데스크톱 RViz

```bash
bash ~/leisaac/scripts/run_rviz_nav2.sh          # SetInitialPose / SetGoal 도구 포함
```

지도는 데스크톱이 직접 읽어 `/map_view` 로 발행한다. 로봇의 `/map` 은
TRANSIENT_LOCAL 이라 WiFi 너머 늦은 구독자에게 잘 전달되지 않기 때문이다.

### 실패하면

- **`Goal failed` + `Control loop missed its desired rate`** → CPU 포화.
  Pi 4 에서 제어 주기 10 Hz 는 실측 7.7 Hz 밖에 안 나왔고, 그 미달이 BT 의
  `compute_path_to_pose` 대기 타임아웃을 유발했다. 현재 **5 Hz** 로 낮춰져
  있다. 0.10 m/s 에서 2 cm 마다 재계획하므로 충분하다.

- **경로 계획이 아예 시작을 못 한다** → 고립 점유 셀이 팽창해 로봇 칸을
  덮었다. 4층의 `despeckle_map.py`.

- **텔레옵이 아무 반응이 없다** → `collision_monitor` 가 마지막 관문인데,
  라이다가 없거나 스캔이 1초보다 오래되면 **모든 명령을 조용히 버린다.**
  우회하려고 `/cmd_vel` 에 직접 발행하면 안전장치가 통째로 사라진다.

- **옴니인데 옆으로 안 간다** → `motion_model` 이 기본값 `DiffDrive` 면
  `vy` 가 통째로 무시된다. `"Omni"` 여야 한다.

---

## 7층 — abo 왕복 미션

**만드는 것**: “이동 → pick → 복귀” 한 사이클.

### 실행

6층 런치에 이미 포함되어 있다 (`abo` 기본값 `true`). 58초 시점에 올라온다.

```bash
ros2 launch ~/launch/lekiwi_nav.launch.py
```

6층을 `abo:=false` 로 띄웠다면 따로:

```bash
bash ~/run_abo.sh
```

### 사용

```bash
ros2 topic pub --once /abo/command std_msgs/String "{data: 'fetch center'}"
ros2 topic echo /abo/status                                          # 지켜보기
ros2 topic pub --once /abo/pick_done std_msgs/Bool "{data: true}"    # pick 담당자
```

목적지는 `~/waypoints.yaml` 에 등록한다. **좌표는 지도마다 다르다** — 지도의
(0,0) 은 그 SLAM 을 시작한 자리이므로 지도를 새로 따면 전부 다시 잡아야 한다.

### 서보 버스 인계

pick 은 ZMQ 로 따로 구현된다. 같은 `/dev/ttyACM0` 을 쓰기 때문에 브리지가
도착 시 `set_bus(False)` 로 버스를 넘기고, `pick_done` 을 받으면
`set_bus(True)` 로 회수한다.

> **회수 뒤에는 반드시 재정합해야 한다.** 양보 중에는 바퀴가 얼마나 돌았는지
> 알 수 없어서 오도메트리가 끊긴다. 브리지는 도착 지점 좌표를 기준으로
> `refine_pose.py --at` 을 자동 실행한다.

---

## 다른 르키위로 옮기기

```bash
~/miniconda3/envs/leisaac/bin/python ~/leisaac/scripts/deploy_lekiwi.py <대상IP>
```

주소 규칙: `lekiwi0N` = `192.168.0.20N` (고정 IP).

**코드는 전 기체 공통이다.** `*.py`, `run_*.sh`, `*.lua`, `nav2_*.yaml` 은
고치지 않는다. 기체 고유값은 **`lekiwi_profile.sh` 하나에만** 있다.

옮긴 뒤 이 문서를 **1층부터 다시** 따라가면서 아래를 재확인한다.

| 값 | 방법 | 층 |
|---|---|---|
| `LEKIWI_LIDAR_YAW` | **직진 0.5 m 주행** — 자기반사만으로는 확정 불가 | 1 |
| `LEKIWI_WHEEL_BLIND` | `deadbeam.py` (LIDAR_YAW 바뀌면 같이 바뀜) | 2 |
| `LEKIWI_BASE_R` | 3바퀴 제자리 회전 | 3 |
| `LEKIWI_WHEEL_R` | 직진 1 m | 3 |
| `LEKIWI_WHEEL_SIGN` | 바퀴가 반대로 도는지 | 3 |

그다음 지도를 새로 따고 `waypoints.yaml` 좌표를 다시 잡는다.

---

## 진단 도구

| 도구 | 용도 |
|---|---|
| `check_config.py` | 설정 3중 일치·장치·TF·토픽 점검 (`--layer N`) |
| `odom_test.py` | 오도메트리 왕복 닫힘 + 축척 보정값 |
| `lidar_test.py` | 라이다 장착각 실측 |
| `procchk.py` | 실행 중인 노드 (`pgrep -f` 자기 매칭 문제 없음) |
| `tf_chain.py` | `/tf` 의 링크별 주기 — 어느 층이 빠졌는지 |
| `scan_servo.py` | 서보 버스 스캔 |
| `set_pose.py X Y YAW도` | 아는 위치를 강제 설정 |
| `relocalize.sh` | 현재 위치 미세 보정 |

## 증상별 빠른 참조

| 증상 | 층 | 확인 |
|---|---|---|
| 노드가 다 켜졌다고 나오는데 부하가 0 | 0 | `pgrep -f` 자기 매칭. `procchk.py` 쓸 것 |
| 서보 전부 무응답, 포트는 열림 | 0 | 본체 전원 |
| `/odom` 은 나오는데 TF 가 없다 | 3 | `run_base.sh` 에 `--tf` 빠짐 |
| `multiple access on port?` | 3 | 두 프로세스가 서보 버스 동시 사용 |
| 방 한가운데 가짜 벽 | 2 | `--min-intensity`, `max_range 4.5` |
| 지도가 두 겹 | 4 | 카토그래퍼가 센서보다 먼저 떴다 |
| `Waiting for map....` | 5 | `/map_server/load_map` 호출 |
| 파티클이 안 뜬다 | 5 | 정상. 움직이거나 `request_nomotion_update` |
| 노란 화살표가 옛 자리에 | 5 | RViz Odometry 디스플레이 껐다 켜기 |
| `Goal failed` + `Control loop missed` | 6 | CPU 포화 |
| `ModuleNotFoundError: rclpy` | 전체 | `/usr/bin/python3` 명시 |
| SSH 가 끊긴다 (exit 144/255) | 전체 | `pkill -f` 가 자기 셸을 죽였다. PID 지정 |
