#!/usr/bin/env python3
"""lekiwi06 에서 검증한 구성을 다른 르키위로 이식한다.

  python nav/deploy/deploy_lekiwi.py lekiwi07            # 이식
  python nav/deploy/deploy_lekiwi.py lekiwi07 --dry-run  # 뭘 보낼지만 확인

공유 코드는 모든 기체에서 동일하게 유지된다. 기체마다 다른 값은 로봇의
~/lekiwi_profile.sh 한 곳에만 있다. 그 기체의 교정값이 저장소
nav/robots/<이름>/ 에 있으면 그걸 보내고, 없으면 새 프로파일을 만든다.
새 프로파일은 기계적 상수를 전부 미검증으로 표시하므로, 그대로 쓰면
lekiwi06 의 값이 기본으로 적용된다 -- 반드시 교정 절차를 거칠 것.

SSH 비밀번호는 코드에 두지 않는다. LEKIWI_SSH_PASS 환경변수나 --password.
"""
import sys, os, hashlib, argparse
import paramiko

# 이 스크립트는 nav/deploy/ 에 있고, 보낼 것들은 nav/ 아래에 갈래별로 있다.
SRC = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# 모든 기체에서 동일한 파일. 저장소 안에서는 갈래별 디렉터리에 있지만,
# 로봇에는 전부 홈 디렉터리에 평평하게 놓인다 (런치와 도구들이 ~/이름 으로
# 서로를 찾기 때문). 그래서 목적지는 basename 만 쓴다.
SHARED = [
    # 센서/구동 노드
    "nodes/ydlidar_node.py", "nodes/bmi160_node.py", "nodes/lekiwi_base_node.py",
    # 위치추정 / 미션
    "mission/refine_pose.py", "mission/set_pose.py", "mission/relocalize.sh",
    "mission/keep_particles.py", "mission/find_pose.py", "mission/align_helper.py",
    "mission/send_goal.py", "mission/abo_nav_bridge.py", "mission/run_abo.sh",
    "mission/waypoints.yaml",
    # 시험 / 진단
    "tools/check_config.py", "tools/odom_test.py", "tools/lidar_test.py",
    "tools/deadbeam.py", "tools/procchk.py", "tools/tf_chain.py",
    "tools/scan_servo.py", "tools/probe_ports.py", "tools/compare_maps.sh",
    # 셸 (런치로 대체됐지만 텔레옵 등 일부는 여전히 필요)
    "shell/run_base.sh", "shell/run_imu.sh", "shell/run_lidar.sh",
    "shell/run_lidar_front60.sh", "shell/run_rsp.sh", "shell/run_teleop.sh",
    "shell/run_amcl.sh", "shell/run_nav2.sh",
    "shell/start_all.sh", "shell/stop_all.sh", "shell/start_nav.sh",
    "shell/stop_nav2.sh", "shell/start_cartographer.sh", "shell/stop_cartographer.sh",
    # 설정 / 설치
    "config/nav2_lekiwi.yaml",
    "install/install_ros2_jazzy.sh", "install/install_docker.sh",
    # URDF 와 카토그래퍼 설정. 파일명에 기체 번호를 넣지 않는다 --
    # SD카드 하나에 로봇 하나이므로 번호가 정보를 더해주지 않고, 대신
    # 모든 소비자(런치·점검 도구)가 파일명을 추측해야 해서 "다른 기체
    # 파일을 가리키는" 버그가 반복됐다. 기체 이름은 프로파일 안에 있다.
    "config/lekiwi.urdf", "config/lekiwi_cartographer.lua",
]

# 통째로 보내는 디렉터리. (원본 하위경로, 대상 하위경로)
DIRS = [("launch", "launch")]
RENAMED = []      # 파일명에 기체 번호를 넣지 않는다 (위 주석 참조)

PROFILE = '''#!/usr/bin/env bash
# ===========================================================================
#  {name} 고유값.  deploy_lekiwi.py 가 생성했다.
#  다른 기체로 옮길 때 고치는 파일은 여기 하나뿐이다.
#
#  !! 아래 값들은 아직 lekiwi06 에서 복사한 것이다. 이 기체에서 측정한 값이
#  !! 아니다. 교정 전까지는 위치추정이 틀릴 수 있다.
# ===========================================================================
export LEKIWI_NAME="{name}"

# --- 구동 기구학 -----------------------------------------------------------
# [미검증] 3바퀴 제자리 회전으로 확인할 것. 제자리 회전이라 테이블에서도 안전.
export LEKIWI_BASE_R="0.13647"
# [미검증] 직진 시험 필요.
export LEKIWI_WHEEL_R="0.05"
# [미검증] 배선이 다르면 뒤집힌다. 무동력으로 확인 가능.
export LEKIWI_WHEEL_SIGN="1 1 1"

# --- 라이다 장착 -----------------------------------------------------------
# [미검증] lekiwi06 은 0.0 이었다. 옴니휠 자기반사로는 확정할 수 없다 --
# 세 바퀴가 120도 간격이라 세 후보가 데이터와 똑같이 맞기 때문이다.
# 확정하려면 직진 시험 한 번:
#     ros2 launch ~/launch/lekiwi_lidar_test.launch.py
# 여기를 바꾸면 lekiwi.urdf 의 lidar joint rpy 와
# launch/lekiwi_sensors.launch.py 의 DEFAULTS["lidar_yaw"] 도 같이 바꿀 것.
# 세 곳이 어긋났는지는 lekiwi_setup.launch.py 가 점검한다.
export LEKIWI_LIDAR_YAW="0.0"

# [미검증] deadbeam.py 로 측정. LEKIWI_LIDAR_YAW 를 바꾸면 같이 바뀐다.
export LEKIWI_WHEEL_BLIND="179.5:38.5,61.0:35.5,-59.3:34.0"

# --- 장치 -----------------------------------------------------------------
export LEKIWI_LIDAR_PORT="/dev/ttyUSB0"
export LEKIWI_SERVO_PORT="/dev/ttyACM0"
export LEKIWI_URDF="$HOME/lekiwi.urdf"
export LEKIWI_CARTO_LUA="lekiwi_cartographer.lua"

export ROS_DOMAIN_ID=42
'''

CHECKLIST = """
================= 이식 후 교정 절차 =================
소프트웨어는 전부 옮겨졌다. 남은 것은 이 기체의 물리 상수 확인이다.
브링업 가이드(bringup-guide.md)의 층 순서와 같다.

ROS2 가 없으면 먼저:
       bash ~/install_ros2_jazzy.sh        # 20~40분

[0~1층: 정지 상태 -- 위험 없음]
  1. 설정/장치/정적 TF 점검
       ros2 launch ~/launch/lekiwi_setup.launch.py layer:=1
       ros2 launch ~/launch/lekiwi_setup.launch.py layer:=3
     lidar_yaw 가 세 곳(profile/launch/urdf)에서 일치하는지도 여기서 본다.

  2. 바퀴 사각지대 측정  ->  LEKIWI_WHEEL_BLIND
       ros2 launch ~/launch/lekiwi_sensors.launch.py layer:=2 &
       python3 ~/deadbeam.py

  3. IMU 가 있다면 축 방향: 평평한 곳에서 imu_link 가속도가 (0,0,+9.81) 인지
     IMU 가 없는 기체라면 lekiwi_cartographer.lua 에서
       tracking_frame = "base_link" / use_imu_data = false 로 고칠 것.
     안 고치면 카토그래퍼가 IMU 를 기다리며 조용히 멈춘다.

[3층: 제자리 회전 -- 테이블 위에서도 안전]
  4. 오도메트리  ->  LEKIWI_BASE_R
       ros2 launch ~/launch/lekiwi_odom_test.launch.py
       (실제 회전량을 재서)  measured_deg:=<실제도> 로 다시 실행

[바닥에서, 앞에 1 m 공간]
  5. 직진  ->  LEKIWI_WHEEL_R
       ros2 launch ~/launch/lekiwi_odom_test.launch.py test:=line dist:=0.30
       (자로 재서)  measured:=<실제m> 로 다시 실행

  6. 라이다 장착각  ->  LEKIWI_LIDAR_YAW      ** 가장 중요 **
       ros2 launch ~/launch/lekiwi_lidar_test.launch.py
     회전으로는 잴 수 없다. 라이다가 회전 중심에 있어 제자리 회전은
     장착각과 무관하게 스캔이 겹치기 때문이다. 반드시 직진이 필요하다.

  ** 4~6번을 건너뛰면 lekiwi06 의 값이 그대로 쓰인다. base_radius 는
     lerobot 공식값이 9% 틀렸던 전례가 있고, 라이다 장착각이 30도 틀어진
     채로 만든 지도를 통째로 버린 적도 있다. **

교정 결과는 코드가 아니라 ~/lekiwi_profile.sh 에만 반영할 것
(단 lidar_yaw 는 urdf/launch 까지 세 곳).

지도와 waypoints.yaml 은 기체마다 새로 만들어야 한다 -- 지도의 (0,0) 은
그 SLAM 을 시작한 자리다.
====================================================
"""


# 로봇에서 셸이 읽는 파일에 CRLF 가 섞이면 값 끝에 \r 이 붙는다.
# LEKIWI_LIDAR_PORT="/dev/ttyUSB0\r" 는 존재하지 않는 경로가 되고, 증상은
# 한참 위층에서야 "라이다 없음" 으로 나타난다 (lekiwi01 프로파일이 실제로
# 이랬다). 보내기 직전에 한 번 정규화한다.
TEXT_EXT = (".sh", ".py", ".yaml", ".lua", ".urdf", ".rviz", ".txt", ".md")


def to_unix(data, label):
    if not label.endswith(TEXT_EXT) or b"\r\n" not in data:
        return data
    print(f"  [주의] {label} 이 CRLF 였다 -- LF 로 고쳐 보낸다")
    return data.replace(b"\r\n", b"\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("target", help="대상 기체 이름 (예: lekiwi07) 또는 IP")
    ap.add_argument("--ip", help="IP 를 직접 지정 (기본: lekiwi0N -> 192.168.0.20N)")
    ap.add_argument("--user", default="roboseasy")
    # 비밀번호를 코드에 박아두지 않는다. 환경변수로 주거나 --password 로 준다.
    ap.add_argument("--password", default=os.environ.get("LEKIWI_SSH_PASS", ""))
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--reset-profile", action="store_true",
                    help="기존 lekiwi_profile.sh 를 템플릿으로 덮어쓴다 (교정값 소실)")
    a = ap.parse_args()

    name = a.target
    ip = a.ip
    if ip is None:
        if name.startswith("lekiwi") and name[6:].isdigit():
            ip = "192.168.0.2%02d" % int(name[6:])
        else:
            ip = name
    print(f"대상: {name}  ({ip})")

    plan = []
    for f in SHARED:
        # 저장소에서는 갈래별 디렉터리, 로봇에서는 홈에 평평하게.
        plan.append((os.path.join(SRC, f), os.path.basename(f)))
    for src, tmpl in RENAMED:
        plan.append((os.path.join(SRC, src), tmpl.format(name=name)))
    for sub, dst in DIRS:
        d = os.path.join(SRC, sub)
        for f in sorted(os.listdir(d)) if os.path.isdir(d) else []:
            if f.endswith(".py"):
                plan.append((os.path.join(d, f), f"{dst}/{f}"))

    missing = [s for s, _ in plan if not os.path.isfile(s)]
    if missing:
        print("원본 파일 없음:")
        for m in missing:
            print("   ", m)
        return 1

    print(f"\n보낼 파일 {len(plan)}개 (+ lekiwi_profile.sh: 없을 때만 생성)")
    for s, d in plan:
        tag = "" if os.path.basename(s) == d else f"   <- {os.path.basename(s)}"
        print(f"   ~/{d}{tag}")
    if a.dry_run:
        print("\n--dry-run: 실제로 보내지 않았다.")
        print(CHECKLIST)
        return 0

    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(ip, username=a.user, password=a.password, timeout=15,
              look_for_keys=False, allow_agent=False)
    sftp = c.open_sftp()

    # 기존 파일은 덮어쓰기 전에 백업한다.
    stamp_cmd = "date +%Y%m%d_%H%M%S"
    _, so, _ = c.exec_command(stamp_cmd)
    stamp = so.read().decode().strip()
    bak = f"lekiwi_backup_{stamp}"
    c.exec_command(f"mkdir -p ~/{bak}")
    # 하위 디렉터리를 먼저 만들어 둔다 (sftp 는 자동 생성하지 않는다)
    for _, dst in DIRS:
        c.exec_command(f"mkdir -p ~/{dst} ~/{bak}/{dst}")

    # 프로파일은 기체 교정값이 사는 유일한 곳이다. 이미 있으면 덮어쓰지
    # 않는다 -- 코드를 고쳐 재배포할 때마다 교정이 날아가면 안 된다.
    # 새로 만들려면 --reset-profile.
    _, so, _ = c.exec_command("[ -f ~/lekiwi_profile.sh ] && echo yes || echo no")
    has_profile = so.read().decode().strip() == "yes"
    write_profile = (not has_profile) or a.reset_profile
    if has_profile and not a.reset_profile:
        print("\n  lekiwi_profile.sh 는 이미 있어 건드리지 않는다 "
              "(교정값 보존). 새로 만들려면 --reset-profile")

    ok = 0
    todo = plan + ([(None, "lekiwi_profile.sh")] if write_profile else [])
    for s, d in todo:
        remote = d
        c.exec_command(f"[ -f ~/{remote} ] && cp ~/{remote} ~/{bak}/ 2>/dev/null")
        if s is None:
            # 이 기체의 교정값이 저장소에 기록돼 있으면 그걸 보낸다. 없으면
            # 템플릿(기계적 상수가 전부 미검증 표시)을 만들어 보낸다.
            rp = os.path.join(SRC, "robots", name, "lekiwi_profile.sh")
            if os.path.isfile(rp):
                data = open(rp, "rb").read()
                print(f"  (프로파일: 저장소 robots/{name}/ 의 교정값 사용)")
            else:
                data = PROFILE.format(name=name).encode()
        else:
            data = open(s, "rb").read()
        data = to_unix(data, remote)
        with sftp.file(remote, "wb") as fh:
            fh.write(data)
        if remote.endswith(".sh") or remote.endswith(".py"):
            sftp.chmod(remote, 0o755)
        # 무결성 확인
        _, so, _ = c.exec_command(f"md5sum ~/{remote} | cut -d' ' -f1")
        got = so.read().decode().strip()
        want = hashlib.md5(data).hexdigest()
        mark = "OK" if got == want else f"** 불일치 {got} != {want} **"
        print(f"  {remote:32s} {mark}")
        ok += (got == want)

    sftp.close()
    c.close()
    total = len(todo)
    print(f"\n{ok}/{total} 전송 확인. 기존 파일 백업: ~/{bak}/")
    print(CHECKLIST)
    return 0 if ok == total else 1


if __name__ == "__main__":
    sys.exit(main())
