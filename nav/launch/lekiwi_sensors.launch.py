"""LeKiwi 센서/구동 계층.  rsp(TF) + 라이다 + IMU + 베이스.

셸 스크립트(run_*.sh)를 대체한다. 런치로 옮긴 이유:
  - respawn: 노드가 죽어도 되살아난다. IMU 가 I2C 오류로 죽으면 카토그래퍼가
    "Queue waiting for data: (0, imu)" 로 조용히 멈춰버렸던 사고가 있었다.
  - 한 번의 Ctrl+C 로 전부 정리된다.
  - 파라미터가 한 곳에 모인다.

사용:
  ros2 launch lekiwi_sensors.launch.py
  ros2 launch lekiwi_sensors.launch.py use_imu:=false        # Nav2 용(AMCL 은 IMU 불필요)
  ros2 launch lekiwi_sensors.launch.py fov_width:=60.0       # 전방 60도만

layer 인자로 TF 체인을 한 층씩 올릴 수 있다. 브링업 가이드의 층 번호와 같다.
아래층이 틀리면 위층이 전부 조용히 틀어지므로, 처음 세팅하거나 다른 기체로
옮긴 뒤에는 한 층씩 확인하며 올릴 것.

  layer:=1   rsp 만          -> base_link -> lidar_link / imu_link  (정적 TF)
  layer:=2   + 라이다 + IMU  -> /scan, /imu
  layer:=3   + 베이스        -> odom -> base_link          (기본값)

  ros2 launch lekiwi_sensors.launch.py layer:=1
  ros2 run tf2_ros tf2_echo base_link lidar_link     # 1층 확인 후 다음 층으로
"""
import io, os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

HOME = os.path.expanduser("~")


# --- 기체 고유값 -----------------------------------------------------------
# 값은 ~/lekiwi_profile.sh 에서 읽는다. 여기 있는 것은 프로파일이 없거나
# 항목이 빠졌을 때의 대비책일 뿐이다.
#
# 예전에는 이 표가 진짜 설정이었는데, 그러면 프로파일보다 런치가 이겨서
# 다른 기체에 이식했을 때 교정한 값이 조용히 무시된다(urdf 경로도 lekiwi06
# 을 가리킨 채로 남는다). 기체 고유값이 사는 곳은 프로파일 하나여야 한다.
FALLBACK = {
    "wheel_r":     "0.05",
    "base_r":      "0.13647",
    "lidar_yaw":   "0.0",      # base_link -> lidar_link. 직진 시험으로 확인할 것
    "wheel_blind": "179.5:38.5,61.0:35.5,-59.3:34.0",
    "wheel_sign":  "1 1 1",
    "min_intensity": "25",     # 약한 반사 제거. 방 한가운데 가짜 벽의 원인이었다
    "wheel_margin":  "2.0",
}


def _profile():
    """lekiwi_profile.sh 의 export 들을 읽는다. 셸을 실행하지 않고 파싱만 한다."""
    import re
    out = {}
    p = f"{HOME}/lekiwi_profile.sh"
    if os.path.exists(p):
        for m in re.finditer(r'^\s*export\s+(LEKIWI_\w+)="?([^"\n#]*)"?',
                             io.open(p).read(), re.M):
            out[m.group(1)] = m.group(2).strip()
    return out


_P = _profile()

DEFAULTS = {
    "wheel_r":     _P.get("LEKIWI_WHEEL_R",     FALLBACK["wheel_r"]),
    "base_r":      _P.get("LEKIWI_BASE_R",      FALLBACK["base_r"]),
    "lidar_yaw":   _P.get("LEKIWI_LIDAR_YAW",   FALLBACK["lidar_yaw"]),
    "wheel_blind": _P.get("LEKIWI_WHEEL_BLIND", FALLBACK["wheel_blind"]),
    "wheel_sign":  _P.get("LEKIWI_WHEEL_SIGN",  FALLBACK["wheel_sign"]),
    # 반사 강도 필터. 벽 재질에 따라 적정값이 크게 달라 기체·장소마다 다르다
    # (lekiwi06 은 25 가 맞았고, lekiwi01 학교 방은 25 에서 유효 빔 10% 로 떨어졌다).
    "min_intensity": _P.get("LEKIWI_MIN_INTENSITY", FALLBACK["min_intensity"]),
    "wheel_margin":  FALLBACK["wheel_margin"],
    # 파일명에 기체 번호를 넣지 않으므로 고정 이름이다. 프로파일이 다른
    # 경로를 지정했다면 그쪽을 쓴다.
    "urdf": (_P.get("LEKIWI_URDF", "").replace("$HOME", HOME)
             or f"{HOME}/lekiwi.urdf"),
}


def _py(script, *args):
    """홈에 있는 파이썬 노드를 실행한다. scservo_sdk 는 lerobot venv 에만 있으므로
    PYTHONPATH 는 아래 SetEnvironmentVariable 에서 붙인다."""
    return ["/usr/bin/python3", "-u", f"{HOME}/{script}", *args]


def _ros_run(pkg, exe, *args):
    """ROS 실행파일을 직접 실행한다.

    `ros2 run` 은 파이썬 래퍼가 실제 바이너리를 **자식 프로세스**로 띄운다.
    그래서 launch 가 보낸 SIGINT 가 래퍼에서 멈추고 노드까지 가지 않는다.
    5초 뒤 SIGTERM 으로 승격되면서 종료할 때마다 에러 두 줄이 남는다:

      failed to terminate '5' seconds after receiving 'SIGINT', escalating

    바이너리를 직접 띄우면 신호가 노드로 바로 가서 조용히 끝나고, 프로세스도
    하나 줄어든다. 경로가 없으면(설치 구조가 다르면) 원래 방식으로 돌아간다.
    """
    direct = "/opt/ros/%s/lib/%s/%s" % (
        os.environ.get("ROS_DISTRO", "jazzy"), pkg, exe)
    if os.path.exists(direct):
        return [direct, *args]
    return ["ros2", "run", pkg, exe, *args]


def _from_layer(n, extra=None):
    """layer >= n 일 때 참. extra 는 'and' 로 덧붙일 조건 문자열 조각 목록."""
    parts = ["int('", LaunchConfiguration("layer"), "') >= %d" % n]
    if extra:
        parts += [" and "] + extra
    return IfCondition(PythonExpression(parts))


def generate_launch_description():
    args = [
        DeclareLaunchArgument("layer", default_value="3",
                              description="TF 체인을 어디까지 올릴지. 1=rsp, 2=+센서, 3=+베이스"),
        DeclareLaunchArgument("use_imu", default_value="true",
                              description="IMU 노드를 띄운다. SLAM 은 필요, AMCL 은 불필요"),
        DeclareLaunchArgument("use_lidar", default_value="true",
                              description="라이다 노드를 띄운다. odom 시험에는 불필요"),
        DeclareLaunchArgument("mask_wheels", default_value="true",
                              description="바퀴 사각지대 마스킹. deadbeam.py 로 잴 때는 false"),
        DeclareLaunchArgument("drive", default_value="true",
                              description="false 면 모터에 쓰지 않는다(odom/TF 만)"),
        DeclareLaunchArgument("fov_width", default_value="0.0",
                              description="라이다 시야 폭(도). 0 이면 전방위"),
        DeclareLaunchArgument("fov_center", default_value="0.0"),
        DeclareLaunchArgument("min_intensity", default_value=DEFAULTS["min_intensity"]),
        DeclareLaunchArgument("lidar_yaw", default_value=DEFAULTS["lidar_yaw"]),
        DeclareLaunchArgument("urdf", default_value=DEFAULTS["urdf"]),
    ]

    # 노드들이 읽는 환경변수. ydlidar_node.py 가 LEKIWI_LIDAR_YAW /
    # LEKIWI_WHEEL_BLIND 를, lekiwi_base_node.py 가 LEKIWI_WHEEL_R /
    # LEKIWI_BASE_R 을 참조한다.
    # scservo_sdk 가 어느 venv 에 있는지는 기체마다 다르다. lekiwi06 은
    # lerobot060_venv, lekiwi01 은 lerobot_venv 였다. 이름을 박아두면 다른
    # 기체에서 조용히 빈 경로가 되어 "scservo_sdk 없음" 으로 죽는다.
    venv_sp = ""
    import glob
    for pat in ("lerobot060_venv", "lerobot_venv", "*venv*"):
        g = sorted(glob.glob(f"{HOME}/{pat}/lib/python3.*/site-packages"))
        g = [d for d in g if os.path.isdir(os.path.join(d, "scservo_sdk"))]
        if g:
            venv_sp = g[0]
            break
    env = [
        SetEnvironmentVariable("ROS_DOMAIN_ID", "42"),
        SetEnvironmentVariable("LEKIWI_WHEEL_R", DEFAULTS["wheel_r"]),
        SetEnvironmentVariable("LEKIWI_BASE_R", DEFAULTS["base_r"]),
        SetEnvironmentVariable("LEKIWI_LIDAR_YAW", LaunchConfiguration("lidar_yaw")),
        SetEnvironmentVariable("LEKIWI_WHEEL_BLIND", DEFAULTS["wheel_blind"]),
    ]
    if venv_sp:
        env.append(SetEnvironmentVariable(
            "PYTHONPATH", os.environ.get("PYTHONPATH", "") + ":" + venv_sp))

    rsp = ExecuteProcess(
        cmd=_ros_run("robot_state_publisher", "robot_state_publisher",
                     LaunchConfiguration("urdf")),
        name="robot_state_publisher", output="screen", respawn=True, respawn_delay=2.0)

    def _lidar(mask):
        """--mask-wheels 는 값이 아니라 플래그라 인자 치환으로 켜고 끌 수 없다.
        노드를 두 벌 선언하고 조건으로 하나만 띄운다."""
        extra = (["--mask-wheels", "--wheel-margin", DEFAULTS["wheel_margin"]]
                 if mask else [])
        cond = ["'", LaunchConfiguration("use_lidar"), "' == 'true' and '",
                LaunchConfiguration("mask_wheels"),
                "' %s 'true'" % ("==" if mask else "!=")]
        return ExecuteProcess(
            cmd=_py("ydlidar_node.py", *extra,
                    "--min-intensity", LaunchConfiguration("min_intensity"),
                    "--deskew", "--verbose",
                    "--fov-center", LaunchConfiguration("fov_center"),
                    "--fov-width", LaunchConfiguration("fov_width")),
            name="ydlidar", output="screen", respawn=True, respawn_delay=3.0,
            condition=_from_layer(2, cond))

    lidar = _lidar(True)
    lidar_nomask = _lidar(False)

    imu = ExecuteProcess(
        cmd=_py("bmi160_node.py", "--verbose"),
        name="bmi160", output="screen", respawn=True, respawn_delay=3.0,
        condition=_from_layer(2, ["'", LaunchConfiguration("use_imu"), "' == 'true'"]))

    base = ExecuteProcess(
        cmd=_py("lekiwi_base_node.py", "--tf", "--sign", *DEFAULTS["wheel_sign"].split()),
        name="lekiwi_base", output="screen", respawn=True, respawn_delay=3.0,
        condition=_from_layer(3, ["'", LaunchConfiguration("drive"), "' == 'true'"]))

    base_nodrive = ExecuteProcess(
        cmd=_py("lekiwi_base_node.py", "--tf", "--no-drive",
                "--sign", *DEFAULTS["wheel_sign"].split()),
        name="lekiwi_base", output="screen", respawn=True, respawn_delay=3.0,
        # drive 가 'true' 가 아니면 무조건 no-drive. 오타로 모터가 도는 것보다
        # 안 도는 쪽이 안전하다.
        condition=_from_layer(3, ["'", LaunchConfiguration("drive"), "' != 'true'"]))

    return LaunchDescription(
        args + env + [rsp, lidar, lidar_nomask, imu, base, base_nodrive])

