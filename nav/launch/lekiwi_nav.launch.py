"""LeKiwi 자율주행.  센서 계층(IMU 제외) + Nav2.

  ros2 launch lekiwi_nav.launch.py
  ros2 launch lekiwi_nav.launch.py map:=/home/roboseasy/maps/map_0827_1534_clean.yaml

주의: 카토그래퍼와 동시에 띄우면 안 된다 -- 둘 다 map->odom 을 발행한다.
"""
import os, glob
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction, ExecuteProcess)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

HOME = os.path.expanduser("~")
HERE = os.path.dirname(os.path.abspath(__file__))


def _newest_map():
    c = sorted(glob.glob(f"{HOME}/maps/*_clean.yaml"), key=os.path.getmtime, reverse=True)
    if not c:
        c = sorted(glob.glob(f"{HOME}/maps/*.yaml"), key=os.path.getmtime, reverse=True)
    return c[0] if c else ""


def generate_launch_description():
    args = [
        DeclareLaunchArgument("map", default_value=_newest_map()),
        DeclareLaunchArgument("params", default_value=f"{HOME}/nav2_lekiwi.yaml"),
        # Nav2 의 costmap 은 map->odom->base_link TF 가 있어야 활성화된다.
        # 센서(특히 서보 초기화가 있는 베이스 노드)가 먼저 떠야 하는데, 6초는
        # 기체/부하에 따라 부족하다. 부족하면 costmap 이
        # "Timed out waiting for transform" 으로 활성화에 실패하고
        # lifecycle_manager 가 planner/bt_navigator 까지 전부 inactive 로 남긴다.
        DeclareLaunchArgument("delay", default_value="20.0"),
        DeclareLaunchArgument("auto_relocalize", default_value="true",
                              description="기동 후 자세를 지도에 자동 정합. 로봇이 "
                                          "표시된 시작 지점에 있다는 전제"),
        DeclareLaunchArgument("abo", default_value="true",
                              description="abo 명령 중개 노드(/abo/command)를 띄운다"),
        # 집기 단계에 브리지가 서보 버스를 넘겨받은 뒤 띄우는 ZMQ 호스트.
        # 예전엔 이 인자가 없어서 host_start() 가 아무것도 안 띄우고 성공으로
        # 넘어갔다 -- 누가 호스트를 계속 켜 두면 주행 중 버스를 뺏겼고(Nav2 사망),
        # 아무도 안 켜면 집기가 붙을 곳이 없었다. none 이면 띄우지 않는다(외부에서 관리 / 이동만 시험).
        # (ros2 launch 는 빈 값 "host_cmd:=" 를 받지 않는다)
        DeclareLaunchArgument("host_cmd", default_value=f"bash {HOME}/start_pick_host.sh",
                              description="집기용 ZMQ 호스트 실행 명령. none 이면 띄우지 않음"),
    ]

    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(HERE, "lekiwi_sensors.launch.py")),
        launch_arguments={"use_imu": "false"}.items())

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            "/opt/ros/jazzy/share/nav2_bringup/launch/bringup_launch.py"),
        launch_arguments={
            "use_sim_time": "False",
            "autostart": "True",
            "slam": "False",
            "map": LaunchConfiguration("map"),
            "params_file": LaunchConfiguration("params"),
        }.items())

    # map_server 가 /map 을 TRANSIENT_LOCAL 로 한 번만 쏘는데, AMCL 이 그보다
    # 먼저 구독하면 늦은 구독자 재전달이 일어나지 않아 "Waiting for map...." 로
    # 멈춘다. 기동이 끝난 뒤 강제로 재발행시킨다.
    republish = TimerAction(period=45.0, actions=[ExecuteProcess(
        cmd=["ros2", "service", "call", "/map_server/load_map",
             "nav2_msgs/srv/LoadMap",
             ["{map_url: ", LaunchConfiguration("map"), "}"]],
        name="republish_map", output="screen")])

    # AMCL 은 nav2_lekiwi.yaml 의 set_initial_pose 때문에 (0,0,0) 에서 시작한다.
    # 로봇을 바닥 표시 위에 놓았다면 그게 대략 맞고, 여기서 몇 cm 를 다듬는다.
    # 지표는 '끝점-벽 거리'다 -- 점유셀 적중률은 허위 반환 흔적이 많은 구역을
    # 편애해서 엉뚱한 방을 92% 로 고른 적이 있다.
    relocalize = TimerAction(period=52.0, actions=[ExecuteProcess(
        cmd=["/usr/bin/python3", "-u", f"{HOME}/refine_pose.py",
             LaunchConfiguration("map"), "--publish"],
        name="relocalize", output="screen",
        condition=IfCondition(LaunchConfiguration("auto_relocalize")))])

    # --map 을 반드시 넘긴다. 없으면 브리지가 "재정합할 지도를 모르겠다" 로
    # 재정합을 통째로 건너뛴다. 특히 pick 후 버스 회수 시점의 재정합이
    # 빠지면, 양보 동안 움직인 만큼의 오도메트리 공백이 복구되지 않는다.
    abo = TimerAction(period=58.0, actions=[ExecuteProcess(
        cmd=["/usr/bin/python3", "-u", f"{HOME}/abo_nav_bridge.py",
             "--map", LaunchConfiguration("map"),
             "--host-cmd", LaunchConfiguration("host_cmd")],
        name="abo_nav_bridge", output="screen", respawn=True, respawn_delay=3.0,
        condition=IfCondition(LaunchConfiguration("abo")))])

    return LaunchDescription(args + [
        sensors,
        TimerAction(period=LaunchConfiguration("delay"), actions=[nav2]),
        republish,
        relocalize,
        abo,
    ])

