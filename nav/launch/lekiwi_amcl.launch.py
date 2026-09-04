"""LeKiwi 위치추정 계층 (5층).  센서 + AMCL + map_server.

Nav2 전체(6층)와 달리 플래너/컨트롤러가 없다. 목표를 줄 수 없고 /cmd_vel 로
아무것도 나가지 않으므로, 텔레옵으로 밀면서 위치추정만 관찰할 때 쓴다.

  ros2 launch lekiwi_amcl.launch.py
  ros2 launch lekiwi_amcl.launch.py map:=/home/roboseasy/maps/map_0827_1534_clean.yaml

주의: 카토그래퍼(4층)와 동시에 띄우면 안 된다. 둘 다 map->odom 을 발행한다.

지도 재발행을 25초 뒤에 한 번 넣어 두었다. map_server 는 /map 을
TRANSIENT_LOCAL 로 한 번만 발행하는데, AMCL 이 늦게 구독하면 놓쳐서
"Waiting for map...." 에서 멈추기 때문이다.
"""
import os, glob
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            ExecuteProcess, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

HOME = os.path.expanduser("~")
HERE = os.path.dirname(os.path.abspath(__file__))


def _newest_map():
    """*_clean 을 우선하고, 없으면 가장 최근 지도. 파일명을 박아두면 새 지도를
    딸 때마다 낡은 지도로 조용히 돌아간다."""
    c = sorted(glob.glob(f"{HOME}/maps/*_clean.yaml"), key=os.path.getmtime, reverse=True)
    if not c:
        c = sorted(glob.glob(f"{HOME}/maps/*.yaml"), key=os.path.getmtime, reverse=True)
    return c[0] if c else ""


def generate_launch_description():
    args = [
        DeclareLaunchArgument("map", default_value=_newest_map()),
        DeclareLaunchArgument("params", default_value=f"{HOME}/nav2_lekiwi.yaml"),
        DeclareLaunchArgument("delay", default_value="6.0",
                              description="센서가 뜬 뒤 AMCL 을 띄우기까지"),
        DeclareLaunchArgument("republish", default_value="true",
                              description="25초 뒤 /map 을 강제로 다시 발행"),
    ]

    # AMCL 은 IMU 를 쓰지 않는다. 3층까지만 올린다.
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(HERE, "lekiwi_sensors.launch.py")),
        launch_arguments={"use_imu": "false", "layer": "3"}.items())

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                os.popen("ros2 pkg prefix nav2_bringup").read().strip()
                or "/opt/ros/jazzy",
                "share", "nav2_bringup", "launch", "localization_launch.py")),
        launch_arguments={
            "use_sim_time": "False",
            "autostart": "True",
            "map": LaunchConfiguration("map"),
            "params_file": LaunchConfiguration("params"),
        }.items())

    republish = TimerAction(period=25.0, actions=[ExecuteProcess(
        cmd=["ros2", "service", "call", "/map_server/load_map",
             "nav2_msgs/srv/LoadMap",
             ["{map_url: ", LaunchConfiguration("map"), "}"]],
        name="map_republish", output="screen",
        condition=IfCondition(LaunchConfiguration("republish")))])

    return LaunchDescription(args + [
        sensors,
        TimerAction(period=LaunchConfiguration("delay"), actions=[localization]),
        republish,
    ])
