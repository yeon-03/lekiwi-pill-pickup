"""LeKiwi SLAM.  센서 계층 + 카토그래퍼.

  ros2 launch lekiwi_slam.launch.py
"""
import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            ExecuteProcess, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

HOME = os.path.expanduser("~")
HERE = os.path.dirname(os.path.abspath(__file__))


def _ros_run(pkg, exe, *args):
    """ROS 실행파일을 직접 실행한다 (lekiwi_sensors.launch.py 와 같은 이유).

    `ros2 run` 은 파이썬 래퍼가 실제 바이너리를 자식으로 띄워서, launch 가
    보낸 SIGINT 가 노드까지 가지 않는다. 5초 뒤 SIGTERM 으로 승격되며
    종료할 때마다 에러가 남는다. 직접 띄우면 조용히 끝난다."""
    direct = "/opt/ros/%s/lib/%s/%s" % (
        os.environ.get("ROS_DISTRO", "jazzy"), pkg, exe)
    return [direct, *args] if os.path.exists(direct) else ["ros2", "run", pkg, exe, *args]


def generate_launch_description():
    args = [
        DeclareLaunchArgument("config_dir", default_value=HOME),
        DeclareLaunchArgument("config", default_value="lekiwi_cartographer.lua"),
        # 카토그래퍼는 IMU 를 기다리므로 센서가 먼저 떠야 한다. IMU 바이어스
        # 추정에 8초가 걸리니 넉넉히 준다.
        DeclareLaunchArgument("delay", default_value="14.0"),
    ]

    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(HERE, "lekiwi_sensors.launch.py")),
        launch_arguments={"use_imu": "true"}.items())

    carto = ExecuteProcess(
        cmd=_ros_run("cartographer_ros", "cartographer_node",
                     "-configuration_directory", LaunchConfiguration("config_dir"),
                     "-configuration_basename", LaunchConfiguration("config"),
                     "--ros-args", "-p", "use_sim_time:=false"),
        name="cartographer_node", output="screen", respawn=True, respawn_delay=5.0)

    occ = ExecuteProcess(
        cmd=_ros_run("cartographer_ros", "cartographer_occupancy_grid_node",
                     "--ros-args", "-p", "use_sim_time:=false",
                     "-p", "resolution:=0.05"),
        name="cartographer_occupancy_grid_node", output="screen",
        respawn=True, respawn_delay=5.0)

    return LaunchDescription(args + [
        sensors,
        TimerAction(period=LaunchConfiguration("delay"), actions=[carto, occ]),
    ])

