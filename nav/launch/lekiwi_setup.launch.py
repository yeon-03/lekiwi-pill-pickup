"""① 설정 런치.  스택을 올리고 0~1층 설정을 점검한다.

  ros2 launch lekiwi_setup.launch.py
  ros2 launch lekiwi_setup.launch.py layer:=1     # rsp 만 올리고 점검

점검 내용
  - lidar_yaw 가 세 곳(profile / launch / URDF)에서 일치하는가
    -> 하나만 고치면 조용히 어긋나고, 증상은 한참 위층에서야 보인다
  - 라이다/서보 장치 노드가 있는가
  - base_link -> lidar_link / imu_link / base_footprint 정적 TF
  - /scan, /odom 주기와 odom -> base_link TF

점검이 끝나면 런치가 스스로 내려간다.
"""
import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            ExecuteProcess, TimerAction, RegisterEventHandler, EmitEvent)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

HOME = os.path.expanduser("~")
HERE = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    args = [
        DeclareLaunchArgument("layer", default_value="3"),
        DeclareLaunchArgument("delay", default_value="12.0",
                              description="센서가 뜬 뒤 점검까지. IMU 바이어스 8초 포함"),
    ]
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(HERE, "lekiwi_sensors.launch.py")),
        launch_arguments={"layer": LaunchConfiguration("layer")}.items())

    check = ExecuteProcess(
        cmd=["/usr/bin/python3", "-u", f"{HOME}/check_config.py",
             "--wait", "8", "--layer", LaunchConfiguration("layer")],
        name="check_config", output="screen")

    return LaunchDescription(args + [
        sensors,
        TimerAction(period=LaunchConfiguration("delay"), actions=[check]),
        RegisterEventHandler(OnProcessExit(target_action=check,
                                           on_exit=[EmitEvent(event=Shutdown())])),
    ])
