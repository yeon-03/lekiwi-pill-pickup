"""② 오도메트리 시험 런치.  3층(odom -> base_link)이 정확한지 잰다.

  ros2 launch lekiwi_odom_test.launch.py                      # 3바퀴 왕복 회전
  ros2 launch lekiwi_odom_test.launch.py test:=line dist:=0.3 # 직진
  ros2 launch lekiwi_odom_test.launch.py measured_deg:=1050   # 실측을 넣어 BASE_R 계산
  ros2 launch lekiwi_odom_test.launch.py test:=line measured:=0.28

회전 시험을 먼저 할 것 -- 제자리라 안전하다. 직진은 앞을 비우고.
라이다와 IMU 는 띄우지 않는다(이 시험에 필요 없다).

주의: 로봇이 스스로 움직인다. collision_monitor 가 없으니 지켜볼 것.
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
        DeclareLaunchArgument("test", default_value="spin", description="spin | line"),
        DeclareLaunchArgument("turns", default_value="3.0"),
        DeclareLaunchArgument("dist", default_value="0.30"),
        DeclareLaunchArgument("wz", default_value="0.15",
                              description="회전 속도(rad/s). 미끄러짐 비교 시 값을 바꿔 재실행"),
        DeclareLaunchArgument("measured", default_value="",
                              description="직진 실측 거리(m). 주면 WHEEL_R 보정값 계산"),
        DeclareLaunchArgument("measured_deg", default_value="",
                              description="회전 실측 각도(도). 주면 BASE_R 보정값 계산"),
        DeclareLaunchArgument("delay", default_value="8.0"),
    ]
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(HERE, "lekiwi_sensors.launch.py")),
        launch_arguments={"layer": "3", "use_imu": "false", "use_lidar": "false"}.items())

    # 빈 문자열이면 옵션 자체를 넘기지 않아야 하므로 노드가 알아서 거른다.
    test = ExecuteProcess(
        cmd=["/usr/bin/python3", "-u", f"{HOME}/odom_test.py",
             "--test", LaunchConfiguration("test"),
             "--turns", LaunchConfiguration("turns"),
             "--dist", LaunchConfiguration("dist"),
             "--wz", LaunchConfiguration("wz"),
             "--measured", LaunchConfiguration("measured"),
             "--measured-deg", LaunchConfiguration("measured_deg")],
        name="odom_test", output="screen")

    return LaunchDescription(args + [
        sensors,
        TimerAction(period=LaunchConfiguration("delay"), actions=[test]),
        RegisterEventHandler(OnProcessExit(target_action=test,
                                           on_exit=[EmitEvent(event=Shutdown())])),
    ])
