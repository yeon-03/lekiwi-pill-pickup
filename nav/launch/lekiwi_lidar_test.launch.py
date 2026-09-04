"""③ 라이다 장착각 시험 런치.  1층의 base_link -> lidar_link yaw 를 실측한다.

  ros2 launch lekiwi_lidar_test.launch.py
  ros2 launch lekiwi_lidar_test.launch.py dist:=0.5

로봇이 앞으로 조금 이동하면서 출발/도착 스캔을 겹쳐 보고, 가장 잘 포개지는
장착각을 찾는다. 설정값과 다르면 고쳐야 할 세 파일을 알려준다.

이 시험은 3층(오도메트리)이 맞다고 가정한다 -- 이동량을 odom 에서 읽기
때문이다. lekiwi_odom_test.launch.py 를 먼저 통과시킬 것.

회전으로는 장착각을 잴 수 없다. 라이다가 회전 중심에 있어서 제자리 회전만
시키면 장착각이 얼마든 스캔이 똑같이 겹친다. 반드시 직진이 필요하다.

주의: 로봇이 스스로 전진한다. 앞을 비우고 지켜볼 것.
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
        DeclareLaunchArgument("dist", default_value="0.30",
                              description="시험 주행 거리(m). 짧으면 각도가 애매해진다"),
        DeclareLaunchArgument("delay", default_value="10.0"),
    ]
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(HERE, "lekiwi_sensors.launch.py")),
        launch_arguments={"layer": "3", "use_imu": "false"}.items())

    test = ExecuteProcess(
        cmd=["/usr/bin/python3", "-u", f"{HOME}/lidar_test.py",
             "--dist", LaunchConfiguration("dist")],
        name="lidar_test", output="screen")

    return LaunchDescription(args + [
        sensors,
        TimerAction(period=LaunchConfiguration("delay"), actions=[test]),
        RegisterEventHandler(OnProcessExit(target_action=test,
                                           on_exit=[EmitEvent(event=Shutdown())])),
    ])
