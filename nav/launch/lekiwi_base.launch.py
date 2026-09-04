"""베이스만 띄운다 (rsp + 구동/오도메트리).  run_base.sh 를 대체한다.

  ros2 launch lekiwi_base.launch.py                 # 구동 + odom + TF
  ros2 launch lekiwi_base.launch.py drive:=false    # 모터에 쓰지 않고 odom 만

라이다와 IMU 는 띄우지 않는다. 텔레옵으로 밀어보거나 오도메트리만 볼 때 쓴다.

셸 대신 런치를 쓰는 이유
  - run_base.sh 는 --tf 를 손으로 붙여야 했다. 빠뜨리면 /odom 은 나오는데
    odom -> base_link TF 가 없어서 위층이 전부 실패하는데, 정작 /odom 은 잘
    보이니 원인을 찾기 어렵다. 런치는 항상 붙인다.
  - 노드가 죽으면 respawn 으로 되살아난다.
  - Ctrl+C 한 번으로 정리된다.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

HERE = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    args = [
        DeclareLaunchArgument("drive", default_value="true",
                              description="false 면 모터에 쓰지 않는다(odom/TF 만)"),
    ]
    # rsp(정적 TF)는 항상 함께 띄운다. 베이스만 있고 base_link->lidar_link 가
    # 없으면 위층에서 TF 를 못 찾는데 원인 찾기가 번거롭다.
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(HERE, "lekiwi_sensors.launch.py")),
        launch_arguments={
            "layer": "3",
            "use_lidar": "false",
            "use_imu": "false",
            "drive": LaunchConfiguration("drive"),
        }.items())
    return LaunchDescription(args + [sensors])
