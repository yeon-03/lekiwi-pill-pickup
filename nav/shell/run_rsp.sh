#!/usr/bin/env bash
# URDF 는 파일 경로로 넘긴다. --ros-args -p robot_description:="$(cat ...)" 는
# 여러 줄 XML 이 ROS 인자 파서를 깨뜨려서 노드가 즉시 abort 한다.
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
# 기체 고유값은 코드가 아니라 프로파일에서 온다. 다른 기체로 옮길 때는
# lekiwi_profile.sh 만 고치면 된다.
[ -f "$HOME/lekiwi_profile.sh" ] && source "$HOME/lekiwi_profile.sh"
exec ros2 run robot_state_publisher robot_state_publisher "${LEKIWI_URDF:-$HOME/lekiwi.urdf}"
