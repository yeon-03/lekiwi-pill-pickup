#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
# 기체 고유값은 코드가 아니라 프로파일에서 온다. 다른 기체로 옮길 때는
# lekiwi_profile.sh 만 고치면 된다.
[ -f "$HOME/lekiwi_profile.sh" ] && source "$HOME/lekiwi_profile.sh"
exec python3 -u "$HOME/bmi160_node.py" "$@"
