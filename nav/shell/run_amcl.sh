#!/usr/bin/env bash
# AMCL(위치추정)만 띄운다.  사용법: run_amcl.sh [지도.yaml]
#
# navigation_launch 를 띄우지 않으므로 controller/planner/behavior 가 아예 없다.
# 목표를 줄 수 없고 /cmd_vel 로 아무것도 나가지 않는다 -- 로봇이 움직일 수단이
# 없는 상태로 파티클 거동만 관찰할 수 있다.
#
# 전제: run_rsp.sh (TF), run_lidar.sh (/scan), run_base.sh (/odom).
#       카토그래퍼는 꺼야 한다 (AMCL 과 둘 다 map->odom 을 발행하면 충돌).
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
[ -f "$HOME/lekiwi_profile.sh" ] && source "$HOME/lekiwi_profile.sh"

MAP="${1:-$(ls -t "$HOME"/maps/*.yaml 2>/dev/null | head -1)}"
[ -f "$MAP" ] || { echo "지도 없음: $MAP"; exit 1; }
echo "지도: $MAP"
exec ros2 launch nav2_bringup localization_launch.py \
  use_sim_time:=False \
  autostart:=True \
  map:="$MAP" \
  params_file:="$HOME/nav2_lekiwi.yaml"
