#!/usr/bin/env bash
# LeKiwi Nav2 기동. 사용법: run_nav2.sh [map.yaml]
#
# 전제: start_all.sh 로 센서/베이스가 이미 떠 있어야 한다. 단 카토그래퍼는
# 꺼야 한다 -- AMCL 과 카토그래퍼가 동시에 map->odom TF 를 발행하면 충돌한다.
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
# 인자를 주면 그 지도, 없으면 ~/maps 에서 가장 최근 것을 쓴다. 기본값을
# 파일명으로 박아두면 새 지도를 딸 때마다 낡은 지도로 조용히 돌아간다.
MAP="${1:-$(ls -t "$HOME"/maps/*.yaml 2>/dev/null | head -1)}"
[ -f "$MAP" ] || { echo "지도 없음: $MAP"; exit 1; }
echo "지도: $MAP"
exec ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=False \
  autostart:=True \
  slam:=False \
  map:="$MAP" \
  params_file:="$HOME/nav2_lekiwi.yaml"
