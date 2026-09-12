#!/usr/bin/env bash
# lekiwi06 실기 SLAM. 시뮬과 달리 use_sim_time 은 false 이고, scan_fixer 도 없다
# (이 기체는 자기 몸 반사가 0개라 걸러낼 것이 없다).
#
# 토픽 이름이 카토그래퍼 기본값(scan / imu / odom)과 이미 같으므로 리맵이 없다.
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42

pkill -f "cartographer_node " 2>/dev/null && sleep 1
pkill -f "cartographer_occupancy_grid_node" 2>/dev/null && sleep 1

echo "cartographer_node 시작..."
setsid nohup ros2 run cartographer_ros cartographer_node \
    -configuration_directory "$HOME" \
    -configuration_basename lekiwi_cartographer.lua \
    --ros-args -p use_sim_time:=false \
    > "$HOME/cartographer.log" 2>&1 < /dev/null &
sleep 2

echo "cartographer_occupancy_grid_node 시작..."
setsid nohup ros2 run cartographer_ros cartographer_occupancy_grid_node \
    --ros-args -p use_sim_time:=false -p resolution:=0.05 -p publish_period_sec:=1.0 \
    > "$HOME/cartographer_occgrid.log" 2>&1 < /dev/null &
sleep 1
echo "완료. 로그: ~/cartographer.log, ~/cartographer_occgrid.log"
