#!/usr/bin/env bash
# Nav2 종료 후 베이스를 확실히 정지시킨다.
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
# Nav2 는 개별 노드가 아니라 component_container_isolated 안에 합성되어 뜬다.
# 노드 이름만으로 pkill 하면 컨테이너가 살아남아 CPU 를 계속 먹는다.
pkill -f "component_container_isolated" 2>/dev/null
pkill -f "bringup_launch" 2>/dev/null
pkill -f "navigation_launch" 2>/dev/null
pkill -f "localization_launch" 2>/dev/null
pkill -f "nav2_bringup" 2>/dev/null
# localization_launch 로 띄우면 위 패턴에 안 걸리는 별도 바이너리로 뜬다.
pkill -f "nav2_map_server/map_server" 2>/dev/null
pkill -f "nav2_amcl/amcl" 2>/dev/null
pkill -f "nav2_lifecycle_manager" 2>/dev/null
sleep 2
# Nav2 가 죽으면 cmd_vel 이 끊기고 베이스 노드 워치독(0.5초)이 알아서 세운다.
# 그래도 명시적으로 0 을 한 번 보낸다.
timeout 3 ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1
echo "Nav2 종료, /cmd_vel 0 전송"
