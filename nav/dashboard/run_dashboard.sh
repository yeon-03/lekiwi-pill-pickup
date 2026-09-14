#!/usr/bin/env bash
# 노트북에서 대시보드 실행. 셸 기본 ROS_DOMAIN_ID(77, 에이보)와 무관하게 로봇 도메인 42 로 고정한다.
#   nav/dashboard/run_dashboard.sh [--robot-ip 223.194.139.15] [--port 8001] [--no-particles] ...
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
ROBOT_IP="223.194.139.15"
ARGS=()
while [ $# -gt 0 ]; do
    case "$1" in
        --robot-ip) ROBOT_IP="$2"; shift 2 ;;
        *) ARGS+=("$1"); shift ;;
    esac
done
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET ROS_STATIC_PEERS="$ROBOT_IP"
exec /usr/bin/python3 -u "$HERE/dashboard_node.py" "${ARGS[@]}"
