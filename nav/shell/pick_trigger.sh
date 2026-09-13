#!/usr/bin/env bash
# 색상 지정 약통 왕복 미션을 한 번 트리거한다.
#
# /abo/command 에 "fetch <목적지> color:<색>" 을 1회 발행만 한다 -- 실제 왕복
# (이동 -> 집기 -> 복귀)은 abo_nav_bridge.py 가 이어서 처리한다.
# 에이보(A-Bo_project, 도메인 77)가 SSH 로 이 로봇(도메인 42)에서 부르는 용도다.
# 에이보 쪽 스킬 이름과 호출 방식은 A-Bo_project 가 정한다.
#
#   bash ~/pick_trigger.sh red              # 목적지 기본값 center (waypoints.yaml)
#   bash ~/pick_trigger.sh green center
set -e
COLOR="${1:?사용법: pick_trigger.sh <red|green|blue> [목적지=center]}"
DEST="${2:-center}"
case "$COLOR" in
    red|green|blue) ;;
    *) echo "지원하지 않는 색: $COLOR (red|green|blue)" >&2; exit 2 ;;
esac
source /opt/ros/jazzy/setup.bash
# SSH 비대화형 셸은 .bashrc 를 안 읽으므로 프로파일에서 도메인을 가져온다.
[ -f "$HOME/lekiwi_profile.sh" ] && source "$HOME/lekiwi_profile.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
exec ros2 topic pub -1 /abo/command std_msgs/msg/String "{data: 'fetch ${DEST} color:${COLOR}'}"
