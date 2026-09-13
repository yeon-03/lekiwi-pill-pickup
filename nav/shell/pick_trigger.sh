#!/usr/bin/env bash
# A-Bo_project(dialogue_node, ROS_DOMAIN_ID=77)가 SSH로 이 로봇(lekiwi01,
# ROS_DOMAIN_ID=42)에 접속해 실행한다 -- 두 로봇은 도메인이 달라 ROS2 토픽을
# 직접 공유하지 못해서, 색상별 트리거를 SSH 커맨드 하나로 여기까지 넘긴다
# (lekiwi_control.py의 SKILL_MAP pick_red/pick_blue/pick_green 항목이 이걸 부른다).
#
# 여기서는 abo_nav_bridge.py 가 구독 중인 /abo/command 에 "fetch center
# color:<색>" 을 1회 발행만 한다 -- 실제 왕복 미션(이동/픽/복귀)은
# abo_nav_bridge.py 가 이어서 처리한다.
#
#   bash ~/pick_trigger.sh red
set -e
COLOR="${1:?사용법: pick_trigger.sh <red|blue|green>}"
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
exec ros2 topic pub -1 /abo/command std_msgs/msg/String "{data: 'fetch center color:${COLOR}'}"
