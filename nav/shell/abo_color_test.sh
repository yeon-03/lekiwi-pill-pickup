#!/usr/bin/env bash
# 에이보 발화 -> 색상 판단 -> pickup/medicine/<색> 이 노트북에 도착하는지 시험한다.
# 노트북(에이보 whisper/dialogue 가 도는 기기)에서 실행. LeKiwi 로는 보내지 않는다(--dry-run).
#
#   bash abo_color_test.sh                      # 마이크로 "빨간약 가져다줘" 라고 말해서 시험
#   bash abo_color_test.sh "빨간약 가져다줘"     # 마이크 없이 문장을 /user_input 으로 넣어서 시험
#
# 성공하면 "수신: pickup/medicine/red" 가 찍힌다. Ctrl+C 로 끝낸다.
# 에이보(laptop_bringup + 파이 pi_bringup)는 먼저 떠 있어야 한다.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
ABO_DOMAIN="${ABO_DOMAIN:-77}"

[ -n "$ROS_DISTRO" ] || source /opt/ros/jazzy/setup.bash
source "$HERE/ros_peers.sh"

python3 -u "$REPO/nav/mission/medicine_relay.py" --dry-run --abo-domain "$ABO_DOMAIN" &
RELAY=$!
trap 'kill $RELAY 2>/dev/null' EXIT

if [ -n "$1" ]; then
    export ROS_DOMAIN_ID="$ABO_DOMAIN"
    sleep 3   # 중계 노드 구독이 붙을 시간
    # dialogue_node 는 세션이 IDLE 이면 /user_input 을 무시한다 -> 웨이크워드로 세션부터 연다.
    ros2 topic pub -w 1 -t 1 /wake_word_detected std_msgs/msg/Empty "{}"
    sleep 5   # 인사말이 끝날 때까지
    ros2 topic pub -w 1 -t 1 /user_input std_msgs/msg/String "{data: '$1'}"
fi
wait $RELAY
