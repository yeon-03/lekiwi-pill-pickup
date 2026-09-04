#!/usr/bin/env bash
# 키보드 텔레옵. 이 기체에 맞게 속도를 낮춰서 띄운다.
#
# 기본값(0.5 m/s, 1.0 rad/s)은 이 로봇의 물리 상한(vx 0.133, wz 0.843)을
# 훨씬 넘고, 회전이 빠르면 스캔 왜곡이 커져 AMCL 위치추정이 흔들린다.
# 실측: 0.60 rad/s 에서 연속 스캔의 21% 가 어긋난다 (0.15 rad/s 에서는 2.5%).
#
# 주의: /cmd_vel 로 직접 발행하므로 Nav2 의 collision_monitor 를 거치지 않는다.
#       장애물 보호가 없으니 화면을 보면서 조작할 것.
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
[ -f "$HOME/lekiwi_profile.sh" ] && source "$HOME/lekiwi_profile.sh"

SPEED="${TELEOP_SPEED:-0.06}"   # m/s  (상한 0.133 의 절반)
TURN="${TELEOP_TURN:-0.15}"     # rad/s (왜곡 2.5% 수준)

echo "속도 ${SPEED} m/s, 회전 ${TURN} rad/s"
echo "  i=전진  ,=후진  j=좌회전  l=우회전  J=좌이동  L=우이동(옴니)  k=정지"
echo "  q/z 로 전체 속도 증감 -- 회전은 0.25 rad/s 를 넘기지 말 것"
exec ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p speed:="$SPEED" -p turn:="$TURN"
