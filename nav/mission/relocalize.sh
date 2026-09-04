#!/usr/bin/env bash
# 현재 AMCL 자세를 지도에 맞춰 정밀 보정한다.
#
# RViz 의 2D Pose Estimate 로 대충 찍은 뒤 이걸 실행하면 된다.
# 마우스 드래그는 ±10~15도 오차가 쉽게 나는데 여기서 잡힌다.
#
# 지표는 '스캔 끝점에서 가장 가까운 벽까지의 거리'다. 예전에 쓰던
# '점유 셀 적중률'은 허위 반환 흔적이 많은 구역을 편애해서, 엉뚱한 방을
# 92% 로 골라버린 적이 있다.
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
[ -f "$HOME/lekiwi_profile.sh" ] && source "$HOME/lekiwi_profile.sh"

MAP="${1:-$(ls -t "$HOME"/maps/*_clean.yaml "$HOME"/maps/*.yaml 2>/dev/null | head -1)}"
[ -f "$MAP" ] || { echo "지도 없음: $MAP"; exit 1; }
echo "지도: $(basename "$MAP")"
exec python3 -u "$HOME/refine_pose.py" "$MAP" --publish "${@:2}"
