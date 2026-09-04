#!/usr/bin/env bash
# 현재 스캔이 어느 지도에 가장 잘 맞는지 비교한다. 읽기 전용, 로봇을 안 움직인다.
#
# 지도마다 원점(= 그 SLAM 을 시작한 자리)이 다르므로, 로봇이 어느 지도의
# 시작 지점 근처에 있는지에 따라 맞는 지도가 달라진다.
#
# 전제: 라이다 노드만 떠 있으면 된다 (Nav2 불필요).
#   bash ~/run_rsp.sh &  bash ~/run_lidar.sh &
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
[ -f "$HOME/lekiwi_profile.sh" ] && source "$HOME/lekiwi_profile.sh"

for m in "$HOME"/maps/*.yaml; do
  echo "=============================================="
  echo "지도: $(basename "$m")"
  timeout 400 python3 -u "$HOME/find_pose.py" --map "$m" --global 2>&1 \
    | grep -E "^   x |정합|최적" | tail -3
done
echo "=============================================="
echo "정합도가 가장 높은 지도를 쓸 것. 80% 이상이면 신뢰할 만하다."
echo "그 지도로 Nav2 를 띄우려면:"
echo "   bash ~/start_nav.sh ~/maps/<고른지도>.yaml"
