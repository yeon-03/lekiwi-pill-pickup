#!/usr/bin/env bash
# 자율주행(위치추정) 모드 기동.  사용법: start_nav.sh [지도.yaml]
#
# start_all.sh 와의 차이:
#   - 카토그래퍼를 띄우지 않는다. AMCL 과 둘 다 map->odom 을 발행하면 충돌한다.
#   - IMU 도 띄우지 않는다. AMCL 은 IMU 를 쓰지 않으므로 8초 바이어스 추정을
#     기다릴 이유가 없고 Pi 4 부하만 는다.
#
# 기동 후 로봇이 지도의 어디에 있는지 AMCL 에 알려줘야 한다:
#   RViz 의 2D Pose Estimate  또는  python3 ~/find_pose.py --publish
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
[ -f "$HOME/lekiwi_profile.sh" ] && source "$HOME/lekiwi_profile.sh"
cd "$HOME"

MAP="${1:-$(ls -t "$HOME"/maps/*.yaml 2>/dev/null | head -1)}"
[ -f "$MAP" ] || { echo "지도 없음: $MAP"; exit 1; }

bash "$HOME/stop_all.sh"  >/dev/null 2>&1
bash "$HOME/stop_nav2.sh" >/dev/null 2>&1
sleep 3

echo "1/4 robot_state_publisher (TF)"
setsid nohup bash run_rsp.sh   > "$HOME/rsp.log"        2>&1 < /dev/null & sleep 2
echo "2/4 라이다 -> /scan"
setsid nohup bash run_lidar.sh --verbose > "$HOME/lidar_node.log" 2>&1 < /dev/null & sleep 3
echo "3/4 베이스 -> /odom + TF"
setsid nohup bash run_base.sh --tf > "$HOME/base_node.log" 2>&1 < /dev/null & sleep 3
echo "4/4 Nav2  (지도: $MAP)"
setsid nohup bash run_nav2.sh "$MAP" > "$HOME/nav2.log" 2>&1 < /dev/null &

echo
echo "기동에 약 60초 걸린다. 확인:"
echo "    grep -c ERROR ~/nav2.log        # 0 이어야 정상"
echo "    ros2 lifecycle get /amcl        # active 여야 정상"
echo
echo "그 다음 반드시 초기 위치를 알려줄 것:"
echo "    python3 ~/find_pose.py --publish"
