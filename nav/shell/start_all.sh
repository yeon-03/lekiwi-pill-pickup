#!/usr/bin/env bash
# lekiwi06 실기 스택 전체 기동. 순서가 중요하다 --
# rsp(TF) -> 센서 -> 베이스 -> 카토그래퍼. 카토그래퍼가 먼저 뜨면 TF 가 없어
# 실패하고, 센서보다 먼저 뜨면 빈 데이터로 첫 서브맵을 만들어 지도가 두 겹이 된다.
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
cd "$HOME"
bash "$HOME/stop_all.sh" >/dev/null 2>&1
sleep 2
# 시야 제한은 환경변수로 전달된다 (run_lidar.sh 가 읽는다). SLAM 스택 전체를
# 좁은 시야로 돌려보는 실험용:  LIDAR_FOV_WIDTH=60 bash ~/start_all.sh
if [ -n "${LIDAR_FOV_WIDTH:-}" ] && [ "${LIDAR_FOV_WIDTH}" != "0" ]; then
  echo "!! 라이다 시야 제한 중심 ${LIDAR_FOV_CENTER:-0.0}° 폭 ${LIDAR_FOV_WIDTH}° (base_link 기준)"
  echo "!! 이 시야로는 카토그래퍼가 제대로 정합하지 못한다. 실험용으로만 쓸 것."
fi

echo "1/4 robot_state_publisher (TF)"
setsid nohup bash run_rsp.sh   > "$HOME/rsp.log"        2>&1 < /dev/null & sleep 2
echo "2/4 라이다 -> /scan"
setsid nohup bash run_lidar.sh --verbose > "$HOME/lidar_node.log" 2>&1 < /dev/null & sleep 3
echo "3/4 IMU -> /imu   (바이어스 추정 8초, 로봇을 건드리지 마세요)"
setsid nohup bash run_imu.sh --verbose   > "$HOME/imu_node.log"   2>&1 < /dev/null & sleep 11
echo "4/4 베이스 -> /odom + TF"
setsid nohup bash run_base.sh --tf       > "$HOME/base_node.log"  2>&1 < /dev/null & sleep 3
echo "    카토그래퍼"
bash "$HOME/start_cartographer.sh"
echo
echo "확인: ros2 node list   /   tail -f ~/cartographer.log"
