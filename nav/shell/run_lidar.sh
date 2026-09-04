#!/usr/bin/env bash
# 라이다 노드. 기본은 전방위(360도).
#
# 시야를 좁히려면 환경변수를 준다. start_all.sh 에서도 그대로 전달되므로
# SLAM 스택 전체를 시야 제한 상태로 돌릴 수 있다 -- 쉘을 따로 띄울 필요 없다:
#
#   LIDAR_FOV_WIDTH=60 bash ~/start_all.sh          # 정면 60도로 SLAM
#   LIDAR_FOV_WIDTH=60 bash ~/run_lidar.sh          # 라이다만
#   LIDAR_FOV_CENTER=-16.75 LIDAR_FOV_WIDTH=60 ...  # 바퀴 사각지대를 피한 60도
#
# 각도는 base_link 기준이다 (lidar_link 는 -30도 틀어져 있고 노드가 변환한다).
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
# 기체 고유값은 코드가 아니라 프로파일에서 온다. 다른 기체로 옮길 때는
# lekiwi_profile.sh 만 고치면 된다.
[ -f "$HOME/lekiwi_profile.sh" ] && source "$HOME/lekiwi_profile.sh"

# 이미 떠 있는 라이다 노드를 정리한다. 같은 /dev/ttyUSB0 를 두 프로세스가
# 잡으면 "device reports readiness to read but returned no data" 로 죽는다.
if pkill -f "[y]dlidar_node" 2>/dev/null; then
  echo "기존 라이다 노드 종료"
  for _ in $(seq 20); do
    fuser /dev/ttyUSB0 >/dev/null 2>&1 || break
    sleep 0.25
  done
fi

# 바퀴 마스킹은 기본 켜짐. 옴니 바퀴 세 개가 시야의 30% 를 가리는데, 그 방향의
# 반환은 실제 장애물이 아니고 그 방향의 미탐지도 "비어 있음"이 아니다.
# 끄려면 LIDAR_MASK_WHEELS=0.
EXTRA=()
if [ "${LIDAR_MASK_WHEELS:-1}" != "0" ]; then
  EXTRA+=(--mask-wheels --wheel-margin "${LIDAR_WHEEL_MARGIN:-2.0}")
fi
# 약한 반사 제거. 실측으로 허위 거리(방 한가운데 가짜 벽)의 주원인이었다.
# 끄려면 LIDAR_MIN_INTENSITY=0.
EXTRA+=(--min-intensity "${LIDAR_MIN_INTENSITY:-25}")

# 스캔 디스큐: 회전 중 왜곡을 /odom 각속도로 보정한다. 기본 켜짐.
# 베이스 노드(/odom)가 없으면 wz=0 이라 보정이 0 이 될 뿐 오동작하지는 않는다.
# 끄려면 LIDAR_DESKEW=0.
if [ "${LIDAR_DESKEW:-1}" != "0" ]; then
  EXTRA+=(--deskew)
fi
# 미탐지를 inf 로 내보내면 카토그래퍼가 그 방향을 비워 낡은 점유를 지울 수 있다.
# 바퀴 마스킹과 함께 써야 안전하다. 기본 꺼짐:  LIDAR_NO_RETURN_INF=1 로 켠다.
if [ "${LIDAR_NO_RETURN_INF:-0}" != "0" ]; then
  EXTRA+=(--no-return-inf)
fi

FOV_ARGS=()
if [ -n "${LIDAR_FOV_WIDTH:-}" ] && [ "${LIDAR_FOV_WIDTH}" != "0" ]; then
  FOV_ARGS=(--fov-center "${LIDAR_FOV_CENTER:-0.0}" --fov-width "${LIDAR_FOV_WIDTH}")
  echo "시야 제한: 중심 ${LIDAR_FOV_CENTER:-0.0}°, 폭 ${LIDAR_FOV_WIDTH}° (base_link 기준)"
fi

exec python3 -u "$HOME/ydlidar_node.py" "${EXTRA[@]}" "${FOV_ARGS[@]}" "$@"
