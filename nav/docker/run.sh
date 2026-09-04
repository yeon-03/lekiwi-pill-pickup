#!/usr/bin/env bash
# LeKiwi 컨테이너 실행.  bash run.sh [명령...]
#
#   bash run.sh                                        # 셸
#   bash run.sh ros2 launch ~/launch/lekiwi_setup.launch.py layer:=1
#   bash run.sh ros2 launch ~/launch/lekiwi_slam.launch.py
#
# 인자가 많아서 스크립트로 감쌌다. 각 인자가 왜 필요한지는 아래 주석 참조.
set -euo pipefail
IMAGE="${LEKIWI_IMAGE:-lekiwi:jazzy}"
NAME="${LEKIWI_CONTAINER:-lekiwi}"

ARGS=(
  --rm -it
  --name "$NAME"

  # ROS 2 는 DDS 멀티캐스트로 노드를 찾는다. 브리지 네트워크에서는 컨테이너
  # 밖(데스크톱 RViz)과 서로 보이지 않는다. host 여야 한다.
  --network host
  --ipc host

  # 홈을 통째로 마운트한다. 코드·lekiwi_profile.sh·launch/·maps/ 가 전부
  # 여기 있다. 이미지에는 환경만 들어 있고 우리 것은 하나도 없다.
  -v "$HOME:$HOME"
  -w "$HOME"
  -e "HOME=$HOME"
  -e ROS_DOMAIN_ID=42
)

# --- 장치 ------------------------------------------------------------------
# 있는 것만 넘긴다. 없는 장치를 --device 로 주면 컨테이너가 아예 뜨지 않는다.
#
# !! 알아둘 것: --device 는 '지금 있는' 장치 노드를 넘긴다. 컨테이너가 뜬 뒤
#    USB 를 뽑았다 꽂으면 호스트에는 새 노드가 생기지만 컨테이너 안에서는
#    사라진 채로 남는다. 라이다를 다시 꽂았다면 컨테이너를 재시작할 것.
#    (매번 겪기 싫다면 LEKIWI_DEV_ALL=1 로 /dev 를 통째로 넘길 수 있다.
#     대신 격리가 사실상 없어지므로 기본값은 아니다.)
if [ "${LEKIWI_DEV_ALL:-0}" = "1" ]; then
  ARGS+=( --privileged -v /dev:/dev )
  echo "!! /dev 를 통째로 넘긴다 (격리 없음)"
else
  for d in /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyACM0 /dev/ttyACM1 /dev/i2c-1 \
           /dev/video0 /dev/video1 /dev/video2 /dev/video4; do
    [ -e "$d" ] && ARGS+=( --device "$d" )
  done
fi

# 서보/라이다는 dialout, IMU 는 i2c 그룹이 필요하다. 호스트의 실제 gid 를 넘긴다.
for g in dialout i2c video; do
  gid=$(getent group "$g" | cut -d: -f3 || true)
  [ -n "${gid:-}" ] && ARGS+=( --group-add "$gid" )
done

echo "이미지 $IMAGE"
exec docker run "${ARGS[@]}" "$IMAGE" "${@:-bash}"
