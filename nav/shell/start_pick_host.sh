#!/usr/bin/env bash
# 집기 단계에서 abo_nav_bridge.py 가 띄우는 ZMQ 호스트 (lerobot lekiwi_host).
# lekiwi_nav.launch.py 의 host_cmd 기본값이다. 직접 실행할 일은 거의 없다.
#
# 핵심은 --robot.disable_torque_on_disconnect=false 다.
#   lerobot 기본값은 True 라서, 호스트가 내려갈 때 finally 의 robot.disconnect() 가
#   팔까지 포함한 모든 모터 토크를 끈다 -- 약통을 쥔 채 복귀하는 순간 떨어뜨린다.
#   false 로 두면 호스트가 어떻게 끝나든 토크가 켜진 채 남고, 서보는 마지막 목표
#   자세를 스스로 유지한다. 그다음 바퀴 노드는 바퀴(7~9)에만 쓰므로 팔은 버틴다.
#   (lerobot 0.6.0/0.6.1 에서 draccus 파싱으로 False 가 되는 것을 확인)
#
# exec 로 파이썬을 직접 띄운다 -- 브리지가 보내는 SIGINT 가 bash 가 아니라
# lekiwi_host 에 바로 닿아야 finally 정리가 돈다.
set -e
[ -f "$HOME/lekiwi_profile.sh" ] && source "$HOME/lekiwi_profile.sh"
VENV="${LEKIWI_HOST_VENV:-$HOME/lerobot060_venv}"
ROBOT_ID="${LEKIWI_NAME:-lekiwi01}"
# 카메라는 MJPG 강제 -- YUYV 로는 640x480 두 대가 USB 대역폭에서 30fps 를 못 낸다.
CAMERAS="{front: {type: opencv, index_or_path: /dev/cam_front, fps: 30, width: 640, height: 480, fourcc: MJPG}, wrist: {type: opencv, index_or_path: /dev/cam_wrist, fps: 30, width: 640, height: 480, fourcc: MJPG}}"
exec "$VENV/bin/python" -m lerobot.robots.lekiwi.lekiwi_host \
    --robot.id="$ROBOT_ID" \
    --robot.cameras="$CAMERAS" \
    --robot.disable_torque_on_disconnect=false \
    --host.connection_time_s="${LEKIWI_HOST_SECONDS:-3600}"
