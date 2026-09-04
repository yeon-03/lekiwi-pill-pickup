#!/usr/bin/env bash
# 전방 60도 라이다만 띄우는 단축 실행 (TF 는 안 띄운다 -- RViz 로 볼 거면
# run_rsp.sh 도 따로 켜야 한다).
#
# SLAM 까지 같이 돌릴 거면 이 스크립트가 아니라 아래를 쓸 것. 쉘 하나로 끝난다:
#   LIDAR_FOV_WIDTH=60 bash ~/start_all.sh
export LIDAR_FOV_CENTER="${LIDAR_FOV_CENTER:-0.0}"
export LIDAR_FOV_WIDTH="${LIDAR_FOV_WIDTH:-60.0}"
exec bash "$HOME/run_lidar.sh" --verbose "$@"
