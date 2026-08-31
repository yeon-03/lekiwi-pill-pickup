#!/usr/bin/env bash
# 노트북 USB 카메라를 YOLO 검출 오버레이와 함께 띄우는 원클릭 스크립트.
# 사용법:
#   ./scripts/run_camera_yolo.sh                       (기본 yolov8n.pt, 전체 클래스 표시)
#   ./scripts/run_camera_yolo.sh --model yolov8s.pt     (더 정확한(느린) 모델)
#   ./scripts/run_camera_yolo.sh --label bottle vase    (라벨 필터)
set -e
cd "$(dirname "$0")/.."
DISPLAY=:1 PYTHONUNBUFFERED=1 env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH \
    ./venv/bin/python scripts/live_view_usb_yolo.py "$@"
