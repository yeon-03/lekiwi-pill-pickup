#!/usr/bin/env bash
# 노트북 USB 카메라를 색상검출과 함께 띄우는 원클릭 스크립트.
# 사용법:
#   ./scripts/run_camera.sh                 (기본: 화이트밸런스 4600K, 시작시 ROI 드래그)
#   ./scripts/run_camera.sh --wb-temp 5200  (화이트밸런스 값 직접 지정)
#   ./scripts/run_camera.sh --no-roi        (영역 지정 없이 화면 전체)
#   ./scripts/run_camera.sh --roi 100 50 400 380   (저장해둔 영역값 바로 재사용)
set -e
cd "$(dirname "$0")/.."
DISPLAY=:1 PYTHONUNBUFFERED=1 env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH \
    ./venv/bin/python scripts/live_view_usb_local.py "$@"
