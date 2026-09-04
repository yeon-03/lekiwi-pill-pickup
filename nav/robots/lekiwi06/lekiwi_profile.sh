#!/usr/bin/env bash
# ===========================================================================
#  이 기체 고유값.  다른 르키위로 옮길 때 고치는 파일은 여기 하나뿐이다.
#  나머지 코드(*.py, run_*.sh, *.lua, nav2 yaml)는 모든 기체에서 동일하다.
#
#  각 항목 옆의 검증 상태:
#    verified  = 실측으로 확인함
#    nominal   = 설계값 그대로, 미검증
#    UNRESOLVED= 확인 필요. 이대로 쓰면 틀릴 수 있음
# ===========================================================================
export LEKIWI_NAME="lekiwi06"

# --- 구동 기구학 -----------------------------------------------------------
# BASE_R: 3바퀴 제자리 회전으로 실측. lerobot 기본값 0.125 는 9% 틀리다.
#         제자리 회전이라 테이블 위에서도 안전하게 잴 수 있다.  [verified]
export LEKIWI_BASE_R="0.13647"
# WHEEL_R: 설계값. 직진 1 m 시험이 측정 정밀도 안에서 맞았을 뿐 정밀 검증은
#          아니다.  [nominal]
export LEKIWI_WHEEL_R="0.05"
# 바퀴 부호 (L B R). 배선이 다르면 뒤집힌다.
export LEKIWI_WHEEL_SIGN="1 1 1"

# --- 라이다 장착 -----------------------------------------------------------
# base_link -> lidar_link 의 yaw(도).  URDF 의 lidar joint 와 반드시 일치할 것.
#
#   [verified]  이 기체는 0.0 으로 확정했다 -- 직진 주행 시 진행 방향과
#   RViz 의 base_link 화살표가 일치함을 확인.  URDF 의 lidar joint rpy 도
#   0 0 0 이다.
#   주의: 다른 기체로 이식할 때는 이 값을 그대로 쓰면 안 된다.  옴니휠
#   자기반사 세 개로 각도는 1도 이내로 잡히지만 세 바퀴 중 어느 것인지는
#   정해지지 않으므로(120도 간격), 직진 0.5 m 주행 1회로 재확정할 것.
export LEKIWI_LIDAR_YAW="0.0"

# 옴니 바퀴가 가리는 구간 "중심:폭" (base_link 기준, 도). deadbeam.py 로 측정.
# LEKIWI_LIDAR_YAW 를 바꾸면 이 값도 같이 바뀐다.  [verified]
export LEKIWI_WHEEL_BLIND="179.5:38.5,61.0:35.5,-59.3:34.0"

# --- 장치 -----------------------------------------------------------------
export LEKIWI_LIDAR_PORT="/dev/ttyUSB0"
export LEKIWI_SERVO_PORT="/dev/ttyACM0"
export LEKIWI_URDF="$HOME/${LEKIWI_NAME}.urdf"
export LEKIWI_CARTO_LUA="${LEKIWI_NAME}_cartographer.lua"

# --- ROS ------------------------------------------------------------------
export ROS_DOMAIN_ID=42
