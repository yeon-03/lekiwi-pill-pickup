#!/usr/bin/env bash
# ===========================================================================
#  lekiwi01 고유값.  deploy_lekiwi.py 가 생성했다.
#  다른 기체로 옮길 때 고치는 파일은 여기 하나뿐이다.
#
#  !! 아래 값들은 아직 lekiwi06 에서 복사한 것이다. 이 기체에서 측정한 값이
#  !! 아니다. 교정 전까지는 위치추정이 틀릴 수 있다.
# ===========================================================================
export LEKIWI_NAME="lekiwi01"

# --- 구동 기구학 -----------------------------------------------------------
# [미검증] 3바퀴 제자리 회전으로 확인할 것. 제자리 회전이라 테이블에서도 안전.
export LEKIWI_BASE_R="0.13647"
# [미검증] 직진 시험 필요.
export LEKIWI_WHEEL_R="0.05"
# [미검증] 배선이 다르면 뒤집힌다. 무동력으로 확인 가능.
export LEKIWI_WHEEL_SIGN="1 1 1"

# --- 라이다 장착 -----------------------------------------------------------
# [미검증] lekiwi06 은 0.0 이었다. 옴니휠 자기반사로는 확정할 수 없다 --
# 세 바퀴가 120도 간격이라 세 후보가 데이터와 똑같이 맞기 때문이다.
# 확정하려면 직진 시험 한 번:
#     ros2 launch ~/launch/lekiwi_lidar_test.launch.py
# 여기를 바꾸면 lekiwi.urdf 의 lidar joint rpy 와
# launch/lekiwi_sensors.launch.py 의 DEFAULTS["lidar_yaw"] 도 같이 바꿀 것.
# 세 곳이 어긋났는지는 lekiwi_setup.launch.py 가 점검한다.
export LEKIWI_LIDAR_YAW="0.0"

# [실측 2026-08-31] deadbeam.py (강도필터 끄고 16초). 세 구간이 정확히 120도
# 간격이라 옴니휠 셋과 맞는다. lekiwi06 값(-59.3/179.5/61.0)과 1도 안에서
# 일치했다 -- 같은 설계다. LEKIWI_LIDAR_YAW 를 바꾸면 이 값도 같이 바뀐다.
export LEKIWI_WHEEL_BLIND="-59.5:41.0,-179.8:36.5,60.0:36.0"

# --- 장치 -----------------------------------------------------------------
export LEKIWI_LIDAR_PORT="/dev/ttyUSB0"
export LEKIWI_SERVO_PORT="/dev/ttyACM0"
export LEKIWI_URDF="$HOME/lekiwi.urdf"
export LEKIWI_CARTO_LUA="lekiwi_cartographer.lua"

export ROS_DOMAIN_ID=42

