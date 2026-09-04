#!/usr/bin/env bash
# lekiwi_base_node.py 를 ROS 환경 + scservo_sdk 경로로 실행한다.
# set -u 는 쓰지 않는다: /opt/ros/.../setup.bash 가 미설정 변수를 읽어서 죽는다.
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
# 기체 고유값은 코드가 아니라 프로파일에서 온다. 다른 기체로 옮길 때는
# lekiwi_profile.sh 만 고치면 된다.
[ -f "$HOME/lekiwi_profile.sh" ] && source "$HOME/lekiwi_profile.sh"
# scservo_sdk 가 든 venv 를 찾는다. 이름은 기체마다 다르다
# (lekiwi06=lerobot060_venv, lekiwi01=lerobot_venv).
SP=$(for d in "$HOME"/*venv*/lib/python3.*/site-packages; do
       [ -d "$d/scservo_sdk" ] && echo "$d" && break
     done)
[ -n "$SP" ] || echo "경고: scservo_sdk 를 못 찾았다" >&2
export PYTHONPATH="${PYTHONPATH:-}:$SP"
SIGN_ARGS=()
[ -n "${LEKIWI_WHEEL_SIGN:-}" ] && SIGN_ARGS=(--sign ${LEKIWI_WHEEL_SIGN})
exec python3 -u "$HOME/lekiwi_base_node.py" "${SIGN_ARGS[@]}" "$@"
