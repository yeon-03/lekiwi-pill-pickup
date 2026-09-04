#!/usr/bin/env bash
# ROS 환경과 기체 프로파일을 읽고 넘겨받은 명령을 실행한다.
set -e
source "/opt/ros/${ROS_DISTRO}/setup.bash"
# 기체 고유값. 마운트한 홈에 있으므로 이미지에는 들어가지 않는다.
[ -f "$HOME/lekiwi_profile.sh" ] && source "$HOME/lekiwi_profile.sh"
exec "$@"
