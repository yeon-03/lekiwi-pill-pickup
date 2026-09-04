#!/usr/bin/env bash
# abo 명령 중개 노드.  /abo/command 를 받아 Nav2 를 부리고, pick 구간에서
# 서보 버스를 ZMQ 쪽에 양보했다가 회수한 뒤 위치를 재정합한다.
#
#   bash ~/run_abo.sh
#   bash ~/run_abo.sh --host-cmd "bash /home/roboseasy/start_lekiwi_host06.sh"
#
# --host-cmd 를 주지 않으면 ZMQ 호스트는 pick 담당자가 직접 관리한다고 본다
# (중개 노드는 버스 양보/회수만 한다).
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
[ -f "$HOME/lekiwi_profile.sh" ] && source "$HOME/lekiwi_profile.sh"

MAP="${LEKIWI_MAP:-$(ls -t "$HOME"/maps/*_clean.yaml "$HOME"/maps/*.yaml 2>/dev/null | head -1)}"
echo "재정합용 지도: $(basename "$MAP")"
exec /usr/bin/python3 -u "$HOME/abo_nav_bridge.py" --map "$MAP" "$@"
