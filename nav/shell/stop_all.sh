#!/usr/bin/env bash
# lekiwi06 실기 스택 전체 정지. 바퀴 정지와 토크 해제를 확실히 한다.
source /opt/ros/jazzy/setup.bash 2>/dev/null
bash "$HOME/stop_cartographer.sh"
for p in "[y]dlidar_node" "[b]mi160_node" "[l]ekiwi_base_node" "[r]obot_state_publisher"; do
    pkill -f "$p" && echo "종료: ${p//[\[\]]/}" || echo "(없음: ${p//[\[\]]/})"
done
sleep 2
# 노드가 비정상 종료했을 경우를 대비해 바퀴를 직접 정지시킨다.
"$HOME/lerobot060_venv/bin/python" - <<'PY'
from scservo_sdk import PacketHandler, PortHandler
p = PortHandler("/dev/ttyACM0"); h = PacketHandler(0)
if p.openPort():
    p.setBaudRate(1000000)
    for i in (7, 8, 9): h.write2ByteTxRx(p, i, 46, 0)     # Goal_Speed = 0
    for i in (7, 8, 9): h.write1ByteTxRx(p, i, 40, 0)     # Torque off
    p.closePort()
    print("바퀴 속도 0 + 토크 해제 확인")
else:
    print("포트를 열 수 없음 (다른 프로세스가 쓰는 중일 수 있음)")
PY
echo "정지 완료."
