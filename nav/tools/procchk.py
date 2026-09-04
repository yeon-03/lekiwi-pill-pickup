#!/usr/bin/env python3
"""실행 중인 스택 노드를 확인한다.  pgrep -f 는 검사 명령 자체를 매칭하므로 쓰지 않는다."""
import os, sys
WANT = [("ydlidar_node.py","라이다"), ("bmi160_node.py","IMU"),
        ("lekiwi_base_node.py","베이스"), ("robot_state_publisher","TF(rsp)"),
        ("cartographer_node","SLAM"), ("component_container_isolated","Nav2"),
        ("teleop_twist_keyboard","텔레옵"), ("abo_nav_bridge.py","abo")]
me = {os.getpid(), os.getppid()}
found = {k: [] for k, _ in WANT}
for pid in os.listdir("/proc"):
    if not pid.isdigit() or int(pid) in me: continue
    try:
        cmd = open(f"/proc/{pid}/cmdline","rb").read().decode("utf-8","replace")
    except Exception: continue
    if not cmd: continue
    parts = cmd.split("\0")
    # 셸이 -c 로 받은 스크립트 문자열은 무시한다 (자기 매칭 방지)
    if parts and os.path.basename(parts[0]) in ("bash","sh","dash") and "-c" in parts: continue
    for k, _ in WANT:
        if k in cmd: found[k].append(pid)
for k, nm in WANT:
    p = found[k]
    print("  %s %-10s %s" % ("●" if p else "○", nm, ("PID " + ",".join(p)) if p else "꺼짐"))
