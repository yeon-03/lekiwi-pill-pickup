"""버스를 되찾을 때 팔을 붙잡아 두는 안전망. 순수 로직이라 ROS 없이 시험한다.

집기 단계가 끝나면 ZMQ 호스트가 내려가고 lekiwi_base_node 가 서보 버스를
되찾는다. 호스트를 disable_torque_on_disconnect=false 로 띄우면 팔 토크는 원래
유지되지만(start_pick_host.sh), host_cmd 를 바꾸거나 호스트가 비정상 종료하면
풀릴 수 있다. 그때 약통을 쥔 팔이 더 처지지 않게, 지금 자리에서 다시 잡는다.

규칙
  - 토크가 이미 켜져 있으면 **아무것도 쓰지 않는다** (정상 경로에서는 무변화).
  - 꺼져 있으면 목표 위치 = 현재 위치를 **먼저** 쓰고 토크를 켠다. 순서가
    반대면 토크가 켜지는 순간 옛 목표 위치로 팔이 튄다.
  - 읽기에 실패한 모터는 건드리지 않고 목록으로 돌려준다.
  - 읽은 위치 레지스터 값을 그대로 되돌려 쓰므로 인코딩(부호 비트 등)과 무관하다.
"""


def hold_arm(bus, ids, addr_torque, addr_pos, addr_goal):
    """(다시 잡은 모터 id 목록, 상태를 못 읽은 모터 id 목록) 을 돌려준다.

    bus 는 r1(id, addr) / r2(id, addr) / w1(id, addr, v) / w2(id, addr, v) 를
    가진 객체 (lekiwi_base_node.Bus). 읽기 실패는 None.
    """
    held, unread = [], []
    for i in ids:
        torque = bus.r1(i, addr_torque)
        if torque is None:
            unread.append(i)
            continue
        if torque:
            continue
        pos = bus.r2(i, addr_pos)
        if pos is None:
            unread.append(i)
            continue
        bus.w2(i, addr_goal, pos)
        bus.w1(i, addr_torque, 1)
        held.append(i)
    return held, unread
