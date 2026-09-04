#!/usr/bin/env python3
"""정지 중에도 AMCL 이 파티클을 계속 발행하게 한다.

AMCL 은 update_min_d(0.10 m) / update_min_a(0.15 rad) 를 넘게 움직여야 한 주기를
돈다. 멈춰 있으면 /particle_cloud 도 /amcl_pose 도 나오지 않아 RViz 화면이 빈다.
/request_nomotion_update 를 주기적으로 불러 강제로 한 주기씩 돌린다.

  keep_particles.py [주기초=0.5] [지속초=0=무한]

주의: 같은 정보로 재추출을 반복하면 파티클이 인위적으로 수렴한다(파티클 고갈).
      화면 표시용으로만 쓰고, 실제 주행 중에는 끌 것.
"""
import rclpy, sys, time
from rclpy.node import Node
from std_srvs.srv import Empty

period = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5
dur    = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0

rclpy.init()
n = Node("keep_particles")
cli = n.create_client(Empty, "/request_nomotion_update")
if not cli.wait_for_service(timeout_sec=10.0):
    print("/request_nomotion_update 서비스가 없다. AMCL 이 떠 있는지 확인할 것.")
    sys.exit(1)
print(f"파티클 강제 갱신 시작 ({period}초 주기{'' if dur == 0 else f', {dur}초 동안'})")
t0 = time.time(); k = 0
try:
    while dur == 0 or time.time() - t0 < dur:
        cli.call_async(Empty.Request()); k += 1
        end = time.time() + period
        while time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.05)
except KeyboardInterrupt:
    pass
print(f"\n{k}회 호출하고 종료")
