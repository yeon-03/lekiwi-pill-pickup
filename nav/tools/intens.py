#!/usr/bin/env python3
"""스캔 강도 분포를 본다.  min_intensity 임계를 이 기체에 맞게 정하려는 것.

임계가 너무 높으면 실제 벽 반환까지 버리고, 너무 낮으면 방 한가운데 가짜
벽이 생긴다. 라이다 유닛마다 강도 스케일이 다르므로 기체별로 확인해야 한다.

전제: 라이다를 min_intensity:=0 으로 띄워 둘 것 (거르기 전 값을 봐야 한다).
"""
import sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

rclpy.init(); n = Node("intens"); I = []
def cb(m):
    R = np.asarray(m.ranges, float); II = np.asarray(m.intensities, float)
    ok = np.isfinite(R) & (R > m.range_min) & (R < m.range_max)
    if len(II) == len(R): I.append(II[ok])
n.create_subscription(LaserScan, "/scan", cb, qos_profile_sensor_data)
sec = float(sys.argv[1]) if len(sys.argv) > 1 else 12
t0 = time.time()
while time.time() - t0 < sec: rclpy.spin_once(n, timeout_sec=0.1)
rclpy.shutdown()
if not I:
    print("  /scan 없음"); sys.exit(1)
a = np.concatenate(I)
print("  유효 반환 %d개 (스캔 %d장)" % (len(a), len(I)))
print("  강도  최소 %.0f  중앙 %.0f  평균 %.0f  최대 %.0f"
      % (a.min(), np.median(a), a.mean(), a.max()))
print("\n  임계별로 남는 비율")
for t in (0, 5, 10, 15, 20, 25, 30, 40, 60):
    keep = (a >= t).mean()
    bar = "#" * int(keep * 40)
    print("    >=%3d  %5.1f%%  %s" % (t, 100 * keep, bar))
