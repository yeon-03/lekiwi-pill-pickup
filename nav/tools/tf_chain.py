#!/usr/bin/env python3
"""/tf 에 실제로 흐르는 링크와 각 주기를 보여준다.  tf_chain.py [측정초=8]

TF 체인을 한 층씩 올릴 때 "이 층이 실제로 붙었는지" 확인하는 도구.
`ros2 topic hz /tf` 는 합계만 주므로 어느 링크가 빠졌는지 알 수 없다.

  정상(SLAM/주행):  map->odom 약 6 Hz,  odom->base_link 25 Hz
  정적 TF(base_link->lidar_link 등)는 /tf_static 이라 여기 안 나온다.
"""
import rclpy, time, sys
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

dur = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0
rclpy.init(); n = Node("tf_chain"); seen = {}
def cb(m):
    for t in m.transforms:
        k = "%s -> %s" % (t.header.frame_id, t.child_frame_id)
        seen[k] = seen.get(k, 0) + 1
n.create_subscription(TFMessage, "/tf", cb, 10)
t0 = time.time()
while time.time() - t0 < dur:
    rclpy.spin_once(n, timeout_sec=0.1)
d = time.time() - t0
if not seen:
    print("  /tf 수신 없음 -- 발행 노드가 죽었거나 아직 안 떴다")
for k, v in sorted(seen.items()):
    print("  %-26s %5.1f Hz" % (k, v / d))
