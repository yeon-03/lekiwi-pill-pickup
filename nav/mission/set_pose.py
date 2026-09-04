#!/usr/bin/env python3
"""AMCL 자세를 지정한 좌표로 직접 설정한다.  set_pose.py X Y YAW도

정합 지표를 거치지 않는다. 사람이 로봇의 실제 위치를 아는 경우 -- 손으로 옮겼거나
시작 지점에 갖다 놓은 경우 -- 지표가 엉뚱한 곳을 골라도 이걸로 강제한다.
"""
import rclpy, math, sys, time
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
x, y, yaw = float(sys.argv[1]), float(sys.argv[2]), math.radians(float(sys.argv[3]))
rclpy.init(); n = Node("set_pose")
pub = n.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
m = PoseWithCovarianceStamped(); m.header.frame_id = "map"
m.pose.pose.position.x = x; m.pose.pose.position.y = y
m.pose.pose.orientation.z = math.sin(yaw/2); m.pose.pose.orientation.w = math.cos(yaw/2)
cov = [0.0]*36; cov[0] = 0.05; cov[7] = 0.05; cov[35] = 0.02
m.pose.covariance = cov
for _ in range(6):
    m.header.stamp = n.get_clock().now().to_msg()
    pub.publish(m); rclpy.spin_once(n, timeout_sec=0.2)
print("  /initialpose <- (%+.3f, %+.3f) yaw %+.1f도" % (x, y, math.degrees(yaw)))
