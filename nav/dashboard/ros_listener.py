"""rclpy 구독 → DashboardState. 메시지에서 값만 꺼내 넘긴다(상태는 ROS 를 모른다).

QoS 는 발행 측에 맞춘다 (2026-09-14 `ros2 topic info -v` 로 확인):
  /scan, /particle_cloud  best_effort   /map, /amcl_pose, /tf_static  reliable + transient_local
"""
from __future__ import annotations

import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.msg import ParticleCloud
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage

from map_geometry import yaw_from_quat

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
RELIABLE = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
TF_QOS = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)


def _pose_from_transform(tr):
    t, q = tr.transform.translation, tr.transform.rotation
    return (t.x, t.y, yaw_from_quat(q.x, q.y, q.z, q.w))


def _pose_from_msg_pose(p):
    q = p.orientation
    return (p.position.x, p.position.y, yaw_from_quat(q.x, q.y, q.z, q.w))


class DashboardListener(Node):
    def __init__(self, state, particles: bool = True) -> None:
        super().__init__("lekiwi_dashboard")
        self.state = state
        sub = self.create_subscription
        sub(TFMessage, "/tf", self._safe(self._on_tf), TF_QOS)
        sub(TFMessage, "/tf_static", self._safe(self._on_tf_static), LATCHED)
        sub(Odometry, "/odom", self._safe(lambda m: state.on_odom(_pose_from_msg_pose(m.pose.pose), time.time())), RELIABLE)
        sub(LaserScan, "/scan", self._safe(self._on_scan), qos_profile_sensor_data)
        if particles:
            sub(ParticleCloud, "/particle_cloud", self._safe(lambda m: state.on_particles(
                [[round(p.pose.position.x, 3), round(p.pose.position.y, 3)] for p in m.particles], time.time())),
                qos_profile_sensor_data)
        sub(PoseWithCovarianceStamped, "/amcl_pose", self._safe(self._on_amcl), LATCHED)
        sub(Path, "/plan", self._safe(lambda m: state.on_plan(
            [[round(ps.pose.position.x, 3), round(ps.pose.position.y, 3)] for ps in m.poses], time.time())), RELIABLE)
        sub(OccupancyGrid, "/map", self._safe(lambda m: state.on_map_msg(
            m.info.width, m.info.height, m.info.resolution,
            m.info.origin.position.x, m.info.origin.position.y, time.time())), LATCHED)
        sub(String, "/abo/state", self._safe(lambda m: state.on_bridge_state(m.data, time.time())), RELIABLE)
        sub(String, "/abo/status", self._safe(lambda m: state.on_bridge_status(m.data, time.time())), RELIABLE)
        sub(String, "/abo/command", self._safe(lambda m: state.on_command(m.data, time.time())), RELIABLE)
        sub(String, "/abo/pick_request", self._safe(lambda m: state.on_pick_request(m.data, time.time())), RELIABLE)
        sub(Bool, "/abo/pick_done", self._safe(lambda m: state.on_pick_done(bool(m.data), time.time())), RELIABLE)
        self._cmd_pub = self.create_publisher(String, "/abo/command", 10)

    def _safe(self, fn):
        def wrapped(msg):
            try:
                fn(msg)
            except Exception as exc:        # 콜백 예외로 노드가 죽지 않게
                self.get_logger().error(f"콜백 오류: {type(exc).__name__}: {exc}", throttle_duration_sec=5.0)
        return wrapped

    def _on_tf(self, msg) -> None:
        now = time.time()
        for tr in msg.transforms:
            self.state.on_tf(tr.header.frame_id.lstrip("/"), tr.child_frame_id.lstrip("/"), _pose_from_transform(tr), now)

    def _on_tf_static(self, msg) -> None:
        for tr in msg.transforms:
            if (tr.header.frame_id.lstrip("/"), tr.child_frame_id.lstrip("/")) == ("base_link", "lidar_link"):
                self.state.lidar = _pose_from_transform(tr)

    def _on_scan(self, m) -> None:
        self.state.on_scan(list(m.ranges), m.angle_min, m.angle_increment, m.range_min, m.range_max, time.time())

    def _on_amcl(self, m) -> None:
        c = m.pose.covariance
        self.state.on_amcl_pose(m.pose.pose.position.x, m.pose.pose.position.y, (c[0] + c[7]) / 2.0, time.time())

    def publish_stop(self) -> None:
        self._cmd_pub.publish(String(data="stop"))
