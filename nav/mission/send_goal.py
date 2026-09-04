#!/usr/bin/env python3
"""Nav2 에 좌표로 목표를 준다. RViz 없이 명령이나 코드에서 부를 수 있다.

  python3 send_goal.py 1.5 0.58            # x y (yaw 는 현재 방향 유지)
  python3 send_goal.py 1.5 0.58 90         # x y yaw(도)
  python3 send_goal.py home                # waypoints.yaml 의 이름
  python3 send_goal.py --list              # 등록된 이름 보기
  python3 send_goal.py 1.5 0.58 --no-wait  # 명령만 넣고 즉시 반환

반환 코드: 0 성공, 1 실패/취소, 2 인자 오류.
Ctrl+C 를 누르면 목표를 취소하고 로봇을 세운다.
"""
import sys, os, math, time, yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener

HERE = os.path.dirname(os.path.abspath(__file__))
WP = os.path.join(HERE, "waypoints.yaml")
if not os.path.exists(WP):
    WP = os.path.expanduser("~/waypoints.yaml")


def load_wp():
    if os.path.exists(WP):
        with open(WP) as f:
            return yaml.safe_load(f) or {}
    return {}


class Goal(Node):
    def __init__(self):
        super().__init__("send_goal")
        self.cli = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)
        self.result = None
        self.done = False

    def current_yaw(self):
        t0 = time.time()
        while time.time() - t0 < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                tf = self.buf.lookup_transform("map", "base_link", rclpy.time.Time())
                q = tf.transform.rotation
                return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            except Exception:
                pass
        return 0.0

    def send(self, x, y, yaw, wait=True):
        if not self.cli.wait_for_server(timeout_sec=10.0):
            print("navigate_to_pose 액션 서버가 없다. Nav2 가 떠 있는지 확인할 것.")
            return 1
        g = NavigateToPose.Goal()
        p = PoseStamped()
        p.header.frame_id = "map"
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.orientation.z = math.sin(yaw/2)
        p.pose.orientation.w = math.cos(yaw/2)
        g.pose = p
        print(f"목표 ({x:+.3f}, {y:+.3f}) yaw {math.degrees(yaw):+.1f}도 전송")
        fut = self.cli.send_goal_async(g, feedback_callback=self._fb)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if not gh.accepted:
            print("목표가 거부됐다 (도달 불가 지점일 수 있다).")
            return 1
        if not wait:
            print("전송만 하고 종료한다.")
            return 0
        self._gh = gh
        rf = gh.get_result_async()
        try:
            rclpy.spin_until_future_complete(self, rf)
        except KeyboardInterrupt:
            print("\n중단 요청 -- 목표를 취소한다.")
            gh.cancel_goal_async()
            rclpy.spin_once(self, timeout_sec=2.0)
            return 1
        st = rf.result().status
        # 4 = SUCCEEDED
        if st == 4:
            print("도착했다.")
            return 0
        print(f"실패 (status={st}).")
        return 1

    def _fb(self, fb):
        d = fb.feedback.distance_remaining
        n = fb.feedback.number_of_recoveries
        print(f"  남은 거리 {d:5.2f} m, 복구 {n}회", end="\r", flush=True)


def main():
    a = [x for x in sys.argv[1:] if not x.startswith("--")]
    wait = "--no-wait" not in sys.argv
    wps = load_wp()
    if "--list" in sys.argv:
        print(f"waypoints: {WP}")
        print(f"  기준 지도: {wps.get('map','(없음)')}")
        for k, v in (wps.get("points") or {}).items():
            print(f"  {k:10s} x {v['x']:+.2f}  y {v['y']:+.2f}  yaw {v.get('yaw',0):+.1f}")
        return 0
    if not a:
        print(__doc__)
        return 2

    rclpy.init()
    n = Goal()
    try:
        if len(a) == 1:
            pt = (wps.get("points") or {}).get(a[0])
            if pt is None:
                print(f"'{a[0]}' 이름을 찾을 수 없다. --list 로 확인할 것.")
                return 2
            x, y, yaw = pt["x"], pt["y"], math.radians(pt.get("yaw", 0.0))
        else:
            x, y = float(a[0]), float(a[1])
            yaw = math.radians(float(a[2])) if len(a) > 2 else n.current_yaw()
        return n.send(x, y, yaw, wait)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
