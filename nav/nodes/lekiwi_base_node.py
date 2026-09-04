#!/usr/bin/env python3
"""LeKiwi 옴니 베이스: /cmd_vel -> 바퀴, 엔코더 -> /odom + TF.

시뮬의 lekiwi_cmdvel_bridge.py 가 하던 일을 실기에서 한다. ros2_control 대신
직접 구현한 이유는 계층을 하나씩 검증하기 위해서다 -- 여기서 odom 이 맞는지
확인한 뒤에 ros2_control 을 얹으면, 문제가 생겼을 때 어느 층인지 바로 안다.

기구학은 lerobot 의 LeKiwi 구현과 같은 규약을 쓴다:
  바퀴 각도 [240, 0, 120] - 90 = [150, -90, 30] 도   (left, back, right = ID 7, 8, 9)
  wheel_radius 0.05 m,  base_radius 0.125 m
  m = [[cos(a), sin(a), base_radius] ...]
  바퀴선속도 = m . [vx, vy, wz]        (역: 바디속도 = m^-1 . 바퀴선속도)

odom 은 Present_Velocity 가 아니라 **위치 차분**으로 적분한다. 속도 레지스터는
양자화와 샘플링 지터가 커서 적분하면 흘러간다.

    --no-drive  로 실행하면 모터에 아무것도 쓰지 않는다. 손으로 밀어서
                odom 을 검증할 때 쓴다. 부호 규약도 여기서 드러난다.
"""
import argparse, math, time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from std_srvs.srv import SetBool
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from scservo_sdk import COMM_SUCCESS, PacketHandler, PortHandler

PORT, BAUD = "/dev/ttyACM0", 1000000
LEFT, BACK, RIGHT = 7, 8, 9
WHEELS = [LEFT, BACK, RIGHT]
ADDR_MODE, ADDR_TORQUE, ADDR_GOAL_SPEED = 33, 40, 46
ADDR_POS = 56
TICKS = 4096
# 실측 교정값. 환경변수로 덮어쓸 수 있다.
#   직진 1 m 시험 -> wheel_radius 는 0.05 가 측정 정밀도 안에서 맞음
#   1080도 회전 시험 -> 실제는 990도밖에 안 돌아, wheel_R/base_R = 0.366377
#   따라서 base_radius = 0.05 / 0.366377 = 0.13647 (lerobot 기본값 0.125 가 아님)
import os as _os
WHEEL_R = float(_os.environ.get("LEKIWI_WHEEL_R", "0.05"))
BASE_R  = float(_os.environ.get("LEKIWI_BASE_R",  "0.13647"))
# lerobot 원래 값. 이 각도들은 '바퀴의 위치 방위'가 아니라 '바퀴가 구르는
# 방향'이다 -- 옴니휠에서 둘은 90도 차이가 난다. 실측 바퀴 위치(라이다 기준
# 61 / 179.5 / -59.3)에 90 을 더하면 [151, 269.5, 30.7] = [150, -90, 30] 으로
# 여기 값과 일치한다. 즉 base_link 는 라이다와 이미 정렬되어 있으며, 틀렸던
# 것은 URDF 의 라이다 장착각(-30도)뿐이었다.
ANGLES = np.radians(np.array([240.0, 0.0, 120.0]) - 90.0)
M = np.array([[math.cos(a), math.sin(a), BASE_R] for a in ANGLES])
M_INV = np.linalg.inv(M)


class Bus:
    def __init__(self):
        self.p = PortHandler(PORT); self.h = PacketHandler(0)
        if not self.p.openPort(): raise SystemExit(f"{PORT} 열기 실패")
        self.p.setBaudRate(BAUD)
    def r2(self, i, a):
        v, c, _ = self.h.read2ByteTxRx(self.p, i, a)
        return v if c == COMM_SUCCESS else None
    def w1(self, i, a, v): self.h.write1ByteTxRx(self.p, i, a, v)
    def w2(self, i, a, v): self.h.write2ByteTxRx(self.p, i, a, v)
    def close(self):
        try: self.p.closePort()
        except Exception: pass
    def reopen(self):
        self.p = PortHandler(PORT); self.h = PacketHandler(0)
        if not self.p.openPort(): return False
        self.p.setBaudRate(BAUD)
        return True


def unwrap(prev, cur):
    d = cur - prev
    if d >  TICKS//2: d -= TICKS
    if d < -TICKS//2: d += TICKS
    return d


def sign_mag(v):
    """STS 는 속도를 부호-크기로 받는다. bit15 가 방향."""
    v = int(round(v))
    return (abs(v) & 0x7FFF) | (0x8000 if v < 0 else 0)


class BaseNode(Node):
    def __init__(self, args):
        super().__init__("lekiwi_base")
        self.args = args
        self.bus = Bus()
        self.sign = np.array(args.sign, dtype=float)     # 바퀴별 부호 보정
        for i in WHEELS:
            if self.bus.r2(i, ADDR_POS) is None:
                raise SystemExit(f"ID {i} 읽기 실패")
        if not args.no_drive:
            for i in WHEELS:
                self.bus.w1(i, ADDR_MODE, 1)             # 속도(휠) 모드
                self.bus.w1(i, ADDR_TORQUE, 1)
            self.get_logger().warn("구동 활성화 -- 로봇이 움직입니다")
        else:
            self.get_logger().info("--no-drive: 모터에 쓰지 않습니다 (읽기 전용)")

        self.prev = {i: self.bus.r2(i, ADDR_POS) for i in WHEELS}
        self.x = self.y = self.th = 0.0
        self.cmd = np.zeros(3)
        self.cmd_t = 0.0
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd, 10)
        self.last = time.time()
        # pick 로직이 ZMQ 로 같은 서보 버스를 써야 할 때 포트를 양보한다.
        # 프로세스를 죽이는 대신 이 방식을 쓰는 이유: 런치의 respawn 이 즉시
        # 되살려서 포트를 다시 뺏어가기 때문이다.
        #   ros2 service call /lekiwi_base/set_bus std_srvs/srv/SetBool "{data: false}"  # 양보
        #   ros2 service call /lekiwi_base/set_bus std_srvs/srv/SetBool "{data: true}"   # 회수
        self.bus_on = True
        self.create_service(SetBool, "~/set_bus", self.on_set_bus)
        self.create_timer(1.0/args.rate, self.tick)
        self.report_t = time.time()

    def on_set_bus(self, req, resp):
        if req.data and not self.bus_on:
            if not self.bus.reopen():
                resp.success = False; resp.message = f"{PORT} 재개방 실패"
                return resp
            for i in WHEELS:
                self.bus.w1(i, ADDR_MODE, 1); self.bus.w1(i, ADDR_TORQUE, 1)
            # 그동안 바퀴가 얼마나 돌았는지는 알 수 없다. 현재 위치를 새 기준으로
            # 삼아 odom 이 튀지 않게 한다. 잃어버린 이동은 라이다 재정합으로
            # 회복해야 한다 (refine_pose.py).
            for i in WHEELS:
                self.prev[i] = self.bus.r2(i, ADDR_POS)
            self.bus_on = True
            self.get_logger().warn("서보 버스 회수. odom 기준을 현재 위치로 재설정했다 "
                                   "-- 그동안의 이동은 반영되지 않으니 재정합할 것.")
            resp.success = True; resp.message = "버스 회수"
        elif not req.data and self.bus_on:
            for i in WHEELS:
                try:
                    self.bus.w2(i, ADDR_GOAL_SPEED, 0); self.bus.w1(i, ADDR_TORQUE, 0)
                except Exception: pass
            self.bus.close()
            self.bus_on = False
            self.get_logger().warn(f"서보 버스 양보. {PORT} 를 놓았다.")
            resp.success = True; resp.message = "버스 양보"
        else:
            resp.success = True; resp.message = "이미 그 상태"
        return resp

    def on_cmd(self, m):
        self.cmd = np.array([m.linear.x, m.linear.y, m.angular.z])
        self.cmd_t = time.time()

    def drive(self):
        if not self.bus_on: return
        # 워치독: 명령이 끊기면 즉시 정지. 이게 없으면 마지막 명령으로 계속 달린다.
        v = self.cmd if (time.time() - self.cmd_t) < self.args.timeout else np.zeros(3)
        lin = M.dot(v)                       # 바퀴 선속도 m/s
        degps = lin / WHEEL_R * 180.0 / math.pi
        raw = degps * TICKS / 360.0
        mx = np.max(np.abs(raw))
        if mx > self.args.max_raw:
            raw = raw * (self.args.max_raw / mx)
        for k, i in enumerate(WHEELS):
            self.bus.w2(i, ADDR_GOAL_SPEED, sign_mag(raw[k] * self.sign[k]))

    def tick(self):
        if not self.bus_on: return
        now = time.time(); dt = now - self.last; self.last = now
        d_ticks = np.zeros(3)
        for k, i in enumerate(WHEELS):
            p = self.bus.r2(i, ADDR_POS)
            if p is None: continue
            d_ticks[k] = unwrap(self.prev[i], p) * self.sign[k]
            self.prev[i] = p
        # 틱 -> 바퀴 회전각(rad) -> 바퀴 선이동(m) -> 바디 이동
        d_lin = d_ticks / TICKS * 2*math.pi * WHEEL_R
        dx, dy, dth = M_INV.dot(d_lin)
        # 바디 프레임 이동을 odom 프레임으로 (중점 각도 사용)
        c, s = math.cos(self.th + dth/2), math.sin(self.th + dth/2)
        self.x += c*dx - s*dy
        self.y += s*dx + c*dy
        self.th = (self.th + dth + math.pi) % (2*math.pi) - math.pi

        if not self.args.no_drive:
            self.drive()

        st = self.get_clock().now().to_msg()
        q = (math.sin(self.th/2), math.cos(self.th/2))
        o = Odometry()
        o.header.stamp = st; o.header.frame_id = "odom"; o.child_frame_id = "base_link"
        o.pose.pose.position.x = self.x; o.pose.pose.position.y = self.y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = q
        if dt > 0:
            o.twist.twist.linear.x = dx/dt; o.twist.twist.linear.y = dy/dt
            o.twist.twist.angular.z = dth/dt
        self.odom_pub.publish(o)
        if self.args.tf:
            t = TransformStamped()
            t.header.stamp = st; t.header.frame_id = "odom"; t.child_frame_id = "base_link"
            t.transform.translation.x = self.x; t.transform.translation.y = self.y
            t.transform.rotation.z, t.transform.rotation.w = q
            self.tf.sendTransform(t)

        if self.args.verbose and now - self.report_t > 0.5:
            self.report_t = now
            print(f"\rodom  x{self.x:+7.3f} y{self.y:+7.3f} th{math.degrees(self.th):+8.2f}도   "
                  f"틱 L{d_ticks[0]:+6.0f} B{d_ticks[1]:+6.0f} R{d_ticks[2]:+6.0f}",
                  end="", flush=True)

    def stop(self):
        if not self.bus_on: return
        if not self.args.no_drive:
            for i in WHEELS:
                self.bus.w2(i, ADDR_GOAL_SPEED, 0)
                self.bus.w1(i, ADDR_TORQUE, 0)
        self.bus.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-drive", action="store_true", help="모터에 쓰지 않음 (odom 검증용)")
    ap.add_argument("--rate", type=float, default=25.0)   # 50Hz 는 /tf 를 구독하는 모든 Nav2 노드에 비용을 물린다
    ap.add_argument("--max-raw", type=float, default=1500, help="바퀴 원시속도 상한")
    ap.add_argument("--timeout", type=float, default=0.5, help="cmd_vel 워치독 (초)")
    ap.add_argument("--sign", type=float, nargs=3, default=[1, 1, 1], help="바퀴별 부호 (L B R)")
    ap.add_argument("--tf", action="store_true", help="odom->base_link TF 발행")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()
    rclpy.init()
    n = BaseNode(args)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.stop(); print()
        # SIGTERM 으로 죽을 때 rclpy 가 이미 context 를 내린 뒤라 두 번 부르면
        # RCLError 가 난다. 실제 종료에는 영향이 없지만 로그가 지저분해진다.
        if rclpy.ok():
            rclpy.shutdown()

main()
