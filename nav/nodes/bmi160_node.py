#!/usr/bin/env python3
"""BMI160 (I2C 0x68) -> sensor_msgs/Imu.

카토그래퍼가 tracking_frame="imu_link", use_imu_data=true 로 이 토픽에 자세를
맡기므로, 원시값을 그대로 흘리면 안 되고 두 가지를 보정한다:

1) 자이로 바이어스 -- 실측 Z축 -0.2679 도/s. 보정 없이 두면 가만히 서 있어도
   지도가 분당 16도씩 돌아간다. **상수로 박지 않고 시작 시 재추정한다**:
   MEMS 자이로 바이어스는 온도에 따라 변해서, 한 번 잰 값이 다음에는 틀리다.
   추정 중에는 로봇이 정지해 있어야 한다.

2) 중력 스케일 -- 실측 |a| = 9.9725 로 9.80665 대비 +1.69%. 가속도계 초기
   스케일 오차이며, 중력 방향 정렬에 쓰이므로 맞춰준다.

방향(orientation)은 발행하지 않는다. 지자기계가 없어 요를 절대적으로 알 수
없고, 카토그래퍼도 각속도와 가속도만 쓴다. orientation_covariance[0] = -1 이
"방향 없음"을 뜻하는 ROS 규약이다.

    python3 bmi160_node.py --bias-seconds 8
"""
import argparse, os, fcntl, math, struct, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

BUS = "/dev/i2c-1"
ADDR = 0x68
I2C_SLAVE = 0x0703
ACC_LSB_PER_G = 16384.0     # +/-2g  (ACC_RANGE=0x03 확인)
GYR_LSB_PER_DPS = 16.4      # +/-2000dps (GYR_RANGE=0x00 확인)
G = 9.80665
DEG = math.pi / 180.0


class BMI160(Node):
    def __init__(self, a):
        super().__init__("bmi160")
        self.a = a
        self.fd = os.open(BUS, os.O_RDWR)
        fcntl.ioctl(self.fd, I2C_SLAVE, ADDR)
        cid = self.rd(0x00)[0]
        if cid != 0xD1:
            raise SystemExit(f"CHIP_ID 0x{cid:02X} -- BMI160(0xD1) 이 아닙니다")
        self.wr(0x7E, 0x11); time.sleep(0.10)      # accel normal
        self.wr(0x7E, 0x15); time.sleep(0.15)      # gyro normal
        pmu = self.rd(0x03)[0]
        if (pmu & 0x14) != 0x14:
            raise SystemExit(f"PMU_STATUS 0x{pmu:02X} -- normal 모드 진입 실패")

        self.bias = np.zeros(3)
        self.ascale = 1.0
        self.calibrate(a.bias_seconds)

        self.pub = self.create_publisher(Imu, a.topic, qos_profile_sensor_data)
        self.n = 0; self.report = time.time()
        self.create_timer(1.0/a.rate, self.tick)
        self.get_logger().info(f"발행 시작: {a.topic} @{a.rate:.0f} Hz, frame={a.frame}")

    # I2C 는 가끔 Errno 5 (I/O error) 를 낸다. 예전에는 그 한 번에 노드가 죽었고,
    # 그러면 카토그래퍼가 "Queue waiting for data: (0, imu)" 로 조용히 멈춰서
    # 지도가 아예 안 나왔다. 증상만 보면 원인을 찾기 어려우므로 재시도한다.
    def rd(self, reg, n=1, tries=3):
        for k in range(tries):
            try:
                os.write(self.fd, bytes([reg])); return os.read(self.fd, n)
            except OSError:
                if k == tries - 1: raise
                time.sleep(0.002)
    def wr(self, reg, val, tries=3):
        for k in range(tries):
            try:
                os.write(self.fd, bytes([reg, val])); return
            except OSError:
                if k == tries - 1: raise
                time.sleep(0.002)

    def raw(self):
        gx, gy, gz, ax, ay, az = struct.unpack("<hhhhhh", self.rd(0x0C, 12))
        g = np.array([gx, gy, gz], dtype=float) / GYR_LSB_PER_DPS      # 도/s
        acc = np.array([ax, ay, az], dtype=float) / ACC_LSB_PER_G * G  # m/s^2
        return g, acc

    def calibrate(self, secs):
        self.get_logger().warn(f"자이로 바이어스 추정 {secs:.0f}초 -- 로봇을 정지시켜 두세요")
        G_, A_ = [], []
        t0 = time.time()
        fails = 0
        while time.time() - t0 < secs:
            try:
                g, acc = self.raw(); G_.append(g); A_.append(acc)
            except OSError:
                fails += 1
            time.sleep(0.002)
        if fails:
            self.get_logger().warn(f"바이어스 추정 중 I2C 실패 {fails}회")
        if len(G_) < 50:
            raise RuntimeError(f"바이어스 추정 표본 부족 ({len(G_)}개). I2C 확인 필요.")
        G_ = np.array(G_); A_ = np.array(A_)
        self.bias = G_.mean(axis=0)
        norm = np.linalg.norm(A_, axis=1).mean()
        self.ascale = G / norm
        moved = G_.std(axis=0).max()
        self.get_logger().info(
            f"바이어스 X{self.bias[0]:+.4f} Y{self.bias[1]:+.4f} Z{self.bias[2]:+.4f} 도/s "
            f"(표준편차 최대 {moved:.4f})")
        self.get_logger().info(f"중력 실측 {norm:.4f} -> 스케일 보정 {self.ascale:.5f}")
        if moved > 0.5:
            self.get_logger().warn("추정 중 움직임이 감지됐습니다. 바이어스가 부정확할 수 있습니다.")
        self.gstd = float(np.deg2rad(G_.std(axis=0).mean()))
        self.astd = float((A_.std(axis=0)).mean())

    def tick(self):
        # 재시도로도 안 되면 이번 주기만 건너뛴다. 노드를 죽이지 않는다 --
        # IMU 가 사라지면 카토그래퍼 전체가 멈추기 때문이다.
        try:
            g, acc = self.raw()
        except OSError as e:
            self.i2c_err = getattr(self, "i2c_err", 0) + 1
            if self.i2c_err in (1, 10, 100) or self.i2c_err % 500 == 0:
                self.get_logger().error(
                    f"I2C 읽기 실패 {self.i2c_err}회 ({e}). 센서 커넥터를 확인하세요. "
                    f"계속 재시도 중 -- 카토그래퍼는 /imu 가 끊기면 멈춥니다.")
            return
        if getattr(self, "i2c_err", 0):
            self.get_logger().warn(f"I2C 복구됨 (누적 실패 {self.i2c_err}회)")
            self.i2c_err = 0
        g = (g - self.bias) * DEG          # rad/s
        acc = acc * self.ascale
        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.a.frame
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = g
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = acc
        gv, av = self.gstd**2, self.astd**2
        m.angular_velocity_covariance = [gv,0.,0., 0.,gv,0., 0.,0.,gv]
        m.linear_acceleration_covariance = [av,0.,0., 0.,av,0., 0.,0.,av]
        m.orientation_covariance = [-1.]+[0.]*8      # 방향 없음
        self.pub.publish(m)
        self.n += 1
        if self.a.verbose and time.time() - self.report > 5.0:
            self.report = time.time()
            self.get_logger().info(f"{self.n/5:.0f} Hz  gz={g[2]/DEG:+.3f} 도/s  "
                                   f"|a|={np.linalg.norm(acc):.3f}")
            self.n = 0


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--topic", default="imu")
    p.add_argument("--frame", default="imu_link")
    p.add_argument("--rate", type=float, default=100.0, help="센서 ODR 이 100Hz")
    p.add_argument("--bias-seconds", type=float, default=8.0)
    p.add_argument("--verbose", action="store_true")
    a = p.parse_args()
    rclpy.init()
    n = BMI160(a)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        os.close(n.fd)
        if rclpy.ok(): rclpy.shutdown()

main()
