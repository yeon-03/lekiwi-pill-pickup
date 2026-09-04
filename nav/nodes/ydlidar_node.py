#!/usr/bin/env python3
"""YDLidar (model 0x97) -> sensor_msgs/LaserScan. 순수 파이썬 드라이버.

ydlidar_ros2_driver 를 소스 빌드하는 대신 직접 만든 이유: 이 모델은 샘플이
**3바이트(밝기1 + 거리2)** 인데, 2바이트로 읽는 구현을 붙이면 프레이밍이
어긋나 거리가 튀고 체크섬이 전부 깨진다. 실기에서 확인한 포맷을 그대로 쓴다.

  AA 55 | CT | LSN | FSA(2) | LSA(2) | CS(2) | [밝기1 거리2] * LSN
  거리 = Si/4 [mm],  각도 = (FSA>>1)/64 [도],  CT bit0 = 한 바퀴 시작

**회전 방향**: 이 기체에서는 원시 각도가 이미 ROS(REP-103, 반시계 양) 규약과
맞으므로 **뒤집지 않는다**. 로봇을 +62.75도 돌렸을 때 스캔이 -62.75도가 아니라
+62.00도 이동하는지로 실측 확인했다 (scan_sign_test.py). 뒤집으면 지도가 좌우
대칭이 되는데, 벽이 대칭인 방에서는 눈으로 절대 못 잡는 종류의 버그다.
--invert 로 다시 켤 수 있다.

미탐지(-1.0)는 0.0 으로 내보낸다. range_min 미만이라 소비자가 무효로 버린다.
inf 로 두면 카토그래퍼가 missing_data_ray_length 만큼 그 방향을 비워버린다.
"""
import argparse
import os as _os, math, struct, time
from collections import deque
import numpy as np
import rclpy, serial
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class YDLidar(Node):
    def __init__(self, a):
        super().__init__("ydlidar")
        self.a = a
        self.ser = serial.Serial(a.port, a.baud, timeout=0.5)
        self.ser.reset_input_buffer()
        self.ser.write(bytes([0xA5, 0x60])); self.ser.flush()
        self.pub = self.create_publisher(LaserScan, a.topic, qos_profile_sensor_data)
        self.buf = bytearray()
        self.bins = np.zeros(a.bins); self.hits = np.zeros(a.bins)
        self.inten = np.zeros(a.bins)
        self.rev_t = self.get_clock().now()
        self.scans = 0; self.report = time.time()
        self.fov_mask = self._build_fov_mask()
        self.wheel_mask = self._build_wheel_mask()

        # --- 스캔 디스큐 ---------------------------------------------------
        # 라이다는 한 바퀴(약 164 ms) 도는 동안의 점을 전부 같은 자리에서 잰
        # 것처럼 내보낸다. 그동안 로봇이 회전하면 각 점의 실제 기준 방향이
        # 달라진다. 실측: 0.60 rad/s 회전에서 연속 스캔의 29.3% 가 어긋난다
        # (정지 시 0.1%). 카토그래퍼는 이걸 보정하지만 AMCL 은 하지 않는다.
        #
        # 보정 방법: 회전 시작 시각을 기준으로 삼고, 각 점이 측정된 시점까지
        # 로봇이 돈 각도만큼 그 점의 방위를 되돌린다. 등속 회전을 가정하며,
        # 이 로봇은 최대 0.25 rad/s 로 느려서 한 스캔 안에서는 충분히 정확하다.
        self.wz = 0.0            # 현재 각속도 (rad/s)
        # odom 의 twist.angular.z 는 쓸 수 없다. 베이스가 50 Hz 로 엔코더를 읽는데
        # 서보의 Present_Position 갱신이 그보다 느려서 절반 이상의 주기에서 틱
        # 변화가 0 이 되고, 가끔 몰아서 튄다. 실측: 중앙값 0.0000, 최대 0.3445,
        # 참값 0.1032 rad/s. 그래서 자세(yaw) 이력의 기울기로 직접 구한다.
        self.yhist = deque(maxlen=40)   # (시각, yaw) -- 50 Hz 로 0.8 초분
        self.rev_a0 = None       # 이번 회전이 시작된 각도
        self.rev_T = 1.0 / 6.0   # 직전 회전 주기 (초) -- wz 평활 창 폭에만 쓴다
        # 한 회전 동안 로봇이 실제로 돈 각도(rad). wz x T 로 계산하지 않는다:
        # poll() 이 read(4096) 에서 블로킹되어 회전 주기가 0.15~0.30 초로 들쭉
        # 날쭉하기 때문이다. 오도메트리 yaw 차이를 직접 쓰면 주기를 몰라도 된다.
        self.rev_dyaw = 0.0
        self.rev_yaw0 = None
        self.desk_max = 0.0      # 이번 스캔에서 적용된 최대 보정각 (로그용)
        self.f_max = 0.0         # 진단용: 이번 회전에서 f 가 어디까지 갔나
        if self.a.deskew:
            from nav_msgs.msg import Odometry
            self.create_subscription(Odometry, "/odom", self._on_odom, 10)
            self.get_logger().info("스캔 디스큐 켜짐 (/odom 각속도 사용)")
        # -------------------------------------------------------------------

        self.create_timer(0.002, self.poll)
        self.get_logger().info(f"{a.port} @{a.baud}, {a.bins}빈, frame={a.frame}, "
                               f"각도부호 {'반전' if a.invert else '그대로'}, "
                               f"intensity 하한 {a.min_intensity:.0f}")

    def _on_odom(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.yhist:
            # 언랩: -pi/pi 경계를 넘을 때 튀지 않게 한다
            prev = self.yhist[-1][1]
            while yaw - prev > math.pi:  yaw -= 2*math.pi
            while yaw - prev < -math.pi: yaw += 2*math.pi
        self.yhist.append((t, yaw))
        # 스캔 주기(약 0.16초)와 비슷한 창으로 기울기를 낸다. 짧으면 양자화
        # 잡음이 그대로 들어오고, 길면 각속도 변화를 못 따라간다.
        win = max(self.rev_T, 0.12)
        t1, y1 = self.yhist[-1]
        base = None
        for tt, yy in self.yhist:
            if t1 - tt <= win * 1.5:
                base = (tt, yy); break
        if base is None or t1 - base[0] < 1e-3:
            return
        self.wz = (y1 - base[1]) / (t1 - base[0])

    def _build_wheel_mask(self):
        """옴니 바퀴에 가려진 세 구간. None 이면 마스킹 안 함."""
        if not self.a.mask_wheels:
            return None
        n = self.a.bins
        ang_base = (-180.0 + np.arange(n) * (360.0 / n)) + LIDAR_YAW_DEG
        mask = np.zeros(n, dtype=bool)
        for c, w in WHEEL_BLIND:
            d = (ang_base - c + 180.0) % 360.0 - 180.0
            mask |= np.abs(d) <= (w + 2 * self.a.wheel_margin) / 2.0
        self.get_logger().info(
            f"바퀴 마스킹: {int(mask.sum())}/{n} 빈 ({mask.sum()*360.0/n:.1f}°) 무효 처리 "
            f"(여유 {self.a.wheel_margin:.1f}°)")
        return mask

    def _build_fov_mask(self):
        """시야를 base_link 기준 부채꼴로 제한한다. None 이면 전방위(기본).

        사용자는 "로봇 정면"으로 생각하지만 스캔은 lidar_link 기준이고 그 사이에
        LIDAR_YAW_DEG 만큼 틀어져 있다. 여기서 변환해 주지 않으면 지정한 창이
        실제로는 30도 어긋난 곳에 생긴다.
        """
        if self.a.fov_width <= 0 or self.a.fov_width >= 360:
            return None
        n = self.a.bins
        b = np.arange(n)
        ang_lidar = -180.0 + b * (360.0 / n)          # 빈 -> lidar_link 각도
        ang_base = ang_lidar + LIDAR_YAW_DEG           # -> base_link 각도
        d = (ang_base - self.a.fov_center + 180.0) % 360.0 - 180.0
        mask = np.abs(d) <= self.a.fov_width / 2.0
        lo = self.a.fov_center - self.a.fov_width/2.0
        hi = self.a.fov_center + self.a.fov_width/2.0
        # 바퀴 사각지대와 얼마나 겹치는지 알려준다. 실측 중심/폭(base_link 기준).
        blocked = np.zeros(n, dtype=bool)
        for c, w in WHEEL_BLIND:
            dd = (ang_base - c + 180.0) % 360.0 - 180.0
            blocked |= np.abs(dd) <= w / 2.0
        usable = int((mask & ~blocked).sum())
        self.get_logger().warning(
            f"시야 제한: base_link {lo:+.1f}° ~ {hi:+.1f}° "
            f"(빈 {int(mask.sum())}/{n}), 이 중 바퀴에 막힌 {int((mask&blocked).sum())}개를 빼면 "
            f"실사용 {usable}개 = {usable*360.0/n:.1f}°. "
            f"SLAM/AMCL 은 이 시야로 제대로 동작하지 않는다.")
        return mask

    def emit(self):
        n = self.a.bins
        r = np.zeros(n, dtype=np.float32)
        it = np.zeros(n, dtype=np.float32)
        m = self.hits > 0
        r[m] = (self.bins[m] / self.hits[m] / 1000.0).astype(np.float32)   # mm -> m
        it[m] = (self.inten[m] / self.hits[m]).astype(np.float32)
        # 순서가 중요하다. inf -> 바퀴 -> 시야창 순으로 덮어써야 한다.
        if self.a.no_return_inf:
            # 반환이 없다는 것은 "그 방향은 비어 있다"는 정보다. 0.0 으로 두면
            # range_min 미만이라 카토그래퍼가 버리고, 점유 확률을 내릴 수단이
            # 사라져 한 번 찍힌 장애물이 영원히 지워지지 않는다.
            r[~m] = float("inf")
        if self.wheel_mask is not None:
            # 바퀴에 가려진 방향은 "비어 있다"가 아니라 "모른다"다. 여기를 inf 로
            # 두면 바퀴 그림자 방향으로 missing_data_ray_length 만큼 지워버려
            # 실제 벽까지 사라진다. 반드시 0.0(무효)이어야 한다.
            r[self.wheel_mask] = 0.0
            it[self.wheel_mask] = 0.0
        if self.fov_mask is not None:
            # 창 밖도 같은 이유로 0.0.
            r[~self.fov_mask] = 0.0
            it[~self.fov_mask] = 0.0
        s = LaserScan()
        now = self.get_clock().now()
        s.header.stamp = self.rev_t.to_msg()
        s.header.frame_id = self.a.frame
        s.angle_min = -math.pi
        s.angle_max = math.pi - 2*math.pi/n
        s.angle_increment = 2*math.pi/n
        s.scan_time = max((now - self.rev_t).nanoseconds * 1e-9, 1e-3)
        s.time_increment = s.scan_time / n
        s.range_min = max(self.a.range_min, self.a.self_radius)
        s.range_max = self.a.range_max
        s.ranges = r.tolist()
        s.intensities = it.tolist()
        # 시야를 좁혔을 때 실제로 발행되는 빔 수. 마스크 적용 전 self.hits 로
        # 세면 걸러낸 것까지 포함해 실제보다 크게 나온다.
        self.last_valid = int(((r >= s.range_min) & np.isfinite(r)).sum())
        self.pub.publish(s)
        # 다음 회전의 디스큐에 쓸 주기. 측정값이라 라이다 속도가 변해도 따라간다.
        self.rev_T = min(max(s.scan_time, 0.05), 0.5)
        # 이번 회전 동안 실제로 돈 각도를 재서 다음 회전의 보정에 쓴다.
        if self.a.deskew and self.yhist:
            y_now = self.yhist[-1][1]
            if self.rev_yaw0 is not None:
                self.rev_dyaw = y_now - self.rev_yaw0
            self.rev_yaw0 = y_now
        self.rev_a0 = None
        self.pub_desk = self.desk_max; self.desk_max = 0.0
        self.pub_f = self.f_max; self.f_max = 0.0
        self.pub_wz = self.wz
        self.rev_t = now
        self.bins[:] = 0; self.hits[:] = 0; self.inten[:] = 0
        self.scans += 1
        if self.a.verbose and time.time() - self.report > 5.0:
            self.report = time.time()
            dk = (f", 디스큐 {math.degrees(getattr(self,'pub_desk',0.0)):.2f}도 "
                  f"(f_max {getattr(self,'pub_f',0.0):.2f}, "
                  f"회전량 {math.degrees(self.rev_dyaw):+.2f}도/회전)"
                  if self.a.deskew else "")
            self.get_logger().info(
                f"{self.scans/5:.2f} Hz, 발행 유효빔 {self.last_valid}/{n}{dk}")
            self.scans = 0

    def poll(self):
        d = self.ser.read(4096)
        if d: self.buf += d
        n = self.a.bins
        while True:
            i = self.buf.find(b"\xAA\x55")
            if i < 0 or len(self.buf) - i < 10:
                if i > 0: del self.buf[:i]
                break
            ct, lsn = self.buf[i+2], self.buf[i+3]
            need = 10 + lsn*3
            if len(self.buf) - i < need: break
            pk = bytes(self.buf[i:i+need]); del self.buf[:i+need]
            if ct & 1 and self.hits.sum() > 0:
                self.emit()
            fsa, lsa, _ = struct.unpack_from("<HHH", pk, 4)
            a0, a1 = (fsa >> 1)/64.0, (lsa >> 1)/64.0
            span = (a1 - a0) % 360.0
            for k in range(lsn):
                ii = pk[10 + k*3]
                dist = struct.unpack_from("<H", pk, 11 + k*3)[0] / 4.0    # mm
                if dist <= 0: continue
                # 자기 몸(옴니휠) 반사 제거. 실측으로 세 바퀴가 0.151~0.158 m 에
                # 찍힌다. 안 걸러내면 카토그래퍼가 로봇에 고정된 이 점들을 실제
                # 장애물로 보고 정합하려 들어 지도가 뭉개지고, Nav2 는 로봇이
                # 영구히 장애물에 둘러싸인 것으로 본다.
                if dist < self.a.self_radius * 1000.0: continue
                # 약한 반사는 거리를 잘못 읽는다. 실측: 0.82~0.98 m 구간에
                # 반환의 39% 가 몰리는데 그 절반이 intensity ~19 로, 정상
                # 반환(~51)과 뚜렷이 갈린다. 방 한가운데에 없는 벽이 그려지는
                # 원인이었다. 임계 25 에서 인공물 절반을 걷어내고 정상 반환은
                # 0.3% 만 잃는다.
                if ii < self.a.min_intensity: continue
                ang = (a0 + span * k / max(lsn-1, 1)) % 360.0
                if self.a.invert:
                    ang = (360.0 - ang) % 360.0
                if self.a.deskew:
                    # 이 점이 회전의 어느 지점에서 측정됐는지 (0~1). 라이다는
                    # 등속 회전하므로 각도 진행률이 곧 시간 진행률이다.
                    if self.rev_a0 is None:
                        self.rev_a0 = ang
                    f = ((ang - self.rev_a0) % 360.0) / 360.0
                    # 회전 시작 이후 로봇이 돈 각도. 센서 프레임이 그만큼
                    # 돌았으므로, 시작 시점 기준으로 되돌리려면 더해준다.
                    # 직전 회전의 실측 회전량에 진행률을 곱한다.
                    dyaw = f * self.rev_dyaw
                    ang = (ang + math.degrees(dyaw)) % 360.0
                    if abs(dyaw) > abs(self.desk_max):
                        self.desk_max = dyaw
                    if f > self.f_max: self.f_max = f
                b = int((ang + 180.0) % 360.0 / 360.0 * n) % n
                self.bins[b] += dist; self.hits[b] += 1; self.inten[b] += ii

    def stop(self):
        try:
            self.ser.write(bytes([0xA5, 0x65])); self.ser.flush(); self.ser.close()
        except Exception:
            pass


# ---- 기체 고유값 -----------------------------------------------------------
# 이 두 개는 기체마다 다르다. 코드를 고치지 말고 ~/lekiwi_profile.sh 에서
# 환경변수로 준다. 그래야 이 파일이 모든 르키위에서 동일하게 유지된다.
#
# LEKIWI_LIDAR_YAW : base_link -> lidar_link 의 yaw(도). URDF 와 반드시 같아야
#   한다. 옴니휠 자기반사 세 개가 120도 간격으로 찍히는 것으로 각도는 잡히지만,
#   세 바퀴 중 어느 것인지는 그것만으로 정해지지 않는다(120도 모호성).
#   직진 주행 1회로 확정할 것.
# LEKIWI_WHEEL_BLIND : "중심:폭,중심:폭,중심:폭" (base_link 기준, 도).
#   deadbeam.py 로 측정한다.
LIDAR_YAW_DEG = float(_os.environ.get("LEKIWI_LIDAR_YAW", "-30.0"))

def _parse_blind(txt):
    out = []
    for part in txt.split(","):
        part = part.strip()
        if not part:
            continue
        c, w = part.split(":")
        out.append((float(c), float(w)))
    return out

WHEEL_BLIND = _parse_blind(
    _os.environ.get("LEKIWI_WHEEL_BLIND", "149.5:38.5,31.0:35.5,-89.3:34.0"))
# ---------------------------------------------------------------------------


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", default="/dev/ttyUSB0")
    p.add_argument("--baud", type=int, default=230400)
    p.add_argument("--topic", default="scan")
    p.add_argument("--frame", default="lidar_link")
    p.add_argument("--bins", type=int, default=720, help="한 바퀴 분할 수 (0.5도)")
    p.add_argument("--range-min", type=float, default=0.05)
    p.add_argument("--self-radius", type=float, default=0.21,
                   help="이 반경 안의 반환은 자기 몸으로 보고 버림 (실측 0.158 m + 여유)")
    p.add_argument("--range-max", type=float, default=12.0)
    p.add_argument("--invert", action="store_true", help="각도 부호 반전 (이 기체에서는 불필요)")
    p.add_argument("--min-intensity", type=float, default=0.0,
                   help="이 값 미만의 약한 반사는 버린다 (허위 거리 제거). 권장 25")
    p.add_argument("--deskew", action="store_true",
                   help="회전 중 스캔 왜곡을 /odom 각속도로 보정 (권장). "
                        "베이스 노드가 떠 있어야 한다")
    p.add_argument("--mask-wheels", action="store_true",
                   help="옴니 바퀴에 가려진 세 구간을 무효 처리")
    p.add_argument("--wheel-margin", type=float, default=2.0,
                   help="바퀴 구간 양옆 여유 (deg)")
    p.add_argument("--no-return-inf", action="store_true",
                   help="미탐지를 inf 로 발행 (카토그래퍼가 그 방향을 비운다). "
                        "--mask-wheels 와 반드시 함께 쓸 것")
    p.add_argument("--fov-center", type=float, default=0.0,
                   help="시야 중심 (base_link 기준 deg, 0=정면)")
    p.add_argument("--fov-width", type=float, default=0.0,
                   help="시야 폭 (deg). 0 이면 전방위(기본). 예: 60")
    p.add_argument("--verbose", action="store_true")
    a = p.parse_args()
    rclpy.init()
    n = YDLidar(a)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.stop()
        if rclpy.ok(): rclpy.shutdown()

main()
