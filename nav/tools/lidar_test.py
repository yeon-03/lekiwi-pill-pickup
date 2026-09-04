#!/usr/bin/env python3
"""라이다 장착각(base_link -> lidar_link 의 yaw)을 실측한다.

  lidar_test.py                 앞으로 0.30 m 가면서 측정
  lidar_test.py --dist 0.50
  lidar_test.py --no-drive      이미 움직인 뒤라면 사람이 직접 밀어도 된다

원리
----
로봇이 직진하면 주변 벽은 스캔 안에서 '진행 방향의 반대'로 흘러간다. 출발
스캔과 도착 스캔을 오도메트리 이동량만큼 겹쳐 보면, 장착각이 맞을 때만 두
스캔의 벽이 포개진다. 후보 각도를 훑어 잔차가 최소가 되는 값을 찾는다.

왜 '직진'이어야 하나
--------------------
라이다는 회전 중심(base_link 원점) 바로 위에 있다. 제자리 회전만 시키면
라이다도 같은 각도로 돌 뿐이라 장착각이 얼마든 스캔이 똑같이 겹친다 --
회전으로는 장착각을 볼 수 없다. 병진이 있어야 관측된다.

옴니휠 자기반사로 각도를 재려는 시도가 실패하는 이유도 같은 종류다. 바퀴
셋이 120도 간격이라 -30 / +90 / -150 세 후보가 데이터와 똑같이 맞는다.

주의: /cmd_vel 로 직접 발행한다. 장애물 보호가 없으니 앞을 비우고 지켜볼 것.
"""
import math, os, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

VX = 0.05
NPTS = 180          # 스캔당 표본 수. 잔차 행렬이 NPTS^2 이라 너무 키우지 말 것
CAP = 0.25          # 잔차 상한(m). 소수의 이상점이 결과를 지배하지 않게


def opt(k, d):
    return float(sys.argv[sys.argv.index(k) + 1]) if k in sys.argv else d


class LidarTest(Node):
    def __init__(self):
        super().__init__("lidar_test")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.scan = None
        self.pose = None
        self.create_subscription(LaserScan, "/scan",
                                 lambda m: setattr(self, "scan", m), qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)

    def _on_odom(self, m):
        q = m.pose.pose.orientation
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y,
                     math.atan2(2 * (q.w * q.z + q.x * q.y),
                                1 - 2 * (q.y * q.y + q.z * q.z)))

    def spin_for(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def stop(self):
        for _ in range(10):
            self.pub.publish(Twist()); self.spin_for(0.05)

    def wait_ready(self, sec=20.0):
        t0 = time.time()
        while time.time() - t0 < sec and (self.scan is None or self.pose is None):
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.scan is not None and self.pose is not None

    def grab(self):
        """방금 들어온 스캔과 그때의 자세를 함께 얻는다."""
        self.scan = None
        t0 = time.time()
        while self.scan is None and time.time() - t0 < 5.0:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.scan, self.pose

    def drive(self, dist):
        s = self.pose
        t0 = time.time()
        while math.hypot(self.pose[0] - s[0], self.pose[1] - s[1]) < dist:
            if time.time() - t0 > 30 + dist / VX * 3:
                self.stop(); raise RuntimeError("시간 초과")
            t = Twist(); t.linear.x = VX
            self.pub.publish(t)
            rclpy.spin_once(self, timeout_sec=0.02)
        self.stop(); self.spin_for(1.5)


def pts(scan, n=NPTS):
    """스캔을 라이다 프레임의 xy 점들로. 장착각은 아직 적용하지 않는다."""
    R = np.asarray(scan.ranges, dtype=np.float64)
    a = scan.angle_min + np.arange(len(R)) * scan.angle_increment
    ok = np.isfinite(R) & (R > scan.range_min) & (R < scan.range_max)
    R, a = R[ok], a[ok]
    if len(R) > n:
        k = np.linspace(0, len(R) - 1, n).astype(int)
        R, a = R[k], a[k]
    return np.stack([R * np.cos(a), R * np.sin(a)], axis=1)


def residual(PA, PB, dx, dy, dyaw, theta):
    """장착각 theta 를 가정했을 때 두 스캔이 얼마나 안 겹치는지(m)."""
    c, s = math.cos(theta), math.sin(theta)
    Rt = np.array([[c, -s], [s, c]])
    A = PA @ Rt.T                                  # A 시점 몸통 프레임
    c2, s2 = math.cos(dyaw), math.sin(dyaw)
    Rd = np.array([[c2, -s2], [s2, c2]])
    B = (PB @ Rt.T) @ Rd.T + np.array([dx, dy])    # B 를 A 시점 몸통 프레임으로
    d = np.linalg.norm(B[:, None, :] - A[None, :, :], axis=2).min(axis=1)
    return float(np.minimum(d, CAP).mean())


def main():
    dist = opt("--dist", 0.30)
    drive = "--no-drive" not in sys.argv
    cur = float(os.environ.get("LEKIWI_LIDAR_YAW", "0.0"))

    rclpy.init()
    n = LidarTest()
    try:
        if not n.wait_ready():
            print("\n/scan 또는 /odom 이 오지 않는다. 2~3층을 먼저 확인할 것.")
            sys.exit(1)

        sA, pA = n.grab()
        if drive:
            print("\n앞으로 %.2f m 이동한다. 앞을 비울 것." % dist)
            n.drive(dist)
        else:
            input("\n로봇을 앞으로 %.2f m 정도 밀고 Enter: " % dist)
            n.spin_for(1.0)
        sB, pB = n.grab()

        # A 시점 몸통 프레임에서 본 이동량
        ddx, ddy = pB[0] - pA[0], pB[1] - pA[1]
        ca, sa = math.cos(-pA[2]), math.sin(-pA[2])
        dx, dy = ca * ddx - sa * ddy, sa * ddx + ca * ddy
        dyaw = (pB[2] - pA[2] + math.pi) % (2 * math.pi) - math.pi
        moved = math.hypot(dx, dy)
        print("  이동 %.4f m (몸통기준 dx %+.4f, dy %+.4f), 회전 %+.2f도"
              % (moved, dx, dy, math.degrees(dyaw)))
        if moved < 0.05:
            print("\n이동량이 너무 작다(%.3f m). 장착각은 병진이 있어야 관측된다." % moved)
            sys.exit(1)

        PA, PB = pts(sA), pts(sB)
        print("  스캔 점 %d / %d" % (len(PA), len(PB)))

        # 1차: 1도 간격 전역 탐색
        cand = np.arange(-180.0, 180.0, 1.0)
        res = np.array([residual(PA, PB, dx, dy, dyaw, math.radians(t)) for t in cand])
        best = cand[int(res.argmin())]
        # 2차: 최적점 주변 0.1도
        fine = np.arange(best - 2.0, best + 2.0 + 1e-9, 0.1)
        rf = np.array([residual(PA, PB, dx, dy, dyaw, math.radians(t)) for t in fine])
        best = float(fine[int(rf.argmin())]); bres = float(rf.min())

        print("\n--- 결과 ---")
        print("  측정된 장착각   %+.2f도   (잔차 %.4f m)" % (best, bres))
        print("  현재 설정값     %+.2f도   (잔차 %.4f m)"
              % (cur, residual(PA, PB, dx, dy, dyaw, math.radians(cur))))

        # 다른 국소 최소가 비슷하게 좋으면 애매하다는 뜻
        far = np.abs((cand - best + 180) % 360 - 180) > 20
        second = float(res[far].min()) if far.any() else 9e9
        if second < bres * 1.3:
            print("  [주의] 20도 이상 떨어진 곳에도 비슷한 최소가 있다(잔차 %.4f)." % second)
            print("         벽이 대칭이거나 이동이 짧을 때 생긴다. 더 멀리/다른 곳에서 재볼 것.")

        d = (best - cur + 180) % 360 - 180
        if abs(d) < 1.5:
            print("\n  설정값과 %.2f도 차이 -- 맞다." % abs(d))
        else:
            print("\n  설정값과 %+.2f도 차이. 아래 세 곳을 모두 %+.2f 로 고칠 것:" % (d, best))
            print("    ~/lekiwi_profile.sh            LEKIWI_LIDAR_YAW")
            print("    ~/launch/lekiwi_sensors.launch.py   DEFAULTS[\"lidar_yaw\"]")
            print("    ~/lekiwi06.urdf                base_link_to_lidar_link 의 rpy (라디안 %.6f)"
                  % math.radians(best))
    except KeyboardInterrupt:
        print("\n중단됨")
    except Exception as e:
        print("\n시험 실패: %s" % e)
    finally:
        try:
            n.stop()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
