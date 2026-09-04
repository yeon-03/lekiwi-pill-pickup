#!/usr/bin/env python3
"""오도메트리 정확도 시험.  로봇이 스스로 정해진 만큼 움직이고 숫자를 낸다.

  odom_test.py --test spin --turns 3
  odom_test.py --test line --dist 0.30
  odom_test.py --test spin --turns 3 --measured-deg 1050    # 실측을 넣어 보정값 계산
  odom_test.py --test line --dist 0.30 --measured 0.28

두 가지를 서로 다른 방법으로 잰다.

  왕복 닫힘 (자동)  +N바퀴 돌고 -N바퀴 되돌아온다. 끝나고 yaw 가 0 이면
                    오도메트리가 스스로 일관된다는 뜻이다. 다만 BASE_R 이
                    틀려도 왕복하면 0 이 되므로 '축척'은 검증되지 않는다.

  축척 (사람 필요)  로봇이 'odom 기준으로' 정확히 목표만큼 움직인 뒤 멈춘다.
                    자를 대고 실제 값을 재서 --measured 로 다시 실행하면
                    보정값을 계산해 준다. 외부 기준 없이는 축척을 알 수 없다.

주의: /cmd_vel 로 직접 발행한다. Nav2 의 collision_monitor 를 거치지 않으므로
      장애물 보호가 없다. 반드시 지켜보면서 돌릴 것.
"""
import math, sys, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

VX = 0.05      # m/s   상한 0.133 의 40%
# 회전 속도 기본값. run_teleop.sh 의 0.15 를 물려받았는데, 그 값은 "회전이
# 빠르면 스캔 왜곡으로 AMCL 이 흔들린다" 는 이유로 정해진 것이다. 이 시험은
# 라이다를 띄우지 않으므로(use_lidar:=false) 그 제약이 적용되지 않는다.
# 여기서 느리게 갈 유일한 이유는 바퀴 미끄러짐 억제다.
#
# 주의: BASE_R 오차와 미끄러짐은 신호가 같다(둘 다 "odom 은 다 돌았다는데
# 실제로는 덜 돔"). 바퀴 수를 늘려도 구분되지 않는다 -- 둘 다 비례해 커진다.
# 구분하려면 **같은 바퀴 수를 다른 속도로** 돌려 본다. 미끄러짐은 속도에
# 따라 커지고 BASE_R 오차는 속도와 무관하다.
WZ_DEFAULT = 0.15
SETTLE = 1.5   # 정지 후 안정화 대기


def opt(k, d):
    """런치가 빈 문자열을 넘길 수 있다(인자 미지정). 그때는 기본값으로 본다."""
    if k not in sys.argv:
        return d
    v = sys.argv[sys.argv.index(k) + 1].strip()
    return float(v) if v else d


class OdomTest(Node):
    def __init__(self):
        super().__init__("odom_test")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.p = None          # (x, y, yaw_unwrapped)
        self._yaw_prev = None
        self._yaw_acc = 0.0
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)

    def _on_odom(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if self._yaw_prev is None:
            self._yaw_prev = yaw
        d = yaw - self._yaw_prev
        d = (d + math.pi) % (2 * math.pi) - math.pi      # 언랩
        self._yaw_acc += d
        self._yaw_prev = yaw
        self.p = (m.pose.pose.position.x, m.pose.pose.position.y, self._yaw_acc)

    # ---------- 저수준 ----------
    def spin_for(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def send(self, vx=0.0, wz=0.0):
        t = Twist(); t.linear.x = vx; t.angular.z = wz
        self.pub.publish(t)

    def stop(self):
        for _ in range(10):
            self.send(); self.spin_for(0.05)

    def wait_odom(self, sec=15.0):
        t0 = time.time()
        while time.time() - t0 < sec and self.p is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.p is not None

    def run(self, kind, target, timeout):
        """목표만큼 움직이고 실제 odom 변화량을 돌려준다. kind: 'line'|'spin'"""
        s = self.p
        sign = 1.0 if target >= 0 else -1.0
        t0 = time.time()
        stuck_t, last = time.time(), 0.0
        while True:
            rclpy.spin_once(self, timeout_sec=0.02)
            if kind == "line":
                cur = math.hypot(self.p[0] - s[0], self.p[1] - s[1])
            else:
                cur = abs(self.p[2] - s[2])
            if cur >= abs(target):
                break
            if time.time() - t0 > timeout:
                self.stop()
                raise RuntimeError("시간 초과 (%.1fs). %.3f / %.3f 만 진행됨"
                                   % (timeout, cur, abs(target)))
            # 오도메트리가 멈춰 있으면 즉시 중단 -- 바퀴가 헛돌거나 노드가 죽은 것
            if cur - last > 1e-4:
                stuck_t, last = time.time(), cur
            elif time.time() - stuck_t > 4.0:
                self.stop()
                raise RuntimeError("오도메트리가 4초간 변하지 않는다. 바퀴/전원 확인")
            if kind == "line":
                self.send(vx=sign * VX)
            else:
                self.send(wz=sign * WZ)
        self.stop()
        self.spin_for(SETTLE)
        return (math.hypot(self.p[0] - s[0], self.p[1] - s[1]) if kind == "line"
                else self.p[2] - s[2])


def main():
    global WZ
    WZ = opt("--wz", WZ_DEFAULT)
    if not (0.02 <= WZ <= 0.5):
        print("--wz 는 0.02~0.5 rad/s 범위여야 한다 (기체 상한 0.843)"); sys.exit(1)
    kind = sys.argv[sys.argv.index("--test") + 1] if "--test" in sys.argv else "spin"
    turns = opt("--turns", 3.0)
    dist = opt("--dist", 0.30)
    meas_deg = opt("--measured-deg", None)
    meas_m = opt("--measured", None)

    rclpy.init()
    n = OdomTest()
    try:
        if not n.wait_odom():
            print("\n/odom 이 오지 않는다. 베이스 노드가 떠 있는지 확인할 것.")
            sys.exit(1)

        if kind == "spin":
            tgt = math.radians(360.0 * turns)
            print("\n+%.0f바퀴 회전 @ %.2f rad/s (약 %.0f초). 시작 방향을 표시해 두세요."
                  % (turns, WZ, math.radians(360.0 * turns) / WZ))
            a = n.run("spin", tgt, timeout=60 + 200 * turns)
            print("  odom %+.2f도 만큼 돌고 정지" % math.degrees(a))

            print("\n--- 축척 ---")
            print("  odom 기준 %.0f도 = 정확히 %.0f바퀴이므로, 오도메트리가 맞다면"
                  % (360.0 * turns, turns))
            print("  로봇은 **시작 방향 그대로** 멈춰 있어야 한다.")
            if meas_deg is None:
                print("\n  시작 표시에서 몇 도 어긋났는지(Δ) 재고,")
                print("  실제 회전량 %.0f-Δ 를 넣어 다시 실행:" % (360.0 * turns))
                print("    odom_test.py --test spin --turns %g --measured-deg <실제도>"
                      % turns)
            else:
                import os
                cur = float(os.environ.get("LEKIWI_BASE_R", "0.13647"))
                newr = cur * (360.0 * turns) / meas_deg
                print("\n  실제 %.1f도 / odom %.1f도" % (meas_deg, 360.0 * turns))
                print("  LEKIWI_BASE_R  %.5f -> %.5f  (%+.1f%%)"
                      % (cur, newr, 100 * (newr / cur - 1)))

            if "--round-trip" in sys.argv:
                print("\n-%.0f바퀴 되돌아오기" % turns)
                b = n.run("spin", -tgt, timeout=60 + 200 * turns)
                close = math.degrees(a + b)
                print("  odom %+.2f도" % math.degrees(b))
                print("\n--- 왕복 닫힘 ---")
                print("  잔차 %+.2f도  %s" % (close, "좋음 (5도 미만)" if abs(close) < 5
                                              else "큼 -- 미끄러짐/부호/전원 확인"))

        else:
            print("\n앞으로 %.2f m (odom 기준). 앞이 비어 있는지 확인할 것." % dist)
            d = n.run("line", dist, timeout=30 + dist / VX * 3)
            print("  odom %.4f m 이동 후 정지" % d)
            print("\n--- 축척 ---")
            if meas_m is None:
                print("  자를 대고 실제 이동 거리를 재서 다시 실행:")
                print("    odom_test.py --test line --dist %g --measured <실제m>" % dist)
            else:
                import os
                cur = float(os.environ.get("LEKIWI_WHEEL_R", "0.05"))
                new = cur * meas_m / d
                print("  실제 %.4f m / odom %.4f m" % (meas_m, d))
                print("  LEKIWI_WHEEL_R  %.5f -> %.5f  (%+.1f%%)"
                      % (cur, new, 100 * (new / cur - 1)))
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
