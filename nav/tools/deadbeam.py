#!/usr/bin/env python3
"""옴니휠이 가리는 라이다 사각지대를 실측한다  ->  LEKIWI_WHEEL_BLIND

  deadbeam.py                 12초 측정
  deadbeam.py --sec 20 --min-frac 0.8

전제: 라이다를 **마스킹 없이, 강도 필터도 끄고** 띄워야 한다.

      ros2 launch ~/launch/lekiwi_sensors.launch.py \
          layer:=2 use_imu:=false mask_wheels:=false min_intensity:=0

      mask_wheels:=false  마스킹된 스캔을 재면 이미 지워진 구간을 다시 재는
                          셈이 된다.
      min_intensity:=0    **이걸 빠뜨리면 측정이 실패한다.** 강도 필터가 켜져
                          있으면 무효 빔이 70% 까지 올라가 바퀴 사각지대와
                          환경 미탐지가 뒤섞인다. 실측에서 후보 구간이
                          120도 간격을 이루지 못하고 엉뚱한 값이 나왔다.
                          필터를 끄면 무효율이 49% 로 내려가고 세 구간이
                          정확히 120도 간격으로 잡힌다.

원리
----
바퀴는 라이다 바로 옆에 있어 그 방향의 빔을 막는다. 반환이 아예 없거나
자기 반경 안쪽에서 돌아온다. 여러 스캔에 걸쳐 '항상' 무효인 각도 구간을
찾으면 그것이 사각지대다.

주의 -- 먼 벽도 무효로 보인다
------------------------------
max_range 를 넘는 방향도 반환이 없다. 정지 상태 측정만으로는 바퀴와
'먼 벽'을 구분할 수 없다. 구분하려면 로봇을 손으로 60도쯤 돌려 다시 재라.
바퀴 사각지대는 로봇과 함께 돌므로 **로봇 기준 각도가 그대로**이고,
먼 벽 때문에 생긴 구간은 각도가 바뀐다.

각도는 base_link 기준이다(라이다 노드가 LEKIWI_LIDAR_YAW 로 이미 변환한다).
그래서 LIDAR_YAW 를 바꾸면 이 값도 같이 바뀐다 -- 다시 재야 한다.
"""
import math, os, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def opt(k, d):
    return float(sys.argv[sys.argv.index(k) + 1]) if k in sys.argv else d


def main():
    sec = opt("--sec", 12.0)
    min_frac = opt("--min-frac", 0.85)     # 이 비율 이상 무효면 사각지대 후보
    min_w = opt("--min-width", 8.0)        # 이보다 좁은 구간은 잡음으로 본다
    max_w = opt("--max-width", 70.0)       # 이보다 넓으면 바퀴가 아니다

    rclpy.init()
    n = Node("deadbeam")
    acc = {"n": 0, "bad": None, "ang": None}

    def cb(m):
        R = np.asarray(m.ranges, dtype=np.float64)
        bad = ~(np.isfinite(R) & (R > m.range_min) & (R < m.range_max))
        if acc["bad"] is None:
            acc["bad"] = np.zeros(len(R))
            acc["ang"] = np.degrees(m.angle_min + np.arange(len(R)) * m.angle_increment)
        if len(R) == len(acc["bad"]):
            acc["bad"] += bad
            acc["n"] += 1

    n.create_subscription(LaserScan, "/scan", cb, qos_profile_sensor_data)
    print("%.0f초 측정. 로봇을 움직이지 마세요." % sec)
    t0 = time.time()
    while time.time() - t0 < sec:
        rclpy.spin_once(n, timeout_sec=0.1)
    rclpy.shutdown()

    if not acc["n"]:
        print("\n/scan 을 받지 못했다. 라이다 노드가 떠 있는지 확인할 것.")
        sys.exit(1)

    frac = acc["bad"] / acc["n"]
    ang = acc["ang"]
    nb = len(frac)
    step = 360.0 / nb
    print("스캔 %d개, 빈 %d개 (%.2f도/빈), 무효 평균 %.1f%%"
          % (acc["n"], nb, step, 100 * frac.mean()))

    # 원형이므로 두 바퀴 이어붙여 연속 구간을 찾는다
    m2 = np.concatenate([frac >= min_frac] * 2)
    runs, i = [], 0
    while i < len(m2):
        if m2[i]:
            j = i
            while j < len(m2) and m2[j]:
                j += 1
            if i < nb:                       # 시작이 첫 바퀴 안인 것만
                runs.append((i, j - i))
            i = j
        else:
            i += 1
    # 같은 구간이 두 번 잡히지 않게 정리
    seen, sectors = set(), []
    for s, ln in runs:
        c = (s + ln / 2.0) % nb
        key = round(c / 3)
        if key in seen:
            continue
        seen.add(key)
        w = ln * step
        if min_w <= w <= max_w:
            center = (ang[0] + c * step + 180) % 360 - 180
            sectors.append((center, w, ln))

    sectors.sort(key=lambda t: -t[2])
    print("\n--- 사각지대 후보 (무효율 %.0f%% 이상, 폭 %.0f~%.0f도) ---"
          % (100 * min_frac, min_w, max_w))
    if not sectors:
        print("  없음. --min-frac 을 낮추거나 측정을 길게 해볼 것.")
    for c, w, ln in sectors:
        print("  중심 %+7.1f도  폭 %5.1f도  (%d빈)" % (c, w, ln))

    top = sectors[:3]
    if len(top) == 3:
        s = ",".join("%.1f:%.1f" % (c, w) for c, w, _ in top)
        print("\n--- ~/lekiwi_profile.sh 에 넣을 값 ---")
        print('  export LEKIWI_WHEEL_BLIND="%s"' % s)
        # 각도순으로 정렬한 뒤 이웃 간격을 본다. 정렬하지 않으면 임의 순서로
        # 빼게 되어 120도가 그 여각인 240도로 나온다.
        cs = sorted(c for c, _, _ in top)
        gaps = sorted(((cs[(i + 1) % 3] - cs[i]) % 360) for i in range(3))
        print("\n  구간 간격: %s" % ", ".join("%.0f도" % g for g in gaps))
        if all(abs(g - 120) < 25 for g in gaps):
            print("  세 구간이 120도 간격이다 -- 옴니휠 세 개와 맞는다.")
        else:
            print("  [주의] 120도 간격이 아니다. 먼 벽을 바퀴로 잘못 잡았을 수 있다.")
            print("         로봇을 60도쯤 돌려 다시 재고, 각도가 그대로면 바퀴다.")
    else:
        print("\n  구간이 3개가 아니라 값을 자동 생성하지 않았다 (옴니휠은 3개).")
        print("  측정 시간을 늘리거나 --min-frac 을 조정할 것.")


if __name__ == "__main__":
    main()
