#!/usr/bin/env python3
"""브링업 0~1층 점검.  설정이 서로 어긋나지 않았는지, 장치가 붙어 있는지 본다.

  check_config.py                점검만
  check_config.py --wait 20      토픽이 뜰 때까지 기다렸다가 점검
  check_config.py --layer 1      1층까지만 올렸을 때 (그 위는 건너뛴다)

특히 lidar_yaw 는 두 곳에 적혀 있어서 하나만 고치면 조용히 어긋난다:
  lekiwi_profile.sh 의 LEKIWI_LIDAR_YAW        (센서 런치가 여기서 읽는다)
  <기체>.urdf 의 base_link_to_lidar_link rpy
둘이 다르면 지도가 통째로 틀어지는데 증상은 한참 위층에서야 보인다.
(예전에는 런치에도 따로 적혀 있어 세 곳이었다. 런치가 프로파일을 읽도록
 바꾸면서 한 곳이 줄었다.)
"""
import os, re, sys, math, time, glob

HOME = os.path.expanduser("~")
OK, BAD, WARN = "  [OK]  ", "  [실패]", "  [주의]"
fails = []


def say(tag, msg):
    print(tag + " " + msg)
    if tag is BAD:
        fails.append(msg)


# ---------- 설정 파일 세 곳의 lidar_yaw ----------
def _profile_name():
    p = f"{HOME}/lekiwi_profile.sh"
    if not os.path.exists(p):
        return "?"
    m = re.search(r'LEKIWI_NAME="?([\w-]+)', open(p).read())
    return m.group(1) if m else "?"


def _profile_yaw():
    p = f"{HOME}/lekiwi_profile.sh"
    if not os.path.exists(p):
        return None
    m = re.search(r'LEKIWI_LIDAR_YAW="?([-\d.]+)', open(p).read())
    return float(m.group(1)) if m else None


def _urdf_path():
    """파일명에 기체 번호를 넣지 않으므로 고정 이름. 프로파일이 다른 경로를
    지정했다면 그쪽을 우선한다."""
    m = re.search(r'LEKIWI_URDF="?([^"\n]+)', open(f"{HOME}/lekiwi_profile.sh").read()) \
        if os.path.exists(f"{HOME}/lekiwi_profile.sh") else None
    for c in ([m.group(1).replace("$HOME", HOME)] if m else []) + [f"{HOME}/lekiwi.urdf"]:
        if os.path.exists(c):
            return c
    return None


def _urdf_yaw():
    p = _urdf_path()
    if not p or not os.path.exists(p):
        return None
    s = open(p).read()
    m = re.search(r'base_link_to_lidar_link.*?rpy="([-\d.eE]+)\s+([-\d.eE]+)\s+([-\d.eE]+)"',
                  s, re.S)
    return math.degrees(float(m.group(3))) if m else None


def check_config_files():
    print("\n--- 설정 파일 ---")
    vals = {"profile": _profile_yaw(), "urdf": _urdf_yaw()}
    missing = [k for k, v in vals.items() if v is None]
    if missing:
        say(WARN, "lidar_yaw 를 못 읽은 곳: " + ", ".join(missing))
    have = {k: v for k, v in vals.items() if v is not None}
    if len(have) >= 2:
        lo, hi = min(have.values()), max(have.values())
        detail = ", ".join("%s=%.2f도" % (k, v) for k, v in have.items())
        if hi - lo < 0.01:
            say(OK, "lidar_yaw 일치 (%s)" % detail)
        else:
            say(BAD, "lidar_yaw 불일치! %s  <- 하나로 맞출 것" % detail)


# ---------- 장치 ----------
def check_devices(layer):
    print("\n--- 장치 ---")
    want = []
    if layer >= 2:
        want.append(("라이다", os.environ.get("LEKIWI_LIDAR_PORT", "/dev/ttyUSB0")))
    if layer >= 3:
        want.append(("서보", os.environ.get("LEKIWI_SERVO_PORT", "/dev/ttyACM0")))
    if not want:
        print("  (1층은 장치가 필요 없다)")
    for name, dev in want:
        if os.path.exists(dev):
            say(OK, "%s %s 존재" % (name, dev))
        else:
            found = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
            say(BAD, "%s %s 없음.  현재 있는 포트: %s"
                % (name, dev, ", ".join(found) or "없음"))


# ---------- ROS ----------
def check_ros(wait, layer):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from tf2_ros import Buffer, TransformListener

    rclpy.init()
    n = Node("check_config")
    buf = Buffer(); TransformListener(buf, n)
    seen = {}
    n.create_subscription(LaserScan, "/scan",
                          lambda m: seen.__setitem__("scan", seen.get("scan", 0) + 1),
                          qos_profile_sensor_data)
    n.create_subscription(Odometry, "/odom",
                          lambda m: seen.__setitem__("odom", seen.get("odom", 0) + 1), 10)
    from sensor_msgs.msg import Imu
    n.create_subscription(Imu, "/imu",
                          lambda m: seen.__setitem__("imu", seen.get("imu", 0) + 1),
                          qos_profile_sensor_data)
    t0 = time.time()
    while time.time() - t0 < wait:
        rclpy.spin_once(n, timeout_sec=0.1)
    dur = time.time() - t0

    print("\n--- 1층: 정적 TF ---")
    import rclpy.time
    for child in ("lidar_link", "imu_link", "base_footprint"):
        try:
            tf = buf.lookup_transform("base_link", child, rclpy.time.Time())
            t = tf.transform.translation
            q = tf.transform.rotation
            yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                          1 - 2 * (q.y * q.y + q.z * q.z)))
            say(OK, "base_link -> %-14s [%.3f, %.3f, %.3f] yaw %.2f도"
                % (child, t.x, t.y, t.z, yaw))
            if child == "lidar_link":
                pf = _profile_yaw()
                if pf is not None and abs(yaw - pf) > 0.5:
                    say(BAD, "  실제 TF yaw(%.2f)가 profile(%.2f)과 다르다 -- rsp 재시작 필요"
                        % (yaw, pf))
        except Exception:
            say(BAD, "base_link -> %s 없음  (rsp 가 떠 있나?)" % child)

    if layer < 2:
        print("\n--- 2~3층: layer=1 이라 건너뜀 ---")
        rclpy.shutdown()
        return
    print("\n--- 2~3층: 토픽 ---")
    want = [("scan", "/scan", 4.0, 9.0)]
    # IMU 는 없는 기체도 있다(lekiwi05). 없으면 실패가 아니라 주의로 알린다 --
    # 단 카토그래퍼는 IMU 를 기다리며 조용히 멈추므로 그 경고를 같이 낸다.
    want.append(("imu", "/imu", 0.0, 1e9))
    if layer >= 3:
        want.append(("odom", "/odom", 18.0, 32.0))
    for key, label, lo, hi in want:
        hz = seen.get(key, 0) / dur
        if seen.get(key):
            (say(OK, "%-6s %.1f Hz" % (label, hz)) if lo <= hz <= hi
             else say(WARN, "%-6s %.1f Hz  (기대 %.0f~%.0f)" % (label, hz, lo, hi)))
        elif key == "imu":
            say(WARN, "/imu   없음 -- IMU 가 없는 기체라면 정상. "
                      "그렇다면 <기체>_cartographer.lua 에서 "
                      "tracking_frame=\"base_link\", use_imu_data=false 로 고칠 것 "
                      "(안 고치면 SLAM 이 IMU 를 기다리며 조용히 멈춘다)")
        else:
            say(BAD, "%-6s 수신 없음" % label)

    if layer >= 3:
        try:
            buf.lookup_transform("odom", "base_link", rclpy.time.Time())
            say(OK, "odom -> base_link 있음")
        except Exception:
            say(BAD, "odom -> base_link 없음  (베이스 노드에 --tf 가 빠졌나?)")
    rclpy.shutdown()


def main():
    wait = 8.0
    if "--wait" in sys.argv:
        wait = float(sys.argv[sys.argv.index("--wait") + 1])
    layer = 3
    if "--layer" in sys.argv:
        layer = int(sys.argv[sys.argv.index("--layer") + 1])
    print("=" * 62)
    print(" LeKiwi 설정 점검   %s   (1~%d층)"
          % (os.environ.get("LEKIWI_NAME") or _profile_name(), layer))
    print("=" * 62)
    check_config_files()
    check_devices(layer)
    try:
        check_ros(wait, layer)
    except Exception as e:
        say(BAD, "ROS 점검 실패: %s" % e)
    print("\n" + "=" * 62)
    if fails:
        print(" 실패 %d건 -- 아래층부터 고칠 것" % len(fails))
        for f in fails:
            print("   - " + f)
        sys.exit(1)
    nxt = {1: "layer:=2 로 다시", 2: "layer:=3 으로 다시",
           3: "lekiwi_odom_test.launch.py"}[min(layer, 3)]
    print(" 전부 통과. 다음: %s" % nxt)


if __name__ == "__main__":
    main()
