"""lidar_test.py 의 추정 알고리즘을 합성 데이터로 검증한다.
방을 만들고, 알려진 장착각으로 스캔을 생성하고, 추정값이 그 각을 되찾는지 본다."""
import math, sys, importlib.util
import numpy as np

spec = importlib.util.spec_from_file_location("lt", sys.argv[1])
lt = importlib.util.module_from_spec(spec)
sys.modules["lt"] = lt
# rclpy import 를 피하려고 필요한 함수만 소스에서 뽑아 실행
src = open(sys.argv[1]).read()
ns = {"np": np, "math": math, "CAP": 0.25}
start = src.index("def residual(")
end = src.index("def main(")
exec(src[start:end], ns)
residual = ns["residual"]

WALLS = [(-2, -1.5, 3, -1.5), (-2, 2.0, 3, 2.0), (-2, -1.5, -2, 2.0), (3, -1.5, 3, 2.0),
         (0.5, 0.2, 1.6, 0.2)]      # 방 + 내부 칸막이 (비대칭성 확보)

def ray(px, py, ang, rmax=5.0):
    best = rmax
    dx, dy = math.cos(ang), math.sin(ang)
    for x1, y1, x2, y2 in WALLS:
        ex, ey = x2 - x1, y2 - y1
        den = dx * ey - dy * ex
        if abs(den) < 1e-12: continue
        t = ((x1 - px) * ey - (y1 - py) * ex) / den
        u = ((x1 - px) * dy - (y1 - py) * dx) / den
        if t > 0.05 and 0 <= u <= 1: best = min(best, t)
    return best

def scan_at(px, py, yaw, mount, n=180):
    """몸통 자세 (px,py,yaw), 장착각 mount 일 때 라이다 프레임 점들."""
    out = []
    for i in range(n):
        a_l = -math.pi + 2 * math.pi * i / n          # 라이다 프레임 각
        a_w = yaw + mount + a_l                        # 월드 각
        r = ray(px, py, a_w)
        if r < 4.9: out.append((r * math.cos(a_l), r * math.sin(a_l)))
    return np.array(out)

print("  참값(도)   추정(도)   오차     잔차")
ok = True
for true_mount_deg in (0.0, -30.0, 90.0, -150.0, 17.5):
    m = math.radians(true_mount_deg)
    ax, ay, ayaw = 0.0, 0.0, 0.3
    d = 0.30
    bx, by, byaw = ax + d * math.cos(ayaw), ay + d * math.sin(ayaw), ayaw
    PA = scan_at(ax, ay, ayaw, m)
    PB = scan_at(bx, by, byaw, m)
    # 몸통 프레임 이동량
    ddx, ddy = bx - ax, by - ay
    ca, sa = math.cos(-ayaw), math.sin(-ayaw)
    dx, dy = ca * ddx - sa * ddy, sa * ddx + ca * ddy
    cand = np.arange(-180.0, 180.0, 1.0)
    res = np.array([residual(PA, PB, dx, dy, 0.0, math.radians(t)) for t in cand])
    b = cand[int(res.argmin())]
    fine = np.arange(b - 2, b + 2.001, 0.1)
    rf = np.array([residual(PA, PB, dx, dy, 0.0, math.radians(t)) for t in fine])
    est = float(fine[int(rf.argmin())])
    err = (est - true_mount_deg + 180) % 360 - 180
    flag = "OK" if abs(err) <= 1.0 else "실패"
    if abs(err) > 1.0: ok = False
    print("  %+8.1f   %+8.1f   %+6.2f   %.4f  %s" % (true_mount_deg, est, err, rf.min(), flag))
print("\n  " + ("전부 통과" if ok else "일부 실패"))
