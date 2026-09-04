#!/usr/bin/env python3
"""현재 스캔이 지도와 가장 잘 맞는 로봇 자세를 찾는다. 읽기 전용.

  python3 find_pose.py                 # 방향은 지금 AMCL 값 +-20도, 위치만 탐색
  python3 find_pose.py --yaw 171       # 방향을 지정 (도, map 기준)
  python3 find_pose.py --global        # 방향까지 360도 전체 탐색
  python3 find_pose.py --publish       # 찾은 자세를 /initialpose 로 보낸다

로봇을 놓는 방향은 매번 거의 같고 위치만 달라지므로, 기본은 방향을 좁게
묶고 위치만 훑는다. 방향까지 열어두면 좌우 대칭인 방에서 엉뚱한 곳으로
수렴하기 쉽다.

주의: 여기서 말하는 방향은 base_link 의 +x 축, 즉 linear.x 를 양수로 줬을 때
로봇이 실제로 가는 방향이다. 카메라가 달린 쪽과 다를 수 있다.
"""
import rclpy, numpy as np, math, time, sys, argparse
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener

ap = argparse.ArgumentParser()
ap.add_argument("--yaw", type=float, default=None, help="방향 지정 (도, map 기준)")
ap.add_argument("--yaw-window", type=float, default=20.0, help="방향 탐색 폭 +-도")
ap.add_argument("--global", dest="glob", action="store_true", help="방향 360도 전체 탐색")
ap.add_argument("--publish", action="store_true", help="/initialpose 로 발행")
ap.add_argument("--step", type=float, default=0.05, help="위치 탐색 간격 (m)")
ap.add_argument("--near", type=float, default=None,
                help="현재 AMCL 자세 주변 +-N m 만 탐색 (정밀 보정용). 예: --near 0.5")
ap.add_argument("--map", default=None,
                help="지도 yaml 을 파일에서 직접 읽는다. 주면 Nav2 가 꺼져 있어도 된다 "
                     "(/scan 만 있으면 되므로 라이다 노드만 띄우면 충분하다)")
a = ap.parse_args()

mq = QoSProfile(depth=1); mq.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
mq.reliability = QoSReliabilityPolicy.RELIABLE

class P(Node):
    def __init__(self):
        super().__init__("findpose"); self.scan=None; self.map=None
        self.buf=Buffer(); self.tl=TransformListener(self.buf,self)
        self.create_subscription(LaserScan,"/scan",lambda m:setattr(self,"scan",m),qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid,"/map",lambda m:setattr(self,"map",m),mq)
        self.pub=self.create_publisher(PoseWithCovarianceStamped,"/initialpose",10)

def load_map_file(path):
    """map_server 없이 yaml+pgm 을 직접 읽어 점유격자로 만든다."""
    import yaml, os
    from PIL import Image
    with open(path) as f: y = yaml.safe_load(f)
    img_path = y["image"]
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(os.path.abspath(path)), img_path)
    a = np.array(Image.open(img_path).convert("L")).astype(np.float32)
    a = np.flipud(a)                       # ROS 지도는 y 가 아래에서 위
    neg = int(y.get("negate", 0))
    p = a/255.0 if neg else (255.0-a)/255.0    # 점유 확률
    ot = float(y.get("occupied_thresh", 0.65)); ft = float(y.get("free_thresh", 0.196))
    grid = np.full(a.shape, -1, dtype=np.int16)
    grid[p > ot] = 100
    grid[p < ft] = 0
    return grid, float(y["resolution"]), float(y["origin"][0]), float(y["origin"][1])

rclpy.init(); n=P(); t0=time.time()
cur_yaw=None; cur_xy=None
# base_link -> lidar_link. 탐색은 스캔(=lidar_link) 프레임에서 하지만 AMCL 에는
# base_link 자세를 줘야 한다. 이 변환을 빼먹으면 장착각(-30도)만큼 통째로
# 틀어진 자세를 발행하게 된다.
lidar_tf=None
need_map = a.map is None
while time.time()-t0 < 20:
    rclpy.spin_once(n, timeout_sec=0.1)
    if cur_yaw is None:
        try:
            tf=n.buf.lookup_transform("map","base_link",rclpy.time.Time())
            q=tf.transform.rotation
            cur_yaw=math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
            cur_xy=(tf.transform.translation.x, tf.transform.translation.y)
        except Exception: pass
    if lidar_tf is None:
        try:
            t=n.buf.lookup_transform("base_link","lidar_link",rclpy.time.Time())
            q2=t.transform.rotation
            lidar_tf=(t.transform.translation.x, t.transform.translation.y,
                      math.atan2(2*(q2.w*q2.z+q2.x*q2.y),1-2*(q2.y*q2.y+q2.z*q2.z)))
        except Exception: pass
    if n.scan is not None and (not need_map or n.map is not None) \
       and cur_yaw is not None and lidar_tf is not None: break
if n.scan is None:
    print("/scan 수신 실패 -- 라이다 노드가 떠 있는지 확인할 것"); raise SystemExit(1)
if lidar_tf is None:
    import os as _os
    ly = math.radians(float(_os.environ.get("LEKIWI_LIDAR_YAW", "-30.0")))
    lidar_tf = (0.0, 0.0, ly)
    print("base_link->lidar_link TF 없음. 프로파일 값 %.1f도 사용." % math.degrees(ly))
print("base_link -> lidar_link: (%.3f, %.3f) yaw %+.1f도"
      % (lidar_tf[0], lidar_tf[1], math.degrees(lidar_tf[2])))

if a.map is not None:
    grid, res, ox, oy = load_map_file(a.map)
    H, W = grid.shape
    print(f"지도 파일: {a.map}")
else:
    if n.map is None:
        print("/map 수신 실패. Nav2 가 꺼져 있으면 --map 으로 파일을 직접 지정할 것."); raise SystemExit(1)
    g=n.map; W,H,res=g.info.width,g.info.height,g.info.resolution
    ox,oy=g.info.origin.position.x,g.info.origin.position.y
    grid=np.array(g.data,dtype=np.int16).reshape(H,W)
occ=grid>=65; free=(grid>=0)&(grid<=25)

s=n.scan
R=np.array(s.ranges,dtype=np.float32)
ang=s.angle_min+np.arange(len(R))*s.angle_increment
ok=(R>=s.range_min)&(R<=s.range_max)&np.isfinite(R)
R,ang=R[ok],ang[ok]
if len(R)>180:
    idx=np.linspace(0,len(R)-1,180).astype(int); R,ang=R[idx],ang[idx]

if a.glob:
    yaws=np.arange(-180,180,2.0); mode="방향 360도 전체"
else:
    base = a.yaw if a.yaw is not None else (cur_yaw if cur_yaw is not None else 0.0)
    yaws=np.arange(base-a.yaw_window, base+a.yaw_window+0.1, 2.0)
    mode=f"방향 {base:+.1f}도 +-{a.yaw_window:.0f}도"
print(f"지도 {W}x{H} @{res:.3f}   빔 {len(R)}개   탐색: {mode}, 위치 간격 {a.step:.2f} m")

# 빈 공간 위 격자점만 후보로. --near 면 현재 자세 주변만.
if a.near is not None and cur_xy is not None:
    gx=np.arange(cur_xy[0]-a.near, cur_xy[0]+a.near+1e-9, a.step)
    gy=np.arange(cur_xy[1]-a.near, cur_xy[1]+a.near+1e-9, a.step)
    print("정밀 탐색: (%.2f, %.2f) 주변 +-%.2f m" % (cur_xy[0], cur_xy[1], a.near))
else:
    gx=np.arange(ox+res/2, ox+W*res, a.step)
    gy=np.arange(oy+res/2, oy+H*res, a.step)
cand=[]
for cy in gy:
    j=int((cy-oy)/res)
    if not (0<=j<H): continue
    for cx in gx:
        i=int((cx-ox)/res)
        if 0<=i<W and free[j,i]: cand.append((cx,cy))
print(f"후보 위치 {len(cand)}곳 x 방향 {len(yaws)}개 = {len(cand)*len(yaws):,} 조합")

cos_a=np.cos(ang); sin_a=np.sin(ang)
best=(-1,None); results=[]
for yd in yaws:
    yr=math.radians(yd); c,sn=math.cos(yr),math.sin(yr)
    rx=R*(cos_a*c-sin_a*sn); ry=R*(cos_a*sn+sin_a*c)
    for (cx,cy) in cand:
        i=((cx+rx-ox)/res).astype(np.int32); j=((cy+ry-oy)/res).astype(np.int32)
        v=(i>=0)&(i<W)&(j>=0)&(j<H)
        if not v.any(): continue
        sc=occ[j[v],i[v]].sum()/len(R)
        results.append((sc,cx,cy,yd))
        if sc>best[0]: best=(sc,(cx,cy,yd))

sc,(bx,by,byaw)=best
results.sort(reverse=True)

def to_base(lx_map, ly_map, lyaw_deg):
    """lidar_link 자세 -> base_link 자세."""
    lx, ly, lyaw = lidar_tf
    byaw_r = math.radians(lyaw_deg) - lyaw
    return (lx_map - (math.cos(byaw_r)*lx - math.sin(byaw_r)*ly),
            ly_map - (math.sin(byaw_r)*lx + math.cos(byaw_r)*ly),
            math.degrees(byaw_r))

print("\n=== 상위 5개 (base_link 기준) ===")
for r in results[:5]:
    a1,b1,c1 = to_base(r[1],r[2],r[3])
    print("   (%+.2f, %+.2f) yaw %+6.1f   정합 %5.1f%%" % (a1,b1,c1,100*r[0]))
bx,by,byaw = to_base(bx,by,byaw)
print("\n=== 최적 (base_link 기준, AMCL 에 넣을 값) ===")
print("   x %+.3f   y %+.3f   yaw %+.1f 도   정합 %.1f%%" % (bx,by,byaw,100*sc))
if sc<0.70:
    print("   ** 70%% 미만. 로봇이 지도 밖이거나 환경이 바뀌었을 수 있다. --global 을 시도해 볼 것. **")

# near 모드(기본)는 현재 AMCL 방향 주변만 본다. Nav2/AMCL 을 막 재시작했다면
# AMCL 은 (0,0,0) 이라 실제 방향이 탐색 창 밖일 수 있고, 그러면 좁은 창 안에서
# 억지로 고른 엉뚱한 자세가 나온다. 점수가 낮으면 그 신호이므로 막는다.
if a.publish and not a.glob and sc < 0.72:
    print()
    print(f"   ** 정합 {100*sc:.1f}% 로 낮다. 발행하지 않았다. **")
    print("   near 모드는 현재 AMCL 방향 +-%.0f도 만 본다. Nav2 를 재시작했거나" % a.yaw_window)
    print("   로봇을 많이 옮겼다면 방향이 이 창 밖이다. 다시 실행할 것:")
    print("      python3 ~/find_pose.py --global --publish")
    rclpy.shutdown()
    raise SystemExit(2)

if a.publish:
    m=PoseWithCovarianceStamped()
    m.header.frame_id="map"
    m.pose.pose.position.x=float(bx); m.pose.pose.position.y=float(by)
    yr=math.radians(byaw)
    m.pose.pose.orientation.z=math.sin(yr/2); m.pose.pose.orientation.w=math.cos(yr/2)
    cov=[0.0]*36; cov[0]=0.05; cov[7]=0.05; cov[35]=0.02
    m.pose.covariance=cov
    for _ in range(5):
        m.header.stamp=n.get_clock().now().to_msg()
        n.pub.publish(m); rclpy.spin_once(n, timeout_sec=0.2)
    print("\n   /initialpose 로 발행했다.")
else:
    print("\n   (발행 안 함. --publish 를 주면 AMCL 에 적용한다)")
rclpy.shutdown()
