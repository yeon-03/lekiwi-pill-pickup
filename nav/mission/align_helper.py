#!/usr/bin/env python3
"""로봇을 손으로 옮겨 지도 원점에 맞출 때 쓰는 실시간 안내.

  python3 align_helper.py --map ~/maps/map_0826_1744.yaml

로봇이 지도 원점 (0,0,0) 에 있다고 가정했을 때의 정합도를 계속 출력하고,
어느 쪽으로 얼마나 옮겨야 하는지 알려준다. 로봇을 손으로 밀면서 숫자가
올라가는 걸 보면 된다. 90% 를 넘으면 충분하다.

읽기 전용 -- 모터를 전혀 건드리지 않는다. Nav2 없이 라이다만 있으면 된다.
"""
import rclpy, numpy as np, math, time, argparse, os
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

ap = argparse.ArgumentParser()
ap.add_argument("--map", required=True)
ap.add_argument("--search", type=float, default=0.6, help="탐색 반경 (m)")
ap.add_argument("--anchor", choices=["origin","tf"], default="origin",
                help="origin: 지도 원점에 맞추려는 경우 (홈 포지션 잡기). "
                     "tf: 지금 AMCL 이 믿는 자세 주변에서 최적을 찾는다 "
                     "(로봇을 조금씩 옮기며 정합도만 보고 싶을 때)")
a = ap.parse_args()

def load_map(path):
    import yaml
    from PIL import Image
    with open(path) as f: y = yaml.safe_load(f)
    ip = y["image"]
    if not os.path.isabs(ip): ip = os.path.join(os.path.dirname(os.path.abspath(path)), ip)
    img = np.flipud(np.array(Image.open(ip).convert("L")).astype(np.float32))
    p = img/255.0 if int(y.get("negate",0)) else (255.0-img)/255.0
    g = np.full(img.shape, -1, np.int16)
    g[p > float(y.get("occupied_thresh",0.65))] = 100
    g[p < float(y.get("free_thresh",0.196))] = 0
    return g, float(y["resolution"]), float(y["origin"][0]), float(y["origin"][1])

grid,res,ox,oy = load_map(a.map)
H,W = grid.shape
occ = grid >= 65
LY = math.radians(float(os.environ.get("LEKIWI_LIDAR_YAW","-30.0")))
print(f"지도 {os.path.basename(a.map)}  {W}x{H} @{res}  라이다 장착각 {math.degrees(LY):+.1f}도")
print("로봇을 조금씩 옮기면서 숫자를 보세요. Ctrl+C 로 종료.\n")

class S(Node):
    def __init__(self):
        super().__init__("align"); self.s=None
        self.buf=Buffer(); self.tl=TransformListener(self.buf,self)
        self.create_subscription(LaserScan,"/scan",lambda m:setattr(self,"s",m),qos_profile_sensor_data)
rclpy.init(); n=S()

def anchor_pose():
    """기준 자세 (base_link 의 map 상 자세)."""
    if a.anchor == "origin":
        return 0.0, 0.0, 0.0
    try:
        t=n.buf.lookup_transform("map","base_link",rclpy.time.Time())
        q=t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
    except Exception:
        return None

def score(R, ang, bx, by, byaw):
    """base_link 자세 -> 정합도. 라이다는 base 원점에 있으므로 회전만 반영."""
    a2 = ang + byaw + LY
    ex = bx + R*np.cos(a2); ey = by + R*np.sin(a2)
    i=((ex-ox)/res).astype(np.int32); j=((ey-oy)/res).astype(np.int32)
    v=(i>=0)&(i<W)&(j>=0)&(j<H)
    if not v.any(): return 0.0
    return occ[j[v],i[v]].sum()/len(R)

try:
    while rclpy.ok():
        t0=time.time(); n.s=None
        while n.s is None and time.time()-t0 < 3: rclpy.spin_once(n, timeout_sec=0.2)
        if n.s is None: print("스캔 없음..."); continue
        s=n.s
        R=np.array(s.ranges,dtype=np.float32)
        ang=s.angle_min+np.arange(len(R))*s.angle_increment
        ok=(R>=s.range_min)&(R<=s.range_max)&np.isfinite(R)
        R,ang=R[ok],ang[ok]
        if len(R)>150:
            k=np.linspace(0,len(R)-1,150).astype(int); R,ang=R[k],ang[k]

        ap_ = anchor_pose()
        if ap_ is None:
            print("map->base_link TF 없음 (AMCL 미실행?). --anchor origin 을 쓸 것.")
            time.sleep(1); continue
        ax_, ay_, ayaw_ = ap_
        here = score(R,ang,ax_,ay_,ayaw_)
        best=(-1,0,0,0)
        for dyaw in np.arange(-40,41,2.0):
            yr=ayaw_+math.radians(dyaw)
            for dx in np.arange(-a.search,a.search+1e-9,0.05):
                for dyy in np.arange(-a.search,a.search+1e-9,0.05):
                    sc=score(R,ang,ax_+dx,ay_+dyy,yr)
                    if sc>best[0]: best=(sc,dx,dyy,dyaw)
        sc,bx,by,byaw = best
        bar = "#"*int(here*40)
        c,sn = math.cos(ayaw_), math.sin(ayaw_)
        fwd  = -( bx*c + by*sn)      # 로봇 기준 앞뒤
        left = -(-bx*sn + by*c)      # 로봇 기준 좌우
        print(f"현재 {100*here:5.1f}% |{bar:<40}| 최적 {100*sc:5.1f}%  "
              f"→ 앞뒤 {fwd*100:+5.0f}cm  좌우 {left*100:+5.0f}cm  회전 {-byaw:+5.1f}도")
        time.sleep(0.5)
except KeyboardInterrupt:
    print("\n종료")
finally:
    if rclpy.ok(): rclpy.shutdown()
