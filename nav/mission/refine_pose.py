#!/usr/bin/env python3
"""현재 AMCL 자세 주변을 거리 기반 지표로 정밀 보정한다.

점유셀 적중률은 지도에 점이 많은 곳을 편애한다(허위 반환 흔적 때문).
여기서는 '끝점에서 가장 가까운 벽까지의 거리'를 최소화한다 -- ICP 잔차와 같은
개념이라 그런 편향이 없다.

  refine_pose.py <map.yaml> [--publish] [--range 0.4] [--yaw 15] [--at X Y YAW]

탐색은 성긴 격자에서 시작해 세 번에 걸쳐 조여 들어간다.  전수 조사를 하면
평가 횟수가 범위의 세제곱으로 늘어 --range 0.8 --yaw 40 에서 53만 번이 되는데,
단계별로 좁히면 같은 범위를 1.3만 번으로 훑는다 (42배).
"""
import rclpy, numpy as np, math, time, sys, os, yaml
from collections import deque
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener
from PIL import Image

MAP=sys.argv[1]; PUB="--publish" in sys.argv
def opt(k,d):
    return float(sys.argv[sys.argv.index(k)+1]) if k in sys.argv else d
RNG=opt("--range",0.4); YW=opt("--yaw",15.0)
AT=None                                   # --at X Y YAW : 탐색 기준점을 직접 준다
if "--at" in sys.argv:
    _i=sys.argv.index("--at"); AT=tuple(float(v) for v in sys.argv[_i+1:_i+4])
LY=math.radians(float(os.environ.get("LEKIWI_LIDAR_YAW","0.0")))

y=yaml.safe_load(open(MAP)); ip=y["image"]
if not os.path.isabs(ip): ip=os.path.join(os.path.dirname(os.path.abspath(MAP)),ip)
img=np.flipud(np.array(Image.open(ip).convert("L")).astype(np.float32))
p=(255.0-img)/255.0
occ=p>y.get("occupied_thresh",0.65)
free=img>=250                      # map_server 규약: 254/255=자유, 205=미탐색, 0=점유
res=float(y["resolution"]); ox,oy=y["origin"][0],y["origin"][1]
H,W=occ.shape
INF=10**6
dist=np.full((H,W),INF,np.int32); q=deque()
for i,j in zip(*np.nonzero(occ)): dist[i,j]=0; q.append((i,j))
while q:
    i,j=q.popleft()
    for di,dj in ((1,0),(-1,0),(0,1),(0,-1)):
        a,b=i+di,j+dj
        if 0<=a<H and 0<=b<W and dist[a,b]>dist[i,j]+1:
            dist[a,b]=dist[i,j]+1; q.append((a,b))
D=dist.astype(np.float32)*res

class P(Node):
    def __init__(self):
        super().__init__("rf"); self.s=None
        self.buf=Buffer(); self.tl=TransformListener(self.buf,self)
        self.create_subscription(LaserScan,"/scan",lambda m:setattr(self,"s",m),qos_profile_sensor_data)
        self.pub=self.create_publisher(PoseWithCovarianceStamped,"/initialpose",10)
rclpy.init(); n=P(); t0=time.time(); tf=None
while time.time()-t0<20:
    rclpy.spin_once(n,timeout_sec=0.1)
    if tf is None:
        try: tf=n.buf.lookup_transform("map","base_link",rclpy.time.Time())
        except Exception: pass
    if n.s is not None and tf is not None: break
if tf is not None:
    qq=tf.transform.rotation
    acy=math.degrees(math.atan2(2*(qq.w*qq.z+qq.x*qq.y),1-2*(qq.y*qq.y+qq.z*qq.z)))
    acx,acyy=tf.transform.translation.x,tf.transform.translation.y
elif AT is not None:
    acx,acyy,acy=AT
else:
    print("map->base_link TF 도 --at 도 없어 기준점을 못 잡는다."); sys.exit(1)

# 탐색 기준점.  --at 이 있으면 그쪽을 쓴다 -- pick 중에는 오도메트리가 끊겨
# AMCL 추정이 흐트러지지만, Nav2 가 실제로 향한 goal 좌표는 우리가 아는 값이라
# 흐트러진 추정보다 나은 출발점이다.
cx,cyy,cy = AT if AT is not None else (acx,acyy,acy)
s=n.s; R=np.array(s.ranges,dtype=np.float32)
ang=s.angle_min+np.arange(len(R))*s.angle_increment
ok=(R>=s.range_min)&(R<=s.range_max)&np.isfinite(R); R,ang=R[ok],ang[ok]
if len(R)>220:
    k=np.linspace(0,len(R)-1,220).astype(int); R,ang=R[k],ang[k]
ca,sa=np.cos(ang),np.sin(ang)
print(f"AMCL ({acx:+.3f},{acyy:+.3f}) yaw {acy:+.1f}도,  빔 {len(R)}")
if AT is not None:
    print(f"기준 ({cx:+.3f},{cyy:+.3f}) yaw {cy:+.1f}도  <- goal 좌표")

CAP=0.30                           # 이상치 상한 -- 소수 빔이 결과를 지배하지 않게

def cost(x,yv,yd):
    # 로봇 자신이 지도 밖이거나 벽 속이면 후보 자격이 없다.
    ri=int((x-ox)/res); rj=int((yv-oy)/res)
    if not (0<=ri<W and 0<=rj<H): return 9e9,0,0
    if occ[rj,ri]: return 9e9,0,0
    # 로봇은 자기가 지나온 곳, 즉 '지도에 자유공간으로 찍힌 곳' 근처에 있다.
    # 미탐색 한복판을 후보로 두면 빔이 대부분 미지 영역에 떨어져 벌점을 피하고
    # 엉뚱한 곳이 최적으로 뽑힌다 (실제로 미탐색 (2.08,0.22) 을 93% 로 보고함).
    # 로봇 셀 자체는 자기 몸에 가려 미탐색일 수 있으므로 반경 3칸을 본다.
    if not free[max(0,rj-3):rj+4, max(0,ri-3):ri+4].any(): return 9e9,0,0
    yr=math.radians(yd)+LY; c,sn=math.cos(yr),math.sin(yr)
    ex=x+R*(ca*c-sa*sn); ey=yv+R*(ca*sn+sa*c)
    i=((ex-ox)/res).astype(np.int32); j=((ey-oy)/res).astype(np.int32)
    v=(i>=0)&(i<W)&(j>=0)&(j<H)
    # 지도 밖으로 나간 끝점은 '버리지 않고' 최대 벌점을 준다.  버리면 지도
    # 가장자리 밖 자세가 안 맞는 빔만 골라 버려서 점수를 조작할 수 있다
    # (실제로 로봇을 지도 밖 y=-0.893 에 놓고 83% 로 보고한 적이 있다).
    d=np.full(len(R), CAP, np.float32)
    d[v]=np.minimum(D[j[v],i[v]], CAP)
    return float(d.mean()), float((d<0.05).mean()), float((d>=CAP).mean())

def search(px,py,pyaw,rng,yw,xs,ys):
    """(px,py,pyaw) 주변 +-rng / +-yw 를 xs,ys 간격으로 훑어 최저 비용을 찾는다."""
    best=(9e9,px,py,pyaw); nev=0
    for yd in np.arange(pyaw-yw, pyaw+yw+1e-9, ys):
        for dx in np.arange(-rng,rng+1e-9,xs):
            for dy in np.arange(-rng,rng+1e-9,xs):
                c0,_,_=cost(px+dx,py+dy,yd); nev+=1
                if c0<best[0]: best=(c0,px+dx,py+dy,yd)
    return best,nev

# 단계별로 (범위, yaw범위, 위치간격, yaw간격).  1단계가 전체를 훑고,
# 이후는 직전 최적점 주변만 촘촘히 본다.
STAGES=[(RNG,   YW,  0.08, 3.0),
        (0.10,  3.0, 0.03, 1.0),
        (0.03,  1.0, 0.01, 0.5)]
best=(9e9,cx,cyy,cy); tot=0; tS=time.time()
for k,(rg,yw,xs,ys) in enumerate(STAGES):
    px,py,pyaw = (cx,cyy,cy) if k==0 else (best[1],best[2],best[3])
    b,nev=search(px,py,pyaw,rg,yw,xs,ys); tot+=nev
    if b[0]<best[0]: best=b
    print(f"  {k+1}단계 (+-{rg:.2f}m/{yw:.0f}도, {xs*100:.0f}cm/{ys:.1f}도): "
          f"{nev:6d}회 -> 비용 {best[0]:.4f}")
print(f"  합계 {tot}회, {time.time()-tS:.1f}초")
c0,bx,by,byaw=best
m0,h0,o0=cost(acx,acyy,acy); m1,h1,o1=cost(bx,by,byaw)
print(f"\n{'':>10} {'평균거리':>9} {'<5cm':>7} {'>=30cm':>8}")
print(f"  AMCL     {m0:8.3f}m {100*h0:6.1f}% {100*o0:7.1f}%")
if AT is not None:
    mg,hg,og=cost(cx,cyy,cy)
    print(f"  goal     {mg:8.3f}m {100*hg:6.1f}% {100*og:7.1f}%")
print(f"  보정후   {m1:8.3f}m {100*h1:6.1f}% {100*o1:7.1f}%")
print(f"\n보정: ({bx:+.3f},{by:+.3f}) yaw {byaw:+.1f}도  "
      f"(AMCL 대비 이동 {math.hypot(bx-acx,by-acyy)*100:.1f} cm, 회전 {byaw-acy:+.1f}도)")
# 보정 결과가 지금 AMCL 보다 나쁘면 발행하지 않는다.  --at 을 줬는데 로봇이
# 실제로는 그 goal 에 못 갔을 때(주행 실패, 큰 회전 오차) 좁은 탐색창이
# 엉뚱한 국소 최적에 빠지는데, 그걸 AMCL 에 밀어넣으면 위치를 잃는다.
if PUB and m1 > m0 + 1e-4:
    print(f"   발행 안 함: 보정후({m1:.3f}m)가 현재 AMCL({m0:.3f}m)보다 나쁘다.")
    PUB=False
if PUB:
    mm=PoseWithCovarianceStamped(); mm.header.frame_id="map"
    mm.pose.pose.position.x=float(bx); mm.pose.pose.position.y=float(by)
    yr=math.radians(byaw)
    mm.pose.pose.orientation.z=math.sin(yr/2); mm.pose.pose.orientation.w=math.cos(yr/2)
    cov=[0.0]*36; cov[0]=0.03; cov[7]=0.03; cov[35]=0.01
    mm.pose.covariance=cov
    for _ in range(5):
        mm.header.stamp=n.get_clock().now().to_msg()
        n.pub.publish(mm); rclpy.spin_once(n,timeout_sec=0.15)
    print("   /initialpose 로 발행했다.")
rclpy.shutdown()
