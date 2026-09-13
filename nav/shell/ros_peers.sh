# 같은 와이파이(서브넷)가 아니어도 ROS 2 노드끼리 서로 찾게 하는 고정 피어 설정.
# 실행하지 말고 source 한다. 에이보 파이 / 노트북 / LeKiwi 세 기기 모두 같은 내용.
#
#   source ros_peers.sh
#
# ROS 2 Jazzy 의 ROS_STATIC_PEERS: 멀티캐스트 대신 여기 적힌 IP 로 직접(유니캐스트)
# 디스커버리 패킷을 보낸다. IP 를 비워두면 아무것도 바꾸지 않는다(기존 멀티캐스트 그대로).
#
# !! 알아둘 것
#  - 서로 ping 이 돼야 한다. 피어 설정은 "찾는 방법"만 바꾸지 길을 만들어 주지 않는다.
#    각자 다른 공유기/핫스팟의 NAT 뒤이거나 AP 클라이언트 격리(eduroam 등)면 안 된다
#    -> 그때는 Tailscale 같은 VPN 을 세 기기에 깔고 VPN IP(100.x.x.x)를 적는다.
#  - ROS_DOMAIN_ID 는 따로 맞춘다: 파이<->노트북 77, 노트북<->LeKiwi 42.
#    노트북의 medicine_relay.py 가 두 도메인을 동시에 쓴다.
#  - 값을 바꾼 뒤엔 새 터미널에서 `ros2 daemon stop` 한 번 (데몬이 옛 설정을 기억한다).
#  - 방화벽이 UDP 7400~7700 대역을 막으면 안 된다.

ABO_PI_IP="${ABO_PI_IP:-}"      # 에이보 라즈베리파이
LAPTOP_IP="${LAPTOP_IP:-}"      # whisper/dialogue/medicine_relay 가 도는 노트북
LEKIWI_IP="${LEKIWI_IP:-}"      # LeKiwi (lekiwi01)

_peers=""
for _ip in "$ABO_PI_IP" "$LAPTOP_IP" "$LEKIWI_IP"; do
    [ -n "$_ip" ] && _peers="${_peers:+$_peers;}$_ip"
done
if [ -n "$_peers" ]; then
    export ROS_STATIC_PEERS="$_peers"
fi
unset _peers _ip
