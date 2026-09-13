#!/usr/bin/env python3
"""에이보가 판단한 약통 색상(도메인 77)을 LeKiwi 왕복 명령(도메인 42)으로 넘긴다.

  입력  pickup/medicine/red|blue|green  std_msgs/String  data="red"   (에이보 dialogue_node)
  출력  /abo/command                    std_msgs/String  "fetch <목적지> color:<색>"
                                                               (LeKiwi abo_nav_bridge.py)

  되돌림 (LeKiwi -> 에이보, 에이보 동반 웹앱·터미널 뷰어가 "지금 무엇을 하는지" 보여주도록)
  입력  /abo/state, /abo/status         std_msgs/String  (LeKiwi abo_nav_bridge.py, 도메인 42)
  출력  /lekiwi/mission_state           std_msgs/String  JSON (도메인 77)
          {"state", "action_text": "빨간색 약을 향해 가는 중", "status", "color", "target", "dry_run", "ts"}
          명령을 넘긴 순간엔 state="sent". 문구는 mission_text.py 한 곳에서 만든다.

  ** 이 노드는 노트북에서 돈다 (에이보 whisper/dialogue 와 같은 노트북). **

에이보(파이+노트북)는 ROS_DOMAIN_ID 77, LeKiwi 는 42 라서 토픽이 서로 안 보인다.
이 노드는 한 프로세스 안에서 두 도메인에 각각 참여해 경계를 넘긴다 -- 셸의
ROS_DOMAIN_ID 는 무시하고 --abo-domain/--lekiwi-domain 을 쓴다.

    python3 medicine_relay.py                # 받으면 LeKiwi 로 넘김
    python3 medicine_relay.py --dry-run      # 받기만 하고 로그 (에이보 쪽 시험용)

같은 와이파이가 아니면 nav/shell/ros_peers.sh 를 먼저 source 할 것.
"""
import argparse
import threading

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from color_intent import COLORS
from mission_text import mission_state_json


def fetch_command(dest, color):
    return f"fetch {dest} color:{color}"


class MedicineRelay:
    """에이보 쪽 노드에서 색상 토픽을 구독하고, LeKiwi 쪽 발행자로 명령을 낸다."""

    def __init__(self, abo_node, lekiwi_node, dest, dry_run):
        self.log = abo_node.get_logger()
        self.dest = dest
        self.dry_run = dry_run
        self.pub = lekiwi_node.create_publisher(String, "/abo/command", 10)
        # 되돌림: 르키위 상태 -> 에이보 쪽 한 줄 문장
        self.state_pub = abo_node.create_publisher(String, "/lekiwi/mission_state", 10)
        self.color = None
        self.last_status = ""
        self.last_state = None
        lekiwi_node.create_subscription(String, "/abo/state", self.on_lekiwi_state, 10)
        lekiwi_node.create_subscription(String, "/abo/status", self.on_lekiwi_status, 10)
        for color in COLORS:
            abo_node.create_subscription(
                String, f"pickup/medicine/{color}",
                lambda msg, c=color: self.on_color(c, msg), 10)

    def on_color(self, color, msg):
        if msg.data != color:
            self.log.warn(f"pickup/medicine/{color} 에 data={msg.data!r} -- 토픽 색({color})을 쓴다")
        self.log.info(f"수신: pickup/medicine/{color}")
        cmd = fetch_command(self.dest, color)
        self.color = color
        self.last_status = ""
        self.last_state = "sent"
        self.publish_state("sent")
        if self.dry_run:
            self.log.info(f"--dry-run 이라 LeKiwi 로 안 보냄 (보냈다면 /abo/command '{cmd}')")
            return
        if self.pub.get_subscription_count() == 0:
            # 발행은 한다. 다만 LeKiwi 의 abo_nav_bridge 가 안 보이면 받을 곳이 없다.
            self.log.warn("/abo/command 구독자가 안 보인다 -- LeKiwi 쪽 abo_nav_bridge, "
                          "도메인, ROS_STATIC_PEERS 확인")
        self.pub.publish(String(data=cmd))
        self.log.info(f"LeKiwi 로 전달: /abo/command '{cmd}'")

    # --- 되돌림 (LeKiwi 도메인 콜백, 별도 스레드에서 불린다) ---
    def on_lekiwi_status(self, msg):
        # 브리지는 say() 에서 status 와 state 를 연달아 보내지만 **서로 다른 토픽**이라
        # 도착 순서가 바뀔 수 있다. 설명이 늦게 오면 이미 보낸 상태에 새 설명을 붙여
        # 한 번 더 보낸다 -- 그래야 마지막 표시가 한 단계 늦은 설명으로 남지 않는다.
        self.last_status = msg.data
        if self.last_state:
            self.publish_state(self.last_state)

    def on_lekiwi_state(self, msg):
        self.last_state = msg.data
        self.publish_state(msg.data)

    def publish_state(self, state):
        payload = mission_state_json(state, self.last_status, self.color, self.dest, self.dry_run)
        self.state_pub.publish(String(data=payload))
        self.log.info(f"에이보로 상태 전달: {payload}")


def main():
    ap = argparse.ArgumentParser(description="에이보 색상 토픽 -> LeKiwi /abo/command 중계 (노트북)")
    ap.add_argument("--abo-domain", type=int, default=77)
    ap.add_argument("--lekiwi-domain", type=int, default=42)
    ap.add_argument("--dest", default="center", help="waypoints.yaml 의 목적지 이름")
    ap.add_argument("--dry-run", action="store_true", help="수신 로그만 남기고 LeKiwi 로 보내지 않는다")
    a = ap.parse_args()

    abo_ctx, lekiwi_ctx = Context(), Context()
    rclpy.init(context=abo_ctx, domain_id=a.abo_domain)
    rclpy.init(context=lekiwi_ctx, domain_id=a.lekiwi_domain)
    abo_node = Node("medicine_relay_abo", context=abo_ctx)
    lekiwi_node = Node("medicine_relay_lekiwi", context=lekiwi_ctx)
    MedicineRelay(abo_node, lekiwi_node, a.dest, a.dry_run)
    abo_node.get_logger().info(
        f"준비됨. 에이보 도메인 {a.abo_domain} pickup/medicine/{{{','.join(COLORS)}}} -> "
        f"LeKiwi 도메인 {a.lekiwi_domain} /abo/command (목적지 {a.dest}"
        f"{', dry-run' if a.dry_run else ''})")

    # 에이보 쪽 구독(색상 토픽)은 메인 스레드, LeKiwi 쪽 구독(/abo/state 되돌림)은
    # 별도 스레드에서 돈다 -- 실행기는 컨텍스트마다 따로 있어야 한다.
    lk_ex = SingleThreadedExecutor(context=lekiwi_ctx)
    lk_ex.add_node(lekiwi_node)
    threading.Thread(target=lk_ex.spin, daemon=True).start()
    ex = SingleThreadedExecutor(context=abo_ctx)
    ex.add_node(abo_node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        lk_ex.shutdown()
        abo_node.destroy_node()
        lekiwi_node.destroy_node()
        abo_ctx.try_shutdown()
        lekiwi_ctx.try_shutdown()


if __name__ == "__main__":
    main()
