#!/usr/bin/env python3
"""에이보 대화를 터미널 글자로 본다 -- 스피커가 안 나올 때 대답 확인용. 수신 전용.

  source /opt/ros/jazzy/setup.bash
  source nav/shell/ros_peers.sh        # 같은 와이파이가 아니면
  python3 nav/tools/abo_console.py     # 에이보 도메인 77

보여주는 것 (전부 받기만 한다, 아무것도 발행하지 않는다):
  /user_input               내가 한 말 (Whisper 전사)
  /llm_response             에이보 대답 -- TTS 가 읽었어야 할 문장, 문장 단위로 온다
  /llm_response_done        대답 한 턴의 끝
  pickup/medicine/<색>      에이보가 만든 약통 픽업 토픽
  /lekiwi/mission_state     르키위가 지금 하는 일 (medicine_relay.py 가 돌려보냄)
"""
import argparse
import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String

COLORS = ("red", "blue", "green")


def stamp():
    return time.strftime("%H:%M:%S")


class AboConsole(Node):
    def __init__(self, out=sys.stdout):
        super().__init__("abo_console")
        self.out = out
        self.in_reply = False
        self.create_subscription(String, "/user_input", self.on_user_input, 10)
        self.create_subscription(String, "/llm_response", self.on_reply, 10)
        self.create_subscription(Empty, "/llm_response_done", self.on_reply_done, 10)
        for color in COLORS:
            self.create_subscription(String, f"pickup/medicine/{color}",
                                     lambda m, c=color: self.on_pickup(c, m), 10)
        self.create_subscription(String, "/lekiwi/mission_state", self.on_mission, 10)
        self.say(f"[{stamp()}] 에이보 대화 보기 시작 (도메인 {os.environ.get('ROS_DOMAIN_ID', '0')}). "
                 "Ctrl+C 로 종료")

    def say(self, line):
        print(line, file=self.out, flush=True)

    def on_user_input(self, msg):
        self.in_reply = False
        self.say(f"\n[{stamp()}] 🗣  나     : {msg.data}")

    def on_reply(self, msg):
        text = (msg.data or "").strip()
        if not text:
            return
        prefix = f"[{stamp()}] 🤖 에이보 : " if not self.in_reply else " " * 20
        self.in_reply = True
        self.say(prefix + text)

    def on_reply_done(self, _msg):
        self.in_reply = False

    def on_pickup(self, color, msg):
        self.say(f"[{stamp()}] 📦 토픽   : /pickup/medicine/{color}  (data: {msg.data!r})")

    def on_mission(self, msg):
        try:
            d = json.loads(msg.data)
            text = d.get("action_text") or d.get("state")
            extra = " (연습 모드 -- 르키위로는 안 보냄)" if d.get("dry_run") else ""
        except ValueError:
            text, extra = msg.data, ""
        self.say(f"[{stamp()}] 🚗 르키위 : {text}{extra}")


def main():
    ap = argparse.ArgumentParser(description="에이보 대화를 터미널 글자로 본다 (수신 전용)")
    ap.add_argument("--domain", type=int, default=None, help="ROS_DOMAIN_ID (기본: 환경변수, 없으면 77)")
    a = ap.parse_args()
    os.environ["ROS_DOMAIN_ID"] = str(a.domain if a.domain is not None else os.environ.get("ROS_DOMAIN_ID", "77"))
    rclpy.init()
    node = AboConsole()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
