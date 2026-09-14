#!/usr/bin/env python3
"""르키위 약통 심부름 기록기 -- 에이보 말·르키위 상태·집기 결과를 시각과 함께 파일로 남긴다.

집기가 실패했을 때 "어느 단계에서 무엇이 왔고 무엇이 안 왔는지"를 나중에 볼 수 있게 한다
(2026-09-13 웹 데모 첫 집기 실패는 서버 로그에 버튼·상태조회 기록뿐이라 원인을 못 찾았다).

받기만 한다 -- 로봇에 명령을 보내지 않고, ZMQ 로 로봇에 붙지도 않는다. 미션 중에 켜 둬도 안전하다.

    # 노트북에서 (중계기 medicine_relay.py 와 같은 네트워크 설정)
    export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
    export ROS_STATIC_PEERS="<에이보 IP>;<르키위 IP>"
    python3 nav/tools/mission_topic_logger.py                     # ~/pickplace_logs/<날짜_시각>/topics.log
    python3 nav/tools/mission_topic_logger.py --out run1.log      # 파일 지정

한 줄 예:
    [18:52:43.278] 🦾집기요청 center:green
    [18:52:43.301] 🤖에이보말 도착했어요. 약을 집을게요.
    [18:53:37.774] ✅집기결과 True

에이보(도메인 77)
  /llm_response            에이보가 TTS 로 말하는 문장 (스피커가 없어도 여기서 확인)
  /lekiwi/mission_state    중계기가 되돌려주는 르키위 상태 JSON (state, picked ...)
  pickup/medicine/<색>     음성·웹 버튼 요청
  /user_input              음성 인식 결과
르키위(도메인 42)
  /abo/command /abo/state /abo/status   중계기 명령과 브리지 상태
  /abo/pick_request /abo/pick_done      집기 요청과 결과 (노트북 pick_adapter.py)
"""
import argparse
import threading
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from std_msgs.msg import Bool, String

ABO_TOPICS = [
    (String, "/llm_response", "🤖에이보말"),
    (String, "/lekiwi/mission_state", "📨르키위상태JSON"),
    (String, "pickup/medicine/red", "📦토픽red"),
    (String, "pickup/medicine/blue", "📦토픽blue"),
    (String, "pickup/medicine/green", "📦토픽green"),
    (String, "/user_input", "🗣사용자"),
]
LEKIWI_TOPICS = [
    (String, "/abo/command", "➡️르키위명령"),
    (String, "/abo/state", "🚗브리지state"),
    (String, "/abo/status", "💬브리지status"),
    (String, "/abo/pick_request", "🦾집기요청"),
    (Bool, "/abo/pick_done", "✅집기결과"),
]


class Recorder:
    def __init__(self, path):
        self._f = open(path, "a", encoding="utf-8", buffering=1)
        self._lock = threading.Lock()

    def write(self, tag, text):
        now = time.time()
        stamp = time.strftime("%H:%M:%S", time.localtime(now)) + f".{int(now * 1000) % 1000:03d}"
        with self._lock:
            self._f.write(f"[{stamp}] {tag} {text}\n")


def spin_domain(domain, topics, rec):
    """도메인 하나를 자기 컨텍스트로 받아 별도 스레드에서 돈다 (한 프로세스에서 두 도메인)."""
    ctx = Context()
    # rclpy 가 Ctrl+C 를 가로채면 main 의 KeyboardInterrupt 가 안 떠 "종료" 줄이 안 남는다.
    rclpy.init(context=ctx, domain_id=domain, signal_handler_options=SignalHandlerOptions.NO)
    node = Node(f"mission_topic_logger_{domain}", context=ctx)
    for typ, topic, tag in topics:
        node.create_subscription(typ, topic, lambda m, tag=tag: rec.write(tag, m.data), 50)
    ex = SingleThreadedExecutor(context=ctx)
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    return ctx, ex, node, th


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="", help="기록 파일 (기본: ~/pickplace_logs/<날짜_시각>/topics.log)")
    ap.add_argument("--abo-domain", type=int, default=77)
    ap.add_argument("--lekiwi-domain", type=int, default=42)
    a = ap.parse_args()

    out = Path(a.out).expanduser() if a.out else (
        Path.home() / "pickplace_logs" / datetime.now().strftime("%Y%m%d_%H%M%S") / "topics.log")
    out.parent.mkdir(parents=True, exist_ok=True)
    rec = Recorder(out)
    domains = [spin_domain(a.abo_domain, ABO_TOPICS, rec), spin_domain(a.lekiwi_domain, LEKIWI_TOPICS, rec)]
    rec.write("기록", f"시작 (에이보 {a.abo_domain}, 르키위 {a.lekiwi_domain})")
    print(f"기록 중: {out}  (Ctrl+C 로 종료)", flush=True)
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        rec.write("기록", "종료")
        # 받는 스레드를 먼저 멈추고 노드를 정리한 뒤 ROS 를 내린다 -- 스레드가 돌고 있는데
        # rclpy.shutdown 을 먼저 하면 C++ 쪽이 "terminate called recursively" 로 강제 종료된다.
        for ctx, ex, node, th in domains:
            try:
                ex.shutdown(timeout_sec=1.0)
                node.destroy_node()
                rclpy.shutdown(context=ctx)
            except Exception:
                pass
            th.join(timeout=2.0)


if __name__ == "__main__":
    main()
