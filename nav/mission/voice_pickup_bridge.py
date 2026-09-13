#!/usr/bin/env python3
"""발화(음성 인식 텍스트) -> 약통 색상 인식 -> pickup/medicine/<색상> 발행.

  입력  ~voice_text_topic (기본 /abo/voice_text)   std_msgs/String
          음성 인식된 원문 텍스트. 예: "빨간색 약 좀 갖다줘"
          기본 토픽명은 임시값이다 -- 에이보 쪽 실제 발화 파이프라인(별도
          저장소) 토픽에 맞춰 파라미터로 바꿔 배선할 것.

  출력  pickup/medicine/red    std_msgs/String   data: "red"
        pickup/medicine/blue   std_msgs/String   data: "blue"
        pickup/medicine/green  std_msgs/String   data: "green"
          인식된 색상의 토픽에만 1회 발행한다.
        /abo/status            std_msgs/String   사람이 읽는 한 줄(기존 관례,
                                                  abo_nav_bridge.py 와 동일 패턴)

  색상 인식 자체는 color_intent.extract_color 가 한다(빨강/파랑/초록 세 가지만
  지원, 같은 디렉터리에 있음). 못 알아들었거나 애매하면(두 색 이상 언급) 아무
  pickup 토픽도 발행하지 않고 상태만 안내한다 -- 틀린 색 약을 집어오는 것보다
  그게 안전하다.

  다음 단계(lekiwi_yolo_pick.py 로 실제 집기)와 잇는 법: 이 세 토픽을
  pick_adapter.py 가 구독하도록 확장하고, 받으면 --target_color=<색> 로
  lekiwi_yolo_pick.py 를 실행하면 된다 -- 이 파일은 그 앞단(발화->색상->토픽)
  까지만 담당한다.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from color_intent import extract_color, COLORS, COLOR_KR


class VoicePickupBridge(Node):
    def __init__(self):
        super().__init__("voice_pickup_bridge")
        self.declare_parameter("voice_text_topic", "/abo/voice_text")
        topic = self.get_parameter("voice_text_topic").value

        self.pub_status = self.create_publisher(String, "/abo/status", 10)
        self.pub_pickup = {
            color: self.create_publisher(String, f"pickup/medicine/{color}", 10)
            for color in COLORS
        }
        self.create_subscription(String, topic, self.on_voice_text, 10)
        self.say(f"대기 중입니다. (입력 토픽 {topic})")
        self.get_logger().info(
            f"입력 토픽 {topic}, 출력 토픽 pickup/medicine/{{{','.join(COLORS)}}}")

    def say(self, text):
        self.pub_status.publish(String(data=text))
        self.get_logger().info(text)

    def on_voice_text(self, msg):
        try:
            color = extract_color(msg.data)
        except Exception as e:
            # extract_color 는 원래 예외를 던지지 않지만, 콜백에서 예외가
            # 새면 executor 가 spin 을 멈출 수 있어 방어적으로 한 번 더 막는다.
            self.get_logger().error(f"색상 인식 중 오류: {e}")
            self.say("죄송해요, 처리 중 문제가 생겼어요.")
            return

        if color is None:
            self.say("무슨 색 약인지 못 알아들었어요. 빨강, 파랑, 초록 중에서 말씀해 주세요.")
            return

        self.pub_pickup[color].publish(String(data=color))
        self.say(f"{COLOR_KR[color]} 약을 가져올게요.")


def main():
    rclpy.init()
    node = VoicePickupBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
