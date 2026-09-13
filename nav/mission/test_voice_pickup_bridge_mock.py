#!/usr/bin/env python3
"""voice_pickup_bridge 노드의 콜백 로직(on_voice_text)을 rclpy 없이 검증한다.

이 맥에는 실제 rclpy 가 없어서 최소 스텁으로 노드 로직을 검증한다. 실제 ROS2
환경(로봇 등)에서는 실물 pub/sub 로도 확인할 것 -- 이 파일은 그걸 대체하지
않는다(디스커버리, QoS, 토픽 이름 오타 같은 실제 ROS2 배선 문제는 못 잡는다).

  python3 -m unittest test_voice_pickup_bridge_mock.py -v
"""
import sys
import types
import unittest
from unittest.mock import patch


def _install_fake_ros_modules():
    """voice_pickup_bridge.py 가 실제로 쓰는 rclpy/std_msgs API 만큼만
    흉내내는 최소 스텁."""

    class _Param:
        def __init__(self, value):
            self.value = value

    class FakePublisher:
        def __init__(self, topic):
            self.topic = topic
            self.published = []

        def publish(self, msg):
            self.published.append(msg.data)

    class _NullLogger:
        def info(self, *a, **k): pass
        def warn(self, *a, **k): pass
        def error(self, *a, **k): pass

    class FakeNode:
        def __init__(self, name):
            self._name = name
            self._params = {}

        def declare_parameter(self, name, default):
            self._params[name] = _Param(default)

        def get_parameter(self, name):
            return self._params[name]

        def create_publisher(self, msg_type, topic, qos):
            return FakePublisher(topic)

        def create_subscription(self, msg_type, topic, callback, qos):
            return None

        def get_logger(self):
            return _NullLogger()

    class FakeString:
        def __init__(self, data=""):
            self.data = data

    fake_rclpy = types.ModuleType("rclpy")
    fake_rclpy.init = lambda *a, **k: None
    fake_rclpy.shutdown = lambda *a, **k: None
    fake_rclpy.spin = lambda *a, **k: None
    fake_rclpy_node = types.ModuleType("rclpy.node")
    fake_rclpy_node.Node = FakeNode
    fake_rclpy.node = fake_rclpy_node

    fake_std_msgs = types.ModuleType("std_msgs")
    fake_std_msgs_msg = types.ModuleType("std_msgs.msg")
    fake_std_msgs_msg.String = FakeString
    fake_std_msgs.msg = fake_std_msgs_msg

    sys.modules["rclpy"] = fake_rclpy
    sys.modules["rclpy.node"] = fake_rclpy_node
    sys.modules["std_msgs"] = fake_std_msgs
    sys.modules["std_msgs.msg"] = fake_std_msgs_msg


_install_fake_ros_modules()

import voice_pickup_bridge as vpb  # noqa: E402  (스텁 설치 후 임포트)
from color_intent import COLORS  # noqa: E402
from std_msgs.msg import String  # noqa: E402  (위에서 설치한 FakeString)


def tearDownModule():
    # 이 파일 밖으로 가짜 rclpy/std_msgs 가 새지 않게 정리한다.
    for name in ("rclpy", "rclpy.node", "std_msgs", "std_msgs.msg", "voice_pickup_bridge"):
        sys.modules.pop(name, None)


class TestVoicePickupBridgeCallback(unittest.TestCase):
    def setUp(self):
        self.node = vpb.VoicePickupBridge()

    def _send(self, text):
        self.node.on_voice_text(String(data=text))

    def _pickup(self, color):
        return self.node.pub_pickup[color].published

    def _status(self):
        return self.node.pub_status.published

    def test_red_publishes_only_red_topic(self):
        self._send("빨간색 약 좀 갖다줘")
        self.assertEqual(self._pickup("red"), ["red"])
        self.assertEqual(self._pickup("blue"), [])
        self.assertEqual(self._pickup("green"), [])
        self.assertTrue(self._status())

    def test_blue_and_green_each_only_own_topic(self):
        self._send("파란 약통 집어줘")
        self.assertEqual(self._pickup("blue"), ["blue"])
        self._send("초록색 약이요")
        self.assertEqual(self._pickup("green"), ["green"])
        self.assertEqual(self._pickup("red"), [])

    def test_no_color_publishes_nothing_but_status(self):
        self._send("약 좀 찾아서 갖다줘")
        for c in COLORS:
            self.assertEqual(self._pickup(c), [])
        self.assertTrue(self._status())

    def test_ambiguous_publishes_nothing(self):
        self._send("빨간거 말고 파란거")
        for c in COLORS:
            self.assertEqual(self._pickup(c), [])

    def test_empty_text_does_not_crash_node(self):
        self._send("")
        for c in COLORS:
            self.assertEqual(self._pickup(c), [])
        self._send("초록색 약 좀")
        self.assertEqual(self._pickup("green"), ["green"])

    def test_color_extraction_exception_does_not_crash_node(self):
        with patch.object(vpb, "extract_color", side_effect=RuntimeError("boom")):
            self._send("아무거나")
        for c in COLORS:
            self.assertEqual(self._pickup(c), [])
        self.assertTrue(self._status())
        self._send("빨간색 약")
        self.assertEqual(self._pickup("red"), ["red"])

    def test_rapid_sequential_requests_each_handled_independently(self):
        for text, expect in [("빨간색 약", "red"), ("파란색 약", "blue"), ("초록색 약", "green")]:
            self._send(text)
            self.assertEqual(self._pickup(expect), [expect])


if __name__ == "__main__":
    unittest.main()
