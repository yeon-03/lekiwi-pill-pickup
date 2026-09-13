#!/usr/bin/env python3
"""medicine_relay 의 콜백 로직을 rclpy 없이 검증한다.

실제 두 도메인 간 전달(디스커버리, ROS_STATIC_PEERS)은 못 잡는다 -- 그건
docs/nav/abo-color-topic-test.md 순서대로 실물에서 확인할 것.

  python3 -m unittest test_medicine_relay_mock.py -v
"""
import sys
import types
import unittest


class FakePublisher:
    def __init__(self, topic, subs=1):
        self.topic = topic
        self.subs = subs
        self.published = []

    def publish(self, msg):
        self.published.append(msg.data)

    def get_subscription_count(self):
        return self.subs


class FakeLogger:
    def __init__(self):
        self.lines = []

    def info(self, m): self.lines.append(("info", m))
    def warn(self, m): self.lines.append(("warn", m))


class FakeNode:
    def __init__(self):
        self.subs = {}
        self.pubs = {}
        self.logger = FakeLogger()

    def create_subscription(self, _type, topic, cb, _qos):
        self.subs[topic] = cb

    def create_publisher(self, _type, topic, _qos):
        self.pubs[topic] = FakePublisher(topic)
        return self.pubs[topic]

    def get_logger(self):
        return self.logger


class FakeString:
    def __init__(self, data=""):
        self.data = data


_MODULES = ("rclpy", "rclpy.context", "rclpy.executors", "rclpy.node", "std_msgs", "std_msgs.msg")
for _name in _MODULES:
    sys.modules[_name] = types.ModuleType(_name)
sys.modules["rclpy.context"].Context = object
sys.modules["rclpy.executors"].SingleThreadedExecutor = object
sys.modules["rclpy.node"].Node = object
sys.modules["std_msgs.msg"].String = FakeString

import medicine_relay as mr  # noqa: E402


def tearDownModule():
    for name in _MODULES + ("medicine_relay",):
        sys.modules.pop(name, None)


class TestMedicineRelay(unittest.TestCase):
    def make(self, dry_run=False):
        self.abo, self.kiwi = FakeNode(), FakeNode()
        mr.MedicineRelay(self.abo, self.kiwi, "center", dry_run)
        return self.kiwi.pubs["/abo/command"]

    def test_subscribes_three_color_topics(self):
        self.make()
        self.assertEqual(set(self.abo.subs),
                         {"pickup/medicine/red", "pickup/medicine/blue", "pickup/medicine/green"})

    def test_red_forwards_fetch_command(self):
        pub = self.make()
        self.abo.subs["pickup/medicine/red"](FakeString("red"))
        self.assertEqual(pub.published, ["fetch center color:red"])

    def test_topic_color_wins_over_data(self):
        pub = self.make()
        self.abo.subs["pickup/medicine/green"](FakeString("blue"))
        self.assertEqual(pub.published, ["fetch center color:green"])
        self.assertIn("warn", [lvl for lvl, _ in self.abo.logger.lines])

    def test_dry_run_publishes_nothing(self):
        pub = self.make(dry_run=True)
        self.abo.subs["pickup/medicine/blue"](FakeString("blue"))
        self.assertEqual(pub.published, [])
        self.assertTrue(any("수신: pickup/medicine/blue" in m for _, m in self.abo.logger.lines))

    def test_no_subscriber_still_publishes_with_warning(self):
        pub = self.make()
        pub.subs = 0
        self.abo.subs["pickup/medicine/red"](FakeString("red"))
        self.assertEqual(pub.published, ["fetch center color:red"])
        self.assertIn("warn", [lvl for lvl, _ in self.abo.logger.lines])


if __name__ == "__main__":
    unittest.main()
