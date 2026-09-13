#!/usr/bin/env python3
"""arm_hold.hold_arm 을 가짜 버스로 검증한다 (로봇·ROS 불필요).

  python3 nav/nodes/test_arm_hold.py -v
"""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from arm_hold import hold_arm  # noqa: E402

TORQUE, POS, GOAL = 40, 56, 42


class FakeBus:
    def __init__(self, torque, pos):
        self.torque, self.pos, self.writes = dict(torque), dict(pos), []

    def r1(self, i, a):
        assert a == TORQUE
        return self.torque.get(i)

    def r2(self, i, a):
        assert a == POS
        return self.pos.get(i)

    def w1(self, i, a, v):
        self.writes.append(("w1", i, a, v))
        if a == TORQUE:
            self.torque[i] = v

    def w2(self, i, a, v):
        self.writes.append(("w2", i, a, v))


class TestHoldArm(unittest.TestCase):
    def test_torque_already_on_writes_nothing(self):
        bus = FakeBus({1: 1, 2: 1}, {1: 2000, 2: 2100})
        self.assertEqual(hold_arm(bus, [1, 2], TORQUE, POS, GOAL), ([], []))
        self.assertEqual(bus.writes, [])

    def test_torque_off_sets_goal_then_enables(self):
        bus = FakeBus({3: 0}, {3: 1234})
        self.assertEqual(hold_arm(bus, [3], TORQUE, POS, GOAL), ([3], []))
        self.assertEqual(bus.writes, [("w2", 3, GOAL, 1234), ("w1", 3, TORQUE, 1)])

    def test_unreadable_torque_is_reported_and_untouched(self):
        bus = FakeBus({}, {4: 100})
        self.assertEqual(hold_arm(bus, [4], TORQUE, POS, GOAL), ([], [4]))
        self.assertEqual(bus.writes, [])

    def test_unreadable_position_does_not_enable_torque(self):
        bus = FakeBus({5: 0}, {})
        self.assertEqual(hold_arm(bus, [5], TORQUE, POS, GOAL), ([], [5]))
        self.assertEqual(bus.writes, [])

    def test_mixed_arm(self):
        bus = FakeBus({1: 1, 2: 0, 3: None, 4: 0}, {2: 10, 4: 40})
        held, unread = hold_arm(bus, [1, 2, 3, 4], TORQUE, POS, GOAL)
        self.assertEqual((held, unread), ([2, 4], [3]))


if __name__ == "__main__":
    unittest.main()
