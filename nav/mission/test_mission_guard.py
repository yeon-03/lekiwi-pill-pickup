#!/usr/bin/env python3
"""abo_nav_bridge.fetch_conflict 검증 (로봇·Nav2 불필요, ROS 파이썬으로 import 만 한다).

  source /opt/ros/jazzy/setup.bash && python3 nav/mission/test_mission_guard.py
"""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from abo_nav_bridge import fetch_conflict  # noqa: E402


def mission(target="center", color="red", phase="going"):
    return {"target": target, "color": color, "phase": phase, "return_to": "출발 자리"}


class TestFetchConflict(unittest.TestCase):
    def test_idle_accepts(self):
        self.assertIsNone(fetch_conflict(None, "center", "red"))

    def test_same_command_is_duplicate_in_every_phase(self):
        for phase in ("going", "picking", "returning"):
            self.assertEqual(fetch_conflict(mission(phase=phase), "center", "red"), "duplicate", phase)

    def test_other_color_is_busy(self):
        self.assertEqual(fetch_conflict(mission(), "center", "green"), "busy")

    def test_other_target_is_busy(self):
        self.assertEqual(fetch_conflict(mission(), "upper", "red"), "busy")

    def test_colorless_mission(self):
        self.assertEqual(fetch_conflict(mission(color=None), "center", None), "duplicate")
        self.assertEqual(fetch_conflict(mission(color=None), "center", "red"), "busy")


if __name__ == "__main__":
    unittest.main()
