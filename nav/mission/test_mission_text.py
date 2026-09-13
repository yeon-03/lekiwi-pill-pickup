#!/usr/bin/env python3
"""mission_text 검증 (ROS 불필요).  python3 nav/mission/test_mission_text.py"""
import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from mission_text import action_text, effective_state, mission_state_json  # noqa: E402


class TestActionText(unittest.TestCase):
    def test_phrases_with_color(self):
        self.assertEqual(action_text("moving", "red"), "빨간색 약을 향해 가는 중")
        self.assertEqual(action_text("picking", "red"), "빨간색 약을 집는 중")
        self.assertEqual(action_text("returning", "green"), "초록색 약을 가지고 돌아오는 중")
        self.assertEqual(action_text("done", "blue"), "파란색 약을 가져왔어요")
        self.assertEqual(action_text("sent", "red"), "빨간색 약을 가져오라고 르키위에게 전달했어요")

    def test_without_color(self):
        self.assertEqual(action_text("moving"), "약을 향해 가는 중")
        self.assertEqual(action_text("picking", None), "약을 집는 중")

    def test_color_independent_states(self):
        self.assertEqual(action_text("canceled", "red"), "르키위가 멈췄어요")
        self.assertEqual(action_text("idle"), "르키위 대기 중")

    def test_unknown_state_falls_back_to_status(self):
        self.assertEqual(action_text("weird", "red", "제 위치를 다시 확인할게요."), "제 위치를 다시 확인할게요.")
        self.assertEqual(action_text("weird"), "weird")


class TestEffectiveState(unittest.TestCase):
    def test_moving_after_pick_is_returning(self):
        self.assertEqual(effective_state("picking", "moving"), "returning")
        self.assertEqual(effective_state("returning", "moving"), "returning")

    def test_moving_before_pick_stays_moving(self):
        self.assertEqual(effective_state("sent", "moving"), "moving")
        self.assertEqual(effective_state(None, "moving"), "moving")
        self.assertEqual(effective_state("moving", "moving"), "moving")

    def test_other_states_unchanged(self):
        self.assertEqual(effective_state("picking", "done"), "done")
        self.assertEqual(effective_state("returning", "failed"), "failed")


class TestMissionStateJson(unittest.TestCase):
    def test_fields(self):
        d = json.loads(mission_state_json("moving", "가는 중이에요. 1.2 m 남았어요.", "red", "center",
                                          dry_run=False, now=100.0))
        self.assertEqual(d["state"], "moving")
        self.assertEqual(d["action_text"], "빨간색 약을 향해 가는 중")
        self.assertEqual(d["status"], "가는 중이에요. 1.2 m 남았어요.")
        self.assertEqual((d["color"], d["target"], d["dry_run"], d["ts"]), ("red", "center", False, 100.0))

    def test_received_topic_and_command(self):
        d = json.loads(mission_state_json(
            "sent", color="red", target="center", dry_run=True, now=5.0,
            received={"topic": "/pickup/medicine/red", "data": "red", "at": 4.9},
            command="fetch center color:red"))
        self.assertEqual((d["received_topic"], d["received_data"], d["received_at"]),
                         ("/pickup/medicine/red", "red", 4.9))
        self.assertEqual(d["command"], "fetch center color:red")

    def test_received_defaults_empty(self):
        d = json.loads(mission_state_json("moving", color="red"))
        self.assertIsNone(d["received_topic"])
        self.assertEqual(d["command"], "")

    def test_korean_not_escaped(self):
        self.assertIn("빨간색", mission_state_json("picking", color="red"))


if __name__ == "__main__":
    unittest.main()
