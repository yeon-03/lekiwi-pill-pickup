#!/usr/bin/env python3
"""mission_state_json 의 picked(집기 결과) 필드 -- 에이보 진행 안내가 이 값을 쓴다."""
import json
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mission_text import mission_state_json  # noqa: E402


class TestPicked(unittest.TestCase):
    def test_default_is_unknown(self):
        self.assertIsNone(json.loads(mission_state_json("moving", color="green"))["picked"])

    def test_true_false(self):
        self.assertIs(json.loads(mission_state_json("returning", picked=True))["picked"], True)
        self.assertIs(json.loads(mission_state_json("returning", picked=False))["picked"], False)

    def test_truthy_values_become_bool(self):
        self.assertIs(json.loads(mission_state_json("returning", picked=1))["picked"], True)


if __name__ == "__main__":
    unittest.main()
