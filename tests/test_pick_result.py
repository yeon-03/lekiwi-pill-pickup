import json
from pathlib import Path

import pytest

from lekiwi_pill_pickup.pick_result import read_result_file, write_result_file

VERDICT = {"skill": "pick_pill_bottle", "color": "green", "success": True,
           "grasped": True, "gripper_grasped": True, "elapsed_sec": 94.2,
           "verdict_reason": "원래 자리가 비어있음"}


def test_write_then_read_roundtrips(tmp_path):
    p = tmp_path / "lekiwi_result_pick_pill_bottle.json"
    write_result_file(p, VERDICT)
    assert read_result_file(p) == VERDICT


def test_write_is_atomic_no_partial_file_left(tmp_path):
    p = tmp_path / "r.json"
    write_result_file(p, VERDICT)
    # 임시파일이 남지 않아야 한다
    leftovers = [x.name for x in tmp_path.iterdir() if x.name != "r.json"]
    assert leftovers == []


def test_read_missing_file_returns_none(tmp_path):
    assert read_result_file(tmp_path / "nope.json") is None


def test_read_corrupt_json_returns_none(tmp_path):
    p = tmp_path / "bad.json"
    p.write_text("{not json")
    assert read_result_file(p) is None


def test_read_empty_file_returns_none(tmp_path):
    p = tmp_path / "empty.json"
    p.write_text("")
    assert read_result_file(p) is None
