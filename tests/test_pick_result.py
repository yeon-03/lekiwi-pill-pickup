from pathlib import Path

from lekiwi_pill_pickup.pick_result import read_result_file, write_result_file

VERDICT = {"skill": "pick_pill_bottle", "color": "green", "success": True,
           "grasped": True, "gripper_grasped": True, "elapsed_sec": 94.2,
           "verdict_reason": "원래 자리가 비어있음"}


def test_write_then_read_roundtrips(tmp_path: Path) -> None:
    p = tmp_path / "lekiwi_result_pick_pill_bottle.json"
    write_result_file(p, VERDICT)
    result = read_result_file(p)
    assert result == VERDICT
    # 한글이 정확히 보존되었는지 명시적으로 확인 (다중 런타임 인코딩 검증)
    assert result["verdict_reason"] == "원래 자리가 비어있음"


def test_write_is_atomic_no_partial_file_left(tmp_path: Path) -> None:
    p = tmp_path / "r.json"
    write_result_file(p, VERDICT)
    # 임시파일이 남지 않아야 한다
    leftovers = [x.name for x in tmp_path.iterdir() if x.name != "r.json"]
    assert leftovers == []


def test_read_missing_file_returns_none(tmp_path: Path) -> None:
    assert read_result_file(tmp_path / "nope.json") is None


def test_read_corrupt_json_returns_none(tmp_path: Path) -> None:
    p = tmp_path / "bad.json"
    p.write_text("{not json", encoding='utf-8')
    assert read_result_file(p) is None


def test_read_empty_file_returns_none(tmp_path: Path) -> None:
    p = tmp_path / "empty.json"
    p.write_text("", encoding='utf-8')
    assert read_result_file(p) is None


def test_read_non_dict_json_returns_none(tmp_path: Path) -> None:
    """비-dict JSON(배열)은 None을 반환해야 한다"""
    p = tmp_path / "array.json"
    p.write_text("[]", encoding='utf-8')
    assert read_result_file(p) is None


def test_write_result_file_accepts_str_path(tmp_path: Path) -> None:
    """write_result_file이 str 경로를 허용해야 한다"""
    p = str(tmp_path / "str_path.json")
    write_result_file(p, VERDICT)
    result = read_result_file(p)
    assert result == VERDICT


def test_double_write_overwrites(tmp_path: Path) -> None:
    """같은 경로에 두 번 쓸 때 두 번째 값이 덮어써진다"""
    p = tmp_path / "overwrite.json"
    first_verdict = {"skill": "pick_pill_bottle", "success": True}
    second_verdict = {"skill": "pick_pill_bottle", "success": False}
    write_result_file(p, first_verdict)
    write_result_file(p, second_verdict)
    result = read_result_file(p)
    assert result == second_verdict
