"""집기 사이클의 판정 결과를 파일로 주고받는다 — pick_cycle.py(쓰기)와
robot_ws의 lekiwi_command_node(읽기) 사이의 계약. ROS 의존 없음.

경로 규약: /tmp/lekiwi_result_{skill}.json (스킬당 하나, 다음 실행이 덮어씀).
스키마: {skill, color, success(bool), grasped, gripper_grasped, elapsed_sec, verdict_reason}
"""
import json
import os
from pathlib import Path


def write_result_file(path, verdict: dict) -> None:
    path = Path(path)
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(verdict, ensure_ascii=False, indent=2))
    os.replace(tmp, path)


def read_result_file(path):
    try:
        raw = Path(path).read_text()
    except OSError:
        return None
    raw = raw.strip()
    if not raw:
        return None
    try:
        data = json.loads(raw)
    except ValueError:
        return None
    return data if isinstance(data, dict) else None
