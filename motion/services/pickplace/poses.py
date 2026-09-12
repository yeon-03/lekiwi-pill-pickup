"""자세 파일 읽기/쓰기 — lekiwi_save_pose.py 이식 + 앱용 저장 함수 추가.

원본: `/home/khw/workspace/lekiwi/yolo_and_pick/lekiwi_save_pose.py`
스키마는 원본과 동일하게 유지한다: `{"name", "saved_at", "robot_id", "pose"}`.
"""

import json
import shutil
import time
from pathlib import Path

import numpy as np

from services.pickplace import PickPlaceError


def load_pose(path: Path) -> dict[str, float]:
    """저장된 자세 파일을 읽는다 ({"arm_shoulder_pan.pos": ..., ...} 형식)."""
    data = json.loads(Path(path).read_text())
    pose = data["pose"] if "pose" in data else data
    if not pose or not all(isinstance(v, (int, float)) for v in pose.values()):
        raise ValueError(f"{path} 는 자세 파일이 아닙니다 (관절→값 dict 또는 {{'pose': {{...}}}} 형식이어야 함)")
    return {k: float(v) for k, v in pose.items()}


def read_arm_pose(robot, samples: int, interval_s: float) -> dict[str, float]:
    """관절 .pos 값을 여러 번 읽어 평균낸다. `robot` 은 `get_observation()` 만 있으면 된다 (덕타입)."""
    acc: dict[str, list[float]] = {}
    for _ in range(max(1, samples)):
        obs = robot.get_observation()
        for k, v in obs.items():
            if k.endswith(".pos"):
                acc.setdefault(k, []).append(float(v))
        time.sleep(interval_s)
    if not acc:
        raise PickPlaceError("error: 팔 관절 위치(.pos)를 하나도 받지 못했습니다. 호스트가 팔에 연결돼 있는지 확인하세요.")
    return {k: float(np.mean(v)) for k, v in acc.items()}


def save_pose(path: Path, name: str, robot_id: str, pose: dict[str, float], *, backup: bool = True) -> Path:
    """자세를 원본과 같은 스키마로 저장한다.

    부모 디렉토리를 만들고, `backup=True` 이고 파일이 이미 있으면 먼저 `<stem>_prev.json` 으로
    복사해 둔다 (원본 `*_prev.json` 관례 유지).
    """
    p = Path(path).expanduser()
    p.parent.mkdir(parents=True, exist_ok=True)
    if backup and p.exists():
        prev = p.with_name(f"{p.stem}_prev.json")
        shutil.copy2(p, prev)
    p.write_text(
        json.dumps(
            {"name": name, "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"), "robot_id": robot_id, "pose": pose},
            indent=2,
            ensure_ascii=False,
        )
        + "\n"
    )
    return p


# README §7 표에 쓰인 라벨 (arm_ 접두사·.pos 접미사를 떼고 shoulder_pan→pan, shoulder_lift→lift,
# elbow_flex→elbow, wrist_roll→roll 처럼 짧게 줄인 이름). wrist_flex 는 원본 표기 그대로 유지한다.
_JOINT_LABELS = {
    "arm_shoulder_pan.pos": "pan",
    "arm_shoulder_lift.pos": "lift",
    "arm_elbow_flex.pos": "elbow",
    "arm_wrist_flex.pos": "wrist_flex",
    "arm_wrist_roll.pos": "roll",
    "arm_gripper.pos": "gripper",
}


def pose_summary(pose: dict[str, float]) -> str:
    """자세를 한 줄로 요약한다 (예: "pan -1.9, lift 21.6, elbow 5.2, ..."). README §7 표 형식."""
    parts = []
    for key, value in pose.items():
        label = _JOINT_LABELS.get(key)
        if label is None:
            label = key.removeprefix("arm_").removesuffix(".pos")
        parts.append(f"{label} {value:.1f}")
    return ", ".join(parts)
