"""저장된 자세(pre_pick/grasp/grasp_closed 등)를 지금 로봇 팔의 현재 자세로 다시 찍는다.

teleoperate 로 팔을 원하는 자세로 만든 뒤 teleoperate 를 끄면(호스트는 마지막 자세를
그대로 유지한다) 이 스크립트로 그 자세를 읽어 poses/<name>.json 으로 저장한다.
원본 CLI(`lekiwi_save_pose.py`, roboseasy/lekiwi.git)와 같은 역할이지만, 이 레포에
이미 있는 `services.pickplace.poses.read_arm_pose/save_pose` 를 그대로 재사용한다.

사용 (teleoperate 끈 뒤):
    python webui/save_pose.py --name grasp
    python webui/save_pose.py --name grasp --robot.remote_ip=223.194.139.15
"""
from dataclasses import dataclass, field
from pathlib import Path
import sys

import draccus
from lerobot.robots.lekiwi import LeKiwiClient

_MOTION_DIR = Path(__file__).resolve().parent.parent
if str(_MOTION_DIR) not in sys.path:
    sys.path.insert(0, str(_MOTION_DIR))

from services.pickplace.poses import pose_summary, read_arm_pose, save_pose  # noqa: E402
from services.pickplace.robot_args import LeKiwiRobotArgs  # noqa: E402

_DEFAULT_POSES_DIR = Path("~/.PhysicalLabs/pickplace/lekiwi01/poses").expanduser()


@dataclass
class SavePoseConfig:
    robot: LeKiwiRobotArgs = field(
        default_factory=lambda: LeKiwiRobotArgs(remote_ip="223.194.139.15", id="lekiwi01")
    )
    name: str = "grasp"
    samples: int = 10
    interval_s: float = 0.1
    poses_dir: str = str(_DEFAULT_POSES_DIR)


@draccus.wrap()
def main(cfg: SavePoseConfig) -> None:
    robot = LeKiwiClient(cfg.robot.to_config())
    robot.connect()
    try:
        pose = read_arm_pose(robot, cfg.samples, cfg.interval_s)
    finally:
        robot.disconnect()

    path = Path(cfg.poses_dir).expanduser() / f"{cfg.name}.json"
    save_pose(path, cfg.name, cfg.robot.id, pose)
    print(f"저장됨: {path} (이전 값은 {path.stem}_prev.json 으로 백업됨)")
    print(pose_summary(pose))


if __name__ == "__main__":
    main()
