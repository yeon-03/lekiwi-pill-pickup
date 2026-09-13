"""Pick&Place 시연용 웹 서버 진입점.

사용: python webui/run_pickplace_ui.py --robot.remote_ip=10.42.0.141 --robot.id=lekiwi01 ...
인자 스타일은 원본 CLI(lekiwi_yolo_pick.py) 와 동일하다 (중첩 dataclass,
`--그룹.필드=값`, `lerobot.configs.parser` 사용).

SIGINT/SIGTERM 을 별도로 잡지 않는다 — uvicorn 이 그 신호들을 받아 서버를 정리하고
`uvicorn.run()` 이 정상적으로 리턴하면, 아래 `finally` 가 항상 `worker.request_stop()`
을 부르므로 결과적으로 같은 정지 경로를 탄다. (단, kill -9 처럼 프로세스를 강제
종료하는 신호는 어떤 파이썬 코드로도 막을 수 없다 — 그건 물리적 비상정지로 대응한다.)
"""
import sys
from dataclasses import dataclass, field
from pathlib import Path

import uvicorn
from lerobot.configs import parser
from lerobot.robots.lekiwi import LeKiwiClient
from lerobot.utils.utils import init_logging

_MOTION_DIR = Path(__file__).resolve().parent.parent
if str(_MOTION_DIR) not in sys.path:
    sys.path.insert(0, str(_MOTION_DIR))

from services.pickplace.approach import ApproachArgs  # noqa: E402
from services.pickplace.arm_sequencer import PickArgs  # noqa: E402
from services.pickplace.config import PickPlaceConfig  # noqa: E402
from services.pickplace.grasp_check import GraspCheckArgs  # noqa: E402
from services.pickplace.headless_worker import PickPlaceHeadlessWorker  # noqa: E402
from services.pickplace.poses import load_pose  # noqa: E402
from services.pickplace.robot_args import LeKiwiRobotArgs  # noqa: E402
from services.pickplace.wrist_servo import GraspArgs  # noqa: E402
from services.pickplace.yolo_detect import YoloArgs  # noqa: E402

from webui.app import create_app  # noqa: E402

_DEFAULT_POSES_DIR = Path("~/.PhysicalLabs/pickplace/lekiwi01/poses").expanduser()
_DEFAULT_MODEL = "/home/roboseasy/YOLO/outputs/runs/green_pill/weights/best.pt"


@dataclass
class RunConfig:
    robot: LeKiwiRobotArgs = field(
        default_factory=lambda: LeKiwiRobotArgs(remote_ip="10.42.0.141", id="lekiwi01")
    )
    yolo: YoloArgs = field(default_factory=lambda: YoloArgs(path=_DEFAULT_MODEL))
    approach: ApproachArgs = field(default_factory=ApproachArgs)
    pick: PickArgs = field(
        default_factory=lambda: PickArgs(pose_file=str(_DEFAULT_POSES_DIR / "pre_pick.json"))
    )
    grasp: GraspArgs = field(
        default_factory=lambda: GraspArgs(
            grasp_pose_file=str(_DEFAULT_POSES_DIR / "grasp.json"),
            close_pose_file=str(_DEFAULT_POSES_DIR / "grasp_closed.json"),
        )
    )
    check: GraspCheckArgs = field(default_factory=GraspCheckArgs)
    views: list[str] = field(default_factory=lambda: ["front", "wrist"])
    fps: int = 30
    dry_run: bool = False
    crosshair_views: list[str] = field(default_factory=lambda: ["front", "wrist"])
    host: str = "0.0.0.0"
    port: int = 8000
    rollout_time_s: float = 2.0

    def to_pickplace_config(self) -> PickPlaceConfig:
        return PickPlaceConfig(
            yolo=self.yolo,
            approach=self.approach,
            pick=self.pick,
            grasp=self.grasp,
            check=self.check,
            views=list(self.views),
            fps=self.fps,
            dry_run=self.dry_run,
            crosshair_views=list(self.crosshair_views),
        )


def _load_poses(cfg: RunConfig) -> dict[str, dict[str, float]]:
    poses: dict[str, dict[str, float]] = {}
    for name, path in (
        ("pre_pick", cfg.pick.pose_file),
        ("grasp", cfg.grasp.grasp_pose_file),
        ("grasp_closed", cfg.grasp.close_pose_file),
    ):
        if path:
            poses[name] = load_pose(Path(path).expanduser())
    return poses


@parser.wrap()
def main(cfg: RunConfig) -> None:
    init_logging()
    pp_cfg = cfg.to_pickplace_config()
    pp_cfg.validate()
    poses = _load_poses(cfg)

    robot = LeKiwiClient(cfg.robot.to_config())
    worker = PickPlaceHeadlessWorker(robot, pp_cfg, poses, rollout_time_s=cfg.rollout_time_s)
    worker.start_background()

    app = create_app(worker)
    try:
        # timeout_graceful_shutdown 을 짧게 주지 않으면, 브라우저가 /stream/* 을 계속
        # 물고 있는 동안(무한 MJPEG) uvicorn 이 그 연결이 끊기길 무한정 기다려서
        # Ctrl+C/SIGTERM 이 먹통이 된다 (2026-09-13 실사용 중 발견 — 매번 강제 종료해야 했음).
        uvicorn.run(app, host=cfg.host, port=cfg.port, timeout_graceful_shutdown=3)
    finally:
        worker.request_stop()
        worker.join(timeout=30.0)


if __name__ == "__main__":
    main()
