"""Pick&Place 워커 전체 실행 설정 — 로봇 무관 (dataclass 묶음).

원본 `lekiwi_yolo_pick.py::LeKiwiPickConfig` 는 lerobot 로봇 설정(`LeKiwiRobotArgs`)을 끼고
있어 그대로 가져오지 않는다. 이 `PickPlaceConfig` 는 `LeKiwiPickConfig.validate()` 의 검증
로직만 옮긴 것이다. `display`/`print_status`/`run_time_s`/`start_paused` 필드는 GUI(워크스페이스
Step 3)가 대신하므로 제외했고, 자세 파일 존재 검사는 하지 않는다 — 워커가 자세 dict 를
직접 주입하므로 검증 시점에는 아직 파일이 없을 수 있다.
"""

from dataclasses import dataclass, field

from services.pickplace import PickPlaceError
from services.pickplace.approach import ApproachArgs
from services.pickplace.arm_sequencer import PickArgs
from services.pickplace.grasp_check import GraspCheckArgs
from services.pickplace.wrist_servo import GraspArgs
from services.pickplace.yolo_detect import YoloArgs


@dataclass
class PickPlaceConfig:
    """Pick&Place 워커 하나를 돌리는 데 필요한 설정 전체."""

    yolo: YoloArgs = field(default_factory=YoloArgs)
    approach: ApproachArgs = field(default_factory=ApproachArgs)
    pick: PickArgs = field(default_factory=PickArgs)
    grasp: GraspArgs = field(default_factory=GraspArgs)
    check: GraspCheckArgs = field(default_factory=GraspCheckArgs)

    # 추론/표시할 카메라. approach.view 는 자동으로 포함된다.
    views: list[str] = field(default_factory=lambda: ["front", "wrist"])
    # 제어 루프 주기
    fps: int = 30
    # true 면 계산만 하고 바퀴/팔 명령은 정지만 보낸다
    dry_run: bool = False
    # 가상의 중앙 가로선/세로선(초록)을 그릴 뷰
    crosshair_views: list[str] = field(default_factory=lambda: ["front", "wrist"])

    def validate(self) -> None:
        a = self.approach
        if self.fps <= 0:
            raise PickPlaceError(f"error: fps 는 1 이상이어야 합니다 (받은 값: {self.fps})")
        if a.size_metric not in ("width", "height", "max"):
            raise PickPlaceError(
                f"error: approach.size_metric 은 width/height/max 중 하나여야 합니다 (받은 값: {a.size_metric})"
            )
        if not 0.0 < a.line_ratio < 1.0:
            raise PickPlaceError(f"error: approach.line_ratio 는 0~1 사이여야 합니다 (받은 값: {a.line_ratio})")
        if a.target_size_px <= 0:
            raise PickPlaceError(f"error: approach.target_size_px 는 1 이상이어야 합니다 (받은 값: {a.target_size_px})")
        if a.size_tolerance_px < 0 or a.center_tolerance_px < 0 or a.center_hysteresis_px < 0:
            raise PickPlaceError(
                "error: approach.size_tolerance_px / center_tolerance_px / center_hysteresis_px 는 0 이상이어야 합니다"
            )
        if a.clip_aspect_min <= 0:
            raise PickPlaceError(f"error: approach.clip_aspect_min 은 0 보다 커야 합니다 (받은 값: {a.clip_aspect_min})")
        if a.bottom_margin_px < 0:
            raise PickPlaceError(f"error: approach.bottom_margin_px 는 0 이상이어야 합니다 (받은 값: {a.bottom_margin_px})")
        if a.lateral_mode not in ("rotate", "strafe"):
            raise PickPlaceError(
                f"error: approach.lateral_mode 는 rotate/strafe 중 하나여야 합니다 (받은 값: {a.lateral_mode})"
            )
        if a.max_theta_speed <= 0 or a.min_theta_speed < 0 or a.min_theta_speed > a.max_theta_speed:
            raise PickPlaceError(
                "error: 0 <= approach.min_theta_speed <= approach.max_theta_speed, max_theta_speed > 0 이어야 합니다 "
                f"(받은 값: min={a.min_theta_speed}, max={a.max_theta_speed})"
            )
        if a.max_speed <= 0 or a.min_speed < 0 or a.min_speed > a.max_speed:
            raise PickPlaceError(
                "error: 0 <= approach.min_speed <= approach.max_speed, approach.max_speed > 0 이어야 합니다 "
                f"(받은 값: min={a.min_speed}, max={a.max_speed})"
            )
        if a.view not in self.views:
            self.views = [a.view, *self.views]
        if self.grasp.enabled and self.pick.enabled:
            self.grasp.validate()
            if self.grasp.view not in self.views:
                self.views = [*self.views, self.grasp.view]
            if self.check.enabled:
                self.check.validate()
                for v in (self.check.front_view, self.check.wrist_view):
                    if v not in self.views:
                        self.views = [*self.views, v]
        if self.pick.enabled:
            if self.pick.move_time_s <= 0:
                raise PickPlaceError(f"error: pick.move_time_s 는 0 보다 커야 합니다 (받은 값: {self.pick.move_time_s})")
            if self.pick.drift_frames < 0:
                raise PickPlaceError(f"error: pick.drift_frames 는 0 이상이어야 합니다 (받은 값: {self.pick.drift_frames})")
