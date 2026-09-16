"""접근 완료 후 팔 자세 전환 상태기 — lekiwi_yolo_pick.py 의 `ArmSequencer` 이식.

원본: `/home/khw/workspace/lekiwi/yolo_and_pick/lekiwi_yolo_pick.py`
"""

import cv2
import numpy as np
from dataclasses import dataclass

from services.pickplace.approach import SIZE_REF_COLOR, STATE_COLORS, TARGET_COLOR
from services.pickplace.wrist_servo import GRIPPER_JOINT, GraspArgs, WristServo
from services.pickplace.yolo_detect import Detection


@dataclass
class PickArgs:
    """접근 완료(ALIGNED) 후 팔을 pick 직전 자세로 보내는 설정."""

    # false 면 팔은 시작 자세를 유지하고 접근/정렬만 한다
    enabled: bool = True
    # Step 2 자세 준비에서 저장한 자세 파일 (앱은 워커가 자세 dict 를 직접 주입한다)
    pose_file: str = ""
    # 현재 자세 → pick 자세로 보내는 데 걸리는 시간(초). 선형 보간으로 천천히 움직인다
    move_time_s: float = 2.0
    # pick 자세에 도달하면 스크립트를 끝낸다 (팔은 그 자세로 남는다). false 면 자세를 유지하며 계속 감시
    exit_when_ready: bool = False
    # pick 자세로 있는 동안 큐브가 정렬에서 이 프레임 수 이상 연속 벗어나면(또는 사라지면)
    # 팔을 시작 자세로 되돌린 뒤 다시 접근한다. 0 이면 되돌리지 않고 계속 유지
    drift_frames: int = 30


class ArmSequencer:
    """접근 완료 후 팔을 pick 자세로 보내고, 이어서 그리퍼를 열고 손목 뷰 서보로 큐브에 다가가는 상태기.

    상태:
        HOME(시작 자세 유지) → TO_PICK(보간) → PICK(pick 자세, 잠깐 대기)
            → OPEN_GRIPPER(그리퍼 최대 열기) → SERVO(손목 뷰 보며 뻗기, WristServo) → GRASP_READY(정지·유지)
        PICK 에서 큐브가 벗어나면 TO_HOME(보간 복귀) → HOME 으로 돌아가 다시 접근한다.
        --grasp.enabled=false 면 PICK 에서 멈춘다.
    팔이 HOME 이 아닐 때는 바퀴를 움직이면 안 된다 (`base_locked`).
    """

    def __init__(
        self,
        home: dict[str, float],
        pick: dict[str, float] | None,
        cfg: PickArgs,
        grasp_cfg: GraspArgs,
        grasp_pose: dict[str, float] | None = None,
    ):
        self.cfg = cfg
        self.grasp_cfg = grasp_cfg
        self.grasp_pose = dict(grasp_pose) if grasp_pose else None
        self.home = dict(home)
        self.pick = dict(pick) if pick else None
        self.state = "HOME"
        self.current = dict(home)  # 지금 보내고 있는 자세
        self._from: dict[str, float] = {}
        self._to: dict[str, float] = {}
        self._t0 = 0.0
        self._dur = 1.0
        self._pick_t = 0.0
        self.progress = 0.0
        self.drift = 0
        self.servo: WristServo | None = None
        self.just_ready = False  # 이번 프레임에 PICK 에 도달했는지
        self.grasp_just_ready = False  # 이번 프레임에 GRASP_READY 가 됐는지
        self.just_grasped = False  # 이번 프레임에 그리퍼가 다 닫혔는지 (GRASPED)
        self._ready_t = 0.0
        self.retries = 0  # 이번 pick 시도 안에서 GRIP FAIL 뒤 집기를 재시도한 횟수
        self.pick_attempts = 0  # pick 자세로 내려간 횟수 (첫 시도 포함)
        self._base_target_size = grasp_cfg.target_size_px  # 재시도로 키운 목표 크기를 새 pick 시도에서 되돌리기 위해
        self._after_open = "SERVO"  # OPEN_GRIPPER 가 끝나면 갈 곳: SERVO(집기) / HOME(포기하고 복귀)
        self._home_wait_t0 = 0.0
        self.gave_up = False  # pick 시도를 다 써서 포기했는지
        self.just_done = False  # 이번 프레임에 큐브를 문 채 시작 자세에 도착했는지

    @property
    def can_retry(self) -> bool:
        return self.state == "GRASPED" and self.retries < self.grasp_cfg.max_retries

    @property
    def can_restart_pick(self) -> bool:
        return self.state == "GRASPED" and self.pick_attempts < self.grasp_cfg.max_pick_attempts

    def retry(self, now: float) -> None:
        """GRIP FAIL → 그리퍼를 다시 벌리고(OPEN_GRIPPER), 끝나면 서보를 더 깊이 이어간다."""
        self.retries += 1
        self._after_open = "SERVO"
        target = dict(self.current)
        target[GRIPPER_JOINT] = self.grasp_cfg.gripper_open_pct
        self._start_move(target, "OPEN_GRIPPER", now, self.grasp_cfg.gripper_open_time_s)

    def restart_pick(self, now: float) -> None:
        """집기 재시도를 다 썼다 → 그리퍼를 벌리고 시작 자세로 돌아가 잠깐 기다린 뒤 접근부터 다시."""
        self._after_open = "HOME"
        target = dict(self.current)
        target[GRIPPER_JOINT] = self.grasp_cfg.gripper_open_pct
        self._start_move(target, "OPEN_GRIPPER", now, self.grasp_cfg.gripper_open_time_s)

    def give_up(self) -> None:
        """pick 시도까지 다 썼다 → 그 자리에서 멈춘다 (더는 움직이지 않음)."""
        self.gave_up = True

    def carry_home(self, now: float) -> None:
        """집기 성공 → 큐브를 문 채(그리퍼 그대로) 시작 자세로 천천히 돌아간다. 도착하면 DONE."""
        target = dict(self.home)
        target[GRIPPER_JOINT] = self.current.get(GRIPPER_JOINT, target.get(GRIPPER_JOINT, 0.0))  # 그리퍼는 닫힌 채
        self._start_move(target, "CARRY_HOME", now, self.grasp_cfg.carry_time_s)

    @property
    def done(self) -> bool:
        return self.state == "DONE"

    @property
    def enabled(self) -> bool:
        return self.cfg.enabled and self.pick is not None

    @property
    def grasp_enabled(self) -> bool:
        return self.enabled and self.grasp_cfg.enabled

    @property
    def base_locked(self) -> bool:
        return self.state != "HOME"

    def _start_move(self, target: dict[str, float], state: str, now: float, duration_s: float) -> None:
        self._from = dict(self.current)
        self._to = {k: target.get(k, self.current[k]) for k in self.current}
        self._t0 = now
        self._dur = max(duration_s, 1e-3)
        self.progress = 0.0
        self.state = state

    def _interpolate(self, now: float) -> float:
        a = min(1.0, (now - self._t0) / self._dur)
        self.progress = a
        self.current = {k: self._from[k] + (self._to[k] - self._from[k]) * a for k in self.current}
        return a

    def update(
        self,
        aligned: bool,
        tracking_ok: bool,
        allow_motion: bool,
        now: float,
        wrist_dets: list[Detection] | None = None,
        wrist_shape: tuple[int, ...] | None = None,
        dt: float = 1 / 30,
        descend_hint: bool = False,
    ) -> dict[str, float]:
        """이번 프레임에 보낼 팔 목표 자세를 돌려준다.

        aligned      : Approacher 가 ALIGNED 인지
        tracking_ok  : 큐브가 여전히 정렬 허용치 안에 있는지 (size_ok and center_ok)
        allow_motion : 일시정지/dry-run 이 아닌지
        wrist_dets   : 손목 뷰 검출 (SERVO 단계에서 사용)
        """
        self.just_ready = False
        self.grasp_just_ready = False
        self.just_grasped = False
        self.just_done = False
        if not self.enabled:
            return self.current

        if self.gave_up or self.state == "DONE":
            return self.current

        if self.state == "CARRY_HOME":
            if self._interpolate(now) >= 1.0:
                self.state = "DONE"
                self.just_done = True
            return self.current

        if self.state == "HOME":
            if aligned and allow_motion:
                # 새 pick 시도: 집기 재시도 카운터와 키워 둔 목표 크기를 원래대로
                self.pick_attempts += 1
                self.retries = 0
                self.servo = None
                self.grasp_cfg.target_size_px = self._base_target_size
                self._start_move(self.pick, "TO_PICK", now, self.cfg.move_time_s)
        elif self.state in ("TO_PICK", "TO_HOME"):
            if self._interpolate(now) >= 1.0:
                if self.state == "TO_PICK":
                    self.state = "PICK"
                    self.drift = 0
                    self.just_ready = True
                    self._pick_t = now
                elif self._after_open == "HOME":
                    # 포기 후 복귀: 잠깐 기다렸다가 다시 접근한다
                    self._after_open = "SERVO"
                    self._home_wait_t0 = now
                    self.state = "HOME_WAIT"
                else:
                    self.state = "HOME"
        elif self.state == "HOME_WAIT":
            if now - self._home_wait_t0 >= self.grasp_cfg.pick_retry_wait_s:
                self.state = "HOME"
        elif self.state == "PICK":
            if self.grasp_enabled and allow_motion and now - self._pick_t >= self.grasp_cfg.pick_dwell_s:
                # 다음 단계: 그리퍼를 최대로 연다 (정규화 100 = 캘리브레이션 range_max)
                target = dict(self.current)
                target[GRIPPER_JOINT] = self.grasp_cfg.gripper_open_pct
                self._start_move(target, "OPEN_GRIPPER", now, self.grasp_cfg.gripper_open_time_s)
            elif self.cfg.drift_frames > 0:
                self.drift = 0 if tracking_ok else self.drift + 1
                if self.drift >= self.cfg.drift_frames and allow_motion:
                    self._start_move(self.home, "TO_HOME", now, self.cfg.move_time_s)
        elif self.state == "OPEN_GRIPPER":
            if self._interpolate(now) >= 1.0:
                if self._after_open == "HOME":
                    self._start_move(self.home, "TO_HOME", now, self.cfg.move_time_s)
                elif self.servo is None:
                    self.servo = WristServo(self.grasp_cfg, self.current, self.grasp_pose)
                    self.state = "SERVO"
                else:
                    self.servo.resume()  # 재시도: 같은 자리에서 목표를 키워 이어간다
                    self.state = "SERVO"
        elif self.state == "SERVO":
            if wrist_dets is not None and wrist_shape is not None:
                self.current = self.servo.update(wrist_dets, wrist_shape, dt, now, allow_motion, descend_hint)
                if self.servo.just_ready:
                    self.state = "GRASP_READY"
                    self.grasp_just_ready = True
                    self._ready_t = now
        elif self.state == "GRASP_READY":
            # 집기: 잠깐 멈춘 뒤 그리퍼를 측정해 둔 값까지 닫는다
            g = self.grasp_cfg
            if g.close_after_ready and allow_motion and now - self._ready_t >= g.grasp_dwell_s:
                target = dict(self.current)
                target[GRIPPER_JOINT] = g.gripper_close_pct if g.gripper_close_pct is not None else g.resolve_close_pct()
                self._start_move(target, "CLOSE_GRIPPER", now, g.gripper_close_time_s)
        elif self.state == "CLOSE_GRIPPER":
            if self._interpolate(now) >= 1.0:
                self.state = "GRASPED"
                self.just_grasped = True
        # GRASPED: 그대로 유지 (큐브를 물고 있음)
        return self.current

    @property
    def label(self) -> str | None:
        if self.gave_up:
            return "GIVE_UP"
        if self.state == "SERVO" and self.servo is not None:
            return {
                "CENTERING": "WRIST_CENTERING",
                "APPROACHING": "WRIST_APPROACH",
                "LOST": "WRIST_LOST",
                "REFINING": "WRIST_REFINE",
                "READY": "GRASP_READY",
            }.get(self.servo.state)
        return {
            "TO_PICK": "ARM_TO_PICK",
            "PICK": "PICK_READY",
            "TO_HOME": "ARM_TO_HOME",
            "OPEN_GRIPPER": "GRIPPER_OPEN",
            "GRASP_READY": "GRASP_READY",
            "CLOSE_GRIPPER": "GRIPPER_CLOSE",
            "GRASPED": "GRASPED",
            "HOME_WAIT": "HOME_WAIT",
            "CARRY_HOME": "CARRY_HOME",
            "DONE": "DONE",
        }.get(self.state)


def draw_wrist_servo(frame_bgr: np.ndarray, arm: ArmSequencer, state_label: str) -> np.ndarray:
    """손목 뷰 오버레이: 화면 중앙점, 목표 박스, 목표 크기 참조, 서보 상태."""
    canvas = frame_bgr
    h, w = canvas.shape[:2]
    cx0, cy0 = w // 2, h // 2
    servo = arm.servo
    active = arm.state in ("SERVO", "GRASP_READY") and servo is not None

    # 화면 중앙점: 서보 중엔 박스 안/밖에 따라 초록/빨강, 아니면 흰색
    if active and servo.target is not None:
        color = (0, 220, 0) if servo.inside else (0, 0, 255)
    else:
        color = (255, 255, 255)
    cv2.circle(canvas, (cx0, cy0), 7, color, 2, cv2.LINE_AA)
    cv2.circle(canvas, (cx0, cy0), 2, color, -1, cv2.LINE_AA)

    if active and servo.target is not None:
        x1, y1, x2, y2 = servo.target.xyxy
        bx, by = servo.target.center
        cv2.rectangle(canvas, (x1, y1), (x2, y2), TARGET_COLOR, 3)
        # 세로선에 맞추는 기준 변(왼쪽/오른쪽) 또는 중심선을 두껍게 표시하고, 맞지 않으면 화살표
        ax, ay = servo.anchor_x, servo.anchor_y
        edge_color = (0, 220, 0) if servo.x_ok else TARGET_COLOR
        cv2.line(canvas, (ax, y1 - 8), (ax, y2 + 8), edge_color, 4)
        if servo.cfg.y_anchor != "inside":
            # 세로 기준(위 변/중심)도 두껍게
            cv2.line(canvas, (x1 - 8, ay), (x2 + 8, ay), (0, 220, 0) if servo.y_ok else TARGET_COLOR, 4)
        # 참조 목표점(노란 십자): 기준점이 여기 와야 한다
        tx, ty = cx0 + servo.cfg.x_target_dx, cy0 + (servo.cfg.y_target_dy if servo.cfg.y_anchor != "inside" else 0)
        cv2.drawMarker(canvas, (tx, ty), SIZE_REF_COLOR, cv2.MARKER_CROSS, 24, 2, cv2.LINE_AA)
        if not (servo.x_ok and servo.y_ok):
            cv2.arrowedLine(canvas, (tx, ty), (ax, ay if servo.cfg.y_anchor != "inside" else by), TARGET_COLOR, 2, tipLength=0.2)
        # 목표 크기 참조(노랑): 박스 중심에 target_size 정사각형
        ts = servo.cfg.target_size_px
        cv2.rectangle(canvas, (bx - ts // 2, by - ts // 2), (bx + ts // 2, by + ts // 2), SIZE_REF_COLOR, 1, cv2.LINE_AA)
        info = (
            f"{servo.cfg.size_metric}={servo.size}/{ts}px  dx={servo.dx:+d} dy={servo.dy:+d}  "
            f"{servo.cfg.x_anchor}-x {'FRZ' if servo.pan_frozen else ('OK' if servo.x_ok else '..')} "
            f"y {'OK' if servo.y_ok else '..'}"
        )
        # 검출 라벨(박스 위)과 겹치지 않게 박스 아래에, 화면 아래로 나가면 박스 위 라벨보다 더 위에
        ty = y2 + 22 if y2 + 22 < h - 30 else max(y1 - 30, 40)
        cv2.putText(canvas, info, (x1, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.55, TARGET_COLOR, 2, cv2.LINE_AA)

    if arm.state in ("OPEN_GRIPPER", "SERVO", "GRASP_READY", "CLOSE_GRIPPER", "GRASPED", "CARRY_HOME", "DONE"):
        col = STATE_COLORS.get(state_label, (255, 255, 255))
        if arm.state in ("OPEN_GRIPPER", "CLOSE_GRIPPER", "CARRY_HOME"):
            txt = f"{state_label}  {arm.progress * 100:3.0f}%  gripper {arm.current.get(GRIPPER_JOINT, 0):.0f}%"
        elif arm.state in ("GRASPED", "DONE"):
            txt = f"{state_label}  gripper {arm.current.get(GRIPPER_JOINT, 0):.1f}%"
        else:
            txt = (
                f"{state_label}  reach {servo.progress_pct:3.0f}%  "
                f"pan{servo.pan_delta:+.1f} tilt{servo.tilt_delta:+.1f} deg"
            )
        cv2.rectangle(canvas, (0, h - 26), (w, h), (0, 0, 0), -1)
        cv2.putText(canvas, txt, (6, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2, cv2.LINE_AA)
    return canvas
