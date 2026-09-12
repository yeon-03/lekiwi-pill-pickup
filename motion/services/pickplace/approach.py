"""접근(전진) + 좌우 정렬 제어 — lekiwi_yolo_pick.py 의 접근 단계 이식.

원본: `/home/khw/workspace/lekiwi/yolo_and_pick/lekiwi_yolo_pick.py`
`Approacher` 는 카메라 검출로부터 베이스 속도 명령(x/y/theta.vel)을 계산하는 P 제어기 +
상태 추적기다. 게인·판별 순서는 원본과 동일하게 유지한다.

원본의 `LeKiwiPickConfig` 는 lerobot 로봇 설정을 끼고 있어 가져오지 않는다 — 로봇 무관
설정 묶음은 `services/pickplace/config.py` 의 `PickPlaceConfig` 를 쓴다.
"""

from dataclasses import dataclass

import cv2
import numpy as np

from services.pickplace.yolo_detect import Detection

LINE_COLOR = (0, 255, 0)  # 가운데 세로선 (초록, BGR) — yolo_detect 의 십자선과 같은 색
BAND_COLOR = (0, 180, 0)  # 좌우 허용 범위
TARGET_COLOR = (255, 0, 255)  # 목표(가장 큰) 박스 강조
SIZE_REF_COLOR = (0, 255, 255)  # 목표 크기 참조 박스 (노랑)
IGNORED_COLOR = (140, 140, 140)  # 가로선 위라서 무시한 검출 (회색)
STATE_COLORS = {
    "SEARCHING": (200, 200, 200),
    "ABOVE_LINE": (200, 200, 200),
    "ROTATING": (255, 200, 0),
    "TOO_CLOSE": (0, 0, 255),
    "ARM_TO_PICK": (255, 255, 0),
    "PICK_READY": (0, 255, 0),
    "ARM_TO_HOME": (255, 255, 0),
    "GRIPPER_OPEN": (255, 255, 0),
    "WRIST_CENTERING": (0, 165, 255),
    "WRIST_APPROACH": (255, 200, 0),
    "WRIST_LOST": (0, 0, 255),
    "WRIST_REFINE": (0, 200, 255),
    "GRASP_READY": (0, 255, 0),
    "GRIPPER_CLOSE": (255, 255, 0),
    "GRASPED": (0, 255, 0),
    "GRASP_CHECK": (0, 200, 255),
    "HOME_WAIT": (200, 200, 200),
    "GIVE_UP": (0, 0, 255),
    "CARRY_HOME": (0, 255, 0),
    "DONE": (0, 255, 0),
    "GRASP_OK": (0, 255, 0),
    "GRASP_FAIL": (0, 0, 255),
    "ALIGNING": (0, 165, 255),
    "ALIGNED": (0, 220, 0),
    "PAUSED": (255, 120, 0),
    "DRY_RUN": (255, 200, 0),
}


@dataclass
class ApproachArgs:
    """접근(전진) + 좌우 정렬 제어 설정."""

    # 추론/정렬에 쓸 카메라 (호스트 --robot.cameras 이름)
    view: str = "front"

    # --- 후보 필터: 가운데 가로선 아래에 있는 검출만 ---

    # 박스 중심이 이 가로선(이미지 높이 × line_ratio) 아래에 있어야 목표가 된다. 0.5 = 정중앙
    line_ratio: float = 0.5
    # false 면 가로선 위/아래 상관없이 가장 큰 박스를 따라간다
    only_below_line: bool = True

    # --- 전진: 박스 크기 기준 ---

    # 이 크기(px)로 보일 때까지 전진한다.
    target_size_px: int = 117
    # 크기 판정 기준: width(폭, 기본) / height(높이) / max(둘 중 큰 값)
    size_metric: str = "width"
    # 크기가 목표 - 이 값 이상이면 "도착"으로 본다
    size_tolerance_px: int = 10
    # 박스가 화면 바닥에 붙어 있으면서 납작하거나(아래 clip_aspect_min) 목표보다 작으면
    # "너무 가까워 잘린 것"으로 보고 후진한다
    backward_when_clipped: bool = True
    # 박스 아래 변이 (이미지 높이 - 이 값) 이상이면 "바닥에 붙었다"고 본다
    bottom_margin_px: int = 5
    # 바닥에 붙은 박스의 높이/폭 이 이 값보다 작으면 잘린 것. 온전한 큐브는 ≈1.1 (117x128)
    clip_aspect_min: float = 0.8
    # 크기 오차(px) → 전진 속도 게인 (m/s per px). 오차 50px → 0.1 m/s
    kp_forward: float = 0.002
    # 목표보다 (tolerance 이상) 커졌을 때 뒤로 물러나 크기를 맞춘다. false 면 그냥 정지
    allow_backward: bool = True

    # --- 좌우: 가운데 세로선 기준 ---

    # 박스 중심 x 가 세로 중앙선에서 이 픽셀 안에 들면 좌우 정렬된 것으로 본다
    center_tolerance_px: int = 100
    # rotate (기본) : 제자리 회전(theta.vel)으로 먼저 맞추고, 맞은 뒤에만 전진한다
    # strafe        : 옆 이동(y.vel)으로 맞추며 전진과 동시에 한다
    lateral_mode: str = "rotate"
    # [rotate] 전진 중 중심이 tolerance + 이 값 이상 벗어나야 다시 회전 단계로 돌아간다 (떨림 방지)
    center_hysteresis_px: int = 10
    # [rotate] 좌우 오차(px) → 회전 속도 게인 (deg/s per px). 오차 100px → 30 deg/s
    kp_rotate: float = 0.3
    # [rotate] 회전 속도 상/하한 (deg/s). LeKiwi 텔레옵 저속 단계가 30 deg/s
    max_theta_speed: float = 30.0
    min_theta_speed: float = 8.0
    # [strafe] 좌우 오차(px) → 옆 이동 속도 게인 (m/s per px)
    kp_lateral: float = 0.002

    # --- 공통 ---

    # 속도 상/하한 (m/s). min 은 바퀴가 실제로 굴러가는 최소치, max 는 안전 상한
    max_speed: float = 0.1
    min_speed: float = 0.03
    # 연속 N 프레임 두 조건 모두 만족이면 ALIGNED
    settle_frames: int = 10
    # 검출이 이 시간(초) 이상 없으면 바퀴 정지
    lost_timeout_s: float = 0.5
    # true 면 ALIGNED 되는 순간 정지하고 스크립트를 끝낸다. false 면 계속 감시/재정렬
    stop_when_done: bool = False


STOP = {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}


def box_size(det: Detection, metric: str) -> int:
    x1, y1, x2, y2 = det.xyxy
    w, h = x2 - x1, y2 - y1
    if metric == "width":
        return w
    if metric == "height":
        return h
    return max(w, h)


def largest(dets: list[Detection]) -> Detection | None:
    """면적이 가장 큰 검출 (없으면 None)."""
    if not dets:
        return None
    return max(dets, key=lambda d: (d.xyxy[2] - d.xyxy[0]) * (d.xyxy[3] - d.xyxy[1]))


def p_speed(error_px: float, kp: float, tolerance_px: int, cfg: ApproachArgs) -> float:
    """픽셀 오차 크기 → 속도 크기 (허용치 안이면 0, 아니면 [min, max] 로 클램프한 P 제어)."""
    if abs(error_px) <= tolerance_px:
        return 0.0
    return float(np.clip(kp * abs(error_px), cfg.min_speed, cfg.max_speed))


class Approacher:
    """가장 큰 박스를 향해 (크기 → 전진, 중심 x → 좌우) 맞추는 P 제어기 + 상태 추적."""

    def __init__(self, cfg: ApproachArgs):
        self.cfg = cfg
        self.settled = 0
        self.last_seen = -float("inf")
        self.target: Detection | None = None
        self.ignored: list[Detection] = []  # 가로선 위라서 무시한 검출 (표시용)
        self.size = 0  # 현재 박스 크기(px)
        self.size_error = 0  # 목표 - 현재 (+ : 아직 작다 = 멀다)
        self.center_error = 0  # 중심 x - 화면 중앙 (+ : 화면 오른쪽)
        self.size_ok = False
        self.center_ok = False
        self.aspect = 0.0  # 박스 높이/폭 (온전한 큐브 ≈ 1.1, 잘리면 작아진다)
        self.touching_bottom = False  # 박스 아래 변이 화면 바닥에 붙어 있음
        self.too_close = False  # 바닥에 붙었는데 목표보다 작다 → 잘려 나감 → 후진
        # [rotate 모드] ROTATE: 제자리 회전으로 중심 맞추는 중 / FORWARD: 맞았으니 전진
        self.phase = "ROTATE"
        self.state = "SEARCHING"

    def line_y(self, frame_h: int) -> int:
        return int(round(frame_h * self.cfg.line_ratio))

    def update(self, dets: list[Detection], frame_shape: tuple[int, ...], now: float) -> dict[str, float]:
        """검출 결과로 베이스 속도 명령을 계산한다. 반환값은 항상 x/y/theta.vel 세 개."""
        h, w = frame_shape[:2]

        # 가운데 가로선 아래(가까운 쪽)에 중심이 있는 검출만 후보. 위쪽은 멀리 있는 것으로 보고 무시.
        if self.cfg.only_below_line:
            line_y = self.line_y(h)
            candidates = [d for d in dets if d.center[1] > line_y]
            self.ignored = [d for d in dets if d.center[1] <= line_y]
        else:
            candidates, self.ignored = list(dets), []
        self.target = largest(candidates)

        if self.target is None:
            if self.ignored:
                # 보이긴 하지만 전부 가로선 위 → 확실한 신호이므로 바로 정지/대기
                self.settled = 0
                self.state = "ABOVE_LINE"
            elif now - self.last_seen > self.cfg.lost_timeout_s:
                self.settled = 0
                self.state = "SEARCHING"
            # 그 외(lost_timeout 이전)는 잠깐 놓친 것으로 보고 정지만 (상태 유지)
            return dict(STOP)

        self.last_seen = now
        cx, _ = self.target.center
        self.size = box_size(self.target, self.cfg.size_metric)
        self.size_error = self.cfg.target_size_px - self.size
        self.center_error = cx - w // 2

        # 도착 판정: 크기는 목표 - tol 이상 (allow_backward 면 목표 + tol 이하도), 좌우는 ±tol
        self.size_ok = self.size >= self.cfg.target_size_px - self.cfg.size_tolerance_px
        if self.cfg.allow_backward:
            self.size_ok = self.size_ok and self.size <= self.cfg.target_size_px + self.cfg.size_tolerance_px
        self.center_ok = abs(self.center_error) <= self.cfg.center_tolerance_px

        # 너무 가까워서 잘렸는지: 바닥에 붙어 있으면서
        #   (a) 납작하다 (높이/폭 < clip_aspect_min) — 윗면 한 줌만 보이는 상태, 폭은 커도 잘린 것
        #   (b) 크기가 목표 - tol 보다 작다
        x1, y1, x2, y2 = self.target.xyxy
        self.aspect = (y2 - y1) / max(1, x2 - x1)
        self.touching_bottom = y2 >= h - self.cfg.bottom_margin_px
        too_small = self.size < self.cfg.target_size_px - self.cfg.size_tolerance_px
        self.too_close = (
            self.cfg.backward_when_clipped
            and self.touching_bottom
            and (self.aspect < self.cfg.clip_aspect_min or too_small)
        )
        if self.too_close:
            self.size_ok = False  # 잘린 폭은 믿을 수 없다 → 도착으로 치지 않는다

        x_vel = y_vel = theta_vel = 0.0
        rotating = False

        if self.too_close:
            # 0순위: 후진. 잘린 상태에서는 폭/중심 모두 믿을 수 없으니 회전/전진은 하지 않는다.
            #   얼마나 잘렸는지는 모르므로 일정 속도(max_speed 의 절반, min_speed 이상)로 물러난다.
            x_vel = -max(self.cfg.min_speed, self.cfg.max_speed * 0.5)
            self.phase = "ROTATE"  # 후진이 끝나면 중심부터 다시 맞춘다
        elif self.cfg.lateral_mode == "rotate":
            # 1순위: 제자리 회전으로 중심 맞추기. 회전 중에는 전진하지 않는다.
            #   ROTATE  → 중심이 ±tol 안에 들면 FORWARD 로
            #   FORWARD → 중심이 ±(tol + hysteresis) 밖으로 벗어나면 다시 ROTATE 로
            if self.phase == "FORWARD" and abs(self.center_error) > (
                self.cfg.center_tolerance_px + self.cfg.center_hysteresis_px
            ):
                self.phase = "ROTATE"
            if self.phase == "ROTATE" and self.center_ok:
                self.phase = "FORWARD"

            if self.phase == "ROTATE":
                # 화면 오른쪽(+) → 우회전. LeKiwi 는 theta + 가 좌회전(CCW) 이므로 -theta.
                theta_vel = -np.sign(self.center_error) * self._theta_speed(self.center_error)
                rotating = True
            else:
                x_vel = self._forward_speed()
        else:
            # strafe: 옆 이동과 전진을 동시에. 화면 오른쪽(+) → 오른쪽 이동 = 몸체 y 는 왼쪽이 + 이므로 -y.
            y_vel = -np.sign(self.center_error) * p_speed(
                self.center_error, self.cfg.kp_lateral, self.cfg.center_tolerance_px, self.cfg
            )
            x_vel = self._forward_speed()

        self.settled = self.settled + 1 if (self.size_ok and self.center_ok) else 0
        if self.settled >= self.cfg.settle_frames:
            self.state = "ALIGNED"
        elif self.too_close:
            self.state = "TOO_CLOSE"
        elif rotating:
            self.state = "ROTATING"
        else:
            self.state = "ALIGNING"

        return {"x.vel": float(x_vel), "y.vel": float(y_vel), "theta.vel": float(theta_vel)}

    def _forward_speed(self) -> float:
        """크기 오차 → 전진 속도. 아직 작다(+) → 앞으로. 커졌으면 기본은 정지, allow_backward 면 후진."""
        if self.size_error > 0:
            return p_speed(self.size_error, self.cfg.kp_forward, self.cfg.size_tolerance_px, self.cfg)
        if self.cfg.allow_backward:
            return -p_speed(self.size_error, self.cfg.kp_forward, self.cfg.size_tolerance_px, self.cfg)
        return 0.0

    def _theta_speed(self, error_px: float) -> float:
        """좌우 픽셀 오차 크기 → 회전 속도 크기 (deg/s), [min, max] 로 클램프."""
        if abs(error_px) <= self.cfg.center_tolerance_px:
            return 0.0
        return float(np.clip(self.cfg.kp_rotate * abs(error_px), self.cfg.min_theta_speed, self.cfg.max_theta_speed))

    @property
    def done(self) -> bool:
        return self.state == "ALIGNED"


def draw_alignment(
    frame_bgr: np.ndarray, ap: Approacher, cmd: dict[str, float], state_label: str
) -> np.ndarray:
    """오버레이: 가운데 세로선 + 좌우 허용 띠 + 목표 박스 강조 + 목표 크기 참조 박스 + 오차/명령 텍스트."""
    canvas = frame_bgr
    h, w = canvas.shape[:2]
    tol = ap.cfg.center_tolerance_px
    mx = w // 2

    # 좌우 허용 범위 띠(반투명) + 가운데 세로선
    band = canvas.copy()
    cv2.rectangle(band, (mx - tol, 0), (mx + tol, h), BAND_COLOR, -1)
    cv2.addWeighted(band, 0.25, canvas, 0.75, 0, canvas)
    cv2.line(canvas, (mx, 0), (mx, h), LINE_COLOR, 1, cv2.LINE_AA)

    # 가운데 가로선: 이 선 아래의 검출만 목표가 된다
    if ap.cfg.only_below_line:
        ly = ap.line_y(h)
        cv2.line(canvas, (0, ly), (w, ly), LINE_COLOR, 1, cv2.LINE_AA)
        cv2.putText(canvas, "target zone", (6, ly + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, LINE_COLOR, 1, cv2.LINE_AA)

    # 가로선 위라서 무시한 검출은 회색으로 덮어 표시
    for d in ap.ignored:
        x1, y1, x2, y2 = d.xyxy
        cv2.rectangle(canvas, (x1, y1), (x2, y2), IGNORED_COLOR, 2)
        cv2.line(canvas, (x1, y1), (x2, y2), IGNORED_COLOR, 1)
        cv2.line(canvas, (x1, y2), (x2, y1), IGNORED_COLOR, 1)
        cv2.putText(canvas, "ignored", (x1, max(y1 - 6, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, IGNORED_COLOR, 1, cv2.LINE_AA)

    t = ap.target
    if t is not None:
        x1, y1, x2, y2 = t.xyxy
        cx, cy = t.center
        cv2.rectangle(canvas, (x1, y1), (x2, y2), TARGET_COLOR, 3)
        cv2.circle(canvas, (cx, cy), 6, TARGET_COLOR, 2)
        # 중심 → 세로선까지의 좌우 오차를 가로 화살표로
        if abs(ap.center_error) > tol:
            cv2.arrowedLine(canvas, (cx, cy), (mx, cy), TARGET_COLOR, 2, tipLength=0.3)
        # 목표 크기 참조: 박스 중심에 목표 폭(높이는 현재 비율 유지)의 노란 사각형
        ts = ap.cfg.target_size_px
        ref_w = ts if ap.cfg.size_metric != "height" else max(1, int(round((x2 - x1) * ts / max(1, y2 - y1))))
        ref_h = ts if ap.cfg.size_metric != "width" else max(1, int(round((y2 - y1) * ts / max(1, x2 - x1))))
        cv2.rectangle(canvas, (cx - ref_w // 2, cy - ref_h // 2), (cx + ref_w // 2, cy + ref_h // 2), SIZE_REF_COLOR, 1, cv2.LINE_AA)

        if ap.touching_bottom:
            # 바닥에 붙은 박스: 아래 변을 두껍게 표시. 너무 가까우면 빨간 경고까지
            edge_color = STATE_COLORS["TOO_CLOSE"] if ap.too_close else TARGET_COLOR
            cv2.line(canvas, (x1, h - 2), (x2, h - 2), edge_color, 4)
            if ap.too_close:
                cv2.putText(canvas, f"TOO CLOSE (h/w={ap.aspect:.2f}) - backing up", (x1, max(y1 - 52, 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, edge_color, 2, cv2.LINE_AA)

        size_txt = f"{ap.cfg.size_metric}={ap.size}/{ts}px" + (" OK" if ap.size_ok else "")
        dx_txt = f"dx={ap.center_error:+d}px" + (" OK" if ap.center_ok else "")
        cv2.putText(canvas, f"{size_txt}  {dx_txt}", (x1, max(y1 - 30, 40)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, TARGET_COLOR, 2, cv2.LINE_AA)

    color = STATE_COLORS.get(state_label, (255, 255, 255))
    cmd_txt = f"x={cmd['x.vel']:+.2f} y={cmd['y.vel']:+.2f} m/s  th={cmd['theta.vel']:+5.1f} deg/s"
    cv2.rectangle(canvas, (0, h - 26), (w, h), (0, 0, 0), -1)
    cv2.putText(canvas, f"{state_label}  {cmd_txt}", (6, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)
    return canvas


def status_line(ap: Approacher, cmd: dict[str, float], state_label: str, hz: float) -> str:
    if ap.target is None:
        target = f"target: - (가로선 위 {len(ap.ignored)}개 무시)" if ap.ignored else "target: -"
    else:
        cx, cy = ap.target.center
        target = (
            f"target {ap.target.name} {ap.target.conf:.2f} @({cx},{cy}) "
            f"{ap.cfg.size_metric}={ap.size}/{ap.cfg.target_size_px}{'✓' if ap.size_ok else ''} "
            f"dx={ap.center_error:+d}{'✓' if ap.center_ok else ''}"
            + (f" [바닥·잘림 h/w={ap.aspect:.2f}→후진]" if ap.too_close else f" [바닥 h/w={ap.aspect:.2f}]" if ap.touching_bottom else "")
        )
    return (
        f"\r[{state_label:10s}] {target} | x={cmd['x.vel']:+.2f} y={cmd['y.vel']:+.2f} th={cmd['theta.vel']:+5.1f} "
        f"| settled {ap.settled}/{ap.cfg.settle_frames} | {hz:5.1f} Hz   "
    )
