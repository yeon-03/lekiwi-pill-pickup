"""YOLO 추론 + 오버레이 그리기 — lekiwi_yolo_view.py 이식.

원본: `/home/khw/workspace/lekiwi/yolo_and_pick/lekiwi_yolo_view.py`
로봇 접속(`LeKiwiRobotArgs`)·표시 루프(`view_loop`/`main`)는 이식하지 않는다 — 그 역할은
워크스페이스의 드라이런/Step 3 이 대신한다.
"""

import logging
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from services.pickplace import PickPlaceError

# 클래스별 박스 색 (BGR). 클래스 수보다 적으면 순환해서 쓴다.
BOX_COLORS = [(60, 60, 255), (60, 220, 60), (255, 170, 40), (220, 60, 220), (40, 220, 220)]
# 가상 중앙 가로선/세로선 색 (BGR, 초록)
CROSSHAIR_COLOR = (0, 255, 0)


@dataclass
class YoloArgs:
    """ultralytics YOLO 추론 설정."""

    # 가중치 경로. 앱에서는 사용자가 파일창으로 고른 경로를 채운다 (상대 경로 기본값 금지).
    path: str = ""
    conf: float = 0.4
    iou: float = 0.45
    imgsz: int = 640
    # None 이면 ultralytics 가 알아서 고른다 ("cpu", "0", "cuda:0" 등).
    device: str | None = None
    max_det: int = 10
    # 특정 클래스만 보고 싶을 때
    classes: list[int] | None = None
    half: bool = False


@dataclass
class Detection:
    """한 개의 검출 결과 (픽셀 좌표계)."""

    name: str
    conf: float
    xyxy: tuple[int, int, int, int]
    cls: int

    @property
    def center(self) -> tuple[int, int]:
        x1, y1, x2, y2 = self.xyxy
        return (x1 + x2) // 2, (y1 + y2) // 2


def load_model(cfg: YoloArgs):
    """ultralytics 모델 로드 (없으면 무엇을 해야 하는지 알려주고 오류)."""
    try:
        from ultralytics import YOLO
    except ImportError:
        raise PickPlaceError(
            "ultralytics 가 설치돼 있지 않습니다. 워크스페이스 > YOLO 모델 단계에서 설치하세요."
        ) from None

    path = Path(cfg.path).expanduser()
    if not cfg.path or not path.exists():
        raise PickPlaceError(
            f"error: 모델 파일이 없습니다: {path}\n"
            "  워크스페이스 > YOLO 모델 단계에서 모델 파일(.pt)을 선택하세요."
        )

    logging.info("YOLO 모델 로드: %s", path)
    model = YOLO(str(path))
    logging.info("클래스: %s", model.names)
    return model


def precision_kwargs(cfg: YoloArgs) -> dict:
    """`half` 옵션 → `predict()` 인자.

    ultralytics 8.4 부터 `half` 가 `quantize` 로 대체돼 **호출마다** "'half' is deprecated" 경고를
    찍는다 (30 Hz 루프에서 로그가 넘친다 — 2026-09-08 실측). 기본값(`half=False`)이면 아무 인자도
    넘기지 않고, 켜져 있으면 버전에 따라 `quantize=16`(8.4+) 또는 `half=True`(구버전)를 준다.
    원본 `lekiwi_yolo_view.py` 는 항상 `half=cfg.half` 를 넘긴다 — 의도적으로 다른 지점.
    """
    if not cfg.half:
        return {}
    try:
        import ultralytics

        major, minor = (int(x) for x in str(ultralytics.__version__).split(".")[:2])
        if (major, minor) >= (8, 4):
            return {"quantize": 16}
    except Exception:
        pass
    return {"half": True}


def infer(model, cfg: YoloArgs, frames_bgr: dict[str, np.ndarray]) -> dict[str, list[Detection]]:
    """뷰 여러 개를 한 번의 배치로 추론한다."""
    names = list(frames_bgr)
    results = model.predict(
        [frames_bgr[n] for n in names],
        conf=cfg.conf,
        iou=cfg.iou,
        imgsz=cfg.imgsz,
        device=cfg.device,
        max_det=cfg.max_det,
        classes=cfg.classes,
        verbose=False,
        **precision_kwargs(cfg),
    )

    out: dict[str, list[Detection]] = {}
    for view, result in zip(names, results, strict=True):
        dets: list[Detection] = []
        for box in result.boxes:
            cls = int(box.cls.item())
            x1, y1, x2, y2 = (int(v) for v in box.xyxy[0].tolist())
            dets.append(
                Detection(
                    name=result.names.get(cls, str(cls)),
                    conf=float(box.conf.item()),
                    xyxy=(x1, y1, x2, y2),
                    cls=cls,
                )
            )
        dets.sort(key=lambda d: d.conf, reverse=True)
        out[view] = dets
    return out


def draw_crosshair(canvas: np.ndarray) -> np.ndarray:
    """이미지 정중앙을 지나는 가로선/세로선을 제자리에 그린다 (초록)."""
    h, w = canvas.shape[:2]
    cx, cy = w // 2, h // 2
    cv2.line(canvas, (0, cy), (w, cy), CROSSHAIR_COLOR, 1, cv2.LINE_AA)
    cv2.line(canvas, (cx, 0), (cx, h), CROSSHAIR_COLOR, 1, cv2.LINE_AA)
    cv2.circle(canvas, (cx, cy), 5, CROSSHAIR_COLOR, 1, cv2.LINE_AA)
    return canvas


def draw(
    frame_bgr: np.ndarray, view: str, dets: list[Detection], fps: float | None, crosshair: bool = False
) -> np.ndarray:
    """검출 박스 + 중심점 + 뷰 이름(+ 옵션으로 중앙 십자선)을 그린 새 이미지를 돌려준다."""
    canvas = frame_bgr.copy()
    if crosshair:
        draw_crosshair(canvas)  # 박스보다 먼저 그려서 박스/라벨이 선 위에 오도록
    for det in dets:
        x1, y1, x2, y2 = det.xyxy
        color = BOX_COLORS[det.cls % len(BOX_COLORS)]
        cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 2)

        label = f"{det.name} {det.conf:.2f}"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        ty = max(y1, th + 4)
        cv2.rectangle(canvas, (x1, ty - th - 4), (x1 + tw + 4, ty), color, -1)
        cv2.putText(canvas, label, (x1 + 2, ty - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

        cx, cy = det.center
        cv2.circle(canvas, (cx, cy), 4, color, -1)
        cv2.putText(
            canvas, f"({cx},{cy})", (cx + 6, cy - 6),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA,
        )

    header = f"{view}  det={len(dets)}"
    if fps is not None:
        header += f"  {fps:4.1f} Hz"
    cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 22), (0, 0, 0), -1)
    cv2.putText(canvas, header, (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return canvas


def hstack_views(images: list[np.ndarray], height: int) -> np.ndarray:
    """높이를 맞춰 가로로 이어 붙인다 (뷰 해상도가 서로 달라도 된다)."""
    resized = []
    for img in images:
        scale = height / img.shape[0]
        resized.append(cv2.resize(img, (max(1, int(round(img.shape[1] * scale))), height)))
    return np.hstack(resized)
