"""Ultralytics YOLO 추론 래퍼.

원시 ultralytics Results 파싱(순수 로직, `parse_detections`/`best_detection`/
`Detection`)과 실제 모델 로딩+추론(하드웨어 의존, `load_model_and_detect`)을 분리한다
— 전자는 실제 모델 파일 없이 테스트 가능, 후자는 Task 11(YOLO 1차 시험)/Task 12
(통합 스크립트)에서 실기기로 검증한다."""
from dataclasses import dataclass


@dataclass
class Detection:
    label: str
    confidence: float
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def center_x(self) -> float:
        return (self.x1 + self.x2) / 2

    @property
    def center_y(self) -> float:
        return (self.y1 + self.y2) / 2

    @property
    def height_px(self) -> float:
        return self.y2 - self.y1


def parse_detections(boxes_xyxy: list[tuple[float, float, float, float]],
                      confidences: list[float], class_names: list[str],
                      class_ids: list[int]) -> list['Detection']:
    detections = []
    for (x1, y1, x2, y2), conf, cls_id in zip(boxes_xyxy, confidences, class_ids):
        detections.append(Detection(
            label=class_names[cls_id], confidence=conf, x1=x1, y1=y1, x2=x2, y2=y2))
    return detections


def best_detection(detections: list['Detection'], target_label: str,
                    min_confidence: float) -> 'Detection | None':
    candidates = [d for d in detections
                  if d.label == target_label and d.confidence >= min_confidence]
    if not candidates:
        return None
    return max(candidates, key=lambda d: d.confidence)


def load_model_and_detect(model_path: str, frame, target_label: str,
                           min_confidence: float) -> 'Detection | None':
    """실제 YOLO 모델을 로드해 frame(numpy BGR 이미지)에 추론 후 최적 검출 하나를
    반환한다. ultralytics 의존 — 이 함수 자체는 단위테스트하지 않고(모델 파일+무거운
    의존성 필요), Task 11/12의 실기기 검증으로 대신한다."""
    from ultralytics import YOLO
    model = load_model_and_detect._cached_model
    if model is None or load_model_and_detect._cached_path != model_path:
        model = YOLO(model_path)
        load_model_and_detect._cached_model = model
        load_model_and_detect._cached_path = model_path
    results = model(frame, verbose=False)[0]
    boxes_xyxy = results.boxes.xyxy.tolist()
    confidences = results.boxes.conf.tolist()
    class_ids = [int(c) for c in results.boxes.cls.tolist()]
    class_names = [results.names[i] for i in range(len(results.names))]
    detections = parse_detections(boxes_xyxy, confidences, class_names, class_ids)
    return best_detection(detections, target_label, min_confidence)


load_model_and_detect._cached_model = None
load_model_and_detect._cached_path = None
