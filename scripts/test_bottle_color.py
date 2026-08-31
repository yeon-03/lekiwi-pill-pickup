#!/usr/bin/env python
"""사전학습 YOLO로 병(bottle) 검출 후, 박스 안 픽셀의 HSV 색상으로 빨강/초록/파랑을
구별할 수 있는지 테스트하는 1회성 스파이크 스크립트. 파인튜닝 없이 색상 후처리만으로
충분한지 확인하는 목적 — 결과 보고 실제 파이프라인에 넣을지 판단한다."""
import sys

import cv2
import numpy as np
from ultralytics import YOLO


def classify_color(bgr_crop: np.ndarray) -> tuple[str, dict]:
    hsv = cv2.cvtColor(bgr_crop, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
    # 채도 낮은(흰색/회색 반사광, 배경) 픽셀은 제외 — 액체 색만 보고 싶음
    mask = s > 60
    if mask.sum() < 20:
        return 'unknown(채도 부족)', {}

    hues = h[mask]
    red_frac = float(np.mean((hues < 10) | (hues > 170)))
    green_frac = float(np.mean((hues >= 35) & (hues <= 85)))
    blue_frac = float(np.mean((hues >= 95) & (hues <= 130)))
    scores = {'red': red_frac, 'green': green_frac, 'blue': blue_frac}
    label = max(scores, key=scores.get)
    if scores[label] < 0.3:
        label = 'unknown(구간 불명확)'
    return label, scores


def main(image_path: str) -> None:
    model = YOLO('yolov8n.pt')
    img = cv2.imread(image_path)
    if img is None:
        print(f'이미지를 못 읽음: {image_path}')
        sys.exit(1)

    results = model(image_path, verbose=False)[0]
    bottle_boxes = [
        (box, float(conf))
        for box, cls, conf in zip(results.boxes.xyxy, results.boxes.cls, results.boxes.conf)
        if results.names[int(cls)] == 'bottle'
    ]
    print(f'검출된 bottle 개수: {len(bottle_boxes)}')

    annotated = img.copy()
    for i, (box, conf) in enumerate(bottle_boxes):
        x1, y1, x2, y2 = map(int, box.tolist())
        crop = img[y1:y2, x1:x2]
        label, scores = classify_color(crop)
        print(f'  [{i}] box=({x1},{y1},{x2},{y2}) conf={conf:.2f} -> 색상={label} '
              f'(점수: {", ".join(f"{k}={v:.2f}" for k, v in scores.items())})')

        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 255), 2)
        cv2.putText(annotated, f'{label} {conf:.2f}', (x1, max(0, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    out_path = image_path.rsplit('.', 1)[0] + '_annotated.jpg'
    cv2.imwrite(out_path, annotated)
    print(f'결과 이미지 저장: {out_path}')


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print(f'사용법: {sys.argv[0]} <이미지_경로>')
        sys.exit(1)
    main(sys.argv[1])
