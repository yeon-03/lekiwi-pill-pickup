#!/usr/bin/env python
"""노트북에 직접 연결된 USB 카메라를 열어 YOLO 검출 결과를 실시간으로 오버레이한다.
q 또는 ESC로 종료. live_view_usb_local.py(로컬 카메라 열기)와
live_view_yolo.py(YOLO 오버레이 그리기) 두 스크립트의 패턴을 합친 것.

⚠️ 2026-08-24 세션에서 YOLO가 소형 병을 vase로 오분류/미검출해 실제 파이프라인은
색상검출(color_detect.py)로 교체됨(docs/session_2026-08-24_progress.md 참고) — 이
스크립트는 클래스 필터를 기본으로 걸지 않고 전체 검출 결과를 보여줘서 어떤 라벨로
잡히는지 직접 확인하는 진단용이다."""
import argparse
import sys

import cv2
from ultralytics import YOLO


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument('--index', type=int, default=0, help='/dev/videoN 의 N')
    p.add_argument('--model', default='yolov8n.pt',
                    help='yolov8n.pt(빠름)/yolov8s.pt/yolov8m.pt(느리지만 정확) 중 선택')
    p.add_argument('--conf', type=float, default=0.25, help='YOLO confidence threshold')
    p.add_argument('--label', nargs='*', default=None,
                    help='이 라벨만 표시(예: --label bottle vase). 생략하면 전체 클래스 표시')
    args = p.parse_args()

    print(f'YOLO 모델({args.model}) 로딩 중...')
    model = YOLO(args.model)

    cap = cv2.VideoCapture(args.index)
    if not cap.isOpened():
        print(f'/dev/video{args.index} 를 열 수 없어요', file=sys.stderr)
        sys.exit(1)

    window = f'USB 카메라 video{args.index} + YOLO({args.model}) (q로 종료)'
    label_filter = set(args.label) if args.label else None
    print(f'/dev/video{args.index} 열림 — 창에서 q 또는 ESC 누르면 종료')
    if label_filter is not None:
        print(f'표시 라벨 필터: {sorted(label_filter)}')

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print('프레임을 못 읽었어요', file=sys.stderr)
                break

            results = model(frame, verbose=False, conf=args.conf)[0]
            for box, cls, conf in zip(results.boxes.xyxy, results.boxes.cls, results.boxes.conf):
                name = results.names[int(cls)]
                if label_filter is not None and name not in label_filter:
                    continue
                x1, y1, x2, y2 = map(int, box.tolist())
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
                cv2.putText(frame, f'{name} {float(conf):.2f}', (x1, max(0, y1 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            cv2.imshow(window, frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f'실패: {type(e).__name__}: {e}', file=sys.stderr)
        sys.exit(1)
