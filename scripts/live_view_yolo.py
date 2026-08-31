#!/usr/bin/env python
"""LeKiwi 카메라 실시간 미리보기 + YOLO bottle 검출 + 색상판정 오버레이.
q 또는 ESC로 종료. test_bottle_color.py의 판정 로직을 프레임마다 그대로 적용."""
import argparse
import sys

import cv2
from ultralytics import YOLO

from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig

sys.path.insert(0, str(__import__('pathlib').Path(__file__).parent))
from test_bottle_color import classify_color  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--lekiwi-host', default='192.168.0.201')
    parser.add_argument('--camera', choices=('wrist', 'front'), default='front')
    parser.add_argument('--conf', type=float, default=0.25, help='YOLO confidence threshold')
    args = parser.parse_args()

    print('YOLO 모델 로딩 중...')
    model = YOLO('yolov8n.pt')

    client = LeKiwiClient(LeKiwiClientConfig(remote_ip=args.lekiwi_host))
    print(f'{args.lekiwi_host}에 연결 중...')
    client.connect()
    print('연결됨 — 창에서 q 또는 ESC 누르면 종료')
    window_name = f'LeKiwi {args.camera} + YOLO (q로 종료)'
    try:
        while True:
            obs = client.get_observation()
            # lerobot은 RGB로 넘겨줌 — cv2/YOLO 둘 다 BGR 기준이라 되돌려줌
            frame = cv2.cvtColor(obs[args.camera], cv2.COLOR_RGB2BGR)

            results = model(frame, verbose=False, conf=args.conf)[0]
            for box, cls, conf in zip(results.boxes.xyxy, results.boxes.cls, results.boxes.conf):
                if results.names[int(cls)] != 'bottle':
                    continue
                x1, y1, x2, y2 = map(int, box.tolist())
                crop = frame[y1:y2, x1:x2]
                label, _ = classify_color(crop) if crop.size else ('?', {})
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
                cv2.putText(frame, f'{label} {float(conf):.2f}', (x1, max(0, y1 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            cv2.imshow(window_name, frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                break
    finally:
        client.disconnect()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f'실패: {type(e).__name__}: {e}', file=sys.stderr)
        sys.exit(1)
