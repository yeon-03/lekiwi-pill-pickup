#!/usr/bin/env python
"""색상 기반 실시간 검출 미리보기 — YOLO 안 씀.

YOLO가 이 작은 점안액 병을 'bottle'이 아니라 'vase'로 분류하거나 아예 놓치는 문제가
반복돼서(2026-08-24 실측), 형태를 안 보고 색만 보는 방식으로 대체한 것."""
import argparse
import sys

import cv2

from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig

sys.path.insert(0, str(__import__('pathlib').Path(__file__).parent))
from color_detect import COLOR_RANGES, find_color_blobs  # noqa: E402

DRAW = {'red': (0, 0, 255),
        'green': (0, 255, 0), 'blue': (255, 0, 0), 'white': (200, 200, 200),
        'purple': (255, 0, 255)}


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument('--lekiwi-host', default='192.168.0.201')
    p.add_argument('--camera', choices=('wrist', 'front'), default='wrist')
    # purple(집게)은 정렬 로직 전용 — 화면 표시 기본값에선 제외
    p.add_argument('--colors', nargs='*',
                   default=[c for c in COLOR_RANGES if c != 'purple'])
    p.add_argument('--min-area', type=int, default=150)
    args = p.parse_args()

    client = LeKiwiClient(LeKiwiClientConfig(remote_ip=args.lekiwi_host))
    print(f'{args.lekiwi_host} 연결 중...')
    client.connect()
    print('연결됨 — q/ESC로 종료')
    window = f'{args.camera} 색상검출 (q로 종료)'
    try:
        while True:
            obs = client.get_observation()
            frame = cv2.cvtColor(obs[args.camera], cv2.COLOR_RGB2BGR)
            for color in args.colors:
                for x1, y1, x2, y2, area in find_color_blobs(frame, color, args.min_area):
                    cv2.rectangle(frame, (x1, y1), (x2, y2), DRAW[color], 2)
                    cv2.putText(frame, f'{color} {area}', (x1, max(0, y1 - 6)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, DRAW[color], 2)
            cv2.imshow(window, frame)
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
