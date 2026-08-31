#!/usr/bin/env python
"""실시간 화면을 보면서 스페이스바로 원하는 순간에 촬영한다.
캘리브레이션용 체커보드 촬영처럼 타이밍이 중요한 경우에 쓴다.

스페이스 = 촬영 / q 또는 ESC = 종료
체커보드 코너가 검출되면 화면에 표시해줘서, 쓸만한 사진인지 바로 알 수 있다."""
import argparse
import sys
from pathlib import Path

import cv2

from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('out_dir')
    parser.add_argument('--camera', choices=('wrist', 'front'), default='front')
    parser.add_argument('--lekiwi-host', default='192.168.0.201')
    parser.add_argument('--cols', type=int, default=0, help='체커보드 내부 코너 가로(0이면 검출 안 함)')
    parser.add_argument('--rows', type=int, default=0, help='체커보드 내부 코너 세로')
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    saved = len(list(out_dir.glob('*.jpg')))

    client = LeKiwiClient(LeKiwiClientConfig(remote_ip=args.lekiwi_host))
    print(f'{args.lekiwi_host} 연결 중...')
    client.connect()
    print('연결됨 — 스페이스=촬영, q/ESC=종료')
    window = f'{args.camera} 카메라 — 스페이스로 촬영'
    detect = args.cols > 0 and args.rows > 0
    try:
        while True:
            obs = client.get_observation()
            frame = cv2.cvtColor(obs[args.camera], cv2.COLOR_RGB2BGR)
            display = frame.copy()

            found = False
            found_size = None
            if detect:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                # 사용자가 준 값이 칸 개수인지 내부코너 개수인지 헷갈리기 쉬워서
                # 근처 후보들을 다 시도하고 맞는 걸 찾아 알려준다
                candidates = [(args.cols, args.rows), (args.cols - 1, args.rows - 1),
                              (args.rows, args.cols), (args.rows - 1, args.cols - 1)]
                for size in candidates:
                    if size[0] < 3 or size[1] < 3:
                        continue
                    found, corners = cv2.findChessboardCorners(gray, size, None)
                    if found:
                        found_size = size
                        cv2.drawChessboardCorners(display, size, corners, found)
                        break

            status = f'saved={saved}'
            if detect:
                status += f'  CORNERS OK {found_size}' if found else '  no corners'
            color = (0, 255, 0) if (not detect or found) else (0, 0, 255)
            cv2.putText(display, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

            cv2.imshow(window, display)
            key = cv2.waitKey(1) & 0xFF
            if key == ord(' '):
                saved += 1
                path = out_dir / f'{saved:02d}.jpg'
                cv2.imwrite(str(path), frame)
                print(f'저장: {path}' + ('  (코너 검출됨)' if found else ''))
            elif key == ord('q') or key == 27:
                break
    finally:
        client.disconnect()
        cv2.destroyAllWindows()
        print(f'총 {saved}장')


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f'실패: {type(e).__name__}: {e}', file=sys.stderr)
        sys.exit(1)
