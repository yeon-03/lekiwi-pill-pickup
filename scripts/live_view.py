#!/usr/bin/env python
"""LeKiwi 카메라 실시간 미리보기 창 — 물체/각도 잡을 때 쓰는 임시 디버그 도구.
q 또는 ESC로 종료. 실기기 위치 확인용, 저장은 안 함."""
import argparse
import sys

import cv2

from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--lekiwi-host', default='192.168.0.201')
    parser.add_argument('--camera', choices=('wrist', 'front'), default='front')
    args = parser.parse_args()

    client = LeKiwiClient(LeKiwiClientConfig(remote_ip=args.lekiwi_host))
    print(f'{args.lekiwi_host}에 연결 중...')
    client.connect()
    print('연결됨 — 창에서 q 또는 ESC 누르면 종료')
    window_name = f'LeKiwi {args.camera} 카메라 (실시간, q로 종료)'
    try:
        while True:
            obs = client.get_observation()
            # lerobot 카메라 코드가 BGR->RGB로 변환해서 넘겨줌(ML 관례) — cv2.imshow는
            # BGR을 기대하므로 되돌려줘야 함, 안 그러면 빨강/파랑이 뒤바뀌어 보임
            frame = cv2.cvtColor(obs[args.camera], cv2.COLOR_RGB2BGR)
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
