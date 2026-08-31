#!/usr/bin/env python
"""LeKiwiClient로 카메라 프레임을 받아 파일로 저장한다.

⚠️ 실기기 미검증 — LeRobot 소스(config_lekiwi.py) 기준으로 카메라 키는 'front'
또는 'wrist' 둘 중 하나(기본 wrist — 실제 픽업 파이프라인이 쓰는 것과 동일).

기본은 여러 장 연속 촬영(--count, 기본 12장) — 매번 파일명 바꿔가며 명령을 여러 번
치는 게 번거로워서, 한 번 실행하면 --interval(기본 3초) 간격으로 자동으로 찍는다.
그 사이에 물체 각도/거리를 바꿀 시간을 벌기 위한 카운트다운을 출력한다.

사용법:
    python capture_frame.py data/pretrained_test          # 12장, 3초 간격
    python capture_frame.py data/pretrained_test --count 20 --interval 2
    python capture_frame.py data/pretrained_test --camera front
    python capture_frame.py out.jpg --count 1              # 예전처럼 한 장만(파일명 직접 지정)
"""
import argparse
import sys
import time
from pathlib import Path

import cv2

from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('out', help='저장 위치 — count>1이면 디렉터리(자동으로 01.jpg, 02.jpg...), '
                                     'count=1이면 파일 경로 그대로 사용')
    parser.add_argument('--lekiwi-host', default='192.168.0.201')
    parser.add_argument('--camera', choices=('wrist', 'front'), default='wrist')
    parser.add_argument('--count', type=int, default=12, help='찍을 장수 (기본 12)')
    parser.add_argument('--interval', type=float, default=3.0,
                         help='촬영 사이 대기 시간(초) — 그 사이 물체를 움직일 시간 (기본 3초)')
    args = parser.parse_args()

    if args.count > 1:
        out_dir = Path(args.out)
        out_dir.mkdir(parents=True, exist_ok=True)
        existing = len(list(out_dir.glob('*.jpg')))
    else:
        out_dir = None

    client = LeKiwiClient(LeKiwiClientConfig(remote_ip=args.lekiwi_host))
    print(f'{args.lekiwi_host}에 연결 중... (수 초 걸릴 수 있음)')
    client.connect()
    try:
        for i in range(args.count):
            if i > 0:
                print(f'{args.interval:.0f}초 후 다음 촬영 — 그동안 물체 위치/각도를 바꿔주세요...')
                time.sleep(args.interval)

            obs = client.get_observation()
            # lerobot 카메라 코드가 BGR->RGB로 변환해서 넘겨줌(ML 관례) — cv2.imwrite는
            # BGR을 기대하므로 되돌려줘야 함, 안 그러면 저장된 사진의 빨강/파랑이 뒤바뀜
            frame = cv2.cvtColor(obs[args.camera], cv2.COLOR_RGB2BGR)

            if out_dir is not None:
                out_path = out_dir / f'{existing + i + 1:02d}.jpg'
            else:
                out_path = Path(args.out)

            cv2.imwrite(str(out_path), frame)
            print(f'[{i + 1}/{args.count}] 저장됨: {out_path} ({frame.shape[1]}x{frame.shape[0]})')
    finally:
        client.disconnect()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f'실패: {type(e).__name__}: {e}', file=sys.stderr)
        sys.exit(1)
