#!/usr/bin/env python
"""체커보드로 카메라 렌즈 왜곡 계수를 구한다(OpenCV 표준 방식).

사용 순서:
  1) capture: 체커보드를 여러 각도/위치로 들고 20장 정도 촬영
       python calibrate_distortion.py capture data/calib_front --camera front
  2) compute: 찍은 사진들로 카메라 행렬 + 왜곡계수 계산
       python calibrate_distortion.py compute data/calib_front --cols 7 --rows 5
     결과는 같은 디렉터리에 camera_calib.npz + 요약 텍스트로 저장

--cols/--rows는 "내부 코너" 개수다(칸 개수가 아님).
8x6칸 체스판이면 내부 코너는 7x5.
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np


def capture(out_dir: Path, camera: str, host: str, count: int, interval: float) -> None:
    from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig
    import time

    out_dir.mkdir(parents=True, exist_ok=True)
    existing = len(list(out_dir.glob('*.jpg')))
    client = LeKiwiClient(LeKiwiClientConfig(remote_ip=host))
    print(f'{host} 연결 중...')
    client.connect()
    try:
        for i in range(count):
            if i > 0:
                print(f'{interval:.0f}초 후 촬영 — 체커보드 각도/위치를 바꿔주세요...')
                time.sleep(interval)
            obs = client.get_observation()
            frame = cv2.cvtColor(obs[camera], cv2.COLOR_RGB2BGR)
            path = out_dir / f'{existing + i + 1:02d}.jpg'
            cv2.imwrite(str(path), frame)
            print(f'[{i + 1}/{count}] 저장: {path}')
    finally:
        client.disconnect()


def compute(img_dir: Path, cols: int, rows: int, square_mm: float) -> None:
    pattern = (cols, rows)
    # 체커보드 3D 좌표(평면이므로 z=0), 실제 칸 크기(mm)를 곱해 단위를 맞춘다
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_mm

    obj_points, img_points = [], []
    paths = sorted(img_dir.glob('*.jpg'))
    if not paths:
        print(f'{img_dir}에 jpg가 없어요')
        sys.exit(1)

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    shape = None
    for p in paths:
        img = cv2.imread(str(p))
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        shape = gray.shape[::-1]
        found, corners = cv2.findChessboardCorners(gray, pattern, None)
        if found:
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            obj_points.append(objp)
            img_points.append(corners)
        print(f'{p.name}: {"코너 검출됨" if found else "실패"}')

    print(f'\n사용 가능한 사진: {len(obj_points)}/{len(paths)}장')
    if len(obj_points) < 5:
        print('최소 5장 이상 필요해요 — 더 촬영해주세요')
        sys.exit(1)

    rms, mtx, dist, _, _ = cv2.calibrateCamera(obj_points, img_points, shape, None, None)
    print(f'\n재투영 오차(RMS): {rms:.4f} px  (1.0 미만이면 양호)')
    print(f'초점거리: fx={mtx[0, 0]:.1f}  fy={mtx[1, 1]:.1f}')
    print(f'주점(중심): cx={mtx[0, 2]:.1f}  cy={mtx[1, 2]:.1f}')
    print(f'왜곡계수: {dist.ravel()}')

    out = img_dir / 'camera_calib.npz'
    np.savez(out, camera_matrix=mtx, dist_coeffs=dist, rms=rms, image_size=shape)
    print(f'\n저장: {out}')


def main() -> None:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest='mode', required=True)

    c = sub.add_parser('capture')
    c.add_argument('out_dir')
    c.add_argument('--camera', choices=('wrist', 'front'), default='front')
    c.add_argument('--lekiwi-host', default='192.168.0.201')
    c.add_argument('--count', type=int, default=20)
    c.add_argument('--interval', type=float, default=4.0)

    m = sub.add_parser('compute')
    m.add_argument('img_dir')
    m.add_argument('--cols', type=int, required=True, help='내부 코너 가로 개수')
    m.add_argument('--rows', type=int, required=True, help='내부 코너 세로 개수')
    m.add_argument('--square-mm', type=float, default=25.0, help='칸 한 변 길이(mm)')

    args = parser.parse_args()
    if args.mode == 'capture':
        capture(Path(args.out_dir), args.camera, args.lekiwi_host, args.count, args.interval)
    else:
        compute(Path(args.img_dir), args.cols, args.rows, args.square_mm)


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f'실패: {type(e).__name__}: {e}', file=sys.stderr)
        sys.exit(1)
