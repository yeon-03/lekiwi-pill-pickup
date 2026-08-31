#!/usr/bin/env python
"""손목 카메라의 '팔 기준 위치'(외부 파라미터)를 체커보드로 구한다.

전제: 팔을 정해진 스캔 자세에 파킹한 상태. 그 자세에서만 유효한 값이 나오므로,
이후 물체를 찾을 때도 반드시 같은 자세로 파킹해야 한다.

절차:
  1) 팔을 스캔 자세로 파킹 (goto_pose.py 등으로)
  2) 체커보드를 책상 위 '팔 기준 알려진 위치'에 평평하게 놓는다
     - 보드의 (0,0) 코너 위치를 팔 밑동 기준으로 자로 잰다 -> --board-x/--board-y/--board-z
     - 보드의 +X 방향이 팔 +X와 같으면 --board-yaw 0
  3) 이 스크립트 실행 -> solvePnP -> T_base_camera 저장

결과는 data/extrinsic_wrist.npz. 같이 저장되는 scan_pose는 나중에 같은 자세로
파킹했는지 확인하는 용도다.
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent / 'src'))
from lekiwi_pill_pickup.pixel_to_base import (  # noqa: E402
    extrinsic_from_checkerboard, matrix_from_xyz_rpy,
)

from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig  # noqa: E402

JOINTS = ['arm_shoulder_pan', 'arm_shoulder_lift', 'arm_elbow_flex',
          'arm_wrist_flex', 'arm_wrist_roll', 'arm_gripper']
DEFAULT_INTRINSIC = Path(__file__).parent.parent / 'data' / 'calib_wrist' / 'camera_calib.npz'
DEFAULT_OUT = Path(__file__).parent.parent / 'data' / 'extrinsic_wrist.npz'


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument('--lekiwi-host', default='192.168.0.201')
    p.add_argument('--camera', choices=('wrist', 'front'), default='wrist')
    p.add_argument('--cols', type=int, default=9, help='체커보드 내부 코너 가로')
    p.add_argument('--rows', type=int, default=6, help='체커보드 내부 코너 세로')
    p.add_argument('--square-m', type=float, default=0.025, help='칸 한 변(m)')
    p.add_argument('--board-x', type=float, required=True,
                   help='보드 원점의 팔 기준 x(m, 앞쪽이 +)')
    p.add_argument('--board-y', type=float, required=True,
                   help='보드 원점의 팔 기준 y(m, 왼쪽이 +)')
    p.add_argument('--board-z', type=float, required=True,
                   help='보드 평면의 팔 기준 z(m, 위쪽이 +). 책상 높이와 같다')
    p.add_argument('--board-yaw', type=float, default=0.0,
                   help='보드 +X와 팔 +X 사이 각도(rad)')
    # 보드를 책상에 평평하게 놓으면 보드 좌표계 z축이 아래를 향하므로 180도 뒤집어야
    # 한다(참고한 사내 구현의 기본값도 pi). 보드를 벽에 세우는 등 다른 배치면 조정.
    p.add_argument('--board-roll', type=float, default=np.pi,
                   help='보드 roll(rad). 책상에 평평히 놓으면 기본값 pi 그대로')
    p.add_argument('--board-pitch', type=float, default=0.0)
    p.add_argument('--intrinsic', default=str(DEFAULT_INTRINSIC))
    p.add_argument('--out', default=str(DEFAULT_OUT))
    args = p.parse_args()

    calib = np.load(args.intrinsic)
    K, D = calib['camera_matrix'], calib['dist_coeffs']
    print(f'내부 파라미터 로드: fx={K[0, 0]:.1f} fy={K[1, 1]:.1f} '
          f'cx={K[0, 2]:.1f} cy={K[1, 2]:.1f}')

    client = LeKiwiClient(LeKiwiClientConfig(remote_ip=args.lekiwi_host))
    client.connect()
    try:
        obs = client.get_observation()
        scan_pose = np.array([float(obs[f'{j}.pos']) for j in JOINTS])
        print('현재 스캔 자세:', np.round(scan_pose, 1))

        frame = cv2.cvtColor(obs[args.camera], cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        pattern = (args.cols, args.rows)
        found, corners = cv2.findChessboardCorners(gray, pattern, None)
        if not found:
            print(f'체커보드({args.cols}x{args.rows} 내부코너)를 못 찾았어요. '
                  f'보드 전체가 화면에 여백까지 들어오게 놓고 다시 시도하세요.')
            cv2.imwrite('/tmp/extrinsic_fail.jpg', frame)
            print('실패한 프레임 저장: /tmp/extrinsic_fail.jpg')
            sys.exit(1)

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        objp = np.zeros((args.rows * args.cols, 3), np.float32)
        objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square_m

        ok, rvec, tvec = cv2.solvePnP(objp, corners, K, D)
        if not ok:
            print('solvePnP 실패')
            sys.exit(1)

        # 재투영 오차 확인 — 이 값이 크면 보드 인식이나 내부 파라미터가 잘못된 것
        proj, _ = cv2.projectPoints(objp, rvec, tvec, K, D)
        err = float(np.mean(np.linalg.norm(proj.reshape(-1, 2) - corners.reshape(-1, 2), axis=1)))
        print(f'재투영 오차: {err:.3f} px (1.0 미만이면 양호)')

        T_base_board = matrix_from_xyz_rpy(
            [args.board_x, args.board_y, args.board_z],
            [args.board_roll, args.board_pitch, args.board_yaw])
        T_base_camera = extrinsic_from_checkerboard(T_base_board, rvec, tvec)

        cam_pos = T_base_camera[:3, 3]
        print(f'\n카메라 위치(팔 기준): x={cam_pos[0]:.3f} y={cam_pos[1]:.3f} '
              f'z={cam_pos[2]:.3f} m')
        print('이 값이 실제 카메라 위치와 대충 맞는지 눈으로 확인하세요 — '
              '많이 다르면 보드 위치 측정값이나 방향이 틀린 것입니다.')

        out = Path(args.out)
        out.parent.mkdir(parents=True, exist_ok=True)
        np.savez(out, T_base_camera=T_base_camera, scan_pose=scan_pose,
                 table_height_m=args.board_z, reprojection_error_px=err,
                 camera=args.camera)
        print(f'\n저장: {out}')
    finally:
        client.disconnect()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f'실패: {type(e).__name__}: {e}', file=sys.stderr)
        sys.exit(1)
