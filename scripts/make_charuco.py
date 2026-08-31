#!/usr/bin/env python
"""인쇄용 ChArUco 보드 이미지를 만든다.

일반 체커보드와 달리 흰 칸마다 ArUco 마커가 들어 있어서, 보드 일부가 잘리거나
가려져도 인식된다(실측으로 반복해서 겪은 문제 — 44장 중 6장이 잘림/가림으로 실패).

인쇄 후 반드시 자로 실제 칸 크기를 재서 캘리브레이션 때 그 값을 넣을 것 —
프린터가 배율을 조정해버리면 설계값과 달라진다."""
import argparse
import sys

import cv2
import numpy as np

# A4 300dpi 기준 픽셀 크기
A4_300DPI = (2480, 3508)
MM_PER_INCH = 25.4


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument('--out', default='/tmp/charuco_a4.png')
    p.add_argument('--cols', type=int, default=7, help='가로 칸 수')
    p.add_argument('--rows', type=int, default=10, help='세로 칸 수')
    p.add_argument('--square-mm', type=float, default=25.0, help='한 칸 크기(mm)')
    p.add_argument('--marker-ratio', type=float, default=0.75,
                   help='칸 대비 마커 크기 비율')
    p.add_argument('--dpi', type=int, default=300)
    p.add_argument('--margin-mm', type=float, default=10.0, help='흰 여백(mm)')
    args = p.parse_args()

    px_per_mm = args.dpi / MM_PER_INCH
    square_px = int(round(args.square_mm * px_per_mm))
    marker_mm = args.square_mm * args.marker_ratio
    margin_px = int(round(args.margin_mm * px_per_mm))

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    board = cv2.aruco.CharucoBoard(
        (args.cols, args.rows), args.square_mm / 1000.0, marker_mm / 1000.0, aruco_dict)

    board_px = (args.cols * square_px, args.rows * square_px)
    img = board.generateImage(board_px, marginSize=0, borderBits=1)

    # 흰 여백 추가 — 여백이 없으면 코너 검출이 불안정해진다
    canvas = np.full((board_px[1] + 2 * margin_px, board_px[0] + 2 * margin_px),
                     255, dtype=np.uint8)
    canvas[margin_px:margin_px + board_px[1], margin_px:margin_px + board_px[0]] = img

    cv2.imwrite(args.out, canvas)

    w_mm = canvas.shape[1] / px_per_mm
    h_mm = canvas.shape[0] / px_per_mm
    print(f'저장: {args.out}')
    print(f'  보드: {args.cols}x{args.rows} 칸, 한 칸 {args.square_mm}mm, '
          f'마커 {marker_mm:.1f}mm')
    print(f'  내부 코너: {args.cols - 1}x{args.rows - 1}개')
    print(f'  전체 크기: {w_mm:.0f} x {h_mm:.0f} mm '
          f'(A4는 210x297mm — 넘으면 --square-mm을 줄이세요)')
    print(f'  사전: DICT_4X4_250')
    print()
    print('⚠️ 인쇄할 때 "실제 크기 / 100% / 배율 조정 안 함"으로 출력하고,')
    print('   인쇄된 칸을 자로 재서 실제 크기를 확인하세요.')

    if w_mm > 210 or h_mm > 297:
        print(f'\n❌ A4를 넘습니다 — --square-mm 값을 줄여주세요', file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f'실패: {type(e).__name__}: {e}', file=sys.stderr)
        sys.exit(1)
