#!/usr/bin/env python
"""노트북에 직접 연결된 USB 카메라(르키위 원격 카메라 아님)를 색상 검출과 함께
바로 띄워서 확인하는 스크립트. cv2.VideoCapture로 로컬 장치를 직접 연다."""
import argparse
import subprocess
import sys
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).parent))
from color_detect import COLOR_RANGES, find_color_blobs  # noqa: E402

DRAW = {'red': (0, 0, 255), 'yellow': (0, 200, 255),
        'green': (0, 255, 0), 'blue': (255, 0, 0)}

# 자동 화이트밸런스를 켜두면 검은 보드가 조명에 따라 파란색으로 왜곡되어
# 찍히는 문제를 실측으로 확인함(2026-08-25) — 색상검출 전에 반드시 고정할 것.
# 이 카메라의 실제 허용 범위(v4l2-ctl --list-ctrls 확인): 2800~6500, step 10.
WB_TEMPERATURE_MIN = 2800
WB_TEMPERATURE_MAX = 6500
WB_TEMPERATURE_DEFAULT = 4600

# 사용자가 키보드(w/a/s/d, -/=, ,/.)로 실시간 조절해서 확정한 값
# (2026-08-25) — 이 웹캠이 지금 놓인 위치에서 나무 바닥을 빼고 검은 보드만
# 남긴다. 카메라를 옮기면 안 맞으니 --grid로 새 좌표를 읽어 --roi로 갱신할 것.
DEFAULT_ROI = (110, 90, 410, 390)

# 조명이 카메라 광축과 거의 일직선이라 화면 중앙에 정반사 하이라이트가 생겨
# 채도가 날아가는 문제를 실측으로 확인함(2026-08-25) — auto_exposure를 수동으로
# 낮추면 완전히 날아간(흰색) 픽셀이 크게 줄어듦(같은 지점 기준 503->63개).
# 노출을 너무 낮추면 화면 전체가 어두워지므로 8000이 밝기/글레어 절충값.
# 이 카메라 실제 허용 범위(v4l2-ctl --list-ctrls): exposure_time_absolute 0~10000.
EXPOSURE_MIN = 0
EXPOSURE_MAX = 10000
EXPOSURE_DEFAULT = 8000


def fix_white_balance(index: int, temperature: int) -> None:
    dev = f'/dev/video{index}'
    try:
        subprocess.run(
            ['v4l2-ctl', '-d', dev,
             '--set-ctrl=white_balance_automatic=0',
             f'--set-ctrl=white_balance_temperature={temperature}'],
            check=True, capture_output=True, text=True)
        print(f'{dev} 화이트밸런스 고정: {temperature}K')
    except (subprocess.CalledProcessError, FileNotFoundError) as e:
        print(f'화이트밸런스 고정 실패(무시하고 진행): {e}', file=sys.stderr)


def fix_exposure(index: int, exposure: int) -> None:
    dev = f'/dev/video{index}'
    try:
        subprocess.run(
            ['v4l2-ctl', '-d', dev,
             '--set-ctrl=auto_exposure=1',  # 1=Manual Mode
             f'--set-ctrl=exposure_time_absolute={exposure}'],
            check=True, capture_output=True, text=True)
        print(f'{dev} 노출 고정: {exposure} (글레어 완화)')
    except (subprocess.CalledProcessError, FileNotFoundError) as e:
        print(f'노출 고정 실패(무시하고 진행): {e}', file=sys.stderr)


def draw_grid(frame, step: int = 50) -> None:
    """좌표를 눈으로 읽어 --roi 값을 정할 수 있게 격자+숫자를 그린다.

    마우스 드래그(cv2.selectROI/setMouseCallback)가 이 환경(OpenCV 5.0.0 Qt
    백엔드)에서 'NULL window handler'로 매번 크래시해서(2026-08-25 실측)
    클릭 대신 이 격자를 보고 좌표를 직접 --roi로 넘기는 방식을 쓴다."""
    h, w = frame.shape[:2]
    for x in range(0, w, step):
        cv2.line(frame, (x, 0), (x, h), (0, 255, 255), 1)
        cv2.putText(frame, str(x), (x + 2, 15), cv2.FONT_HERSHEY_SIMPLEX,
                    0.35, (0, 255, 255), 1)
    for y in range(0, h, step):
        cv2.line(frame, (0, y), (w, y), (0, 255, 255), 1)
        cv2.putText(frame, str(y), (2, y + 12), cv2.FONT_HERSHEY_SIMPLEX,
                    0.35, (0, 255, 255), 1)


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument('--index', type=int, default=0, help='/dev/videoN 의 N')
    p.add_argument('--colors', nargs='*', default=list(COLOR_RANGES))
    p.add_argument('--min-area', type=int, default=150)
    p.add_argument('--no-color', action='store_true', help='색상검출 없이 원본만 보기')
    p.add_argument('--no-wb-fix', action='store_true', help='화이트밸런스 자동고정 끄기')
    p.add_argument('--wb-temp', type=int, default=WB_TEMPERATURE_DEFAULT,
                   help=f'화이트밸런스 색온도(K), {WB_TEMPERATURE_MIN}~{WB_TEMPERATURE_MAX}. '
                        f'낮을수록 화면이 파랗게, 높을수록 노랗게/따뜻하게 보정됨. '
                        f'실행 중 [ ] 키로도 100 단위로 조절 가능')
    p.add_argument('--roi', type=int, nargs=4, metavar=('X', 'Y', 'W', 'H'),
                   default=list(DEFAULT_ROI),
                   help=f'이 영역 밖은 검사 안 함(바닥 등 배경 제외용). '
                        f'기본값 {DEFAULT_ROI}는 2026-08-25 카메라 위치 기준 — '
                        f'카메라를 옮겼으면 --grid로 새 좌표를 읽어서 갱신할 것')
    p.add_argument('--no-roi', action='store_true', help='영역 지정 없이 화면 전체 사용')
    p.add_argument('--grid', action='store_true',
                   help='격자+좌표 오버레이만 보여줌(--roi 값 정하는 용도, 색상검출 안 함)')
    p.add_argument('--no-exposure-fix', action='store_true',
                   help='노출 수동고정 끄기(자동노출 그대로 사용)')
    p.add_argument('--exposure', type=int, default=EXPOSURE_DEFAULT,
                   help=f'노출값 {EXPOSURE_MIN}~{EXPOSURE_MAX}, 낮을수록 어둡지만 '
                        f'중앙 정반사 글레어가 줄어듦(기본 {EXPOSURE_DEFAULT})')
    args = p.parse_args()

    wb_temp = max(WB_TEMPERATURE_MIN, min(WB_TEMPERATURE_MAX, args.wb_temp))
    if not args.no_wb_fix:
        fix_white_balance(args.index, wb_temp)
    exposure = max(EXPOSURE_MIN, min(EXPOSURE_MAX, args.exposure))
    if not args.no_exposure_fix:
        fix_exposure(args.index, exposure)

    cap = cv2.VideoCapture(args.index)
    if not cap.isOpened():
        print(f'/dev/video{args.index} 를 열 수 없어요', file=sys.stderr)
        sys.exit(1)

    window = f'USB 카메라 video{args.index} (q로 종료)'
    roi = None if args.no_roi else tuple(args.roi)

    print(f'/dev/video{args.index} 열림 — q/ESC로 종료, [ ]로 화이트밸런스, '
          f'9/0으로 노출(글레어) -500/+500 조절')
    if args.grid:
        print('격자 모드: 보드 모서리의 x,y를 읽어서 다음처럼 실행하세요:\n'
              '  ./scripts/run_camera.sh --roi <x> <y> <가로폭> <세로높이>')
    if roi is not None:
        print('노란 박스(ROI) 실시간 조절 — 마우스 대신 키보드로:\n'
              '  w/a/s/d      박스 이동(위/왼쪽/아래/오른쪽), 5px\n'
              '  W/A/S/D(대문자) 같은 방향으로 20px씩 크게 이동\n'
              '  - / =        폭 줄이기/늘리기\n'
              '  , / .        높이 줄이기/늘리기\n'
              '  p            현재 --roi 값을 콘솔에 출력')
    try:
        rx, ry, rw, rh = roi if roi is not None else (0, 0, 0, 0)
        while True:
            ok, frame = cap.read()
            if not ok:
                print('프레임을 못 읽었어요', file=sys.stderr)
                break
            fh, fw = frame.shape[:2]

            if args.grid:
                draw_grid(frame)
                cv2.imshow(window, frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:
                    break
                continue

            if roi is not None:
                cv2.rectangle(frame, (rx, ry), (rx + rw, ry + rh), (0, 255, 255), 1)
                view = frame[ry:ry + rh, rx:rx + rw]
            else:
                view = frame

            if not args.no_color:
                for color in args.colors:
                    for x1, y1, x2, y2, area in find_color_blobs(view, color, args.min_area):
                        cv2.rectangle(view, (x1, y1), (x2, y2), DRAW[color], 2)
                        cv2.putText(view, f'{color} {area}', (x1, max(0, y1 - 6)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, DRAW[color], 2)

            cv2.putText(frame, f'WB {wb_temp}K  EXP {exposure}  ROI {rx},{ry},{rw},{rh}',
                        (5, frame.shape[0] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.imshow(window, frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                break
            elif key == ord('[') and not args.no_wb_fix:
                wb_temp = max(WB_TEMPERATURE_MIN, wb_temp - 100)
                fix_white_balance(args.index, wb_temp)
            elif key == ord(']') and not args.no_wb_fix:
                wb_temp = min(WB_TEMPERATURE_MAX, wb_temp + 100)
                fix_white_balance(args.index, wb_temp)
            elif key == ord('9') and not args.no_exposure_fix:
                exposure = max(EXPOSURE_MIN, exposure - 500)
                fix_exposure(args.index, exposure)
            elif key == ord('0') and not args.no_exposure_fix:
                exposure = min(EXPOSURE_MAX, exposure + 500)
                fix_exposure(args.index, exposure)
            elif roi is not None and key in (ord('w'), ord('a'), ord('s'), ord('d'),
                                              ord('W'), ord('A'), ord('S'), ord('D'),
                                              ord('-'), ord('='), ord(','), ord('.'),
                                              ord('p')):
                step = 20 if chr(key).isupper() else 5
                k = chr(key).lower()
                if k == 'w':
                    ry -= step
                elif k == 's':
                    ry += step
                elif k == 'a':
                    rx -= step
                elif k == 'd':
                    rx += step
                elif key == ord('-'):
                    rw = max(10, rw - 5)
                elif key == ord('='):
                    rw += 5
                elif key == ord(','):
                    rh = max(10, rh - 5)
                elif key == ord('.'):
                    rh += 5
                # 프레임 밖으로 나가지 않게 고정
                rx = max(0, min(rx, fw - 10))
                ry = max(0, min(ry, fh - 10))
                rw = min(rw, fw - rx)
                rh = min(rh, fh - ry)
                print(f'ROI: --roi {rx} {ry} {rw} {rh}')
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f'실패: {type(e).__name__}: {e}', file=sys.stderr)
        sys.exit(1)
