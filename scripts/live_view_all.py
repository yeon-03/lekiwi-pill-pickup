#!/usr/bin/env python
"""고정캠 + 베이스캠(front) + 손목캠(wrist, 있으면)을 한 창 안에 좌우로
나란히 이어붙여서 항상 띄워두는 뷰어 — 카메라 장치나 로봇 연결을 이
스크립트가 직접 여는 게 아니라, fixed_cam_server.py / robot_bridge.py가
이미 써둔 공유 프레임 파일만 읽는다.

그래서 이 창은 정렬(align_and_grasp.py)이든 이동(base_nudge.py 등)이든
다른 어떤 스크립트가 돌고 있어도 상관없이 항상 켜둘 수 있다 — 실행/종료
순서를 신경 안 써도 됨.

각 카메라를 창 여러 개(cv2.imshow 여러 번)로 따로 띄우면 창 관리자가 매번
같은 기본 위치에 겹쳐 띄워서 하나만 보이는 것처럼 보이는 문제가 있었다
(2026-08-26 실측 발견) — 창을 여러 개 만들고 moveWindow로 흩어놓는 대신,
아예 프레임들을 옆으로 이어붙여(hstack) 창 하나로만 보여주는 방식으로
바꿔서 이 문제 자체를 없앤다(사용자 요청, 2026-08-26).

align_and_grasp.py가 실제로 보는 것과 같은 것(색상 블롭 박스 + 흰 스티커
십자선)도 같이 그려서, 이 창만 보고도 정렬이 잘 될지 눈으로 가늠할 수
있게 한다(live_view_usb_local.py에 있던 오버레이를 옮겨옴).

사전조건(터미널 2개에 따로 띄워둘 것):
  python scripts/fixed_cam_server.py
  python scripts/robot_bridge.py --lekiwi-host 192.168.0.201
둘 중 하나가 아직 안 떠 있어도 에러 없이 "대기중" 화면만 나오고, 나중에
띄우면 자동으로 그 화면부터 나오기 시작한다."""
import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from color_detect import (find_board_bbox,  # noqa: E402
                          find_bottles_on_board, find_color_blobs,
                          find_white_marker)
from approach_board import approach_state as _approach_state  # noqa: E402

FRAME_DIR = Path('/dev/shm/lekiwi_cam')
STALE_SEC = 2.0
PANEL_HEIGHT = 360  # 기본값. --panel-height / --fullscreen으로 키울 수 있다

# 노랑은 안 쓴다(2026-08-27) — 빨강/초록/파랑 3개만 그린다
DRAW = {'red': (0, 0, 255),
        'green': (0, 255, 0), 'blue': (255, 0, 0)}


def _read_shared_frame(name: str):
    path = FRAME_DIR / f'{name}.jpg'
    try:
        if (time.time() - path.stat().st_mtime) > STALE_SEC:
            return None
    except OSError:
        return None
    return cv2.imread(str(path))


def _placeholder(text: str, w: int = 480, h: int = PANEL_HEIGHT):  # noqa: B008
    img = np.zeros((h, w, 3), dtype='uint8')
    cv2.putText(img, text, (10, h // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
    return img


def _annotate(frame, label: str, with_marker_crosshair: bool):
    """검출 결과를 그린다.

    ■ 베이스캠은 '제어가 실제로 채택한 박스'를 굵게, 버려진 후보는 가늘게
      그린다(2026-08-27). 예전엔 원시 후보를 전부 같은 굵기로 그려서, 배경
      반사 같은 버려진 후보까지 병처럼 보였고 "오탐이 너무 많다"는 오해가
      생겼다. 실제 제어는 approach_board.approach_state() 가 고른 3개만 쓴다.
    """
    vis = frame.copy()
    board = None if label.startswith('베이스캠') else find_board_bbox(vis)
    chosen = []
    if label.startswith('베이스캠'):
        try:
            st = _approach_state()
            if st:
                # ⚠️ 좌표 정확일치로 맞추면 안 된다 — approach_state()는 파일을
                #    다시 읽으므로 1px 차이가 난다
                #    (실측: 제어 432~495 vs 원시 432~494).
                chosen = [(x1, x2) for x1, x2, _c in st['boxes']]
        except Exception:
            chosen = []
    for color, bgr in DRAW.items():
        # 고정캠은 판 바깥의 나무 바닥이 빨강으로 잡히므로 '판 위'만 그린다.
        # 베이스캠은 approach_state() 가 이미 고른 것을 아래에서 굵게 표시한다.
        if label.startswith('베이스캠'):
            blobs = find_color_blobs(vis, color, min_area=150, profile='base')
        else:
            blobs = find_bottles_on_board(vis, color, board=board)
        for x1, y1, x2, y2, area in blobs:
            picked = any(abs(cx1 - x1) <= 6 and abs(cx2 - x2) <= 6
                         for cx1, cx2 in chosen)
            if chosen and not picked:
                # 버려진 후보 — 가는 회색 점선 느낌으로만 표시
                cv2.rectangle(vis, (x1, y1), (x2, y2), (110, 110, 110), 1)
                continue
            cv2.rectangle(vis, (x1, y1), (x2, y2), bgr, 3 if picked else 2)
            cv2.putText(vis, f'{color} {area}', (x1, max(0, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, bgr, 2)
    if with_marker_crosshair:
        marker = find_white_marker(frame)
        if marker is not None:
            cx, cy = int(marker[0]), int(marker[1])
            h, w = vis.shape[:2]
            cv2.line(vis, (0, cy), (w, cy), (0, 255, 255), 1)
            cv2.line(vis, (cx, 0), (cx, h), (0, 255, 255), 1)
            cv2.circle(vis, (cx, cy), 12, (0, 255, 255), 2)
    cv2.putText(vis, label, (5, vis.shape[0] - 8), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (255, 255, 255), 2)
    return vis


def _resize_to_height(img, height: int):
    h, w = img.shape[:2]
    if h == height:
        return img
    scale = height / h
    return cv2.resize(img, (max(1, int(w * scale)), height))


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument('--panel-height', type=int, default=PANEL_HEIGHT,
                   help='패널 하나의 높이(px). 크게 줄수록 창이 커진다')
    p.add_argument('--fullscreen', action='store_true', help='전체화면으로 띄운다')
    args = p.parse_args()
    panel_height = args.panel_height

    window = '상시 뷰어 (고정캠 | 베이스캠 | 손목캠) — q/ESC로 종료'
    print(f'{window} 시작 '
          '(fixed_cam_server.py / robot_bridge.py가 안 떠 있으면 "대기중" 화면만 나옴)')
    # NORMAL이어야 창 크기를 마우스로 조절할 수 있다(기본 AUTOSIZE는 고정)
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    if args.fullscreen:
        cv2.setWindowProperty(window, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    try:
        while True:
            fixed = _read_shared_frame('fixed')
            fixed_panel = (_annotate(fixed, '고정캠', True) if fixed is not None
                           else _placeholder('고정캠 대기중 (fixed_cam_server.py?)'))

            front = _read_shared_frame('front')
            front_panel = (_annotate(front, '베이스캠', False) if front is not None
                           else _placeholder('베이스캠 대기중 (robot_bridge.py?)'))

            panels = [fixed_panel, front_panel]

            # 지금은 손목캠을 안 쓰기로 함(2026-08-26) — 파일이 실제로 있을
            # 때만 세 번째 패널로 붙는다. 나중에 다시 달면 자동으로 붙음.
            wrist = _read_shared_frame('wrist')
            if wrist is not None:
                panels.append(_annotate(wrist, '손목캠', False))

            combined = np.hstack([_resize_to_height(p, panel_height) for p in panels])
            cv2.imshow(window, combined)

            key = cv2.waitKey(30) & 0xFF
            if key == ord('q') or key == 27:
                break
    finally:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
