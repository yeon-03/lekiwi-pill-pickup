"""YOLO 없이 색상만으로 병을 찾는다(HSV 임계값 + 윤곽 검출).

배경(흰 책상)과 색이 뚜렷이 구분되는 상황을 전제로 한다. YOLO가 "이게 병인가"를
따지다 실패하는 문제(작은 점안액 병을 vase로 분류 등)를 우회하는 목적."""
import sys

import cv2
import numpy as np

# HSV 색상 범위 — 빨강은 Hue가 0 근처에서 wrap되므로 두 구간으로 나눔.
# 실측 Hue(2026-08-24): 빨간병 1~5 / 초록병 77~83 / 파란병 109~114 / 보라 그리퍼 127~130.
# 파랑 상한을 120으로 잘라 로봇 자신의 보라색 그리퍼가 파랑으로 안 잡히게 한다.
# 흰색(2026-08-25 실측): 채도(S)만으론 어두운 보드(S~20)와 안 갈라져서
# 명도(V)를 높게 잡아 구분(흰병 V 132~255 vs 검은보드 V~59).
COLOR_RANGES = {
    'red': [((0, 80, 60), (10, 255, 255)), ((170, 80, 60), (180, 255, 255))],
    'yellow': [((20, 80, 80), (33, 255, 255))],
    'green': [((35, 60, 40), (90, 255, 255))],
    'blue': [((95, 80, 40), (120, 255, 255))],
    'white': [((0, 0, 180), (179, 40, 255))],
}
MIN_AREA_PX = 150      # 이보다 작은 덩어리는 노이즈로 무시
MAX_AREA_PX = 25000    # 이보다 크면 배경(보드 등)이 색조 오염된 것으로 보고 무시
MIN_FILL_RATIO = 0.25  # 바운딩박스 대비 실제 색 픽셀 비율(선/그림자 걸러냄)


def find_color_blobs(frame_bgr: np.ndarray, color: str,
                     min_area: int = MIN_AREA_PX,
                     max_area: int = MAX_AREA_PX) -> list[tuple[int, int, int, int, int]]:
    """(x1, y1, x2, y2, area) 목록을 면적 큰 순으로 반환.

    max_area: 배경(검은 보드 등)이 조명 때문에 물체와 같은 색조로 찍혀 화면
    대부분을 뒤덮는 경우가 있어(2026-08-25 실측, 보드 전체가 파란색으로 오탐),
    실제 물체 크기를 한참 벗어나는 거대한 덩어리는 배경으로 보고 제외한다."""
    if color not in COLOR_RANGES:
        raise ValueError(f'모르는 색: {color}')
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

    mask = None
    for lo, hi in COLOR_RANGES[color]:
        m = cv2.inRange(hsv, np.array(lo), np.array(hi))
        mask = m if mask is None else cv2.bitwise_or(mask, m)

    # 잡음 제거 + 구멍 메우기(반사광으로 물체 내부에 생기는 빈틈 — 커널을
    # 크게 잡아야 넓은 하이라이트도 다리를 놔서 하나의 덩어리로 붙는다)
    open_kernel = np.ones((5, 5), np.uint8)
    close_kernel = np.ones((15, 15), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    results = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area or area > max_area:
            continue
        x, y, w, h = cv2.boundingRect(c)
        if area / (w * h) < MIN_FILL_RATIO:
            continue
        results.append((x, y, x + w, y + h, int(area)))
    results.sort(key=lambda r: r[4], reverse=True)
    return results


def main() -> None:
    if len(sys.argv) < 2:
        print(f'사용법: {sys.argv[0]} <이미지경로> [색상...]')
        sys.exit(1)
    path = sys.argv[1]
    colors = sys.argv[2:] or list(COLOR_RANGES)

    img = cv2.imread(path)
    if img is None:
        print(f'이미지를 못 읽음: {path}')
        sys.exit(1)

    annotated = img.copy()
    draw_color = {'red': (0, 0, 255), 'yellow': (0, 200, 255),
                  'green': (0, 255, 0), 'blue': (255, 0, 0), 'white': (200, 200, 200)}
    for color in colors:
        blobs = find_color_blobs(img, color)
        print(f'{color}: {len(blobs)}개')
        for x1, y1, x2, y2, area in blobs:
            print(f'  box=({x1},{y1},{x2},{y2}) 크기={x2 - x1}x{y2 - y1} 면적={area}')
            cv2.rectangle(annotated, (x1, y1), (x2, y2), draw_color[color], 2)
            cv2.putText(annotated, color, (x1, max(0, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, draw_color[color], 2)

    out = path.rsplit('.', 1)[0] + '_color.jpg'
    cv2.imwrite(out, annotated)
    print(f'저장: {out}')


if __name__ == '__main__':
    main()
