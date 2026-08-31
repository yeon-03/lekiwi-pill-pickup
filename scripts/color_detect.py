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
# ■ 카메라마다 조명이 완전히 달라서 임계값을 하나로 못 쓴다 (2026-08-27 실측)
#     같은 병인데   고정캠(밝음)   베이스캠(어두움)
#       초록 채도       107            178
#       파랑 채도       136            213
#   베이스캠 쪽을 조이면 고정캠 인식률이 떨어지고, 고정캠에 맞추면 베이스캠이
#   검은 판·나무 바닥을 병으로 잡는다. 그래서 프로파일을 나눈다.
#   기본값은 'fixed' — 기존 코드(align_and_grasp 등)는 전부 고정캠을 쓴다.
COLOR_RANGES = {
    # 고정캠: 판을 정면에서 밝게 본다. 병이 밝고 채도가 낮게 찍힌다.
    'red': [((0, 80, 60), (10, 255, 255)), ((170, 80, 60), (180, 255, 255))],
    'yellow': [((20, 80, 80), (33, 255, 255))],
    # '초록' 병은 실제로는 청록(teal)이다 — 상한을 90 에 두면 파랑 하한과의
    # 틈(90~95)에 빠져 검출이 깜빡인다. 98 로 올리고 파랑 하한을 99 로 민다.
    'green': [((35, 60, 40), (98, 255, 255))],
    # 파랑 상한을 120 으로 잘라 로봇 자신의 보라색 그리퍼(H 127~131)가
    # 파랑으로 안 잡히게 한다.
    'blue': [((99, 80, 40), (120, 255, 255))],
    # 그리퍼 자체(정렬 기준점으로 실시간 추적할 때 사용, 2026-08-25 실측 H=131.8)
    'purple': [((121, 80, 30), (145, 255, 255))],
    # 흰색: 조명이 어두워 실제로는 회색빛(2026-08-25 실측 손목 흰스티커
    # S=70 V=154, 주변 검은 모터 V=9) — 채도 상한을 넉넉히, 명도 하한을
    # 낮춰야 잡힌다. 검은 배경과는 명도(V)로 확실히 갈린다.
    'white': [((0, 0, 100), (179, 110, 255))],
}
# 베이스캠: 어두운 판을 옆에서 본다. 병은 채도가 매우 높게, 배경(검은 판·나무
# 바닥)은 낮게 찍힌다 — 채도 하한을 높여야 배경을 병으로 안 잡는다.
#   실측: 파란병 S=249 / 검은판 S=145 / 청바지 S=111
#         초록병 S=178 / 판의 청록 반사 S=80~122
#         살구색 병 S=81 / 나무 바닥 S=46~56
BASE_COLOR_RANGES = dict(COLOR_RANGES)
BASE_COLOR_RANGES.update({
    'red': [((0, 65, 60), (10, 255, 255)), ((170, 65, 60), (180, 255, 255))],
    'green': [((35, 125, 40), (98, 255, 255))],
    'blue': [((99, 170, 90), (120, 255, 255))],
})
PROFILES = {'fixed': COLOR_RANGES, 'base': BASE_COLOR_RANGES}

MIN_AREA_PX = 150      # 이보다 작은 덩어리는 노이즈로 무시
MAX_AREA_PX = 25000    # 이보다 크면 배경(보드 등)이 색조 오염된 것으로 보고 무시
MIN_FILL_RATIO = 0.25  # 바운딩박스 대비 실제 색 픽셀 비율(선/그림자 걸러냄)


def find_color_blobs(frame_bgr: np.ndarray, color: str,
                     min_area: int = MIN_AREA_PX,
                     max_area: int = MAX_AREA_PX,
                     profile: str = 'fixed') -> list[tuple[int, int, int, int, int]]:
    """(x1, y1, x2, y2, area) 목록을 면적 큰 순으로 반환.

    max_area: 배경(검은 보드 등)이 조명 때문에 물체와 같은 색조로 찍혀 화면
    대부분을 뒤덮는 경우가 있어(2026-08-25 실측, 보드 전체가 파란색으로 오탐),
    실제 물체 크기를 한참 벗어나는 거대한 덩어리는 배경으로 보고 제외한다."""
    ranges = PROFILES.get(profile)
    if ranges is None:
        raise ValueError(f'모르는 프로파일: {profile} (fixed/base)')
    if color not in ranges:
        raise ValueError(f'모르는 색: {color}')
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

    mask = None
    for lo, hi in ranges[color]:
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


def find_white_marker(frame_bgr: np.ndarray,
                      min_area: int = 150, max_area: int = 900,
                      min_circularity: float = 0.6,
                      max_surround_v: int = 100) -> tuple[float, float] | None:
    """손목 모터에 붙인 흰 원형 스티커의 중심 좌표를 반환한다.

    그냥 흰색으로만 찾으면 병뚜껑/나무바닥/케이블까지 10개 넘게 잡히므로
    (2026-08-25 실측), 이 스티커만의 특징 두 가지로 좁힌다:
      1) 원형에 가깝다 (bounding box가 정사각형에 가깝고 채워짐 비율이 높음)
      2) 검은 모터 위에 붙어있다 (주변 픽셀의 명도 V가 매우 낮음)
    """
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    lo, hi = COLOR_RANGES['white'][0]
    mask = cv2.inRange(hsv, np.array(lo), np.array(hi))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # 약병 표면의 정반사 하이라이트가 흰 스티커로 오인되는 사고가 실제로
    # 있었다(2026-08-26): 빨간병 위 하이라이트를 마커로 잡는 바람에, 로봇이
    # 움직여도 그 "마커"가 안 움직여서 방향보정 행렬이 통째로 엉터리로
    # 측정됐고 정렬이 반대 방향으로 폭주했다. 병 위치는 어차피 색으로 이미
    # 알 수 있으니, 병 영역 안의 후보는 마커에서 제외한다.
    forbidden = []
    for bottle_color in ('red', 'yellow', 'green', 'blue'):
        for bx1, by1, bx2, by2, _ in find_color_blobs(frame_bgr, bottle_color):
            forbidden.append((bx1, by1, bx2, by2))

    h_img, w_img = mask.shape
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area or area > max_area:
            continue
        x, y, w, h = cv2.boundingRect(c)
        aspect = w / h if h else 0
        if not (0.6 < aspect < 1.6):          # 원형이면 가로세로 비슷
            continue
        if area / (w * h) < min_circularity:  # 원이 사각형을 채우는 비율
            continue

        cx, cy = x + w / 2, y + h / 2
        if any(bx1 <= cx <= bx2 and by1 <= cy <= by2
               for bx1, by1, bx2, by2 in forbidden):
            continue  # 병 위 하이라이트 — 마커 아님

        # 주변 링(박스를 넓힌 영역)의 명도 — 검은 모터 위면 매우 어둡다
        pad = max(w, h)
        rx1, ry1 = max(0, x - pad), max(0, y - pad)
        rx2, ry2 = min(w_img, x + w + pad), min(h_img, y + h + pad)
        ring = hsv[ry1:ry2, rx1:rx2, 2].astype(float).copy()
        ring[y - ry1:y - ry1 + h, x - rx1:x - rx1 + w] = np.nan  # 스티커 자신 제외
        surround_v = np.nanmean(ring)
        # 링 전체가 스티커 박스로 덮여 전부 NaN이면 nanmean도 NaN이 되는데,
        # `NaN > max_surround_v`는 False라서 필터가 조용히 무력화된다(독립
        # 검증에서 발견, 2026-08-25) — NaN을 명시적으로 걸러낸다.
        if np.isnan(surround_v) or surround_v > max_surround_v:
            continue

        score = area
        if best is None or score > best[0]:
            best = (score, x + w / 2, y + h / 2)

    return (best[1], best[2]) if best else None


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
                  'green': (0, 255, 0), 'blue': (255, 0, 0), 'white': (200, 200, 200),
                  'purple': (255, 0, 255)}
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


# ---------------------------------------------------------------------------
# 고정캠 전용: '판 위의 병'만 남기는 필터
#
# ■ 왜 필요한가 (2026-08-27 실측)
#   고정캠은 판을 위에서 내려다보는데 판 바깥은 나무 바닥이다. 나무는 따뜻한
#   색이라 빨강으로 잡히고, 채도로는 안 갈린다 — 진짜 살구색 병이 S=124 인데
#   나무 오탐 중에 S=153, S=174 도 있었다. 게다가 **가장 큰 빨강 덩어리가
#   나무(면적 10708)라 진짜 병(3551)보다 커서**, 빨간 병을 집으라고 하면
#   엉뚱한 바닥을 겨냥하게 된다(화면 노이즈가 아니라 실제 오작동).
#
#   색으로 못 가르니 위치로 가른다. 병은 항상 **판 위**에 있고, 판은 어둡고
#   채도가 낮은 가장 큰 덩어리다. 그리고 병은 화면 가장자리에 닿지 않는다
#   (닿는 건 잘려나간 배경이다).
BOARD_V_MAX = 120        # 판은 이보다 어둡다
BOARD_S_MAX = 110        # 판은 이보다 채도가 낮다(무채색 계열)
BOARD_MIN_AREA_RATIO = 0.10   # 화면의 이만큼은 돼야 '판'으로 인정


def find_board_bbox(frame_bgr: np.ndarray) -> tuple[int, int, int, int] | None:
    """판(어둡고 채도 낮은 가장 큰 영역)의 바운딩박스. 못 찾으면 None."""
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    mask = ((hsv[:, :, 2] < BOARD_V_MAX)
            & (hsv[:, :, 1] < BOARD_S_MAX)).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((25, 25), np.uint8))
    n, _lab, stats, _c = cv2.connectedComponentsWithStats(mask, 8)
    if n < 2:
        return None
    i = max(range(1, n), key=lambda k: stats[k, cv2.CC_STAT_AREA])
    h, w = mask.shape
    if stats[i, cv2.CC_STAT_AREA] < w * h * BOARD_MIN_AREA_RATIO:
        return None
    x, y, bw, bh = (stats[i, cv2.CC_STAT_LEFT], stats[i, cv2.CC_STAT_TOP],
                    stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT])
    return (int(x), int(y), int(x + bw), int(y + bh))


def find_bottles_on_board(frame_bgr: np.ndarray, color: str,
                          min_area: int = 800,
                          profile: str = 'fixed',
                          board: tuple[int, int, int, int] | None = None,
                          ) -> list[tuple[int, int, int, int, int]]:
    """판 위에 있는 병만 반환. 판을 못 찾으면 테두리 조건만 적용한다."""
    h, w = frame_bgr.shape[:2]
    if board is None:
        board = find_board_bbox(frame_bgr)
    # 1순위: 밝기 기반(색이 날아가도 잡힌다, find_bottles_bright 주석 참고).
    bright = find_bottles_bright(frame_bgr, board=board)
    if color in bright:
        return [bright[color]]
    # 2순위: 밝기 기반이 못 찾으면 예전 색 임계값 방식으로 폴백한다.
    out = []
    for b in find_color_blobs(frame_bgr, color, min_area=min_area,
                              profile=profile):
        x1, y1, x2, y2, _a = b
        if x1 <= 1 or y1 <= 1 or x2 >= w - 2 or y2 >= h - 2:
            continue                      # 화면 밖으로 잘린 배경
        if board is not None:
            cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
            if not (board[0] <= cx <= board[2] and board[1] <= cy <= board[3]):
                continue                  # 판 밖 = 병이 아니다
        out.append(b)
    return out


# ---------------------------------------------------------------------------
# 고정캠 전용 2: '판 위의 밝은 덩어리를 찾아 색조로 분류'
#
# ■ 왜 색별 임계값으로는 안 되는가 (2026-08-27 실측)
#   빨간 병이 조명에 날아가서 거의 흰색으로 찍힌다. 실측값이 H=28(주황빛),
#   S=44 로, 빨강 범위(H<=10, S>=80)에 전혀 안 걸린다. 카메라 노출을 낮춰
#   해결하려 했으나 이 카메라는 수동 노출이 작동하지 않고(어떤 값을 줘도
#   화면이 검게 나옴), 자동 노출이 brightness 조정을 계속 상쇄해 값이 들쭉날쭉했다.
#
#   그래서 색을 먼저 정하지 않는다. 판은 검고 그 위에서 밝은 건 병뿐이므로,
#   **밝은 덩어리를 먼저 찾고 그다음 색조로 이름을 붙인다**. 날아간 병도
#   색조(H)는 남아 있어서 이 순서면 잡힌다.
#   실측: 폭 54/48/51px 로 셋이 고르게, H=28(빨강) / 77(초록) / 114(파랑).
BRIGHT_V_MIN = 140       # 판 위에서 이보다 밝으면 병 후보
BRIGHT_MIN_AREA = 600
BRIGHT_MIN_S = 30        # 완전 무채색(흰 마커·반사)은 제외
BRIGHT_WIDTH_OUTLIER = 2.5
# 색조 -> 이름. 빨간 병은 날아가면 주황(H~28)까지 밀리므로 상한을 넉넉히.
HUE_CLASSES = (('red', 0, 34), ('green', 35, 98), ('blue', 99, 125))


def _classify_hue(h: int) -> str | None:
    if h >= 168:
        return 'red'                       # 빨강은 0 근처에서 wrap
    for name, lo, hi in HUE_CLASSES:
        if lo <= h <= hi:
            return name
    return None


def find_bottles_bright(frame_bgr: np.ndarray,
                        board: tuple[int, int, int, int] | None = None,
                        ) -> dict[str, tuple[int, int, int, int, int]]:
    """판 위 밝은 덩어리를 색조로 분류해 {색: (x1,y1,x2,y2,area)} 로 반환."""
    if board is None:
        board = find_board_bbox(frame_bgr)
    if board is None:
        return {}
    bx1, by1, bx2, by2 = board
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    sub = hsv[by1:by2, bx1:bx2]
    mask = ((sub[:, :, 2] > BRIGHT_V_MIN)
            & (sub[:, :, 1] >= BRIGHT_MIN_S)).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, _lab, stats, _c = cv2.connectedComponentsWithStats(mask, 8)

    cands = []
    sh, sw = mask.shape
    for k in range(1, n):
        x, y, w, h, a = (stats[k, cv2.CC_STAT_LEFT], stats[k, cv2.CC_STAT_TOP],
                         stats[k, cv2.CC_STAT_WIDTH], stats[k, cv2.CC_STAT_HEIGHT],
                         stats[k, cv2.CC_STAT_AREA])
        if a < BRIGHT_MIN_AREA:
            continue
        # 판 경계에 닿은 건 판 바깥(나무 바닥 등)이 새어들어온 것이다
        if x <= 2 or y <= 2 or x + w >= sw - 2 or y + h >= sh - 2:
            continue
        cands.append((x, y, w, h, a))
    if not cands:
        return {}
    # 병 3개는 크기가 비슷하다 — 유난히 넓은 건 배경이 붙은 것
    widths = sorted(c[2] for c in cands)
    med_w = widths[len(widths) // 2]
    cands = [c for c in cands if c[2] <= med_w * BRIGHT_WIDTH_OUTLIER]

    out: dict[str, tuple[int, int, int, int, int]] = {}
    for x, y, w, h, a in cands:
        roi = sub[y:y + h, x:x + w].reshape(-1, 3)
        roi = roi[roi[:, 2] > BRIGHT_V_MIN]
        if len(roi) == 0:
            continue
        name = _classify_hue(int(np.median(roi[:, 0])))
        if name is None:
            continue
        box = (int(x + bx1), int(y + by1), int(x + w + bx1), int(y + h + by1),
               int(a))
        if name not in out or a > out[name][4]:
            out[name] = box
    return out
