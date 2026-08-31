#!/usr/bin/env python
"""고정캠 시차보정 계수를 구하는 도구 (parallax.py 참고).

두 가지 방법 중 편한 쪽을 쓰면 된다.

■ 방법 1 — 자로 두 번 재기 (로봇 없이 30초, 권장 시작점)
    python scripts/calibrate_parallax.py from-distance \
        --camera-to-board 80 --gripper-offset 14
  camera-to-board : 카메라 렌즈 ~ 보드 표면 거리
  gripper-offset  : 보드 표면 ~ 손목 흰 스티커 거리 (파지 자세에서)
  단위는 아무거나(cm/mm) 상관없고 둘만 같으면 된다.

■ 방법 2 — 실제 성공 위치를 기록해서 맞추기 (더 정확, 로봇 필요)
  좌/우 등 서로 다른 위치의 병에 대해, 집게가 **실제로 제대로 맞은 순간**마다:
    python scripts/calibrate_parallax.py record --color blue --note "왼쪽, 수동정렬"
  2개 이상(가급적 좌·중·우 3개) 모은 뒤:
    python scripts/calibrate_parallax.py fit
  이 방법은 마커~집게끝 고정 오프셋까지 같이 흡수하므로 물리량을 몰라도 된다.

현재 상태 확인:
    python scripts/calibrate_parallax.py show
"""
import argparse
import json
import sys
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).parent))
import parallax  # noqa: E402
from align_and_grasp import TARGET_ROI, blob_center  # noqa: E402
from color_detect import find_white_marker  # noqa: E402
from fixed_cam_server import read_fixed_frame  # noqa: E402


def _get_frame(image_path: str | None):
    if image_path:
        frame = cv2.imread(image_path)
        if frame is None:
            raise SystemExit(f'이미지를 못 읽음: {image_path}')
        return frame
    frame = read_fixed_frame()
    if frame is None:
        raise SystemExit('고정캠 프레임이 없음 — scripts/fixed_cam_server.py 를 먼저 띄우세요 '
                         '(또는 --image 로 저장된 사진 지정)')
    return frame


def _bottle_point(frame, color):
    """align_and_grasp 와 완전히 같은 기준점을 쓴다(정의가 갈리면 보정이 무의미)."""
    rx, ry, rw, rh = TARGET_ROI
    point = blob_center(frame[ry:ry + rh, rx:rx + rw], color)
    if point is None:
        return None
    return (point[0] + rx, point[1] + ry)


def _load_samples() -> list[dict]:
    try:
        return json.loads(parallax.SAMPLES_PATH.read_text())
    except (OSError, ValueError):
        return []


def _save_samples(samples: list[dict]) -> None:
    parallax.SAMPLES_PATH.parent.mkdir(parents=True, exist_ok=True)
    parallax.SAMPLES_PATH.write_text(
        json.dumps(samples, indent=2, ensure_ascii=False) + '\n')


def cmd_record(args) -> None:
    """⚠️ 팔을 **내리기 전에** 실행할 것.

    "제대로 맞았다"의 진짜 판정 기준은 눈이 아니라 "실제로 집혔는가"인데,
    집는 시점에는 이미 팔이 내려가 마커 위치가 달라져 있다. 그래서 순서는
    반드시 [정렬/수동보정 -> record -> 내려서 집어보기 -> 실패하면 undo] 다.
    """
    frame = _get_frame(args.image)
    if args.bottle_x is not None and args.bottle_y is not None:
        bottle = (args.bottle_x, args.bottle_y)
    else:
        bottle = _bottle_point(frame, args.color)
        if bottle is None:
            raise SystemExit(f'{args.color} 병을 못 찾음 — 화면과 색상을 확인하거나 '
                             '--bottle-x/--bottle-y 로 직접 넣으세요')
    if args.marker_x is not None and args.marker_y is not None:
        marker = (args.marker_x, args.marker_y)
    else:
        marker = find_white_marker(frame)
        if marker is None:
            raise SystemExit('흰 스티커를 못 찾음 — 팔이 화면에 보이는 자세인지 '
                             '확인하거나 --marker-x/--marker-y 로 직접 넣으세요')

    samples = _load_samples()
    samples.append({'color': args.color, 'note': args.note,
                    'bottle': [round(bottle[0], 1), round(bottle[1], 1)],
                    'marker': [round(marker[0], 1), round(marker[1], 1)]})
    _save_samples(samples)
    print(f'기록: 병=({bottle[0]:.0f},{bottle[1]:.0f}) '
          f'마커=({marker[0]:.0f},{marker[1]:.0f}) '
          f'차이=({marker[0] - bottle[0]:+.0f},{marker[1] - bottle[1]:+.0f})')
    print(f'총 {len(samples)}개 샘플 -> {parallax.SAMPLES_PATH}')
    print('※ 이제 내려서 실제로 집어보세요. 실패하면 이 샘플은 잘못된 것이니 '
          'calibrate_parallax.py undo 로 취소하세요.')
    if len(samples) < 2:
        print(f'※ 서로 다른 위치로 2개 이상(가급적 좌·중·우 3개) 모은 뒤 fit 실행')


def cmd_fit(args) -> None:
    samples = _load_samples()
    if not samples:
        raise SystemExit(f'샘플이 없음 — 먼저 record 로 모으세요 ({parallax.SAMPLES_PATH})')
    params, notes = parallax.fit(samples)
    for note in notes:
        print(note)
    if parallax.is_identity(params):
        print('\n=> 두 축 다 못 맞췄습니다. 저장하지 않습니다.')
        return
    path = parallax.save(params)
    print(f'\n{parallax.describe(params)}')
    print(f'저장: {path}')


def _find_anchor(frame, args):
    """보정을 0으로 둘 기준점 = "지금 보정 없이도 잘 잡히는 병"의 좌표.

    지정이 없으면 화면중심에 가장 가까운 병을 자동으로 고른다 — 지금까지
    성공한 게 정확히 가운데 병이기 때문(parallax.from_distance 설명 참고).
    """
    if args.anchor_x is not None and args.anchor_y is not None:
        return (args.anchor_x, args.anchor_y), '직접 지정'
    h, w = frame.shape[:2]
    best = None
    for color in ('red', 'green', 'blue'):
        point = _bottle_point(frame, color)
        if point is None:
            continue
        d = (point[0] - w / 2) ** 2 + (point[1] - h / 2) ** 2
        if best is None or d < best[0]:
            best = (d, point, color)
    if best is None:
        raise SystemExit('화면에서 병을 하나도 못 찾아 기준점을 정할 수 없음 — '
                         '--anchor-x/--anchor-y 로 직접 지정하세요')
    return best[1], f'{best[2]} 병(화면중심에 가장 가까움)'


def cmd_from_distance(args) -> None:
    frame = _get_frame(args.image)
    anchor, how = _find_anchor(frame, args)
    print(f'기준점(여기서는 보정 0): ({anchor[0]:.0f},{anchor[1]:.0f}) — {how}')
    params = parallax.from_distance(args.camera_to_board, args.gripper_offset,
                                    anchor[0], anchor[1])
    path = parallax.save(params)
    print(f'{parallax.describe(params)}')
    print(f'저장: {path}')
    cmd_show(args)


def cmd_show(args) -> None:
    params = parallax.load()
    print(f'\n현재 설정: {parallax.describe(params)}')
    try:
        frame = _get_frame(args.image)
    except SystemExit as exc:
        print(f'(현재 화면 확인 생략: {exc})')
        return
    print(f'{"색":6s} {"검출된 병":>14s} {"보정된 목표":>14s} {"이동량":>12s}')
    for color in ('blue', 'green', 'red'):
        bottle = _bottle_point(frame, color)
        if bottle is None:
            continue
        target = parallax.correct(bottle, params)
        print(f'{color:6s} ({bottle[0]:6.0f},{bottle[1]:6.0f}) '
              f'({target[0]:6.0f},{target[1]:6.0f}) '
              f'({target[0] - bottle[0]:+5.0f},{target[1] - bottle[1]:+5.0f})')


def cmd_undo(args) -> None:
    samples = _load_samples()
    if not samples:
        raise SystemExit('취소할 샘플이 없습니다')
    dropped = samples.pop()
    _save_samples(samples)
    print(f'취소: {dropped["color"]} 병=({dropped["bottle"][0]:.0f},{dropped["bottle"][1]:.0f}) '
          f'마커=({dropped["marker"][0]:.0f},{dropped["marker"][1]:.0f})'
          + (f' "{dropped["note"]}"' if dropped.get('note') else ''))
    print(f'남은 샘플 {len(samples)}개')


def cmd_list(args) -> None:
    samples = _load_samples()
    if not samples:
        print(f'샘플 없음 ({parallax.SAMPLES_PATH})')
        return
    print(f'{"#":>2s} {"색":6s} {"병":>14s} {"마커":>14s} {"차이":>12s}  메모')
    for i, s in enumerate(samples):
        b, m = s['bottle'], s['marker']
        print(f'{i:2d} {s["color"]:6s} ({b[0]:6.0f},{b[1]:6.0f}) ({m[0]:6.0f},{m[1]:6.0f}) '
              f'({m[0]-b[0]:+5.0f},{m[1]-b[1]:+5.0f})  {s.get("note", "")}')
    xs = [s['bottle'][0] for s in samples]
    print(f'\nx축 샘플 폭: {max(xs)-min(xs):.0f}px '
          f'(fit 하려면 {parallax.MIN_SPREAD_PX:.0f}px 이상 필요)')


def cmd_clear(args) -> None:
    _save_samples([])
    print(f'샘플을 비웠습니다: {parallax.SAMPLES_PATH}')


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--image', help='고정캠 대신 저장된 사진으로 계산(오프라인 확인용)')
    sub = p.add_subparsers(dest='cmd', required=True)

    r = sub.add_parser('record', help='집게가 실제로 맞은 순간의 (병, 마커) 좌표 기록')
    r.add_argument('--color', required=True, choices=('red', 'green', 'blue'))
    r.add_argument('--note', default='', help='나중에 알아보기 위한 메모')
    r.add_argument('--bottle-x', type=float, help='자동검출 대신 직접 지정')
    r.add_argument('--bottle-y', type=float)
    r.add_argument('--marker-x', type=float, help='자동검출 대신 직접 지정')
    r.add_argument('--marker-y', type=float)
    r.set_defaults(func=cmd_record)

    f = sub.add_parser('fit', help='모아둔 샘플로 계수 맞추기')
    f.set_defaults(func=cmd_fit)

    d = sub.add_parser('from-distance', help='자로 잰 두 거리로 계수 계산')
    d.add_argument('--camera-to-board', type=float, required=True)
    d.add_argument('--gripper-offset', type=float, required=True)
    d.add_argument('--anchor-x', type=float,
                   help='보정을 0으로 둘 기준점(생략하면 화면중심에 가장 가까운 병)')
    d.add_argument('--anchor-y', type=float)
    d.set_defaults(func=cmd_from_distance)

    s = sub.add_parser('show', help='현재 계수와 예상 목표좌표 보기')
    s.set_defaults(func=cmd_show)

    u = sub.add_parser('undo', help='마지막 샘플 취소(집기 실패했을 때)')
    u.set_defaults(func=cmd_undo)

    l = sub.add_parser('list', help='모아둔 샘플 보기')
    l.set_defaults(func=cmd_list)

    c = sub.add_parser('clear', help='모아둔 샘플 비우기')
    c.set_defaults(func=cmd_clear)

    args = p.parse_args()
    args.func(args)


if __name__ == '__main__':
    main()
