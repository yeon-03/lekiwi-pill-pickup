#!/usr/bin/env python
"""고정 설치된 외부 USB 카메라(노트북에 직결, 르키위 원격캠 아님)를 혼자 계속
열어두고, 최신 프레임을 공유 파일로 계속 갱신해두는 상시 서버.

배경: OpenCV VideoCapture는 한 프로세스만 장치를 열 수 있어서, 지금까지는
실시간 미리보기와 정렬(align_and_grasp.py)이 카메라를 동시에 못 썼다 —
정렬을 돌릴 때마다 보고 있던 창을 꺼야 했다. 이 서버가 장치를 계속 물고
있고, 다른 스크립트는 이 서버가 써둔 최신 프레임(파일)만 읽으면 되므로
여러 스크립트가 동시에 봐도 충돌이 없다.

align_and_grasp.py는 이 서버가 떠 있으면 자동으로 이 공유 프레임을 쓰고,
없으면 지금까지처럼 카메라를 직접 연다(하위호환 — 이 서버 없이 단독
실행도 그대로 됨)."""
import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

import cv2

FRAME_DIR = Path('/dev/shm/lekiwi_cam')
FRAME_PATH = FRAME_DIR / 'fixed.jpg'
STALE_SEC = 1.0
POLL_INTERVAL_SEC = 0.03  # 최대 약 33fps


def _atomic_write_bytes(path: Path, data: bytes) -> None:
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_bytes(data)
    os.replace(tmp, path)


def fixed_frame_available() -> bool:
    """서버가 살아서 최근에 갱신했는지. 오래됐으면(죽었으면) False —
    align_and_grasp.py가 이걸로 직접연결 폴백 여부를 판단한다."""
    try:
        return (time.time() - FRAME_PATH.stat().st_mtime) < STALE_SEC
    except OSError:
        return False


def read_fixed_frame():
    """공유 프레임을 BGR numpy 배열로 반환. 오래됐거나 없으면 None."""
    if not fixed_frame_available():
        return None
    return cv2.imread(str(FRAME_PATH))


def _find_usb_camera_index(max_index: int = 6) -> int:
    """/dev/video0..max_index 중 실제로 프레임을 읽을 수 있는 첫 장치를 찾는다.
    USB 카메라는 재연결/다른 장치가 꽂히면 인덱스가 밀린다(2026-08-26 실측:
    0->1로 밀려서 --usb-index 기본값이 안 맞게 됨) — 매번 사람이 index를
    알아내서 넘겨야 하는 걸 피한다."""
    for i in range(max_index + 1):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ok, _ = cap.read()
            cap.release()
            if ok:
                return i
    raise RuntimeError(f'사용 가능한 USB 카메라를 못 찾음 (/dev/video0..{max_index} 확인)')


def _restore_auto_exposure(index: int) -> None:
    """카메라를 자동 노출로 되돌린다.

    ■ 왜 필요한가 (2026-08-27 실측)
      이 카메라(/dev/video1)가 auto_exposure=1(Manual) + brightness=-25 로
      잠긴 채 남아 있어서, 화면 평균밝기가 3.7 로 사실상 깜깜했다. 장면은
      제대로 보고 있는데 색 검출이 전부 0개가 되어 '고정캠이 죽었나'로
      오해하기 쉽다. 이 설정은 카메라에 남아 있어 프로세스를 다시 띄워도
      안 풀리므로, 기동할 때마다 명시적으로 되돌린다.
      (auto_exposure: 1=Manual, 3=Aperture Priority=자동)
    """
    try:
        subprocess.run(
            ['v4l2-ctl', '-d', f'/dev/video{index}',
             '--set-ctrl=auto_exposure=3', '--set-ctrl=brightness=0'],
            check=False, capture_output=True, timeout=5)
    except (OSError, subprocess.SubprocessError) as e:
        print(f'노출 자동복원 실패(무시하고 계속): {e}', file=sys.stderr)


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument('--usb-index', type=int, default=-1,
                   help='기본값 -1은 기동 시 자동탐색. 여러 카메라가 동시에 꽂혀 '
                        '있어 특정 장치를 강제해야 할 때만 실제 인덱스를 지정')
    args = p.parse_args()

    FRAME_DIR.mkdir(parents=True, exist_ok=True)
    index = args.usb_index
    if index < 0:
        index = _find_usb_camera_index()
        print(f'USB 카메라 자동탐색 결과: /dev/video{index} 사용')
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        print(f'/dev/video{index} 를 열 수 없음', file=sys.stderr)
        sys.exit(1)
    _restore_auto_exposure(index)

    print(f'/dev/video{index} 열림 — 공유 프레임: {FRAME_PATH} (Ctrl+C로 종료)')
    try:
        while True:
            cap.grab()
            ok, frame = cap.retrieve()
            if ok:
                ok2, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
                if ok2:
                    _atomic_write_bytes(FRAME_PATH, buf.tobytes())
            time.sleep(POLL_INTERVAL_SEC)
    except KeyboardInterrupt:
        print('\n종료')
    finally:
        cap.release()


if __name__ == '__main__':
    main()
