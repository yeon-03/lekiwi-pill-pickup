#!/usr/bin/env python
"""손목 카메라로 지정한 색 병을 자동 추적 — 키보드 없이 폐루프로 팔이 따라간다.

"보고 -> 오차만큼 조금 움직이고 -> 다시 보고"를 반복하는 비주얼 서보잉.
안전장치: 스텝당 최대 이동각 제한, 중앙 진입 시 자동 종료, 최대 반복 횟수 제한,
미검출 시 즉시 중단."""
import argparse
import sys
import time

import cv2
from ultralytics import YOLO

from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig

sys.path.insert(0, str(__import__('pathlib').Path(__file__).parent))
from test_bottle_color import classify_color  # noqa: E402

ARM_KEYS = [
    'arm_shoulder_pan.pos', 'arm_shoulder_lift.pos', 'arm_elbow_flex.pos',
    'arm_wrist_flex.pos', 'arm_wrist_roll.pos', 'arm_gripper.pos',
]
# 픽셀 오차 1당 몇 도 움직일지 — 작게 시작(진동 방지)
GAIN_DEG_PER_PX = 0.02
MAX_STEP_DEG = 3.0          # 스텝당 최대 이동각(안전)
CENTER_TOLERANCE_PX = 25.0  # 이 안에 들어오면 정렬 완료로 보고 종료
MAX_STEPS = 20              # 무한 루프 방지
MISS_LIMIT = 3              # 연속 미검출 허용 횟수(흔들림 흡수)
STEP_INTERVAL_SEC = 0.15


def find_target(model, frame, target_color: str, conf: float):
    results = model(frame, verbose=False, conf=conf)[0]
    for box, cls in zip(results.boxes.xyxy, results.boxes.cls):
        if results.names[int(cls)] != 'bottle':
            continue
        x1, y1, x2, y2 = map(int, box.tolist())
        crop = frame[y1:y2, x1:x2]
        if not crop.size:
            continue
        label, _ = classify_color(crop)
        if label == target_color:
            return (x1, y1, x2, y2)
    return None


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--lekiwi-host', default='192.168.0.201')
    parser.add_argument('--target-color', default='red', choices=('red', 'green', 'blue'))
    parser.add_argument('--conf', type=float, default=0.25)
    args = parser.parse_args()

    model = YOLO('yolov8n.pt')
    client = LeKiwiClient(LeKiwiClientConfig(remote_ip=args.lekiwi_host))
    client.connect()
    misses = 0
    try:
        for step in range(MAX_STEPS):
            obs = client.get_observation()
            frame = cv2.cvtColor(obs['wrist'], cv2.COLOR_RGB2BGR)
            w = frame.shape[1]

            box = find_target(model, frame, args.target_color, args.conf)
            if box is None:
                misses += 1
                print(f'[{step + 1}/{MAX_STEPS}] {args.target_color} 병 미검출 ({misses}/{MISS_LIMIT})')
                if misses >= MISS_LIMIT:
                    print('연속 미검출 — 중단')
                    return
                time.sleep(STEP_INTERVAL_SEC)
                continue
            misses = 0

            x1, _, x2, _ = box
            x_offset = (x1 + x2) / 2 - w / 2
            if abs(x_offset) <= CENTER_TOLERANCE_PX:
                print(f'[{step + 1}/{MAX_STEPS}] 정렬 완료 (오차 {x_offset:+.0f}px) — 종료')
                return

            delta = max(-MAX_STEP_DEG, min(MAX_STEP_DEG, x_offset * GAIN_DEG_PER_PX))
            current_pan = obs['arm_shoulder_pan.pos']
            action = {k: obs[k] for k in ARM_KEYS}
            action['arm_shoulder_pan.pos'] = current_pan + delta
            # 호스트 send_action()이 이 키들을 먼저 꺼내 써서, 없으면 KeyError로
            # 팔 명령까지 통째로 무시됨(실측 확인)
            action['x.vel'] = 0.0
            action['y.vel'] = 0.0
            action['theta.vel'] = 0.0
            client.send_action(action)
            print(f'[{step + 1}/{MAX_STEPS}] 오차 {x_offset:+.0f}px -> pan {current_pan:.1f} '
                  f'{delta:+.2f}도')
            time.sleep(STEP_INTERVAL_SEC)
        print('최대 스텝 도달 — 종료')
    finally:
        client.disconnect()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\n사용자 중단')
    except Exception as e:
        print(f'실패: {type(e).__name__}: {e}', file=sys.stderr)
        sys.exit(1)
