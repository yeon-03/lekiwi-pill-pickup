#!/usr/bin/env python
"""상하 정렬 1단계 스파이크: 손목 카메라로 지정한 색 병을 찾아서, 화면 중앙 기준
상하 오프셋 방향으로 지정한 관절을 아주 조금(5도) 움직여본다 — 딱 1번만.
어느 관절이 상하를 담당하고 부호가 어느 쪽인지 확인용."""
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
STEP_DEG = 5.0


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--lekiwi-host', default='192.168.0.201')
    parser.add_argument('--target-color', default='red', choices=('red', 'green', 'blue'))
    parser.add_argument('--joint', default='arm_wrist_flex',
                        choices=('arm_wrist_flex', 'arm_shoulder_lift', 'arm_elbow_flex'))
    args = parser.parse_args()

    model = YOLO('yolov8n.pt')
    client = LeKiwiClient(LeKiwiClientConfig(remote_ip=args.lekiwi_host))
    client.connect()
    try:
        obs = client.get_observation()
        frame = cv2.cvtColor(obs['wrist'], cv2.COLOR_RGB2BGR)
        h = frame.shape[0]

        results = model(frame, verbose=False, conf=0.25)[0]
        target_box = None
        for box, cls in zip(results.boxes.xyxy, results.boxes.cls):
            if results.names[int(cls)] != 'bottle':
                continue
            x1, y1, x2, y2 = map(int, box.tolist())
            crop = frame[y1:y2, x1:x2]
            if not crop.size:
                continue
            label, _ = classify_color(crop)
            if label == args.target_color:
                target_box = (x1, y1, x2, y2)
                break

        if target_box is None:
            print(f'{args.target_color} 병을 못 찾았어요 — 관절 안 움직임')
            return

        _, y1, _, y2 = target_box
        center_y = (y1 + y2) / 2
        y_offset = center_y - h / 2
        print(f'{args.target_color} 병 찾음, 세로 오프셋={y_offset:.0f}px '
              f'({"아래쪽" if y_offset > 0 else "위쪽"})')

        key = f'{args.joint}.pos'
        current = obs[key]
        delta = STEP_DEG if y_offset > 0 else -STEP_DEG
        print(f'{args.joint}: {current:.1f} -> {current + delta:.1f} (delta={delta:+.1f}도)')

        action = {k: obs[k] for k in ARM_KEYS}
        action[key] = current + delta
        action['x.vel'] = 0.0
        action['y.vel'] = 0.0
        action['theta.vel'] = 0.0
        for _ in range(10):
            client.send_action(action)
            time.sleep(0.1)
        print('전송 완료(1초 유지) — 팔이 위/아래 중 어느 쪽으로 움직였는지 알려주세요')
    finally:
        client.disconnect()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f'실패: {type(e).__name__}: {e}', file=sys.stderr)
        sys.exit(1)
