#!/usr/bin/env python
"""1단계 스파이크: 지정한 색 병을 프론트 카메라로 찾아서, 화면 중앙 기준 좌우
오프셋 방향으로 아주 짧게(0.3초) 회전만 해본다 — 폐루프 아니고 딱 1번만.
방향이 맞는지 확인용. 팔 위치는 현재 값 그대로 유지해서 같이 보낸다(안 그러면
호스트가 StopIteration으로 죽는 기존 발견된 버그 회피, lekiwi_control.py 참고)."""
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
THETA_SPEED_DEG_S = 30.0  # 제일 낮은 속도 단계
PULSE_SEC = 0.3


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--lekiwi-host', default='192.168.0.201')
    parser.add_argument('--target-color', default='red', choices=('red', 'green', 'blue'))
    args = parser.parse_args()

    model = YOLO('yolov8n.pt')
    client = LeKiwiClient(LeKiwiClientConfig(remote_ip=args.lekiwi_host))
    client.connect()
    try:
        obs = client.get_observation()
        frame = cv2.cvtColor(obs['front'], cv2.COLOR_RGB2BGR)
        h, w = frame.shape[:2]

        results = model(frame, verbose=False, conf=0.25)[0]
        target_box = None
        for box, cls, conf in zip(results.boxes.xyxy, results.boxes.cls, results.boxes.conf):
            if results.names[int(cls)] != 'bottle':
                continue
            x1, y1, x2, y2 = map(int, box.tolist())
            crop = frame[y1:y2, x1:x2]
            label, _ = classify_color(crop) if crop.size else ('?', {})
            if label == args.target_color:
                target_box = (x1, y1, x2, y2)
                break

        if target_box is None:
            print(f'{args.target_color} 병을 못 찾았어요 — 회전 안 함')
            return

        x1, y1, x2, y2 = target_box
        center_x = (x1 + x2) / 2
        x_offset = center_x - w / 2
        print(f'{args.target_color} 병 찾음, 중앙 대비 오프셋={x_offset:.0f}px '
              f'({"오른쪽" if x_offset > 0 else "왼쪽"})')

        # 오른쪽에 있으면 오른쪽으로(theta 음수), 왼쪽에 있으면 왼쪽으로(theta 양수)
        theta_cmd = -THETA_SPEED_DEG_S if x_offset > 0 else THETA_SPEED_DEG_S

        action = {k: obs[k] for k in ARM_KEYS}
        action['x.vel'] = 0.0
        action['y.vel'] = 0.0
        action['theta.vel'] = theta_cmd
        print(f'회전 명령 전송: theta.vel={theta_cmd} deg/s, {PULSE_SEC}초간')
        client.send_action(action)
        time.sleep(PULSE_SEC)

        stop_action = {k: obs[k] for k in ARM_KEYS}
        stop_action['x.vel'] = 0.0
        stop_action['y.vel'] = 0.0
        stop_action['theta.vel'] = 0.0
        client.send_action(stop_action)
        print('정지 명령 전송 완료')
    finally:
        client.disconnect()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f'실패: {type(e).__name__}: {e}', file=sys.stderr)
        sys.exit(1)
