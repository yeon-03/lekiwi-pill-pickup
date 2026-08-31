#!/usr/bin/env python
"""1단계 스파이크: 손목 카메라로 지정한 색 병을 찾아서, 화면 중앙 기준 좌우
오프셋 방향으로 arm_shoulder_pan 관절만 아주 조금(5도) 움직여본다 — 폐루프 아니고
딱 1번만. 방향이 맞는지 확인용."""
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
PAN_STEP_DEG = 5.0


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
        frame = cv2.cvtColor(obs['wrist'], cv2.COLOR_RGB2BGR)
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
            print(f'{args.target_color} 병을 못 찾았어요 — 관절 안 움직임')
            return

        x1, y1, x2, y2 = target_box
        center_x = (x1 + x2) / 2
        x_offset = center_x - w / 2
        print(f'{args.target_color} 병 찾음, 중앙 대비 오프셋={x_offset:.0f}px '
              f'({"오른쪽" if x_offset > 0 else "왼쪽"})')

        current_pan = obs['arm_shoulder_pan.pos']
        # 오른쪽에 있으면 +방향, 왼쪽이면 -방향으로 시도(실제 맞는지는 결과 보고 판단)
        delta = PAN_STEP_DEG if x_offset > 0 else -PAN_STEP_DEG
        new_pan = current_pan + delta
        print(f'arm_shoulder_pan: {current_pan:.1f} -> {new_pan:.1f} (delta={delta:+.1f}도)')

        action = {k: obs[k] for k in ARM_KEYS}
        action['arm_shoulder_pan.pos'] = new_pan
        # 호스트 send_action()이 x.vel/y.vel/theta.vel을 먼저 꺼내 쓰는데 없으면
        # KeyError로 죽어서 그 아래 팔 Goal_Position 쓰기까지 통째로 실행 안 됨
        # (이 프로젝트가 예전에 겪은 "팔 값 누락→바퀴 명령 실패" 버그의 반대 케이스)
        action['x.vel'] = 0.0
        action['y.vel'] = 0.0
        action['theta.vel'] = 0.0
        # 워치독(500ms)보다 빠른 주기로 1초간 반복 전송 — 안 그러면 연결 끊길 때
        # disable_torque_on_disconnect가 발동해서 목표 위치 도달 전에 힘이 풀림
        for _ in range(10):
            client.send_action(action)
            time.sleep(0.1)
        print('관절 이동 명령 전송 완료(1초간 유지) — 실제로 어느 쪽으로 움직였는지 확인해주세요')
    finally:
        client.disconnect()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f'실패: {type(e).__name__}: {e}', file=sys.stderr)
        sys.exit(1)
