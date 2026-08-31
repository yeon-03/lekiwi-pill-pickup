#!/usr/bin/env python
"""손목 카메라로 지정한 색 병을 보면서 바퀴로 목표 거리까지 자동 접근.

거리는 모노큘러 추정(픽셀 높이 + 캘리브레이션한 focal_length)으로 계산한다.
안전장치: 짧은 펄스로만 전진, 목표 거리 도달 시 자동 정지, 최대 스텝 제한,
미검출 시 즉시 정지, 종료 시 반드시 정지 명령 전송."""
import argparse
import sys
import time

import cv2
from ultralytics import YOLO

from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig

sys.path.insert(0, str(__import__('pathlib').Path(__file__).parent))
from test_bottle_color import classify_color  # noqa: E402

sys.path.insert(0, str(__import__('pathlib').Path(__file__).parent.parent / 'src'))
from lekiwi_pill_pickup.distance_estimation import estimate_distance_cm  # noqa: E402

ARM_KEYS = [
    'arm_shoulder_pan.pos', 'arm_shoulder_lift.pos', 'arm_elbow_flex.pos',
    'arm_wrist_flex.pos', 'arm_wrist_roll.pos', 'arm_gripper.pos',
]
FOCAL_LENGTH_PX = 550.7      # 2026-08-24 실측(손목카메라)
BOTTLE_HEIGHT_CM = 8.0       # 시럽병 실측 높이
TARGET_DISTANCE_CM = 25.0    # 여기까지만 접근(팔 사정거리 밖에서 멈춤 — 보수적)
DISTANCE_TOLERANCE_CM = 3.0
X_SPEED_M_S = 0.1            # 제일 낮은 속도 단계
PULSE_SEC = 0.3
MAX_STEPS = 15
MISS_LIMIT = 3


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


def send_base(client, obs, x_vel: float) -> None:
    action = {k: obs[k] for k in ARM_KEYS}
    action['x.vel'] = x_vel
    action['y.vel'] = 0.0
    action['theta.vel'] = 0.0
    client.send_action(action)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--lekiwi-host', default='192.168.0.201')
    parser.add_argument('--target-color', default='red', choices=('red', 'green', 'blue'))
    parser.add_argument('--conf', type=float, default=0.25)
    parser.add_argument('--target-distance', type=float, default=TARGET_DISTANCE_CM)
    args = parser.parse_args()

    model = YOLO('yolov8n.pt')
    client = LeKiwiClient(LeKiwiClientConfig(remote_ip=args.lekiwi_host))
    client.connect()
    misses = 0
    last_obs = None
    try:
        for step in range(MAX_STEPS):
            obs = client.get_observation()
            last_obs = obs
            frame = cv2.cvtColor(obs['wrist'], cv2.COLOR_RGB2BGR)

            box = find_target(model, frame, args.target_color, args.conf)
            if box is None:
                misses += 1
                print(f'[{step + 1}/{MAX_STEPS}] {args.target_color} 병 미검출 '
                      f'({misses}/{MISS_LIMIT}) — 정지')
                send_base(client, obs, 0.0)
                if misses >= MISS_LIMIT:
                    print('연속 미검출 — 중단')
                    return
                time.sleep(PULSE_SEC)
                continue
            misses = 0

            _, y1, _, y2 = box
            pixel_height = y2 - y1
            distance = estimate_distance_cm(pixel_height, BOTTLE_HEIGHT_CM, FOCAL_LENGTH_PX)
            error = distance - args.target_distance
            print(f'[{step + 1}/{MAX_STEPS}] 픽셀높이={pixel_height}px '
                  f'거리={distance:.1f}cm (목표 {args.target_distance:.0f}cm, 오차 {error:+.1f}cm)')

            if abs(error) <= DISTANCE_TOLERANCE_CM:
                print('목표 거리 도달 — 정지')
                send_base(client, obs, 0.0)
                return
            if error < 0:
                print('목표보다 가까움 — 전진 안 함, 정지')
                send_base(client, obs, 0.0)
                return

            send_base(client, obs, X_SPEED_M_S)
            time.sleep(PULSE_SEC)
            send_base(client, obs, 0.0)   # 펄스 끝나면 즉시 정지
            time.sleep(0.1)
        print('최대 스텝 도달 — 종료')
    finally:
        if last_obs is not None:
            try:
                send_base(client, last_obs, 0.0)
            except Exception:
                pass
        client.disconnect()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\n사용자 중단')
    except Exception as e:
        print(f'실패: {type(e).__name__}: {e}', file=sys.stderr)
        sys.exit(1)
