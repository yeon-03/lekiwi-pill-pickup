#!/usr/bin/env python
"""잘 보이는 거리에서 한 번 측정하고, 그 값만큼 눈 감고(개루프) 전진한다.

왜 이렇게 하냐: 가까워질수록 그리퍼가 손목카메라 시야에서 병을 가려서, 끝까지
보면서 접근하는 폐루프가 구조적으로 불가능함(2026-08-24 실측 확인). 베이스 카메라는
내려다보는 각도라 병 인식 자체가 안 됨. 그래서 원래 설계(pick_state_machine.py)대로
"보고 판단 -> 안 보고 실행" 방식으로 되돌린 것.

측정만 하고 끝내려면 --dry-run."""
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
FOCAL_LENGTH_PX = 550.7
BOTTLE_HEIGHT_CM = 8.0
X_SPEED_M_S = 0.1
# 실측 보정 계수 — 명령한 시간×속도와 실제 이동거리가 다를 수 있어 여기서 조정한다
# (1.0에서 시작해 실제 이동량 재보고 맞춤)
SPEED_CALIBRATION = 1.0
MEASURE_SAMPLES = 5   # 여러 프레임 측정해서 중앙값 사용(인식 흔들림 흡수)


def detect(model, frame, target_color, conf):
    r = model(frame, verbose=False, conf=conf)[0]
    for box, cls, c in zip(r.boxes.xyxy, r.boxes.cls, r.boxes.conf):
        if r.names[int(cls)] != 'bottle':
            continue
        x1, y1, x2, y2 = map(int, box.tolist())
        crop = frame[y1:y2, x1:x2]
        if not crop.size:
            continue
        label, _ = classify_color(crop)
        if label == target_color:
            return (x1, y1, x2, y2), float(c)
    return None, 0.0


def send_base(client, obs, x_vel):
    action = {k: obs[k] for k in ARM_KEYS}
    action['x.vel'] = x_vel
    action['y.vel'] = 0.0
    action['theta.vel'] = 0.0
    client.send_action(action)


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument('--lekiwi-host', default='192.168.0.201')
    p.add_argument('--target-color', default='green', choices=('red', 'green', 'blue'))
    p.add_argument('--conf', type=float, default=0.25)
    p.add_argument('--stop-distance', type=float, default=18.0,
                   help='병 앞 이 거리에서 멈춤(cm) — 팔 사정거리 안')
    p.add_argument('--dry-run', action='store_true', help='측정만 하고 안 움직임')
    args = p.parse_args()

    model = YOLO('yolov8n.pt')
    client = LeKiwiClient(LeKiwiClientConfig(remote_ip=args.lekiwi_host))
    client.connect()
    try:
        # --- 1단계: 여러 프레임 측정해서 중앙값 ---
        dists, offsets = [], []
        obs = None
        for i in range(MEASURE_SAMPLES):
            obs = client.get_observation()
            frame = cv2.cvtColor(obs['wrist'], cv2.COLOR_RGB2BGR)
            box, conf = detect(model, frame, args.target_color, args.conf)
            if box is None:
                print(f'  샘플 {i + 1}: 미검출')
                time.sleep(0.15)
                continue
            x1, y1, x2, y2 = box
            d = estimate_distance_cm(y2 - y1, BOTTLE_HEIGHT_CM, FOCAL_LENGTH_PX)
            off = (x1 + x2) / 2 - frame.shape[1] / 2
            dists.append(d)
            offsets.append(off)
            print(f'  샘플 {i + 1}: conf={conf:.2f} 거리={d:.1f}cm 좌우={off:+.0f}px')
            time.sleep(0.15)

        if len(dists) < 3:
            print(f'측정 실패 — {len(dists)}/{MEASURE_SAMPLES}개만 검출됨. '
                  f'병이 잘 보이는 위치에서 다시 시도하세요.')
            return

        dists.sort()
        offsets.sort()
        distance = dists[len(dists) // 2]
        offset = offsets[len(offsets) // 2]
        travel = distance - args.stop_distance
        print(f'\n측정 결과(중앙값): 거리={distance:.1f}cm 좌우오차={offset:+.0f}px')
        print(f'이동해야 할 거리: {travel:.1f}cm')

        if travel <= 0:
            print('이미 충분히 가까움 — 이동 안 함')
            return

        duration = (travel / 100.0) / X_SPEED_M_S * SPEED_CALIBRATION
        print(f'전진 예정: {X_SPEED_M_S}m/s로 {duration:.1f}초')

        if args.dry_run:
            print('(dry-run이라 실제로 안 움직임)')
            return

        # --- 2단계: 개루프 전진 ---
        print('전진 시작 (이 동안 카메라 안 봄)')
        elapsed = 0.0
        while elapsed < duration:
            send_base(client, obs, X_SPEED_M_S)
            time.sleep(0.1)
            elapsed += 0.1
        send_base(client, obs, 0.0)
        print('정지')

        # --- 3단계: 결과 확인(보이면 보고, 안 보여도 정상) ---
        time.sleep(0.5)
        obs2 = client.get_observation()
        frame2 = cv2.cvtColor(obs2['wrist'], cv2.COLOR_RGB2BGR)
        box2, conf2 = detect(model, frame2, args.target_color, args.conf)
        if box2 is None:
            print('이동 후: 병 안 보임(그리퍼에 가려진 것으로 추정 — 예상된 동작)')
        else:
            x1, y1, x2, y2 = box2
            d2 = estimate_distance_cm(y2 - y1, BOTTLE_HEIGHT_CM, FOCAL_LENGTH_PX)
            print(f'이동 후: 거리={d2:.1f}cm (목표 {args.stop_distance:.0f}cm, '
                  f'실제 이동 {distance - d2:.1f}cm / 예상 {travel:.1f}cm)')
            print(f'  -> 보정계수 참고값: {(distance - d2) / travel:.2f}')
    finally:
        try:
            if obs is not None:
                send_base(client, obs, 0.0)
        except Exception:
            pass
        client.disconnect()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\n중단')
    except Exception as e:
        print(f'실패: {type(e).__name__}: {e}', file=sys.stderr)
        sys.exit(1)
