#!/usr/bin/env python
"""teach_pose.py로 기록한 자세로 팔을 부드럽게(보간) 이동시킨다.

goto_pose.py의 프리셋 대신 data/poses.json에 저장된, 직접교시로 얻은 실제
자세를 재생한다. 목적지 도달 후 그리퍼만 별도로 닫을 수 있게 --then-close 옵션도
둔다(잡기 동작의 마지막 단계)."""
import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from robot_link import RobotLink  # noqa: E402

JOINTS = ['arm_shoulder_pan', 'arm_shoulder_lift', 'arm_elbow_flex',
          'arm_wrist_flex', 'arm_wrist_roll', 'arm_gripper']
POSES_PATH = Path(__file__).parent.parent / 'data' / 'poses.json'


def load_pose(name: str) -> dict:
    if not POSES_PATH.exists():
        print(f'{POSES_PATH}가 없어요 — teach_pose.py로 먼저 자세를 기록하세요')
        sys.exit(1)
    data = json.loads(POSES_PATH.read_text())
    if name not in data:
        print(f'"{name}" 자세가 없어요. 저장된 것: {list(data)}')
        sys.exit(1)
    return data[name]


def interpolate_to(client, target: dict, duration: float, gripper_override: float | None = None):
    obs = client.get_observation()
    start = {j: float(obs[f'{j}.pos']) for j in JOINTS}
    goal = dict(target)
    if gripper_override is not None:
        goal['arm_gripper'] = gripper_override

    steps = max(1, int(duration / 0.1))
    for s in range(1, steps + 1):
        ratio = s / steps
        action = {f'{j}.pos': start[j] + (goal[j] - start[j]) * ratio for j in JOINTS}
        action['x.vel'] = 0.0
        action['y.vel'] = 0.0
        action['theta.vel'] = 0.0
        client.send_action(action)
        time.sleep(0.1)
    return client.get_observation()


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument('name', help='data/poses.json에 저장된 자세 이름')
    p.add_argument('--lekiwi-host', default='192.168.0.201')
    p.add_argument('--duration', type=float, default=4.0)
    p.add_argument('--then-close', action='store_true',
                   help='도달 후 그리퍼를 추가로 닫는다(값은 --close-value)')
    p.add_argument('--close-value', type=float, default=5.0)
    p.add_argument('--close-duration', type=float, default=1.5)
    args = p.parse_args()

    target = load_pose(args.name)
    missing = [j for j in JOINTS if j not in target]
    if missing:
        print(f'저장된 자세에 빠진 관절이 있어요: {missing}')
        sys.exit(1)

    print(f'"{args.name}" 자세로 이동합니다:')
    for j in JOINTS:
        print(f'  {j:20s} -> {target[j]:8.2f}')

    client = RobotLink(args.lekiwi_host)
    client.connect()
    try:
        after = interpolate_to(client, target, args.duration)
        print('도달값:')
        for j in JOINTS:
            print(f'  {j:20s} {after[f"{j}.pos"]:8.2f}')

        if args.then_close:
            print(f'\n그리퍼 닫기: {after["arm_gripper.pos"]:.1f} -> {args.close_value}')
            action = {f'{j}.pos': after[f'{j}.pos'] for j in JOINTS}
            action['arm_gripper.pos'] = args.close_value
            action['x.vel'] = 0.0
            action['y.vel'] = 0.0
            action['theta.vel'] = 0.0
            steps = max(1, int(args.close_duration / 0.1))
            for _ in range(steps):
                client.send_action(action)
                time.sleep(0.1)
            final = client.get_observation()
            print(f'그리퍼 실제 도달값: {final["arm_gripper.pos"]:.1f}')
    finally:
        client.disconnect()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f'실패: {type(e).__name__}: {e}', file=sys.stderr)
        sys.exit(1)
