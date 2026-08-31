#!/usr/bin/env python
"""그리퍼를 지정한 값으로 벌린 채 일정 시간 유지한다(자로 폭을 재기 위한 용도).
팔 관절은 현재 자세 그대로 유지."""
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from robot_link import RobotLink  # noqa: E402

ARM_KEYS = [
    'arm_shoulder_pan.pos', 'arm_shoulder_lift.pos', 'arm_elbow_flex.pos',
    'arm_wrist_flex.pos', 'arm_wrist_roll.pos', 'arm_gripper.pos',
]


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument('--lekiwi-host', default='192.168.0.201')
    p.add_argument('--value', type=float, required=True, help='그리퍼 목표값 (0~100)')
    p.add_argument('--hold-sec', type=float, default=20.0)
    args = p.parse_args()

    client = RobotLink(args.lekiwi_host)
    client.connect()
    try:
        obs = client.get_observation()
        action = {k: obs[k] for k in ARM_KEYS}
        action['arm_gripper.pos'] = args.value
        action['x.vel'] = 0.0
        action['y.vel'] = 0.0
        action['theta.vel'] = 0.0

        print(f'그리퍼 {obs["arm_gripper.pos"]:.1f} -> {args.value} 로 벌리고 '
              f'{args.hold_sec:.0f}초 유지합니다. 지금 자로 재주세요.')
        steps = int(args.hold_sec / 0.1)
        for i in range(steps):
            client.send_action(action)
            if i % 50 == 0 and i > 0:
                print(f'  {i * 0.1:.0f}초 경과...')
            time.sleep(0.1)

        after = client.get_observation()
        print(f'유지 끝. 실제 도달값: {after["arm_gripper.pos"]:.1f}')
    finally:
        client.disconnect()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f'실패: {type(e).__name__}: {e}', file=sys.stderr)
        sys.exit(1)
