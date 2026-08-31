#!/usr/bin/env python
"""지정한 관절 하나를 지정한 각도만큼만 움직여보는 수동 탐색 도구.
어느 관절이 어느 방향으로 움직이는지 실기기에서 확인할 때 쓴다."""
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
    parser = argparse.ArgumentParser()
    parser.add_argument('--lekiwi-host', default='192.168.0.201')
    parser.add_argument('--joint', required=True, choices=[k.removesuffix('.pos') for k in ARM_KEYS])
    parser.add_argument('--delta', type=float, required=True, help='움직일 각도(도)')
    parser.add_argument('--hold-sec', type=float, default=1.5)
    args = parser.parse_args()

    client = RobotLink(args.lekiwi_host)
    client.connect()
    try:
        obs = client.get_observation()
        key = f'{args.joint}.pos'
        current = obs[key]
        target = current + args.delta
        print(f'{args.joint}: {current:.1f} -> {target:.1f} ({args.delta:+.1f}도)')

        action = {k: obs[k] for k in ARM_KEYS}
        action[key] = target
        action['x.vel'] = 0.0
        action['y.vel'] = 0.0
        action['theta.vel'] = 0.0
        for _ in range(int(args.hold_sec / 0.1)):
            client.send_action(action)
            time.sleep(0.1)

        after = client.get_observation()
        print(f'  실제 도달값: {after[key]:.1f}')
    finally:
        client.disconnect()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f'실패: {type(e).__name__}: {e}', file=sys.stderr)
        sys.exit(1)
