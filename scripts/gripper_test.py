#!/usr/bin/env python
"""그리퍼만 열고 닫아보는 안전 테스트 — 팔 관절은 전혀 안 움직인다.
Task 3(그립 성공 판정용 부하값 실측)의 사전 단계이기도 하다."""
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


def hold(client, obs, gripper_value: float, seconds: float) -> None:
    action = {k: obs[k] for k in ARM_KEYS}
    action['arm_gripper.pos'] = gripper_value
    action['x.vel'] = 0.0
    action['y.vel'] = 0.0
    action['theta.vel'] = 0.0
    for _ in range(int(seconds / 0.1)):
        client.send_action(action)
        time.sleep(0.1)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--lekiwi-host', default='192.168.0.201')
    parser.add_argument('--open-value', type=float, default=40.0,
                        help='그리퍼 열림 값 (RANGE_0_100)')
    parser.add_argument('--close-value', type=float, default=5.0)
    args = parser.parse_args()

    client = RobotLink(args.lekiwi_host)
    client.connect()
    try:
        obs = client.get_observation()
        print(f'현재 그리퍼 값: {obs["arm_gripper.pos"]:.1f}')

        print(f'열기 -> {args.open_value}')
        hold(client, obs, args.open_value, 1.5)
        obs2 = client.get_observation()
        print(f'  실제 도달값: {obs2["arm_gripper.pos"]:.1f}')

        print(f'닫기 -> {args.close_value}')
        hold(client, obs, args.close_value, 1.5)
        obs3 = client.get_observation()
        print(f'  실제 도달값: {obs3["arm_gripper.pos"]:.1f}')

        print('원래 값으로 복귀')
        hold(client, obs, obs['arm_gripper.pos'], 1.0)
    finally:
        client.disconnect()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f'실패: {type(e).__name__}: {e}', file=sys.stderr)
        sys.exit(1)
