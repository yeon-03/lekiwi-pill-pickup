#!/usr/bin/env python
"""목표 관절 자세로 천천히(보간해서) 이동한다. 급격한 점프를 피하려고 여러 스텝에
나눠 보낸다. lerobot/svla_so101_pickplace 데이터셋에서 뽑은 표준 자세들을 프리셋으로 둔다."""
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from robot_link import RobotLink  # noqa: E402

JOINTS = ['arm_shoulder_pan', 'arm_shoulder_lift', 'arm_elbow_flex',
          'arm_wrist_flex', 'arm_wrist_roll', 'arm_gripper']

# lerobot/svla_so101_pickplace 에피소드0에서 읽은 실제 사람 시연 자세
PRESETS = {
    # 0~30% 구간의 대기(홈) 자세
    'home': dict(arm_shoulder_pan=2.0, arm_shoulder_lift=-98.7, arm_elbow_flex=99.0,
                 arm_wrist_flex=74.8, arm_wrist_roll=-51.5, arm_gripper=1.4),
    # 40% 구간 — 뻗는 중간, 그리퍼 이미 열림
    'reach_mid': dict(arm_shoulder_pan=13.2, arm_shoulder_lift=-31.8, arm_elbow_flex=56.1,
                      arm_wrist_flex=55.4, arm_wrist_roll=-48.0, arm_gripper=20.6),
    # 50% 구간 — 물체 앞 도달, 그리퍼 열린 상태
    'pre_grasp': dict(arm_shoulder_pan=8.7, arm_shoulder_lift=-0.1, arm_elbow_flex=36.6,
                      arm_wrist_flex=51.3, arm_wrist_roll=-50.4, arm_gripper=20.6),
}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--lekiwi-host', default='192.168.0.201')
    parser.add_argument('--preset', required=True, choices=sorted(PRESETS))
    parser.add_argument('--duration', type=float, default=4.0, help='이동에 쓸 시간(초)')
    args = parser.parse_args()

    target = PRESETS[args.preset]
    client = RobotLink(args.lekiwi_host)
    client.connect()
    try:
        obs = client.get_observation()
        start = {j: obs[f'{j}.pos'] for j in JOINTS}
        print(f'현재 -> 목표({args.preset}):')
        for j in JOINTS:
            print(f'  {j:20s} {start[j]:7.1f} -> {target[j]:7.1f}')

        steps = int(args.duration / 0.1)
        for s in range(1, steps + 1):
            ratio = s / steps
            action = {f'{j}.pos': start[j] + (target[j] - start[j]) * ratio for j in JOINTS}
            action['x.vel'] = 0.0
            action['y.vel'] = 0.0
            action['theta.vel'] = 0.0
            client.send_action(action)
            time.sleep(0.1)

        after = client.get_observation()
        print('도달값:')
        for j in JOINTS:
            print(f'  {j:20s} {after[f"{j}.pos"]:7.1f}')
    finally:
        client.disconnect()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f'실패: {type(e).__name__}: {e}', file=sys.stderr)
        sys.exit(1)
