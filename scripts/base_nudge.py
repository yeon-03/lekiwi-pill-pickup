#!/usr/bin/env python
"""바퀴를 짧은 펄스로만 움직이는 수동 탐색 도구 (전/후/좌/우/회전).
팔 관절은 현재 값 그대로 유지해서 같이 보낸다(안 보내면 호스트가 KeyError로 죽음)."""
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


def send(client, obs, x=0.0, y=0.0, theta=0.0):
    action = {k: obs[k] for k in ARM_KEYS}
    action['x.vel'] = x
    action['y.vel'] = y
    action['theta.vel'] = theta
    client.send_action(action)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--lekiwi-host', default='192.168.0.201')
    parser.add_argument('--x', type=float, default=0.0, help='전진(+)/후진(-) m/s')
    parser.add_argument('--y', type=float, default=0.0, help='좌(+)/우(-) m/s')
    parser.add_argument('--theta', type=float, default=0.0, help='좌회전(+)/우회전(-) deg/s')
    parser.add_argument('--duration', type=float, default=0.5, help='이동 시간(초)')
    args = parser.parse_args()

    client = RobotLink(args.lekiwi_host)
    client.connect()
    try:
        obs = client.get_observation()
        print(f'이동: x={args.x} y={args.y} theta={args.theta}, {args.duration}초')
        elapsed = 0.0
        while elapsed < args.duration:
            send(client, obs, args.x, args.y, args.theta)
            time.sleep(0.1)
            elapsed += 0.1
        send(client, obs, 0.0, 0.0, 0.0)
        print('정지')
    finally:
        try:
            send(client, obs, 0.0, 0.0, 0.0)
        except Exception:
            pass
        client.disconnect()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f'실패: {type(e).__name__}: {e}', file=sys.stderr)
        sys.exit(1)
