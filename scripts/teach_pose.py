#!/usr/bin/env python
"""손으로 팔을 원하는 자세로 옮긴 뒤 그 관절값을 기록한다(직접교시).

관절을 하나씩 명령으로 더듬어 찾는 것보다 훨씬 빠르고 정확하다. 모방학습이 아니라
숫자 6개를 기록하는 것뿐이라 학습/일반화 문제가 없다.

동작 방식: lekiwi_host가 시리얼 포트를 점유하므로 먼저 끄고, SSH로 Pi의 모터 버스에
직접 붙어 토크 해제 -> 사람이 손으로 자세 잡음 -> 관절값 읽기 순으로 진행한다.

⚠️ 토크를 끄면 팔이 중력으로 처지므로 손으로 받친 상태에서 진행할 것.
"""
import argparse
import json
import subprocess
import sys
from pathlib import Path

JOINTS = ['arm_shoulder_pan', 'arm_shoulder_lift', 'arm_elbow_flex',
          'arm_wrist_flex', 'arm_wrist_roll', 'arm_gripper']
POSES_PATH = Path(__file__).parent.parent / 'data' / 'poses.json'
CALIB_PATH = '~/.cache/huggingface/lerobot/calibration/robots/lekiwi/lekiwi01.json'

_BUS_SETUP = f'''
import json, os
from lerobot.motors import Motor, MotorNormMode, MotorCalibration
from lerobot.motors.feetech import FeetechMotorsBus

bus = FeetechMotorsBus(port='/dev/ttyACM0', motors={{
    'arm_shoulder_pan': Motor(1, 'sts3215', MotorNormMode.DEGREES),
    'arm_shoulder_lift': Motor(2, 'sts3215', MotorNormMode.DEGREES),
    'arm_elbow_flex': Motor(3, 'sts3215', MotorNormMode.DEGREES),
    'arm_wrist_flex': Motor(4, 'sts3215', MotorNormMode.DEGREES),
    'arm_wrist_roll': Motor(5, 'sts3215', MotorNormMode.DEGREES),
    'arm_gripper': Motor(6, 'sts3215', MotorNormMode.RANGE_0_100),
}})
raw = json.load(open(os.path.expanduser('{CALIB_PATH}')))
calib = {{k: MotorCalibration(**v) for k, v in raw.items() if k in bus.motors}}
bus.connect()
bus.write_calibration(calib)
arm = list(bus.motors.keys())
'''


def run_on_pi(host: str, code: str) -> str:
    # 코드는 stdin으로 넘긴다 — ssh 인자로 넘기면 원격 셸이 개행/따옴표를 다시
    # 해석해서 깨진다(실측 확인)
    result = subprocess.run(
        ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10', host,
         '~/lerobot_venv/bin/python3 -'],
        input=_BUS_SETUP + code, capture_output=True, text=True, timeout=40)
    if result.returncode != 0:
        raise RuntimeError(f'Pi 실행 실패:\n{result.stderr}')
    return result.stdout


def stop_host(host: str) -> None:
    subprocess.run(['ssh', '-o', 'BatchMode=yes', host,
                    'pkill -f lekiwi_host || true; sleep 2'],
                   capture_output=True, timeout=30)


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument('name', help='저장할 자세 이름 (예: grasp_green)')
    p.add_argument('--pi-ssh', default='roboseasy@192.168.0.201')
    p.add_argument('--keep-torque-off', action='store_true',
                   help='기록 후에도 토크를 켜지 않음(연속으로 여러 자세 기록할 때)')
    args = p.parse_args()

    print('1) lekiwi_host 종료 (시리얼 포트 해제)')
    stop_host(args.pi_ssh)

    print('2) 팔 토크 해제 — 팔을 손으로 받쳐주세요')
    run_on_pi(args.pi_ssh, "bus.disable_torque(arm)\nprint('토크 해제됨')")

    print('\n손으로 팔을 원하는 자세로 옮긴 뒤 엔터를 누르세요. (Ctrl+C로 취소)')
    input()

    print('3) 관절값 읽는 중...')
    out = run_on_pi(args.pi_ssh, """
pos = bus.sync_read('Present_Position', arm)
print('JSON:' + json.dumps({k: round(float(v), 2) for k, v in pos.items()}))
""")
    line = next(ln for ln in out.splitlines() if ln.startswith('JSON:'))
    pose = json.loads(line[len('JSON:'):])

    print(f'\n기록된 자세 "{args.name}":')
    for j in JOINTS:
        print(f'  {j:20s} {pose.get(j, float("nan")):8.2f}')

    POSES_PATH.parent.mkdir(parents=True, exist_ok=True)
    data = json.loads(POSES_PATH.read_text()) if POSES_PATH.exists() else {}
    data[args.name] = pose
    POSES_PATH.write_text(json.dumps(data, indent=2, ensure_ascii=False))
    print(f'\n저장: {POSES_PATH}')

    if not args.keep_torque_off:
        print('4) 토크 다시 켬 (현재 자세 유지)')
        run_on_pi(args.pi_ssh, "bus.enable_torque(arm)\nprint('토크 켜짐')")
    else:
        print('4) 토크 해제 상태 유지 — 다음 자세를 이어서 기록할 수 있어요')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\n취소됨')
    except Exception as e:
        print(f'실패: {type(e).__name__}: {e}', file=sys.stderr)
        sys.exit(1)
