#!/usr/bin/env python
"""약병 집기 전체 사이클을 한 번에 실행한다: 정렬 -> 하강 -> 파지 -> 들어올리기 -> 홈.

지금까지는 이 단계들을 사람이 매번 따로 실행했는데(align_and_grasp.py 돌리고,
joint_nudge로 내리고, gripper_hold로 닫고...), 반복 성공률을 재려면 한 번에
끝까지 가는 게 필요해서 묶은 것(2026-08-26).

내부적으로 align_and_grasp.py를 그대로 재사용한다 — 정렬 로직을 여기 복사하지
않고 서브프로세스로 부른다(그 파일이 이 프로젝트에서 계속 튜닝되는 곳이라,
복사해두면 두 벌이 갈라진다).

사전조건: robot_bridge.py / fixed_cam_server.py 가 떠 있어야 함.

사용법:
  python scripts/pick_cycle.py --color blue            # 1회
  python scripts/pick_cycle.py --color blue --repeat 5 # 5회 반복(사이마다 확인 대기)
"""
import argparse
import subprocess
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).parent.parent / 'src'))
from lekiwi_pill_pickup.pick_result import write_result_file  # noqa: E402

# ⚠️ 르키위 파이는 DHCP라 IP가 바뀐다(2026-08-27에 실제로 바뀌어 전부 실패했다).
# 여기 한 곳에서만 정하고, lekiwi.sh 가 --lekiwi-host 로 실제 값을 넘긴다.
LEKIWI_HOST = os.environ.get('LEKIWI_HOST', '192.168.0.201')

from color_detect import (  # noqa: E402
    find_bottles_on_board, find_color_blobs, find_white_marker,
)
from fixed_cam_server import fixed_frame_available, read_fixed_frame  # noqa: E402
from fixed_cam_server import read_fixed_frame
from robot_link import RobotLink, bridge_available  # noqa: E402

JOINTS = ['arm_shoulder_pan', 'arm_shoulder_lift', 'arm_elbow_flex',
          'arm_wrist_flex', 'arm_wrist_roll', 'arm_gripper']

# 2026-08-26 실기기에서 손으로 맞춰 성공한 파지 자세(그리퍼 제외).
# 정렬이 베이스를 매번 같은 상대위치로 맞춰준다는 전제로 재사용한다.
# shoulder_lift 60.04로 내리면 병의 아래쪽(펌프 노즐 쪽, 가늘어서 잘 빠지는
# 부분)을 물어서 들어올리다 떨어뜨리는 일이 있었다(2026-08-26 실측) —
# 덜 내려가서 병 몸통을 물도록 조정.
# arm_shoulder_lift: 클수록 더 깊이 내려간다. 2026-08-26 실측(각 1회):
#   56.0  -> 공중을 잡음(그리퍼 18.1 = 빈손)
#   60.04 -> 성공(그리퍼 19.2, 원래 자리 비워짐)
#   60.84 -> 병을 눌러 쓰러뜨림(42px 밀림)
# ⚠️ 단 도달값은 60.1 / 60.2로 거의 같았다 — 즉 그 차이를 만든 진짜 변수는
#    깊이가 아니라 정렬오차(7px vs 14px)였을 가능성이 크다.
# 명령값보다 항상 0.6~0.8도쯤 덜 내려가므로(하중/마찰), 더 깊이 가려면 명령을
# 그만큼 올려서 준다. 현장에서 바꿔볼 땐 --grasp-lift 로 조절할 것.
GRASP_POSE = dict(arm_shoulder_pan=-1.49, arm_shoulder_lift=62.04,
                  arm_elbow_flex=-11.56, arm_wrist_flex=45.54,
                  arm_wrist_roll=-5.49)
HOME_POSE = dict(arm_shoulder_pan=2.0, arm_shoulder_lift=-98.7,
                 arm_elbow_flex=99.0, arm_wrist_flex=74.8, arm_wrist_roll=-51.5)
SEARCH_POSE = dict(arm_shoulder_pan=-2.02, arm_shoulder_lift=7.30,
                   arm_elbow_flex=9.01, arm_wrist_flex=59.87, arm_wrist_roll=-5.14)

GRIPPER_OPEN = 31.0
# 17.7로 닫으면 실제로는 18.8쯤에서 멈춰 살짝 헐거웠음 — 사용자 판단으로
# 17.9까지 확실히 물리도록 조정(2026-08-26)
# 작을수록 세게 문다(31.0=열림). 2026-08-26: 17.9로 성공했지만 사용자가
# "아주 약간만 더 세게"를 요청해 17.0으로 내림. 실제 도달값은 병에 막혀
# 19.2쯤에서 멈추므로 파지 판정(GRASP_DETECT_MARGIN)에는 영향이 없다.
GRIPPER_CLOSED = 17.0
# 닫으라고 명령한 값보다 이만큼 이상 덜 닫혔으면 "뭔가 물려서 못 닫힌 것"으로
# 보고 파지 성공으로 판정한다. 빈손이면 명령값 근처까지 그냥 닫혀버린다.
GRASP_DETECT_MARGIN = 0.6
# 집은 뒤에도 병이 보드에서 보이는데 원래 자리에서 이만큼 넘게 움직였으면
# "잡은 게 아니라 밀어서 넘어뜨린 것"으로 본다(그래도 결론은 실패).
BOTTLE_DISTURBED_PX = 25
# 들어올린 뒤 병이 화면에서 이만큼 위로 올라갔으면 같이 들린 것으로 본다
LIFTED_MIN_DY_PX = 20
# 원래 자리에서 이 반경 안에 병이 보이면 "아직 보드에 있다"로 본다
BOTTLE_SAME_SPOT_PX = 70
# 홈으로 곧장 휘두르면 병이 빠지므로, 먼저 이만큼만 수직으로 든다
LIFT_DELTA_DEG = 20.0
HOME_RETREAT_SEC = 7.0


def interpolate(client, target: dict, duration: float = 4.0,
                keep_gripper: bool = False) -> dict:
    obs = client.get_observation()
    start = {j: float(obs[f'{j}.pos']) for j in JOINTS}
    goal = dict(target)
    if keep_gripper or 'arm_gripper' not in goal:
        goal['arm_gripper'] = start['arm_gripper']

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


def hold_gripper(client, value: float, seconds: float = 2.5) -> float:
    obs = client.get_observation()
    action = {f'{j}.pos': float(obs[f'{j}.pos']) for j in JOINTS}
    action['arm_gripper.pos'] = value
    action['x.vel'] = 0.0
    action['y.vel'] = 0.0
    action['theta.vel'] = 0.0
    for _ in range(int(seconds / 0.1)):
        client.send_action(action)
        time.sleep(0.1)
    return float(client.get_observation()['arm_gripper.pos'])


def target_visible(color: str) -> bool:
    frame = read_fixed_frame()
    if frame is None:
        return False
    return bool(find_color_blobs(frame, color, min_area=150))


def run_alignment(color: str, script_dir: Path, no_parallax: bool = False) -> bool:
    """align_and_grasp.py를 그대로 실행. 정렬이 다 되면 집게가 물체를 덮어
    '물체를 못 찾음'으로 끝나는 경우가 있는데(실측 2026-08-26), 그건 실패가
    아니라 이미 정렬됐다는 신호에 가깝다 — 호출부에서 판단하도록 그대로 반환.

    no_parallax: 가운데(기준) 병을 맞출 때는 반드시 True로 불러야 한다
    (2026-08-27 실측) — 시차계수(k=1.62)는 '기준점에서 벗어난 만큼'을
    증폭하는 식이라, 기준점 자체인 가운데 병에 적용하면 그냥 프레임마다
    생기는 자연스러운 검출 흔들림(실측 314->300, 14px)까지 62%나 증폭해서
    (290,336) 같은 엉뚱한 목표로 밀어버린다. 시차보정은 가운데에서 옆으로
    옮길 때(move_marker_to_bottle)만 필요하다.
    """
    cmd = [sys.executable, '-u', str(script_dir / 'align_and_grasp.py'),
           '--color', color, '--no-show']
    if no_parallax:
        cmd.append('--no-parallax')
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=180)
    print(result.stdout, end='')
    if result.stderr.strip():
        print(result.stderr, end='', file=sys.stderr)
    return '정렬 완료' in result.stdout


def bottle_position(color: str, retries: int = 4):
    """목표 병의 중심 좌표. 안 보이면 None.

    팔이 막 움직인 직후엔 모션블러/프레임 지연으로 한 프레임만 보면 놓치는
    경우가 있다(2026-08-26 실측: 실제로 잡아 들었는데 '못 찾음'으로 오판) —
    몇 번 다시 본다."""
    for _ in range(retries):
        frame = read_fixed_frame()
        if frame is not None:
            # 판 영역·화면가장자리·밝기우선 필터가 든 검출을 쓴다 — raw
            # find_color_blobs 는 나무 바닥/반사광을 '병'으로 잡아 판정을
            # 뒤집었다(2026-08-27 실측: 걸이에서 빠졌는데 (16,441) 바닥
            # 얼룩을 '아직 있음'으로 오판). 정렬 경로(_fixed_bottle)와 동일 검출.
            blobs = find_bottles_on_board(frame, color)
            if blobs:
                x1, y1, x2, y2, _ = blobs[0]
                return ((x1 + x2) / 2, (y1 + y2) / 2)
        time.sleep(0.3)
    return None


def bottle_near(color: str, spot, radius: float, retries: int = 4):
    """`spot` 근처에 그 색 병이 아직 있으면 그 좌표를, 없으면 None.

    '들어올렸나'를 병을 쫓아다니며 판정하는 대신 '원래 자리가 비었나'로
    판정하기 위한 것 — 팔에 가려 병이 안 보이는 상황에 훨씬 강하다."""
    for _ in range(retries):
        frame = read_fixed_frame()
        if frame is not None:
            # bottle_position 과 동일 이유로 판 영역 필터가 든 검출을 쓴다.
            for x1, y1, x2, y2, _ in find_bottles_on_board(frame, color):
                cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
                if ((cx - spot[0]) ** 2 + (cy - spot[1]) ** 2) ** 0.5 <= radius:
                    return (cx, cy)
        time.sleep(0.3)
    return None


def marker_visible() -> bool:
    frame = read_fixed_frame()
    return frame is not None and find_white_marker(frame) is not None


def ensure_marker_visible(script_dir: Path, max_nudges: int = 4,
                          target_color: str | None = None) -> str:
    """집기 전에 매번 **베이스 정렬부터 다시** 한다.

    순서(2026-08-26 사용자 정의):
      1) 베이스캠에 좌·중·우 3개가 다 보이고 좌우 끝에 딱 붙을 때까지 조향+접근
         (3개가 다 안 보이면 너무 가까운 것 -> 물러난다)
      2) 팔을 파지 준비 자세로 든다
      3) 고정캠에서 손목 흰 마커가 보일 때까지 조금씩 전진

    예전엔 마커만 찾고 베이스 위치는 안 봤는데, 사이클을 반복할수록 베이스가
    옆으로 밀려서 결국 팔이 화면 구석에 처박혔다(2026-08-26 실측: 마커가
    x=8, x=138 같은 프레임 가장자리에서 잡히고 정렬이 시작도 못 함).
    """
    sys.path.insert(0, str(script_dir))
    import approach_board as ab

    class _Args:
        fwd_vel = 0.07
        theta_vel = 22.0
        creep_vel = 0.045
        creep_sec = 0.25
        # MARKER_SAFE_Y(2026-08-27 도입, 180으로 상향) 때문에 '겨우 보임'이
        # 아니라 '충분히 안쪽까지 들어와야' 통과한다 — 시도횟수도 늘린다.
        marker_tries = 14
        # ⚠️ 20으로는 부족할 수 있다(2026-08-27 실측) — 판에 완전히 붙은
        # 상태(여백 0)에서는 균형 교정을 안전하게(느리게) 하느라 -64%에서
        # -14%까지 오는 데만 18스텝이 걸렸다(진동은 없이 매끈하게 줄었지만
        # 느림). 남은 회전까지 마칠 여유를 준다.
        max_steps = 35
        search_tries = 12

    args = _Args()
    client = RobotLink(LEKIWI_HOST)
    client.connect()
    try:
        if not ab.back_off_until_visible(client, args):
            return 'fail'
        if not ab.align_base(client, args):
            return 'fail'
        # 판 전체에 정렬한 뒤, 목표 병 앞으로 한 번 더 옮긴다.
        # (2026-08-27: 이게 시차보정을 대신한다 — approach_board 주석 참고)
        # ■ 예전엔 여기서 베이스를 목표 병 앞으로 옮겼는데(center_on_target)
        #   그 방식은 판 정렬을 무너뜨리고 마커도 안 보이게 만들었다.
        #   지금은 판 가운데에 선 채로 마커를 확보하고, 목표 병으로는
        #   나중에 마커를 보면서 옆걸음한다(move_marker_to_bottle).
        if ab.raise_arm_and_find_marker(client, args):
            return 'ok'
        return 'fail'
    finally:
        client.disconnect()


def one_cycle(color: str, script_dir: Path, grasp_lift: float | None = None,
              stop_after_align: bool = False, grasp_only: bool = False,
              no_open_loop: bool = False) -> dict:
    # 파지 성공 판정을 그리퍼 위치로만 하면, 병을 실제로 잡은 경우와 병을
    # 밀어 넘어뜨리고 손가락만 걸린 경우가 구분이 안 된다(2026-08-26 실측:
    # 둘 다 18.8에서 멈춰 "잡음"으로 오판, 실제로는 병이 보드에 그대로
    # 남아있거나 옆으로 넘어져 있었음). 그래서 집기 전 병 위치를 기억해두고,
    # 들어올린 뒤 "그 자리에서 실제로 사라졌는지"까지 확인한다.
    grasp_pose = dict(GRASP_POSE)
    if grasp_lift is not None:
        grasp_pose['arm_shoulder_lift'] = grasp_lift
        print(f'  하강 깊이 지정: shoulder_lift={grasp_lift} '
              f'(기본 {GRASP_POSE["arm_shoulder_lift"]})')

    before = bottle_position(color)
    if before is None:
        print(f'  -> 시작 전 {color} 병을 못 찾음 — 사이클 중단')
        return {'success': False, 'grasped': False, 'gripper_grasped': False,
                'verdict_reason': '집기 전 병을 못 찾음'}
    print(f'  집기 전 {color} 병 위치: ({before[0]:.0f},{before[1]:.0f})')

    if grasp_only:
        # ■ --stop-after-align 로 멈춰둔 자리에서 하강만 이어서 한다
        #   (2026-08-27) — 정렬을 처음부터 다시 하면 이미 맞춰둔 위치가
        #   쓸모없어진다. 로봇/팔은 지금 있는 그대로 두고 바로 하강한다.
        print('[1-2/5] 건너뜀 — 지금 자세에서 바로 하강 (--grasp-only)')
    else:
        client = RobotLink(LEKIWI_HOST)
        client.connect()
        try:
            print('[1/5] search_pose로 이동')
            interpolate(client, {**SEARCH_POSE, 'arm_gripper': GRIPPER_OPEN}, 3.0)
        finally:
            client.disconnect()

        status = ensure_marker_visible(script_dir, target_color=color)
        if status == 'fail':
            print('  -> 베이스를 목표 병 앞으로 못 가져감 — 이번 사이클 실패')
            return {'success': False, 'grasped': False, 'gripper_grasped': False,
                    'verdict_reason': '베이스를 목표 병 앞으로 못 가져감'}

        # ■ 시차 대응: 가운데 병에 먼저 맞추고, 거기서 목표 병으로 옆걸음한다
        #   (2026-08-27 사용자 제안). 가운데 병에서는 마커↔집게 오차가 0에 가까워서
        #   마커 정렬이 정확하다. 그 자세 그대로 옆으로만 움직이면 깊이 관계가
        #   유지되므로, 가운데에서 맞았던 조건이 목표 병에서도 그대로 성립한다.
        import approach_board as _ab
        mid = _ab.middle_color() or 'green'
        # 팔을 들기 전에 병 좌표를 확정해둔다(팔이 가리면 검출이 흔들린다)
        frozen = {c: _ab._fixed_bottle(c) for c in ('red', 'green', 'blue')}
        print(f'  병 좌표 확정: ' + '  '.join(
            f'{c}={"(%.0f,%.0f)" % v if v else "없음"}' for c, v in frozen.items()))
        # ■ 정렬을 시작하기도 전에 가려져 있으면 안 된다 (2026-08-27 실측)
        #   마커를 화면 안쪽까지 확실히 들여오려고 MARKER_SAFE_Y를 키웠더니
        #   (70->180) 그만큼 더 전진해서 이번엔 초록이 아예 안 보이는 채로
        #   시작했다 — align_and_grasp이 첫 프레임에서 목표를 못 찾아 죽고,
        #   그걸 '가려서 정렬된 것'으로 오판해 정렬을 통째로 건너뛰었다.
        #   (그 예외처리는 원래 '정렬 다 된 뒤 집게가 덮은' 경우를 위한
        #   것이지, '시작부터 안 보인' 경우가 아니다.) 시작 전에 실제로
        #   보이는지 먼저 확인하고, 안 보이면 살짝 물러나 드러낸다.
        client = RobotLink(LEKIWI_HOST)
        client.connect()
        try:
            for _ in range(3):
                if _ab._fixed_bottle(mid) is not None:
                    break
                print(f'    가운데 병({mid})이 시작부터 가려짐 — 살짝 물러남')
                _ab.pulse(client, vx=-0.045, duration=0.25, hold_arm=_ab.SEARCH_POSE)
        finally:
            client.disconnect()

        print(f'[2/5] 먼저 가운데 병({mid})에 마커 정렬')
        # align_and_grasp.py가 자체적으로 연결하므로 위에서 반드시 끊고 부른다
        aligned = run_alignment(mid, script_dir, no_parallax=True)
        if aligned and color != mid:
            client = RobotLink(LEKIWI_HOST)
            client.connect()
            try:
                class _A:
                    fwd_vel = 0.05
                    theta_vel = 22.0
                    max_steps = 16
                _A.no_open_loop = no_open_loop
                mid_xy = frozen.get(mid)
                center_x = mid_xy[0] if mid_xy else 320.0
                if not _ab.move_marker_to_bottle(client, color, _A(),
                                                 target=frozen.get(color),
                                                 center_x=center_x):
                    print(f'  -> 마커를 {color} 병 위로 못 옮김 — 이번 사이클 실패')
                    return {'success': False, 'grasped': False,
                            'gripper_grasped': False,
                            'verdict_reason': f'마커를 {color} 병 위로 못 옮김'}
            finally:
                client.disconnect()
        if not aligned:
            # ⚠️ 여기서 봐야 할 건 '집기 목표(color)'가 아니라 방금 정렬을
            #   시도한 대상(mid) 이다 — run_alignment(mid, ...) 가 실패했으니,
            #   '가려서 정렬된 것으로 판단'하는 예외도 mid 기준이어야 앞뒤가
            #   맞는다(2026-08-27 코드리뷰로 발견, color 기준이면 둘이 무관한
            #   병을 섞어 보는 것이라 이 판정이 사실상 무의미했다).
            if target_visible(mid):
                print('  -> 정렬 미완료이고 목표(가운데)도 아직 보임 — 이번 사이클 실패로 처리')
                return {'success': False, 'grasped': False,
                        'gripper_grasped': False,
                        'verdict_reason': '초록 정렬 미완료'}
            print('  -> 목표(가운데)가 집게에 가려 안 보임(=정렬된 것으로 판단) — 계속 진행')

    if stop_after_align:
        # ■ 내려가기 전에 사람이 확인/보정할 수 있게 여기서 멈춘다
        #   (2026-08-27 사용자 제안). 시차보정 계수를 맞출 때 특히 유용하다 —
        #   내려간 뒤에는 집게가 병을 가려서 얼마나 어긋났는지 볼 수가 없다.
        #   여기서 고정캠을 보고 마커 위치를 재고, base_nudge.py 로 조금
        #   움직인 뒤 --grasp-only 로 하강만 따로 실행하면 된다.
        from color_detect import find_white_marker
        frame = read_fixed_frame()
        mk = None if frame is None else find_white_marker(frame)
        now = bottle_position(color)
        print('[정지] 하강 직전에서 멈춤 (--stop-after-align)')
        print(f'  마커={mk}  {color} 병={now}')
        if mk is not None and now is not None:
            print(f'  마커 - 병 = ({mk[0]-now[0]:+.0f}, {mk[1]-now[1]:+.0f}) px')
        print('  이어서 하강하려면:  ./scripts/lekiwi.sh grasp ' + color)
        return {'success': False, 'grasped': False, 'gripper_grasped': False,
                'verdict_reason': 'stop-after-align (하강 안 함)'}

    client = RobotLink(LEKIWI_HOST)
    client.connect()
    try:
        print('[3/5] 파지 자세로 하강(그리퍼는 열어둔 채)')
        interpolate(client, {**grasp_pose, 'arm_gripper': GRIPPER_OPEN}, 4.0)
        # 명령만 보내고 실제 도달값을 안 보면 "덜 내려가서 공중을 잡는" 상황을
        # 로그만 봐서는 알 수가 없다(2026-08-26 사용자 보고) — 관절별로 찍는다.
        reached_arm = client.get_observation()
        gap = {j: float(reached_arm[f'{j}.pos']) - grasp_pose[j]
               for j in grasp_pose}
        print('  하강 도달값: ' + '  '.join(
            f'{j.replace("arm_", "")}={float(reached_arm[f"{j}.pos"]):.1f}'
            f'(목표{grasp_pose[j]:.1f}{gap[j]:+.1f})' for j in grasp_pose))
        worst = max(gap, key=lambda j: abs(gap[j]))
        if abs(gap[worst]) > 5.0:
            print(f'  ⚠️ {worst.replace("arm_", "")}가 목표에서 {gap[worst]:+.1f}도 '
                  '벗어남 — 공중을 잡을 수 있습니다(모터가 하중/마찰로 못 따라감)')

        print('[4/5] 그리퍼 닫기')
        reached = hold_gripper(client, GRIPPER_CLOSED)
        gripper_says_grasped = reached > GRIPPER_CLOSED + GRASP_DETECT_MARGIN
        print(f'  그리퍼 도달값 {reached:.1f} (명령 {GRIPPER_CLOSED}) '
              f'-> {"뭔가 물림" if gripper_says_grasped else "빈손으로 보임"}')

        # 곧장 홈으로 가면 어깨가 60도->-98.7도로 160도 가까이 휘둘러서,
        # 잡았던 병이 관성으로 빠져 떨어지는 일이 실제로 있었다(2026-08-26).
        # 먼저 수직으로 살짝만 들어 보드에서 떼고, 그 상태에서 검증한 뒤
        # 천천히 홈으로 간다.
        print('[5/7] 수직으로 살짝 들어올리기')
        obs = client.get_observation()
        lift = {j: float(obs[f'{j}.pos']) for j in JOINTS}
        lift['arm_shoulder_lift'] -= LIFT_DELTA_DEG
        interpolate(client, lift, 2.0, keep_gripper=True)

        print('[6/7] 홈으로 물러나기(천천히, 잡은 상태 유지)')
        interpolate(client, HOME_POSE, HOME_RETREAT_SEC, keep_gripper=True)
    finally:
        client.disconnect()

    # 파지 판정: "들어올린 병이 어디 있나"가 아니라 "보드의 원래 자리가
    # 비었나"로 본다. 들어올린 직후엔 위에서 볼 때 팔 몸체가 병을 가려서
    # 병을 못 찾는 일이 잦았고(2026-08-26 실측), 그 오판이 "실패 -> 내려놓기
    # 생략 -> 병을 든 채 다음 사이클 시작"으로 연쇄되며 5회 전부 망가졌다.
    # 홈까지 물러나면 팔이 보드 위에서 비켜나므로 원래 자리를 깨끗하게 볼 수 있다.
    time.sleep(1.0)
    still_there = bottle_near(color, before, BOTTLE_SAME_SPOT_PX)
    if still_there is None:
        print(f'  판정: 원래 자리({before[0]:.0f},{before[1]:.0f})가 비어있음 '
              f'-> 들어올린 것으로 판정')
        grasped = True
    else:
        moved = ((still_there[0] - before[0]) ** 2
                 + (still_there[1] - before[1]) ** 2) ** 0.5
        print(f'  판정: 병이 아직 보드에 있음 ({still_there[0]:.0f},'
              f'{still_there[1]:.0f}, {moved:.0f}px 이동) -> 실패')
        grasped = False

    # ⚠️ 그리퍼는 반드시 보드 위(파지 자세)에서만 연다. 예전 코드는 홈으로
    # 갈 때 목표 자세에 GRIPPER_OPEN이 섞여 있어서 홈에서 그리퍼가 열렸고,
    # 들고 있던 병을 로봇 위쪽에서 그대로 떨어뜨렸다(2026-08-26 실제 발생).
    # 판정이 실패로 나왔더라도(오판일 수 있으므로) 똑같이 보드 위에서 놓는다 —
    # 빈손이면 그냥 열렸다 닫히는 것뿐이라 무해하다.
    # 되돌려놓기 로직은 제거했다(2026-08-26 사용자 요청: "전혀 필요없어").
    # 집은 병은 홈에서 든 채로 끝낸다 — 사람이 받아가면 된다.
    # 되돌려놓기 로직은 여기서 끝난다(2026-08-26 사용자 요청: "전혀 필요없어").
    # (예전엔 이 return 뒤에 죽은 코드 — 절대 실행 안 되는 '내려놓고 홈으로'
    # 블록 — 가 그대로 남아 있었다. 주석은 "제거했다"고 했는데 코드는 안
    # 지워져 있었다 — 2026-08-27 코드리뷰로 발견, 정리함.)
    reason = ('원래 자리가 비어있음' if grasped
              else '병이 아직 보드에 있음')
    return {'success': grasped and gripper_says_grasped,
            'grasped': grasped,
            'gripper_grasped': gripper_says_grasped,
            'verdict_reason': reason}


def ensure_servers(script_dir: Path, lekiwi_host: str, wait_sec: float = 25.0) -> None:
    """고정캠 서버와 로봇 브릿지가 안 떠 있으면 여기서 띄운다.

    사람이 터미널에서 돌릴 때는 미리 띄워두면 되지만, 에이보가 음성명령으로
    SSH 트리거할 때는 아무도 미리 안 띄워준다 — 그 경우 지금까지는 "프레임을
    못 읽음"으로 그냥 죽었다. 이미 떠 있으면 건드리지 않는다(중복 실행하면
    카메라 장치와 LeKiwi 연결을 두 프로세스가 다투게 되므로).

    여기서 띄운 프로세스는 일부러 죽이지 않고 남겨둔다 — 다음 호출에서 재사용
    되고, 매번 띄우면 LeKiwi 연결 오버헤드(약 9초)를 계속 다시 문다.
    """
    started = []
    if not fixed_frame_available():
        print('고정캠 서버가 안 떠 있어 여기서 띄웁니다')
        started.append(subprocess.Popen(
            [sys.executable, '-u', str(script_dir / 'fixed_cam_server.py')],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True))
    if not bridge_available():
        print('로봇 브릿지가 안 떠 있어 여기서 띄웁니다')
        started.append(subprocess.Popen(
            [sys.executable, '-u', str(script_dir / 'robot_bridge.py'),
             '--lekiwi-host', lekiwi_host],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True))
    if not started:
        return

    deadline = time.time() + wait_sec
    while time.time() < deadline:
        if fixed_frame_available() and bridge_available():
            print('선행 서버 준비 완료')
            return
        time.sleep(0.5)
    missing = []
    if not fixed_frame_available():
        missing.append('고정캠 서버(USB 카메라 연결 확인)')
    if not bridge_available():
        missing.append(f'로봇 브릿지(LeKiwi {lekiwi_host} 전원/네트워크 확인)')
    raise RuntimeError(f'{wait_sec:.0f}초 안에 준비되지 않음: {", ".join(missing)}')


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument('--color', required=True,
                   choices=('red', 'green', 'blue'))
    p.add_argument('--repeat', type=int, default=1)
    p.add_argument('--lekiwi-host', default=LEKIWI_HOST)
    p.add_argument('--stop-after-align', action='store_true',
                   help='하강 직전에 멈춘다(시차보정용 — 사람이 보고 보정)')
    p.add_argument('--grasp-only', action='store_true',
                   help='정렬 없이 지금 자세에서 바로 하강+집기 '
                        '(--stop-after-align 으로 멈춰둔 뒤 이어서 실행)')
    p.add_argument('--no-autostart', action='store_true',
                   help='선행 서버를 자동으로 띄우지 않음(직접 관리할 때)')
    p.add_argument('--no-open-loop', action='store_true',
                   help='옆걸음 뒤 오픈루프 넛지를 생략(Δ 재측정/순수 서보 디버깅용)')
    p.add_argument('--grasp-lift', type=float, default=None,
                   help=f'하강 깊이(arm_shoulder_lift, 기본 {GRASP_POSE["arm_shoulder_lift"]}). '
                        '클수록 더 깊이 내려간다 — 공중을 잡으면 올리고, '
                        '보드를 찍으면 내릴 것')
    p.add_argument('--result-file', default='',
                   help='집기 판정 결과를 이 경로에 JSON으로 남긴다(에이보 연동용)')
    args = p.parse_args()
    # 아래 함수들이 모듈 상수를 쓰므로, 넘겨받은 값으로 한 번에 갱신한다.
    globals()['LEKIWI_HOST'] = args.lekiwi_host

    script_dir = Path(__file__).parent
    if not args.no_autostart:
        ensure_servers(script_dir, args.lekiwi_host)
    results = []
    for i in range(1, args.repeat + 1):
        print(f'\n===== 사이클 {i}/{args.repeat} =====')
        t0 = time.time()
        try:
            res = one_cycle(args.color, script_dir, grasp_lift=args.grasp_lift,
                            stop_after_align=args.stop_after_align,
                            grasp_only=args.grasp_only,
                            no_open_loop=args.no_open_loop)
        except Exception as e:
            print(f'사이클 실패: {type(e).__name__}: {e}', file=sys.stderr)
            res = {'success': False, 'grasped': False, 'gripper_grasped': False,
                   'verdict_reason': f'예외: {type(e).__name__}: {e}'}
        ok = res['success']
        results.append(ok)
        if args.result_file:
            write_result_file(args.result_file, {
                'skill': 'pick_pill_bottle', 'color': args.color,
                'elapsed_sec': round(time.time() - t0, 1), **res})
        print(f'===== 사이클 {i} 결과: {"성공" if ok else "실패"} =====')

        # 성공한 사이클은 로봇이 스스로 제자리에 돌려놓으므로 사람 개입이
        # 필요 없다. 실패했을 때만(병이 넘어졌을 수 있으므로) 확인을 받는다.
        if not ok and i < args.repeat:
            if sys.stdin.isatty():
                input('실패 — 병을 원래 자리에 세우고 엔터를 누르세요...')
            else:
                print('실패 — 병 상태를 확인하세요(비대화형이라 그대로 진행)')

    n_ok = sum(results)
    print(f'\n총 {len(results)}회 중 {n_ok}회 성공 '
          f'({n_ok / len(results) * 100:.0f}%) — {results}')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\n중단됨')
    except Exception as e:
        print(f'실패: {type(e).__name__}: {e}', file=sys.stderr)
        sys.exit(1)
