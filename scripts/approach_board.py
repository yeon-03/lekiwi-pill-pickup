#!/usr/bin/env python
"""르키위가 스스로 판 앞으로 가서, 팔을 들고, 손목 흰 마커까지 확보한다.

사용자가 정의한 절차 그대로다(2026-08-26):
  1) 베이스캠으로 접근 — 병 3개가 화면 **양쪽 사이드까지 꽉 차게** 들어오면
     판에 충분히 가까이 온 것으로 본다
  2) 판에 붙었으면 팔을 파지 준비 자세(search_pose)로 든다
  3) 고정캠에서 손목 흰 마커를 찾는다
  4) (이 뒤는 align_and_grasp.py 담당) 약통 바운딩박스와 흰 마커를 맞춘다

■ 왜 "양쪽이 꽉 찼는가"를 기준으로 쓰는가
처음엔 베이스캠의 "어두운 픽셀 비율"로 판까지 거리를 쟀는데 완전히 실패했다
(2026-08-26 실측: 책상 모서리·모니터·종이상자가 전부 어둡게 잡혀서 비율이
21%->10%->3%->20%->33%로 널뜀). 병 3개는 이 장면에서 유일하게 확실히 구분되는
표적이고, **가까이 갈수록 화면에서 좌우로 벌어지므로** 벌어진 폭이 그대로
거리 신호가 된다. 병 위치가 고정이라는 전제가 있어 더 잘 맞는다.

■ 부호를 가정하지 않는 이유
LeKiwi 베이스에는 오도메트리도 헤딩 센서도 없다. 사람이 손으로 옮기거나 방향을
모른 채 밀면 "+x가 전진"이 더 이상 참이 아니다. 실제로 이날 그 상태에서 정렬이
통째로 실패했다 — 방향보정 결과가 x.vel/y.vel 둘 다 마커를 가로로만 움직여서,
세로로 300px 내려가야 하는데 갈 방법이 아예 없었다. 그래서 회전/전진 부호를
매번 작은 테스트 펄스로 실측해서 쓴다(align_and_grasp.py와 같은 방식).
"""
import argparse
import json
import hashlib
import sys
import time
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).parent))
from color_detect import find_color_blobs, find_white_marker  # noqa: E402
from robot_link import RobotLink  # noqa: E402

sys.path.insert(0, str(Path(__file__).parent.parent / 'src'))
from lekiwi_pill_pickup import sideways_nudge  # noqa: E402

FRAME_DIR = Path('/dev/shm/lekiwi_cam')
# 노랑 병은 안 쓴다(2026-08-27 사용자 확정) — 빨강/초록/파랑 3개만.
BOTTLE_COLORS = ('red', 'green', 'blue')
# 병은 판 위에 놓여 있으므로 베이스캠 화면의 **아래쪽**에만 있다. 위쪽에 잡히는
# 건 사람·상자·검은 판 반사 같은 가짜다(2026-08-27 실측: 가짜들의 세로중심이
# 화면의 17~37% 지점, 진짜 병은 63~65% 지점).
BOTTLE_MIN_Y_RATIO = 0.45
# 병 3개는 크기가 비슷하다. 어떤 색의 박스가 나머지의 이 배율을 넘으면
# 배경(바닥·판 반사)이 붙어버린 것이므로 그 색의 다음 후보를 쓴다.
# (2026-08-27 실측: 로봇이 멈춰 있는데도 오른쪽 박스가 35 <-> 136px 로 널뛰어
#  균형이 -9% <-> -74% 로 요동치며 한쪽으로만 계속 게걸음했다.)
BOX_OUTLIER_RATIO = 2.5
# 병 3개는 같은 판 위에 나란히 있으므로 화면에서의 **세로 위치도 비슷하다**.
# 세로중심이 다른 병들의 중앙값에서 이 비율(화면높이 대비)만큼 벗어나면
# 병이 아니라 바닥·판 모서리 같은 것이다.
# (2026-08-27 실측: 나무 바닥이 빨강으로 잡혀 x31~309(폭 278) 박스가 생겼는데
#  세로중심이 412 로, 진짜 병 3개의 287 과 크게 달랐다.)
BOX_Y_TOL_RATIO = 0.18

# ■ "기본 위치"의 정의 (2026-08-26 사용자 확정)
#   베이스캠에 **좌·중·우 3개**의 바운딩박스가 다 보이고, 맨 왼쪽/맨 오른쪽
#   병이 화면 좌우 끝에 **딱 붙어야** 한다.
#   퍼짐 비율만 보던 예전 기준은 병이 2개만 보이는데도 통과해버렸다
#   (실측: "병 2개, 좌우퍼짐 83%"로 통과 -> 잘못된 위치에서 집기 시작).
REQUIRED_BOTTLES = 3
# 가운데 자리에 놓는 병의 색.
#   None  = 자동으로 알아낸다(기본). 3개가 깨끗하게 보이는 프레임에서
#           화면 가운데 박스의 색을 읽어 기억하고, 그 뒤로는 그 색만 쓴다.
#   'green' 처럼 직접 박아두면 그 색으로 고정된다(--middle-color).
# ■ 왜 '위치'가 아니라 '색'으로 기억하는가
#   반사광이 가짜 박스로 잡히거나 병 하나를 놓치면 '화면 가운데'는 매 프레임
#   다른 병을 가리킨다. 색은 안 변하므로, 위치는 처음 한 번만 믿고 그 뒤로는
#   색으로 추적한다. 병 배치를 바꾸면 다시 실행하기만 하면 새로 배운다.
MIDDLE_COLOR = None
_MID = {'color': None, 'vote': None, 'count': 0}
# 자동 판별을 믿기 전에 같은 답이 연속으로 이만큼 나와야 한다
MID_LATCH_FRAMES = 2
EDGE_MARGIN_PX = 35      # 좌우 끝에서 이 안쪽까지 와야 "꽉 찼다"로 본다
# ⚠️ 2026-08-27, 사용자 요청으로 대폭 완화: 베이스 정렬을 정밀하게
#   맞추는 데 매달리지 않고 '대충 자리 잡히면 바로 팔 들고 내려가서
#   시도'하는 쪽으로 바꾼다. 실패 원인은 대부분 정렬 오차가 아니라
#   하강 깊이(grasp_lift)였다 — 초록은 정밀 정렬 없이도 이미 성공했다.
# 병이 3개 다 안 보이는데 보이는 박스가 이만큼 크면 "너무 가까워서 시야 밖으로
# 나간 것"이다 -> 전진이 아니라 후진해야 한다(2026-08-26 사용자 지적).
# 여백만 보던 조건은 이 상황을 못 잡았다(실측: 병 2개, 좌여백 39px이라 통과).
TOO_CLOSE_BOX_W = 70
# 맨 왼쪽/맨 오른쪽 병의 박스 폭이 이 비율 이상 차이나면 로봇이 한쪽으로
# 치우쳐 비스듬히 보고 있는 것이다(가까운 쪽 병이 더 크게 찍힌다).
# 좌우로 게걸음(y.vel)해서 맞춘다 — 사용자 기준 추가(2026-08-26).
BALANCE_TOL = 0.40
# **가운데 병**이 화면중심에서 이 안에 들어오면 조향 완료
CENTER_TOL_PX = 70
# 병 3개는 실제로 같은 간격으로 놓여 있으므로, 로봇이 판을 정면으로 마주보면
# 화면상 박스 **중심 간격**도 같아진다(세 병의 깊이가 같아지기 때문).
# ■ 제어에는 안 쓰고 '완료 조건'으로만 쓰는 이유 (2026-08-27 실측)
#     자극            간격차    가운데오차   폭균형
#     회전 7.7도      -2.3%p    +110px      -0.6%p
#     게걸음 2.5cm    +0.9%p     +34px     -12.7%p
#   간격은 같은 오차를 폭균형의 1/6 크기로만 나타낸다 — 회전이 카메라 중심
#   기준이라 병까지의 깊이 차이가 2차 효과로만 생기기 때문이다. 그래서 움직임은
#   민감한 신호(가운데오차=회전, 폭균형=평행이동)로 만들고, 간격은 '정말
#   정면으로 섰는가'를 마지막에 확인하는 데 쓴다.
SPACING_TOL = 0.14
# 옆/가운데 폭 비율이 이보다 작으면 = 너무 가까움(가운데만 크게 보임).
#   ratio = 1/sqrt(1+(s/d)^2)  — s는 병 간격, d는 가운데 병까지 거리.
#   멀면 1에 가깝고 가까울수록 작아진다.
# ⚠️ 이 기본값은 화각 70도 가정에서 나온 추정치다. 실제로 성공한 위치에서
#    `lekiwi.sh status` 의 '옆/가운데' 값을 읽어 그 값보다 좀 낮게 잡을 것.
SIDE_RATIO_MIN = 0.55
# 바퀴 정지마찰 때문에 이보다 짧은 펄스는 실제로 안 움직일 수 있다.
# 비례제어가 짧은 시간을 계산해도 이 아래로는 안 내려간다 — 안 그러면
# "명령은 보냈는데 화면이 안 변함 -> 검출 흔들림으로 오판 -> 무한 재시도"가 된다.
MIN_PULSE_SEC = 0.18
# 마커가 이보다 위(작은 y)에 있으면 '가장자리에 겨우 걸침'으로 본다 — 다음
# 단계(align_and_grasp의 방향보정 테스트, 0.06m/s x 0.4초 펄스)가 살짝만
# 움직여도 화면 밖으로 나갈 위험이 있다. 그 테스트의 화면변환계수가 실측상
# 1m/s당 최대 ~5000px까지 나온 적이 있어(거리에 따라 요동) 0.024m짜리
# 작은 테스트펄스도 100px 넘게 밀 수 있다 — 70px 마진은 두 번 다 뚫렸다.
# 넉넉하게 잡는다.
MARKER_SAFE_Y = 180
MAX_PULSE_SEC = 0.45
# 게걸음은 미세조정이라 한 번에 크게 움직이면 시야가 통째로 밀려난다.
MAX_STRAFE_SEC = 0.28
# 게걸음 균형 개선속도의 실측 정상범위는 16~83%p/초였다. 이보다 훨씬 크게
# 나오면 검출 흔들림으로 인한 노이즈이지 실제 속도가 아니다 — 버린다.
BALANCE_RATE_SANITY_MAX = 1.0
# 여백이 이보다 작아지면 균형/회전이 서로 부수는 거리다(위 3순위 직전 참고).
FLUSH_GAP_PX = 6
# 3개가 안 보일 때, 보이는 무리의 중심이 화면중심에서 이만큼 벗어나 있으면
# '너무 가까워서' 가 아니라 '한쪽을 보고 있어서' 못 보는 것이다 -> 후진이 아니라 회전.
#   (2026-08-27 실측: 여백 좌0/우359px 인데 계속 후진만 하다 스텝을 다 씀.
#    병들이 화면 왼쪽 끝에 몰려 있었으니 왼쪽으로 돌았어야 했다.)
ROTATE_FIRST_PX = 60
# 후진을 이만큼 연속으로 했는데도 병 개수가 안 늘면, 거리 문제가 아니라
# 애초에 안 보이는 병이 있는 것이다(색 검출 실패, 병이 치워짐 등).
# 계속 물러나면 로봇이 판에서 아주 멀어져 아무것도 못 본다
# (2026-08-27 실측: 폭 82px -> 29px 까지 물러나다 전부 놓침).
MAX_CONSECUTIVE_BACKOFF = 6
# 시작 위치에서 이만큼(m/도) 넘게 움직였으면 뭔가 잘못된 것이다 — 멈춘다.
# (2026-08-27: 부호가 반대인 채로 루프가 돌면서 로봇이 판을 등지고 방을
#  가로질러 갔다. 그때 이 제한이 있었으면 몇 걸음 만에 멈췄을 것이다.)
MAX_TRAVEL_M = 0.9
MAX_TURN_DEG = 200.0
# 베이스캠 프레임이 이 시간 넘게 '내용이' 안 바뀌면 카메라가 멈춘 것이다.
# ⚠️ 파일 시각(mtime)으로 재면 안 된다 — 카메라가 죽어도 브릿지는 마지막
#    프레임을 계속 같은 파일에 다시 쓰기 때문에 시각은 계속 최신이다
#    (2026-08-27 실측: mtime 은 방금인데 내용은 5분째 동일).
FRAME_STALE_SEC = 6.0
MIN_BOTTLES = 2          # 이보다 적게 보이면 판단 자체를 못 한다
SETTLE_SEC = 0.7         # 펄스 후 관성이 멎고 프레임이 갱신될 때까지

JOINTS = ['arm_shoulder_pan', 'arm_shoulder_lift', 'arm_elbow_flex',
          'arm_wrist_flex', 'arm_wrist_roll', 'arm_gripper']
ARM_KEYS = [f'{j}.pos' for j in JOINTS]

# pick_cycle.py의 SEARCH_POSE와 같은 값이어야 한다(팔을 든 자세에서 마커를 찾으므로)
SEARCH_POSE = dict(arm_shoulder_pan=-2.02, arm_shoulder_lift=7.30,
                   arm_elbow_flex=9.01, arm_wrist_flex=59.87, arm_wrist_roll=-5.14,
                   arm_gripper=31.0)


def send_base(client, vx, vy, vtheta=0.0, hold_arm: dict | None = None):
    """hold_arm을 주면 그 자세를 계속 명령해서 붙잡는다.

    안 잡으면 팔이 중력으로 처져서, 애써 화면에 들여놓은 마커가 베이스를
    움직이는 사이에 다시 사라진다(2026-08-26 실측)."""
    obs = client.get_observation()
    action = {k: obs[k] for k in ARM_KEYS}
    if hold_arm:
        action.update({f'{j}.pos': hold_arm[j] for j in hold_arm})
    action.update({'x.vel': vx, 'y.vel': vy, 'theta.vel': vtheta})
    client.send_action(action)


# 시작점 대비 누적 이동량(명령 기준 추정치 — 오도메트리가 없어서 이게 최선)
_TRAVEL = {'x': 0.0, 'y': 0.0, 'theta': 0.0}


def travel_exceeded() -> str | None:
    d = (_TRAVEL['x'] ** 2 + _TRAVEL['y'] ** 2) ** 0.5
    if d > MAX_TRAVEL_M:
        return f'시작 위치에서 약 {d:.2f}m 이동'
    if abs(_TRAVEL['theta']) > MAX_TURN_DEG:
        return f'누적 회전 약 {_TRAVEL["theta"]:.0f}도'
    return None


def pulse(client, vx=0.0, vy=0.0, vtheta=0.0, duration=0.4, hold_arm=None):
    end = time.time() + duration
    while time.time() < end:
        send_base(client, vx, vy, vtheta, hold_arm)
        time.sleep(0.05)
    send_base(client, 0.0, 0.0, 0.0, hold_arm)
    time.sleep(SETTLE_SEC)
    _TRAVEL['x'] += vx * duration
    _TRAVEL['y'] += vy * duration
    _TRAVEL['theta'] += vtheta * duration


def interpolate(client, target: dict, duration: float = 3.0, steps: int = 40):
    obs = client.get_observation()
    start = {j: float(obs[f'{j}.pos']) for j in JOINTS}
    for i in range(1, steps + 1):
        t = i / steps
        action = {f'{j}.pos': start[j] + (target.get(j, start[j]) - start[j]) * t
                  for j in JOINTS}
        action.update({'x.vel': 0.0, 'y.vel': 0.0, 'theta.vel': 0.0})
        client.send_action(action)
        time.sleep(duration / steps)


def approach_state():
    """베이스캠에서 본 좌·중·우 병 3개의 상태.

    반환: dict
      n           보이는 병 개수
      mid_off     가운데 박스 중심 - 화면 중심 (px). 회전 오차를 뜻한다.
      left_gap    화면 왼끝 ~ 맨 왼쪽 박스 왼끝 (px)
      right_gap   맨 오른쪽 박스 오른끝 ~ 화면 오른끝 (px)
      w_left/w_mid/w_right   각 박스 폭 (px)
      max_w       가장 큰 박스 폭 (px)
      side_ratio  (좌우 평균) / 가운데. 가까울수록 작아진다. (n<3이면 None)

    ■ 왜 '가운데 박스'를 따로 쓰는가 (2026-08-27 사용자 확정)
      예전엔 병 무리 전체의 중심으로 조향했는데, 그건 좌우 평행이동에도
      같이 흔들려서 회전과 구분이 안 된다. 가운데 병 하나만 보면
        - 회전하면       -> 가운데 박스가 화면에서 좌우로 밀린다 (크기는 그대로)
        - 옆으로 평행이동 -> 좌우 박스 크기가 달라진다 (mid는 거의 그대로)
      로 두 자유도가 깔끔하게 분리된다.

    ■ side_ratio 가 거리 신호인 이유
      가운데 병까지 d, 좌우 간격 s면 옆 병까지는 sqrt(d^2+s^2)이고
      화면 크기는 거리에 반비례하므로  ratio = 1/sqrt(1+(s/d)^2).
      멀면 1에 가깝고(셋 다 비슷) 가까울수록 작아진다(가운데만 커진다).
      여백은 병이 화면 밖으로 나가면 0에서 포화돼 더 못 쓰지만, 이 값은
      3개가 보이는 동안 계속 변해서 '나가기 전에' 너무 가까움을 잡아낸다.
    """
    img = cv2.imread(str(FRAME_DIR / 'front.jpg'))
    if img is None:
        return None
    h, w = img.shape[:2]
    # ⚠️ 색마다 **가장 큰 덩어리 하나만** 쓴다. 병은 색당 정확히 1개인데,
    # 예전엔 모든 덩어리를 다 세어서 반사광·케이블 때문에 "병 4개", "병 8개"가
    # 나왔고(2026-08-26 실측) 그러면 좌우 끝 박스가 엉뚱한 걸 가리켜서 균형/접근
    # 로직이 통째로 헛돌았다. min_area도 80은 너무 낮아 노이즈를 주웠다.
    cands = {}
    for color in BOTTLE_COLORS:
        # ⚠️ max_area 를 키우면 안 된다. 예전엔 200000 으로 덮어써서 배경인
        # 검은 판(면적 138594)이 병으로 잡혔다 — color_detect 의 기본값
        # MAX_AREA_PX(25000)가 바로 그걸 막으려고 있는 값이다.
        # profile='base' — 어두운 베이스캠 전용 임계값(검은 판·나무 바닥을
        # 병으로 안 잡도록 채도 하한이 높다). 고정캠은 조명이 달라서
        # 같은 값을 쓰면 오히려 진짜 병을 놓친다(color_detect.py 주석 참고).
        blobs = [b for b in find_color_blobs(img, color, min_area=400,
                                             profile='base')
                 if (b[1] + b[3]) / 2 >= h * BOTTLE_MIN_Y_RATIO]
        if blobs:
            cands[color] = blobs                # find_color_blobs는 면적 내림차순

    # ■ 색만 보고 가장 큰 덩어리를 그대로 쓰면 배경에 속는다.
    #   병 3개는 (1) 크기가 비슷하고 (2) 같은 판 위라 세로 위치도 비슷하다.
    #   그래서 '색마다 첫 후보'로 폭·세로위치의 중앙값을 잡고, 거기서 크게
    #   벗어나는 후보는 버린 뒤 다음 후보를 쓴다. 색이 3개라 중앙값은 하나가
    #   엉터리여도 흔들리지 않는다.
    #   (2026-08-27 실측: 나무 바닥이 빨강으로 x31~309(폭 278, 세로중심 412)로
    #    잡혔는데 진짜 병 3개는 폭 68~77, 세로중심 287 이었다.)
    def _median(vals):
        s = sorted(vals)
        return s[len(s) // 2]

    boxes = []
    if cands:
        med_w = _median([b[0][2] - b[0][0] for b in cands.values()])
        med_y = _median([(b[0][1] + b[0][3]) / 2 for b in cands.values()])
        w_limit = med_w * BOX_OUTLIER_RATIO
        y_tol = h * BOX_Y_TOL_RATIO
        for color, blobs in cands.items():
            pick = next((b for b in blobs
                         if b[2] - b[0] <= w_limit
                         and abs((b[1] + b[3]) / 2 - med_y) <= y_tol), None)
            if pick is None:
                # 믿을 만한 후보가 없으면 이 색은 '못 봤다'로 친다.
                # 억지로 이상치를 쓰면 좌우 균형이 통째로 오염된다.
                continue
            boxes.append((float(pick[0]), float(pick[2]), color))
    if len(boxes) < MIN_BOTTLES:
        return None
    boxes.sort()                                # 화면 왼쪽부터
    widths = [x2 - x1 for x1, x2, _c in boxes]
    n = len(boxes)
    st = {
        'n': n,
        'left_gap': boxes[0][0],
        'right_gap': float(w) - boxes[-1][1],
        'w_left': widths[0],
        'w_right': widths[-1],
        'max_w': max(widths),
        'frame_w': float(w),
        'boxes': list(boxes),      # 뷰어가 '채택된 박스'를 표시할 때 쓴다
    }
    # ■ 가운데 병은 '화면 순서'가 아니라 **색**으로 고른다 (2026-08-27 사용자 확정)
    #   순서로 고르면 반사광이 가짜 박스로 잡히거나 병 하나를 놓쳤을 때
    #   엉뚱한 병이 '가운데'가 되어 조향이 통째로 틀어진다. 어느 색이 가운데인지는
    #   처음 깨끗한 프레임에서 한 번만 배우고(_latch_middle_color) 그 뒤로는
    #   그 색으로만 추적한다. 그 색이 안 보이면 조향은 건너뛴다.
    _latch_middle_color(boxes, widths)
    mid_color = middle_color()
    mid_idx = (None if mid_color is None else
               next((i for i, b in enumerate(boxes) if b[2] == mid_color), None))
    if n >= REQUIRED_BOTTLES:
        centers = sorted((x1 + x2) / 2 for x1, x2, _c in boxes)
        gl, gr = centers[1] - centers[0], centers[2] - centers[1]
        st['spacing_err'] = ((gl - gr) / max(gl, gr)) if max(gl, gr) > 0 else 0.0
    else:
        st['spacing_err'] = None
    if n >= REQUIRED_BOTTLES and mid_idx is not None:
        mid = boxes[mid_idx]
        st['w_mid'] = widths[mid_idx]
        st['mid_off'] = (mid[0] + mid[1]) / 2 - w / 2
        avg_side = (widths[0] + widths[-1]) / 2
        st['side_ratio'] = avg_side / st['w_mid'] if st['w_mid'] > 0 else None
        st['mid_color_seen'] = True
    else:
        # 3개가 안 보이면 '가운데'가 어느 병인지 알 수 없다. 무리 중심으로
        # 대신 채워두되, 이 값으로 조향하지 않는다(후진이 먼저다).
        st['w_mid'] = None
        st['mid_off'] = (boxes[0][0] + boxes[-1][1]) / 2 - w / 2
        st['side_ratio'] = None
        st['mid_color_seen'] = False
    return st


def middle_color() -> str | None:
    """지금 '가운데'로 쓰고 있는 색. 아직 못 정했으면 None."""
    return MIDDLE_COLOR or _MID['color']


def _latch_middle_color(boxes, widths) -> None:
    """3개가 깨끗하게 보이면 화면 가운데 박스의 색을 가운데 색으로 확정한다.

    확정 조건을 일부러 빡빡하게 둔다 — 한 번 잘못 배우면 그 뒤 모든 조향이
    엉뚱한 병을 따라가기 때문이다.
      * 정확히 3개만 보인다 (4개 이상이면 가짜가 섞인 것)
      * 세 박스가 서로 겹치지 않는다
      * 같은 답이 MID_LATCH_FRAMES 번 연속 나온다
    """
    if MIDDLE_COLOR is not None or _MID['color'] is not None:
        return
    if len(boxes) != REQUIRED_BOTTLES:
        return
    if not (boxes[0][1] < boxes[1][0] and boxes[1][1] < boxes[2][0]):
        return                                   # 박스끼리 겹침 -> 못 믿는다
    if min(widths) <= 0:
        return
    cand = boxes[1][2]
    if cand == _MID['vote']:
        _MID['count'] += 1
    else:
        _MID['vote'], _MID['count'] = cand, 1
    if _MID['count'] >= MID_LATCH_FRAMES:
        _MID['color'] = cand
        others = [b[2] for b in boxes if b[2] != cand]
        print(f"  [학습] 가운데 병 = {cand} (좌우: {', '.join(others)})")


def describe(st) -> str:
    """진단용 한 줄 요약 — 기준값을 실측으로 정할 때 이 출력을 읽는다."""
    if st is None:
        return '병 안 보임'
    txt = (f"병 {st['n']}개 | 가운데오차 {st['mid_off']:+.0f}px | "
           f"여백 좌{st['left_gap']:.0f}/우{st['right_gap']:.0f}px | "
           f"폭 좌{st['w_left']:.0f}")
    if st['w_mid'] is not None:
        txt += f"/중{st['w_mid']:.0f}"
    txt += f"/우{st['w_right']:.0f}px"
    txt += f" | 균형 {balance_error(st)*100:+.0f}%"
    if st['side_ratio'] is not None:
        txt += f" | 옆/가운데 {st['side_ratio']*100:.0f}%"
    if st.get('spacing_err') is not None:
        txt += f" | 간격차 {st['spacing_err']*100:+.0f}%"
    return txt


def gripper_y():
    """고정캠에서 보라 집게 아래끝의 y. 로봇이 판 쪽으로 갈수록 커진다.

    고정캠은 판을 위에서 내려다보고, 흰 책상 쪽에서 팔이 들어온다. 그래서
    '집게가 화면 아래로 내려간다' = '판 쪽으로 전진한다'가 된다 — 이걸로
    +x가 전진인지 후진인지를 헤딩을 몰라도 매번 실측할 수 있다."""
    img = cv2.imread(str(FRAME_DIR / 'fixed.jpg'))
    if img is None:
        return None
    blobs = find_color_blobs(img, 'purple', min_area=300, max_area=300000)
    return float(max(b[3] for b in blobs)) if blobs else None


def balance_error(st) -> float:
    """(왼쪽박스폭 - 오른쪽박스폭) / 큰 쪽. 음수면 왼쪽이 작다(=왼쪽이 더 멀다).

    큰 쪽을 기준으로 나누므로 '작은 쪽을 큰 쪽에 맞춘다'는 사용자 기준과 같다.
    """
    biggest = max(st['w_left'], st['w_right'])
    return 0.0 if biggest <= 0 else (st['w_left'] - st['w_right']) / biggest


def too_close(st) -> bool:
    """너무 가까운가.

    두 가지 신호를 쓴다.
      1) 3개가 안 보이는데 보이는 박스가 크다  -> 옆 병이 이미 화면 밖으로 나감
      2) 옆/가운데 폭 비율이 SIDE_RATIO_MIN 미만 -> 아직 3개 다 보이지만
         가운데만 급격히 커지는 중 = 나가기 직전
    2번이 있어야 '화면 밖으로 나간 뒤에야 알아채는' 뒷북을 피할 수 있다.
    """
    if st['n'] < REQUIRED_BOTTLES and st['max_w'] >= TOO_CLOSE_BOX_W:
        return True
    r = st['side_ratio']
    return r is not None and r < SIDE_RATIO_MIN


def too_far(st) -> bool:
    """좌우 박스가 아직 화면 끝에 안 붙었는가 = 더 가야 하는가.

    ■ 둘 **다** 붙어야 한다 (2026-08-27 사용자 기준)
      예전엔 '둘 다 여백이 클 때만' 전진하도록 돼 있어서, 한쪽만 붙으면
      거기서 완료 처리해버렸다(실측: 좌19/우36px 에서 '기본 위치 도달').
      한쪽만 붙은 건 거리 문제가 아니라 좌우로 치우친 것이므로, 전진은
      '둘 다 붙을 때까지' 계속하고 치우침은 균형/회전이 따로 잡는다.
    """
    return max(st['left_gap'], st['right_gap']) > EDGE_MARGIN_PX


def at_base_position(st) -> bool:
    """기본 위치인가 — 사용자 정의 4조건을 전부 만족해야 한다.

      (1) 좌·중·우 3개가 다 보인다
      (2) 가운데(초록) 박스가 화면 중앙에 있다        <- 회전
      (3) 좌우 박스 폭이 비슷하다                      <- 좌우 평행이동
      (4) 좌우 박스가 화면 끝에 딱 붙고, 너무 가깝지도 않다  <- 전후
    """
    return (st['n'] >= REQUIRED_BOTTLES
            and st['mid_color_seen']
            and abs(st['mid_off']) <= CENTER_TOL_PX
            and abs(balance_error(st)) <= BALANCE_TOL
            and not too_far(st)
            and not too_close(st))


def marker_pos():
    img = cv2.imread(str(FRAME_DIR / 'fixed.jpg'))
    return None if img is None else find_white_marker(img)


def _dur(err_abs, per_sec, fallback):
    """비례제어 시간. 정지마찰 때문에 MIN_PULSE_SEC 아래로는 안 내려간다."""
    d = fallback if not per_sec else err_abs / per_sec
    return max(MIN_PULSE_SEC, min(MAX_PULSE_SEC, d))


# ■ 바퀴 명령의 실제 방향은 '측정한 값'만 쓴다 (2026-08-27)
#   예전엔 규약을 가정하고, 오차가 커지면 그 자리에서 부호를 뒤집는
#   '자기교정'을 했다. 그게 하루를 날린 원인이다 — 검출이 한 프레임 튀거나
#   병이 화면 밖으로 나가면 비교가 무의미한데도 부호를 뒤집어버려서, 그 뒤로는
#   오차를 줄이려는 명령이 계속 오차를 키웠고 로봇이 판을 등지고 방을 가로질러
#   갔다. 부호는 사후 교정이 아니라 사전 측정이어야 한다.
#     측정:  ./venv/bin/python scripts/calibrate_base_signs.py
#     결과:  config/base_signs.json
#   (실측 결과는 base_nudge.py 주석의 규약과 정확히 일치했다 —
#    x+=전진, y+=좌, theta+=좌회전. 규약이 틀린 게 아니라 자기교정이 틀렸다.)
SIGNS_PATH = Path(__file__).resolve().parent.parent / 'config' / 'base_signs.json'
_DEFAULT_SIGNS = {'theta_to_reduce_mid_off': -1.0, 'y_to_grow_left': 1.0,
                  'x_forward': 1.0}


def _load_signs() -> dict:
    try:
        data = json.loads(SIGNS_PATH.read_text())
        return {k: float(data.get(k, v)) for k, v in _DEFAULT_SIGNS.items()}
    except (OSError, ValueError):
        print(f'  ⚠️ {SIGNS_PATH.name} 없음 — 규약 기본값 사용 '
              '(calibrate_base_signs.py 로 한 번 재두는 게 안전합니다)')
        return dict(_DEFAULT_SIGNS)


SIGNS = _load_signs()


def back_off_until_visible(client, args) -> bool:
    """병이 안 보이면 다시 찾는다 — 후진과 제자리 회전을 번갈아 시도한다.

    ■ 왜 회전이 필요한가 (2026-08-27 실측)
      예전엔 후진만 했다. 그런데 실제로 놓치는 상황은 두 가지다.
        (a) 판에 코를 박아서 병이 시야 밖으로 벌어짐 -> 후진이 답
        (b) 로봇이 판을 등지도록 돌아가 버림          -> 후진은 아무 소용 없음
      (b)에서 후진만 반복하면 판에서 점점 멀어지기만 한다(실측: 정렬 실패 후
      로봇이 판을 등진 채 뒤로만 가다 완전히 놓침). 그래서 후진 몇 번마다
      한 바퀴 훑는 회전을 섞는다.
    """
    if approach_state() is not None:
        return True
    print('  병이 하나도 안 보임 — 후진/회전으로 찾는 중')
    scan = 1.0
    for i in range(args.search_tries):
        if i % 3 == 2:
            pulse(client, vtheta=args.theta_vel * scan, duration=0.5)
            how = '회전'
        else:
            pulse(client, vx=-args.fwd_vel * _fwd_sign(), duration=0.5)
            how = '후진'
        st = approach_state()
        print(f'    {how} {i+1}회 -> {describe(st)}')
        if st is not None:
            return True
        if i == 5:
            # 절반을 써도 못 찾으면 반대쪽으로 훑는다(부호 뒤집기가 아니라
            # 탐색 방향 전환 — 여기서는 '어느 쪽에 있는지' 자체를 모른다)
            scan = -scan
            print('    ⚠️ 절반을 훑어도 못 찾음 — 반대 방향으로 훑습니다')
    print('  어느 쪽으로도 병을 못 찾음')
    return False


# +x 가 전진이라는 규약을 기본으로 쓰되, 실제로 반대로 동작하면 한 번 뒤집는다.
def _fwd_sign() -> float:
    return SIGNS['x_forward']




def _rotate(client, off, args, per_sec):
    """가운데(또는 무리) 오차 off 를 줄이는 쪽으로 회전. 부호는 자기교정한다."""
    dur = _dur(abs(off), per_sec, 0.35)
    pulse(client,
          vtheta=args.theta_vel * SIGNS['theta_to_reduce_mid_off']
          * (1.0 if off > 0 else -1.0),
          duration=dur)
    return dur


_SEEN = {'digest': None, 'since': 0.0}


def _frame_age() -> float:
    """베이스캠 **내용**이 마지막으로 바뀐 뒤 흐른 시간(초)."""
    try:
        data = (FRAME_DIR / 'front.jpg').read_bytes()
    except OSError:
        return 1e9
    d = hashlib.md5(data).hexdigest()
    now = time.time()
    if d != _SEEN['digest']:
        _SEEN['digest'], _SEEN['since'] = d, now
        return 0.0
    return now - _SEEN['since']


def wait_fresh_frame(limit: float = 10.0) -> bool:
    """움직이기 전에 카메라가 살아 있는지 먼저 확인한다."""
    _frame_age()                      # 기준 프레임 기록
    t0 = time.time()
    while time.time() - t0 < limit:
        time.sleep(0.4)
        if _frame_age() < 0.1:        # 내용이 바뀌었다
            return True
    return False


def _go(client, forward: bool, args, per_sec=None, err_abs=0.0) -> None:
    """판 쪽으로(forward=True) 또는 반대로 한 걸음."""
    d = _dur(err_abs, per_sec, 0.35)
    pulse(client, vx=args.fwd_vel * _fwd_sign() * (1.0 if forward else -1.0),
          duration=d)


def align_base(client, args) -> bool:
    """기본 위치로 맞춘다 — 하나의 루프에서 엄격한 우선순위로만 움직인다.

    ■ 우선순위 (2026-08-27 사용자 확정, 위에서부터 먼저)
      1) 병이 3개 미만이거나 너무 가까움  -> **후진만** 한다
         2개만 보일 때의 '맨 왼쪽/맨 오른쪽'은 3개일 때와 다른 병이라,
         그 값으로 조향·균형을 하면 서로 싸우며 3<->2를 왕복한다.
      2) 좌우 박스 크기 균형            -> 게걸음(y). **회전보다 먼저다.**
      3) 가운데(초록) 박스 중앙 정렬     -> 회전(theta)
      4) 좌우 끝에 붙을 때까지           -> 전진(x)

    ■ 왜 균형이 회전보다 먼저인가 (이게 예전 진동의 원인)
        회전하면  -> 가운데 박스가 좌우로 밀린다, 좌우 폭은 안 변한다
        평행이동  -> 좌우 폭이 달라진다, 가운데 박스도 같이 밀린다
      즉 좌우 폭은 회전에 안 흔들리지만 가운데 위치는 둘 다에 흔들린다.
      그래서 흔들리지 않는 쪽(균형)을 먼저 확정하고 회전으로 마무리해야
      서로를 무너뜨리지 않는다. 반대로 하면 균형을 잡을 때마다 조향이 틀어진다.

    ■ 부호는 전부 규약으로 정해진다 — 베이스캠이 로봇에 붙어 있기 때문
        y +  = 왼쪽으로  -> 왼쪽 병이 가까워져 커진다
        theta + = 좌회전 -> 화면이 오른쪽으로 밀린다(가운데 오차 +)
        x +  = 전진      -> 여백이 줄어든다
      고정캠과 달리 헤딩을 몰라도 되므로 테스트 펄스가 필요 없다.
    """
    mc = middle_color() or '자동판별'
    print(f'[접근] 기본 위치 맞추기 — 좌·중({mc})·우 3개, '
          f'가운데 중앙 ±{CENTER_TOL_PX}px, 균형 ±{BALANCE_TOL*100:.0f}%, '
          f'여백 ≤{EDGE_MARGIN_PX}px  (간격차는 기록만)')
    if not wait_fresh_frame():
        print('  ✘ 베이스캠이 새 프레임을 안 보냄 — 카메라/르키위 전원 또는 '
              'robot_bridge.py를 재기동하세요')
        return False
    mid_per_sec = None      # 회전 1초당 가운데오차가 몇 px 줄어드는지(실측)
    bal_per_sec = None      # 게걸음 1초당 폭균형이 몇 %p 줄어드는지(실측)
    backoffs = 0            # 연속 후진 횟수 — 너무 많으면 원인이 거리가 아니다
    stable = 0
    for step in range(args.max_steps):
        far = travel_exceeded()
        if far is not None:
            print(f'  ✘ {far} — 뭔가 잘못됐습니다. 안전을 위해 멈춥니다.')
            print('     로봇을 판 앞에 다시 놓고, 부호가 맞는지 확인하세요:')
            print('     ./venv/bin/python scripts/calibrate_base_signs.py')
            return False
        age = _frame_age()
        if age > FRAME_STALE_SEC:
            print(f'  ✘ 베이스캠 프레임이 {age:.0f}초째 안 갱신됨 — '
                  '르키위 전원/robot_bridge.py를 확인하세요')
            return False
        st = approach_state()
        print(f'  [{step}] {describe(st)}')

        if st is None:
            if not back_off_until_visible(client, args):
                return False
            stable = 0
            continue

        # ---- 1순위: 3개가 다 안 보인다 ----
        # 원인이 두 가지라 구분해야 한다(2026-08-27 실측으로 배움):
        #   (a) 너무 가까워서 좌우로 벌어져 밖으로 나감  -> 후진
        #   (b) 한쪽을 보고 있어서 반대쪽 병이 시야 밖   -> 그쪽으로 회전
        # 구분법: 보이는 무리가 화면 한쪽 끝에 몰려 있으면 (b)다.
        if st['n'] < REQUIRED_BOTTLES:
            off = st['mid_off']
            if abs(off) > ROTATE_FIRST_PX:
                side = '왼쪽' if off < 0 else '오른쪽'
                print(f'    {st["n"]}개만 보이고 무리가 {side}에 몰림({off:+.0f}px) '
                      f'— 못 본 병이 {side} 밖에 있음, {side}으로 회전')
                _rotate(client, off, args, None)
                stable = 0
                continue
            backoffs += 1
            if backoffs > MAX_CONSECUTIVE_BACKOFF:
                print(f'  ✘ {MAX_CONSECUTIVE_BACKOFF}번 물러나도 {st["n"]}개뿐 — '
                      '거리 문제가 아닙니다. 안 잡히는 색이 있는지 확인하세요:')
                print(f'     ./scripts/lekiwi.sh status  또는 '
                      f'approach_board.py --report')
                return False
            print(f'    {st["n"]}개만 보이고 무리는 중앙 — 너무 가까움, 후진 '
                  f'({backoffs}/{MAX_CONSECUTIVE_BACKOFF})')
            before_n = st['n']
            _go(client, forward=False, args=args)
            approach_state()
            stable = 0
            continue

        # ■ 여백이 완전히 0에 붙으면 균형↔회전이 서로를 부순다 (2026-08-27
        #   세 번째 발견) — 가까울수록 두 지표 다 민감해지는 건 이미 알았는데,
        #   그 정도가 너무 심해져서 이 거리에선 **서로 결합**해버린다: 균형을
        #   -16%까지 맞춰도 다음 회전 한 번에 -79%로 부서졌다(먼 거리에서
        #   실측했던 "회전은 균형에 거의 안 흔들린다"는 전제가 이 근접거리에선
        #   더 이상 안 맞는다). 두 지표를 동시에 안정시키려 하는 대신, 아예
        #   그 거리에 들어가지 않도록 살짝 물러나 여유를 만든다.
        if st['left_gap'] <= FLUSH_GAP_PX and st['right_gap'] <= FLUSH_GAP_PX:
            print(f"    여백이 완전히 붙음(좌{st['left_gap']:.0f}/우"
                  f"{st['right_gap']:.0f}px) — 균형·회전이 서로 부수는 거리라 "
                  "살짝 후진해 여유 확보")
            _go(client, forward=False, args=args, err_abs=0.0)
            stable = 0
            continue

        # 3개는 보이는데 옆/가운데 비율이 무너졌으면 나가기 직전이다 -> 후진
        if too_close(st):
            print(f'    옆/가운데 {st["side_ratio"]*100:.0f}% < '
                  f'{SIDE_RATIO_MIN*100:.0f}% — 너무 가까움, 후진')
            _go(client, forward=False, args=args)
            stable = 0
            continue

        backoffs = 0
        # 3개가 두 번 연속 보여야 측정값을 믿는다(한 프레임만 보고 움직이면
        # 검출이 깜빡일 때마다 엉뚱한 방향으로 밀린다)
        stable += 1
        if stable < 2:
            time.sleep(0.3)
            continue

        if at_base_position(st):
            print('  ✅ 기본 위치 도달')
            return True

        # ---- 2순위: 좌우 박스 크기 균형 (게걸음) — 회전보다 먼저 ----
        # ■ 회전보다 먼저 해야 하는 이유 (2026-08-27, 순서가 실수로
        #   뒤집혀 있던 걸 여기서 다시 바로잡음 — 아래 align_base
        #   docstring과 실제 코드 순서가 어긋나 있었다)
        #   실측: 게걸음 0.45초 -> 폭균형 -12.7%p, 회전 -> -0.6%p.
        #   즉 폭균형은 회전에 거의 안 흔들리는 신호인데, 가운데오차는
        #   회전(당연히)과 평행이동(균형 교정) 둘 다에 흔들린다. 그래서
        #   흔들리지 않는 쪽(균형)을 먼저 확정해야 회전이 그걸 다시 깨지
        #   않는다 — 반대 순서로 두면 균형<->회전이 서로를 계속 무너뜨려
        #   무한 진동한다(실측: 균형 -29%->회전->가운데+55px->회전->
        #   균형 -28%->... 로 20스텝 내내 수렴 안 함).
        err = balance_error(st)
        if abs(err) > BALANCE_TOL:
            toward = '왼쪽' if err < 0 else '오른쪽'
            print(f'    균형 {err*100:+.0f}% — {toward} 박스가 작음, {toward}으로 이동')
            # ■ '오차 크기'가 아니라 '거리'로 판단해야 한다 (2026-08-27,
            #   두 번째 발견) — 좌우 여백이 둘 다 0(판에 완전히 붙음)일 땐
            #   오차가 77~83%로 커도 물리적 민감도가 극단적이라 풀속도 한
            #   펄스가 40~83%p 씩 밀어버렸다. 처음엔 "오차가 작으면 느리게"로
            #   고쳤는데 그건 큰 오차엔 안 걸려서 부족했다 — 가까이 붙어있으면
            #   오차 크기와 무관하게 항상 느리게 가야 한다.
            very_close = st['left_gap'] <= 5 and st['right_gap'] <= 5
            fine = abs(err) < BALANCE_TOL * 2.5
            vy_scale = 0.15 if very_close else (0.35 if fine else 1.0)
            vy = (args.fwd_vel * vy_scale * SIGNS['y_to_grow_left']
                  * (-1.0 if err > 0 else 1.0))
            dur = min(_dur(abs(err), bal_per_sec, 0.30), MAX_STRAFE_SEC)
            if fine:
                dur = min(dur, MIN_PULSE_SEC * 1.5)
            pulse(client, vy=vy, duration=dur)
            cur = approach_state()
            if cur is not None and cur['n'] >= REQUIRED_BOTTLES and bal_per_sec is None:
                moved = abs(balance_error(cur) - err)   # 넘어가도 배운다
                rate = moved / dur
                # ⚠️ 한 번의 튄 측정을 영구히 믿으면 안 된다 (2026-08-27 실측)
                #   검출이 한 프레임 흔들려서 140%p/초 같은 말도 안 되는 값이
                #   나온 적이 있는데, 그걸 그대로 믿은 뒤로 모든 이동시간 계산이
                #   오염되어 -57%<->+31%<->-85% 로 폭주했다. 실측으로 관찰된
                #   정상 범위(16~83%p/초)를 벗어나면 버리고 다음 기회에 다시 잰다.
                if moved > 0.03 and rate <= BALANCE_RATE_SANITY_MAX:
                    bal_per_sec = rate
                    print(f'      [실측] 게걸음 1초당 균형 {bal_per_sec*100:.0f}%p 개선')
                elif moved > 0.03:
                    print(f'      [실측] {rate*100:.0f}%p/초는 비정상적으로 큼 — 이 '
                          '측정은 버리고 다음에 다시 잽니다')
            stable = 0
            continue

        # ---- 3순위: 가운데 병 중앙 정렬 (회전) ----
        # ■ 간격차로 회전을 잡아보려다 되돌린 기록 (2026-08-27)
        #   병 간격이 고르면 판을 정면으로 본 것이라는 기준은 기하학적으로 맞다.
        #   그런데 이 셋업에서 실측한 간격차는 +26% 였고, 회전으로 줄이는 속도가
        #   16%p/초라 다 고치려면 약 35도를 돌려야 했다. 실제로 돌려보니 왼쪽
        #   병이 106px -> 23px 로 화면 밖으로 밀려나며 정렬이 통째로 무너졌다.
        #   그래서 간격은 '기록만' 하고, 회전은 가운데 병 위치로 잡는다.
        #   (간격차가 이 셋업에서 원래 그 정도인지 확인하려면 성공한 순간의
        #    값을 보면 된다 — describe() 에 항상 찍히게 해뒀다.)
        off = st['mid_off']
        if not st['mid_color_seen']:
            print(f'    가운데 색({middle_color() or "미정"})이 안 보임 — 조향 건너뜀')
        elif abs(off) > CENTER_TOL_PX:
            print(f'    가운데 박스가 {off:+.0f}px 치우침 — 회전')
            # ■ 오차가 작으면 회전 속도를 낮춘다 (2026-08-27)
            #   최소 펄스(0.18초)로도 약 67px 움직이는데 허용오차는 40px 라,
            #   같은 속도로는 최소 펄스가 이미 목표를 지나친다. 실제로 -46 <->
            #   +84 로 무한 왕복했다.
            vel = args.theta_vel * (0.45 if abs(off) < CENTER_TOL_PX * 2.5 else 1.0)
            dur = _dur(abs(off), mid_per_sec, 0.22)
            pulse(client,
                  vtheta=vel * SIGNS['theta_to_reduce_mid_off']
                  * (1.0 if off > 0 else -1.0),
                  duration=dur)
            cur = approach_state()
            # ■ '오차가 줄었을 때만' 이동량을 기록하면 안 된다 — 매번 넘어가면
            #   영영 못 배우고 계속 넘어간다. 부호와 무관한 **절대 이동량**으로
            #   배우면 한 번에 배운다.
            if (cur is not None and cur['mid_color_seen'] and st['mid_color_seen']
                    and mid_per_sec is None):
                moved = abs(cur['mid_off'] - off)
                if moved > 10:
                    mid_per_sec = moved / dur * (vel / args.theta_vel)
                    print(f'      [실측] 회전 1초당 {mid_per_sec:.0f}px 이동')
            stable = 0
            continue

        # ---- 4순위: 좌우 끝에 붙을 때까지 전진 ----
        if too_far(st):
            # ■ 진짜 원인 (2026-08-27, 재수정) — _go()의 지속시간은
            #   _dur(err_abs, per_sec, fallback)인데 per_sec을 안 넘기면
            #   err_abs와 무관하게 항상 fallback(0.35초) 고정이다. 그래서
            #   1차 수정(err_abs만 넘김)은 실제로는 아무 효과가 없었다.
            #   전진 속도(px/초)를 따로 측정해두지 않았으니, 대신 이미 어느
            #   정도 가까워졌으면(여백 60px 미만) 아예 짧은 고정 펄스로
            #   바꿔서 매번 0까지 밀어붙이지 않게 한다. 이렇게 해야 여백이
            #   허용범위(20px) 근처에서 멈추고, 균형·회전이 서로 부수는
            #   근접거리(여백 0)에 강제로 들어가지 않는다.
            gap = min(st['left_gap'], st['right_gap'])
            print(f"    여백 좌{st['left_gap']:.0f}/우{st['right_gap']:.0f}px — 전진")
            if gap < 60:
                pulse(client, vx=args.fwd_vel * _fwd_sign() * 0.5,
                      duration=MIN_PULSE_SEC)
            else:
                _go(client, forward=True, args=args)
            stable = 0
            continue
        print('    더 줄일 오차가 없는데 완료 조건을 못 만족 — 한 번 더 확인')
        stable = 0

    print('  접근 미완료(스텝 소진)')
    return False


# 예전 이름 호환 — pick_cycle.py 등이 approach()/steer()를 부른다.
# steer 는 이제 align_base 안으로 흡수됐으므로 아무것도 안 한다.
def approach(client, args) -> bool:
    return align_base(client, args)


def steer(client, args) -> bool:
    return True


def raise_arm_and_find_marker(client, args) -> bool:
    """팔을 파지 준비 자세로 들고, 마커가 보일 때까지 조금씩 판 쪽으로 간다.

    마커가 안 보이는 건 각도 문제가 아니라 **로봇이 조금 뒤에 있어서**다
    (2026-08-26 사용자 확인).

    ■ 전진 방향은 여기서 추측하지 않는다 (2026-08-27)
      예전엔 펄스를 한 번 주고 집게가 화면에서 어느 쪽으로 움직이는지 보고
      방향을 정했다. 그런데 그 변화가 3px(사실상 노이즈)일 때도 그대로 믿어서
      방향을 반대로 잡았고, 팔이 판에서 멀어지며 화면 밖으로 나가버렸다
      (실측: 집게y 163 -> 41 -> 소실). 방향은 이미
      calibrate_base_signs.py 로 측정해 뒀으니 그 값을 쓴다.
      집게 y 는 이제 '얼마나 가까워졌는지' 진행 상황을 보여주는 용도다.
    """
    print('[4/4] 팔 들기 + 손목 흰 마커 확보')
    interpolate(client, SEARCH_POSE, 3.0)
    time.sleep(1.0)

    # ⚠️ '처음 보이는 순간'에 바로 만족하면 안 된다 (2026-08-27 실측)
    #   마커가 화면 위쪽 가장자리에 겨우 걸친 채(y~32) 발견되는 게 보통이다
    #   (SEARCH_POSE에서 전진하며 위쪽부터 들어오기 때문). 그 상태를 그대로
    #   넘기면 align_and_grasp.py의 방향보정 테스트(작게 움직여 보는 것)가
    #   마커를 화면 밖으로 밀어내 "방향보정 테스트 중 마커를 놓침"으로 죽는다.
    #   화면 안쪽으로 충분히 들어올 때까지(MARKER_SAFE_Y) 조금 더 전진한다.
    def _safely_visible(pos):
        return pos is not None and pos[1] >= MARKER_SAFE_Y

    m = marker_pos()
    if _safely_visible(m):
        print(f'  ✅ 마커 확보 ({m[0]:.0f},{m[1]:.0f})')
        return True
    if m is not None:
        print(f'  마커가 가장자리에 겨우 보임({m[0]:.0f},{m[1]:.0f}) — 더 안쪽으로 전진')

    last_y = None
    for i in range(args.marker_tries):
        gy = gripper_y()
        if gy is None:
            print(f'  [{i}] 집게가 아직 고정캠에 안 보임 — 계속 전진')
        else:
            trend = '' if last_y is None else f' ({gy - last_y:+.0f})'
            print(f'  [{i}] 집게 아래끝 y={gy:.0f}{trend} '
                  '(아래로 갈수록 판에 가까움)')
            # 판 쪽으로 가는데 오히려 멀어지면 부호가 틀린 것이다 -> 즉시 중단.
            # 예전처럼 계속 밀고 가면 팔이 화면 밖으로 나가 복구가 어려워진다.
            if last_y is not None and gy - last_y < -12:
                print('  ✘ 전진했는데 집게가 판에서 멀어짐 — 부호가 의심됩니다.')
                print('     ./venv/bin/python scripts/calibrate_base_signs.py '
                      '로 다시 측정하세요')
                return False
            last_y = gy

        pulse(client, vx=args.creep_vel * _fwd_sign(), duration=args.creep_sec,
              hold_arm=SEARCH_POSE)
        m = marker_pos()
        if _safely_visible(m):
            print(f'  ✅ 마커 확보 ({m[0]:.0f},{m[1]:.0f})')
            return True
    print('  ✘ 마커를 화면 안쪽까지 못 들여옴')
    return False


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument('--lekiwi-host', default='192.168.0.201')
    p.add_argument('--max-steps', type=int, default=20)
    p.add_argument('--theta-vel', type=float, default=22.0, help='회전 속도 deg/s')
    p.add_argument('--fwd-vel', type=float, default=0.07, help='전진 속도 m/s')
    p.add_argument('--marker-tries', type=int, default=10,
                   help='마커를 찾으며 전진해보는 횟수. 많이 주면 마커가 '
                        '안 보이는 다른 이유(가림 등)일 때 로봇이 계속 앞으로 간다')
    p.add_argument('--search-tries', type=int, default=12,
                   help='병을 놓쳤을 때 후진/회전으로 찾아보는 횟수')
    p.add_argument('--creep-vel', type=float, default=0.045,
                   help='마커를 찾을 때의 미세 전진 속도 m/s')
    p.add_argument('--creep-sec', type=float, default=0.25,
                   help='미세 전진 한 번의 시간(초)')
    p.add_argument('--skip-arm', action='store_true', help='접근까지만 하고 팔은 안 듦')
    p.add_argument('--middle-color', default=None,
                   help='가운데 병 색을 직접 지정(기본: 자동 판별)')
    p.add_argument('--report', action='store_true',
                   help='움직이지 않고 지금 상태만 출력(기준값 실측용)')
    args = p.parse_args()
    if args.middle_color:
        globals()['MIDDLE_COLOR'] = args.middle_color

    if args.report:
        # 로봇에 붙지 않고 프레임만 읽는다 — 기준값(SIDE_RATIO_MIN 등)을
        # 실제로 성공한 위치에서 읽어 정하기 위한 진단 모드.
        for _ in range(MID_LATCH_FRAMES + 1):
            st = approach_state()
        print(describe(st))
        if st is not None:
            print(f'  기본 위치인가: {"예" if at_base_position(st) else "아니오"}'
                  f'  (너무가까움={too_close(st)}, 더가야함={too_far(st)})')
        return

    client = RobotLink(args.lekiwi_host)
    client.connect()
    try:
        if not back_off_until_visible(client, args):
            raise SystemExit('베이스캠에서 병을 못 찾음 — 로봇이 판을 등지고 있거나 '
                             'robot_bridge.py가 안 떠 있습니다')
        print(f'시작: {describe(approach_state())}')
        if not align_base(client, args):
            sys.exit(1)
        if args.skip_arm:
            return
        if not raise_arm_and_find_marker(client, args):
            sys.exit(1)
        print('\n준비 완료 — 이제 집기를 실행하세요')
    finally:
        try:
            send_base(client, 0.0, 0.0, 0.0)
        except Exception:
            pass
        client.disconnect()


if __name__ == '__main__':
    main()


# ---------------------------------------------------------------------------
# 마커를 가운데 병에서 목표 병으로 옆걸음시키기
#
# ■ 왜 이렇게 하는가 (2026-08-27 사용자 제안)
#   고정캠의 흰 마커는 손목에 있고 실제 집게 끝은 그보다 앞이라, 마커를 병에
#   맞춰도 양옆 병은 빗나간다(시차). 그런데 **가운데 병에서는 이 오차가 0에
#   가깝다** — 실제로 초록은 마커 정렬만으로 두 번 다 성공했다.
#   그래서 순서를 바꾼다:
#     1) 마커를 **가운데 병**에 맞춘다 (여기서는 정확하다)
#     2) 그 자세 그대로 옆으로 이동해서 **마커를 목표 병 위로** 가져간다
#   옆으로만 움직이면 깊이 관계가 그대로라, 가운데에서 맞았던 조건이 유지된다.
#   시차 계수를 구할 필요가 없다.
# ⚠️ 허용오차를 고정 숫자로 못 박으면 안 된다 (2026-08-27 코드리뷰로 발견)
#   최소 펄스(0.18초)는 정지마찰 때문에 그 밑으로 줄일 수 없는데, 미세모드
#   속도(0.28배)로도 최소 이동량이 10.7px 다. 그런데 허용오차를 6px 로
#   박아뒀었다 — **최소 이동량이 이미 허용오차보다 커서 그 안으로 절대
#   못 들어가는** 목표를 세운 것이다(실제로 144->206->148 로 무한 왕복했다).
#   그래서 허용오차는 '실측한 최소 이동량'을 기준으로 그때그때 정한다 —
#   숫자 두 개를 손으로 맞춰서 우연히 앞뒤가 맞기를 바라지 않는다.
MARKER_FINE_ERR_PX = 45      # 이보다 가까우면 미세모드
MARKER_FINE_VEL_SCALE = 0.28
MARKER_MOVE_TOL_MIN_PX = 6   # 이보다 타이트하게는 안 잡는다(성공/실패 경계 13px 참고)


def _fixed_marker():
    img = cv2.imread(str(FRAME_DIR / 'fixed.jpg'))
    return None if img is None else find_white_marker(img)


def _fixed_bottle(color: str):
    """고정캠에서 해당 병의 목표점(align_and_grasp 와 같은 기준: x중앙, y아래끝)."""
    img = cv2.imread(str(FRAME_DIR / 'fixed.jpg'))
    if img is None:
        return None
    from color_detect import find_bottles_on_board
    b = find_bottles_on_board(img, color)
    if not b:
        return None
    x1, _y1, x2, y2, _a = b[0]
    return ((x1 + x2) / 2, float(y2))


def _achievable_tol(rate_px_per_sec: float | None, vel_scale: float) -> float:
    """이 속도로 실제 낼 수 있는 최소 이동량 — 허용오차는 이보다 작으면 안 된다."""
    if rate_px_per_sec is None:
        return MARKER_MOVE_TOL_MIN_PX
    return max(MARKER_MOVE_TOL_MIN_PX,
               rate_px_per_sec * vel_scale * MIN_PULSE_SEC * 1.15)  # 여유 15%


def move_marker_to_bottle(client, color: str, args,
                          target=None, center_x: float = 320.0) -> bool:
    """마커가 목표 병의 가로 위치에 오도록 게걸음으로 옮긴다.

    ■ 목표 좌표는 **팔을 들기 전에 확정**해서 넘겨야 한다 (2026-08-27 실측)
      팔이 병을 가리면 남은 조각의 색조가 날아가 다른 색으로 분류된다.
      실제로 목표x가 158 <-> 244 <-> 294 로 튀면서 옆걸음이 헛돌았다
      (팔이 홈에 있을 때는 10프레임 연속 158 로 완전히 안정적이었다).
      병은 움직이지 않으므로 한 번 잰 값을 끝까지 쓰는 게 맞다.

    ■ 서보 뒤 오픈루프 넛지 (2026-08-27 스파이크)
      바깥 슬롯 병은 파랄랙스 언더슈트 + 마커가 화면 끝에서 검출 안 됨 때문에
      시각서보만으로는 항상 목표에 못 미친다. 서보가 끝난 뒤(도달이든 놓침이든)
      config/sideways_nudge.json 의 슬롯별 펄스 수만큼 목표 쪽으로 더 민다.
      방향은 아래 루프가 실측한 `sign`을 재사용한다 — 실측 못 했으면(서보가
      너무 일찍 마커를 놓침) 블라인드 주행하지 않고 그대로 실패 반환.
      center_x: 슬롯 판정 기준점(가운데 병에 정렬됐을 때의 마커 x).
    """
    if target is None:
        target = _fixed_bottle(color)
    if target is None:
        print(f'  고정캠에서 {color} 병을 못 찾음')
        return False
    # ■ 목표는 병 그 자체가 아니라 **시차보정된 지점**이다.
    #   align_and_grasp 가 가운데 병에 맞출 때 쓰는 것과 같은 보정을 여기서도
    #   써야 조건이 일치한다(안 그러면 세로는 맞는데 가로만 어긋난다 — 실측).
    try:
        import parallax
        target = parallax.correct(target, parallax.load())
    except Exception as e:
        print(f'  (시차보정 로드 실패, 원좌표 사용: {e})')
    print(f'[마커이동] 마커를 {color} 병(x={target[0]:.0f}) 위로 옆걸음')
    px_per_sec = None
    sign = None            # +vy 가 마커 x 를 늘리는가 줄이는가(고정캠 기준, 실측)
    last_mk_x = None       # 마지막으로 본 마커 x (오픈루프 로그용)
    outcome = 'budget'     # 'reached' / 'lost' / 'budget' — 서보 종료 사유
    for step in range(args.max_steps):
        mk = _fixed_marker()
        if mk is None:
            if sign is None:
                # 방향도 못 잰 채 놓쳤으면 오픈루프도 방향을 모른다 — 그냥 실패.
                print(f'  [{step}] 마커를 놓침 — 중단 (방향 미실측)')
                return False
            print(f'  [{step}] 마커를 놓침 — 서보 종료, 오픈루프로 마무리')
            outcome = 'lost'
            break
        last_mk_x = mk[0]
        tgt = target            # 고정 좌표 — 팔이 가려도 안 흔들린다
        err = tgt[0] - mk[0]
        tol = _achievable_tol(px_per_sec, MARKER_FINE_VEL_SCALE)
        print(f'  [{step}] 마커x={mk[0]:.0f} 목표x={tgt[0]:.0f} '
              f'(오차 {err:+.0f}px, 허용 ±{tol:.0f}px)')
        if abs(err) <= tol:
            print(f'  ✅ 마커가 {color} 병 위에 도달 '
                  f'(세로차 {tgt[1]-mk[1]:+.0f}px)')
            outcome = 'reached'
            break
        if sign is None:
            # ⚠️ 고정캠은 로봇 밖에 있어서 헤딩을 모르면 방향을 알 수 없다.
            #    베이스캠과 달리 여기서는 반드시 실측해야 한다.
            before = mk[0]
            pulse(client, vy=args.fwd_vel, duration=0.3, hold_arm=SEARCH_POSE)
            after = _fixed_marker()
            if after is None:
                print('    방향 실측 중 마커를 놓침 — 중단')
                return False
            moved = after[0] - before
            if abs(moved) < 6:
                print(f'    게걸음이 마커를 거의 못 움직임({moved:+.0f}px) — 재시도')
                continue
            sign = 1.0 if moved > 0 else -1.0
            px_per_sec = abs(moved) / 0.3
            print(f'    [실측] y+ 0.3초 -> 마커 {moved:+.0f}px '
                  f'({px_per_sec:.0f}px/초, 부호 {sign:+g})')
            continue
        vel = args.fwd_vel
        rate = px_per_sec
        if abs(err) < MARKER_FINE_ERR_PX:      # 미세모드 — 속도를 낮춘다
            vel = args.fwd_vel * MARKER_FINE_VEL_SCALE
            rate = px_per_sec * MARKER_FINE_VEL_SCALE
        dur = max(MIN_PULSE_SEC, min(MAX_STRAFE_SEC, abs(err) / rate))
        pulse(client, vy=vel * sign * (1.0 if err > 0 else -1.0),
              duration=dur, hold_arm=SEARCH_POSE)
    else:
        outcome = 'budget'
        print('  마커이동 미완료(스텝 소진) — 오픈루프로 마무리 시도')

    # ---- 서보 뒤 오픈루프 넛지 ----
    servo_ok = (outcome == 'reached')
    if getattr(args, 'no_open_loop', False):
        return servo_ok
    try:
        cfg = sideways_nudge.load()
    except Exception as e:
        print(f'  (넛지 설정 로드 실패, 오픈루프 생략: {e})')
        return servo_ok
    n_pulses, vy_sign = sideways_nudge.open_loop_plan(
        sign, target[0], center_x, cfg)
    if n_pulses <= 0:
        return servo_ok
    shown_x = f'{last_mk_x:.0f}' if last_mk_x is not None else '?'
    print(f'  [오픈루프] 서보종료({outcome}) 마커x={shown_x} 목표x={target[0]:.0f} '
          f'기준x={center_x:.0f} → {n_pulses}펄스 vy{vy_sign:+g}')
    for i in range(n_pulses):
        pulse(client, vy=vy_sign * sideways_nudge.NUDGE_VEL,
              duration=sideways_nudge.NUDGE_PULSE_SEC, hold_arm=SEARCH_POSE)
        time.sleep(sideways_nudge.NUDGE_GAP_SEC)
    return True
