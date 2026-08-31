#!/usr/bin/env python
"""고정 설치된 외부(노트북) USB캠으로 손목의 흰 스티커(집게가 내려갈 지점의
대리표식)를 실시간 추적해서, 목표 색상 물체 위치에 맞춰지도록 베이스를
조금씩 움직인다(폐루프 P제어, eye-to-hand 방식).

⚠️ LeKiwi 베이스는 바퀴에 오도메트리/헤딩 센서가 전혀 없다(공식 문서: "바퀴
모터는 캘리브레이션 필요 없음" — 방향을 추적하는 게 아예 없다는 뜻).
그래서 사람이 계속 손으로 옮기다 보면 로봇이 지금 어느 방향을 보고 있는지
아무도 모르는 상태가 되고, "화면 오른쪽 오차 -> x.vel 음수" 같은 고정된
부호 매핑이 그때그때 안 맞게 된다(2026-08-25 실측: 같은 세션 안에서도
집게가 화면 오른쪽에서 들어올 때와 아래쪽에서 들어올 때가 다 있었음).
그래서 정렬 시작 전에 작은 테스트 이동으로 "지금 이 헤딩"에서의 베이스속도
->화면이동 매핑을 매번 새로 재서 쓴다(아래 calibrate_direction_matrix).

⚠️ 카메라는 로봇에 달려있지 않고, 약병들이 걸린 **수직 보드를 정면에서**
보는 고정 카메라다(2026-08-26 프레임 확인 — 예전 주석의 "위에서 수직으로
내려다본다"는 틀린 설명이었음). 로봇이 움직여도 물체는 화면에서 안 움직이고
팔/스티커만 움직이므로, "물체 위치 - 고정된 기준점"이 아니라 "물체 위치 -
스티커의 실시간 위치"로 오차를 계산해야 한다.

⚠️ 마커(그리퍼)는 보드보다 카메라 쪽으로 튀어나와 있어 병과 **깊이가 다르다**.
그래서 "마커 화면좌표 == 병 화면좌표"는 화면중심 부근에서만 맞고, 가장자리로
갈수록 어긋난다(가운데 병 10/10 성공 / 좌·우 병 실패의 원인). 목표좌표를
parallax.py로 보정해서 쓴다 — 자세한 기하는 그 파일 참고.

⚠️ 목표 물체 위치는 맨 처음 한 번만 측정해서 고정한다 — 매 프레임 다시
찾으면, 정렬이 다 될수록 스티커(팔)가 카메라 시점에서 물체를 가려버려서
색상검출이 흔들리는 문제가 있었다(2026-08-25 실측: 좌표가 프레임마다
완전히 다른 곳으로 튐). 물체는 어차피 안 움직이는 고정 카메라 환경이라
한 번만 재도 충분하다.

3D 좌표계산/IK 없음 — 화면 2D 오차만 보고 베이스를 움직인다. 팔은 이미
data/poses.json의 search_pose로 고정돼 있다고 가정.

안전을 위해 한 스텝당 이동량을 작게 제한한다(이전에 큰 폭 이동으로 벽에
부딪힌 사고가 있었음, 세션 기록 참고)."""
import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import parallax  # noqa: E402
from color_detect import (find_bottles_on_board,  # noqa: E402
                          find_white_marker)
from fixed_cam_server import fixed_frame_available, read_fixed_frame  # noqa: E402
from robot_link import RobotLink  # noqa: E402

# live_view_usb_local.py의 DEFAULT_ROI와 동일 — 목표 색상 물체(항상 검은
# 보드 위에 있음)를 찾을 때만 적용한다. 마커(흰 스티커)는 로봇이 멀리 있을
# 때 이 영역 밖(나무 바닥 위)에도 나타날 수 있어 ROI를 적용하지 않는다.
# 적용 안 했더니 나무 바닥이 빨간색으로 오탐되어(면적이 실제 병보다 커서)
# 그쪽을 목표로 잘못 고정한 사고가 있었음(2026-08-25 실측 발견).
TARGET_ROI = (110, 90, 410, 390)

ARM_KEYS = [
    'arm_shoulder_pan.pos', 'arm_shoulder_lift.pos', 'arm_elbow_flex.pos',
    'arm_wrist_flex.pos', 'arm_wrist_roll.pos', 'arm_gripper.pos',
]

# 화면 픽셀오차 -> 베이스 속도 변환 계수. 아직 실측 전 추정값이라 작게 시작.
GAIN_MPS_PER_PX = 0.0004
MAX_STEP_MPS = 0.05
# 이 미만의 속도 명령은 정지마찰에 막혀 실제 이동이 안 일어난다(실측 2026-08-26)
MIN_EFFECTIVE_MPS = 0.03
STEP_DURATION_SEC = 0.3
# ⚠️ 되돌림 (2026-08-27, 같은 날 재검토) — 좁히면 고쳐질 거라 생각했는데
#   틀렸다. 성공 오프셋 (+3,-19)=거리19.2px, 실패 오프셋 (+10,0)=거리10px —
#   **실패 쪽이 오히려 거리가 더 짧았다.** 즉 스칼라 거리로는 둘을 애초에
#   구분할 수 없어서, 좁혀도 원래 문제(성공/실패가 허용오차 안에서 갈림)는
#   안 고쳐지고 수렴 실패(정렬이 끝나지도 못함)만 새로 생겼다.
#   진짜 원인은 정렬 정밀도가 아니라 **시차보정량 자체가 부족한 것**으로
#   보임(2026-08-27 사용자 관찰: "실제로는 보이는 것보다 더 오른쪽으로 가야
#   함") — parallax 계수를 점 2개로 다시 맞추는 쪽으로 접근할 것.
PIXEL_TOLERANCE = 25
SETTLE_SEC = 0.4

# 한 스텝(최대속도 0.05m/s x 0.3s + settle)에서 실제로 로봇이 이동 가능한
# 거리는 방향보정 스케일(보통 수백~수천 px/m/s)로 환산해도 대략 수십px
# 안팎이다. 그런데 실측에서 마커가 한 스텝 만에 300px 넘게 "순간이동"한
# 것처럼 보인 적이 있었다(2026-08-25) — 실제 이동이 아니라 다른 흰색
# 물체(하이라이트 등)를 순간적으로 잘못 잡은 오탐이었다. 이런 튐을 그대로
# 믿고 최대속도로 계속 쫓아가서 엉뚱한 방향으로 여러 스텝 이동하는 사고가
# 실제로 있었음 — 이전 스텝의 마커 위치 대비 너무 크게 튀면(물리적으로
# 불가능한 이동) 그 프레임은 버리고 이번 스텝은 정지 상태로 넘어간다.
MAX_MARKER_JUMP_PX = 130
# 튐으로 거부된 위치가 다음 프레임에도 이만큼 안에서 다시 관측되면 오탐이
# 아니라 실제 이동으로 인정한다(위 거부 로직이 영원히 갇히는 것 방지).
MARKER_SETTLED_PX = 40


def send_base(client, obs, x=0.0, y=0.0):
    action = {k: obs[k] for k in ARM_KEYS}
    action['x.vel'] = x
    action['y.vel'] = y
    action['theta.vel'] = 0.0
    client.send_action(action)


class FrameSource:
    """고정캠 프레임을 어디서 읽을지 감춘다. fixed_cam_server.py가 떠 있으면
    그 공유 파일을 읽고(카메라 장치를 이 프로세스가 직접 점유하지 않음 —
    2026-08-26, 실시간 뷰어와 상시 병행하기 위한 변경), 없으면 지금까지처럼
    이 프로세스가 직접 cv2.VideoCapture를 연다(하위호환, 서버 없이 단독
    실행도 그대로 됨).

    카메라를 매번 새로 열면 오래된(캐시된) 프레임을 읽는 문제가 있어서
    (2026-08-25 실측 발견) 직접 여는 경우엔 핸들을 계속 열어두고 grab()으로
    버퍼를 비운 뒤 최신 프레임만 읽는다."""

    def __init__(self, usb_index: int):
        self._cap = None
        self._use_server = fixed_frame_available()
        if self._use_server:
            print('[align_and_grasp] fixed_cam_server.py 감지 — 공유 프레임 사용 '
                  '(카메라 직접 점유 안 함, 다른 뷰어와 동시 사용 가능)')
        else:
            self._cap = cv2.VideoCapture(usb_index)
            if not self._cap.isOpened():
                raise RuntimeError(f'/dev/video{usb_index} 를 열 수 없음')

    def read(self):
        if self._use_server:
            frame = read_fixed_frame()
            if frame is None:
                raise RuntimeError('fixed_cam_server 프레임이 오래됐거나 없음 '
                                    '(서버가 죽었을 수 있음)')
            return frame
        for _ in range(3):
            self._cap.grab()
        ok, frame = self._cap.retrieve()
        if not ok:
            raise RuntimeError('USB 카메라 프레임을 못 읽음')
        return frame

    def release(self) -> None:
        if self._cap is not None:
            self._cap.release()


# 정렬 중 흰 마커를 순간적으로 놓쳤을 때 다시 보는 횟수/간격
MARKER_RETRIES = 5
MARKER_RETRY_SEC = 0.35


def blob_center(frame, color):
    """정렬 목표 좌표: x는 바운딩박스 중앙, y는 중앙이 아니라 아래쪽 끝(y2) —
    실측 관찰 결과 집게가 실제로 내려가 잡아야 할 지점이 병 실루엣의
    세로 중앙이 아니라 아래쪽이라 y2를 기준으로 맞추는 게 더 정확하다고
    판단됨(2026-08-26, 사용자 육안 관찰 — 처음엔 y1로 반대로 넣었다가 수정)."""
    # ⚠️ find_color_blobs 를 그대로 쓰면 안 된다 — 고정캠은 판 바깥이 나무
    # 바닥이라 빨강 오탐이 잔뜩 나오고, 그중 **가장 큰 덩어리가 나무**여서
    # (면적 10708 vs 진짜 병 3551) 빨간 병을 집으라고 하면 바닥을 겨냥한다
    # (2026-08-27 실측). 판 위에 있는 것만 남긴다.
    blobs = find_bottles_on_board(frame, color)
    if not blobs:
        return None
    x1, y1, x2, y2, area = blobs[0]
    return ((x1 + x2) / 2, y2)


def pulse(client, obs, vx, vy, duration):
    elapsed = 0.0
    while elapsed < duration:
        send_base(client, obs, vx, vy)
        time.sleep(0.05)
        elapsed += 0.05
    send_base(client, obs, 0.0, 0.0)


def calibrate_direction_matrix(client, cap, test_vel=0.06, test_duration=0.4):
    """현재 헤딩에서 베이스속도(vx,vy) -> 화면이동(dpx,dpy) 매핑을 작은
    테스트 이동 2번으로 실측한다. 반환: 2x2 행렬 M (screen_delta ≈ M @ [vx,vy]).

    test_vel=0.04/duration=0.3(옛값)로는 y(옆) 방향 테스트가 정지마찰을 못
    이겨 실제 이동량이 0으로 측정되는 경우가 실측으로 반복 확인됨(2026-08-26)
    — x는 잘 되는데 y만 실패하는 패턴. 값을 키워서 해결.

    각 축은 테스트 후 반대 방향으로 즉시 되돌아와서 순이동은 거의 0이다."""
    def marker(retries=3):
        # SETTLE_SEC 직후 한 프레임만 보면 모션블러 등으로 순간적으로
        # 못 잡는 경우가 있었다(실측 확인, 2026-08-25) — 몇 번 재시도한다.
        for _ in range(retries):
            frame = cap.read()
            m = find_white_marker(frame)
            if m is not None:
                return m
            time.sleep(0.1)
        return None

    m0 = marker()
    if m0 is None:
        raise RuntimeError('방향보정 시작 전 마커를 못 찾음')

    # x.vel 테스트
    obs = client.get_observation()
    pulse(client, obs, test_vel, 0.0, test_duration)
    time.sleep(SETTLE_SEC)
    m1 = marker()
    obs = client.get_observation()
    pulse(client, obs, -test_vel, 0.0, test_duration)
    time.sleep(SETTLE_SEC)

    # y.vel 테스트
    m2 = marker()
    obs = client.get_observation()
    pulse(client, obs, 0.0, test_vel, test_duration)
    time.sleep(SETTLE_SEC)
    m3 = marker()
    obs = client.get_observation()
    pulse(client, obs, 0.0, -test_vel, test_duration)
    time.sleep(SETTLE_SEC)

    if m1 is None or m2 is None or m3 is None:
        raise RuntimeError('방향보정 테스트 중 마커를 놓침(화면 밖으로 나갔을 수 있음)')

    denom = test_vel * test_duration
    col_x = np.array([m1[0] - m0[0], m1[1] - m0[1]]) / denom
    col_y = np.array([m3[0] - m2[0], m3[1] - m2[1]]) / denom
    matrix = np.column_stack([col_x, col_y])

    # det()을 절대값(1e-6)으로 검사하면 행렬 값 자체가 크므로(px/m 단위,
    # 보통 수백~수천) 통과 기준이 무의미하게 낮아진다(독립 검증에서 발견,
    # 2026-08-25) — 크기와 무관한 조건수(cond)로 "두 축이 실질적으로 같은
    # 방향으로만 움직였는지"를 판단한다.
    cond = np.linalg.cond(matrix)
    if cond > 20:
        raise RuntimeError(f'방향보정 실패 — 행렬 조건이 나쁨(cond={cond:.1f}, '
                           f'x/y 테스트 이동이 화면에서 거의 같은 방향으로 보임): {matrix}')

    print(f'방향보정 완료 — x.vel 1m/s당 화면이동 {col_x.round(0)}px, '
          f'y.vel 1m/s당 {col_y.round(0)}px')
    return matrix


def main() -> None:
    p = argparse.ArgumentParser()
    # 'white'는 목표색으로 못 씀 — 손목 마커(find_white_marker)와 같은 HSV
    # 범위라 로봇 자기 자신을 목표로 착각할 위험이 있음(독립 검증에서 발견).
    p.add_argument('--color', required=True, choices=('red', 'green', 'blue'))
    p.add_argument('--lekiwi-host', default='192.168.0.201')
    p.add_argument('--usb-index', type=int, default=0)
    # ⚠️ PIXEL_TOLERANCE 를 25->12로 좁혔더니(2026-08-27) 같은 25스텝으로는
    # 못 끝내는 경우가 생겼다(실측: 280px 거리에서 시작해 25스텝 소진, "정렬
    # 미완료이고 목표도 아직 보임"). 정밀도는 유지하고 스텝만 늘린다.
    p.add_argument('--max-steps', type=int, default=40)
    p.add_argument('--dry-run', action='store_true', help='실제로 베이스를 움직이지 않고 계산만')
    p.add_argument('--no-calibrate', action='store_true',
                   help='방향 자동보정 끄고 고정 부호(--x-sign/--y-sign) 사용')
    p.add_argument('--x-sign', type=float, default=-1.0,
                   help='(--no-calibrate 전용) 화면 x오차->베이스 x속도 부호')
    p.add_argument('--y-sign', type=float, default=1.0,
                   help='(--no-calibrate 전용) 화면 y오차->베이스 y속도 부호')
    p.add_argument('--no-parallax', action='store_true',
                   help='시차보정을 끄고 병 좌표를 그대로 목표로 씀(예전 동작)')
    p.add_argument('--no-show', action='store_true',
                   help='진행 상황을 창으로 안 보여줌(기본은 보여줌)')
    p.add_argument('--diagonal', action='store_true',
                   help='x,y를 동시에(대각선으로) 움직임. 기본은 한 축씩 번갈아 교정')
    args = p.parse_args()

    window = 'align_and_grasp (자동 종료됨)' if not args.no_show else None

    cap = FrameSource(args.usb_index)

    client = RobotLink(args.lekiwi_host)
    client.connect()

    prev_dist = None
    stuck_count = 0
    prev_gripper = None
    jumped_to = None
    try:
        # 물체 위치는 맨 처음 한 번만 측정해서 고정한다 — 정렬이 다 되면 스티커가
        # 물체 바로 위(카메라 시점에서는 그 위에)를 덮어서 색상검출이 흔들리는
        # 문제를 실측으로 확인함(2026-08-25). 그 뒤로는 계속 잘 보이는 흰 스티커의
        # 실시간 위치만 이 고정 좌표에 맞춰가면 된다(eye-to-hand, 목표는 정지물체).
        first_frame = cap.read()
        rx, ry, rw, rh = TARGET_ROI
        roi_view = first_frame[ry:ry + rh, rx:rx + rw]
        target_in_roi = blob_center(roi_view, args.color)
        if target_in_roi is None:
            raise RuntimeError(f'{args.color} 물체를 초기 프레임(ROI 내)에서 못 찾음')
        bottle_px = (target_in_roi[0] + rx, target_in_roi[1] + ry)
        # 깊이차 보정: 마커가 실제로 가야 할 화면좌표는 병이 보이는 좌표가
        # 아니라 화면중심에서 조금 더 바깥쪽이다(parallax.py 설명 참고).
        # 미설정이면 항등이라 예전 동작 그대로.
        params = parallax.IDENTITY if args.no_parallax else parallax.load()
        target = parallax.correct(bottle_px, params)
        fh, fw = first_frame.shape[:2]
        if not (0 <= target[0] < fw and 0 <= target[1] < fh):
            raise RuntimeError(
                f'보정된 목표({target[0]:.0f},{target[1]:.0f})가 화면({fw}x{fh}) 밖입니다 '
                '— 시차 계수가 과합니다. calibrate_parallax.py 로 다시 맞추거나 '
                '--no-parallax 로 끄세요')
        print(f'목표 좌표 고정: 병=({bottle_px[0]:.0f},{bottle_px[1]:.0f}) -> '
              f'마커목표=({target[0]:.0f},{target[1]:.0f}) '
              f'[{parallax.describe(params)}]')

        # ⚠️ 이 보정도 실제로 베이스를 움직이는 단계라 try 안에 있어야 한다 —
        # 마커를 놓치는 실패는 이 함수 자체가 "일어날 수 있다"고 문서화한
        # 상황인데, try 밖에 있으면 그 실패 시 정지 명령/연결해제가 전혀 안
        # 되는 채로 죽는 문제가 있었음(독립 검증에서 발견, 2026-08-25).
        inv_matrix = None
        if not args.no_calibrate:
            inv_matrix = np.linalg.inv(calibrate_direction_matrix(client, cap))

        for step in range(args.max_steps):
            obs = client.get_observation()
            frame = cap.read()

            # 기준점: 손목 모터의 흰 원형 스티커(직각 자세에서 집게가 내려갈
            # 지점의 대리표식). 보라색 집게 형상을 추적하는 것보다 안정적.
            gripper = find_white_marker(frame)
            if gripper is None:
                # ⚠️ 한 프레임 놓쳤다고 바로 포기하면 안 된다 — 움직이는 중에는
                # 모션블러나 팔의 흰 케이블·밝은 병 때문에 순간적으로 놓치는데,
                # 멈추고 다시 보면 대개 바로 잡힌다(2026-08-27 실측: 정렬 중
                # '못 찾음'으로 중단됐는데 직후 같은 자리에서 (292,284)로 정상
                # 검출됨). 잠깐 세우고 몇 번 다시 본다.
                send_base(client, obs, 0.0, 0.0)
                for retry in range(MARKER_RETRIES):
                    time.sleep(MARKER_RETRY_SEC)
                    gripper = find_white_marker(cap.read())
                    if gripper is not None:
                        print(f'[{step}] 흰 스티커 재확보 (재시도 {retry + 1}회)')
                        break
                if gripper is None:
                    print(f'[{step}] 흰 스티커를 {MARKER_RETRIES}번 다시 봐도 '
                          '못 찾음 — 정지')
                    break

            if prev_gripper is not None:
                jump = ((gripper[0] - prev_gripper[0]) ** 2
                        + (gripper[1] - prev_gripper[1]) ** 2) ** 0.5
                if jump > MAX_MARKER_JUMP_PX:
                    # 튐이 연속으로 같은 자리에서 반복되면 오탐이 아니라 마커가
                    # 실제로 그리 옮겨간 것이다(로봇이 크게 움직였거나 앞의
                    # prev값이 오탐이었던 경우) — 그걸 계속 거부하면 남은 스텝을
                    # 전부 "무시"로 날려버린다(2026-08-26 실측: 427px 튐이 5스텝
                    # 연속 거부되며 정렬이 통째로 실패). 2회 연속 같은 위치면 인정.
                    if (jumped_to is not None
                            and abs(gripper[0] - jumped_to[0]) < MARKER_SETTLED_PX
                            and abs(gripper[1] - jumped_to[1]) < MARKER_SETTLED_PX):
                        print(f'[{step}] 튄 위치가 2회 연속 같음 — 실제 이동으로 인정')
                        prev_gripper = gripper
                        jumped_to = None
                    else:
                        print(f'[{step}] 마커가 한 스텝만에 {jump:.0f}px 튐(물리적으로 '
                              f'불가능) — 오탐으로 보고 이번 프레임은 무시')
                        jumped_to = gripper
                        send_base(client, obs, 0.0, 0.0)
                        continue
                else:
                    jumped_to = None
            prev_gripper = gripper

            err_x = target[0] - gripper[0]
            err_y = target[1] - gripper[1]
            dist = (err_x ** 2 + err_y ** 2) ** 0.5
            print(f'[{step}] 스티커=({gripper[0]:.0f},{gripper[1]:.0f}) '
                  f'물체=({target[0]:.0f},{target[1]:.0f}) '
                  f'오차=({err_x:+.0f},{err_y:+.0f}) 거리={dist:.0f}px')

            if window is not None:
                vis = frame.copy()
                gx, gy = int(gripper[0]), int(gripper[1])
                tx, ty = int(target[0]), int(target[1])
                cv2.circle(vis, (gx, gy), 12, (0, 255, 255), 2)   # 스티커: 노랑
                cv2.circle(vis, (tx, ty), 12, (0, 0, 255), 2)     # 목표: 빨강
                cv2.line(vis, (gx, gy), (tx, ty), (255, 255, 255), 1)
                cv2.putText(vis, f'step {step} dist {dist:.0f}px', (5, vis.shape[0] - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.imshow(window, vis)
                cv2.waitKey(1)

            if dist < PIXEL_TOLERANCE:
                print('정렬 완료')
                send_base(client, obs, 0.0, 0.0)
                break

            if prev_dist is not None and abs(dist - prev_dist) < 3:
                stuck_count += 1
                if stuck_count >= 3:
                    print('화면 변화가 감지되지 않음 — 안전을 위해 중단')
                    send_base(client, obs, 0.0, 0.0)
                    break
            else:
                stuck_count = 0
            prev_dist = dist

            if inv_matrix is not None:
                # inv_matrix @ err는 "오차를 한번에 없앨 실제 이동거리(m)".
                # 그대로 다 가면 오버슈트하니 P_GAIN만큼만, 시간으로 나눠 속도로 변환.
                P_GAIN = 0.35
                dist_m = inv_matrix @ np.array([err_x, err_y])
                v = dist_m * P_GAIN / STEP_DURATION_SEC
                vx, vy = float(v[0]), float(v[1])
            else:
                vx = args.x_sign * err_x * GAIN_MPS_PER_PX
                vy = args.y_sign * err_y * GAIN_MPS_PER_PX

            if not args.diagonal:
                # 대각선(x,y 동시)으로 안 가고 한 축씩 번갈아 교정 — 대각 이동은
                # 행렬의 두 열이 동시에 섞여서 어느 한쪽 계측오차가 있으면 매
                # 스텝마다 그 오차가 같이 실리는데, 한 축만 움직이면 그 축의
                # 계측값에만 의존하게 되어 덜 흔들린다는 지적을 반영(사용자 의견).
                if step % 2 == 0:
                    vy = 0.0
                else:
                    vx = 0.0

            speed = (vx ** 2 + vy ** 2) ** 0.5
            if speed > MAX_STEP_MPS:
                vx, vy = vx * MAX_STEP_MPS / speed, vy * MAX_STEP_MPS / speed
            elif 0 < speed < MIN_EFFECTIVE_MPS:
                # P제어라 오차가 줄수록 속도가 작아지는데, 이 베이스는 일정
                # 크기 미만이면 정지마찰을 못 이겨 명령만 나가고 실제로는 안
                # 움직인다(2026-08-26 실측: 0.008~0.012에서 화면 변화 없음 ->
                # 오차 40~56px을 남긴 채 최대스텝 소진). 허용오차보다 오차가
                # 아직 큰데도 속도가 이 미만이면 "실제로 움직이는 최소값"까지
                # 올려준다. 안 그러면 남은 오차를 영원히 못 좁힌다.
                vx, vy = vx * MIN_EFFECTIVE_MPS / speed, vy * MIN_EFFECTIVE_MPS / speed
            print(f'    -> 베이스 속도 x={vx:.3f} y={vy:.3f} ({STEP_DURATION_SEC}s)')

            if not args.dry_run:
                elapsed = 0.0
                while elapsed < STEP_DURATION_SEC:
                    send_base(client, obs, vx, vy)
                    time.sleep(0.05)
                    elapsed += 0.05
                send_base(client, obs, 0.0, 0.0)
                time.sleep(SETTLE_SEC)
        else:
            print('최대 스텝 도달 — 정렬 못 끝냄')
    finally:
        try:
            obs = client.get_observation()
            send_base(client, obs, 0.0, 0.0)
        except Exception:
            pass
        client.disconnect()
        cap.release()
        if window is not None:
            cv2.waitKey(500)
            cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f'실패: {type(e).__name__}: {e}', file=sys.stderr)
        sys.exit(1)
