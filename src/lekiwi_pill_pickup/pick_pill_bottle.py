#!/usr/bin/env python
"""약통 픽업 메인 스크립트 — LeKiwiClient로 연결해 손목 카메라+YOLO로 약통을 찾아
정렬 후 집는다. robot_ws의 lekiwi_command_node.py가 SSH로 이 스크립트를 실행하고,
표준출력 마지막 줄(SUCCESS/FAILED)로 결과를 판정한다.

⚠️ 아래 상수 중 GAIN_DEG_PER_PX/SWEEP_STEP_DEG/GRIPPER_MOTOR_NAME 등은 실기기
튜닝값이 채워지기 전까지 자리표시 값이다 — Task 15(실기기 튜닝)에서 실측 후 갱신할 것.
"""
import argparse
import sys
import time

from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig
from lerobot.motors.feetech import FeetechMotorsBus

from lekiwi_pill_pickup.detector import load_model_and_detect
from lekiwi_pill_pickup.distance_estimation import estimate_distance_cm
from lekiwi_pill_pickup.grip_sensing import grasped_something
from lekiwi_pill_pickup.pick_state_machine import (
    Action, ActionType, Observation, PickState, tick,
)
from lekiwi_pill_pickup.servo_control import compute_alignment_error, is_aligned

# --- Task 3/4/11 실측 결과로 채울 상수 (현재 자리표시 값) ---
FOCAL_LENGTH_PX = 800.0          # Task 4 캘리브레이션 결과로 교체
TARGET_HEIGHT_CM = 10.0          # 약통 실제 높이(cm) — 실측해서 교체
TARGET_DISTANCE_CM = 15.0        # 그립을 시도할 목표 거리 — 실기기 튜닝
EMPTY_CLOSE_LOAD = 100           # Task 3 실측 결과로 교체
GRIP_LOAD_MARGIN = 50            # Task 3 실측 결과로 교체
MODEL_PATH = 'yolov8n.pt'        # Task 11 결과에 따라 파인튜닝 모델 경로로 교체 가능
TARGET_LABEL = 'bottle'          # Task 12 진행 시 커스텀 클래스명으로 교체
MIN_CONFIDENCE = 0.5
CENTER_TOLERANCE_PX = 15.0
DISTANCE_TOLERANCE_CM = 1.5
SEARCH_TIMEOUT_SEC = 8.0
GRIPPER_MOTOR_NAME = 'gripper'   # Task 3에서 확인한 실제 명칭으로 교체
GRIPPER_MOTOR_ID = 6             # Task 3에서 확인한 실제 ID로 교체
LOOP_INTERVAL_SEC = 0.1


def _read_gripper_load(bus: FeetechMotorsBus) -> int:
    return bus.read('Present_Load', GRIPPER_MOTOR_NAME)


def _apply_action(client: LeKiwiClient, action: Action) -> None:
    """상태머신이 요청한 Action을 실제 로봇 명령으로 변환해 전송한다.
    실제 관절 델타 값(게인)은 실기기 튜닝 전까지 자리표시 — Task 15에서 확정."""
    if action.type == ActionType.NONE:
        return
    if action.type == ActionType.SWEEP:
        pass  # TODO(Task 15): 팔을 좌우로 소폭 스윕하는 관절 명령
    elif action.type == ActionType.NUDGE:
        pass  # TODO(Task 15): 마지막 정렬 오차 방향으로 관절 미세 조정
    elif action.type == ActionType.DESCEND_AND_GRIP:
        pass  # TODO(Task 15): 고정 시퀀스(내려가기 → 그리퍼 닫기)
    elif action.type == ActionType.LIFT:
        pass  # TODO(Task 15): 들어올리기 + retract


def run(lekiwi_host: str) -> bool:
    client = LeKiwiClient(LeKiwiClientConfig(remote_ip=lekiwi_host))
    client.connect()
    gripper_bus = FeetechMotorsBus(port='/dev/ttyACM0', motors={})  # Task 15에서 실제 구성

    state = PickState.SEARCHING
    state_entered_at = time.time()
    last_alignment_error = None

    try:
        while state not in (PickState.SUCCEEDED, PickState.FAILED):
            obs_frame = client.get_observation()
            frame = obs_frame['wrist']
            detection = load_model_and_detect(MODEL_PATH, frame, TARGET_LABEL, MIN_CONFIDENCE)

            aligned = False
            grasped = False
            if detection is not None:
                distance_cm = estimate_distance_cm(
                    detection.height_px, TARGET_HEIGHT_CM, FOCAL_LENGTH_PX)
                last_alignment_error = compute_alignment_error(
                    detection.center_x, detection.center_y,
                    frame.shape[1], frame.shape[0], distance_cm, TARGET_DISTANCE_CM)
                aligned = is_aligned(
                    last_alignment_error, CENTER_TOLERANCE_PX, DISTANCE_TOLERANCE_CM)

            elapsed = time.time() - state_entered_at
            observation = Observation(detected=detection is not None, aligned=aligned)
            new_state, action = tick(state, observation, elapsed, SEARCH_TIMEOUT_SEC)

            _apply_action(client, action)

            if action.type == ActionType.DESCEND_AND_GRIP:
                load = _read_gripper_load(gripper_bus)
                grasped = grasped_something(load, EMPTY_CLOSE_LOAD, GRIP_LOAD_MARGIN)
                observation = Observation(detected=True, grasped=grasped)
                new_state, action = tick(new_state, observation, 0.0, SEARCH_TIMEOUT_SEC)
                _apply_action(client, action)

            if new_state != state:
                state_entered_at = time.time()
            state = new_state
            time.sleep(LOOP_INTERVAL_SEC)
    finally:
        client.disconnect()

    return state == PickState.SUCCEEDED


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--lekiwi-host', default='192.168.0.201')
    args = parser.parse_args()

    success = run(args.lekiwi_host)
    if success:
        print('SUCCESS')
        sys.exit(0)
    else:
        print('FAILED: 정해진 시간 안에 약통을 찾거나 집지 못함')
        sys.exit(1)


if __name__ == '__main__':
    main()
