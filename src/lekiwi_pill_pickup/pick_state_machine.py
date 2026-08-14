"""약통 픽업 상태머신 — 순수 로직. 관측값(비전 감지결과/정렬여부/그립성공여부)을
입력받아 다음 상태와 이번 틱에 수행할 행동(Action)을 결정만 한다. 실제 하드웨어
I/O(카메라 프레임 읽기, 팔 관절 명령 전송)는 pick_pill_bottle.py(Task 12)가 담당하고
이 모듈은 아무것도 실행하지 않는다 — robot_ws의 session_manager.py와 동일 패턴.

상태 전이 시 그 상태에서 처음 해야 할 행동을 함께 반환한다(예: APPROACHING에서
GRASPING으로 전이하는 순간 DESCEND_AND_GRIP 행동을 반환) — 호출부는 그 행동을
실행한 뒤 결과를 다음 tick() 호출의 Observation에 담아 다시 전달한다."""
from dataclasses import dataclass
from enum import Enum, auto


class PickState(Enum):
    SEARCHING = auto()
    APPROACHING = auto()
    GRASPING = auto()
    LIFTING = auto()
    SUCCEEDED = auto()
    FAILED = auto()


class ActionType(Enum):
    SWEEP = auto()             # 탐색: 팔을 좌우로 소폭 스윕
    NUDGE = auto()              # 접근/정렬: 오차 방향으로 팔 미세 조정
    DESCEND_AND_GRIP = auto()   # 하강+그리퍼 닫기 고정 시퀀스 실행
    LIFT = auto()                # 들어올리기+retract
    NONE = auto()                # 이번 틱은 할 일 없음


@dataclass
class Action:
    type: ActionType


@dataclass
class Observation:
    detected: bool
    aligned: bool = False
    grasped: bool = False


def tick(state: PickState, obs: Observation, elapsed_in_state_sec: float,
         search_timeout_sec: float) -> tuple[PickState, Action]:
    if state == PickState.SEARCHING:
        if obs.detected:
            return PickState.APPROACHING, Action(ActionType.NONE)
        if elapsed_in_state_sec >= search_timeout_sec:
            return PickState.FAILED, Action(ActionType.NONE)
        return PickState.SEARCHING, Action(ActionType.SWEEP)

    if state == PickState.APPROACHING:
        if not obs.detected:
            return PickState.SEARCHING, Action(ActionType.NONE)
        if obs.aligned:
            return PickState.GRASPING, Action(ActionType.DESCEND_AND_GRIP)
        return PickState.APPROACHING, Action(ActionType.NUDGE)

    if state == PickState.GRASPING:
        # 이 상태로 진입한 턴에 이미 DESCEND_AND_GRIP을 호출부가 실행했다는 전제 —
        # 그 결과(그리퍼 부하값 기반 판정)를 obs.grasped에 담아 다시 tick()을 호출한다
        if obs.grasped:
            return PickState.LIFTING, Action(ActionType.LIFT)
        return PickState.FAILED, Action(ActionType.NONE)

    if state == PickState.LIFTING:
        return PickState.SUCCEEDED, Action(ActionType.NONE)

    # SUCCEEDED/FAILED는 종단 상태
    return state, Action(ActionType.NONE)
