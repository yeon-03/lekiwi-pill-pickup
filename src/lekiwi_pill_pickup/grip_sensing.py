"""그리퍼 서보의 Present_Load 값으로 "확실히 뭔가 잡았는지" 판정하는 순수 함수.

실제 레지스터 판독(하드웨어 I/O)은 pick_pill_bottle.py(Task 12)가 lerobot의
FeetechMotorsBus로 수행하고, 이 함수는 판독된 숫자만 받아 판정한다.
empty_close_load/margin 값은 Task 3(실기기 측정)의 결과를 사용할 것 — 이 모듈
자체엔 기본값을 두지 않는다(잘못된 기본값으로 조용히 오판정하는 것을 방지)."""


def grasped_something(load_reading: int, empty_close_load: int, margin: int) -> bool:
    """load_reading이 '빈손으로 닫았을 때' 기준(empty_close_load)보다 margin 이상
    크면 뭔가에 걸려 완전히 안 닫힌 것으로 보고 성공으로 추정한다."""
    return load_reading > empty_close_load + margin
