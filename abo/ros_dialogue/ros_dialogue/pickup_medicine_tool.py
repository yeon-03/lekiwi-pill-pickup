"""약통 픽업(색상 지정) LLM 도구 선언.

lekiwi_tool.py와 동일 패턴 -- 여기 선언된 함수는 실행되지 않고(dialogue_node가
tool_call 이름으로 가로채 직접 처리), LLM이 자연어에서 색상을 판단하는 용도로만
쓰인다. 실제 실행(SSH 트리거)은 lekiwi_command_node가 /lekiwi_command 토픽을
구독해 처리한다(SKILL_MAP의 pick_red/pick_blue/pick_green 참고).
"""
from typing import Literal

from langchain_core.tools import tool

PICKUP_MEDICINE_TOOL_NAME = 'pickup_medicine'


@tool(PICKUP_MEDICINE_TOOL_NAME)
def pickup_medicine(color: Literal['red', 'blue', 'green']) -> str:
    """사용자가 특정 색깔의 약통을 가져다 달라고 요청하면 사용한다.

    color 값: red(빨강/빨간색)/blue(파랑/파란색)/green(초록/초록색). 이 세 가지
    색만 지원한다 -- 사용자가 다른 색(노란색 등)을 말하거나 색을 말하지 않으면
    이 도구를 쓰지 말고, 어떤 색인지(빨강/파랑/초록 중에서) 되물어라.
    """
    raise RuntimeError('pickup_medicine은 dialogue_node가 직접 처리해야 합니다')
