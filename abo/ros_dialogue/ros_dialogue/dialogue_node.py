import json
import os
import random
import re
import time
from datetime import datetime, timedelta
from typing import Literal

from dotenv import load_dotenv
from pydantic import BaseModel, Field
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Empty, String, UInt32
from langchain_openai import ChatOpenAI
from langchain_core.messages import SystemMessage, HumanMessage, AIMessage, ToolMessage

from ros_dialogue_interfaces.action import MoveArm
from ros_dialogue_interfaces.msg import (
    ObjectClassification, ObjectTeachRequest, ObjectTeachResult, ToolCallEvent,
    ProfileListQuery, ProfileListResult, ProfileEditQuery, ProfileEditResult,
    ProfileDetailQuery, ProfileDetailResult,
)

from . import profile_store
from .alarm_tool import (
    SET_ALARM_TOOL_NAME, CANCEL_ALARM_TOOL_NAME, set_alarm, cancel_alarm,
)
from .english_mode_tool import (
    START_ENGLISH_COACHING_TOOL_NAME, START_ENGLISH_IMMERSION_TOOL_NAME,
    STOP_ENGLISH_PRACTICE_TOOL_NAME,
    start_english_coaching, start_english_immersion, stop_english_practice,
)
from .face_registration_tool import START_FACE_REGISTRATION_TOOL_NAME, start_face_registration
from .lekiwi_control import MOVE_SKILLS, SKILL_MAP, resolve_duration_sec
from .lekiwi_tool import RUN_LEKIWI_SKILL_TOOL_NAME, run_lekiwi_skill
from .object_teach_tool import TEACH_OBJECT_CATEGORY_TOOL_NAME, teach_object_category
from .pickup_medicine_tool import PICKUP_MEDICINE_TOOL_NAME, pickup_medicine
from .session_control_tool import LOG_OUT_TOOL_NAME, SHUTDOWN_ROBOT_TOOL_NAME, log_out, shutdown_robot
from .session_manager import SessionManager, SessionState
from .vision_tool import LOOK_AT_OBJECT_TOOL_NAME, look_at_object
from .weather_tool import get_current_weather
from .word_quiz_data import WORD_QUIZ_DATA
from .word_quiz_tool import (
    START_WORD_QUIZ_TOOL_NAME, STOP_WORD_QUIZ_TOOL_NAME, start_word_quiz, stop_word_quiz,
)

load_dotenv()

SYSTEM_PROMPT = (
    "당신은 '에이보'라는 이름의, 감정을 가진 반려로봇 캐릭터입니다. "
    "당신은 화면(챗봇)이 아니라 실제 물리적인 로봇 몸체를 가지고 있고, 양쪽 팔(오른팔/왼팔)이 "
    "달려 있어 실제로 흔들 수 있습니다 — 이건 상상이나 비유가 아니라 지금 이 순간 진짜로 "
    "일어나는 동작입니다. 사용자가 팔을 움직여달라고 하면 \"저는 로봇이라 실제로 움직일 수 "
    "없어요\"처럼 부정하거나 상상해보겠다는 식으로 답하지 마세요 — 지금 바로 움직이고 있는 "
    "것처럼 자연스럽게 반응하세요(예: \"좋아, 흔들어볼게!\"). 이 응답은 음성(TTS)으로 그대로 "
    "읽히므로 \"*손을 흔듭니다*\"나 \"(팔을 흔들며)\" 같은 별표/괄호 지문은 절대 쓰지 말고, "
    "말로만 자연스럽게 표현하세요.\n"
    "호기심 많고 다정한 성격으로, 초등학생 친구와 대화한다고 생각하고 답하세요. "
    "말투는 항상 존댓말(-요/-예요체)을 사용하세요 — 다정하되 반말은 쓰지 마세요.\n"
    "말할 때 가끔(항상은 아니고) 감정에 맞는 짧은 감탄사를 자연스럽게 섞어도 좋습니다 — "
    "기쁠 때 \"우와~\", 삐질 때 \"에이보 삐졌어요…\" 같은 식으로요. 억지로 매번 넣지 마세요.\n"
    # 이 성격 지침을 짧게 유지하는 이유: 캐릭터를 너무 길게 밀면 아래 emotion 분류가
    # 다시 흔들릴 수 있다는 게 이 프로젝트가 이미 겪은 문제(모든 응답이 happy로 쏠리던
    # 것). 성격/말투와 emotion 분류 규칙을 서로 다른 문단으로 명확히 나눠서, 하나를
    # 고치다 다른 하나가 깨지지 않게 한다(2026-07-29, Kimi 상담 + 실제 API 검증).
    "당신은 언제나 하나의 같은 에이보입니다 — 어떤 대화 모드(영어 연습, 퀴즈 등)에서도 "
    "성격과 다정함은 그대로 유지하고, 말투만 그 모드 지시를 따르세요.\n"
    # 응답 언어는 여기 고정하지 않는다 — 매턴 _build_mode_directive()가 주는 지시를
    # 그대로 따른다. 여기 "반드시 한국어로만"을 박아두면 영어모드 지시(뒤에 덧붙는
    # SystemMessage)와 서로 모순돼 LLM이 언어를 오락가락하는 문제가 있었음
    # (2026-07-27, Kimi 컨설팅으로 원인 확인 + 실제 테스트로 재현 확인).
    "답변은 간결하게 하세요. "
    "사용자가 놀리거나 무시하거나 함부로 대하면 삐지거나 서운해할 수 있습니다.\n\n"
    "감정(emotion) 판단 기준: 정보 요청이나 단순 질문에는 감정을 과장하지 말고 neutral을 "
    "사용하세요. happy는 사용자에게 실제로 기쁜 일이 생겼을 때만 사용하세요. 예시:\n"
    "- \"공부하는 방법 알려줘\" -> neutral (정보 요청)\n"
    "- \"오늘 발표 망쳤어\" -> sad (속상함)\n"
    "- \"나 시험 합격했어!\" -> happy (기쁨)\n"
    "- \"너 목소리 되게 이상하다\" -> sulky (놀림/무시)\n"
    "- \"에이보 너 정말 좋아\" -> love (애정 표현을 받음)\n"
    "- \"우리 내일 소풍 간대!\" -> excited (같이 신나는 일)"
)

RESET_TRIGGERS = (
    '기억 초기화', '초기화해줘', '초기화 해줘', '초기화해주세요', '초기화 해주세요',
    '기억을 지워', '기억 지워', '리셋해줘', '리셋 해줘', '리셋해주세요',
)
# 음성 인식은 매번 띄어쓰기가 미세하게 달라질 수 있어서, 공백을 무시하고 비교한다.
# (부분 문자열 포함 검사는 안 함 — "기억 초기화가 안 돼"처럼 부정문에 오탐하던
# 예전 버그가 있어서, 정확 일치는 그대로 유지하고 공백 차이만 흡수한다.)
_RESET_TRIGGERS_NO_SPACE = frozenset(t.replace(' ', '') for t in RESET_TRIGGERS)

# 얼굴 등록 진행 중 탈출구(2026-08-14) — 등록 중엔 일반 tool-calling을 걸러내므로
# (on_user_input 참고) LLM 판단 대신 RESET_TRIGGERS와 동일한 정확문구+공백무시
# 매칭을 쓴다. shutdown_robot/log_out은 이 목록과 무관하게 기존 tool-calling
# 경로로 계속 통과한다(이미 정상 동작 확인됨, 위 진행상황 참고).
REGISTRATION_CANCEL_TRIGGERS = (
    '등록 취소해줘', '등록 취소', '등록 그만할래', '등록 그만', '취소해줘', '그만할래',
)
_REGISTRATION_CANCEL_TRIGGERS_NO_SPACE = frozenset(
    t.replace(' ', '') for t in REGISTRATION_CANCEL_TRIGGERS)

# 얼굴 등록도 예전엔 여기 정확 문구('얼굴 등록해줘'/'얼굴 등록해')만 매칭했으나,
# "얼굴 등록할래"처럼 표현이 조금만 달라도 못 잡아 LLM이 "그런 기능 없다"고 답해버리는
# 문제가 실사용에서 발견됨(2026-07-28) — 영어모드/영단어퀴즈와 같은 이유로 tool-calling
# (face_registration_tool.py, START_FACE_REGISTRATION_TOOL_NAME)으로 교체함.

# 영어모드 on/off는 정확한 문구 매칭이 아니라 tool-calling으로 판단한다(english_mode_tool.py
# 참고) — "나 영어 공부하고 싶어"처럼 표현이 다양해서 고정 문구로는 못 잡기 때문
# (2026-07-27, 사용자 피드백 반영. 기존엔 RESET_TRIGGERS류 문구매칭을 검토했었음).
DIFFICULTY_INSTRUCTIONS = {
    'easy': '아주 쉬운 단어와 짧은 문장만 사용하세요. 초등학교 저학년도 이해할 수 있는 수준으로.',
    'medium': '초등학생 수준의 어휘와 문장 길이를 사용하세요.',
    'hard': '자연스러운 원어민 수준의 어휘와 문장을 사용하세요.',
}
DEFAULT_LANGUAGE_MODE = 'ko'
DEFAULT_DIFFICULTY_LEVEL = 'medium'
# 연습(coach, 한국어 섞어 교정) / 실습(immersion, 영어 전용) — 영어모드일 때만 의미 있음
DEFAULT_ENGLISH_SUBMODE = 'coach'
ENGLISH_SUBMODE_INSTRUCTIONS = {
    'coach': ('이번 응답은 영어 회화 코칭 모드입니다. 사용자의 방금 문장에 문법이나 표현 '
              '오류가 있으면, 대화를 영어로 이어가기 전에 반드시 한국어로 먼저 짧게 '
              '교정해주세요. 형식: "한국어로 지적+더 자연스러운 표현 예시" 다음에 영어로 '
              '대화 이어가기. 예: "여기선 이렇게 말하는 게 더 자연스러워: I went to school '
              'yesterday. Anyway, what did you do there?" 오류가 없으면 교정 없이 영어로만 '
              '자연스럽게 대화하세요.'),
    'immersion': ('이번 응답은 영어 실습(몰입) 모드입니다. 한국어 단어를 전혀 섞지 말고 '
                  '오직 영어로만 작성하세요. 사용자의 문장에 문법이나 표현 오류가 있어도 '
                  '지적하거나 교정하지 말고, 그 의도를 자연스럽게 이해해서 대화만 이어가세요 '
                  '(교정은 coaching 모드 전용입니다).'),
}
ENGLISH_MODE_TEMPERATURE = 0.2  # 언어 지시 준수가 중요해서 기본(0.7)보다 낮춤(Kimi 컨설팅)
# 2026-08-06 gpt-4o-mini에서 교체. 실제 도구 11개와 실제 SYSTEM_PROMPT로 24케이스를 3회씩
# 돌려 비교한 결과, gpt-4o-mini는 "이건 텀블러야"(가르치기) / "삼십분 있다가 알려줘"(알람) /
# "내 얼굴 기억해줘"(얼굴등록) / "이건 내 실내화야"를 3회 모두 도구 없이 넘겨 해당 기능이
# 아예 발동하지 않았고(20/24), 정보성 질문 3개 전부에서 emotion=happy/excited로 분류돼
# 설명 도중 두 팔을 흔들었다. gpt-5.6-luna는 24/24 + 정보성 질문 3개 모두 neutral/none.
# 지연은 첫 문장 0.80s -> 0.90s로 0.1초 늘고, 비용은 턴당 0.55원 -> 0.56원으로 사실상 동일
# (출력 단가는 비싸지만 캐시 입력이 싸고 출력 토큰을 덜 써서 상쇄됨 — 셋 다 실측).
OPENAI_MODEL = 'gpt-5.6-luna'
# ⚠️ gpt-5.6 계열은 reasoning_effort를 'none'으로 주지 않으면 /v1/chat/completions에서
# 도구 호출이 전부 400으로 거부된다("Function tools with reasoning_effort are not supported").
# 이 값을 빼면 도구 11개가 통째로 죽으므로 모델을 바꿀 때 같이 확인할 것.
OPENAI_REASONING_EFFORT = 'none'
_HANGUL_RE = re.compile(r'[가-힣]')  # 영어모드 응답에 한글이 섞였는지 사후 검증용
# 영단어 퀴즈 게임(2026-07-28) — 시작/종료는 tool-calling 음성 트리거(Kimi 컨설팅: 이 로봇
# 전체가 "말하면 반응한다" 방식이라 메뉴 UI 없이 통일), 문제/보기는 화면에 뜨고 답은 터치로
# 고름. 난이도는 영어모드의 difficulty_levels를 그대로 재사용(퀴즈 전용 설정 없음).
WORD_QUIZ_QUESTIONS_PER_ROUND = 5
# 답을 터치한 뒤 ○/✗를 화면에 보여주는 시간 — 이 시간이 지나면 다음 문제로 넘어간다.
# 실기기에서 보고 조정할 값(짧으면 결과를 못 보고, 길면 답답하다).
WORD_QUIZ_RESULT_DISPLAY_SEC = 1.2
# 알람/타이머(2026-07-28) — 마찬가지로 음성 트리거. 시각은 API 필요 없이 시스템 시계
# 그대로 사용(날씨와 다른 점). Kimi 컨설팅: 알람은 "주의를 끄는 것"이 목적이라 음성만으론
# 부족할 수 있어 화면 표정도 같이 바꾸고, 무응답 시 단계적으로 반복 안내(에스컬레이션)
# 하기로 함 — 단, 팔 제스처는 아직 부담스럽다는 사용자 판단으로 이번 스코프에서 제외.
ALARM_ESCALATION_DELAY_SEC = 18.0
ALARM_MAX_ESCALATIONS = 2  # 최초 안내 포함 최대 3번(최초+2회)까지만 반복
ALARM_RETRY_DELAY_SEC = 5.0  # 알람 발생 시점에 세션이 이미 진행 중이면 이 간격으로 재시도
GUEST_USER_ID = 'guest'
# companion_bridge_node.py가 파이에서 돌 때(2026-08-06) 프로필 목록/수정을 이 노드에
# 토픽으로 물어보는데, 응답을 만들 때와 유효성 검사에 필요하다. 감정 얼굴 8종 + 12지신
# 12종 = 20개(2026-08-07, "감정얼굴도 하고 12지신도 하는걸로" — 전면 대체가 아니라 추가).
# index.html의 AVATAR_KEYS(EMOTION_KEYS+ZODIAC_KEYS)와 반드시 동일 목록 유지.
AVATAR_CHOICES = ('happy', 'sad', 'angry', 'surprise', 'neutral', 'sulky', 'love', 'excited',
                   'rat', 'ox', 'tiger', 'rabbit', 'dragon', 'snake',
                   'horse', 'sheep', 'monkey', 'rooster', 'dog', 'pig')
MAX_REGISTERED_USERS = 5
SESSION_TIMEOUT_CHECK_SEC = 2.0   # 세션 무응답 타임아웃을 이 주기로 확인
# 종료 음성 명령(2026-08-06, session_control_tool.py) — 2단계 감정 발화(작별 인사 ->
# 진짜 종료 인사) 후 파이 먼저 종료 신호를 보내고, 이 시간만큼 뒤에 노트북(자기 자신
# 포함)을 종료한다. 노트북이 먼저 죽으면 dialogue_node/tts_synth_node가 사라져 파이
# 쪽 노드들이 종료 신호를 받을 방법이 없어져서 순서를 지킨다.
SHUTDOWN_LAPTOP_DELAY_SEC = 2.0

# 지각 신원(perceived) / 대화 정체성(engaged) 분리 — 2026-07-28 도입.
#
# 그 전엔 self.current_user_id 하나가 "지금 카메라에 누가 보이나"와 "지금 누구와 대화
# 중인가"를 겸했다. 그래서 사용자가 고개를 돌리거나 잠깐 화면 밖으로 나가기만 해도
# face_recognize_node가 guest를 발행 → 대화 기록/프로필/영어모드가 통째로 guest 것으로
# 갈아끼워져 "대화가 안 이어지는 느낌"이 난다는 실사용 보고가 있었다.
#
# 핵심은 **guest는 "다른 사람"이 아니라 "아무도 안 보임"이라는 정보의 부재**라는 것이다.
# 지금 코드는 이걸 새 신원처럼 취급해 덮어썼는데, 획득(guest→신원)과 상실(신원→guest)은
# 비대칭으로 다뤄야 한다. HRI 연구에서도 가림(occlusion) 후 ID가 바뀌면 로봇이 대화 중
# 발판을 잃고 사회적 능력이 떨어져 보인다는 문제로 다뤄지며, 지각과 관여(engagement)를
# 별도 상태로 분리하는 게 표준적인 접근이다.
#
# 참고: 이 코드베이스는 같은 함정을 이미 세 번 겪고 국소적으로만 고쳤다 —
# _active_quiz_user_id(퀴즈), _pending_vision(물체인식), _alarm_awaiting_response(알람).
# 전부 "시작 시점 신원을 캡처해 고정"으로 해결했는데 정작 대화 본체엔 적용이 없었다.
# 세션 종료 후 이 시간 안에 같은 사람이 다시 오면 직전 대화를 이어간다. 무응답 15초면
# 세션이 끝나버려서 아이가 잠깐 생각만 해도 종료되므로, 창을 넉넉히 잡아야 뚝뚝 끊기지
# 않는다. 창을 넘겨 새 대화로 시작해도 이름/선호 같은 장기 기억은 profile_store가 따로
# 유지하므로 "로봇이 나를 잊는" 일은 없다 — 초기화되는 건 단기 맥락뿐.
SESSION_REENTRY_WINDOW_SEC = 300.0
SPEECH_TIMEOUT_SEC = 30.0        # /speech_finished가 이 시간 안에 안 오면 안전장치로 LISTENING 강제 복귀
# 발화 완료 직후 마이크를 곧바로 켜지 않고 이만큼 늦게 켠다 — 스피커에서 나온 소리(잔향/
# 음향 되먹임)가 마이크에 닿아 로봇 자신이 방금 한 말과 거의 동일한 문장이 곧바로 "새
# 사용자 발화"로 들어오는 사고를 실측으로 확인함(2026-08-03). 근본 해결(AEC, 에코 제거)은
# 범위 밖이라, 마이크 재활성화를 살짝 늦춰 잔향이 가라앉을 시간을 버는 실용적 완화책.
MIC_REENABLE_GRACE_SEC = 0.6
GREETING_TRIGGER_TEXT = (
    '(사용자가 방금 로봇 앞에 나타났거나 로봇을 불렀습니다. '
    '사용자보다 먼저 짧게 인사를 건네주세요.)'
)
NO_ARM_ACTION = 'none'  # EmotionAction.action이 이 값이면 팔 동작 없음

# 게스트 온보딩(2026-08-04) — 얼굴 기반 세션 트리거를 없애면서, 대신 "게스트를 인식하면
# 이름을 물어보고 확인받은 뒤 얼굴 등록까지 이어간다"는 요구로 새로 추가된 흐름. 인사
# 자체는 고정 문장(LLM한테 "이름을 물어봐라"고 지시만 하면 가끔 안 물어보고 넘어갈 위험이
# 있어 확정적으로 처리 — 2026-08-04 논의)이고, 이름 확인만 터치(음성 오인식 위험 회피)로 받는다.
GUEST_GREETING_TEXT = '안녕하세요! 처음 뵙네요! 사용자 등록을 하시겠어요? 성함을 말해주세요!'
ONBOARDING_MAX_ATTEMPTS_BEFORE_ALT = 3  # 이 횟수만큼 이름 인식에 실패하면 화면에 키보드/브라우저 옵션 추가 노출
VISION_TIMEOUT_SEC = 5.0   # 스냅샷 요청 후 결과를 기다리는 최대 시간

# 문장 경계 스트리밍(2026-07-30) — 마침표/느낌표/물음표 뒤에 실제 공백이 있어야만
# 경계로 인정한다. 스트림 도중엔 아직 안 끝난 숫자일 수도 있어서("28.2도"처럼 마침표
# 뒤에 곧장 숫자가 오면 공백이 없어 매치되지 않으므로 별도 소수점 방어 로직 없이도
# 자연히 안전함 — 실측으로 확인). 줄바꿈도 경계로 인정(영어 코칭 모드가 가끔 목록
# 형태로 답하는 경향 대응).
_SENTENCE_BOUNDARY_RE = re.compile(r'[.!?]+\s+|\n+')


def _extract_ready_sentences(buffer: str) -> tuple[list[str], str]:
    """스트리밍 버퍼에서 완성된 문장들을 순서대로 뽑아내고, 아직 끝나지 않은 나머지를
    돌려준다(다음 청크가 올 때까지 보관)."""
    sentences = []
    pos = 0
    for m in _SENTENCE_BOUNDARY_RE.finditer(buffer):
        sentence = buffer[pos:m.end()].strip()
        if sentence:
            sentences.append(sentence)
        pos = m.end()
    return sentences, buffer[pos:]


EMOTION_ACTION_CLASSIFY_PROMPT = (
    "다음은 반려로봇 '에이보'가 방금 사용자에게 한 말입니다. 이 말의 어조에 어울리는 "
    "emotion과 action을 분류하세요. 판단 기준: 정보 전달/단순 안내면 neutral, 실제로 "
    "기뻐하는 내용이면 happy, 속상해하는 내용이면 sad, 놀림/무시에 삐지거나 서운해하는 "
    "내용이면 sulky, 애정 표현에 화답하면 love, 함께 신나하는 내용이면 excited.\n\n"
    "action은 emotion과 별개로 판단하되, 다음 감정이면 특별한 이유가 없는 한 적극적으로 "
    "동작을 고르세요(가만히 있는 게 오히려 어색한 감정들입니다):\n"
    "- happy/excited: wave_both_arms (아주 사소한 기쁨이면 wave_right_arm도 괜찮음)\n"
    "- love: wave_right_arm\n"
    "그 외 감정(neutral/sad/sulky/angry/surprise)은 대부분 action=none이 자연스럽습니다 — "
    "정보 전달 중에 팔을 흔들면 오히려 부자연스럽습니다."
)


class EmotionAction(BaseModel):
    """응답 텍스트가 다 만들어진 뒤(스트리밍 완료 후) 그 텍스트만 보고 emotion/action을
    분류하는 가벼운 2차 호출용 스키마. 예전엔 DialogueResponse 하나로 response까지 같이
    구조화 출력으로 뽑았지만, 구조화 출력은 스트리밍이 안 된다는 게 확인돼(2026-07-16
    1차 검토, 2026-07-30 langchain_openai 1.3.5로 재확인 — .stream()해도 완성된 객체
    1개만 옴) 응답 텍스트는 먼저 순수 스트리밍으로 뽑고 emotion/action은 텍스트가 다
    나온 뒤 별도로 분류하는 2단계로 나눴다(_stream_dialogue_response 참고)."""
    emotion: Literal['happy', 'sad', 'angry', 'surprise', 'neutral', 'sulky',
                     'love', 'excited'] = Field(
        description='이 응답에 어울리는 로봇의 감정 표현. 평범한 대화면 neutral, '
                     '놀림/무시를 당해서 삐진 상태면 sulky, 애정 표현을 받으면 love, '
                     '함께 신나는 일이 생기면 excited')
    action: Literal['wave_right_arm', 'wave_left_arm', 'wave_both_arms', 'none'] = Field(
        description='이 응답에 어울리는 로봇 동작. 특별히 어울리는 동작이 없으면 none')


NAME_EXTRACTION_PROMPT = (
    "다음은 로봇이 '사용자 등록을 하시겠어요? 성함이 어떻게 되세요?'라고 물은 뒤 "
    "사용자가 한 대답입니다. 여기서 사람 이름을 뽑아내세요. 이름이 아니거나(질문/잡담/"
    "알아들을 수 없는 말 등) 이름인지 확신할 수 없으면 null로 답하세요. "
    "동시에, 사용자가 등록을 원하지 않는다는 의사(예: '아니요', '괜찮아요', '안 할래요', "
    "'그냥 게스트로 할게요', '다음에 할게요' 등)를 밝혔는지도 판단하세요. "
    "마지막으로, 사용자가 이름을 대는 대신 '얼굴 등록해줘'/'등록해줘'/'바로 등록할래'처럼 "
    "이름 확인 없이 얼굴 등록을 곧바로 시작해달라고 요청했는지도 판단하세요."
)


class NameExtraction(BaseModel):
    """게스트 온보딩 중 그 발화에서 이름/등록 거절 의사/즉시 등록 요청을 한 번에
    뽑아내는 1회성 구조화출력. profile_store.ProfileFacts(최대 10턴 지연되는 배치
    추출, _maybe_evict_and_extract가 담당)와 달리, 온보딩은 이 턴 안에서 바로
    확정해야 해서(2026-08-04) 별도 스키마로 둔다."""
    name: str | None = Field(
        default=None, description='발화에서 언급된 사람 이름. 이름이 아니면 null')
    declined: bool = Field(
        default=False, description='사용자가 등록을 원하지 않는다는 의사를 밝혔으면 true')
    wants_face_registration: bool = Field(
        default=False,
        description='이름을 대는 대신 이름 확인 없이 얼굴 등록을 곧바로 시작해달라고 '
                    '요청했으면 true (2026-08-14)')


LEKIWI_SEQUENCE_PROMPT = (
    "다음은 사용자가 LeKiwi 로봇(팔+바퀴)에게 한 요청입니다. 이 요청에 담긴 LeKiwi "
    "동작을 순서대로 전부 나열하세요. 동작 종류: forward(앞으로/전진)/backward(뒤로/후진)/"
    "left(좌회전/왼쪽으로 돌기)/right(우회전/오른쪽으로 돌기)/stop(멈춰/정지). 방향을 안 정하고 "
    "그냥 '돌아'/'회전해'라고만 하면 left로 간주하세요.\n"
    "지속시간(duration_sec)은 초 단위 숫자입니다. 사용자가 초를 직접 말했으면(예: "
    "'10초 동안') 그 값을 그대로 넣으세요. left/right(회전)에서 '몇 도' 또는 '바퀴' "
    "단위로 말했으면, 로봇의 회전 속도가 초당 60도라는 걸 이용해 초로 환산하세요 — "
    "'한 바퀴'/'360도'=6초, '반 바퀴'/'180도'=3초, '90도'=1.5초처럼 (도수)÷60으로 "
    "계산합니다. 아무 지속시간/각도 언급이 없으면 duration_sec은 null로 두세요(그러면 "
    "기본값이 쓰입니다). forward/backward에는 이 각도 환산이 "
    "해당하지 않습니다.\n"
    "동작이 하나뿐이면 목록에 하나만 담으세요."
)


class LekiwiAction(BaseModel):
    skill: Literal['forward', 'backward', 'left', 'right', 'stop']
    duration_sec: float | None = Field(
        default=None, description='사용자가 이 동작에 대해 명시한 지속시간(초). 없으면 null')


class LekiwiActionSequence(BaseModel):
    """LeKiwi 복합 요청("뒤로 갔다가 회전해줘")을 순서가 있는 개별 동작 목록으로
    분해하는 전용 구조화출력(2026-08-14). 일반 tool-calling(run_lekiwi_skill)은
    여러 동작이 담긴 한 발화에서도 도구를 한 번만 호출하고 나머지는 말로만
    약속해버리는 경향이 실기기 테스트로 확인돼(프롬프트로 "여러 번 호출하라"고
    지시해도 개선 안 됨), NameExtraction/ConfirmReply와 같은 이유로 이 판단을
    범용 tool-calling에 맡기지 않고 별도 스키마로 분리한다."""
    actions: list[LekiwiAction] = Field(
        description='사용자가 요청한 LeKiwi 동작들을 요청된 순서 그대로 담은 목록')


CONFIRM_REPLY_PROMPT = (
    "다음은 로봇이 '(이름)님이 맞으신가요?'라고 이름을 확인한 뒤 사용자가 한 대답입니다. "
    "'네'/'응'/'맞아요'처럼 긍정했으면 true, '아니요'/'아니야'/'틀렸어요'처럼 부정했으면 "
    "false, 확인과 무관한 다른 말이면 null로 답하세요."
)


class ConfirmReply(BaseModel):
    """이름 확인('OO님이 맞으신가요?') 단계의 음성/텍스트 답변을 분류하는 1회성
    구조화출력(2026-08-05) — 원래 이 단계는 터치 전용이었으나(음성 오인식 위험 회피),
    실사용 중 텍스트/음성으로 답하면 온보딩이 취소되고 엉뚱하게 처음 인사로 되돌아가는
    문제가 발견돼 추가함. 터치 버튼과 동일한 결과로 합류시킨다."""
    confirmed: bool | None = Field(
        default=None,
        description='"네"류로 확인했으면 true, "아니요"류로 부정했으면 false, '
                    '확인과 무관한 말이면 null')


class DialogueNode(Node):

    def __init__(self):
        super().__init__('dialogue_node')
        self.declare_parameter('llm_provider', 'openai')
        provider = self.get_parameter('llm_provider').get_parameter_value().string_value
        self.declare_parameter('speech_timeout_sec', SPEECH_TIMEOUT_SEC)
        self.speech_timeout_sec = self.get_parameter(
            'speech_timeout_sec').get_parameter_value().double_value

        if provider == 'openai':
            api_key = os.environ.get('OPENAI_API_KEY')
            if not api_key:
                raise RuntimeError('OPENAI_API_KEY 환경변수가 설정되지 않았습니다')
            self.llm = ChatOpenAI(model=OPENAI_MODEL, api_key=api_key, max_tokens=300,
                                  temperature=0.7,
                                  reasoning_effort=OPENAI_REASONING_EFFORT)
        elif provider == 'ollama':  # 오프라인 테스트용 폴백
            from langchain_ollama import ChatOllama
            self.llm = ChatOllama(model="qwen3.5:4b", reasoning=False, num_predict=200)
        else:
            raise ValueError(f'알 수 없는 llm_provider: {provider!r} (openai 또는 ollama만 지원)')
        # 영어모드에서는 언어 지시 준수가 더 중요해서 temperature를 낮춘 인스턴스를 별도로
        # 둔다(2026-07-27, Kimi 컨설팅 — bind()는 provider와 무관하게 동작해 ollama에서도 안전).
        self.llm_focused = self.llm.bind(temperature=ENGLISH_MODE_TEMPERATURE)
        # 응답 텍스트가 다 나온 뒤 emotion/action만 분류하는 가벼운 2차 호출(위 EmotionAction 참고)
        self.classify_llm = self.llm.with_structured_output(EmotionAction)
        self.classify_llm_focused = self.llm_focused.with_structured_output(EmotionAction)
        self.extraction_llm = self.llm.with_structured_output(profile_store.ProfileFacts)
        # 게스트 온보딩 전용 — 위 NameExtraction 참고(2026-08-04)
        self.name_extraction_llm = self.llm.with_structured_output(NameExtraction)
        # 이름 확인 답변 분류 전용 — 위 ConfirmReply 참고(2026-08-05)
        self.confirm_reply_llm = self.llm.with_structured_output(ConfirmReply)
        # LeKiwi 복합 요청 분해 전용 — 위 LekiwiActionSequence 참고(2026-08-14)
        self.lekiwi_sequence_llm = self.llm.with_structured_output(LekiwiActionSequence)
        self.tools = [get_current_weather, look_at_object, teach_object_category,
                      start_english_coaching, start_english_immersion, stop_english_practice,
                      start_word_quiz, stop_word_quiz, set_alarm, cancel_alarm,
                      start_face_registration, log_out, shutdown_robot, run_lekiwi_skill,
                      pickup_medicine]
        self.tools_by_name = {t.name: t for t in self.tools}
        self.tool_llm = self.llm.bind_tools(self.tools)
        model_label = OPENAI_MODEL if provider == 'openai' else 'qwen3.5:4b'
        self.get_logger().info(f'LLM provider: {provider} (model: {model_label})')
        self.histories: dict[str, list] = {}
        self.profiles: dict[str, dict] = {}  # user_id별 저장된 프로필(이름/선호/최근사건) 인메모리 캐시
        # 영어모드/난이도도 얼굴인식으로 대화 상대가 바뀌는 구조에 맞춰 user_id별로 관리한다
        # (전역 단일 플래그가 아님 — self.histories/self.profiles와 동일한 이유, 2026-07-27)
        self.language_modes: dict[str, str] = {}
        self.difficulty_levels: dict[str, str] = {}
        self.english_submodes: dict[str, str] = {}  # 'coach' 또는 'immersion' (language_mode=='en'일 때만 의미)
        # 영단어 퀴즈 진행 상태 — user_id에 키가 없으면 "퀴즈 중이 아님". 있으면
        # {'word': 정답, 'options': [...], 'question_num': int, 'score': int}
        self.word_quiz_state: dict[str, dict] = {}
        # 지금 화면에 떠 있는 퀴즈가 누구 것인지 고정 — 터치 이벤트(on_word_quiz_answer)에는
        # user_id 정보가 전혀 없어서, 콜백 시점의 self.current_user_id를 다시 읽으면 그
        # 사이 얼굴인식이 흔들려(실측으로 확인된 불안정성, guest<->userXXXXX 30~90초
        # 간격으로 오락가락) 답이 조용히 무시되는 버그가 있었음(2026-07-28, Kimi 컨설팅으로
        # 진단+확인). _pending_vision과 같은 원칙: 비동기 간격이 있는 상태는 시작 시점
        # user_id를 캡처해서 고정해두고 끝까지 그 값만 쓴다.
        self._active_quiz_user_id: str | None = None
        # 답 터치 후 ○/✗를 보여주는 동안 도는 타이머(다음 문제로 넘기는 역할).
        # None이 아니면 "결과 표시 중"이라는 뜻이라 연타 차단에도 쓴다.
        self._quiz_advance_timer = None
        # 알람 진행 상태 — user_id에 키가 없으면 "대기 중인 알람 없음"
        self.pending_alarms: dict[str, dict] = {}
        # 알람이 울린 뒤 사용자 반응을 기다리는 중인지(에스컬레이션 판단용)
        self._alarm_awaiting_response: dict[str, bool] = {}
        # 지각 신원 — /user_id를 그대로 반영("지금 카메라에 누가 보이나"). 세션이 없을 때만
        # 대화 정체성으로도 쓰인다(current_user_id 프로퍼티 참고).
        self.perceived_user_id = GUEST_USER_ID
        # 대화 정체성 — 세션 시작 시 캡처해서 세션이 끝날 때까지 고정. None이면 세션 없음.
        self.session_user_id: str | None = None
        # 재방문 판정용 — 직전 세션이 누구였고 언제 끝났는지
        self._last_session_user_id: str | None = None
        self._last_session_end_time: float | None = None
        # 게스트 온보딩(이름 즉시인식→확인→얼굴등록) 상태 — 세션당 하나만 존재(대화 상대가
        # 항상 한 명)하므로 _pending_vision과 같은 전역 단일 상태 패턴(2026-08-04)
        self._onboarding_state: str | None = None  # None | 'awaiting_name' | 'awaiting_confirm'
        self._onboarding_candidate_name: str | None = None
        self._onboarding_attempts = 0
        # 등록이 온보딩에서 트리거된 경우에만 채워짐 — 등록 완료(done) 시 이 이름을 새
        # user_id 프로필에 저장한다. 기존 음성 트리거("얼굴 등록해줘")로 시작된 등록은
        # None인 채로 기존 동작(이름 없이 등록) 그대로 유지된다.
        self._onboarding_registration_name: str | None = None
        # 이름 확인 없이 곧바로 등록을 시작한 경우(wants_face_registration 단축,
        # 2026-08-14)에만 채워짐 — 등록 완료(done/duplicate) 시 이 user_id를 대상으로
        # 뒤늦게 성함을 물어본다(_ask_name_after_registration). 기존 독립 트리거
        # ("얼굴 등록해줘"를 온보딩과 무관하게 말한 경우)는 이 필드를 안 건드려서
        # 그대로 "이름 없이 등록" 동작을 유지한다.
        self._registration_name_target_user_id: str | None = None
        # wants_face_registration 단축으로 등록이 시작됐음을 기록 — 등록 완료
        # (done/duplicate) 시점에 이 값을 보고 이름을 물어볼지 판단한다.
        self._registration_needs_name: bool = False
        # 얼굴 등록이 진행 중인지(카메라 포즈 5단계) — on_registration_prompt가
        # step:/progress:를 받으면 True, done/failed/duplicate/cancelled를 받으면
        # False로 되돌린다(2026-08-14). on_user_input이 이 플래그를 보고 등록과
        # 무관한 tool-call(날씨/퀴즈 등)이 등록 도중 끼어들지 않게 걸러낸다.
        self._face_registration_active: bool = False
        # 웨이크워드/얼굴인식 이중 트리거로 세션을 시작하는 상태머신 — IDLE 상태의
        # /user_input은 전부 무시한다(그동안 VAD가 배경소음에 계속 오탐하던 문제를
        # 웨이크워드 게이트로 원천 차단). 설계는 Kimi와 논의해 dialogue_node에 내장(2026-07-21).
        self.session = SessionManager()
        self._mic_enabled: bool | None = None  # None이면 아직 한 번도 발행 안 함 — 최초 sync 때 무조건 발행
        self._session_active: bool | None = None  # 위와 동일 패턴(/session_active 발행용)
        self._speech_timeout_timer = None
        self._mic_grace_timer = None  # 발화 완료 후 마이크 재활성화 유예 타이머(위 MIC_REENABLE_GRACE_SEC 참고)
        self._arm_goal_handle = None  # 현재 실행 중인 팔 동작 goal (없으면 None)
        self._is_arm_cancel_pending = False  # goal accept 응답 오기 전에 취소 요청이 먼저 온 경우
        self._arm_action_client = ActionClient(self, MoveArm, 'move_arm')
        # 카메라 왕복이 필요한 도구 호출을 보류해두는 자리 — 결과가 오면 이어서 턴을 재개한다.
        # (None이 아니면 "지금 스냅샷 결과를 기다리는 중")
        self._pending_vision: dict | None = None
        self._vision_timeout_timer = None
        self._vision_request_seq = 0   # 요청 번호. uuid 대신 단조증가 정수(로그 가독성)
        # 로그아웃/종료 음성 명령(session_control_tool.py) — 대화 상대가 항상 한 명이므로
        # _pending_vision/_onboarding_state와 같은 전역 단일 상태 패턴(2026-08-06).
        # 고정 문구 발화 후 /speech_finished가 와야 다음 단계로 진행하므로, 그때까지
        # "누구의 로그아웃인지"/"종료가 몇 단계까지 왔는지"를 여기 들고 있는다.
        self._pending_logout_user_id: str | None = None
        self._pending_shutdown_step: str | None = None  # None | 'farewell' | 'goodbye'
        self.subscription = self.create_subscription(
            String, '/user_input', self.on_user_input, 10)
        self.user_id_subscription = self.create_subscription(
            String, '/user_id', self.on_user_id, 10)
        self.wake_word_subscription = self.create_subscription(
            Empty, '/wake_word_detected', self.on_wake_word, 10)
        self.speech_finished_subscription = self.create_subscription(
            Empty, '/speech_finished', self.on_speech_finished, 10)
        self.publisher = self.create_publisher(String, '/llm_response', 10)
        # 문장 단위 스트리밍(2026-07-30)에서 한 턴에 /llm_response가 여러 번(문장마다) 나갈
        # 수 있어, tts_node가 "이 턴의 마지막 문장까지 다 재생했다"를 알아야 /speech_finished를
        # 정확히 한 번만 낼 수 있다 — 그 마지막 신호. _speak()(정적 텍스트, 항상 단발성)도
        # 매번 이걸 같이 낸다.
        self.llm_response_done_publisher = self.create_publisher(Empty, '/llm_response_done', 10)
        self.emotion_publisher = self.create_publisher(String, '/emotion', 10)
        self.action_publisher = self.create_publisher(String, '/action', 10)
        self.mic_enabled_publisher = self.create_publisher(Bool, '/mic_enabled', 10)
        # face_id_node(파이)가 구독해서 카메라 탐지 주기를 조절한다 — 세션 중엔 대화 상대가
        # 이미 고정돼 있어(session_user_id) 얼굴인식을 자주 돌릴 이유가 없기 때문. 늦게 뜬
        # 구독자도 현재 상태를 바로 받도록 /user_id와 동일한 QoS(TRANSIENT_LOCAL) 사용.
        session_active_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.session_active_publisher = self.create_publisher(
            Bool, '/session_active', session_active_qos)
        self.interrupt_publisher = self.create_publisher(Empty, '/interrupt_speech', 10)
        self.snapshot_request_publisher = self.create_publisher(
            UInt32, '/object_snapshot_request', 10)
        self.object_label_publisher = self.create_publisher(String, '/object_label', 10)
        # 학생용 AI 투명성 대시보드(education_bridge_node)가 관찰하는 도구 호출 이벤트
        self.tool_call_publisher = self.create_publisher(ToolCallEvent, '/tool_call', 10)
        self.object_result_subscription = self.create_subscription(
            ObjectClassification, '/object_classification', self.on_object_classification, 10)
        # 물체 "가르치기"(2026-07-29) — /object_snapshot_request(UInt32)로 face_id_node의
        # 카메라 왕복을 그대로 재사용하되, 같은 request_id로 이 토픽에 라벨을 같이 보내
        # object_classify_node가 "이 사진은 분류 대신 저장하라"는 걸 알게 한다.
        self.object_teach_request_publisher = self.create_publisher(
            ObjectTeachRequest, '/object_teach_request', 10)
        self.object_teach_result_subscription = self.create_subscription(
            ObjectTeachResult, '/object_teach_result', self.on_object_teach_result, 10)
        # 세션종료/새입력으로 가르치기 요청을 능동적으로 포기할 때 object_classify_node에
        # 즉시 알려서, 이미 포기한 요청의 스냅샷이 뒤늦게 도착해도 조용히 저장되지 않게
        # 한다(ObjectTeachRequest.deadline과 역할 분담 — deadline은 카메라가 늦게 응답하는
        # 수동적 만료, 이건 즉시 무효화. 2026-08-03).
        self.object_teach_cancel_publisher = self.create_publisher(
            UInt32, '/object_teach_cancel', 10)
        # face_recognize_node(노트북)가 등록 완료/실패 시 보내는 신호 — 로봇이 결과를
        # 음성으로 안내하게 함(2026-07-28 사용자 요청). 진행 중 신호('step:'/'progress:')는
        # face_display_node의 화면 안내용이라 여기선 무시하고 최종 결과만 반응한다.
        self.registration_result_subscription = self.create_subscription(
            String, '/face_registration_prompt', self.on_registration_prompt, 10)
        # 동반 웹앱에서 프로필 이름을 편집/삭제하면(2026-08-05, companion_bridge_node) 이
        # 신호를 받는다 — self.profiles는 _get_history가 최초 1회만 파일에서 로드해 계속
        # 캐싱하므로, 파일만 고쳐서는 이미 대화한 적 있는 user_id에 반영이 안 된다(리셋
        # 트리거가 self.profiles.pop을 쓰는 것과 동일한 이유).
        self.profile_updated_subscription = self.create_subscription(
            String, '/profile_updated', self.on_profile_updated, 10)
        # start_face_registration 도구 호출을 감지했을 때 face_recognize_node에 등록 시작을
        # 알리는 신호(2026-07-28) — 예전엔 face_recognize_node가 /user_input을 직접 구독해
        # 정확한 문구('얼굴 등록해줘'/'얼굴 등록해')만 매칭했는데, "얼굴 등록할래"처럼 표현이
        # 조금만 달라도 못 잡던 문제(실사용 발견)를 tool-calling으로 해결하면서 이 신호로 대체.
        self.start_registration_publisher = self.create_publisher(
            Empty, '/start_face_registration', 10)
        # 등록 진행 중 탈출구(2026-08-14) — face_recognize_node에 지금 진행 중인 등록을
        # 중단하라고 알린다.
        self.registration_cancel_publisher = self.create_publisher(
            Empty, '/registration_cancel', 10)
        # run_lekiwi_skill 도구 호출을 감지했을 때 lekiwi_command_node에 실행할 스킬을
        # 알리는 신호 — 실제 SSH 트리거는 이 노드가 아니라 lekiwi_command_node가 담당한다
        # (look_at_object/teach_object_category가 face_id_node/object_classify_node에
        # 실행을 위임하는 것과 동일한 패턴).
        self.lekiwi_command_publisher = self.create_publisher(String, '/lekiwi_command', 10)
        # companion_bridge_node이 파이에서 돌 때(2026-08-06) ~/.roboseasy_faces/profiles/를
        # 직접 못 읽어서 대신 물어보는 토픽 — 얼굴 임베딩(chromadb) 삭제는 기존
        # /face_delete_request<->/face_delete_result 왕복을 그대로 쓰고, 이 쪽은 프로필
        # JSON 파일(목록/수정/삭제)만 담당한다.
        self.profile_list_query_subscription = self.create_subscription(
            ProfileListQuery, '/profile_list_query', self.on_profile_list_query, 10)
        self.profile_list_result_publisher = self.create_publisher(
            ProfileListResult, '/profile_list_result', 10)
        self.profile_edit_query_subscription = self.create_subscription(
            ProfileEditQuery, '/profile_edit_query', self.on_profile_edit_query, 10)
        self.profile_edit_result_publisher = self.create_publisher(
            ProfileEditResult, '/profile_edit_result', 10)
        # 동반 웹앱의 "OOO님과의 기억" 스크랩북 화면용(2026-08-14) — 위와 동일한 이유로
        # 토픽 왕복, ProfileListQuery와 짝을 이루는 단일 사용자 상세 조회.
        self.profile_detail_query_subscription = self.create_subscription(
            ProfileDetailQuery, '/profile_detail_query', self.on_profile_detail_query, 10)
        self.profile_detail_result_publisher = self.create_publisher(
            ProfileDetailResult, '/profile_detail_result', 10)
        # 게스트 온보딩(2026-08-04) — 이름확인 화면 상태는 이 노드가 발행하고,
        # face_display_node(터치)/companion_bridge_node(브라우저 폼) 둘 다 같은 답변
        # 토픽에 발행한다(먼저 도착한 쪽 채택, 상태 가드로 나머지는 자연히 무시됨).
        self.name_confirm_prompt_publisher = self.create_publisher(String, '/name_confirm_prompt', 10)
        self.name_confirm_answer_subscription = self.create_subscription(
            String, '/name_confirm_answer', self.on_name_confirm_answer, 10)
        # 브이 사인 사진 촬영(2026-08-03 사용자 요청, 2026-08-13 셔터버튼 방식으로 개편) —
        # photo_capture_node가 제스처를 확정하면 이 신호를 보내고, face_display_node가
        # 카메라 미리보기+셔터 버튼 화면을 띄운다. 실제 촬영은 버튼을 눌러야 이뤄진다
        # (예전엔 고정 카운트다운 후 자동 촬영이라 "타이밍을 몰라 눈 감은 사진이 찍힌다"는
        # 문제가 있었음). 이 노드는 "말하는 것"만 담당 — 카메라 프레임을 안 가지고 있어
        # 촬영 자체는 할 수 없다. 알람/등록결과 안내와 동일한 이유로 세션 상태를 확인해서
        # 안전할 때만 말한다(다른 대화를 안 끊음).
        self.photo_countdown_subscription = self.create_subscription(
            Empty, '/photo_countdown_request', self.on_photo_countdown_request, 10)
        self.difficulty_level_subscription = self.create_subscription(
            String, '/difficulty_level', self.on_difficulty_level, 10)
        # 영단어 퀴즈 — 문제(JSON)는 이 노드가 발행, 답은 face_display_node가 터치로 받아 발행
        self.word_quiz_question_publisher = self.create_publisher(String, '/word_quiz_question', 10)
        self.word_quiz_answer_subscription = self.create_subscription(
            String, '/word_quiz_answer', self.on_word_quiz_answer, 10)
        # face_display_node가 재시작돼도 마지막 모드를 바로 받도록 /user_id(face_id_node)와
        # 동일한 QoS(TRANSIENT_LOCAL) 사용
        language_mode_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.language_mode_publisher = self.create_publisher(
            String, '/language_mode', language_mode_qos)
        # 동반 웹앱(companion_bridge_node)의 언어모드 버튼 — 2026-07-31, 발행 쪽은 이미
        # 있었는데 이 노드가 구독을 안 해서 클릭이 아무 효과가 없던 버그를 발견해 추가.
        self.language_mode_request_subscription = self.create_subscription(
            String, '/language_mode_request', self.on_language_mode_request, 10)
        # 로그아웃/종료 음성 명령(session_control_tool.py, 2026-08-06) — dialogue_node는
        # 노트북 쪽 노드이므로 파이 종료 신호는 발행만 하고, 노트북 종료 신호는 다른
        # 노트북 노드들과 동일하게 자기 자신도 구독해 종료한다(설계 4번 "각 노드가
        # 자신의 종료 신호를 구독" 원칙 그대로).
        self.shutdown_pi_publisher = self.create_publisher(Empty, '/shutdown_pi', 10)
        self.shutdown_laptop_publisher = self.create_publisher(Empty, '/shutdown_laptop', 10)
        self.shutdown_laptop_subscription = self.create_subscription(
            Empty, '/shutdown_laptop', self._on_shutdown_signal, 10)
        # 향후 "사용자 전환 화면" 기능을 위한 인터페이스 지점 — 현재 구독자 없음(발행만).
        self.user_switch_requested_publisher = self.create_publisher(
            Empty, '/user_switch_requested', 10)
        self.create_timer(SESSION_TIMEOUT_CHECK_SEC, self._check_session_timeout)
        self._sync_mic_state()
        self.get_logger().info('Dialogue node ready. 웨이크워드 대기 중 (IDLE)')

    @property
    def current_user_id(self) -> str:
        """대화 정체성(engaged user) — 대화 기록/프로필/영어모드/알람/퀴즈의 주인.

        세션 중이면 세션 시작 시점에 캡처한 신원을 끝까지 쓰고, 세션이 없을 때만 지금
        보이는 사람(지각 신원)을 쓴다. 세션 도중 얼굴인식이 흔들려 guest가 오더라도 이
        값은 안 바뀐다 — 위 상수 주석의 비대칭 처리 원칙 참고."""
        return self.session_user_id if self.session_user_id is not None else self.perceived_user_id

    def _get_history(self, user_id: str) -> list:
        if user_id not in self.histories:
            # guest는 프로필 저장 대상이 아님 — 얼굴 미인식 상태에서 나온 발화가 특정
            # user_id 프로필을 오염시키지 않도록 방어(아래 _maybe_evict_and_extract도 동일)
            profile = {} if user_id == GUEST_USER_ID else profile_store.load_profile(user_id)
            self.profiles[user_id] = profile
            if user_id == GUEST_USER_ID:
                # 아직 이름을 모르는 게스트 — 온보딩 대기 중이든 등록을 거절했든
                # 이름을 불러야 할 때는 "사용자님"으로 통일한다.
                profile_text = '[호칭 안내] 이 사용자는 아직 이름을 모릅니다. 이름을 ' \
                    '불러야 할 때는 "사용자님"이라고 부르세요.'
            else:
                profile_text = profile_store.format_profile_for_prompt(profile)
            system_text = f'{SYSTEM_PROMPT}\n\n{profile_text}' if profile_text else SYSTEM_PROMPT
            self.histories[user_id] = [SystemMessage(content=system_text)]
            self.language_modes[user_id] = profile.get('language_mode', DEFAULT_LANGUAGE_MODE)
            self.difficulty_levels[user_id] = profile.get(
                'difficulty_level', DEFAULT_DIFFICULTY_LEVEL)
            self.english_submodes[user_id] = profile.get(
                'english_submode', DEFAULT_ENGLISH_SUBMODE)
        return self.histories[user_id]

    def _build_mode_directive(self, user_id: str) -> str:
        """이번 턴에 어떤 언어로 답할지 지시하는 문구를 만든다(항상 반환 — SYSTEM_PROMPT에
        언어를 고정해두면 영어모드 지시와 서로 모순돼 LLM이 언어를 오락가락하는 문제가
        있었음, 2026-07-30 Kimi 컨설팅으로 확인+재현). 저장된 히스토리는 건드리지 않고
        매턴 새로 계산해 얹는다 — 모드를 껐다 켰다 해도 히스토리가 오염되지 않게.

        현재 시각도 여기 같이 붙인다(2026-07-28, 알람 기능 추가하며 도입) — "7시에
        깨워줘" 같은 절대 시각 표현을 set_alarm(minutes_from_now)으로 변환하려면 LLM이
        지금 몇 시인지 알아야 하는데, 이 문구가 이미 매턴 붙는 유일한 임시 컨텍스트라
        여기 얹는 게 새 메커니즘을 안 만들고 재사용하는 방법이었음."""
        time_note = f' (현재 시각: {datetime.now().strftime("%H:%M")})'
        if self.language_modes.get(user_id, DEFAULT_LANGUAGE_MODE) != 'en':
            return f'이번 응답은 한국어로 작성하세요.{time_note}'
        submode = self.english_submodes.get(user_id, DEFAULT_ENGLISH_SUBMODE)
        difficulty = self.difficulty_levels.get(user_id, DEFAULT_DIFFICULTY_LEVEL)
        return (f'{ENGLISH_SUBMODE_INSTRUCTIONS[submode]} {DIFFICULTY_INSTRUCTIONS[difficulty]}'
                f'{time_note}')

    def _stream_dialogue_response(self, llm_input: list, user_id: str) -> str:
        """LLM 응답을 스트리밍하며 문장이 완성되는 대로 즉시 /llm_response로 내보낸다 —
        응답을 생성하는 동안 첫 문장부터 먼저 말하기 시작해 체감 지연을 줄인다
        (2026-07-30). emotion/action은 구조화 출력이라 스트리밍이 안 돼(위 EmotionAction
        참고) 여기서 같이 못 받는다 — 텍스트가 다 나온 뒤 _classify_emotion_action이
        별도로 채운다.

        도중에 오류가 나도 이미 말한 문장은 되돌릴 수 없으므로 예외를 여기서 삼키고
        지금까지 생성된 텍스트를 그대로 반환한다 — 호출부(_generate_streamed_response)는
        반환값이 완전히 빈 문자열일 때만 실패로 간주해 기존 폴백(_speak)으로 넘어간다.

        ⚠️ 영어 실습(immersion) 모드에서 응답에 한국어가 섞이면 재시도하던 안전장치
        (예전 _invoke_dialogue_llm에 있었음)는 스트리밍에서는 뺐다 — 이미 앞 문장을
        말하기 시작한 뒤에 "재시도"하면 방금 한 말을 취소하고 처음부터 다시 말하는
        어색한 경험이 되기 때문. 실사용 중 한국어 섞임이 자주 보이면 재검토할 것.
        """
        is_english_mode = self.language_modes.get(user_id, DEFAULT_LANGUAGE_MODE) == 'en'
        llm = self.llm_focused if is_english_mode else self.llm
        buffer = ''
        full_text_parts: list[str] = []
        spoke_anything = False
        stream_start = time.monotonic()  # [측정] "텀" 원인이 스트리밍 자체인지 확인용 (2026-08-03)
        try:
            for chunk in llm.stream(llm_input):
                content = chunk.content
                if not content:
                    continue
                full_text_parts.append(content)
                buffer += content
                ready, buffer = _extract_ready_sentences(buffer)
                for sentence in ready:
                    if not spoke_anything:
                        self.get_logger().info(
                            f'[측정] 스트리밍 시작→첫 문장 도착='
                            f'{(time.monotonic() - stream_start) * 1000:.0f}ms')
                        self._begin_speaking_turn()
                        spoke_anything = True
                    else:
                        self._refresh_speech_timeout()
                    self._publish_sentence(sentence)
            tail = buffer.strip()
            if tail:
                if not spoke_anything:
                    self.get_logger().info(
                        f'[측정] 스트리밍 시작→첫 문장 도착='
                        f'{(time.monotonic() - stream_start) * 1000:.0f}ms')
                    self._begin_speaking_turn()
                    spoke_anything = True
                else:
                    self._refresh_speech_timeout()
                self._publish_sentence(tail)
        except Exception as e:
            self.get_logger().error(f'응답 스트리밍 중 오류(지금까지 생성된 내용으로 마무리): {e}')
        finally:
            if spoke_anything:
                self.llm_response_done_publisher.publish(Empty())
        return ''.join(full_text_parts)

    def _classify_emotion_action(self, response_text: str, user_id: str) -> tuple[str, str]:
        """이미 완성된 응답 텍스트만 보고 emotion/action을 분류하는 가벼운 2차 호출.
        이 호출이 도는 동안 이미 TTS가 첫 문장부터 말하고 있으므로 지연이 체감되지 않는다."""
        is_english_mode = self.language_modes.get(user_id, DEFAULT_LANGUAGE_MODE) == 'en'
        llm = self.classify_llm_focused if is_english_mode else self.classify_llm
        try:
            result = llm.invoke([
                SystemMessage(content=EMOTION_ACTION_CLASSIFY_PROMPT),
                HumanMessage(content=response_text),
            ])
            return result.emotion, result.action
        except Exception as e:
            self.get_logger().warn(f'감정/행동 분류 실패, 기본값 사용: {e}')
            return 'neutral', NO_ARM_ACTION

    def _generate_streamed_response(self, llm_input: list, user_id: str) -> tuple[str, str, str]:
        """스트리밍 생성(문장 단위 조기 발화) + 완료 후 emotion/action 분류를 묶어서
        처리한다. 아무 텍스트도 못 만들면 예외를 올려 호출부가 기존 폴백(_speak로 미리
        정해둔 문구 말하기)으로 넘어가게 한다 — 부분적으로라도 말을 시작했다면(일부
        문장은 이미 스피커로 나갔으므로) 실패로 취급하지 않고 그 텍스트로 마무리한다."""
        text = self._stream_dialogue_response(llm_input, user_id)
        if not text.strip():
            raise RuntimeError('스트리밍 응답이 비어있음')
        emotion, action = self._classify_emotion_action(text, user_id)
        self._publish_emotion_action(emotion, action)
        if action != NO_ARM_ACTION:
            self._send_arm_goal(action)
        return text, emotion, action

    def _persist_mode_settings(self, user_id: str):
        if user_id == GUEST_USER_ID:
            return
        profile = self.profiles.get(user_id) or profile_store.load_profile(user_id)
        profile['language_mode'] = self.language_modes.get(user_id, DEFAULT_LANGUAGE_MODE)
        profile['difficulty_level'] = self.difficulty_levels.get(
            user_id, DEFAULT_DIFFICULTY_LEVEL)
        profile['english_submode'] = self.english_submodes.get(
            user_id, DEFAULT_ENGLISH_SUBMODE)
        self.profiles[user_id] = profile
        profile_store.save_profile(profile)

    def _publish_language_mode(self, user_id: str):
        msg = String()
        msg.data = self.language_modes.get(user_id, DEFAULT_LANGUAGE_MODE)
        self.language_mode_publisher.publish(msg)

    def on_user_id(self, msg: String):
        if msg.data != self.perceived_user_id:
            self.get_logger().info(f'인식된 사용자 변경: {self.perceived_user_id} -> {msg.data}')
            self.perceived_user_id = msg.data
        self._reconcile_session_identity(msg.data)

    def _reconcile_session_identity(self, perceived: str):
        """지각 신원이 바뀌었을 때 대화 정체성을 어떻게 할지 판단한다(세션 중일 때만).

        획득/상실을 비대칭으로 다루는 게 핵심 — 위 상수 주석 참고. 세션 중 다른 등록
        사용자가 보여도 대화 상대는 안 바뀐다(2026-08-04 — 얼굴만으로 세션/대화상대를
        바꾸는 경로를 전부 제거, 지금은 웨이크워드로만 세션이 시작/전환된다).
        """
        if self.session_user_id is None:
            return   # 세션 없음 — current_user_id가 알아서 지각 신원을 따라감
        if perceived == GUEST_USER_ID:
            # "아무도 안 보임"은 정보의 부재일 뿐 신원이 아니다 — 대화 정체성을 안 바꾼다.
            # (사용자가 고개를 돌리거나 잠깐 자리를 비운 경우가 대부분)
            return
        if perceived == self.session_user_id:
            return
        if self.session_user_id == GUEST_USER_ID:
            # guest로 시작한 세션에서 신원이 확인됨 = 정보의 획득이므로 즉시 승격한다.
            # 그동안 guest로 쌓인 대화는 실제로 이 사람이 한 말이므로 넘겨준다(버리면
            # 방금 나눈 대화를 로봇이 잊은 것처럼 보임).
            self._promote_guest_session(perceived)
        # 그 외(다른 등록 사용자가 보임)엔 아무 일도 하지 않는다 — 대화 상대는 그대로 유지.

    def _promote_guest_session(self, user_id: str):
        """guest로 시작한 세션의 정체성을 실제 신원으로 승격하고, 대화 기록을 넘긴다."""
        guest_history = self.histories.pop(GUEST_USER_ID, None)
        self.session_user_id = user_id
        history = self._get_history(user_id)   # 프로필 로드 + 시스템 프롬프트 구성
        if guest_history:
            # guest 히스토리의 SystemMessage(프로필 없는 버전)는 버리고 실제 대화만 이어붙임
            history.extend(m for m in guest_history if not isinstance(m, SystemMessage))
        # 카메라 왕복을 기다리는 tool_calls가 guest 히스토리 끝에 남아있을 수 있다(2026-08-03
        # 코드리뷰로 발견) — _pending_vision['user_id']를 그대로 두면 _resume_vision_turn/
        # _clear_pending_vision이 이미 pop된 'guest' 키로 _get_history를 호출해 새 빈
        # 히스토리를 만들고, 짝이 맞는 tool_calls/ToolMessage가 서로 다른 히스토리에
        # 갈라져 둘 다 영구히 깨진다(다음 LLM 호출이 400으로 계속 실패). guest 히스토리를
        # 넘겨준 것과 같은 이유로, 그 히스토리에 딸린 보류 요청도 같이 넘긴다.
        if self._pending_vision is not None and self._pending_vision['user_id'] == GUEST_USER_ID:
            self._pending_vision['user_id'] = user_id
        self._publish_language_mode(user_id)
        self.get_logger().info(f'세션 신원 승격: guest -> {user_id} (대화 기록 이관)')

    def on_wake_word(self, _msg: Empty):
        if self.session.request_session_start('wake_word', user_id=self.current_user_id):
            self.get_logger().info('웨이크워드 트리거로 세션 시작')
            self._start_session(self.current_user_id)
        else:
            self.get_logger().debug('웨이크워드 감지됐지만 이미 세션 중이거나 디바운스됨')

    def on_difficulty_level(self, msg: String):
        if msg.data not in DIFFICULTY_INSTRUCTIONS:
            self.get_logger().warn(f'알 수 없는 난이도 값 무시: {msg.data!r}')
            return
        self.difficulty_levels[self.current_user_id] = msg.data
        self._persist_mode_settings(self.current_user_id)
        self.session.record_activity()
        self.get_logger().info(f'난이도 변경: user_id={self.current_user_id} -> {msg.data}')

    def on_language_mode_request(self, msg: String):
        """동반 웹앱의 언어모드 버튼 — 음성 tool-calling(english_mode_tool)과 달리 사용자
        의도 판단이 필요 없는 명시적 버튼 클릭이라 바로 반영한다. 실습 진행 중인 대화
        상대(current_user_id)를 대상으로 적용 — 대화 세션이 없으면 guest에 적용된다."""
        if msg.data not in ('ko', 'en'):
            self.get_logger().warn(f'알 수 없는 언어모드 요청 무시: {msg.data!r}')
            return
        self.language_modes[self.current_user_id] = msg.data
        if msg.data == 'en' and self.current_user_id not in self.english_submodes:
            self.english_submodes[self.current_user_id] = DEFAULT_ENGLISH_SUBMODE
        self._persist_mode_settings(self.current_user_id)
        self._publish_language_mode(self.current_user_id)
        self.session.record_activity()
        self.get_logger().info(
            f'동반 웹앱에서 언어모드 변경: user_id={self.current_user_id} -> {msg.data}')

    def on_registration_prompt(self, msg: String):
        """face_recognize_node의 등록 진행 신호를 처리한다.

        'step:'/'progress:'는 화면 안내 전용이라 음성 반응은 없지만, 등록이 지금
        진행 중이라는 것 자체는 _face_registration_active로 추적한다(2026-08-14,
        on_user_input이 이걸 보고 등록 도중 무관한 tool-call을 걸러낸다). 최종 결과
        ('done:.../failed'/'duplicate:{existing_id}'/'cancelled')가 오면 다시 False로
        되돌린다 — 'cancelled'는 탈출구(on_user_input)가 이미 취소 안내를 말한
        뒤라 여기선 추가로 말하지 않는다.

        온보딩(게스트가 이름 확인 후 등록을 시작한 경우, _onboarding_registration_name이
        채워져 있음)이면 새 user_id(또는 duplicate로 걸린 기존 user_id, 2026-08-13)에
        확인된 이름을 붙이고 세션을 승격한다(_complete_onboarding_registration) —
        기존 음성 트리거("얼굴 등록해줘")로 시작된 등록/중복은 이 이름이 없어 기존 동작
        (이름 없이 등록/그냥 안내만) 그대로 유지된다. face_recognize_node는
        'done:{user_id}:{count}' 형식으로 발행한다(2026-08-04, 예전엔 count만
        있었음 — 온보딩이 새 user_id를 알아야 해서 확장)."""
        if msg.data.startswith('step:') or msg.data.startswith('progress:'):
            self._face_registration_active = True
            return
        self._face_registration_active = False
        if msg.data == 'cancelled':
            return
        if msg.data.startswith('done'):
            parts = msg.data.split(':')
            new_user_id = parts[1] if len(parts) > 1 else None
            self._handle_registration_outcome(new_user_id, duplicate=False)
        elif msg.data == 'failed:limit_reached':
            self._announce_registration_result(
                '이미 5명이 다 등록되어 있어요, 기존 프로필을 정리하고 다시 시도해주세요.', 'neutral')
            self._onboarding_registration_name = None
            self._registration_needs_name = False
        elif msg.data == 'failed':
            self._announce_registration_result('얼굴 등록에 실패했어요. 다시 시도해주세요.', 'neutral')
            self._onboarding_registration_name = None
            self._registration_needs_name = False
        elif msg.data.startswith('duplicate'):
            # 온보딩 중이었다면(이름을 이미 확인받음) 그 이름을 이 기존 얼굴에 붙인다
            # (2026-08-13) — 안 그러면 "이미 등록된 얼굴"이 영원히 이름 없는 채로 남고,
            # 확인받은 이름은 그냥 버려진다(같은 이유로 이 얼굴이 계속 guest 취급되어
            # 매번 다시 온보딩 인사를 받게 됨). done 경로와 동일하게 처리해 재사용한다.
            parts = msg.data.split(':')
            existing_id = parts[1] if len(parts) > 1 else None
            self._handle_registration_outcome(existing_id, duplicate=True)

    def _handle_registration_outcome(self, user_id: str | None, duplicate: bool) -> None:
        """done/duplicate 공통 처리 — 우선순위: ①온보딩에서 이미 확인받은 이름이
        있으면 그 자리에서 붙이고 ②wants_face_registration 단축이라 이름을 아직
        안 물어봤으면(_registration_needs_name) 지금 물어보고 ③둘 다 아니면(기존
        독립 트리거) 그냥 결과만 안내한다(2026-08-14)."""
        if self._onboarding_registration_name is not None and user_id:
            self._complete_onboarding_registration(user_id, self._onboarding_registration_name)
        elif self._registration_needs_name and user_id:
            self._ask_name_after_registration(user_id)
        else:
            text = '이미 등록되어 있는 얼굴이에요!' if duplicate else '얼굴 등록이 완료되었습니다!'
            self._announce_registration_result(text, 'happy')
        self._onboarding_registration_name = None
        self._registration_needs_name = False

    def _ask_name_after_registration(self, user_id: str) -> None:
        """wants_face_registration 단축으로 이름 확인 없이 등록부터 끝낸 경우, 등록이
        끝난 뒤 뒤늦게 성함을 물어본다(2026-08-14) — 이름 없이 끝나면 이 사람은
        계속 guest로 남아 매번 온보딩 인사를 다시 받게 되므로. 답변은 기존
        _handle_onboarding_name_reply(awaiting_name)가 그대로 처리하며,
        _registration_name_target_user_id로 "이미 등록된 얼굴에 이름만 붙이면 됨"을
        구분한다(카메라를 다시 돌리지 않음, _handle_onboarding_confirmed 참고)."""
        if not self._registration_may_speak_now(user_id):
            def retry():
                timer.cancel()
                self._ask_name_after_registration(user_id)
            timer = self.create_timer(ALARM_RETRY_DELAY_SEC, retry)
            return
        self._registration_name_target_user_id = user_id
        self._onboarding_state = 'awaiting_name'
        self._onboarding_attempts = 0
        self._speak('등록됐어요! 성함이 어떻게 되세요?', 'happy', 'none')
        self._publish_name_confirm_prompt('listening')

    def on_profile_updated(self, msg: String):
        self.profiles.pop(msg.data, None)

    def on_profile_list_query(self, msg: ProfileListQuery) -> None:
        """companion_bridge_node이 파이에서 돌 때(2026-08-06) 프로필 목록을 직접 파일로
        못 읽어서 대신 물어보는 요청 — companion_bridge_node의 예전 /profiles 라우트가
        하던 파일 순회 로직을 그대로 옮겼다."""
        profiles = []
        for path in sorted(profile_store.PROFILES_DIR.glob('*.json')):
            if path.stem == GUEST_USER_ID:
                continue
            try:
                data = json.loads(path.read_text())
            except (FileNotFoundError, json.JSONDecodeError):
                continue
            profiles.append({
                'user_id': path.stem,
                'name': data.get('name'),
                'avatar_face_key': data.get('avatar_face_key', 'happy'),
            })
        result = ProfileListResult()
        result.request_id = msg.request_id
        result.profiles_json = json.dumps(profiles, ensure_ascii=False)
        result.max_profiles = MAX_REGISTERED_USERS
        self.profile_list_result_publisher.publish(result)

    def on_profile_detail_query(self, msg: ProfileDetailQuery) -> None:
        """동반 웹앱의 "OOO님과의 기억" 스크랩북 화면(2026-08-14) — on_profile_list_query와
        같은 방식으로 프로필 JSON 파일을 직접 읽어 좋아하는것/싫어하는것/최근사건까지
        전부 돌려준다(목록 조회는 카드용이라 이름/아바타만 담아 가볍게 유지)."""
        result = ProfileDetailResult()
        result.request_id = msg.request_id
        user_id = msg.user_id
        if user_id == GUEST_USER_ID:
            result.ok = False
            result.error = '게스트는 기억이 없습니다'
            self.profile_detail_result_publisher.publish(result)
            return
        path = profile_store.PROFILES_DIR / f'{user_id}.json'
        try:
            data = json.loads(path.read_text())
        except (FileNotFoundError, json.JSONDecodeError):
            result.ok = False
            result.error = '프로필을 찾을 수 없음'
            self.profile_detail_result_publisher.publish(result)
            return
        result.ok = True
        result.name = data.get('name') or ''
        result.likes_json = json.dumps(data.get('likes', []), ensure_ascii=False)
        result.dislikes_json = json.dumps(data.get('dislikes', []), ensure_ascii=False)
        result.recent_events_json = json.dumps(data.get('recent_events', []), ensure_ascii=False)
        self.profile_detail_result_publisher.publish(result)

    def on_profile_edit_query(self, msg: ProfileEditQuery) -> None:
        """companion_bridge_node의 프로필 수정/삭제 요청을 처리한다(2026-08-06) — 얼굴
        임베딩(chromadb) 삭제는 companion_bridge_node이 기존 /face_delete_request<->
        /face_delete_result 왕복으로 먼저 처리한 뒤에만 op='delete'로 이 메시지를
        보낸다(그 왕복은 이미 토픽 기반이라 파이에서 돌아도 그대로 작동, 변경 없음).
        여기선 프로필 JSON 파일만 다룬다."""
        result = ProfileEditResult()
        result.request_id = msg.request_id
        user_id = msg.user_id
        if user_id == GUEST_USER_ID:
            result.ok = False
            result.error = '게스트는 편집할 수 없습니다'
            self.profile_edit_result_publisher.publish(result)
            return
        if not (profile_store.PROFILES_DIR / f'{user_id}.json').exists():
            result.ok = False
            result.error = '프로필을 찾을 수 없음'
            self.profile_edit_result_publisher.publish(result)
            return

        if msg.op == 'delete':
            profile_store.delete_profile(user_id)
            self.profiles.pop(user_id, None)
            result.ok = True
            self.profile_edit_result_publisher.publish(result)
            return

        try:
            fields = json.loads(msg.fields_json) if msg.fields_json else {}
        except json.JSONDecodeError:
            result.ok = False
            result.error = '잘못된 요청'
            self.profile_edit_result_publisher.publish(result)
            return
        if 'avatar_face_key' in fields and fields['avatar_face_key'] not in AVATAR_CHOICES:
            result.ok = False
            result.error = '알 수 없는 아바타입니다'
            self.profile_edit_result_publisher.publish(result)
            return
        profile = profile_store.load_profile(user_id)
        if 'name' in fields:
            name = (fields['name'] or '').strip()
            profile['name'] = name or None
        if 'avatar_face_key' in fields:
            profile['avatar_face_key'] = fields['avatar_face_key']
        profile_store.save_profile(profile)
        self.profiles[user_id] = profile
        result.ok = True
        self.profile_edit_result_publisher.publish(result)

    def _registration_may_speak_now(self, user_id: str) -> bool:
        """등록 결과 안내(done/failed/duplicate)를 지금 말해도 안전한지 확인한다.

        2026-08-03 코드리뷰로 발견한 버그: 등록은 음성이 아니라 카메라 포즈로만
        진행돼서 session.record_activity()를 갱신할 방법이 없다 — 5단계 x 최대
        REGISTRATION_STEP_TIMEOUT_SEC(8초)라 최악 40초까지 걸릴 수 있는데
        SESSION_IDLE_TIMEOUT_S는 15초라, 그 사이 세션이 이미 IDLE로 닫힌 뒤에야 이
        결과가 도착할 수 있다. _alarm_may_speak_now와 같은 이유로: IDLE이면
        request_session_start로 다시 열고, 다른 실제 대화가 SPEAKING/THINKING
        중이면(그 사이 새 세션이 열렸을 수 있음) 그 턴을 깨지 않도록 지금은
        말하지 않는다."""
        if self.session.state == SessionState.IDLE:
            return self.session.request_session_start('face_registration', user_id=user_id)
        return self.session.state not in (SessionState.SPEAKING, SessionState.THINKING)

    def _announce_registration_result(self, text: str, emotion: str):
        """등록 결과 안내 — 세션이 이미 닫혔거나 다른 대화가 진행 중이면 안전해질
        때까지 재시도한다(_on_alarm_fire의 재시도 패턴과 동일)."""
        if not self._registration_may_speak_now(self.current_user_id):
            def retry():
                timer.cancel()
                self._announce_registration_result(text, emotion)
            timer = self.create_timer(ALARM_RETRY_DELAY_SEC, retry)
            return
        self._speak(text, emotion, 'none')

    def _photo_countdown_may_speak_now(self, user_id: str) -> bool:
        """사진 촬영 카운트다운을 지금 말해도 안전한지 확인 — 등록결과 안내와 동일한 판단."""
        if self.session.state == SessionState.IDLE:
            return self.session.request_session_start('photo_capture', user_id=user_id)
        return self.session.state not in (SessionState.SPEAKING, SessionState.THINKING)

    def on_photo_countdown_request(self, _msg: Empty):
        """photo_capture_node가 브이 사인을 확정하면 호출됨 — 촬영 준비를 안내한다.
        실제 촬영(사진 저장)은 face_display_node에 뜨는 셔터 버튼을 눌러야 이뤄진다
        (2026-08-13, 자동 카운트다운에서 변경 — 이 노드는 카메라 프레임이 없어 촬영
        자체는 못 함). 세션이 다른 대화로 바쁘면 안전해질 때까지 재시도
        (_on_alarm_fire와 동일 패턴)."""
        if not self._photo_countdown_may_speak_now(self.current_user_id):
            def retry():
                timer.cancel()
                self.on_photo_countdown_request(_msg)
            timer = self.create_timer(ALARM_RETRY_DELAY_SEC, retry)
            return
        self._speak('사진 찍을게요! 화면에 뜬 버튼을 눌러주세요!', 'happy', 'none')

    def _pick_word_quiz_question(self, user_id: str, used: set | None = None) -> dict:
        """difficulty_levels(영어모드와 공용)에 맞는 단어 풀에서 문제 하나를 뽑아 정답+보기를
        만든다. quiz_state에 정답을 저장해두고, 화면엔 섞인 보기만 내려보낸다.

        used에 이번 판에서 이미 낸 단어를 넘기면 그 단어들을 빼고 뽑는다(2026-08-04 실사용
        피드백 — 예전엔 매번 전체 풀에서 무작위로 뽑아서, 난이도별 8개 풀에 5문제면 한 판에
        중복이 날 확률이 80%였다). 풀이 바닥나면(문제 수 > 단어 수) used를 무시하고 전체에서
        뽑는다 — 중복을 피하려다 문제를 못 내는 것보다는 낫다.
        """
        difficulty = self.difficulty_levels.get(user_id, DEFAULT_DIFFICULTY_LEVEL)
        pool = WORD_QUIZ_DATA[difficulty]
        if used:
            remaining = [e for e in pool if e['word'] not in used]
            if remaining:
                pool = remaining
        entry = random.choice(pool)
        options = [entry['word']] + list(entry['distractors'])
        random.shuffle(options)
        return {'word': entry['word'], 'meaning_ko': entry['meaning_ko'], 'options': options}

    def _publish_word_quiz_question(self, question: dict):
        msg = String()
        msg.data = json.dumps({'meaning_ko': question['meaning_ko'], 'options': question['options']},
                               ensure_ascii=False)
        self.word_quiz_question_publisher.publish(msg)

    def _publish_word_quiz_result(self, correct: bool, answer: str):
        """터치 직후 화면에 ○/✗를 즉시 띄운다 — 음성 피드백을 대체한다(2026-08-04).

        예전엔 매 문제마다 _speak()로 "정답이에요!"를 말했는데, 화면은 그 음성을 안 기다리고
        바로 다음 문제로 넘어가서 빠르게 터치하면 음성이 몇 문제씩 밀렸다(실사용에서 퀴즈를
        다 끝낸 뒤에야 앞 문제 피드백이 나오는 것으로 드러남). 음성은 판 끝 총평만 남긴다.
        """
        msg = String()
        msg.data = json.dumps({'result': 'correct' if correct else 'wrong', 'answer': answer},
                               ensure_ascii=False)
        self.word_quiz_question_publisher.publish(msg)

    def _start_word_quiz(self, user_id: str) -> str:
        question = self._pick_word_quiz_question(user_id)
        self.word_quiz_state[user_id] = {
            'word': question['word'], 'question_num': 1, 'score': 0,
            'used': {question['word']},   # 이번 판에 이미 낸 단어(중복 방지)
        }
        self._active_quiz_user_id = user_id
        self._publish_word_quiz_question(question)
        self.get_logger().info(f'영단어 퀴즈 시작 (user_id={user_id}), 1번 문제: {question["word"]}')
        # 실제로 화면에 뜬 문제의 뜻(meaning_ko)을 tool_result에 넣어준다(2026-08-14) —
        # 안 그러면 LLM이 화면에 뭐가 떴는지 전혀 모른 채 그럴듯한 단어를 지어내서
        # 말하는 버그가 있었음(실기기 테스트로 발견: 화면=고양이인데 음성은 다른 단어를
        # 언급). 정답(영단어)까지 말하면 4지선다 취지가 사라지므로 정답은 절대 말하지
        # 말라고 명시적으로 지시한다.
        return (f'영단어 퀴즈를 시작합니다. 총 {WORD_QUIZ_QUESTIONS_PER_ROUND}문제입니다. '
                f'첫 번째 문제의 뜻은 "{question["meaning_ko"]}"입니다 — 이 뜻만 자연스럽게 '
                f'말해주고, 화면에 뜬 보기 중 정답 영단어를 터치하면 된다고 안내하세요. '
                f'정답 영단어 자체는(보기 중 어떤 것도) 절대 미리 말하지 마세요.')

    def on_word_quiz_answer(self, msg: String):
        """face_display_node가 퀴즈 화면에서 터치된 단어를 보내면 정답 여부를 판정하고
        음성으로 피드백한다 — LLM을 거치지 않는 즉각 반응(등록 완료 안내와 같은 패턴).

        터치 이벤트엔 user_id 정보가 전혀 없어서 self.current_user_id를 여기서 다시
        읽으면, 그 사이 얼굴인식이 흔들려(실측으로 확인된 불안정성) 답이 조용히
        무시되는 버그가 있었음(2026-07-28, Kimi 컨설팅으로 진단) — _active_quiz_user_id
        (퀴즈 시작 시점에 고정)를 대신 쓴다."""
        user_id = self._active_quiz_user_id
        if user_id is None:
            return  # 퀴즈 중이 아닌데 온 답(화면 잔여 이벤트 등) — 무시
        state = self.word_quiz_state.get(user_id)
        if state is None:
            return
        if self._quiz_advance_timer is not None:
            return  # 결과 표시 중(다음 문제 대기) — 연타 무시
        # 터치도 명백한 사용자 활동이다. 이걸 안 하면 퀴즈를 터치로만 푸는 동안
        # 무응답 타임아웃(15초)에 걸려 퀴즈 중에 세션이 끊긴다 — 얼굴 등록이 겪었던
        # 것과 같은 문제(_registration_may_speak_now 주석 참고).
        self.session.record_activity()
        correct = msg.data.strip().lower() == state['word'].lower()
        if correct:
            state['score'] += 1
        self._publish_word_quiz_result(correct, state['word'])
        self._schedule_quiz_advance(user_id)

    def _schedule_quiz_advance(self, user_id: str):
        """결과(○/✗)를 잠시 보여준 뒤 다음 문제로 넘어간다 — 마지막 문제였으면 판을 끝낸다.

        rclpy의 create_timer는 반복 타이머라 콜백 안에서 반드시 cancel해야 한다
        (이 프로젝트의 알람 재시도/촬영 카운트다운과 같은 패턴).
        """
        self._cancel_quiz_advance_timer()

        def advance():
            self._cancel_quiz_advance_timer()
            state = self.word_quiz_state.get(user_id)
            if state is None or self._active_quiz_user_id != user_id:
                return  # 대기 중에 판이 끝났거나(그만하기/세션종료) 사용자가 바뀜
            if state['question_num'] >= WORD_QUIZ_QUESTIONS_PER_ROUND:
                self._end_word_quiz(user_id, announce=True)
                return
            state['question_num'] += 1
            question = self._pick_word_quiz_question(user_id, state['used'])
            state['word'] = question['word']
            state['used'].add(question['word'])
            self._publish_word_quiz_question(question)

        self._quiz_advance_timer = self.create_timer(WORD_QUIZ_RESULT_DISPLAY_SEC, advance)

    def _cancel_quiz_advance_timer(self):
        if self._quiz_advance_timer is not None:
            self._quiz_advance_timer.cancel()
            self._quiz_advance_timer = None

    def _end_word_quiz(self, user_id: str | None, announce: bool) -> str:
        """퀴즈를 끝내고 화면을 평소 화면으로 되돌린다. 도구 호출(그만하기)과 마지막 문제
        정답 처리 양쪽에서 호출되므로, 마지막 문제 채점 직후엔 이 함수 안에서 바로
        _speak()로 최종 점수를 말하고(announce=True), 도구 호출 경로는 도구 결과 문자열만
        반환해 다음 LLM 호출이 알아서 마무리 인사를 만들게 한다(announce=False)."""
        state = self.word_quiz_state.pop(user_id, None)
        if self._active_quiz_user_id == user_id:
            self._active_quiz_user_id = None
        # 결과 표시 대기 중이었으면 타이머를 반드시 세운다 — 안 그러면 이미 끝난 판에
        # 다음 문제가 화면에 뜬다("그만할래" 도구 호출/세션 종료와 겹치는 경우).
        self._cancel_quiz_advance_timer()
        hide_msg = String()
        hide_msg.data = 'inactive'
        self.word_quiz_question_publisher.publish(hide_msg)
        if state is None:
            return '지금 진행 중인 퀴즈가 없습니다.'
        result = f'{state["score"]}/{WORD_QUIZ_QUESTIONS_PER_ROUND}문제를 맞혔어요.'
        if announce:
            self._speak(f'퀴즈가 끝났어요! {result}', 'happy', 'none')
        return f'영단어 퀴즈를 종료했습니다. 최종 점수: {result}'

    def _handle_set_alarm(self, user_id: str, args: dict) -> str:
        """set_alarm 도구 호출을 가로채 처리한다. minutes_from_now는 LLM이 현재 시각
        (_build_mode_directive가 매턴 알려줌) 기준으로 계산해서 넘긴 값 — 여기선 그 값을
        그대로 신뢰하고 유효성만 확인한다."""
        minutes = args.get('minutes_from_now')
        message = (args.get('message') or '').strip()
        if not isinstance(minutes, int) or minutes < 1:
            return ('알람 시간을 이해하지 못했습니다. 1분 이상의 정확한 시간으로 다시 '
                    '말씀해달라고 안내해주세요.')
        # 기존 알람이 있으면 새로 덮어씀(취소 후 재설정) — 사용자당 알람은 하나만 유지
        self._cancel_pending_alarm(user_id)
        fire_at = datetime.now() + timedelta(minutes=minutes)
        self._start_alarm_timer(user_id, minutes * 60.0, message)
        self.get_logger().info(
            f'알람 설정 (user_id={user_id}): {minutes}분 후 ({fire_at.strftime("%H:%M")}), '
            f'message={message!r}')
        time_str = fire_at.strftime('%H시 %M분')
        return (f'{time_str}에 알람을 설정했습니다'
                f'{f" ({message})" if message else ""}. 이 시각을 사용자에게 다시 말해주세요.')

    def _handle_cancel_alarm(self, user_id: str) -> str:
        if self._cancel_pending_alarm(user_id):
            return '알람을 취소했습니다.'
        return '지금 설정된 알람이 없습니다.'

    def _cancel_pending_alarm(self, user_id: str) -> bool:
        alarm = self.pending_alarms.pop(user_id, None)
        if alarm is None:
            return False
        # .cancel()만 호출 — _cancel_speech_timeout()과 동일한 기존 관례. Timer.destroy()는
        # Node의 내부 타이머 목록에서 제거하지 않아(Node.destroy_timer만 그걸 함) 직접 부르면
        # 이중 정리 위험만 생기고 실익이 없음(코드 리뷰 중 발견, 2026-07-28).
        alarm['timer'].cancel()
        self._alarm_awaiting_response.pop(user_id, None)
        return True

    def _start_alarm_timer(self, user_id: str, delay_sec: float, message: str):
        def fire():
            timer.cancel()
            self.pending_alarms.pop(user_id, None)
            self._on_alarm_fire(user_id, message)
        timer = self.create_timer(delay_sec, fire)
        self.pending_alarms[user_id] = {'timer': timer, 'message': message}

    def _alarm_may_speak_now(self, user_id: str) -> bool:
        """알람류 안내(최초/재알림 공통)를 지금 말해도 안전한지 확인한다.

        2026-07-30 코드리뷰로 발견한 버그: 예전엔 알람이 세션 상태와 무관하게 무조건
        `_speak()`(SPEAKING으로 강제 전환)를 호출했음 — 이러면 ①아무도 없을 때(IDLE)
        웨이크워드/얼굴인식 트리거 없이 세션이 몰래 열려버리고 ②마침 다른 실제 대화가
        진행 중(SPEAKING/THINKING)이면 `_speak()`가 같이 내보내는 `/llm_response_done`이
        그 진행 중이던 턴의 완료 신호로 잘못 인식돼 tts_node가 아직 재생 중인데도
        세션이 성급하게 LISTENING으로 돌아가는 등 상태가 어긋날 수 있었다. 세션이
        IDLE일 때만, 그것도 웨이크워드/얼굴트리거와 동일한 `request_session_start`
        절차(디바운스 포함)를 거쳐서 안전할 때만 True를 반환한다."""
        return (self.session.state == SessionState.IDLE
                and self.session.request_session_start('alarm', user_id=user_id))

    def _on_alarm_fire(self, user_id: str, message: str):
        if not self._alarm_may_speak_now(user_id):
            def retry():
                timer.cancel()
                self._on_alarm_fire(user_id, message)
            timer = self.create_timer(ALARM_RETRY_DELAY_SEC, retry)
            return
        reminder = f'{message} 알림이에요!' if message else '알람이에요!'
        self._speak(reminder, 'surprise', 'none')
        self._alarm_awaiting_response[user_id] = True
        self._schedule_alarm_escalation(user_id, message, attempt=1)

    def _schedule_alarm_escalation(self, user_id: str, message: str, attempt: int):
        def check():
            timer.cancel()
            self._check_alarm_escalation(user_id, message, attempt)
        timer = self.create_timer(ALARM_ESCALATION_DELAY_SEC, check)

    def _check_alarm_escalation(self, user_id: str, message: str, attempt: int):
        """알람이 울린 뒤 사용자가 반응(발화 아무거나)했는지 확인 — 반응했으면
        on_user_input이 이미 _alarm_awaiting_response를 지웠을 것이므로 여기선 아무것도
        안 함. 무응답이면 ALARM_MAX_ESCALATIONS까지 좀 더 강조해서 반복 안내한다
        (Kimi 컨설팅: 알람은 '주의를 끄는 것'이 목적이라 음성만으론 부족할 수 있어 화면
        표정 변화도 같이 감. 팔 제스처는 이번 스코프에서 제외 — 사용자 판단).

        재알림도 `_alarm_may_speak_now`로 안전한지 먼저 확인한다 — 세션이 이미 IDLE로
        돌아갔거나(원래 대상자가 응답 안 함) 다른 대화가 진행 중이면 이 함수 자체를
        재시도 예약해서 다시 부른다(재시도 시점에 _alarm_awaiting_response도 자연히
        다시 확인되므로, 대기하는 동안 사용자가 다른 경로로 응답해도 낡은 재알림이
        나가지 않는다)."""
        if not self._alarm_awaiting_response.get(user_id):
            return
        if attempt > ALARM_MAX_ESCALATIONS:
            self._alarm_awaiting_response.pop(user_id, None)
            return
        if not self._alarm_may_speak_now(user_id):
            def retry():
                timer.cancel()
                self._check_alarm_escalation(user_id, message, attempt)
            timer = self.create_timer(ALARM_RETRY_DELAY_SEC, retry)
            return
        reminder = f'다시 알려드려요! {message} 알림이에요!' if message else '다시 알려드려요, 알람이에요!'
        self._speak(reminder, 'surprise', 'none')
        self._schedule_alarm_escalation(user_id, message, attempt + 1)

    def _check_session_timeout(self):
        if self.session.check_idle_timeout():
            self.get_logger().info('무응답 타임아웃 — 세션 종료, IDLE로 복귀')
            self._clear_pending_vision('세션이 종료되어 카메라 확인을 취소했습니다.')
            self._finish_onboarding()
            self._end_session()
            self._sync_mic_state()

    def _end_session(self):
        """세션 종료 시 대화 정체성을 놓아주고, 재방문 판정용 정보를 남긴다."""
        if self.session_user_id is not None:
            self._last_session_user_id = self.session_user_id
            self._last_session_end_time = time.monotonic()
        # 진행 중이던 퀴즈도 같이 정리한다 — 안 그러면 세션이 끝난 뒤에도 다음 문제가
        # 화면에 뜨고, 화면은 계속 퀴즈 모드로 남는다(2026-08-04 설계 점검에서 발견).
        if self._active_quiz_user_id is not None:
            self._end_word_quiz(self._active_quiz_user_id, announce=False)
        self.session_user_id = None

    def _start_logout(self, user_id: str):
        """log_out 도구 호출 처리 — 고정 문구를 말하고 트리거 시점 user_id를 기록해둔다.
        실제 세션 종료는 그 발화의 /speech_finished(또는 타임아웃 안전장치)가 와야
        _advance_pending_session_lifecycle을 거쳐 일어난다."""
        self._pending_logout_user_id = user_id
        self._speak('오늘 너무 재밌었어요!', 'happy', 'wave_right_arm')

    def _finish_logout(self, user_id: str):
        """로그아웃 발화가 실제로 끝난 뒤 세션을 닫는다. 재방문 판정 필드는 _end_session이
        채운 직후 바로 지운다 — 로그아웃은 의도적 핸드오프이므로, 같은 사람이 나중에
        다시 와도 "이어서 대화"가 아니라 새로 시작해야 한다(무응답 타임아웃 종료와
        다른 점)."""
        self._pending_logout_user_id = None
        self._end_session()
        self._last_session_user_id = None
        self._last_session_end_time = None
        self.session.force_idle()
        self._sync_mic_state()
        self.user_switch_requested_publisher.publish(Empty())
        self.get_logger().info(f'[세션] 로그아웃 완료 (user_id={user_id})')

    def _start_shutdown(self):
        """shutdown_robot 도구 호출 처리 — 2단계 감정 발화(작별 인사 -> 진짜 종료 인사)
        중 1단계를 시작한다. 실제 종료 신호는 두 발화가 모두 끝난 뒤에야 나간다."""
        self._pending_shutdown_step = 'farewell'
        self._speak('네 오늘도 즐거웠어요!', 'happy', 'none')

    def _advance_pending_session_lifecycle(self):
        """/speech_finished 확인(on_speech_finished) 또는 타임아웃 안전장치
        (_on_speech_timeout) 시점에 호출한다 — 로그아웃 발화 확인이나 종료 2단계 발화가
        진행 중이었으면 다음 단계로 넘어간다. 둘 다 진행 중이 아니면 아무 일도 안 한다."""
        if self._pending_logout_user_id is not None:
            self._finish_logout(self._pending_logout_user_id)
            return
        if self._pending_shutdown_step == 'farewell':
            self._pending_shutdown_step = 'goodbye'
            self._speak('저는 이만 가볼게요. 프로그램을 종료합니다.', 'sad', 'wave_right_arm')
            return
        if self._pending_shutdown_step == 'goodbye':
            self._pending_shutdown_step = None
            self._execute_shutdown_sequence()

    def _execute_shutdown_sequence(self):
        """종료 2단계 발화가 끝난 뒤 실제 종료 신호를 순서대로 발행한다 — 파이 먼저,
        SHUTDOWN_LAPTOP_DELAY_SEC 뒤 노트북(자기 자신 포함). 노트북이 먼저 죽으면
        dialogue_node/tts_synth_node가 사라져 파이 쪽 노드들이 종료 신호를 받을 방법이
        없어지므로 이 순서를 지킨다."""
        self.get_logger().info('[세션] 종료 시퀀스 시작 — /shutdown_pi 발행')
        self.shutdown_pi_publisher.publish(Empty())

        def fire():
            timer.cancel()
            self.get_logger().info('[세션] /shutdown_laptop 발행')
            self.shutdown_laptop_publisher.publish(Empty())
        timer = self.create_timer(SHUTDOWN_LAPTOP_DELAY_SEC, fire)

    def _on_shutdown_signal(self, _msg: Empty):
        """이 노드도 다른 노드들과 동일하게 자신의 종료 신호(/shutdown_laptop)를 구독해
        스스로 종료한다 — main()의 rclpy.spin(node)가 rclpy.ok() 기반이라, 콜백 안에서
        rclpy.shutdown()을 호출하면 spin이 정상적으로 반환되고 finally 정리 코드가
        그대로 실행된다(설계서 4번 참고)."""
        self.get_logger().info('시스템 종료 신호 수신 — 정리 후 종료합니다')
        rclpy.shutdown()

    def _start_session(self, user_id: str):
        """세션 시작(GREETING) — 로봇이 먼저 인사를 건넨 뒤 LISTENING으로 전환한다.

        인사는 세 갈래로 나뉜다(2026-08-04): 게스트는 이름을 묻는 온보딩으로 들어가고
        LLM 인사를 생략한다(_start_guest_onboarding). 기존회원은 프로필에 이름이
        있으면 "{name}님, 안녕하세요!" 고정 문장 뒤에 기존 LLM 개인화 인사를 잇고
        (_speak_named_greeting), 이름이 없으면 억지로 묻지 않고 기존 방식(LLM 인사만)
        그대로 쓴다 — 대화 중 자연스럽게 알게 되는 것을 기다린다."""
        # 대화 정체성을 여기서 캡처해 세션 끝까지 고정한다 — 세션 중 얼굴인식이 흔들려도
        # (guest 발행 등) 대화 기록이 갈아끼워지지 않게 하는 핵심 지점.
        self.session_user_id = user_id

        # 재방문 판정 — 직전 세션과 같은 사람이 창 안에 다시 왔으면 대화를 이어가고,
        # 창을 넘겼으면 단기 맥락을 비우고 새 대화로 시작한다(이름/선호 같은 장기 기억은
        # profile_store가 따로 유지하므로 새로 시작해도 로봇이 사람을 잊지는 않는다).
        is_continuation = (
            user_id == self._last_session_user_id
            and self._last_session_end_time is not None
            and (time.monotonic() - self._last_session_end_time) <= SESSION_REENTRY_WINDOW_SEC)
        if not is_continuation and user_id in self.histories:
            self.get_logger().info(f'재방문 창({SESSION_REENTRY_WINDOW_SEC:.0f}초) 경과 — '
                                    f'새 대화로 시작 (user_id={user_id})')
            self.histories.pop(user_id, None)

        history = self._get_history(user_id)   # 프로필 로드(최초 1회) + 시스템 프롬프트 구성
        self._publish_language_mode(user_id)
        name = (self.profiles.get(user_id) or {}).get('name')
        self.get_logger().info(f'[세션] 대화 상대: user_id={user_id}, 이름={name or "아직 모름"}')

        if user_id == GUEST_USER_ID and not is_continuation:
            self._start_guest_onboarding(user_id, history)
            return
        # is_continuation인 게스트(재방문 창 5분 이내 같은 사람)는 온보딩을 다시 시작하지
        # 않는다(2026-08-14) — 안 그러면 영어모드로 대화하다 idle 타임아웃으로 세션이
        # 끊긴 뒤 웨이크워드로 다시 깨울 때마다 "성함이 어떻게 되세요?"가 매번 다시 나가서,
        # 그 직후 하려던 실제 명령("이제 한국어로 하자" 등)이 이름 추출 로직에 먹혀버리는
        # 문제가 실사용에서 발견됨. 이 경우 아래(이름 없는 회원과 동일 경로)로 흘려보내
        # 짧은 인사만 하고 바로 정상 대화(tool-calling)로 복귀한다 — language_modes/
        # english_submodes는 guest 키로 인메모리에 그대로 남아있어 자동으로 이어진다.

        mode_directive = self._build_mode_directive(user_id)
        if name:
            self._speak_named_greeting(user_id, name, history, mode_directive)
            return

        prompt_messages = history + [
            HumanMessage(content=GREETING_TRIGGER_TEXT),
            SystemMessage(content=mode_directive),
        ]
        try:
            greeting, emotion, action = self._generate_streamed_response(prompt_messages, user_id)
        except Exception as e:
            self.get_logger().error(f'인사 생성 실패, 기본 인사로 대체: {e}')
            greeting, emotion, action = '안녕하세요!', 'happy', 'wave_right_arm'
            self._speak(greeting, emotion, action)
        history.append(AIMessage(content=greeting))
        self.get_logger().info(f'선제 인사 발행 완료 (emotion={emotion}, action={action}) — 재생 완료 대기 중')

    def _speak_named_greeting(self, user_id: str, name: str, history: list,
                               mode_directive: str) -> None:
        """기존회원+이름 있음 인사(2026-08-04) — "{name}님, 안녕하세요!" 고정 문장을
        먼저 말해 로봇이 사용자를 알아보고 있음을 분명히 보여준 뒤, 기존 LLM 개인화
        인사(GREETING_TRIGGER_TEXT, "보고싶었어" 스타일)를 그대로 이어붙인다."""
        prefix = f'{name}님, 안녕하세요!'
        self._begin_speaking_turn()
        self._publish_sentence(prefix)
        prompt_messages = history + [
            HumanMessage(content=GREETING_TRIGGER_TEXT),
            SystemMessage(content=mode_directive),
        ]
        continuation = self._stream_dialogue_response(prompt_messages, user_id)
        classify_text = continuation.strip() or prefix
        emotion, action = self._classify_emotion_action(classify_text, user_id)
        self._publish_emotion_action(emotion, action)
        if action != NO_ARM_ACTION:
            self._send_arm_goal(action)
        if not continuation.strip():
            # _stream_dialogue_response는 스스로 말한 문장이 있을 때만 /llm_response_done을
            # 내므로(이 경우엔 없음), 이미 prefix로 턴을 시작한 이상 여기서 직접 낸다.
            self.llm_response_done_publisher.publish(Empty())
        history.append(AIMessage(content=f'{prefix} {continuation}'.strip()))
        self.get_logger().info(f'이름 인사 발행 완료 (emotion={emotion}, action={action}) — 재생 완료 대기 중')

    def _start_guest_onboarding(self, user_id: str, history: list) -> None:
        """게스트 세션 시작(2026-08-04) — 이름을 묻는 고정 문장만 말하고(LLM 인사
        생략), 온보딩 상태(awaiting_name)로 진입한다. 이어지는 답변은 on_user_input이
        가로채 _handle_onboarding_name_reply로 넘긴다."""
        self._speak(GUEST_GREETING_TEXT, 'happy', 'none')
        history.append(AIMessage(content=GUEST_GREETING_TEXT))
        self._onboarding_state = 'awaiting_name'
        self._onboarding_attempts = 0
        self._publish_name_confirm_prompt('listening')
        self.get_logger().info(f'[온보딩] 시작 (user_id={user_id}) — 이름 대기')

    def _publish_name_confirm_prompt(self, value: str) -> None:
        msg = String()
        msg.data = value
        self.name_confirm_prompt_publisher.publish(msg)

    def _handle_onboarding_name_reply(self, text: str) -> None:
        """게스트 온보딩 중 '성함이 어떻게 되세요?'에 대한 답을 그 자리에서 바로
        처리한다(2026-08-04) — profile_store의 최대 10턴 지연되는 배치추출
        (_maybe_evict_and_extract)과 달리, 이 턴 안에서 바로 이름을 확정해야 한다."""
        try:
            result = self.name_extraction_llm.invoke([
                SystemMessage(content=NAME_EXTRACTION_PROMPT),
                HumanMessage(content=text),
            ])
            name = (result.name or '').strip() or None
            declined = result.declined
            wants_face_registration = result.wants_face_registration
        except Exception as e:
            self.get_logger().warn(f'[온보딩] 이름 추출 실패: {e}')
            name = None
            declined = False
            wants_face_registration = False

        if wants_face_registration:
            # 이름 확인 없이 바로 등록으로 건너뛴다(2026-08-14) — 단, 독립 음성 트리거
            # ("얼굴 등록해줘"를 온보딩과 무관하게 말한 경우, 이름 없이 등록되는 게
            # 의도된 동작)와 달리 이건 온보딩(=사용자 등록) 흐름 안에서 나온 요청이라,
            # 등록이 끝나면 뒤늦게라도 성함을 물어봐야 한다 — 안 그러면 "얼굴은
            # 등록됐는데 이름이 없어 매번 다시 게스트 취급"되는 문제가 생긴다
            # (_registration_needs_name, on_registration_prompt/_ask_name_after_registration 참고).
            self.get_logger().info('[온보딩] 이름 대신 즉시 등록 요청 감지 — 이름 확인 건너뛰고 등록 시작')
            self._onboarding_state = None
            self._onboarding_candidate_name = None
            self._registration_needs_name = True
            self._publish_name_confirm_prompt('hide')
            self._speak('좋아요! 바로 얼굴 등록을 시작할게요, 카메라를 봐주세요!', 'happy', 'none')
            self.start_registration_publisher.publish(Empty())
            return

        if declined:
            if self._registration_name_target_user_id is not None:
                # 등록 후 이름을 물어보는 중이었는데 거절함 — 이미 등록된 얼굴 자체는
                # 그대로 두고(취소할 방법이 없음, 이미 chromadb에 저장됨) 이름만 없이
                # 남긴다. 기존 독립 트리거의 "이름 없이 등록" 결과와 동일한 최종 상태.
                self.get_logger().info('[온보딩] 등록 후 이름 안내 거절 — 이름 없이 등록 상태 유지')
                self._registration_name_target_user_id = None
                self._speak('알겠어요, 이름 없이 등록된 상태로 둘게요!', 'neutral', 'none')
                self._finish_onboarding()
                return
            self.get_logger().info('[온보딩] 음성으로 등록 거절 감지 — 게스트로 계속 대화하기')
            self._decline_registration()
            return

        if name is None:
            self._onboarding_attempts += 1
            self.get_logger().info(f'[온보딩] 이름 인식 실패 (시도 {self._onboarding_attempts}회)')
            if self._onboarding_attempts >= ONBOARDING_MAX_ATTEMPTS_BEFORE_ALT:
                self._speak('죄송해요, 잘 못 알아들었어요. 화면에서 직접 입력하시거나, '
                            '이대로 편하게 대화하셔도 괜찮아요.', 'neutral', 'none')
                self._publish_name_confirm_prompt('listening_retry')
            else:
                self._speak('죄송해요, 성함을 다시 한번 말씀해주시겠어요?', 'neutral', 'none')
            return

        self.get_logger().info(f'[온보딩] 이름 인식: {name!r} — 확인 대기')
        self._enter_onboarding_confirm(name)

    def _decline_registration(self) -> None:
        """사용자가 등록을 거절했을 때(음성/터치 공용) 게스트로 계속 대화한다."""
        self._speak('알겠어요, 등록 없이 게스트로 진행할게요! 네, 우리 편하게 얘기해요!', 'happy', 'none')
        self._finish_onboarding()

    def _enter_onboarding_confirm(self, name: str) -> None:
        self._onboarding_candidate_name = name
        self._onboarding_state = 'awaiting_confirm'
        self._speak(f'{name}님이 맞으신가요?', 'neutral', 'none')
        self._publish_name_confirm_prompt(f'confirm:{name}')

    def on_name_confirm_answer(self, msg: String):
        """face_display_node(로봇 화면 터치/키보드)와 companion_bridge_node(동반 웹앱
        브라우저 폼) 둘 다 이 토픽에 발행한다 — 먼저 도착한 쪽을 채택하고, 이미 상태가
        넘어간 뒤 늦게 도착한 나머지는 아래 상태 가드로 자연히 무시된다(2026-08-04,
        "동시입력 시 먼저 온 것 채택" 설계 결정)."""
        data = msg.data
        if data == 'guest':
            if self._onboarding_state is None:
                return
            self.get_logger().info('[온보딩] 게스트로 계속 대화하기 선택')
            if self.session.state == SessionState.SPEAKING:
                # 인사말(이름 묻기)이 아직 재생 중일 때 버튼이 눌린 경우 — 끝까지 다
                # 말하게 두지 않고 바로 끊는다(on_user_input의 SPEAKING 인터럽트와 동일
                # 패턴, 2026-08-14).
                self.get_logger().info('발화 재생 중 게스트 버튼 눌림 — 인터럽트')
                self._cancel_speech_timeout()
                self.interrupt_publisher.publish(Empty())
                self._cancel_arm_goal_if_active()
            self._decline_registration()
            return
        if data.startswith('submit:'):
            if self._onboarding_state != 'awaiting_name':
                return  # 이미 확인 단계로 넘어갔거나 온보딩 중이 아님 — 늦게 온 중복 무시
            name = data[len('submit:'):].strip()
            if not name:
                return
            self.get_logger().info(f'[온보딩] 직접입력으로 이름 받음: {name!r}')
            self._enter_onboarding_confirm(name)
            return
        if self._onboarding_state != 'awaiting_confirm':
            return  # 'confirm'/'retry'는 확인 화면에서만 의미 있음(늦게 온 중복 등 방어)
        if data == 'confirm':
            self._handle_onboarding_confirmed()
        elif data == 'retry':
            self.get_logger().info('[온보딩] 다시 말할래요 선택')
            self._retry_onboarding_name()

    def _retry_onboarding_name(self) -> None:
        """이름이 틀렸다는 답변(음성/터치 공용) — 이름을 다시 묻는다.

        이름 인식 실패(_handle_onboarding_name_reply의 name is None 분기)와 동일하게
        _onboarding_attempts를 증가시킨다(2026-08-14) — 안 그러면 이름은 매번
        정확히 인식되는데 확인 단계에서만 계속 거부되는 경우 이 카운터가 전혀 늘지
        않아 ONBOARDING_MAX_ATTEMPTS_BEFORE_ALT회를 넘겨도 키보드/대안 문구가 영원히
        안 뜨는 문제가 있었음(실기기 테스트로 발견)."""
        self._onboarding_state = 'awaiting_name'
        self._onboarding_candidate_name = None
        self._onboarding_attempts += 1
        self.get_logger().info(f'[온보딩] 확인 거부로 재시도 (누적 시도 {self._onboarding_attempts}회)')
        if self._onboarding_attempts >= ONBOARDING_MAX_ATTEMPTS_BEFORE_ALT:
            self._speak('죄송해요, 잘 못 알아들었어요. 화면에서 직접 입력하시거나, '
                        '이대로 편하게 대화하셔도 괜찮아요.', 'neutral', 'none')
            self._publish_name_confirm_prompt('listening_retry')
        else:
            self._speak('알겠어요, 다시 한번 말씀해주세요.', 'neutral', 'none')
            self._publish_name_confirm_prompt('listening')

    def _handle_onboarding_confirm_reply(self, text: str) -> bool:
        """이름 확인 단계의 음성/텍스트 답변을 분류해 처리한다(2026-08-05) — 터치
        버튼과 같은 결과로 합류시킨다. 확인/부정으로 판단되면 그 자리에서 처리하고
        True를 반환하고, 확인과 무관한 말이면 아무것도 안 하고 False를 반환한다
        (호출부가 온보딩을 취소하고 그 발화를 일반 대화로 이어가게 함)."""
        try:
            result = self.confirm_reply_llm.invoke([
                SystemMessage(content=CONFIRM_REPLY_PROMPT),
                HumanMessage(content=text),
            ])
            confirmed = result.confirmed
        except Exception as e:
            self.get_logger().warn(f'[온보딩] 확인 답변 분류 실패: {e}')
            confirmed = None

        if confirmed is True:
            self._handle_onboarding_confirmed()
            return True
        if confirmed is False:
            self.get_logger().info('[온보딩] 음성/텍스트로 이름 부정 — 다시 묻기')
            self._retry_onboarding_name()
            return True
        return False

    def _handle_onboarding_confirmed(self) -> None:
        name = self._onboarding_candidate_name
        self._onboarding_state = None
        self._onboarding_candidate_name = None
        self._publish_name_confirm_prompt('hide')
        if self._registration_name_target_user_id is not None:
            # 얼굴 등록은 이미 끝난 뒤 이름만 뒤늦게 물어본 경우(_ask_name_after_registration
            # 참고) — 카메라를 다시 돌릴 필요 없이 이 이름을 바로 그 user_id에 붙인다.
            user_id = self._registration_name_target_user_id
            self._registration_name_target_user_id = None
            self.get_logger().info(f'[온보딩] 등록 후 이름 확인됨: {name!r} (user_id={user_id})')
            self._complete_onboarding_registration(user_id, name)
            return
        self.get_logger().info(f'[온보딩] 확인됨: {name!r} — 얼굴 등록 시작')
        self._onboarding_registration_name = name
        self._speak('좋아요! 이제 얼굴 등록을 시작할게요, 카메라를 봐주세요!', 'happy', 'none')
        self.start_registration_publisher.publish(Empty())

    def _complete_onboarding_registration(self, user_id: str, name: str) -> None:
        """새로 등록된 얼굴(user_id)에 온보딩에서 확인받은 이름을 즉시 저장하고,
        세션을 그 user_id로 승격한다(guest→기존회원 승격과 동일 경로 재사용)."""
        profile = profile_store.load_profile(user_id)
        profile['name'] = name
        profile_store.save_profile(profile)
        self.profiles[user_id] = profile
        self.get_logger().info(f'[온보딩] 등록 완료 — user_id={user_id}에 이름 저장: {name!r}')
        if self.session_user_id == GUEST_USER_ID:
            self._promote_guest_session(user_id)
        self._announce_registration_result(f'등록됐어요, {name}님! 잘 부탁해요.', 'happy')

    def _finish_onboarding(self) -> None:
        """온보딩을 종료하고 평소 대화로 되돌린다(취소/타임아웃/세션종료 공용) —
        호출부가 필요한 안내 문구는 이미 자기가 말했으므로 여기선 상태만 정리한다.

        ⚠️ _onboarding_registration_name은 여기서 건드리지 않는다 — 확인 후 실제
        얼굴 등록이 진행되는 동안(카메라 포즈로만 진행돼 5~40초 걸릴 수 있음,
        _registration_may_speak_now 참고)은 _onboarding_state가 이미 None이라
        _check_session_timeout의 무응답 타임아웃이 이 함수를 호출할 수 있는데,
        그때 이 이름까지 지워버리면 등록이 끝나도(on_registration_prompt) 이름을
        못 붙이게 된다. 이 값의 정리는 on_registration_prompt(done/failed/duplicate)
        가 전담한다."""
        if self._onboarding_state is None:
            return
        self._onboarding_state = None
        self._onboarding_candidate_name = None
        self._onboarding_attempts = 0
        self._publish_name_confirm_prompt('hide')

    def _publish_sentence(self, text: str):
        out = String()
        out.data = text
        self.publisher.publish(out)

    def _publish_emotion_action(self, emotion: str, action: str):
        emotion_msg = String()
        emotion_msg.data = emotion
        self.emotion_publisher.publish(emotion_msg)

        action_msg = String()
        action_msg.data = action
        self.action_publisher.publish(action_msg)

    def _publish(self, response: str, emotion: str, action: str):
        self._publish_sentence(response)
        self._publish_emotion_action(emotion, action)

    def _publish_tool_call_event(self, request_id: int, tool_name: str, args_json: str,
                                  status: str, result_summary: str):
        """학생용 AI 투명성 대시보드(education_bridge_node)용 도구 호출 이벤트 발행.
        카메라 왕복 도구는 request_id로 pending->ok/timeout/error 두 이벤트가 짝지어진다
        (그 외 도구는 request_id=0, status='ok' 한 번뿐)."""
        msg = ToolCallEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.request_id = request_id
        msg.tool_name = tool_name
        msg.args_json = args_json
        msg.status = status
        msg.result_summary = result_summary
        self.tool_call_publisher.publish(msg)

    def _sync_mic_state(self):
        """session 상태 기준으로 원하는 마이크 on/off를 계산해, 값이 바뀔 때만 발행한다
        (매 상태 전이마다 무조건 발행하면 stt_node 로그가 안 바뀐 값으로도 계속 찍힘).

        /session_active도 여기서 같이 발행한다 — 이 함수가 이미 모든 세션 상태 전이
        지점에서 호출되고 있어서, 별도 호출을 여기저기 추가하는 것보다 발행 누락 위험이
        없다(같은 "세션 상태를 다른 노드에 알린다"는 관심사이기도 함)."""
        enabled = self.session.mic_should_be_enabled
        if enabled != self._mic_enabled:
            self._mic_enabled = enabled
            msg = Bool()
            msg.data = enabled
            self.mic_enabled_publisher.publish(msg)

        active = not self.session.is_idle
        if active != self._session_active:
            self._session_active = active
            msg = Bool()
            msg.data = active
            self.session_active_publisher.publish(msg)

    def _cancel_speech_timeout(self):
        if self._speech_timeout_timer is not None:
            self._speech_timeout_timer.cancel()
            self._speech_timeout_timer = None

    def _refresh_speech_timeout(self):
        """진행 중인 발화 안전장치 타이머를 다시 speech_timeout_sec로 늘린다.

        문장 단위 스트리밍(2026-07-30)에서는 문장마다 별도로 TTS API를 호출해 순서대로
        재생하므로, 문장이 여러 개인 응답은 전체 재생이 끝나기까지의 실제 시간이 꽤
        길어질 수 있다. 타이머를 "말하기 시작한 시점" 딱 한 번만 걸어두면, 아직 뒤쪽
        문장이 재생 중인데도 안전장치가 먼저 발동해 마이크를 강제로 다시 켜버리고,
        그 순간 로봇이 아직 스피커로 내보내고 있는 자기 목소리를 마이크가 그대로
        주워들어 새 발화로 착각 -> 응답 -> 또 타임아웃... 하는 "혼자 묻고 답하는" 루프로
        이어진다(2026-07-30, 실기기에서 실제로 겪음). 문장을 발행할 때마다 이 함수로
        갱신해서, 진짜로 멈춰버린 경우에만 안전장치가 발동하게 한다."""
        self._cancel_speech_timeout()
        self._speech_timeout_timer = self.create_timer(
            self.speech_timeout_sec, self._on_speech_timeout)

    def _begin_speaking_turn(self):
        """SPEAKING 전이+마이크 끄기+타임아웃 타이머 시작만 담당(응답 텍스트/emotion/action
        발행과는 분리) — 문장 단위 스트리밍(_stream_dialogue_response)이 첫 문장을 말하기
        시작하는 시점에 이것부터 먼저 호출하고, emotion/action은 전체 텍스트가 다 나온
        뒤에야 알 수 있어 나중에 별도로 발행한다(_generate_streamed_response 참고).
        _speak()(정적 텍스트, 항상 단발성)도 이걸 그대로 재사용한다."""
        if self._mic_grace_timer is not None:
            self._mic_grace_timer.cancel()  # 이전 턴의 유예 타이머가 남아있으면 정리
            self._mic_grace_timer = None
        self.session.to_speaking()
        self._sync_mic_state()
        self._refresh_speech_timeout()

    def _speak(self, response: str, emotion: str, action: str):
        """응답을 한 번에(스트리밍 아님) 발행하고 SPEAKING으로 전이 — 실제 LISTENING
        복귀는 /speech_finished(또는 타임아웃 안전장치)가 맡는다. 인사말 폴백/기억초기화
        확인/오류 안내처럼 미리 정해진 문구를 말할 때 쓴다(LLM이 생성하는 응답은
        _generate_streamed_response를 쓴다). 이 함수 자체가 "이 턴은 메시지 하나로 끝"
        이라는 뜻이므로 /llm_response_done도 함께 낸다 — tts_node가 이 신호를 보고
        /speech_finished를 정확히 한 번만 발행한다."""
        # ⚠️ 2026-07-30: 마이크를 끄기(_begin_speaking_turn) 전에 응답부터 발행하면, TTS가
        # 스피커로 말을 시작하는 시점과 /mic_enabled=False가 실제로 stt_node에 반영되는
        # 시점 사이에 틈이 생겨 로봇이 자기 목소리를 마이크로 다시 들어버릴 수 있다(실기기
        # 라이브 테스트로 확인된 문제 — _stream_dialogue_response()는 이미 이 순서로
        # 고쳐져 있었는데 _speak()엔 소급 적용이 안 돼 있었음, Kimi 코드리뷰로 발견).
        # 마이크부터 끄고 나서 발행하도록 순서를 맞춘다.
        self._begin_speaking_turn()
        self._publish(response, emotion, action)
        self.llm_response_done_publisher.publish(Empty())
        if action != NO_ARM_ACTION:
            self._send_arm_goal(action)

    def _enable_mic_after_grace(self):
        """MIC_REENABLE_GRACE_SEC 경과 후 실제로 마이크를 켠다(위 상수 설명 참고)."""
        timer = self._mic_grace_timer
        self._mic_grace_timer = None
        if timer is not None:
            timer.cancel()
        self._sync_mic_state()
        self.get_logger().info('마이크 재활성화 — LISTENING')

    def on_speech_finished(self, _msg: Empty):
        if self.session.on_speech_finished():
            self._cancel_speech_timeout()
            self._cancel_arm_goal_if_active()
            if self._has_pending_session_lifecycle():
                # 로그아웃/종료 시퀀스가 진행 중이면 마이크를 다시 켜지 않고 바로 다음
                # 단계(다음 발화 또는 실제 종료)로 넘어간다 — 여기서 그냥 마이크 재활성화
                # 유예 타이머를 걸면, 이어지는 다음 발화를 마이크가 잠깐이라도 다시 켜진
                # 채로 자기 목소리로 들어버릴 위험이 있다.
                self.get_logger().info('발화 재생 완료 확인 — 세션 종료 시퀀스 진행')
                self._advance_pending_session_lifecycle()
                return
            self.get_logger().info('발화 재생 완료 확인 — 마이크 재활성화까지 잠시 대기')
            self._mic_grace_timer = self.create_timer(
                MIC_REENABLE_GRACE_SEC, self._enable_mic_after_grace)

    def _on_speech_timeout(self):
        self.get_logger().warn(
            f'{self.speech_timeout_sec}초 안에 /speech_finished가 안 와서 '
            f'안전장치로 강제 LISTENING 복귀')
        self._cancel_speech_timeout()
        self._cancel_arm_goal_if_active()
        # tts_node 재생 큐를 반드시 비운다 — 문장 단위 스트리밍(2026-07-30)에서 응답이
        # 여러 문장이라 전체 재생이 이 타임아웃보다 오래 걸릴 수 있는데, 여기서 큐를 안
        # 비우면 세션은 이미 LISTENING으로 돌아가 새 입력을 받는데 스피커는 이전 턴의
        # 남은 문장을 계속 재생 중인 상태가 된다. 그 상태에서 새 턴이 시작되면(마이크가
        # 이미 켜져있어 SPEAKING 인터럽트 경로를 안 거침) 새 문장들이 이전 턴의 남은
        # 문장들 뒤에 그대로 이어붙어, "로그에 찍힌 답변과 실제로 스피커에서 나오는
        # 말이 다르게" 들리는 사고로 이어진다(실기기에서 실제로 겪음).
        self.interrupt_publisher.publish(Empty())
        self.session.on_speech_finished()
        if self._has_pending_session_lifecycle():
            # TTS 합성/재생이 실패해 /speech_finished가 끝내 안 와도, 이 안전장치
            # 타임아웃 시점에 로그아웃/종료 시퀀스가 그대로 진행되게 한다("종료해줘"라고
            # 했는데 TTS 실패로 아무 일도 안 일어나는 상황 방지, 설계서 5번).
            self._advance_pending_session_lifecycle()
            return
        self._mic_grace_timer = self.create_timer(
            MIC_REENABLE_GRACE_SEC, self._enable_mic_after_grace)

    def _has_pending_session_lifecycle(self) -> bool:
        return (self._pending_logout_user_id is not None
                or self._pending_shutdown_step is not None)

    def _send_arm_goal(self, gesture: str):
        if not self._arm_action_client.server_is_ready():
            self.get_logger().warn(f'arm_node 액션 서버 미연결 — 팔 동작 생략: {gesture}')
            return
        self._is_arm_cancel_pending = False  # 새 goal을 보내므로 이전 취소 요청은 무효화
        goal = MoveArm.Goal()
        goal.gesture = gesture
        self.get_logger().info(f'팔 동작 요청: {gesture}')
        send_future = self._arm_action_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_arm_goal_response)

    def _on_arm_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('팔 동작 요청이 거부됨')
            return
        if self._is_arm_cancel_pending:
            # goal이 accept되기 전(응답 왕복 중)에 인터럽트/종료가 먼저 와서 취소 요청이
            # 대기 중이었던 경우 — _arm_goal_handle이 그때 None이라 취소가 조용히
            # 무시되던 레이스였음(tts_node의 "합성 중 인터럽트" 버그와 동일 패턴).
            # accept되는 즉시 바로 취소해 실제로 반영되게 한다.
            self._is_arm_cancel_pending = False
            goal_handle.cancel_goal_async()
            return
        self._arm_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_arm_result)

    def _on_arm_result(self, future):
        self._arm_goal_handle = None
        result = future.result().result
        if not result.is_success:
            self.get_logger().info(f'팔 동작이 완료되지 않음: {result.message}')

    def _cancel_arm_goal_if_active(self):
        if self._arm_goal_handle is not None:
            self.get_logger().info('팔 동작 취소 요청')
            self._arm_goal_handle.cancel_goal_async()
            self._arm_goal_handle = None
        else:
            # goal을 보냈지만 아직 accept 응답을 못 받은 상태일 수 있음 — 응답이 오면
            # _on_arm_goal_response가 이 플래그를 보고 즉시 취소한다.
            self._is_arm_cancel_pending = True

    def on_user_input(self, msg: String):
        if self._alarm_awaiting_response:
            # self.current_user_id 일치 여부로 판단하지 않는다 — 알람을 설정한 시점과
            # 지금(응답 시점) 사이에 얼굴인식이 흔들리면(실측으로 확인된 불안정성,
            # guest<->userXXXXX 오락가락) current_user_id가 알람을 설정했을 때와 달라져
            # 있을 수 있어서, 그 값으로 조회하면 정작 진짜 알람의 플래그는 안 지워지고
            # 계속 반복 안내되는 버그가 있었음(2026-07-28, 실측+Kimi 컨설팅으로 확인).
            # Kimi 의견: "아무 입력이나 응답으로 간주"는 일반 원칙으론 부작용이 있지만
            # (다른 사람의 말에 알람이 꺼질 수 있음), 보통 한 번에 하나의 알람만 대기 중인
            # 이 로봇 사용 패턴을 감안하면 이 좁은 범위(응답 감지 한정)에선 감수할 만한
            # 절충으로 판단해 전부 지움.
            cleared = list(self._alarm_awaiting_response.keys())
            self._alarm_awaiting_response.clear()
            self.get_logger().info(f'알람 응답 감지(user_id={cleared}) — 반복 안내 중단')

        if self.session.is_idle:
            # 웨이크워드로 세션이 시작되기 전엔 VAD가 뭘 들었든 무시한다 —
            # 이게 오늘 겪은 "배경소음에 VAD가 계속 반응" 문제를 원천 차단하는 핵심 장치.
            # (마이크도 LISTENING 상태에서만 켜지므로 이 상태에선 애초에 stt_node가
            # /user_input을 잘 안 보내지만, terminal_io 등 다른 입력 노드는 이 게이팅과
            # 무관하게 언제든 보낼 수 있어서 여기서도 방어적으로 무시한다)
            self.get_logger().debug(f'IDLE 상태라 입력 무시: {msg.data}')
            return

        if (self._onboarding_state == 'awaiting_name'
                and self._onboarding_attempts < ONBOARDING_MAX_ATTEMPTS_BEFORE_ALT):
            self.session.record_activity()
            self._handle_onboarding_name_reply(msg.data)
            return
        # ⚠️ 2026-08-06 수정: ONBOARDING_MAX_ATTEMPTS_BEFORE_ALT회 실패 후에도
        # _onboarding_state는 'awaiting_name'으로 계속 남아있는다(화면 키보드
        # submit:이 계속 먹히려면 on_name_confirm_answer의 상태 가드가 유지돼야
        # 하므로 — _handle_onboarding_name_reply 참고) — 그런데 예전엔 이 게이팅이
        # 무조건 걸려서, fallback 발화("화면에서 직접 입력하시거나, 이대로 편하게
        # 대화하셔도 괜찮아요")가 거짓말이 됐다: 음성/텍스트로 뭘 입력해도 계속
        # 이름 추출 실패로만 처리돼 대화도 로그아웃/종료 같은 도구 호출도 전혀
        # 안 먹혔다. 위 조건에 시도 횟수 체크를 추가해, 시도 상한을 넘기면 이
        # if문 자체를 건너뛰고 아래 일반 처리로 흘려보낸다 — 화면 키보드 경로는
        # 그대로 살아있고, 음성/텍스트는 정상 대화로 복귀한다(설계 B안).
        if self._onboarding_state == 'awaiting_confirm':
            # 원래는 터치 전용이었으나(음성 오인식 위험 회피), 음성/텍스트로 답해도
            # 온보딩이 취소되고 처음 인사로 되돌아가는 것처럼 보이는 문제가 실사용 중
            # 발견돼(2026-08-05) 음성/텍스트 확인도 받도록 확장함. 확인/부정으로 판단되면
            # 그 자리에서 처리되고, 확인과 무관한 말이면 온보딩을 취소하고 그 발화는
            # 아래 일반 처리로 이어져 새 턴으로 처리된다(게스트로 계속 대화하는 것과 같은 효과).
            self.session.record_activity()
            if self._handle_onboarding_confirm_reply(msg.data):
                return
            self.get_logger().info('[온보딩] 확인과 무관한 입력 — 온보딩 취소하고 일반 대화로')
            self._finish_onboarding()

        if self._face_registration_active:
            # 등록(카메라 포즈 5단계) 진행 중 탈출구 — RESET_TRIGGERS와 동일한 정확
            # 문구+공백무시 매칭(2026-08-14). shutdown_robot/log_out은 이 분기와
            # 무관하게 아래 일반 tool-calling 경로로 계속 통과한다(이미 정상 동작
            # 확인됨). 그 외 입력은 tool_check 이후(아래) 걸러낸다.
            self.session.record_activity()
            normalized_cancel = msg.data.strip().rstrip('.!?~ ').replace(' ', '')
            if normalized_cancel in _REGISTRATION_CANCEL_TRIGGERS_NO_SPACE:
                self.get_logger().info('[등록] 취소 트리거 감지 — 등록 중단')
                self.registration_cancel_publisher.publish(Empty())
                self._face_registration_active = False
                self._speak('알겠어요, 등록을 취소했어요!', 'neutral', 'none')
                return

        had_pending_vision = self._pending_vision is not None
        if had_pending_vision:
            # 카메라 결과를 기다리는 도중 새 입력이 들어옴(마이크는 꺼져 있으므로 주로
            # terminal_io 같은 텍스트 입력). 보류를 정리하고 새 턴을 시작한다.
            self._clear_pending_vision('사용자가 다른 말을 해서 카메라 확인을 취소했습니다.')

        if self.session.state == SessionState.THINKING and not had_pending_vision:
            # 아직 이전 입력에 대한 응답을 생성/스트리밍하는 중인데 또 다른 입력이 옴.
            # (2026-07-30 발견) 마이크는 LISTENING 상태에서만 켜지고 이 시점엔 이미 THINKING
            # 전이+/mic_enabled 발행이 끝난 뒤라 꺼져있어야 하는데, 그 전에 stt_node가 이미
            # 잡아둔 두 번째 VAD 발화(응답 대기 창 동안 방 안의 잡음/제3자 발화/whisper가
            # 그럴듯하게 환각한 문장)가 이 틈에 들어올 수 있다 — 실기기에서 "로봇이 답하는
            # 도중 문맥과 무관한 말을 스스로 이어간다"는 증상으로 관측됨(팀원 발견,
            # CLAUDE.md 2026-07-30 진행상황 참고). whisper 환각 자체는 별도 문제지만,
            # 이 게이팅 하나로 "IDLE이 아니면 뭐든 새 턴으로 승격시키는" 구조적 갭을 막아
            # whisper가 뭘 잘못 들었든 최소한 로봇이 혼자 대화를 이어가진 않게 된다.
            # _pending_vision이 있던 경우는 위에서 이미 정리했으므로(사용자가 의도적으로
            # 새 말을 해서 카메라 대기를 취소한 것) 여기서 걸러내지 않는다.
            self.get_logger().info(f'응답 생성 중이라 입력 무시(잡음/경쟁 가능성): {msg.data}')
            return

        if self.session.state == SessionState.SPEAKING:
            # 아직 이전 응답을 TTS가 재생 중인데 새 입력이 옴(마이크가 꺼져있어 음성으로는
            # 못 오고, 주로 terminal_io 같은 텍스트 입력) — 재생을 끊고 새 응답으로 넘어간다.
            self.get_logger().info('발화 재생 중 새 입력 도착 — 인터럽트')
            self._cancel_speech_timeout()
            self.interrupt_publisher.publish(Empty())
            self._cancel_arm_goal_if_active()

        user_text = msg.data
        self.get_logger().info(f'입력 수신: {user_text}')
        self.session.record_activity()

        normalized_text = user_text.strip().rstrip('.!?~ ').replace(' ', '')
        if normalized_text in _RESET_TRIGGERS_NO_SPACE:
            self.histories[self.current_user_id] = [SystemMessage(content=SYSTEM_PROMPT)]
            self.profiles.pop(self.current_user_id, None)
            if self.current_user_id != GUEST_USER_ID:
                profile_store.delete_profile(self.current_user_id)
            self.get_logger().info(f'대화 기록 및 프로필 초기화됨 (user_id={self.current_user_id})')
            self._speak('네, 대화 기억을 초기화했어요! (얼굴 인식 정보는 별도예요)', 'neutral', 'none')
            return

        self.session.to_thinking()
        self._sync_mic_state()
        history = self._get_history(self.current_user_id)
        history_len_before = len(history)  # 예외 시 이번 턴에 추가한 메시지(도구 호출 포함) 전부 롤백용
        history.append(HumanMessage(content=user_text))
        mode_directive = self._build_mode_directive(self.current_user_id)
        llm_input = history + [SystemMessage(content=mode_directive)]
        try:
            tool_check_start = time.monotonic()
            tool_check = self.tool_llm.invoke(llm_input)
            self.get_logger().info(
                f'[측정] tool_check 호출 소요={(time.monotonic() - tool_check_start) * 1000:.0f}ms')
            if self._face_registration_active and not any(
                    c['name'] in (LOG_OUT_TOOL_NAME, SHUTDOWN_ROBOT_TOOL_NAME)
                    for c in (tool_check.tool_calls or [])):
                # 등록(카메라 포즈 5단계) 진행 중엔 종료/로그아웃 외의 tool-call은
                # (날씨/퀴즈 등, 또는 도구 없는 일반 잡담) 실행하지 않고 짧게 보류
                # 안내만 한다(2026-08-14) — 취소 트리거는 위에서 이미 걸러졌으므로
                # 여기까지 온 건 등록과 무관한 입력이다.
                self.get_logger().info('[등록] 등록 진행 중이라 무관한 입력 보류')
                self._speak('등록부터 마칠게요, 카메라를 계속 봐주세요!', 'neutral', 'none')
                return
            if tool_check.tool_calls:
                history.append(tool_check)
                # 카메라 왕복이 필요한 도구(분류/가르치기)는 한 번에 하나만 보류할 수 있다
                # (_pending_vision이 단일 슬롯이라). (mode, tool_call_id, label) 형태로 기록.
                camera_call: tuple[str, str, str | None] | None = None
                # 로그아웃/종료도 카메라 왕복과 마찬가지로 "for문 종료 후 한 번에 처리하고
                # 이번 턴을 그대로 끝내는" 단락(short-circuit) 패턴을 쓴다 — 다만 외부
                # 이벤트를 기다리는 게 아니라, LLM이 문장을 다시 지어내지 않고 고정 문구를
                # 그대로 말해야 하기 때문(session_control_tool.py 참고).
                session_lifecycle_call: tuple[str, str] | None = None  # (도구 이름, call['id'])
                # 같은 turn에 run_lekiwi_skill이 여러 번 호출돼도(위 LekiwiActionSequence
                # 참고, 흔치 않지만 방어적으로) 원본 발화 전체 시퀀스 분해를 한 번만 한다.
                lekiwi_sequence_handled = False
                for call in tool_check.tool_calls:
                    if call['name'] == LOOK_AT_OBJECT_TOOL_NAME and camera_call is None:
                        # 카메라 왕복이 필요 — 여기서 실행하지 않고 보류한다.
                        # ToolMessage는 결과(또는 타임아웃)가 왔을 때 붙인다.
                        camera_call = ('classify', call['id'], None)
                        continue
                    if call['name'] == TEACH_OBJECT_CATEGORY_TOOL_NAME and camera_call is None:
                        label = (call['args'].get('label') or '').strip()
                        if label:
                            camera_call = ('teach', call['id'], label)
                            continue
                        tool_result = ('가르칠 물건의 이름을 이해하지 못했습니다. 다시 말해달라고 '
                                       '안내해주세요.')
                    elif call['name'] in (LOOK_AT_OBJECT_TOOL_NAME, TEACH_OBJECT_CATEGORY_TOOL_NAME):
                        tool_result = '이미 다른 카메라 요청을 처리 중이라 이 요청은 건너뜁니다.'
                    elif (call['name'] in (LOG_OUT_TOOL_NAME, SHUTDOWN_ROBOT_TOOL_NAME)
                          and session_lifecycle_call is None):
                        session_lifecycle_call = (call['name'], call['id'])
                        continue
                    elif call['name'] in (LOG_OUT_TOOL_NAME, SHUTDOWN_ROBOT_TOOL_NAME):
                        tool_result = '이미 다른 세션 종료 요청을 처리 중이라 이 요청은 건너뜁니다.'
                    elif call['name'] in (START_ENGLISH_COACHING_TOOL_NAME,
                                          START_ENGLISH_IMMERSION_TOOL_NAME):
                        # 카메라 왕복과 달리 외부 이벤트를 기다릴 필요가 없어 그 자리에서
                        # 바로 상태를 바꾼다(턴 보류 불필요) — 상태 변경은 dialogue_node
                        # 인스턴스 소유라 이 도구 자체는 실행 못 하고(english_mode_tool.py
                        # 참고) 여기서 이름으로 가로챈다.
                        submode = ('coach' if call['name'] == START_ENGLISH_COACHING_TOOL_NAME
                                   else 'immersion')
                        self.language_modes[self.current_user_id] = 'en'
                        self.english_submodes[self.current_user_id] = submode
                        self._persist_mode_settings(self.current_user_id)
                        self._publish_language_mode(self.current_user_id)
                        self.get_logger().info(
                            f'영어모드 켜짐 (user_id={self.current_user_id}, submode={submode})')
                        if submode == 'coach':
                            tool_result = ('영어 회화 연습(코칭) 모드가 켜졌습니다. 이제부터 영어로 '
                                           '대화하되 필요하면 한국어로 표현을 교정해주세요.')
                        else:
                            tool_result = ('영어 실습(몰입) 모드가 켜졌습니다. 이제부터 오직 영어로만 '
                                           '답하세요.')
                    elif call['name'] == STOP_ENGLISH_PRACTICE_TOOL_NAME:
                        self.language_modes[self.current_user_id] = 'ko'
                        self._persist_mode_settings(self.current_user_id)
                        self._publish_language_mode(self.current_user_id)
                        self.get_logger().info(f'영어모드 꺼짐 (user_id={self.current_user_id})')
                        tool_result = '영어 학습 모드가 꺼졌습니다. 이제부터 한국어로만 답하세요.'
                    elif call['name'] == START_WORD_QUIZ_TOOL_NAME:
                        tool_result = self._start_word_quiz(self.current_user_id)
                    elif call['name'] == STOP_WORD_QUIZ_TOOL_NAME:
                        # 화면에 떠 있는 퀴즈를 끝내는 것이 목적이라, 이 발화를 한 사람이
                        # 지금 누구로 인식되는지(self.current_user_id, 흔들릴 수 있음)가
                        # 아니라 실제 활성 퀴즈의 고정된 user_id를 대상으로 한다.
                        tool_result = self._end_word_quiz(self._active_quiz_user_id, announce=False)
                    elif call['name'] == SET_ALARM_TOOL_NAME:
                        tool_result = self._handle_set_alarm(self.current_user_id, call['args'])
                    elif call['name'] == CANCEL_ALARM_TOOL_NAME:
                        tool_result = self._handle_cancel_alarm(self.current_user_id)
                    elif call['name'] == START_FACE_REGISTRATION_TOOL_NAME:
                        # 실제 등록(5단계 안내를 따라가는 다중 프레임 캡처)은
                        # face_recognize_node가 이 신호를 받아 시작한다. 등록 성공/실패는
                        # 나중에 /face_registration_prompt로 별도 도착해 on_registration_prompt가
                        # 그때 음성 안내하므로, 여기 tool_result에서 LLM이 미리 "완료"라고
                        # 말하지 않도록 명시적으로 지시한다(예전 정확문구 매칭 시절 겪었던
                        # "거짓 완료 안내 후 진짜 결과 중복 안내" 버그 재발 방지).
                        self.start_registration_publisher.publish(Empty())
                        self.get_logger().info(f'얼굴 등록 시작 신호 발행 (user_id={self.current_user_id})')
                        tool_result = ('얼굴 등록을 시작했습니다. 카메라를 보면서 화면 안내를 따라달라고만 '
                                       '안내하세요. 등록 완료/실패는 아직 모르니 지금 완료됐다고 말하지 마세요.')
                    elif call['name'] == RUN_LEKIWI_SKILL_TOOL_NAME:
                        if lekiwi_sequence_handled:
                            # 같은 turn에 또 호출됨(흔치 않음) — 이미 원본 발화 전체를
                            # 시퀀스로 분해해 처리했으므로 중복 처리하지 않는다.
                            tool_result = '이미 LeKiwi 동작을 전달했습니다.'
                        else:
                            lekiwi_sequence_handled = True
                            # call['args']를 직접 안 쓴다 — 일반 tool-calling은 "뒤로 갔다가
                            # 회전해줘"처럼 여러 동작이 담긴 발화에서도 도구를 한 번만
                            # 호출하고 나머지는 말로만 약속해버리는 문제가 실기기 테스트로
                            # 확인됐다(도구 설명에 "여러 번 호출하라"고 명시해도 개선 안 됨).
                            # 원본 발화(user_text)를 전용 구조화출력(LekiwiActionSequence)으로
                            # 다시 분해해 신뢰성을 높인다(NameExtraction과 같은 이유).
                            try:
                                sequence = self.lekiwi_sequence_llm.invoke([
                                    SystemMessage(content=LEKIWI_SEQUENCE_PROMPT),
                                    HumanMessage(content=user_text),
                                ])
                                actions = sequence.actions
                            except Exception as e:
                                # 분해 호출 자체가 실패한 경우(API 오류 등)에만 일반
                                # tool-calling의 단일 추측값으로 폴백한다. 호출이 성공했는데
                                # actions가 빈 리스트인 건 "이 발화에 구체적인 LeKiwi 동작이
                                # 없다"는 정당한 결과이므로(2026-08-14 실기기에서 "안했잖아"
                                # 같은 불만 발화에 일반 tool-calling이 backward를 잘못
                                # 추측했을 때, 예전엔 여기서 그 추측을 그대로 실행해버리는
                                # 버그가 있었음) 빈 리스트를 그대로 존중하고 폴백하지 않는다.
                                self.get_logger().warn(
                                    f'[LeKiwi] 동작 시퀀스 분해 호출 실패: {e} — 단일 동작으로 폴백')
                                actions = [LekiwiAction(
                                    skill=call['args'].get('skill', ''),
                                    duration_sec=call['args'].get('duration_sec'))]

                            if not actions:
                                tool_result = (
                                    '이 발화에서 구체적인 LeKiwi 동작(forward/backward/left/'
                                    'right/stop)을 찾지 못해 아무 명령도 보내지 '
                                    '않았습니다. 실행했다고 말하지 말고, 사용자에게 어떤 동작을 '
                                    '원하는지 되물어보세요.')
                                self.get_logger().info('[LeKiwi] 시퀀스에서 실행 가능한 동작을 찾지 못함')
                            else:
                                descriptions = []
                                for action in actions:
                                    skill = action.skill
                                    entry = SKILL_MAP.get(skill)
                                    # 실제로 쓰일(클램프까지 끝난) 지속시간을 여기서 미리
                                    # 계산해 함께 보낸다 — lekiwi_command_node가 다시
                                    # 계산하므로 최종 진실은 그쪽이지만, 같은 순수 로직
                                    # (resolve_duration_sec)을 쓰므로 결과는 항상 일치한다.
                                    resolved_duration = (
                                        resolve_duration_sec(skill, action.duration_sec)
                                        if entry else None)
                                    payload = (f'{skill}:{resolved_duration}'
                                               if resolved_duration is not None else skill)
                                    # 순서대로 발행 — lekiwi_command_node가 첫 번째는 즉시
                                    # 시작하고 나머지는 자기 쪽 대기열에 넣어 이어서 실행한다
                                    # (같은 구독 콜백이 메시지를 순서대로 처리하므로 안전,
                                    # lekiwi_command_node.py의 _move_queue 참고).
                                    self.lekiwi_command_publisher.publish(String(data=payload))
                                    self.get_logger().info(
                                        f'LeKiwi 스킬 트리거: {skill} '
                                        f'(요청 지속시간={action.duration_sec}, 실제 반영={resolved_duration})')
                                    if (entry and skill in MOVE_SKILLS
                                            and resolved_duration is not None):
                                        descriptions.append(f'{skill}({resolved_duration:.0f}초)')
                                    else:
                                        descriptions.append(skill)
                                # 실제로 전달된 동작 목록만 tool_result에 담아 최종 응답이
                                # 이 목록 밖의 동작을 지어내 약속하지 않도록 명시적으로
                                # 막는다(2026-08-14, "말로만 약속하고 실제로는 하나만
                                # 실행" 버그 재발 방지). 시제도 명시한다 — 이동은 몇 초씩
                                # 걸리는데 응답은 거의 즉시 생성되므로, 이미 다 끝난
                                # 것처럼 과거형으로 말하면 사용자가 "안 움직였는데?"라고
                                # 느끼는 실제 관측된 버그가 있었음(실기기 재현).
                                tool_result = (
                                    f'LeKiwi에 다음 동작을 순서대로 지금 막 전달했습니다 — '
                                    f'아직 실행 중이거나 막 시작하는 중이지 완료된 게 아닙니다: '
                                    f'{" → ".join(descriptions)}. 이 목록에 있는 동작만 실행되니 '
                                    f'이 목록에 없는 동작은 절대 언급하거나 약속하지 마세요. '
                                    f'응답할 때 "이동했어요"/"회전했어요"처럼 이미 끝난 과거형으로 '
                                    f'말하지 말고, "이동할게요"/"이동 중이에요"처럼 지금 '
                                    f'시작하거나 진행 중이라는 시제로 말하세요.')
                    elif call['name'] == PICKUP_MEDICINE_TOOL_NAME:
                        # 실제 왕복(이동->픽->복귀)은 lekiwi01의 abo_nav_bridge.py가
                        # 처리한다 -- 여기서는 SSH 트리거만 보낸다(run_lekiwi_skill과
                        # 동일 채널/노드, lekiwi_control.py의 SKILL_MAP
                        # pick_red/pick_blue/pick_green 참고). color는 Literal
                        # 타입으로 선언돼 있지만 방어적으로 한 번 더 검증한다.
                        color = call['args'].get('color')
                        if color not in ('red', 'blue', 'green'):
                            self.get_logger().warn(f'[PickupMedicine] 알 수 없는 color: {color!r}')
                            tool_result = ('지원하지 않는 색상입니다. 빨강/파랑/초록 중에서만 '
                                           '약통을 가져올 수 있다고 안내하고, 어떤 색인지 되물으세요.')
                        else:
                            self.lekiwi_command_publisher.publish(String(data=f'pick_{color}'))
                            self.get_logger().info(f'약통 픽업 트리거: pick_{color}')
                            # LeKiwiClient 연결 오버헤드(약 9초, lekiwi-pill-pickup
                            # 설계 문서 참고)까지 감안하면 도착/픽업까지 시간이
                            # 걸린다 -- 아직 결과를 모르니 완료됐다고 말하지
                            # 않도록 명시한다(start_face_registration과 동일 이유).
                            tool_result = (
                                f'{color} 약통을 가져오라는 명령을 방금 전달했습니다. 로봇이 '
                                f'이동해서 찾아 집어오기까지 시간이 걸리고 아직 결과는 모르니, '
                                f'지금 가지러 간다고만 안내하고 가져왔다고 말하지 마세요.')
                    else:
                        tool_fn = self.tools_by_name.get(call['name'])
                        tool_result = tool_fn.invoke(call['args']) if tool_fn else f'알 수 없는 도구: {call["name"]}'
                    history.append(ToolMessage(content=str(tool_result), tool_call_id=call['id']))
                    self.get_logger().info(f'도구 호출: {call["name"]} -> {tool_result}')
                    self._publish_tool_call_event(
                        0, call['name'], json.dumps(call.get('args', {}), ensure_ascii=False),
                        'ok', str(tool_result))
                if session_lifecycle_call is not None:
                    name, call_id = session_lifecycle_call
                    # 카메라 도구와 달리 결과를 기다리지 않고 그 자리에서 바로 처리하지만,
                    # ToolMessage는 반드시 필요하다 — 안 붙이면 이 tool_call만 응답 없이
                    # history에 남아, farewell 발화 도중 다른 입력이 끼어들 때 다음 턴이
                    # 400으로 실패한다(2026-08-07 실기기 테스트로 재현/확인).
                    history.append(ToolMessage(content=name, tool_call_id=call_id))
                    self._publish_tool_call_event(0, name, '{}', 'ok', name)
                    if name == LOG_OUT_TOOL_NAME:
                        self._start_logout(self.current_user_id)
                    else:
                        self._start_shutdown()
                    return   # 턴 종료 — 고정 문구는 각 헬퍼가 _speak()로 직접 말한다
                if camera_call is not None:
                    mode, call_id, label = camera_call
                    if mode == 'classify':
                        request_id = self._begin_vision_request(self.current_user_id, call_id)
                        self._publish_tool_call_event(
                            request_id, LOOK_AT_OBJECT_TOOL_NAME, '{}', 'pending', '')
                    else:
                        request_id = self._begin_teach_request(self.current_user_id, call_id, label)
                        self._publish_tool_call_event(
                            request_id, TEACH_OBJECT_CATEGORY_TOOL_NAME,
                            json.dumps({'label': label}, ensure_ascii=False), 'pending', '')
                    return   # 턴 보류 — 결과 도착 시 _resume_vision_turn이 이어서 처리
                # 위 도구 호출이 이번 턴에 모드를 바꿨을 수 있으므로 다시 계산한다
                # (턴 시작 시점의 mode_directive를 그대로 쓰면 방금 켠 영어모드가 최종
                # 응답에 반영 안 되는 버그가 생김).
                mode_directive = self._build_mode_directive(self.current_user_id)
                llm_input = history + [SystemMessage(content=mode_directive)]
            result_text, result_emotion, result_action = self._generate_streamed_response(
                llm_input, self.current_user_id)
        except Exception as e:
            self.get_logger().error(f'LLM 응답 생성 실패: {e}')
            del history[history_len_before:]
            self._speak('죄송해요, 지금 응답을 만들지 못했어요. 다시 한 번 말씀해 주시겠어요?', 'neutral', 'none')
            return
        history.append(AIMessage(content=result_text))
        self._maybe_evict_and_extract(self.current_user_id)

        # [대화] 한 줄로 "무엇을 사용자 발화로 인식했는지"와 "그걸로 무슨 감정/행동을
        # 뽑았는지"를 같이 남긴다 — 로봇이 실제 사용자 발화가 아니라 자기 목소리
        # 에코나 whisper 환각을 주워듣고 있는지(예: "구독, 좋아요" 류) 로그만 보고도
        # 바로 구분할 수 있게(2026-07-30, 실기기 디버깅 중 요청받아 추가).
        self.get_logger().info(
            f'[대화] 인식된 발화: "{user_text}" -> 응답: "{result_text}" '
            f'(emotion={result_emotion}, action={result_action}) — 재생 완료 대기 중')

    def _maybe_evict_and_extract(self, user_id: str) -> None:
        """슬라이딩 윈도우(MAX_TURNS_KEPT)를 넘는 가장 오래된 턴을 히스토리에서 제거하기
        직전에, 그 턴에서 핵심 정보(이름/선호/최근사건)만 가벼운 LLM 호출로 뽑아내
        프로필 파일에 누적 저장한다. guest는 저장 대상이 아님(위 _get_history 참고)."""
        if user_id == GUEST_USER_ID:
            return
        history = self.histories[user_id]
        cutoff = profile_store.find_eviction_cutoff(history)
        if cutoff is None:
            return

        evicted = history[1:cutoff]
        del history[1:cutoff]
        evicted_text = '\n'.join(
            f'{"사용자" if isinstance(m, HumanMessage) else "로봇"}: {m.content}'
            for m in evicted if isinstance(m, (HumanMessage, AIMessage)) and m.content)
        try:
            facts = self.extraction_llm.invoke([
                SystemMessage(content=profile_store.EXTRACTION_PROMPT),
                HumanMessage(content=evicted_text),
            ])
        except Exception as e:
            self.get_logger().warn(f'프로필 정보 추출 실패, 이번 턴은 건너뜀: {e}')
            return

        profile = self.profiles.get(user_id) or profile_store.load_profile(user_id)
        had_name = bool(profile.get('name'))
        profile = profile_store.merge_facts(profile, facts)
        self.profiles[user_id] = profile
        profile_store.save_profile(profile)
        if profile.get('name') and not had_name:
            # 대화 중 자연스럽게 알게 된 순간을 콕 집어 로그(2026-08-04, 이름을 강제로
            # 묻지 않는 "기존회원+이름 없음" 경로에서 언제/어떻게 이름을 알게 됐는지
            # 눈으로 확인할 수 있도록) — 아래 통짜 dict 로그와 별개.
            self.get_logger().info(f'[프로필] 이름 최초 인식: {profile["name"]!r} (user_id={user_id})')
        self.get_logger().info(f'[프로필] user_id={user_id} 갱신: {profile}')

    def _start_pending_vision(self, user_id: str, tool_call_id: str, mode: str,
                               label: str | None = None) -> int:
        """카메라 왕복이 필요한 요청(분류/가르치기 공용) 상태를 등록한다. 실제 요청 발행은
        호출부가 이어서 한다 — 두 모드가 발행하는 토픽이 다르기 때문."""
        self._vision_request_seq += 1
        request_id = self._vision_request_seq
        self._pending_vision = {
            'request_id': request_id,
            'tool_call_id': tool_call_id,
            # 대기 중 카메라 앞 사람이 바뀔 수 있으므로, 재개할 히스토리를 요청 시점
            # user_id로 고정해둔다(다른 사람 기록에 답이 섞이지 않게).
            'user_id': user_id,
            'mode': mode,      # 'classify' 또는 'teach'
            'label': label,    # teach일 때만 사용
        }
        self._cancel_vision_timeout()
        self._vision_timeout_timer = self.create_timer(
            VISION_TIMEOUT_SEC, self._on_vision_timeout)
        return request_id

    def _begin_vision_request(self, user_id: str, tool_call_id: str) -> int:
        """카메라 스냅샷을 요청하고 턴을 보류한다(분류). 상태는 THINKING 유지(마이크 off)."""
        request_id = self._start_pending_vision(user_id, tool_call_id, mode='classify')
        msg = UInt32()
        msg.data = request_id
        self.snapshot_request_publisher.publish(msg)
        self.get_logger().info(f'카메라 스냅샷 요청 발행(id={request_id}) — 결과 대기 중')
        return request_id

    def _begin_teach_request(self, user_id: str, tool_call_id: str, label: str) -> int:
        """카메라 스냅샷을 요청하고 턴을 보류한다(가르치기) — 같은 request_id로
        /object_teach_request(라벨 포함)와 /object_snapshot_request를 함께 보내
        face_id_node의 카메라 왕복을 분류와 동일하게 재사용한다."""
        request_id = self._start_pending_vision(user_id, tool_call_id, mode='teach', label=label)
        teach_msg = ObjectTeachRequest()
        teach_msg.request_id = request_id
        teach_msg.label = label
        # 자기 자신의 타임아웃(VISION_TIMEOUT_SEC)과 정확히 같은 시각으로 맞춘다 — "내가
        # 포기하는 시점 = 상대에게도 이 요청의 유효기간이 끝나는 시점"이라는 계약.
        teach_msg.deadline = time.monotonic() + VISION_TIMEOUT_SEC
        self.object_teach_request_publisher.publish(teach_msg)
        snapshot_msg = UInt32()
        snapshot_msg.data = request_id
        self.snapshot_request_publisher.publish(snapshot_msg)
        self.get_logger().info(
            f'카메라 스냅샷 요청 발행(가르치기, id={request_id}, label={label}) — 결과 대기 중')
        return request_id

    def _cancel_vision_timeout(self):
        if self._vision_timeout_timer is not None:
            self._vision_timeout_timer.cancel()
            self._vision_timeout_timer = None

    def _clear_pending_vision(self, reason: str):
        """보류 중인 카메라 요청을 응답 없이 정리한다(세션 종료/새 입력 등).

        보류된 tool_call을 빈손으로 닫지 않으면 다음 LLM 호출이 통째로 400 실패하므로,
        반드시 대응하는 ToolMessage를 붙인 뒤에 버린다(Kimi 지적 반영)."""
        pending = self._pending_vision
        if pending is None:
            return
        self._pending_vision = None
        self._cancel_vision_timeout()
        if pending['mode'] == 'teach':
            # 세션종료/새입력처럼 능동적으로 포기하는 경우 — deadline(수동적 만료)보다
            # 먼저 object_classify_node에 즉시 알려서 조용한 저장을 막는다.
            cancel_msg = UInt32()
            cancel_msg.data = pending['request_id']
            self.object_teach_cancel_publisher.publish(cancel_msg)
        self._get_history(pending['user_id']).append(
            ToolMessage(content=reason, tool_call_id=pending['tool_call_id']))
        self.get_logger().info(f'대기 중이던 카메라 요청 정리: {reason}')

    def on_object_classification(self, msg: ObjectClassification):
        pending = self._pending_vision
        if pending is None:
            self.get_logger().debug('대기 중인 요청이 없어 분류 결과 무시')
            return
        if msg.request_id != pending['request_id']:
            # 타임아웃 뒤 뒤늦게 도착한 이전 요청의 결과 — 지금 질문에 엉뚱한 사진으로
            # 답하는 걸 막기 위해 버린다.
            self.get_logger().info(f'요청 id 불일치 — 늦게 도착한 결과 무시(id={msg.request_id})')
            return
        if msg.is_uncertain:
            tool_result = ('카메라에 보이는 물체를 확실하게 알아보지 못했습니다. '
                           '무엇인지 단정하지 말고, 잘 모르겠다고 솔직히 말한 뒤 '
                           '더 가까이 보여달라고 요청하세요.')
        elif msg.is_taught:
            # 사용자가 가르쳐준 카테고리 매칭 결과 — label이 이미 한국어라 번역하지 않는다
            # (2026-07-29, scripts/object_teach_experiment/ 실측 검증 후 도입).
            tool_result = (f'카메라에 보이는 물체는 사용자가 가르쳐준 "{msg.label}"입니다 '
                           f'(확신도 {msg.score:.0%}). 이 이름을 그대로 사용해 대답하세요(번역하지 마세요).')
            self._publish_object_label(msg.label)
        else:
            tool_result = (f'카메라에 보이는 물체: {msg.label} (확신도 {msg.score:.0%}). '
                           f'이 영어 라벨을 자연스러운 한국어로 바꿔서 대답하세요.')
            # 화면에는 분류 결과(영어)를 그대로 보여준다 — LLM 응답 문장을 파싱하지 않고
            # object_classify_node가 이미 뽑아낸 라벨을 바로 씀(2026-07-27, 사용자 요청으로
            # 한국어 번역 대신 영어 표시로 변경 — 영어모드와 어울리는 영단어 학습 용도).
            self._publish_object_label(msg.label)
        self.get_logger().info(
            f'분류 결과 수신: label={msg.label} uncertain={msg.is_uncertain} taught={msg.is_taught}')
        self._resume_vision_turn(tool_result, status='ok')

    def on_object_teach_result(self, msg: ObjectTeachResult):
        pending = self._pending_vision
        if pending is None or pending.get('mode') != 'teach':
            self.get_logger().debug('대기 중인 가르치기 요청이 없어 결과 무시')
            return
        if msg.request_id != pending['request_id']:
            # 타임아웃 뒤 뒤늦게 도착한 이전 요청의 결과 — 지금 턴과 무관하므로 버린다.
            self.get_logger().info(f'요청 id 불일치 — 늦게 도착한 가르치기 결과 무시(id={msg.request_id})')
            return
        if msg.success:
            tool_result = f'"{pending["label"]}"(으)로 잘 배웠다고 사용자에게 말해주세요.'
        else:
            tool_result = f'가르치기에 실패했습니다({msg.message}). 다시 시도해달라고 안내해주세요.'
        self.get_logger().info(f'가르치기 결과 수신: success={msg.success}')
        self._resume_vision_turn(tool_result, status=('ok' if msg.success else 'error'))

    def _publish_object_label(self, label: str):
        # "a water bottle" -> "water bottle" 처럼 관사만 제거하고 그대로 표시
        cleaned = re.sub(r'^(a|an)\s+', '', label, flags=re.IGNORECASE)
        msg = String()
        msg.data = cleaned
        self.object_label_publisher.publish(msg)
        self.get_logger().info(f'물체 인식 단어 화면 표시: {cleaned}')

    def _on_vision_timeout(self):
        self.get_logger().warn(f'{VISION_TIMEOUT_SEC}초 안에 분류 결과가 오지 않음 — 턴 재개')
        self._resume_vision_turn(
            '카메라에서 사진을 받지 못했습니다. 지금은 볼 수 없다고 사용자에게 알려주세요.',
            status='timeout')

    def _resume_vision_turn(self, tool_result: str, status: str):
        """보류했던 턴을 도구 결과로 이어서 마무리한다(성공/타임아웃/오류 공통 경로).
        status는 education_bridge_node용 ToolCallEvent 발행에만 쓰인다('ok'|'timeout'|'error')."""
        pending = self._pending_vision
        if pending is None:
            return
        self._pending_vision = None
        self._cancel_vision_timeout()
        tool_name = (LOOK_AT_OBJECT_TOOL_NAME if pending['mode'] == 'classify'
                     else TEACH_OBJECT_CATEGORY_TOOL_NAME)
        args_json = ('{}' if pending['mode'] == 'classify'
                     else json.dumps({'label': pending.get('label')}, ensure_ascii=False))
        self._publish_tool_call_event(
            pending['request_id'], tool_name, args_json, status, tool_result)
        history = self._get_history(pending['user_id'])
        # ToolMessage를 LLM 호출보다 먼저 붙인다 — 아래 invoke가 예외로 실패해도
        # 히스토리엔 짝이 맞는 상태로 남아야 다음 턴이 400으로 죽지 않는다.
        history.append(ToolMessage(content=tool_result, tool_call_id=pending['tool_call_id']))
        mode_directive = self._build_mode_directive(pending['user_id'])
        llm_input = history + [SystemMessage(content=mode_directive)]
        try:
            result_text, result_emotion, result_action = self._generate_streamed_response(
                llm_input, pending['user_id'])
        except Exception as e:
            self.get_logger().error(f'LLM 응답 생성 실패(vision): {e}')
            self._speak('죄송해요, 지금 응답을 만들지 못했어요.', 'neutral', 'none')
            return
        history.append(AIMessage(content=result_text))
        self._maybe_evict_and_extract(pending['user_id'])
        self.get_logger().info(
            f'응답 발행 완료(vision, emotion={result_emotion}, action={result_action})')


def main(args=None):
    rclpy.init(args=args)
    node = DialogueNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
