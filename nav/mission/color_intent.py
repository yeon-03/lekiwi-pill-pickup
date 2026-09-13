#!/usr/bin/env python3
"""발화(음성 인식 텍스트)에서 약통 색상을 인식한다.

지원 색상은 빨강/파랑/초록 세 가지뿐이다(약통이 이 세 색으로만 있다는 전제).
형태소 분석기 없이, 이 저장소의 다른 문자열 처리(abo_nav_bridge.py)와 같은
수준의 단순한 부분 문자열 매칭을 쓴다 -- 대회 데모 스코프에 맞춘 실용적 선택.

알려진 한계: "블루투스"처럼 색상 단어가 다른 한 단어 안에 우연히 섞여 있으면
오인식할 수 있다(토큰 단위 부분일치라 완전한 단어 경계 검사는 아니다).
"""
import re
from typing import Optional, Tuple

COLORS = ("red", "blue", "green")

COLOR_KR = {"red": "빨간색", "blue": "파란색", "green": "초록색"}

_KEYWORDS = {
    "red": ("빨강", "빨간", "레드", "red"),
    "blue": ("파랑", "파란", "블루", "blue"),
    "green": ("초록", "그린", "녹색", "green"),
}

_TOKEN_RE = re.compile(r"[가-힣a-zA-Z]+")


def extract_color(text) -> Optional[str]:
    """text 에서 지원 색상("red"/"blue"/"green") 하나를 돌려준다.

    다음은 모두 None(못 알아들음)으로 처리한다 -- 잘못 집어오는 것보다
    "못 알아들었다"고 되묻는 편이 안전하다:
      - 문자열이 아닌 입력(None, 숫자, bytes 등)
      - 빈 문자열/공백만 있는 문자열
      - 색상 언급이 전혀 없음
      - 두 가지 이상의 색상이 함께 언급됨(애매함)
    """
    if not isinstance(text, str):
        return None
    tokens = _TOKEN_RE.findall(text.lower())
    if not tokens:
        return None
    found = {
        color
        for token in tokens
        for color, kws in _KEYWORDS.items()
        if any(kw in token for kw in kws)
    }
    return found.pop() if len(found) == 1 else None


def extract_color_token(tokens: list) -> Tuple[list, Optional[str]]:
    """"color:<값>" 형식의 토큰을 목록에서 찾아 제거한다(abo_nav_bridge.py의
    "fetch <목적지> color:<색> [to <복귀지>]" 명령 파싱에 씀 -- 위치 무관).

    돌려주는 값은 (그 토큰을 뺀 나머지 목록, 소문자로 된 색상값 또는 없으면 None).
    값이 COLORS 에 속하는지는 검증하지 않는다 -- 호출부가 판단(지원하지 않는
    색이면 사용자에게 되물어야 하므로).
    """
    color = None
    remaining = []
    for t in tokens:
        if t.lower().startswith("color:"):
            color = t.split(":", 1)[1].strip().lower()
        else:
            remaining.append(t)
    return remaining, color
