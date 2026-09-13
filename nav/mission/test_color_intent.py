#!/usr/bin/env python3
"""color_intent.extract_color 순수 로직 테스트. ROS2 불필요, 아무 환경에서나:

  python3 -m unittest nav/mission/test_color_intent.py -v
"""
import unittest

from color_intent import extract_color, extract_color_token


class TestExtractColorHappyPath(unittest.TestCase):
    def test_red_variants(self):
        for text in ("빨간색 약 좀 갖다줘", "빨강 약통 집어줘", "저 레드 약", "RED 약 주세요"):
            self.assertEqual(extract_color(text), "red", text)

    def test_blue_variants(self):
        for text in ("파란색 약 좀 갖다줘", "파랑 약통 집어줘", "블루 약 주세요", "blue medicine please"):
            self.assertEqual(extract_color(text), "blue", text)

    def test_green_variants(self):
        for text in ("초록색 약 좀 갖다줘", "초록 약통 집어줘", "그린 약 주세요", "녹색 약 갖다줘"):
            self.assertEqual(extract_color(text), "green", text)


class TestExtractColorNoMatch(unittest.TestCase):
    def test_no_color_mentioned(self):
        self.assertIsNone(extract_color("약 좀 찾아서 갖다줘"))
        self.assertIsNone(extract_color("르키위 오른쪽으로 돌려줘"))

    def test_unsupported_color(self):
        self.assertIsNone(extract_color("노란색 약 좀 갖다줘"))
        self.assertIsNone(extract_color("하얀색 약통 집어줘"))

    def test_empty_and_whitespace(self):
        self.assertIsNone(extract_color(""))
        self.assertIsNone(extract_color("   "))
        self.assertIsNone(extract_color("\n\t"))

    def test_ambiguous_multiple_colors(self):
        # 두 색 이상 언급되면 잘못 집어오는 것보다 안전하게 None(못 알아들음)
        self.assertIsNone(extract_color("빨간거 말고 파란거 갖다줘"))
        self.assertIsNone(extract_color("빨간색이나 초록색 아무거나"))


class TestExtractColorBadInput(unittest.TestCase):
    """문자열이 아닌 입력이 들어와도 예외 없이 None 을 돌려줘야 한다."""

    def test_none_input(self):
        self.assertIsNone(extract_color(None))

    def test_non_string_types(self):
        for bad in (123, 3.14, b"red", ["red"], {"color": "red"}, True):
            self.assertIsNone(extract_color(bad), repr(bad))


class TestExtractColorKnownLimitation(unittest.TestCase):
    """토큰 단위 부분일치라 색상 단어가 다른 한 단어 안에 섞여 있으면
    오인식한다. 버그가 아니라 알려진 한계로 문서화한다."""

    def test_substring_false_positive_is_expected(self):
        self.assertEqual(extract_color("블루투스 스피커 꺼줘"), "blue")


class TestExtractColorToken(unittest.TestCase):
    """abo_nav_bridge.py의 "fetch <목적지> color:<색> [to <복귀지>]" 파싱에 쓰는 헬퍼."""

    def test_no_color_token(self):
        remaining, color = extract_color_token(["center"])
        self.assertEqual(remaining, ["center"])
        self.assertIsNone(color)

    def test_color_token_at_end(self):
        remaining, color = extract_color_token(["center", "color:red"])
        self.assertEqual(remaining, ["center"])
        self.assertEqual(color, "red")

    def test_color_token_in_middle_position_independent(self):
        remaining, color = extract_color_token(["center", "color:blue", "to", "home"])
        self.assertEqual(remaining, ["center", "to", "home"])
        self.assertEqual(color, "blue")

    def test_case_insensitive_value(self):
        _, color = extract_color_token(["color:RED"])
        self.assertEqual(color, "red")

    def test_unsupported_color_value_passed_through_uppercase_stripped(self):
        # 검증(COLORS 소속 여부)은 호출부 책임 -- 여기서는 값만 뽑아준다.
        _, color = extract_color_token(["color:yellow"])
        self.assertEqual(color, "yellow")

    def test_empty_list(self):
        remaining, color = extract_color_token([])
        self.assertEqual(remaining, [])
        self.assertIsNone(color)

    def test_malformed_color_token_no_value(self):
        # "color:" 뒤에 값이 없으면 빈 문자열 -- 예외를 던지지 않는다.
        remaining, color = extract_color_token(["color:"])
        self.assertEqual(remaining, [])
        self.assertEqual(color, "")


if __name__ == "__main__":
    unittest.main()
