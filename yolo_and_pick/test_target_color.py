#!/usr/bin/env python3
"""lekiwi_yolo_pick.py에 추가한 색상 지정 픽업 로직(color_filter/write_result/
LeKiwiPickConfig.validate의 target_color 처리)을 검증한다. 로봇/YOLO 모델 없이
여기서 바로 돈다(lerobot import는 필요 -- 이 저장소의 나머지 파일이 그렇듯).

  python3 test_target_color.py -v
"""
import json
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

import lekiwi_yolo_pick as p
from lekiwi_yolo_view import Detection


def solid_patch(hue: int, size: int = 40) -> np.ndarray:
    hsv = np.zeros((size, size, 3), dtype=np.uint8)
    hsv[:, :, 0] = hue
    hsv[:, :, 1] = 220
    hsv[:, :, 2] = 220
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


def frame_with_patch(hue: int, frame_size=(200, 200), box=(20, 20, 60, 60)) -> np.ndarray:
    frame = np.full((frame_size[1], frame_size[0], 3), 128, dtype=np.uint8)
    x1, y1, x2, y2 = box
    frame[y1:y2, x1:x2] = solid_patch(hue, size=x2 - x1)
    return frame


def make_det(box=(20, 20, 60, 60)) -> Detection:
    return Detection(name="bottle", conf=0.9, xyxy=box, cls=0)


class TestColorFilter(unittest.TestCase):
    def test_keeps_matching_hue(self):
        frame = frame_with_patch(hue=175)
        kept = p.color_filter([make_det()], frame, hue_min=170, hue_max=180, min_ratio=0.5)
        self.assertEqual(len(kept), 1)

    def test_rejects_other_hue(self):
        frame = frame_with_patch(hue=110)  # blue
        kept = p.color_filter([make_det()], frame, hue_min=170, hue_max=180, min_ratio=0.5)
        self.assertEqual(kept, [])

    def test_empty_input(self):
        frame = frame_with_patch(hue=175)
        self.assertEqual(p.color_filter([], frame, 170, 180, min_ratio=0.5), [])

    def test_out_of_bounds_box_does_not_crash(self):
        frame = frame_with_patch(hue=175)
        oob = make_det(box=(-50, -50, 1000, 1000))
        degenerate = make_det(box=(500, 500, 600, 600))
        kept = p.color_filter([oob, degenerate], frame, 170, 180, min_ratio=0.5)
        self.assertEqual(kept, [])


class TestConfigValidateTargetColor(unittest.TestCase):
    def _cfg(self, **overrides):
        cfg = p.LeKiwiPickConfig()
        cfg.pick.enabled = False  # pose 파일 존재 검사를 건너뛴다(이 테스트와 무관)
        for k, v in overrides.items():
            setattr(cfg, k, v)
        return cfg

    def test_unsupported_color_rejected(self):
        with self.assertRaises(SystemExit):
            self._cfg(target_color="yellow").validate()

    def test_default_hue_applied_per_color(self):
        for color, (lo, hi) in p.TARGET_COLOR_HUE_RANGES.items():
            cfg = self._cfg(target_color=color)
            cfg.validate()
            self.assertEqual((cfg.target_hue_min, cfg.target_hue_max), (lo, hi))

    def test_explicit_hue_not_overridden_by_default(self):
        cfg = self._cfg(target_color="red", target_hue_min=0, target_hue_max=10)
        cfg.validate()
        self.assertEqual((cfg.target_hue_min, cfg.target_hue_max), (0, 10))

    def test_empty_target_color_is_noop(self):
        cfg = self._cfg()  # target_color="" 기본값
        cfg.validate()
        self.assertIsNone(cfg.target_hue_min)


class TestWriteResult(unittest.TestCase):
    def test_writes_expected_schema(self):
        with tempfile.TemporaryDirectory() as d:
            path = str(Path(d) / "r.json")
            p.write_result(path, color="red", success=True, grasped=True,
                           elapsed_sec=3.456, verdict_reason="집기 성공")
            data = json.loads(Path(path).read_text(encoding="utf-8"))
            self.assertEqual(data["skill"], "pill_pickup")
            self.assertEqual(data["color"], "red")
            self.assertTrue(data["success"])
            self.assertEqual(data["elapsed_sec"], 3.5)


if __name__ == "__main__":
    unittest.main()
