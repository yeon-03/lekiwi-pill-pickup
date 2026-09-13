#!/usr/bin/env python3
"""pick_cycle.py 의 순수 로직(YOLO/lerobot 없이 여기서 바로 돌아가는 부분)을 검증한다.

  python3 scripts/test_pick_cycle.py -v      (또는 python3 -m unittest ...)

색상 필터(color_filter)는 motion/services/pickplace 가 이미 내부적으로 쓰는
purple_mask() 를 그대로 재사용하는데, 그건 순수 OpenCV/numpy 라 실제
로봇·YOLO 모델 없이도 검증 가능하다. 실제 색 판정 정확도(조명·카메라 특성)는
여기서 못 잡는다 -- 그건 실물로만 확인 가능.
"""
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "motion"))

import pick_cycle  # noqa: E402
from services.pickplace.grasp_check import GraspCheckArgs  # noqa: E402
from services.pickplace.yolo_detect import Detection  # noqa: E402


def solid_patch(hue: int, size: int = 40) -> np.ndarray:
    """단일 hue(HSV, 채도/명도 최대)로 채운 BGR 이미지 패치를 만든다."""
    hsv = np.zeros((size, size, 3), dtype=np.uint8)
    hsv[:, :, 0] = hue
    hsv[:, :, 1] = 220
    hsv[:, :, 2] = 220
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


def frame_with_patch(hue: int, frame_size=(200, 200), box=(20, 20, 60, 60)) -> np.ndarray:
    """회색 배경(hue 필터에 안 걸림) 위에 지정한 hue 사각형 하나를 그린 프레임."""
    frame = np.full((frame_size[1], frame_size[0], 3), 128, dtype=np.uint8)
    x1, y1, x2, y2 = box
    frame[y1:y2, x1:x2] = solid_patch(hue, size=x2 - x1)
    return frame


def make_det(box=(20, 20, 60, 60)) -> Detection:
    return Detection(name="bottle", conf=0.9, xyxy=box, cls=0)


class TestColorFilter(unittest.TestCase):
    def test_keeps_detection_matching_target_hue(self):
        # red 기본 범위(170~180) 안의 hue=175 패치 -- red 필터를 통과해야 한다.
        frame = frame_with_patch(hue=175)
        cfg = GraspCheckArgs(hue_min=170, hue_max=180)
        kept = pick_cycle.color_filter([make_det()], frame, cfg, min_ratio=0.5)
        self.assertEqual(len(kept), 1)

    def test_rejects_detection_of_different_hue(self):
        # blue 범위(hue~110) 패치를 red 필터(170~180)에 통과시키면 걸러져야 한다.
        frame = frame_with_patch(hue=110)
        cfg = GraspCheckArgs(hue_min=170, hue_max=180)
        kept = pick_cycle.color_filter([make_det()], frame, cfg, min_ratio=0.5)
        self.assertEqual(kept, [])

    def test_empty_detection_list_returns_empty(self):
        frame = frame_with_patch(hue=175)
        cfg = GraspCheckArgs(hue_min=170, hue_max=180)
        self.assertEqual(pick_cycle.color_filter([], frame, cfg, min_ratio=0.5), [])

    def test_box_outside_frame_does_not_crash(self):
        # YOLO 박스 좌표가 프레임 경계를 벗어나는 경우(드물지만 실측 가능) 예외 없이
        # 그냥 걸러져야 한다(범위를 clip 하므로).
        frame = frame_with_patch(hue=175)
        cfg = GraspCheckArgs(hue_min=170, hue_max=180)
        oob = make_det(box=(-50, -50, 1000, 1000))  # 프레임보다 훨씬 큼(클리핑됨)
        degenerate = make_det(box=(500, 500, 600, 600))  # 프레임 완전히 밖
        kept = pick_cycle.color_filter([oob, degenerate], frame, cfg, min_ratio=0.5)
        # oob 는 클리핑 후 프레임 전체(회색 배경 대부분)라 통과 못 함, degenerate 는
        # 아예 빈 ROI라 건너뜀 -- 어느 쪽도 예외를 던지지 않는 게 핵심.
        self.assertEqual(kept, [])


class TestParseArgs(unittest.TestCase):
    def test_default_hue_range_per_color(self):
        for color, (lo, hi) in pick_cycle.DEFAULT_HUE_RANGES.items():
            a = pick_cycle.parse_args(
                ["--color", color, "--result-file", "/tmp/r.json", "--model", "m.pt", "--poses-dir", "p"])
            self.assertEqual((a.hue_min, a.hue_max), (lo, hi))

    def test_hue_override(self):
        a = pick_cycle.parse_args(
            ["--color", "red", "--result-file", "/tmp/r.json", "--model", "m.pt", "--poses-dir", "p",
             "--hue-min", "0", "--hue-max", "10"])
        self.assertEqual((a.hue_min, a.hue_max), (0, 10))

    def test_rejects_unsupported_color(self):
        with self.assertRaises(SystemExit):
            pick_cycle.parse_args(
                ["--color", "yellow", "--result-file", "/tmp/r.json", "--model", "m.pt", "--poses-dir", "p"])


class TestWriteResult(unittest.TestCase):
    def test_writes_valid_json(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "r.json")
            pick_cycle.write_result(path, skill="pill_pickup", color="red",
                                     success=True, grasped=True,
                                     elapsed_sec=1.2, verdict_reason="집기 성공")
            data = json.loads(Path(path).read_text(encoding="utf-8"))
            self.assertEqual(data["color"], "red")
            self.assertTrue(data["success"])


class TestLoadPoses(unittest.TestCase):
    def test_missing_pose_files_are_skipped_not_error(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(pick_cycle.load_poses(d), {})

    def test_reads_existing_pose_files(self):
        from services.pickplace.poses import save_pose

        with tempfile.TemporaryDirectory() as d:
            save_pose(Path(d) / "pre_pick.json", "pre_pick", "lekiwi01",
                      {"arm_shoulder_pan.pos": 1.5}, backup=False)
            poses = pick_cycle.load_poses(d)
            self.assertIn("pre_pick", poses)
            self.assertEqual(poses["pre_pick"]["arm_shoulder_pan.pos"], 1.5)
            self.assertNotIn("grasp", poses)  # 안 만든 파일은 없어야 함


if __name__ == "__main__":
    unittest.main()
