#!/usr/bin/env python3
"""pick_cycle.py 의 순수 로직(YOLO/lerobot 없이 여기서 바로 돌아가는 부분)을 검증한다.

  python3 scripts/test_pick_cycle.py -v      (또는 python3 -m unittest ...)

색상 필터는 순수 OpenCV/numpy 라 실제 로봇·YOLO 모델 없이도 검증 가능하다.
실제 색 판정 정확도(조명·카메라 특성)는 여기서 못 잡는다 -- 실물로만 확인 가능.
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

REQ = ["--result-file", "/tmp/r.json", "--model", "m.pt", "--poses-dir", "p"]


def solid_patch(hue: int, size: int = 40) -> np.ndarray:
    """단일 hue(HSV, 채도/명도 높음)로 채운 BGR 이미지 패치."""
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


def spec_for(color: str) -> "pick_cycle.ColorSpec":
    return pick_cycle.ColorSpec(pick_cycle.DEFAULT_HUE_RANGES[color])


class TestColorFilter(unittest.TestCase):
    def test_red_high_side_hue_is_kept(self):
        kept = pick_cycle.color_filter([make_det()], frame_with_patch(175), spec_for("red"), 0.5)
        self.assertEqual(len(kept), 1)

    def test_red_low_side_hue_is_kept(self):
        # 빨강은 0 근처에도 있다. 예전 기본값(170~180)은 이걸 놓쳤다.
        kept = pick_cycle.color_filter([make_det()], frame_with_patch(5), spec_for("red"), 0.5)
        self.assertEqual(len(kept), 1)

    def test_green_is_not_red(self):
        kept = pick_cycle.color_filter([make_det()], frame_with_patch(60), spec_for("red"), 0.5)
        self.assertEqual(kept, [])

    def test_red_is_not_green(self):
        kept = pick_cycle.color_filter([make_det()], frame_with_patch(175), spec_for("green"), 0.5)
        self.assertEqual(kept, [])

    def test_blue_rejected_by_red(self):
        kept = pick_cycle.color_filter([make_det()], frame_with_patch(110), spec_for("red"), 0.5)
        self.assertEqual(kept, [])

    def test_picks_only_matching_bottle_among_two(self):
        # 같은 자리에 빨강·초록 약통이 나란히 있는 실제 배치.
        frame = np.full((200, 200, 3), 128, dtype=np.uint8)
        frame[20:60, 20:60] = solid_patch(175)     # 빨강
        frame[20:60, 120:160] = solid_patch(60)    # 초록
        red, green = make_det((20, 20, 60, 60)), make_det((120, 20, 160, 60))
        self.assertEqual(pick_cycle.color_filter([red, green], frame, spec_for("green"), 0.5), [green])
        self.assertEqual(pick_cycle.color_filter([red, green], frame, spec_for("red"), 0.5), [red])

    def test_float_box_coordinates_do_not_crash(self):
        kept = pick_cycle.color_filter([make_det((20.4, 19.6, 60.2, 59.9))],
                                       frame_with_patch(175), spec_for("red"), 0.5)
        self.assertEqual(len(kept), 1)

    def test_empty_detection_list_returns_empty(self):
        self.assertEqual(pick_cycle.color_filter([], frame_with_patch(175), spec_for("red"), 0.5), [])

    def test_box_outside_frame_does_not_crash(self):
        frame = frame_with_patch(175)
        oob = make_det(box=(-50, -50, 1000, 1000))     # 클리핑되면 대부분 회색
        degenerate = make_det(box=(500, 500, 600, 600))  # 완전히 밖
        self.assertEqual(pick_cycle.color_filter([oob, degenerate], frame, spec_for("red"), 0.5), [])


class TestParseArgs(unittest.TestCase):
    def test_default_hue_ranges_per_color(self):
        for color, ranges in pick_cycle.DEFAULT_HUE_RANGES.items():
            a = pick_cycle.parse_args(["--color", color] + REQ)
            self.assertEqual(a.hue_ranges, ranges)

    def test_red_has_two_ranges(self):
        self.assertEqual(len(pick_cycle.DEFAULT_HUE_RANGES["red"]), 2)

    def test_hue_override_replaces_with_single_range(self):
        a = pick_cycle.parse_args(["--color", "red"] + REQ + ["--hue-min", "0", "--hue-max", "10"])
        self.assertEqual(a.hue_ranges, ((0, 10),))

    def test_hue_override_needs_both_bounds(self):
        with self.assertRaises(SystemExit):
            pick_cycle.parse_args(["--color", "red"] + REQ + ["--hue-min", "0"])

    def test_rejects_unsupported_color(self):
        with self.assertRaises(SystemExit):
            pick_cycle.parse_args(["--color", "yellow"] + REQ)

    def test_required_paths_can_come_from_env(self):
        old = {k: os.environ.get(k) for k in ("PICK_MODEL", "PICK_POSES_DIR", "LEKIWI_HOST_IP")}
        os.environ.update(PICK_MODEL="env.pt", PICK_POSES_DIR="envposes", LEKIWI_HOST_IP="10.0.0.9")
        try:
            a = pick_cycle.parse_args(["--color", "green", "--result-file", "/tmp/r.json"])
            self.assertEqual((a.model, a.poses_dir, a.remote_ip), ("env.pt", "envposes", "10.0.0.9"))
        finally:
            for k, v in old.items():
                if v is None:
                    os.environ.pop(k, None)
                else:
                    os.environ[k] = v


class TestBuildConfigKeepsGripperHue(unittest.TestCase):
    def test_check_hue_is_not_overwritten_by_bottle_color(self):
        # 회귀 시험: 예전 코드는 cfg.check.hue 를 약통 색으로 덮어써 보라색 그리퍼
        # 판정을 망가뜨렸다.
        default = GraspCheckArgs()
        for color in pick_cycle.COLORS:
            cfg = pick_cycle.build_config(pick_cycle.parse_args(["--color", color] + REQ))
            self.assertEqual((cfg.check.hue_min, cfg.check.hue_max),
                             (default.hue_min, default.hue_max), color)


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
            self.assertNotIn("grasp", poses)


if __name__ == "__main__":
    unittest.main()
