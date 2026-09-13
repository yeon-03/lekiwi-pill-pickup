#!/usr/bin/env python3
"""pick_adapter.py 의 --script 인자 번역만 검증한다 (ROS 없이 실행 가능).

    cd nav/mission && python3 -m unittest test_pick_adapter_args.py -v

lerobot 이 설치돼 있으면 번역된 인자를 lekiwi_yolo_pick.py 의 실제 설정 파서에
넣어 이름이 맞는지까지 확인한다.
"""
import os
import sys
import tempfile
import types
import unittest
from pathlib import Path

# rclpy 없이 import 할 수 있게 최소한만 채운다.
for name in ("rclpy", "rclpy.callback_groups", "rclpy.executors", "rclpy.node", "std_msgs", "std_msgs.msg"):
    sys.modules.setdefault(name, types.ModuleType(name))
sys.modules["rclpy.callback_groups"].MutuallyExclusiveCallbackGroup = object
sys.modules["rclpy.executors"].MultiThreadedExecutor = object
sys.modules["rclpy.node"].Node = object
sys.modules["std_msgs.msg"].Bool = sys.modules["std_msgs.msg"].String = object

import pick_adapter as pa  # noqa: E402

REPO = Path(__file__).resolve().parents[2]


class TestScriptArgs(unittest.TestCase):
    def test_script_paths_exist(self):
        for rel in pa.SCRIPTS.values():
            self.assertTrue((REPO / rel).is_file(), rel)

    def test_color_args_pick_cycle(self):
        self.assertEqual(pa.color_args("pick_cycle", "red", "/tmp/r.json"),
                         ["--color", "red", "--result-file", "/tmp/r.json"])

    def test_color_args_yolo_pick(self):
        self.assertEqual(pa.color_args("yolo_pick", "blue", "/tmp/r.json"),
                         ["--target_color=blue", "--result_file=/tmp/r.json"])

    def test_setup_args_pick_cycle(self):
        self.assertEqual(pa.setup_args("pick_cycle", "m.pt", "/p", "1.2.3.4"),
                         ["--model", "m.pt", "--poses-dir", "/p", "--remote-ip", "1.2.3.4"])

    def test_setup_args_empty(self):
        self.assertEqual(pa.setup_args("pick_cycle", "", "", ""), [])
        self.assertEqual(pa.setup_args("yolo_pick", "", "", ""), [])

    def test_setup_args_yolo_pick_only_existing_pose_files(self):
        with tempfile.TemporaryDirectory() as d:
            for n in ("pre_pick", "grasp", "grasp_closed"):  # grasp_ref 없음
                Path(d, n + ".json").write_text("{}")
            got = pa.setup_args("yolo_pick", "m.pt", d, "1.2.3.4")
        self.assertEqual(got, [
            "--yolo.path=m.pt",
            f"--pick.pose_file={os.path.join(d, 'pre_pick.json')}",
            f"--grasp.grasp_pose_file={os.path.join(d, 'grasp.json')}",
            f"--grasp.close_pose_file={os.path.join(d, 'grasp_closed.json')}",
            "--robot.remote_ip=1.2.3.4",
        ])


class TestYoloPickParsesArgs(unittest.TestCase):
    def test_translated_args_accepted_by_draccus(self):
        sys.path.insert(0, str(REPO / "yolo_and_pick"))
        try:
            import draccus
            import lekiwi_yolo_pick as yp
        except ImportError as e:
            self.skipTest(f"lerobot/draccus 없음: {e}")
        poses = REPO / "yolo_and_pick" / "poses"
        args = (pa.color_args("yolo_pick", "green", "/tmp/r.json")
                + pa.setup_args("yolo_pick", "w.pt", str(poses), "10.0.0.5"))
        cfg = draccus.parse(yp.LeKiwiPickConfig, args=args)
        self.assertEqual(cfg.target_color, "green")
        self.assertEqual(cfg.result_file, "/tmp/r.json")
        self.assertEqual(cfg.yolo.path, "w.pt")
        self.assertEqual(cfg.robot.remote_ip, "10.0.0.5")
        self.assertEqual(cfg.pick.pose_file, str(poses / "pre_pick.json"))
        self.assertEqual(cfg.grasp.close_pose_file, str(poses / "grasp_closed.json"))
        self.assertEqual(cfg.grasp.ref_file, str(poses / "grasp_ref.json"))


if __name__ == "__main__":
    unittest.main()
