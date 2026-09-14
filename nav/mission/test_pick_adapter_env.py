#!/usr/bin/env python3
"""pick_adapter.child_env -- 집기 스크립트에 ROS 경로가 새지 않는지 (로봇·ROS 실행 없이)."""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pick_adapter import child_env  # noqa: E402

SEP = os.pathsep


class TestChildEnv(unittest.TestCase):
    def test_removes_opt_ros_entries_keeps_others(self):
        env = child_env({
            "PYTHONPATH": SEP.join(["/opt/ros/jazzy/lib/python3.12/site-packages", "/home/u/mylib"]),
            "LD_LIBRARY_PATH": SEP.join(["/opt/ros/jazzy/lib", "/home/u/.local/nvidia/cudnn/lib"]),
        })
        self.assertEqual(env["PYTHONPATH"], "/home/u/mylib")
        self.assertEqual(env["LD_LIBRARY_PATH"], "/home/u/.local/nvidia/cudnn/lib")

    def test_removes_colcon_workspace_entries(self):
        env = child_env({
            "AMENT_PREFIX_PATH": SEP.join(["/home/u/abo_ws/install/ros_dialogue", "/opt/ros/jazzy"]),
            "PYTHONPATH": SEP.join(["/home/u/abo_ws/install/ros_dialogue/lib/python3.12/site-packages",
                                    "/home/u/abo_ws_other/lib"]),
        })
        self.assertEqual(env["PYTHONPATH"], "/home/u/abo_ws_other/lib")

    def test_deletes_variable_when_only_ros_entries(self):
        env = child_env({"PYTHONPATH": "/opt/ros/jazzy/lib/python3.12/site-packages",
                         "LD_LIBRARY_PATH": "/opt/ros/jazzy/lib" + SEP})
        self.assertNotIn("PYTHONPATH", env)
        self.assertNotIn("LD_LIBRARY_PATH", env)

    def test_other_variables_untouched_and_input_not_modified(self):
        src = {"ROS_DOMAIN_ID": "42", "PATH": "/opt/ros/jazzy/bin:/usr/bin",
               "PYTHONPATH": "/opt/ros/jazzy/lib/python3.12/site-packages"}
        env = child_env(src)
        self.assertEqual(env["ROS_DOMAIN_ID"], "42")
        self.assertEqual(env["PATH"], "/opt/ros/jazzy/bin:/usr/bin")
        self.assertIn("PYTHONPATH", src)

    def test_defaults_to_os_environ(self):
        self.assertIsInstance(child_env(), dict)


if __name__ == "__main__":
    unittest.main()
