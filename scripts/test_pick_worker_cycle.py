#!/usr/bin/env python3
"""pick_worker_cycle.py 를 로봇·YOLO 없이 검증한다.

  python3 -m pytest scripts/test_pick_worker_cycle.py

가짜 워커로 끝나는 경로(성공/포기/오류/시간초과/외부 종료/루프 사망)를 전부 돌리고,
진짜 PickPlaceHeadlessWorker 도 가짜 로봇(motion 테스트의 FakeRobot)으로 한 번
돌려 정지·연결 해제·바퀴 정지까지 확인한다.
"""
import json
import sys
import tempfile
import threading
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "motion"))

import pick_cycle  # noqa: E402
import pick_worker_cycle as pwc  # noqa: E402
from services.pickplace.arm_sequencer import PickArgs  # noqa: E402
from services.pickplace.config import PickPlaceConfig  # noqa: E402
from services.pickplace.headless_worker import PickPlaceHeadlessWorker  # noqa: E402
from services.pickplace.test_headless_worker import FakeRobot, _fake_infer, _fake_load_model  # noqa: E402
from services.pickplace.yolo_detect import Detection  # noqa: E402

REQ = ["--color", "red", "--result-file", "/tmp/r.json", "--model", "m.pt", "--poses-dir", "p"]


class TestResolveTargetClass(unittest.TestCase):
    def test_picks_class_with_color_word(self):
        names = {0: "red_pill_bottle", 1: "green_pill_bottle"}
        self.assertEqual(pwc.resolve_target_class(names, "red"), "red_pill_bottle")
        self.assertEqual(pwc.resolve_target_class(names, "green"), "green_pill_bottle")

    def test_korean_class_names(self):
        self.assertEqual(pwc.resolve_target_class({0: "빨간약통", 1: "초록약통"}, "red"), "빨간약통")

    def test_color_blind_model_falls_back_to_hsv(self):
        self.assertIsNone(pwc.resolve_target_class({0: "cube"}, "red"))
        self.assertIsNone(pwc.resolve_target_class({0: "bottle"}, "green"))

    def test_missing_color_returns_none(self):
        self.assertIsNone(pwc.resolve_target_class({0: "green_pill"}, "red"))

    def test_ambiguous_returns_none(self):
        self.assertIsNone(pwc.resolve_target_class({0: "red_pill", 1: "red_cap"}, "red"))

    def test_explicit_class_must_exist(self):
        names = {0: "a", 1: "b"}
        self.assertEqual(pwc.resolve_target_class(names, "red", explicit="b"), "b")
        with self.assertRaises(ValueError):
            pwc.resolve_target_class(names, "red", explicit="zzz")

    def test_accepts_name_list(self):
        self.assertEqual(pwc.resolve_target_class(["blue_bottle"], "blue"), "blue_bottle")


def patch_frame():
    """왼쪽 40px 는 빨강(H=175), 오른쪽 40px 는 초록(H=60)."""
    import cv2
    hsv = np.zeros((40, 80, 3), dtype=np.uint8)
    hsv[:, :40] = (175, 220, 220)
    hsv[:, 40:] = (60, 220, 220)
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


class TestMakeInfer(unittest.TestCase):
    red_box = Detection(name="bottle", conf=0.9, xyxy=(0, 0, 40, 40), cls=0)
    green_box = Detection(name="bottle", conf=0.9, xyxy=(40, 0, 80, 40), cls=0)

    def setUp(self):
        boxes = [self.red_box, self.green_box]
        # 바운드 메서드는 접근할 때마다 새 객체라 assertIs 로 비교할 수 없다 -- 평범한 함수로 둔다.
        self.base = lambda model, cfg, frames: {v: list(boxes) for v in frames}

    def test_no_spec_passes_through(self):
        self.assertIs(pwc.make_infer(self.base, None, 0.5), self.base)

    def test_hsv_spec_keeps_only_target_color(self):
        spec = pick_cycle.ColorSpec(pick_cycle.DEFAULT_HUE_RANGES["red"])
        out = pwc.make_infer(self.base, spec, 0.5)(None, None, {"front": patch_frame(), "wrist": patch_frame()})
        self.assertEqual(out["front"], [self.red_box])
        self.assertEqual(out["wrist"], [self.red_box])


class TestOutcome(unittest.TestCase):
    def test_states(self):
        self.assertIsNone(pwc.outcome({}))
        self.assertIsNone(pwc.outcome({"state": "ALIGNING"}))
        self.assertEqual(pwc.outcome({"arm_done": True})[:2], (True, True))
        self.assertEqual(pwc.outcome({"gave_up": True})[:2], (False, False))
        ok, grasped, why = pwc.outcome({"state": "ERROR", "error": "카메라 없음"})
        self.assertFalse(ok)
        self.assertIn("카메라 없음", why)


class FakeStatus:
    def __init__(self, seq):
        self.seq = list(seq)

    def get(self):
        return self.seq.pop(0) if len(self.seq) > 1 else self.seq[0]


class FakeWorker:
    def __init__(self, statuses, alive=True):
        self.status = FakeStatus(statuses)
        self.calls = []
        self._thread = type("T", (), {"is_alive": lambda _s: alive})()

    def set_target_class(self, name):
        self.calls.append(("target", name))

    def start_background(self):
        self.calls.append("start")

    def resume(self):
        self.calls.append("resume")

    def request_stop(self):
        self.calls.append("stop")

    def join(self, timeout=None):
        self.calls.append("join")


class TestViewServer(unittest.TestCase):
    """집는 동안 워커가 그린 화면을 보여주는 보기 전용 서버."""

    def setUp(self):
        import socket
        with socket.socket() as sock:
            sock.bind(("127.0.0.1", 0))
            self.port = sock.getsockname()[1]
        frames = type("F", (), {"get": lambda _s, v: b"\xff\xd8JPEG-" + v.encode() + b"\xff\xd9"})()
        self.worker = FakeWorker([{"state": "SERVO", "pick_attempts": 1}])
        self.worker.frames = frames
        self.srv = pwc.start_view_server(self.worker, self.port, host="127.0.0.1")

    def tearDown(self):
        self.srv.shutdown()

    def get(self, path, n=None):
        import urllib.request
        with urllib.request.urlopen(f"http://127.0.0.1:{self.port}{path}", timeout=5) as r:
            return r.headers.get("Content-Type"), (r.read(n) if n else r.read())

    def test_page_and_status(self):
        ctype, body = self.get("/")
        self.assertIn("text/html", ctype)
        page = body.decode("utf-8")
        self.assertIn("'/stream/'", page)          # 영상 주소는 페이지 스크립트가 view 이름을 붙여 만든다
        self.assertIn("'front'", page)
        self.assertIn("'wrist'", page)
        ctype, body = self.get("/status")
        self.assertEqual(json.loads(body)["state"], "SERVO")

    def test_stream_sends_worker_frames(self):
        ctype, chunk = self.get("/stream/wrist", n=200)
        self.assertIn("multipart/x-mixed-replace", ctype)
        self.assertIn(b"JPEG-wrist", chunk)

    def test_no_write_paths(self):
        import urllib.error
        import urllib.request
        with self.assertRaises(urllib.error.HTTPError):
            urllib.request.urlopen(f"http://127.0.0.1:{self.port}/stop", timeout=5)
        req = urllib.request.Request(f"http://127.0.0.1:{self.port}/", method="POST")
        with self.assertRaises(urllib.error.HTTPError):
            urllib.request.urlopen(req, timeout=5)


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, dt):
        self.t += dt


def run(worker, max_seconds=10.0, **kw):
    clk = FakeClock()
    return pwc.run_cycle(worker, max_seconds=max_seconds, clock=clk, sleep=clk.sleep, **kw)


class TestRunCycle(unittest.TestCase):
    def assert_stopped(self, w):
        self.assertIn("stop", w.calls)
        self.assertIn("join", w.calls)
        self.assertLess(w.calls.index("resume"), w.calls.index("stop"))

    def test_success(self):
        w = FakeWorker([{"state": "ALIGNING"}, {"state": "CARRY_HOME"}, {"state": "DONE", "arm_done": True}])
        r = run(w, target_class="red_pill_bottle")
        self.assertTrue(r["success"])
        self.assertTrue(r["grasped"])
        self.assertIn(("target", "red_pill_bottle"), w.calls)
        self.assert_stopped(w)

    def test_no_target_class_does_not_set_filter(self):
        w = FakeWorker([{"arm_done": True}])
        run(w)
        self.assertFalse(any(isinstance(c, tuple) for c in w.calls))

    def test_gave_up(self):
        r = run(FakeWorker([{"state": "GRASP_FAIL"}, {"state": "GIVE_UP", "gave_up": True}]))
        self.assertFalse(r["success"])
        self.assertIn("포기", r["verdict_reason"])

    def test_error(self):
        r = run(FakeWorker([{"state": "ERROR", "error": "8초 안에 카메라 프레임을 받지 못했습니다."}]))
        self.assertFalse(r["success"])
        self.assertIn("카메라", r["verdict_reason"])

    def test_timeout_while_holding_bottle_reports_grasped(self):
        w = FakeWorker([{"state": "GRASP_CHECK"}])
        r = run(w, max_seconds=1.0)
        self.assertFalse(r["success"])
        self.assertTrue(r["grasped"])
        self.assertIn("제한시간", r["verdict_reason"])
        self.assert_stopped(w)

    def test_timeout_before_grasp(self):
        r = run(FakeWorker([{"state": "SEARCHING"}]), max_seconds=1.0)
        self.assertFalse(r["grasped"])

    def test_reports_each_status_change_once(self):
        lines = []
        w = FakeWorker([{"state": "APPROACH", "pick_attempts": 0}, {"state": "APPROACH", "pick_attempts": 0},
                        {"state": "GRASPED", "pick_attempts": 1}, {"state": "DONE", "arm_done": True}])
        run(w, report=lines.append)
        self.assertTrue(lines)
        self.assertIn("APPROACH", lines[0])
        self.assertTrue(any("GRASPED" in ln for ln in lines))
        texts = [ln.split("상태 ", 1)[1] for ln in lines]
        self.assertTrue(all(a != b for a, b in zip(texts, texts[1:])), lines)   # 같은 상태는 한 번만

    def test_no_report_by_default(self):
        w = FakeWorker([{"state": "DONE", "arm_done": True}])
        self.assertTrue(run(w)["success"])

    def test_external_stop_signal(self):
        flag = threading.Event()
        flag.set()
        w = FakeWorker([{"state": "ALIGNING"}])
        r = run(w, stop_flag=flag)
        self.assertFalse(r["success"])
        self.assertIn("외부 종료", r["verdict_reason"])
        self.assert_stopped(w)

    def test_loop_died_without_result(self):
        r = run(FakeWorker([{"state": "ALIGNING"}], alive=False))
        self.assertFalse(r["success"])
        self.assertIn("결과 없이", r["verdict_reason"])


class TestWithRealWorker(unittest.TestCase):
    """진짜 PickPlaceHeadlessWorker + 가짜 로봇. 실제 시간으로 짧게 돈다."""

    def test_camera_missing_ends_as_error_and_disconnects(self):
        robot = FakeRobot()
        robot.get_observation = lambda: {}
        w = PickPlaceHeadlessWorker(robot, PickPlaceConfig(pick=PickArgs(enabled=False)), {},
                                    infer_fn=_fake_infer, load_model_fn=_fake_load_model,
                                    first_obs_timeout_s=0.3, rollout_time_s=0.1)
        r = pwc.run_cycle(w, max_seconds=5.0, poll_s=0.05, join_timeout_s=5.0)
        self.assertFalse(r["success"])
        self.assertIn("카메라", r["verdict_reason"])
        self.assertFalse(robot.is_connected)

    def test_nothing_found_times_out_stops_wheels_and_disconnects(self):
        robot = FakeRobot()
        w = PickPlaceHeadlessWorker(robot, PickPlaceConfig(pick=PickArgs(enabled=False)), {},
                                    infer_fn=_fake_infer, load_model_fn=_fake_load_model,
                                    first_obs_timeout_s=0.5, rollout_time_s=0.1)
        r = pwc.run_cycle(w, max_seconds=0.6, poll_s=0.05, join_timeout_s=5.0,
                          target_class="red_pill_bottle")
        self.assertFalse(r["success"])
        self.assertIn("제한시간", r["verdict_reason"])
        self.assertEqual(w.get_target_class(), "red_pill_bottle")
        self.assertFalse(robot.is_connected)
        self.assertTrue(robot.sent_actions)
        for a in robot.sent_actions:   # 아무것도 못 찾았으니 바퀴는 한 번도 안 굴렀다
            self.assertEqual((a["x.vel"], a["y.vel"], a["theta.vel"]), (0.0, 0.0, 0.0))


class TestMain(unittest.TestCase):
    def test_missing_poses_writes_failure_without_touching_robot(self):
        with tempfile.TemporaryDirectory() as d:
            result = Path(d) / "r.json"
            rc = pwc.main(["--color", "green", "--result-file", str(result),
                           "--model", "없는모델.pt", "--poses-dir", d])
            self.assertEqual(rc, 0)
            data = json.loads(result.read_text(encoding="utf-8"))
            self.assertFalse(data["success"])
            self.assertIn("자세 파일 없음", data["verdict_reason"])
            self.assertEqual(data["color"], "green")

    def test_bad_model_writes_failure(self):
        from services.pickplace.poses import save_pose
        with tempfile.TemporaryDirectory() as d:
            for n in pwc.REQUIRED_POSES:
                save_pose(Path(d) / f"{n}.json", n, "lekiwi01", {"arm_gripper.pos": 10.0}, backup=False)
            result = Path(d) / "r.json"
            pwc.main(["--color", "red", "--result-file", str(result),
                      "--model", str(Path(d) / "없는모델.pt"), "--poses-dir", d,
                      "--remote-ip", "10.255.255.1"])
            data = json.loads(result.read_text(encoding="utf-8"))
            self.assertFalse(data["success"])
            self.assertIn("시작 실패", data["verdict_reason"])

    def test_args_same_as_pick_cycle(self):
        a = pwc.parse_args(REQ + ["--remote-ip", "10.0.0.9"])
        b = pick_cycle.parse_args(REQ + ["--remote-ip", "10.0.0.9"])
        for k in ("color", "result_file", "model", "poses_dir", "remote_ip"):
            self.assertEqual(getattr(a, k), getattr(b, k), k)


if __name__ == "__main__":
    unittest.main()
