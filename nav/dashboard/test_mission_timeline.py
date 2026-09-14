import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from mission_timeline import MissionTimeline  # noqa: E402


def _stages(tl, now):
    return [(s["key"], s["state"], s["sec"]) for s in tl.snapshot(now)["stages"]]


def test_no_mission_snapshot():
    snap = MissionTimeline().snapshot(5.0)
    assert snap["stages"] == [] and snap["elapsed"] is None and snap["target"] is None
    assert snap["outcome"] is None


def test_success_flow_with_moving_feedback_during_return():
    tl = MissionTimeline()
    tl.on_command("fetch center color:blue", 100.0)
    tl.on_state("moving", 100.5)
    tl.on_state("moving", 110.0)          # 주행 피드백
    tl.on_state("arrived", 129.0)
    tl.on_state("picking", 140.0)
    tl.on_state("returning", 212.0)
    tl.on_state("returning", 222.0)       # 위치 다듬기 후 한 번 더
    tl.on_state("moving", 230.0)          # 복귀 주행 피드백 — drive 를 다시 시작하면 안 됨
    assert _stages(tl, 240.0) == [("drive", "done", 39.5), ("pick", "done", 72.0), ("return", "active", 28.0)]
    tl.on_state("done", 255.0)
    snap = tl.snapshot(300.0)
    assert _stages(tl, 300.0) == [("drive", "done", 39.5), ("pick", "done", 72.0), ("return", "done", 43.0)]
    assert snap["elapsed"] == 155.0 and snap["target"] == "center" and snap["color"] == "blue"
    assert snap["outcome"] == "done"
    assert snap["stages"][0]["label"] == "Nav2 주행 home → center"
    assert snap["stages"][2]["label"] == "복귀 center → home"


def test_failure_marks_active_stage():
    tl = MissionTimeline()
    tl.on_command("fetch center color:green", 0.0)
    tl.on_state("moving", 1.0)
    tl.on_state("failed", 9.0)
    assert _stages(tl, 20.0) == [("drive", "failed", 8.0), ("pick", "todo", None), ("return", "todo", None)]
    assert tl.snapshot(20.0)["elapsed"] == 9.0


def test_idle_reply_to_stop_does_not_change_stages():
    tl = MissionTimeline()
    tl.on_command("fetch center", 0.0)
    tl.on_state("moving", 1.0)
    tl.on_state("picking", 5.0)
    tl.on_state("idle", 6.0, stop_latched=True)
    assert _stages(tl, 7.0)[1] == ("pick", "active", 2.0)
    assert tl.snapshot(7.0)["state"] == "idle"


def test_new_fetch_resets_and_return_target_parsed():
    tl = MissionTimeline()
    tl.on_command("fetch center", 0.0)
    tl.on_state("moving", 1.0)
    tl.on_state("canceled", 2.0)
    tl.on_command("fetch center color:red to home2", 50.0)
    snap = tl.snapshot(51.0)
    assert [s["state"] for s in snap["stages"]] == ["todo", "todo", "todo"]
    assert snap["stages"][2]["label"] == "복귀 center → home2"
    assert tl.is_new_mission_since(10.0) and not tl.is_new_mission_since(60.0)


def test_non_fetch_command_only_recorded():
    tl = MissionTimeline()
    tl.on_command("stop", 3.0)
    snap = tl.snapshot(4.0)
    assert snap["command"] == "stop" and snap["stages"] == []


def test_pick_done_success_then_done():
    tl = MissionTimeline()
    tl.on_command("fetch center", 0.0)
    tl.on_state("moving", 1.0)
    tl.on_state("picking", 5.0)
    tl.on_pick_done(True, 20.0)
    tl.on_state("returning", 21.0)
    tl.on_state("done", 30.0)
    assert _stages(tl, 40.0) == [("drive", "done", 4.0), ("pick", "done", 15.0), ("return", "done", 9.0)]
    assert tl.snapshot(40.0)["outcome"] == "done"


def test_failed_pick_still_returns_and_mission_fails():
    # 브리지는 집기 실패에도 returning 을 보내고 돌아온 뒤 failed 로 끝낸다.
    tl = MissionTimeline()
    tl.on_command("fetch center color:red", 0.0)
    tl.on_state("moving", 1.0)
    tl.on_state("picking", 5.0)
    tl.on_pick_done(False, 20.0)
    tl.on_state("returning", 21.0)
    tl.on_state("moving", 25.0)
    assert _stages(tl, 26.0) == [("drive", "done", 4.0), ("pick", "failed", 15.0), ("return", "active", 5.0)]
    tl.on_state("failed", 40.0)
    assert _stages(tl, 50.0) == [("drive", "done", 4.0), ("pick", "failed", 15.0), ("return", "done", 19.0)]
    snap = tl.snapshot(50.0)
    assert snap["outcome"] == "failed" and snap["elapsed"] == 40.0


def test_failed_while_driving_sets_outcome_failed():
    tl = MissionTimeline()
    tl.on_command("fetch center", 0.0)
    tl.on_state("moving", 1.0)
    tl.on_state("canceled", 3.0)
    assert tl.snapshot(4.0)["outcome"] == "failed"


def test_pick_done_ignored_when_pick_not_active():
    tl = MissionTimeline()
    tl.on_command("fetch center", 0.0)
    tl.on_state("moving", 1.0)
    tl.on_pick_done(False, 2.0)
    assert _stages(tl, 3.0)[1] == ("pick", "todo", None)


def test_repeated_fetch_during_mission_keeps_stages():
    tl = MissionTimeline()
    tl.on_command("fetch center color:blue", 0.0)
    tl.on_state("moving", 1.0)
    tl.on_state("picking", 5.0)
    tl.on_command("fetch center color:red", 6.0)          # 브리지는 무시하거나 rejected 로 답한다
    snap = tl.snapshot(7.0)
    assert _stages(tl, 7.0) == [("drive", "done", 4.0), ("pick", "active", 2.0), ("return", "todo", None)]
    assert (snap["target"], snap["color"], snap["elapsed"]) == ("center", "blue", 7.0)
    assert snap["command"] == "fetch center color:red"
    assert not tl.is_new_mission_since(1.0)


def test_rejected_during_mission_does_not_end_it():
    tl = MissionTimeline()
    tl.on_command("fetch center", 0.0)
    tl.on_state("moving", 1.0)
    tl.on_command("fetch shelf", 2.0)
    tl.on_state("rejected", 2.1)
    snap = tl.snapshot(3.0)
    assert _stages(tl, 3.0)[0] == ("drive", "active", 2.0)
    assert snap["state"] == "rejected" and snap["elapsed"] == 3.0 and snap["outcome"] is None
    assert tl.ended_at is None


def test_rejected_fresh_fetch_ends_as_failed():
    tl = MissionTimeline()
    tl.on_command("fetch nowhere", 0.0)
    tl.on_state("rejected", 0.1)
    snap = tl.snapshot(1.0)
    assert snap["outcome"] == "failed" and tl.ended_at == 0.1
    tl.on_command("fetch center", 2.0)                     # 다음 fetch 는 새 미션
    assert tl.is_new_mission_since(1.0)
