import numpy as np

from services.pickplace.frame_stream import FrameAgeTracker, LatestFrame, SharedStatus


def test_latest_frame_returns_none_when_unset():
    frames = LatestFrame()
    assert frames.get("front") is None


def test_latest_frame_round_trip():
    frames = LatestFrame()
    frames.set("front", b"jpegbytes")
    assert frames.get("front") == b"jpegbytes"
    assert frames.get("wrist") is None


def test_shared_status_returns_empty_dict_by_default():
    status = SharedStatus()
    assert status.get() == {}


def test_shared_status_round_trip_returns_copy():
    status = SharedStatus()
    status.set({"state": "PICK"})
    got = status.get()
    assert got == {"state": "PICK"}
    got["state"] = "MUTATED"
    assert status.get() == {"state": "PICK"}  # 내부 dict 가 외부 변경에 영향받지 않음


def test_frame_age_grows_while_frame_is_identical():
    t = FrameAgeTracker()
    f = np.zeros((48, 64, 3), dtype=np.uint8)
    t.update("front", f, now=10.0)
    t.update("front", f.copy(), now=12.5)
    assert t.ages(now=13.0) == {"front": 3.0}


def test_frame_age_resets_when_content_changes():
    t = FrameAgeTracker()
    a = np.zeros((48, 64, 3), dtype=np.uint8)
    b = a.copy()
    b[::4, ::4] = 200
    t.update("wrist", a, now=1.0)
    t.update("wrist", b, now=4.0)
    assert t.ages(now=4.5) == {"wrist": 0.5}


def test_frame_age_is_per_view():
    t = FrameAgeTracker()
    f = np.zeros((8, 8, 3), dtype=np.uint8)
    t.update("front", f, now=0.0)
    t.update("wrist", f, now=2.0)
    assert t.ages(now=3.0) == {"front": 3.0, "wrist": 1.0}
