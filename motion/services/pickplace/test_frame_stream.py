from services.pickplace.frame_stream import LatestFrame, SharedStatus


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
