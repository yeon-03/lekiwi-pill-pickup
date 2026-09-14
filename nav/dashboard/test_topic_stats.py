import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from topic_stats import TopicStats  # noqa: E402


def test_declared_but_never_received():
    ts = TopicStats()
    ts.declare("/scan", "in")
    assert ts.snapshot(10.0) == {"/scan": {"dir": "in", "rate": 0.0, "age": None, "last": ""}}


def test_rate_and_age_over_window():
    ts = TopicStats(rate_window_s=5.0)
    ts.declare("/odom", "in")
    for i in range(30):                      # 6 Hz 로 5 초
        ts.record("/odom", 100.0 + i / 6.0, f"n={i}")
    snap = ts.snapshot(105.0)["/odom"]
    assert snap["rate"] == 6.0
    assert snap["age"] == round(105.0 - (100.0 + 29 / 6.0), 2)
    assert snap["last"] == "n=29"


def test_rate_drops_to_zero_when_stale():
    ts = TopicStats(rate_window_s=5.0)
    ts.declare("/scan", "in")
    ts.record("/scan", 1.0, "x")
    assert ts.snapshot(20.0)["/scan"]["rate"] == 0.0
    assert ts.snapshot(20.0)["/scan"]["age"] == 19.0


def test_undeclared_topic_is_added_as_in_and_order_kept():
    ts = TopicStats()
    ts.declare("/abo/command", "out")
    ts.record("/tf", 1.0, "map→odom")
    assert list(ts.snapshot(2.0)) == ["/abo/command", "/tf"]
    assert ts.snapshot(2.0)["/tf"]["dir"] == "in"
