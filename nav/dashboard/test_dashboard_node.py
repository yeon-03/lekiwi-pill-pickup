import sys
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import dashboard_node  # noqa: E402  (rclpy 를 import 하지 않아야 한다)


def test_defaults_point_to_repo_files():
    a = dashboard_node.parse_args([])
    assert Path(a.map).is_file() and a.map.endswith("nav/maps/lekiwi01/map_0912_1654.yaml")
    assert Path(a.waypoints).is_file()
    assert (a.host, a.port, a.pick_url, a.no_particles) == ("127.0.0.1", 8001, "http://127.0.0.1:8000", False)


def test_flags():
    a = dashboard_node.parse_args(["--host", "0.0.0.0", "--port", "9001", "--no-particles"])
    assert (a.host, a.port, a.no_particles) == ("0.0.0.0", 9001, True)


def test_module_does_not_import_rclpy_at_top():
    assert "rclpy" not in dashboard_node.__dict__


def test_main_shutdown_order(monkeypatch):
    """rclpy.shutdown() 은 spin 스레드 join 전에, node.destroy_node() 는 join 뒤에 와야 한다
    (그렇지 않으면 spin 스레드가 구독/타이머를 쓰는 동안 노드가 파괴되는 경쟁 상태가 생긴다)."""
    calls: list = []

    class FakeNode:
        def create_timer(self, interval_s, cb):
            calls.append(("create_timer", interval_s))

        def publish_stop(self):
            calls.append("publish_stop")

        def destroy_node(self):
            calls.append("destroy")

    fake_rclpy = types.SimpleNamespace(
        init=lambda *a, **kw: calls.append("init"),
        ok=lambda: True,
        spin=lambda node: calls.append("spin"),
        shutdown=lambda: calls.append("shutdown"),
    )
    fake_uvicorn = types.SimpleNamespace(run=lambda *a, **kw: calls.append("run"))
    fake_ros_listener = types.ModuleType("ros_listener")
    fake_ros_listener.DashboardListener = lambda state, particles=True: FakeNode()

    fake_pick = types.SimpleNamespace(start=lambda fn: calls.append("pick_start"))
    fake_pick_client = types.ModuleType("pick_client")
    fake_pick_client.PickClient = lambda url: fake_pick

    def fake_make_stop_handlers(state, pick, publish_stop):
        return (lambda: None, lambda: None, lambda: None, lambda ok, status: None)

    fake_web_app = types.ModuleType("web_app")
    fake_web_app.make_stop_handlers = fake_make_stop_handlers
    fake_web_app.create_app = lambda state, png, meta, on_stop, on_release: object()

    fake_dashboard_state = types.ModuleType("dashboard_state")
    fake_dashboard_state.DashboardState = lambda info: object()

    monkeypatch.setitem(sys.modules, "rclpy", fake_rclpy)
    monkeypatch.setitem(sys.modules, "uvicorn", fake_uvicorn)
    monkeypatch.setitem(sys.modules, "ros_listener", fake_ros_listener)
    monkeypatch.setitem(sys.modules, "pick_client", fake_pick_client)
    monkeypatch.setitem(sys.modules, "web_app", fake_web_app)
    monkeypatch.setitem(sys.modules, "dashboard_state", fake_dashboard_state)

    dashboard_node.main([])

    assert calls.index("run") < calls.index("shutdown") < calls.index("destroy")
