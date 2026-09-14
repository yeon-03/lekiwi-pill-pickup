#!/usr/bin/env python3
"""LeKiwi 관제 대시보드 진입점 (노트북 전용, 로봇에 배포하지 않는다).

    nav/dashboard/run_dashboard.sh --robot-ip 223.194.139.15      # 환경변수까지 맞춰 실행
    브라우저: http://localhost:8001

집기 카메라는 pick_worker_cycle.py 를 -- --web-port 8000 으로 띄웠을 때만 나온다.
"""
from __future__ import annotations

import argparse
import sys
import threading
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--map", default=str(REPO / "nav/maps/lekiwi01/map_0912_1654.yaml"))
    ap.add_argument("--waypoints", default=str(REPO / "nav/mission/waypoints.yaml"))
    ap.add_argument("--host", default="127.0.0.1", help="다른 기기에서 볼 때만 0.0.0.0")
    ap.add_argument("--port", type=int, default=8001)
    ap.add_argument("--pick-url", default="http://127.0.0.1:8000")
    ap.add_argument("--no-particles", action="store_true", help="파티클 구독을 끈다 (로봇 와이파이 송신량 절약)")
    return ap.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    import rclpy
    import uvicorn

    from dashboard_state import DashboardState
    from map_geometry import load_map_info, load_waypoints, map_png_bytes
    from pick_client import PickClient
    from ros_listener import DashboardListener
    from web_app import create_app, make_stop_handlers

    info = load_map_info(args.map)
    meta = {"resolution": info.resolution, "origin_x": info.origin_x, "origin_y": info.origin_y,
            "width": info.width, "height": info.height, "waypoints": load_waypoints(args.waypoints)}
    state = DashboardState(info)
    pick = PickClient(args.pick_url)

    rclpy.init()
    node = DashboardListener(state, particles=not args.no_particles)
    on_stop, on_release, on_tick, on_pick_update = make_stop_handlers(state, pick, node.publish_stop)
    node.create_timer(0.1, on_tick)
    pick.start(on_pick_update)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True, name="ros-spin").start()

    app = create_app(state, map_png_bytes(info), meta, on_stop, on_release)
    print(f"[dashboard] http://{args.host}:{args.port}  지도 {Path(args.map).name}  집기 {args.pick_url}", flush=True)
    try:
        uvicorn.run(app, host=args.host, port=args.port, log_level="warning", timeout_graceful_shutdown=2)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
