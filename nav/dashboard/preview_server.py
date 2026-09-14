#!/usr/bin/env python3
"""대시보드 미리보기 서버 — ROS·로봇 없이 가짜 이벤트로 화면을 띄운다 (스크린샷·시연용).

    /usr/bin/python3 nav/dashboard/preview_server.py --scenario picking      # http://127.0.0.1:8011
    시나리오: idle | driving | picking | done | failed

정지 버튼은 아무것도 발행하지 않고 로그만 남긴다. 카메라 <img> 는 :8000 스트림을 찾으므로
집기 워커가 없으면 "영상 연결 안 됨" 자리표시가 나온다.
"""
from __future__ import annotations

import argparse
import math
import random
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))

from dashboard_state import DashboardState  # noqa: E402
from map_geometry import MapInfo, load_map_info, load_waypoints  # noqa: E402

SCENARIOS = ("idle", "driving", "picking", "done", "failed")
DEFAULT_MAP = REPO / "nav/maps/lekiwi01/map_0912_1654.yaml"
DEFAULT_WAYPOINTS = REPO / "nav/mission/waypoints.yaml"
HOME, CENTER = (0.0, 0.0), (1.10, -0.10)
MAP_ODOM = (0.02, -0.01, 0.01)

PICK_STATUS = {
    "state": "GRASP", "hz": 9.6, "frame_age_s": {"front": 0.1, "wrist": 0.12},
    "purple": {"front": {"ratio": 0.27, "thr": 0.23}, "wrist": {"ratio": 0.12, "thr": 0.20}},
    "pick_attempts": 2, "max_pick_attempts": 5,
}


def _lerp(a, b, f):
    return [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f]


def _ray_ranges(info: MapInfo, pose, n: int = 360) -> tuple[list[float], float, float]:
    """지도 테두리 사각형까지 거리 — 방 벽처럼 보이는 가짜 /scan."""
    x0, y0 = info.origin_x, info.origin_y
    x1, y1 = x0 + info.width * info.resolution, y0 + info.height * info.resolution
    inc = 2 * math.pi / n
    out = []
    for i in range(n):
        th = pose[2] + (-math.pi + i * inc)
        dx, dy = math.cos(th), math.sin(th)
        t = min(((x1 - pose[0]) / dx if dx > 1e-9 else (x0 - pose[0]) / dx if dx < -1e-9 else 1e9),
                ((y1 - pose[1]) / dy if dy > 1e-9 else (y0 - pose[1]) / dy if dy < -1e-9 else 1e9))
        out.append(t if i % 9 else float("inf"))
    return out, -math.pi, inc


def _pose_in_odom(xy, yaw):
    """map 좌표 xy 를 map→odom 을 뺀 odom 좌표로."""
    ox, oy, ot = MAP_ODOM
    c, s = math.cos(-ot), math.sin(-ot)
    dx, dy = xy[0] - ox, xy[1] - oy
    return (c * dx - s * dy, s * dx + c * dy, yaw - ot)


def feed_live(state: DashboardState, xy, yaw: float, now: float, span_s: float = 5.0, info: MapInfo | None = None,
              pick: bool = False) -> None:
    """최근 span_s 동안 주기 토픽(/tf /odom /scan /particle_cloud) 을 채운다 — 주기·나이가 실제처럼 보이게."""
    info = info or state.info
    pose = _pose_in_odom(xy, yaw)
    rng = random.Random(7)
    steps = int(span_s * 10)
    for k in range(steps + 1):
        t = now - span_s + k * span_s / steps
        state.on_tf("map", "odom", MAP_ODOM, t)
        state.on_tf("odom", "base_link", pose, t)
        state.on_odom(pose, t)
        if k % 2 == 0:
            ranges, amin, inc = _ray_ranges(info, (xy[0], xy[1], yaw))
            state.on_scan(ranges, amin, inc, 0.05, 12.0, t)
        if k % 10 == 0:
            state.on_particles([[xy[0] + rng.gauss(0, 0.06), xy[1] + rng.gauss(0, 0.06)] for _ in range(400)], t)
        if pick:
            state.set_pick(True, dict(PICK_STATUS), t)
    state.on_amcl_pose(xy[0], xy[1], 0.012, now)


def _trail(state: DashboardState, pts, t0: float, t1: float) -> None:
    for i, p in enumerate(pts):
        state.on_odom(_pose_in_odom(p, 0.0), t0 + (t1 - t0) * i / max(1, len(pts) - 1))


def _path(a, b, n=24):
    return [_lerp(a, b, i / (n - 1)) for i in range(n)]


def build_state(scenario: str, now: float, info: MapInfo | None = None) -> DashboardState:
    """시나리오의 가짜 이벤트를 시각 순서대로 넣은 DashboardState."""
    if scenario not in SCENARIOS:
        raise ValueError(f"unknown scenario {scenario!r} (choose {', '.join(SCENARIOS)})")
    info = info or load_map_info(DEFAULT_MAP)
    s = DashboardState(info)
    T = now
    s.on_tf("map", "odom", MAP_ODOM, T - 200)
    s.on_map_msg(info.width, info.height, info.resolution, info.origin_x, info.origin_y, T - 200)

    if scenario == "idle":
        s.on_bridge_state("idle", T - 60)
        s.on_bridge_status("가고 있지 않아요.", T - 60)
        feed_live(s, HOME, 0.0, T, info=info)
        return s

    go = _path(HOME, CENTER)
    back = _path(CENTER, HOME)
    if scenario == "driving":
        s.on_command("fetch center color:blue", T - 21)
        s.on_bridge_state("moving", T - 20)
        s.on_bridge_status("가는 중이에요.", T - 19)
        s.on_plan(go, T - 19)
        _trail(s, go[:15], T - 19, T - 1)
        s.on_bridge_state("moving", T - 2)
        s.on_bridge_status("가는 중이에요. 0.4 m 남았어요.", T - 1)
        feed_live(s, go[14], -0.09, T, info=info)
        return s

    # picking / done / failed 는 같은 앞부분
    base = {"picking": T - 72, "done": T - 150, "failed": T - 158}[scenario]
    s.on_command("fetch center color:blue", base)
    s.on_bridge_state("moving", base + 1)
    s.on_bridge_status("가는 중이에요.", base + 2)
    s.on_plan(go, base + 2)
    _trail(s, go, base + 2, base + 29)
    s.on_bridge_state("arrived", base + 30)
    s.on_bridge_status("도착했어요.", base + 30)
    s.on_bridge_state("picking", base + 31)
    s.on_pick_request("pick color:blue", base + 31)
    s.on_bridge_status("도착했어요. 물건을 집는 중이에요.", base + 32)
    if scenario == "picking":
        feed_live(s, CENTER, 0.0, T, info=info, pick=True)
        return s

    ok = scenario == "done"
    t_pick = base + 31 + (69 if ok else 77)
    s.set_pick(True, dict(PICK_STATUS), t_pick - 1)
    s.on_pick_done(ok, t_pick)
    s.set_pick(False, None, t_pick + 1)
    s.on_bridge_state("returning", t_pick + 1)
    s.on_bridge_status("물건을 가지고 돌아가는 중이에요." if ok else "물건은 못 집었어요. 돌아가는 중이에요.", t_pick + 2)
    s.on_plan(back, t_pick + 2)
    s.on_bridge_state("moving", t_pick + 10)
    _trail(s, back, t_pick + 2, t_pick + 40)
    end = t_pick + 44
    s.on_bridge_state("done" if ok else "failed", end)
    s.on_bridge_status("돌아왔어요. 물건을 가져왔어요." if ok else "돌아왔어요. 물건은 집지 못했어요.", end)
    feed_live(s, HOME, 0.0, T, info=info)
    return s


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", choices=SCENARIOS, default="picking")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8011)
    ap.add_argument("--map", default=str(DEFAULT_MAP))
    ap.add_argument("--waypoints", default=str(DEFAULT_WAYPOINTS))
    args = ap.parse_args(argv)

    import uvicorn

    from map_geometry import map_png_bytes
    from web_app import create_app, make_stop_handlers

    info = load_map_info(args.map)
    meta = {"resolution": info.resolution, "origin_x": info.origin_x, "origin_y": info.origin_y,
            "width": info.width, "height": info.height, "waypoints": load_waypoints(args.waypoints)}
    now = time.time()
    state = build_state(args.scenario, now, info=info)
    xy = {"driving": _path(HOME, CENTER)[14], "picking": CENTER}.get(args.scenario, HOME)

    class _LogPick:
        def estop(self) -> str:
            print("[preview] estop (보내지 않음)", flush=True)
            return "estop (미리보기 — 보내지 않음)"

    on_stop, on_release, _tick, _upd = make_stop_handlers(
        state, _LogPick(), lambda: print("[preview] stop (발행하지 않음)", flush=True))

    def live():
        while True:
            time.sleep(0.5)
            feed_live(state, xy, 0.0, time.time(), span_s=0.5, info=info, pick=args.scenario == "picking")

    threading.Thread(target=live, daemon=True, name="preview-live").start()
    app = create_app(state, map_png_bytes(info), meta, on_stop, on_release)
    print(f"[preview] {args.scenario}  http://{args.host}:{args.port}", flush=True)
    uvicorn.run(app, host=args.host, port=args.port, log_level="warning", timeout_graceful_shutdown=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
