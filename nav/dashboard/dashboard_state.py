"""ROS 콜백 스레드·웹 스레드가 함께 쓰는 대시보드 상태. ROS 메시지 대신 꺼낸 값만 받는다."""
from __future__ import annotations

import math
import threading

from map_geometry import MapInfo, Pose2D, compose, decimate, scan_points
from mission_timeline import MissionTimeline
from stop_latch import StopLatch
from topic_stats import TopicStats

IN_TOPICS = ["/abo/state", "/abo/status", "/abo/pick_request", "/abo/pick_done", "/tf", "/odom",
             "/scan", "/particle_cloud", "/amcl_pose", "/plan", "/map"]
OUT_TOPICS = ["/abo/command"]


def _age(now: float, t: float | None) -> float | None:
    return None if t is None else round(now - t, 2)


class DashboardState:
    def __init__(self, info: MapInfo, lidar: Pose2D = (0.0, 0.0, 0.0), max_particles: int = 800,
                 trail_max: int = 600, trail_step_m: float = 0.05) -> None:
        self.info, self.lidar = info, lidar
        self.max_particles, self.trail_max, self.trail_step = max_particles, trail_max, trail_step_m
        self._lock = threading.Lock()
        self.timeline = MissionTimeline()
        self.latch = StopLatch()
        self.topics = TopicStats()
        for t in IN_TOPICS:
            self.topics.declare(t, "in")
        for t in OUT_TOPICS:
            self.topics.declare(t, "out")
        self._map_odom: tuple[Pose2D, float] | None = None
        self._odom_base: tuple[Pose2D, float] | None = None
        self._cov_xy: float | None = None
        self._scan: tuple[list, float] | None = None
        self._particles: tuple[list, int, float] | None = None
        self._plan: tuple[list, float] | None = None
        self._trail: list[list[float]] = []
        self._warnings: list[str] = []
        self._pick: tuple[bool, dict | None, float | None] = (False, None, None)

    # ── 내부 ──
    def _robot(self) -> Pose2D | None:
        if self._map_odom is None or self._odom_base is None:
            return None
        return compose(self._map_odom[0], self._odom_base[0])

    # ── 입력 ──
    def on_tf(self, parent: str, child: str, pose: Pose2D, now: float) -> None:
        with self._lock:
            if (parent, child) == ("map", "odom"):
                self._map_odom = (pose, now)
            elif (parent, child) == ("odom", "base_link"):
                self._odom_base = (pose, now)
            self.topics.record("/tf", now, f"{parent}→{child}")

    def on_odom(self, pose_in_odom: Pose2D, now: float) -> None:
        with self._lock:
            self.topics.record("/odom", now, f"x {pose_in_odom[0]:+.2f} y {pose_in_odom[1]:+.2f}")
            if self._map_odom is None:
                return
            x, y, _ = compose(self._map_odom[0], pose_in_odom)
            if self._trail and math.hypot(x - self._trail[-1][0], y - self._trail[-1][1]) < self.trail_step:
                return
            self._trail.append([round(x, 3), round(y, 3)])
            del self._trail[:-self.trail_max]

    def on_scan(self, ranges, angle_min, angle_increment, range_min, range_max, now: float) -> None:
        with self._lock:
            robot = self._robot()
            pts = [] if robot is None else scan_points(ranges, angle_min, angle_increment,
                                                       range_min, range_max, robot, self.lidar)
            valid = sum(1 for r in ranges if math.isfinite(r) and range_min <= r <= range_max)
            self.topics.record("/scan", now, f"{len(ranges)}빔 · 유효 {valid}")
            if robot is not None:
                self._scan = (pts, now)

    def on_particles(self, xy: list[list[float]], now: float) -> None:
        with self._lock:
            self._particles = (decimate(xy, self.max_particles), len(xy), now)
            self.topics.record("/particle_cloud", now, f"{len(xy)}개")

    def on_amcl_pose(self, x: float, y: float, cov_xy: float, now: float) -> None:
        with self._lock:
            self._cov_xy = cov_xy
            self.topics.record("/amcl_pose", now, f"x {x:+.2f} y {y:+.2f} σ² {cov_xy:.3f}")

    def on_plan(self, xy: list[list[float]], now: float) -> None:
        with self._lock:
            self._plan = (xy, now)
            self.topics.record("/plan", now, f"{len(xy)}점")

    def on_map_msg(self, width, height, resolution, origin_x, origin_y, now: float) -> None:
        with self._lock:
            i = self.info
            same = (width, height) == (i.width, i.height) and abs(resolution - i.resolution) < 1e-3 \
                and abs(origin_x - i.origin_x) < 1e-3 and abs(origin_y - i.origin_y) < 1e-3
            self._warnings = [] if same else [
                f"/map 이 파일 지도와 다름 ({width}x{height}, {resolution:.3f}, {origin_x:.3f},{origin_y:.3f})"]
            self.topics.record("/map", now, f"{width}x{height}")

    def on_bridge_state(self, state: str, now: float) -> None:
        with self._lock:
            self.timeline.on_state(state, now, stop_latched=self.latch.latched)
            self.latch.on_bridge_state(state, now)
            self.topics.record("/abo/state", now, state)

    def on_bridge_status(self, text: str, now: float) -> None:
        with self._lock:
            self.timeline.on_status(text)
            self.topics.record("/abo/status", now, text)

    def on_command(self, text: str, now: float) -> None:
        with self._lock:
            self.timeline.on_command(text, now)
            if self.timeline.is_new_mission_since(now - 1e-9):
                self._trail = []
            self.topics.record("/abo/command", now, f'"{text}"')

    def on_pick_request(self, text: str, now: float) -> None:
        with self._lock:
            self.topics.record("/abo/pick_request", now, f'"{text}"')

    def on_pick_done(self, ok: bool, now: float) -> None:
        with self._lock:
            self.topics.record("/abo/pick_done", now, "true" if ok else "false")

    def set_pick(self, reachable: bool, status: dict | None, now: float) -> list[str]:
        with self._lock:
            prev_t = self._pick[2]
            self._pick = (reachable, status if reachable else None, now if reachable else prev_t)
            return self.latch.on_pick_reachable(reachable)

    # ── 정지 ──
    def engage_stop(self, now: float) -> list[str]:
        with self._lock:
            return self.latch.engage(now)

    def release_stop(self) -> None:
        with self._lock:
            self.latch.release()

    def stop_tick(self, now: float) -> list[str]:
        with self._lock:
            return self.latch.tick(now)

    def note_stop(self, action: str, result: str) -> None:
        with self._lock:
            self.latch.note(action, result)

    # ── 출력 ──
    def snapshot(self, now: float) -> dict:
        with self._lock:
            robot = self._robot()
            pose = None
            if robot is not None:
                older = min(self._map_odom[1], self._odom_base[1])
                pose = {"x": round(robot[0], 3), "y": round(robot[1], 3), "yaw": round(robot[2], 3),
                        "age": _age(now, older), "cov_xy": self._cov_xy}
            reachable, status, pick_t = self._pick
            return {
                "t": now,
                "pose": pose,
                "scan": {"pts": self._scan[0], "age": _age(now, self._scan[1])} if self._scan
                        else {"pts": [], "age": None},
                "particles": {"pts": self._particles[0], "n": self._particles[1],
                              "age": _age(now, self._particles[2])} if self._particles
                             else {"pts": [], "n": 0, "age": None},
                "plan": {"pts": self._plan[0], "age": _age(now, self._plan[1])} if self._plan
                        else {"pts": [], "age": None},
                "trail": list(self._trail),
                "mission": self.timeline.snapshot(now),
                "stop": self.latch.snapshot(),
                "pick": {"reachable": reachable, "status": status, "age": _age(now, pick_t) if reachable else None},
                "topics": self.topics.snapshot(now),
                "warnings": list(self._warnings),
            }
