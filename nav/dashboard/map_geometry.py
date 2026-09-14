"""지도 파일·좌표 변환. ROS 없이 테스트한다.

좌표 규약 (ROS map_server 와 같음)
  - yaml origin 은 pgm 왼쪽 아래 모서리의 map 좌표
  - pgm 첫 행이 y 최대 → 픽셀 py = height - (y - origin_y) / res
"""
from __future__ import annotations

import io
import math
from dataclasses import dataclass
from pathlib import Path

import yaml
from PIL import Image

Pose2D = tuple[float, float, float]


@dataclass(frozen=True)
class MapInfo:
    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int
    image_path: Path


def load_map_info(yaml_path) -> MapInfo:
    yaml_path = Path(yaml_path)
    y = yaml.safe_load(yaml_path.read_text())
    image = (yaml_path.parent / y["image"]).resolve()
    with Image.open(image) as im:
        width, height = im.size
    ox, oy = float(y["origin"][0]), float(y["origin"][1])
    return MapInfo(float(y["resolution"]), ox, oy, width, height, image)


def map_png_bytes(info: MapInfo) -> bytes:
    buf = io.BytesIO()
    with Image.open(info.image_path) as im:
        im.convert("L").save(buf, format="PNG")
    return buf.getvalue()


def load_waypoints(path) -> dict[str, dict[str, float]]:
    data = yaml.safe_load(Path(path).read_text()) or {}
    return {
        name: {"x": float(p.get("x", 0.0)), "y": float(p.get("y", 0.0)), "yaw": float(p.get("yaw", 0.0))}
        for name, p in (data.get("points") or {}).items()
    }


def world_to_pixel(info: MapInfo, x: float, y: float) -> tuple[float, float]:
    return (x - info.origin_x) / info.resolution, info.height - (y - info.origin_y) / info.resolution


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def compose(a: Pose2D, b: Pose2D) -> Pose2D:
    ax, ay, at = a
    bx, by, bt = b
    c, s = math.cos(at), math.sin(at)
    return ax + c * bx - s * by, ay + s * bx + c * by, at + bt


def scan_points(ranges, angle_min, angle_increment, range_min, range_max,
                robot: Pose2D, lidar: Pose2D = (0.0, 0.0, 0.0)) -> list[list[float]]:
    sensor = compose(robot, lidar)
    out = []
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r < range_min or r > range_max:
            continue
        th = angle_min + i * angle_increment
        x, y, _ = compose(sensor, (r * math.cos(th), r * math.sin(th), 0.0))
        out.append([round(x, 3), round(y, 3)])
    return out


def decimate(points: list, max_n: int) -> list:
    if len(points) <= max_n:
        return list(points)
    step = len(points) / max_n
    return [points[int(i * step)] for i in range(max_n)]
