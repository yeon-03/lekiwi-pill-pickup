"""저장된 모션(JSON) 파일 스캔·로드 — 엔드이펙터·관절제어·워크스페이스 **공용**.

## 왜 뺐나 (2026-09-08)

같은 스캔 로직이 `ui/eef_pane.py`(XYZ 제어) 와 `ui/eef_pose_pane.py`(관절 제어) 에
각각 박혀 있었다. 워크스페이스 "Yolo + Pick&Place" 의 Place 단계가 세 계열의
모션을 **한 목록**으로 보여줘야 해서 여기로 모았다. 두 pane 은 이 모듈에
`root=` 를 넘겨 위임한다 (테스트가 pane 의 `_motion_cache_root` 를 바꿔 격리하므로
루트는 인자로 받는다).

## 세 계열과 경로 (전부 `app_cache_dir()` 아래)

| 계열(family) | 경로 | 프레임 관절 키 |
|---|---|---|
| `teleop` | `<robot_id>/motion/teleop/{sim,real}/*.json` | `frame["action"]` (`shoulder_pan.pos` …, 바퀴 `x.vel` 포함 가능) |
| `xyzcontrol` | `<robot_id>/motion/xyzcontrol/{sim,real}/*.json` + 레거시 평탄 `*.json` | `frame["joints"]` (`shoulder_pan` …) + 선택 `frame["base"]` |
| `jointcontrol` | `<robot_id>/motion/jointcontrol/{sim,real}/*.json` + 레거시 평탄 `*.json` | `frame["joints"]` (`EefJointControlPane` Motion Record) |
| `posecontrol` | `<robot_id>/motion/posecontrol/real/*.json` | `frame["joints"]` (`EefPoseControlPane`) |

`frame_joints()` 가 세 모양을 **짧은 관절 이름 → 값** 한 모양으로 정규화하고,
`frame_base()` 가 바퀴 속도를 꺼낸다. 소비자는 계열을 몰라도 된다.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

from config.app_identity import app_cache_dir

#: 스캔 대상 계열. 순서는 화면 표시 순서.
FAMILIES: tuple[str, ...] = ("teleop", "xyzcontrol", "jointcontrol", "posecontrol")

#: 계열별 표시명.
FAMILY_LABELS: dict[str, str] = {
    "teleop": "텔레오퍼레이션",
    "xyzcontrol": "XYZ 제어",
    "jointcontrol": "관절 제어",
    "posecontrol": "포즈 제어",
}

#: 계열별 sim/real 하위 폴더. posecontrol 은 real 만 쓴다.
_FAMILY_SUBDIRS: dict[str, tuple[str, ...]] = {
    "teleop": ("sim", "real"),
    "xyzcontrol": ("sim", "real"),
    "jointcontrol": ("sim", "real"),
    "posecontrol": ("real",),
}

#: 계열 폴더 바로 아래의 평탄 `*.json` 도 스캔하는 계열 (xyzcontrol 레거시 구조).
_FAMILY_FLAT: frozenset[str] = frozenset({"xyzcontrol", "jointcontrol"})

_BASE_KEYS: tuple[str, ...] = ("x.vel", "y.vel", "theta.vel")


def motion_root() -> Path:
    """모션 파일 루트 (`~/.cache/physical-labs`)."""
    return app_cache_dir()


def family_dir(root: Path, robot_id: str, family: str) -> Path:
    return Path(root) / robot_id / "motion" / family


def _candidate_dirs(root: Path, robot_id: str, family: str) -> list[Path]:
    base = family_dir(root, robot_id, family)
    dirs: list[Path] = []
    if family in _FAMILY_FLAT:
        dirs.append(base)
    dirs.extend(base / sub for sub in _FAMILY_SUBDIRS.get(family, ()))
    return dirs


def _has_json(d: Path) -> bool:
    return d.is_dir() and any(d.glob("*.json"))


def any_motion_file_exists(root: Path, family: str) -> bool:
    """어느 로봇이든 이 계열의 모션 파일이 하나라도 있는가."""
    root = Path(root)
    if not root.is_dir():
        return False
    for entry in root.iterdir():
        if not entry.is_dir():
            continue
        if any(_has_json(d) for d in _candidate_dirs(root, entry.name, family)):
            return True
    return False


def scan_robot_dirs(root: Path, family: str) -> list[str]:
    """이 계열의 모션 파일을 가진 robot_id 목록 (이름순)."""
    root = Path(root)
    if not root.is_dir():
        return []
    result: list[str] = []
    for entry in sorted(root.iterdir()):
        if not entry.is_dir():
            continue
        if any(_has_json(d) for d in _candidate_dirs(root, entry.name, family)):
            result.append(entry.name)
    return result


def load_motion(path: Path) -> dict | None:
    """모션 JSON 을 읽는다. 깨진 파일이면 None."""
    try:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
    except Exception:
        return None
    return data if isinstance(data, dict) else None


def scan_motion_files(root: Path, robot_id: str, family: str) -> list[tuple[Path, dict]]:
    """한 로봇·한 계열의 모션 파일을 **최근 수정순**으로 돌려준다 (pane 과 같은 계약)."""
    out: list[tuple[Path, dict]] = []
    for d in _candidate_dirs(root, robot_id, family):
        if not d.is_dir():
            continue
        for p in d.glob("*.json"):
            data = load_motion(p)
            if data is None:
                continue
            out.append((p, data))
    out.sort(key=lambda item: item[0].stat().st_mtime, reverse=True)
    return out


# ── 통합 목록 (워크스페이스 Place 단계) ─────────────────────────────


@dataclass(frozen=True)
class MotionInfo:
    """모션 파일 한 개의 요약. `data` 는 원본 JSON 전체."""

    path: Path
    robot_id: str
    family: str
    source: str            # "sim" | "real" | "" (레거시 평탄 구조)
    name: str
    fps: int
    frame_count: int
    duration_s: float
    has_base_motion: bool
    data: dict = field(repr=False, compare=False)

    @property
    def family_label(self) -> str:
        return FAMILY_LABELS.get(self.family, self.family)

    @property
    def frames(self) -> list[dict]:
        frames = self.data.get("frames")
        return frames if isinstance(frames, list) else []


def _source_of(path: Path, family: str) -> str:
    parent = path.parent.name
    if parent in _FAMILY_SUBDIRS.get(family, ()):
        return parent
    return ""


def describe_motion(path: Path, data: dict, *, robot_id: str, family: str) -> MotionInfo:
    frames = data.get("frames") if isinstance(data.get("frames"), list) else []
    try:
        fps = int(data.get("fps") or 0)
    except (TypeError, ValueError):
        fps = 0
    try:
        frame_count = int(data.get("frame_count") or len(frames))
    except (TypeError, ValueError):
        frame_count = len(frames)
    duration = data.get("duration_seconds")
    if duration is None and frames:
        try:
            duration = float(frames[-1].get("t", 0.0))
        except (TypeError, ValueError, AttributeError):
            duration = 0.0
    try:
        duration_s = float(duration or 0.0)
    except (TypeError, ValueError):
        duration_s = 0.0
    return MotionInfo(
        path=path,
        robot_id=robot_id,
        family=family,
        source=_source_of(path, family),
        name=str(data.get("name") or path.stem),
        fps=fps,
        frame_count=frame_count,
        duration_s=duration_s,
        has_base_motion=frames_have_base_motion(frames),
        data=data,
    )


def scan_all_motions(
    root: Path, robot_id: str, families: tuple[str, ...] = FAMILIES,
) -> list[MotionInfo]:
    """한 로봇의 모든 계열 모션을 최근 수정순 한 목록으로."""
    infos: list[MotionInfo] = []
    for family in families:
        for path, data in scan_motion_files(root, robot_id, family):
            infos.append(describe_motion(path, data, robot_id=robot_id, family=family))
    infos.sort(key=lambda m: m.path.stat().st_mtime if m.path.exists() else 0.0, reverse=True)
    return infos


# ── 프레임 정규화 ───────────────────────────────────────────────────


def _short_joint_name(raw: str) -> str:
    name = raw[: -len(".pos")] if raw.endswith(".pos") else raw
    if "." in name:
        return ""                        # `x.vel` 등 바퀴 속도
    if name.startswith("arm_"):
        name = name[len("arm_"):]
    return name


def frame_joints(frame: dict) -> dict[str, float]:
    """프레임 한 개 → `{짧은 관절 이름: 값}`.

    `joints`(XYZ·관절 제어) 와 `action`(텔레옵) 어느 모양이든 받는다.
    관절이 아닌 키(바퀴 속도)는 버린다.
    """
    if not isinstance(frame, dict):
        return {}
    src = frame.get("joints")
    if not isinstance(src, dict):
        src = frame.get("action")
    if not isinstance(src, dict):
        return {}
    out: dict[str, float] = {}
    for raw, value in src.items():
        if not isinstance(raw, str):
            continue
        name = _short_joint_name(raw)
        if not name:
            continue
        try:
            out[name] = float(value)
        except (TypeError, ValueError):
            continue
    return out


def frame_base(frame: dict) -> dict[str, float]:
    """프레임의 바퀴 속도 `{x.vel, y.vel, theta.vel}` (없는 키는 0)."""
    out = {k: 0.0 for k in _BASE_KEYS}
    if not isinstance(frame, dict):
        return out
    src = frame.get("base")
    if not isinstance(src, dict):
        src = frame.get("action") if isinstance(frame.get("action"), dict) else {}
    for k in _BASE_KEYS:
        try:
            v = float(src.get(k, 0.0))
        except (TypeError, ValueError):
            continue
        if v == v and v not in (float("inf"), float("-inf")):
            out[k] = v
    return out


def frames_have_base_motion(frames) -> bool:
    for frame in frames or ():
        if any(v != 0.0 for v in frame_base(frame).values()):
            return True
    return False


def first_frame_joints(data: dict) -> dict[str, float]:
    frames = data.get("frames") if isinstance(data, dict) else None
    if not isinstance(frames, list) or not frames:
        return {}
    return frame_joints(frames[0])


__all__ = [
    "FAMILIES",
    "FAMILY_LABELS",
    "MotionInfo",
    "any_motion_file_exists",
    "describe_motion",
    "family_dir",
    "first_frame_joints",
    "frame_base",
    "frame_joints",
    "frames_have_base_motion",
    "load_motion",
    "motion_root",
    "scan_all_motions",
    "scan_motion_files",
    "scan_robot_dirs",
]
