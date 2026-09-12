"""앱 아이덴티티 단일 정의 모듈.

제품명은 3계층으로 나뉜다. 값이 흩어지면 `.desktop` 의 `StartupWMClass` 와
Qt 의 `applicationName` 이 어긋나 GNOME 독에서 창-아이콘 매칭이 깨지는 등
조용한 버그가 생기므로 **이 모듈이 유일한 정의처**다.

- `APP_INTERNAL_NAME` : WMClass / setApplicationName / 설정 디렉토리 / 키링 서비스명
- `APP_PKG_NAME`      : Debian 패키지명 / 설치 경로 / 캐시 디렉토리 / .desktop basename
- `APP_DISPLAY_NAME`  : 사용자에게 보이는 이름 (창 제목, .desktop Name=)

`LEGACY_*` 상수는 roboseasy-studio → physical-labs 리네임 이전 값이며
`config/migration.py` 의 유저 데이터 이관에서만 사용한다.

이 모듈은 **stdlib 만 import** 한다 (순환 import 원천 차단).
"""
from __future__ import annotations

import os
from pathlib import Path

APP_INTERNAL_NAME = "PhysicalLabs"
APP_PKG_NAME = "physical-labs"
APP_DISPLAY_NAME = "Physical Labs"

# 리네임 이전 값 — 마이그레이션 전용. 신규 코드에서 참조하지 말 것.
LEGACY_INTERNAL_NAME = "Roboseasy"
LEGACY_PKG_NAME = "roboseasy-studio"


def _config_base() -> Path:
    """플랫폼별 사용자 설정 루트 (앱 이름 미포함)."""
    if os.name == "nt":
        return Path(os.environ.get("APPDATA") or Path.home() / "AppData" / "Roaming")
    if os.name == "posix" and os.uname().sysname == "Darwin":  # type: ignore[attr-defined]
        return Path.home() / "Library" / "Application Support"
    return Path(os.environ.get("XDG_CONFIG_HOME") or Path.home() / ".config")


def _cache_base() -> Path:
    """사용자 캐시 루트 (앱 이름 미포함).

    기존 동작을 그대로 유지하기 위해 Windows 에서도 `~/.cache` 를 쓴다 —
    리네임 이전 녹화 모션이 `~/.cache/roboseasy-studio` 에 있으므로
    베이스를 바꾸면 마이그레이션이 원본을 찾지 못한다.
    """
    return Path.home() / ".cache"


def app_config_dir() -> Path:
    """사용자 설정 디렉토리. 없으면 생성한다.

    - Linux:   ~/.config/PhysicalLabs
    - Windows: %APPDATA%\\PhysicalLabs
    - macOS:   ~/Library/Application Support/PhysicalLabs
    """
    d = _config_base() / APP_INTERNAL_NAME
    d.mkdir(parents=True, exist_ok=True)
    return d


def app_cache_dir() -> Path:
    """사용자 캐시 디렉토리(녹화 모션 JSON 등). 없으면 생성한다.

    Returns:
        `~/.cache/physical-labs` (전 플랫폼 동일).
    """
    d = _cache_base() / APP_PKG_NAME
    d.mkdir(parents=True, exist_ok=True)
    return d


def app_data_dir() -> Path:
    """사용자 산출물 루트(학습 결과 등). 없으면 생성한다.

    Returns:
        `~/.PhysicalLabs` (전 플랫폼 동일).

    설정(`app_config_dir`)·캐시(`app_cache_dir`)와 분리한 이유: 학습 체크포인트는
    수 GB 가 되고 사용자가 직접 열어보거나 옮기는 대상이라 "지워도 되는 캐시" 가
    아니다. 앱이 지우거나 비워서는 안 된다.
    """
    d = Path.home() / f".{APP_INTERNAL_NAME}"
    d.mkdir(parents=True, exist_ok=True)
    return d


def train_outputs_dir() -> Path:
    """학습 출력 루트. 없으면 생성한다.

    Returns:
        `~/.PhysicalLabs/outputs/train`

    **절대 경로여야 한다.** 과거에는 `outputs/train` 상대 경로를 써서 앱을 어디서
    실행했느냐(GNOME 메뉴 → `$HOME`, 터미널 → 그 디렉토리)에 따라 결과물이 흩어졌고,
    '학습 재개' 탭이 다른 위치의 이전 학습을 찾지 못했다.
    """
    d = app_data_dir() / "outputs" / "train"
    d.mkdir(parents=True, exist_ok=True)
    return d


def legacy_train_outputs_roots() -> list[Path]:
    """리네임/경로 이전에 학습 결과가 생겼을 수 있는 위치들. **생성하지 않는다**.

    상대 경로 시절 산출물은 앱을 실행한 디렉토리 아래 `outputs/train` 에 있다.
    가장 흔한 두 곳(홈, 현재 작업 디렉토리)을 재개 스캔 대상으로 함께 본다 —
    사용자가 이미 돌려둔 학습을 경로 변경 때문에 잃지 않도록.
    """
    roots = [Path.home() / "outputs" / "train", Path.cwd() / "outputs" / "train"]
    seen: set[Path] = set()
    out: list[Path] = []
    for root in roots:
        resolved = root.resolve()
        if resolved not in seen:
            seen.add(resolved)
            out.append(root)
    return out


def legacy_config_dir() -> Path:
    """리네임 이전 설정 디렉토리 경로. **생성하지 않는다** (존재 확인 전용)."""
    return _config_base() / LEGACY_INTERNAL_NAME


def legacy_cache_dir() -> Path:
    """리네임 이전 캐시 디렉토리 경로. **생성하지 않는다** (존재 확인 전용)."""
    return _cache_base() / LEGACY_PKG_NAME


def env_flag(name: str) -> str | None:
    """`PHYSICALLABS_<name>` 환경변수를 읽고, 없으면 구 `ROBOSEASY_<name>` 로 폴백.

    구 접두사 폴백은 리네임 직후 1릴리스만 유지한다.

    Args:
        name: 접두사를 뺀 변수 이름 (예: `"DEBUG_LOG"`).

    Returns:
        설정된 값. 둘 다 없으면 None.
    """
    return os.environ.get(f"PHYSICALLABS_{name}") or os.environ.get(f"ROBOSEASY_{name}")
