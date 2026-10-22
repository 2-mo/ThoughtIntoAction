"""Standard-library project paths and configuration, shared by all packages and launchers."""

from __future__ import annotations

from copy import deepcopy
import os
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[3]
LOG_DIR = Path(os.environ.get("QUEST_TELEOP_LOG_DIR", str(PROJECT_ROOT / "outputs/logs"))).expanduser()


def merge(base: dict, override: dict) -> dict:
    result = deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = merge(result[key], value)
        else:
            result[key] = deepcopy(value)
    return result


def resolve_path(value: str) -> Path:
    path = Path(os.path.expandvars(value)).expanduser()
    return path if path.is_absolute() else PROJECT_ROOT / path


def load_config(local_path: str | None = None) -> dict:
    # Path-only imports are also used by Python 3.10 control processes.
    # Application TOML configuration is loaded only in its Python 3.12 environment.
    import tomllib

    with (PROJECT_ROOT / "configs/project.toml").open("rb") as stream:
        config = tomllib.load(stream)
    selected = local_path or os.environ.get("QUEST_TELEOP_CONFIG")
    path = resolve_path(selected) if selected else PROJECT_ROOT / "configs/local.toml"
    if selected or path.exists():
        with path.open("rb") as stream:
            config = merge(config, tomllib.load(stream))
    return config


def port(name: str, config: dict | None = None) -> int:
    """Registered port from [ports] (configs/project.toml, overridable in configs/local.toml)."""
    return int((config or load_config())["ports"][name])

