from __future__ import annotations

import json
import math
from pathlib import Path
import time
from typing import Any, Iterable

from arx5_paths import CALIBRATIONS_ROOT


DEFAULT_COLLECTION_HOME_PATH = CALIBRATIONS_ROOT / "collection" / "home.json"
DEFAULT_TRAINING_START_PATH = CALIBRATIONS_ROOT / "collection" / "training_start.json"


def _joint_values(values: object) -> tuple[float, ...]:
    if not isinstance(values, list) or len(values) != 6:
        raise ValueError("采集零位必须包含 6 个关节位置")
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError("采集零位关节位置必须为有限数值")
    return result


def _load_collection_pose(
    *,
    interface: str,
    path: Path,
) -> dict[str, Any] | None:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, OSError, json.JSONDecodeError):
        return None
    if not isinstance(payload, dict):
        return None
    if payload.get("schema_version") != 1 or payload.get("interface") != interface:
        return None
    try:
        joints = _joint_values(payload.get("joint_position_rad"))
        gripper = float(payload.get("gripper_position_m"))
    except (TypeError, ValueError):
        return None
    if not math.isfinite(gripper):
        return None
    return {
        "schema_version": 1,
        "interface": interface,
        "joint_position_rad": joints,
        "gripper_position_m": gripper,
        "calibrated_at": payload.get("calibrated_at"),
        "method": payload.get("method", "first_collection_prepare"),
    }


def load_collection_home(
    *,
    interface: str,
    path: Path = DEFAULT_COLLECTION_HOME_PATH,
) -> dict[str, Any] | None:
    return _load_collection_pose(interface=interface, path=path)


def load_training_start(
    *,
    interface: str,
    path: Path = DEFAULT_TRAINING_START_PATH,
) -> dict[str, Any] | None:
    return _load_collection_pose(interface=interface, path=path)


def save_collection_home(
    joint_position_rad: Iterable[float],
    gripper_position_m: float,
    *,
    interface: str,
    path: Path = DEFAULT_COLLECTION_HOME_PATH,
) -> dict[str, Any]:
    joints = _joint_values([float(value) for value in joint_position_rad])
    gripper = float(gripper_position_m)
    if not math.isfinite(gripper):
        raise ValueError("采集零位夹爪位置必须为有限数值")
    payload = {
        "schema_version": 1,
        "interface": interface,
        "joint_position_rad": list(joints),
        "gripper_position_m": gripper,
        "calibrated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "method": "first_collection_prepare",
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{time.time_ns()}.tmp")
    try:
        temporary.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)
    return {**payload, "joint_position_rad": joints}
