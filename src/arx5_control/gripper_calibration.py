"""Persistent, validated gripper readout calibration for ARX X5 control."""

from __future__ import annotations

import json
import math
from pathlib import Path
import time
from typing import Any

from arx5_paths import CALIBRATIONS_ROOT


DEFAULT_GRIPPER_OPEN_READOUT = -3.4
DEFAULT_GRIPPER_CALIBRATION_PATH = CALIBRATIONS_ROOT / "gripper" / "calibration.json"


def validate_gripper_open_readout(value: object) -> float:
    """Validate an SDK motor-angle readout for a fully open gripper."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("夹爪全开电机读数必须是数值")
    readout = float(value)
    if not math.isfinite(readout) or not 0.5 <= abs(readout) <= 10.0:
        raise ValueError("夹爪全开电机读数绝对值必须在 0.5 到 10 rad 之间")
    return readout


def load_gripper_calibration(
    *,
    model: str = "X5",
    interface: str = "can0",
    path: Path = DEFAULT_GRIPPER_CALIBRATION_PATH,
) -> dict[str, Any] | None:
    """Load one persistent calibration record, or ``None`` if absent/invalid.

    Schema 1 records were produced by the vendor closed-zero routine too, so
    they remain valid when upgrading to the explicit one-time calibration
    marker in schema 2.
    """
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, OSError, json.JSONDecodeError):
        return None
    if not isinstance(payload, dict):
        return None
    if payload.get("schema_version") not in {1, 2}:
        return None
    if payload.get("model") != model or payload.get("interface") != interface:
        return None
    try:
        readout = validate_gripper_open_readout(payload.get("gripper_open_readout"))
    except ValueError:
        return None
    return {
        "schema_version": int(payload.get("schema_version", 1)),
        "model": str(payload.get("model", model)),
        "interface": str(payload.get("interface", interface)),
        "gripper_open_readout": readout,
        "calibrated_at": payload.get("calibrated_at"),
        "zero_calibrated": payload.get("zero_calibrated", True) is True,
    }


def load_gripper_open_readout(
    *,
    model: str = "X5",
    interface: str = "can0",
    default: float = DEFAULT_GRIPPER_OPEN_READOUT,
    path: Path = DEFAULT_GRIPPER_CALIBRATION_PATH,
) -> float:
    """Load a matching measured readout, otherwise retain the safe default."""
    fallback = validate_gripper_open_readout(default)
    calibration = load_gripper_calibration(
        model=model,
        interface=interface,
        path=path,
    )
    return fallback if calibration is None else calibration["gripper_open_readout"]


def save_gripper_open_readout(
    readout: object,
    *,
    model: str = "X5",
    interface: str = "can0",
    path: Path = DEFAULT_GRIPPER_CALIBRATION_PATH,
) -> float:
    """Atomically persist one measured full-open motor readout."""
    value = validate_gripper_open_readout(readout)
    payload: dict[str, Any] = {
        "schema_version": 2,
        "model": str(model),
        "interface": str(interface),
        "gripper_open_readout": value,
        "zero_calibrated": True,
        "zero_method": "vendor_calibrate_gripper",
        "calibrated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
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
    return value
