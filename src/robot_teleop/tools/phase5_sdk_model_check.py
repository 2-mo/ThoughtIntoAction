"""Compare static SDK defaults with the attributed offline X5 model config."""

from __future__ import annotations

import argparse
import json
import math
from collections.abc import Iterable, Sequence
from pathlib import Path
from typing import Any, cast

from robot_teleop.kinematics.config import KinematicModelConfig, load_model_config

from .arx5_sdk_inspect import (
    Arx5SdkInspectionError,
    inspect_arx5_sdk_isolated,
)


class SdkModelMismatchError(RuntimeError):
    """Raised when package defaults differ from the offline X5 configuration."""


def compare_sdk_model_config(
    config: KinematicModelConfig,
    inspection: dict[str, object],
) -> dict[str, object]:
    """Verify the SDK package and offline model share a static joint envelope."""

    model = _mapping(inspection.get("model"), "inspection.model")
    _equal_int(
        "joint_count", model.get("joint_count"), config.model.joint_limits.joint_count
    )
    _equal_text("base_frame", model.get("base_frame"), config.model.base_frame)
    _equal_text(
        "end_effector_frame",
        model.get("end_effector_frame"),
        config.model.end_effector_frame,
    )
    _equal_vector(
        "joint_position_min_rad",
        model.get("joint_position_min_rad"),
        config.model.joint_limits.lower_rad,
    )
    _equal_vector(
        "joint_position_max_rad",
        model.get("joint_position_max_rad"),
        config.model.joint_limits.upper_rad,
    )
    _equal_vector(
        "joint_velocity_max_rad_s",
        model.get("joint_velocity_max_rad_s"),
        config.model.joint_limits.velocity_max_rad_s,
    )
    return {
        "event": "phase5_sdk_model_check",
        "hardware_enabled": False,
        "controller_constructed": False,
        "can_interface_opened": False,
        "matches": True,
        "offline_model": {
            "name": config.model.name,
            "urdf": str(config.model.urdf_path),
            "base_frame": config.model.base_frame,
            "end_effector_frame": config.model.end_effector_frame,
            "joint_count": config.model.joint_limits.joint_count,
        },
        "sdk_model": model,
    }


def _mapping(value: object, name: str) -> dict[str, object]:
    if not isinstance(value, dict) or not all(isinstance(key, str) for key in value):
        raise SdkModelMismatchError(f"{name} must be a string-keyed mapping")
    return cast(dict[str, object], value)


def _equal_int(name: str, actual: object, expected: int) -> None:
    if isinstance(actual, bool) or not isinstance(actual, int) or actual != expected:
        raise SdkModelMismatchError(f"SDK {name} does not match offline model")


def _equal_text(name: str, actual: object, expected: str) -> None:
    if not isinstance(actual, str) or actual != expected:
        raise SdkModelMismatchError(f"SDK {name} does not match offline model")


def _equal_vector(
    name: str,
    actual: object,
    expected: tuple[float, ...],
) -> None:
    if isinstance(actual, (str, bytes)):
        raise SdkModelMismatchError(f"SDK {name} does not match offline model")
    try:
        values = tuple(float(value) for value in cast(Iterable[Any], actual))
    except (TypeError, ValueError) as exc:
        raise SdkModelMismatchError(f"SDK {name} does not match offline model") from exc
    if len(values) != len(expected) or not all(
        math.isfinite(value) and math.isclose(value, expected_value, abs_tol=1e-12)
        for value, expected_value in zip(values, expected, strict=True)
    ):
        raise SdkModelMismatchError(f"SDK {name} does not match offline model")


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="phase5-sdk-model-check")
    parser.add_argument("--config", type=Path, default=Path("configs/phone/robots/arx5.yaml"))
    parser.add_argument("--model", default="X5")
    args = parser.parse_args(argv)
    try:
        config = load_model_config(args.config)
        inspection = inspect_arx5_sdk_isolated(args.model)
        report = compare_sdk_model_config(config, inspection)
    except (Arx5SdkInspectionError, SdkModelMismatchError, ValueError) as exc:
        parser.error(str(exc))
    print(json.dumps(report, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
