"""Strict loading for offline robot-model configuration outside core."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import cast

import yaml

from robot_teleop.kinematics.models import JointLimits, RobotModel


class ModelConfigurationError(ValueError):
    """Raised for invalid or unsafe offline model metadata."""


@dataclass(frozen=True, slots=True)
class KinematicModelConfig:
    """Offline model and deterministic seed used for feasibility checks."""

    model: RobotModel
    reference_joint_position: tuple[float, ...]
    maximum_branch_step_rad: float

    def __post_init__(self) -> None:
        reference = self.model.joint_limits.validate_position(
            self.reference_joint_position
        )
        branch_step = float(self.maximum_branch_step_rad)
        if not math.isfinite(branch_step) or branch_step <= 0.0:
            raise ValueError("maximum_branch_step_rad must be positive and finite")
        object.__setattr__(self, "reference_joint_position", reference)
        object.__setattr__(self, "maximum_branch_step_rad", branch_step)


def load_model_config(path: Path) -> KinematicModelConfig:
    """Load one non-hardware robot model descriptor from a dedicated YAML file."""

    try:
        loaded: object = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ModelConfigurationError(
            f"could not load model config {path}: {exc}"
        ) from exc
    root = _mapping(loaded, "config")
    robot = _section(root, "robot")
    if _boolean(robot, "hardware_enabled"):
        raise ModelConfigurationError(
            "offline model config must keep hardware_enabled false"
        )
    model_raw = _section(root, "model")
    source = _section(model_raw, "source")
    limits = _section(root, "joint_limits")
    offline_check = _section(root, "offline_check")
    urdf = _path(path.parent, _text(model_raw, "urdf"))
    if not urdf.is_file():
        raise ModelConfigurationError(f"URDF asset does not exist: {urdf}")
    model = RobotModel(
        name=_text(model_raw, "name"),
        urdf_path=urdf,
        base_frame=_text(model_raw, "base_frame"),
        end_effector_frame=_text(model_raw, "end_effector_frame"),
        joint_names=_strings(model_raw, "joint_names"),
        joint_limits=JointLimits(
            lower_rad=_numbers(limits, "position_lower_rad"),
            upper_rad=_numbers(limits, "position_upper_rad"),
            velocity_max_rad_s=_numbers(limits, "velocity_max_rad_s"),
        ),
        source_repository=_text(source, "repository"),
        source_commit=_text(source, "commit"),
        source_license=_text(source, "license"),
    )
    return KinematicModelConfig(
        model=model,
        reference_joint_position=_numbers(
            offline_check,
            "reference_joint_position_rad",
        ),
        maximum_branch_step_rad=_number(
            offline_check,
            "maximum_branch_step_rad",
        ),
    )


def _mapping(value: object, name: str) -> dict[str, object]:
    if not isinstance(value, dict) or not all(isinstance(key, str) for key in value):
        raise ModelConfigurationError(f"{name} must be a string-keyed mapping")
    return cast(dict[str, object], value)


def _section(parent: dict[str, object], key: str) -> dict[str, object]:
    if key not in parent:
        raise ModelConfigurationError(f"missing configuration section: {key}")
    return _mapping(parent[key], key)


def _text(parent: dict[str, object], key: str) -> str:
    value = parent.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ModelConfigurationError(f"{key} must be a non-empty string")
    return value.strip()


def _boolean(parent: dict[str, object], key: str) -> bool:
    value = parent.get(key)
    if not isinstance(value, bool):
        raise ModelConfigurationError(f"{key} must be a boolean")
    return value


def _number(parent: dict[str, object], key: str) -> float:
    value = parent.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ModelConfigurationError(f"{key} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise ModelConfigurationError(f"{key} must be finite")
    return result


def _numbers(parent: dict[str, object], key: str) -> tuple[float, ...]:
    values = parent.get(key)
    if not isinstance(values, (list, tuple)) or not values:
        raise ModelConfigurationError(f"{key} must be a non-empty number sequence")
    result: list[float] = []
    for value in values:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ModelConfigurationError(f"{key} must contain only numbers")
        number = float(value)
        if not math.isfinite(number):
            raise ModelConfigurationError(f"{key} must contain finite values")
        result.append(number)
    return tuple(result)


def _strings(parent: dict[str, object], key: str) -> tuple[str, ...]:
    values = parent.get(key)
    if not isinstance(values, (list, tuple)) or not values:
        raise ModelConfigurationError(f"{key} must be a non-empty string sequence")
    result = tuple(value.strip() for value in values if isinstance(value, str))
    if len(result) != len(values) or any(not value for value in result):
        raise ModelConfigurationError(f"{key} must contain non-empty strings")
    return result


def _path(config_directory: Path, value: str) -> Path:
    candidate = Path(value)
    if not candidate.is_absolute():
        candidate = config_directory / candidate
    return candidate.resolve()
