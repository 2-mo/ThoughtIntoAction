from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import json
import math
from pathlib import Path
from typing import Any

import numpy as np


def _finite_vector(value: Any, length: int, name: str) -> np.ndarray:
    vector = np.asarray(value, dtype=np.float64)
    if vector.shape != (length,) or not np.all(np.isfinite(vector)):
        raise ValueError(f"{name} must contain {length} finite values")
    return vector.copy()


@dataclass(frozen=True)
class TeleoperationProfile:
    translation_gain: float
    rotation_gain: float
    tcp_linear_velocity_m_s: float
    tcp_angular_velocity_rad_s: float
    joint_velocity_rad_s: float
    joint_acceleration_rad_s2: float
    joint_jerk_rad_s3: float

    def __post_init__(self) -> None:
        if not all(
            math.isfinite(value) and value > 0
            for value in self.__dict__.values()
        ):
            raise ValueError("teleoperation profile values must be positive and finite")
        if self.translation_gain > 1 or self.rotation_gain > 1:
            raise ValueError("teleoperation gains must not exceed 1")


@dataclass(frozen=True)
class WorkspaceBounds:
    minimum_m: np.ndarray
    maximum_m: np.ndarray
    soft_boundary_m: float

    def __post_init__(self) -> None:
        minimum = _finite_vector(self.minimum_m, 3, "workspace minimum")
        maximum = _finite_vector(self.maximum_m, 3, "workspace maximum")
        if np.any(minimum >= maximum):
            raise ValueError("workspace minimum must be below maximum")
        if not math.isfinite(self.soft_boundary_m) or self.soft_boundary_m <= 0:
            raise ValueError("soft boundary must be positive and finite")
        if np.any(2 * self.soft_boundary_m >= maximum - minimum):
            raise ValueError("soft boundary must be less than half each workspace axis")
        object.__setattr__(self, "minimum_m", minimum)
        object.__setattr__(self, "maximum_m", maximum)

    def contains(self, position_m: np.ndarray) -> bool:
        point = _finite_vector(position_m, 3, "TCP position")
        return bool(
            np.all(point >= self.minimum_m) and np.all(point <= self.maximum_m)
        )

    def speed_scale(self, position_m: np.ndarray) -> float:
        point = _finite_vector(position_m, 3, "TCP position")
        if not self.contains(point):
            return 0.0
        clearance = np.minimum(point - self.minimum_m, self.maximum_m - point)
        return float(np.clip(np.min(clearance) / self.soft_boundary_m, 0.0, 1.0))


@dataclass(frozen=True)
class TeleoperationConfig:
    operator_to_base: np.ndarray
    workspace: WorkspaceBounds
    profiles: dict[str, TeleoperationProfile]

    def __post_init__(self) -> None:
        matrix = np.asarray(self.operator_to_base, dtype=np.float64).copy()
        valid_entries = np.all(np.isin(matrix, (-1.0, 0.0, 1.0)))
        valid_axes = (
            matrix.shape == (3, 3)
            and valid_entries
            and np.all(np.sum(np.abs(matrix), axis=0) == 1)
            and np.all(np.sum(np.abs(matrix), axis=1) == 1)
        )
        if not valid_axes:
            raise ValueError("operator_to_base must be a signed permutation matrix")
        if set(self.profiles) != {"fine", "fast"}:
            raise ValueError("teleoperation profiles must contain fine and fast")
        object.__setattr__(self, "operator_to_base", matrix)

    def web_payload(self) -> dict[str, Any]:
        return {
            "operator_to_base": self.operator_to_base.tolist(),
            "workspace": {
                "minimum_m": self.workspace.minimum_m.tolist(),
                "maximum_m": self.workspace.maximum_m.tolist(),
                "soft_boundary_m": self.workspace.soft_boundary_m,
            },
            "profiles": {
                name: profile.__dict__.copy()
                for name, profile in self.profiles.items()
            },
        }


def load_teleoperation_config(path: Path) -> TeleoperationConfig:
    raw = json.loads(path.read_text(encoding="utf-8"))
    workspace = WorkspaceBounds(
        minimum_m=raw["workspace"]["minimum_m"],
        maximum_m=raw["workspace"]["maximum_m"],
        soft_boundary_m=float(raw["workspace"]["soft_boundary_m"]),
    )
    profiles = {
        name: TeleoperationProfile(**values)
        for name, values in raw["profiles"].items()
    }
    return TeleoperationConfig(
        operator_to_base=raw["operator_to_base"],
        workspace=workspace,
        profiles=profiles,
    )


class GripperState(str, Enum):
    OPEN = "open"
    CLOSE = "close"


@dataclass(frozen=True)
class TeleoperationTarget:
    delta_xyz_m: np.ndarray
    delta_orientation_xyzw: np.ndarray
    gripper_state: GripperState

    def __post_init__(self) -> None:
        delta = _finite_vector(self.delta_xyz_m, 3, "teleoperation delta")
        orientation = _finite_vector(
            self.delta_orientation_xyzw, 4, "teleoperation quaternion"
        )
        norm = float(np.linalg.norm(orientation))
        if not 0.95 <= norm <= 1.05:
            raise ValueError("teleoperation quaternion norm must be near one")
        object.__setattr__(self, "delta_xyz_m", delta)
        object.__setattr__(self, "delta_orientation_xyzw", orientation / norm)
        object.__setattr__(self, "gripper_state", GripperState(self.gripper_state))


def _unit_quaternion(value: Any, name: str) -> np.ndarray:
    quaternion = _finite_vector(value, 4, name)
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-9:
        raise ValueError(f"{name} norm must be positive")
    return quaternion / norm


def quaternion_multiply_xyzw(first: np.ndarray, second: np.ndarray) -> np.ndarray:
    ax, ay, az, aw = _unit_quaternion(first, "first quaternion")
    bx, by, bz, bw = _unit_quaternion(second, "second quaternion")
    result = np.array(
        [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ]
    )
    return result / np.linalg.norm(result)


def quaternion_distance_rad(first: np.ndarray, second: np.ndarray) -> float:
    a = _unit_quaternion(first, "first quaternion")
    b = _unit_quaternion(second, "second quaternion")
    return 2.0 * math.acos(float(np.clip(abs(np.dot(a, b)), -1.0, 1.0)))


def quaternion_slerp_xyzw(
    first: np.ndarray, second: np.ndarray, fraction: float
) -> np.ndarray:
    if not math.isfinite(fraction):
        raise ValueError("quaternion interpolation fraction must be finite")
    a = _unit_quaternion(first, "first quaternion")
    b = _unit_quaternion(second, "second quaternion")
    dot = float(np.dot(a, b))
    if dot < 0:
        b = -b
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        result = a + fraction * (b - a)
        return result / np.linalg.norm(result)
    angle = math.acos(dot)
    result = (
        math.sin((1.0 - fraction) * angle) / math.sin(angle) * a
        + math.sin(fraction * angle) / math.sin(angle) * b
    )
    return result / np.linalg.norm(result)


def limit_teleoperation_target_step(
    previous: TeleoperationTarget,
    desired: TeleoperationTarget,
    profile: TeleoperationProfile,
    elapsed_s: float,
) -> TeleoperationTarget:
    elapsed = float(np.clip(elapsed_s, 1.0 / 60.0, 0.20))
    translation_step = desired.delta_xyz_m - previous.delta_xyz_m
    distance = float(np.linalg.norm(translation_step))
    maximum_distance = profile.tcp_linear_velocity_m_s * elapsed
    if distance > maximum_distance:
        translation_step *= maximum_distance / distance
    angle = quaternion_distance_rad(
        previous.delta_orientation_xyzw,
        desired.delta_orientation_xyzw,
    )
    maximum_angle = profile.tcp_angular_velocity_rad_s * elapsed
    fraction = (
        1.0
        if angle <= maximum_angle or angle < 1e-9
        else maximum_angle / angle
    )
    return TeleoperationTarget(
        previous.delta_xyz_m + translation_step,
        quaternion_slerp_xyzw(
            previous.delta_orientation_xyzw,
            desired.delta_orientation_xyzw,
            fraction,
        ),
        desired.gripper_state,
    )
