"""Jacobian diagnostics for Cartesian path safety.

The current ARX SDK exposes FK but not a Jacobian. Numerical differentiation
keeps this module independent from a future Pinocchio backend.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable
import math

import numpy as np


@dataclass(frozen=True)
class JacobianDiagnostics:
    """Numerical Cartesian Jacobian and singularity indicators."""

    jacobian: np.ndarray
    scaled_jacobian: np.ndarray
    singular_values: np.ndarray
    minimum_singular_value: float
    condition_number: float
    manipulability: float


def _finite_pose(forward_kinematics: Callable[[np.ndarray], np.ndarray], q: np.ndarray) -> np.ndarray:
    pose = np.asarray(forward_kinematics(q), dtype=np.float64).copy()
    if pose.shape != (6,) or not np.all(np.isfinite(pose)):
        raise ValueError("forward kinematics returned an invalid pose")
    return pose


def _wrapped_angle(value: np.ndarray) -> np.ndarray:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def numerical_jacobian(
    forward_kinematics: Callable[[np.ndarray], np.ndarray],
    joint_position: np.ndarray,
    *,
    step_rad: float = 1e-5,
    orientation_scale_m: float = 0.10,
) -> JacobianDiagnostics:
    """Estimate a 6D geometric Jacobian around one joint position.

    The rotational rows are scaled by a characteristic length before SVD so
    translational metres and rotational radians have comparable influence.
    """
    q = np.asarray(joint_position, dtype=np.float64).copy()
    if q.ndim != 1 or q.size < 1 or not np.all(np.isfinite(q)):
        raise ValueError("joint_position must be a finite vector")
    if not math.isfinite(step_rad) or step_rad <= 0.0:
        raise ValueError("step_rad must be positive and finite")
    if not math.isfinite(orientation_scale_m) or orientation_scale_m <= 0.0:
        raise ValueError("orientation_scale_m must be positive and finite")

    jacobian = np.empty((6, q.size), dtype=np.float64)
    for joint in range(q.size):
        plus = q.copy()
        minus = q.copy()
        plus[joint] += step_rad
        minus[joint] -= step_rad
        plus_pose = _finite_pose(forward_kinematics, plus)
        minus_pose = _finite_pose(forward_kinematics, minus)
        delta = plus_pose - minus_pose
        delta[3:] = _wrapped_angle(delta[3:])
        jacobian[:, joint] = delta / (2.0 * step_rad)

    if not np.all(np.isfinite(jacobian)):
        raise ValueError("numerical Jacobian contains NaN or infinity")
    return analyze_jacobian(jacobian, orientation_scale_m=orientation_scale_m)


def analyze_jacobian(
    jacobian: np.ndarray,
    *,
    orientation_scale_m: float = 0.10,
) -> JacobianDiagnostics:
    """Calculate singularity metrics for a geometric Jacobian.

    Rows must be ordered as linear XYZ followed by angular XYZ. Rotational
    rows are scaled by a characteristic length before SVD so metres and
    radians contribute on a comparable scale.
    """
    jacobian = np.asarray(jacobian, dtype=np.float64).copy()
    if (
        jacobian.ndim != 2
        or jacobian.shape[0] != 6
        or jacobian.shape[1] < 1
        or not np.all(np.isfinite(jacobian))
    ):
        raise ValueError("Jacobian must be a finite 6xN matrix")
    if not math.isfinite(orientation_scale_m) or orientation_scale_m <= 0.0:
        raise ValueError("orientation_scale_m must be positive and finite")
    scaled = jacobian.copy()
    scaled[3:] *= orientation_scale_m
    singular_values = np.linalg.svd(scaled, compute_uv=False)
    singular_values = np.asarray(singular_values, dtype=np.float64)
    minimum = float(singular_values[-1])
    maximum = float(singular_values[0])
    condition = math.inf if minimum <= 1e-12 else maximum / minimum
    manipulability = float(np.prod(singular_values))
    return JacobianDiagnostics(
        jacobian=jacobian,
        scaled_jacobian=scaled,
        singular_values=singular_values,
        minimum_singular_value=minimum,
        condition_number=condition,
        manipulability=manipulability,
    )


def singularity_speed_scale(
    minimum_singular_value: float,
    *,
    warn_threshold: float,
    stop_threshold: float,
    minimum_scale: float = 0.25,
) -> float:
    """Map a Jacobian singular value to a conservative speed multiplier."""
    values = (
        minimum_singular_value,
        warn_threshold,
        stop_threshold,
        minimum_scale,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("singularity thresholds must be finite")
    if warn_threshold <= stop_threshold or stop_threshold <= 0.0:
        raise ValueError("singularity thresholds must satisfy warn > stop > 0")
    if not 0.0 < minimum_scale <= 1.0:
        raise ValueError("minimum_scale must be in (0, 1]")
    if minimum_singular_value <= stop_threshold:
        return 0.0
    if minimum_singular_value >= warn_threshold:
        return 1.0
    fraction = (minimum_singular_value - stop_threshold) / (
        warn_threshold - stop_threshold
    )
    return float(minimum_scale + fraction * (1.0 - minimum_scale))


__all__ = [
    "JacobianDiagnostics",
    "analyze_jacobian",
    "numerical_jacobian",
    "singularity_speed_scale",
]
