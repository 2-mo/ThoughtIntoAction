"""Pose conversions used by the ARX5 GR00T embodiment."""

from __future__ import annotations

import math

import numpy as np


def rpy_to_matrix(value: np.ndarray) -> np.ndarray:
    """Convert fixed-axis roll, pitch, yaw radians to a rotation matrix."""
    rpy = np.asarray(value, dtype=np.float64)
    if rpy.shape[-1:] != (3,) or not np.all(np.isfinite(rpy)):
        raise ValueError("RPY must end in three finite values")
    roll, pitch, yaw = np.moveaxis(rpy, -1, 0)
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    result = np.empty((*rpy.shape[:-1], 3, 3), dtype=np.float64)
    result[..., 0, 0] = cy * cp
    result[..., 0, 1] = cy * sp * sr - sy * cr
    result[..., 0, 2] = cy * sp * cr + sy * sr
    result[..., 1, 0] = sy * cp
    result[..., 1, 1] = sy * sp * sr + cy * cr
    result[..., 1, 2] = sy * sp * cr - cy * sr
    result[..., 2, 0] = -sp
    result[..., 2, 1] = cp * sr
    result[..., 2, 2] = cp * cr
    return result


def rpy_to_rot6d(value: np.ndarray) -> np.ndarray:
    """Return GR00T's first two rotation-matrix rows, flattened."""
    matrix = rpy_to_matrix(value)
    return matrix[..., :2, :].reshape(*matrix.shape[:-2], 6)


def pose_6d_to_eef_9d(value: np.ndarray) -> np.ndarray:
    """Convert ARX5 ``[xyz, fixed-axis rpy]`` to ``[xyz, rotation-6D]``."""
    pose = np.asarray(value, dtype=np.float64)
    if pose.shape[-1:] != (6,) or not np.all(np.isfinite(pose)):
        raise ValueError("pose_6d must end in six finite values")
    return np.concatenate((pose[..., :3], rpy_to_rot6d(pose[..., 3:])), axis=-1)


def unwrap_rpy_series(value: np.ndarray) -> np.ndarray:
    """Unwrap each RPY channel before temporal interpolation."""
    rpy = np.asarray(value, dtype=np.float64)
    if rpy.ndim != 2 or rpy.shape[1] != 3 or not np.all(np.isfinite(rpy)):
        raise ValueError("RPY series must have shape (N, 3)")
    return np.unwrap(rpy, period=2.0 * math.pi, axis=0)
