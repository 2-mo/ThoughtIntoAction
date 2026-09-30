"""Small dependency-free SE(3) helpers with explicit frame conventions."""

from __future__ import annotations

import math
from typing import Iterable

import numpy as np


def project_rotation(value: np.ndarray) -> np.ndarray:
    """Return the closest proper rotation matrix in Frobenius norm."""
    matrix = np.asarray(value, dtype=np.float64).reshape(3, 3)
    u, _, vt = np.linalg.svd(matrix)
    rotation = u @ vt
    if np.linalg.det(rotation) < 0.0:
        u[:, -1] *= -1.0
        rotation = u @ vt
    return rotation


def make_transform(rotation: np.ndarray, translation: Iterable[float]) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = project_rotation(rotation)
    transform[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    if not np.all(np.isfinite(transform)):
        raise ValueError("transform contains NaN or infinity")
    return transform


def validate_transform(value: np.ndarray, *, name: str = "transform") -> np.ndarray:
    transform = np.asarray(value, dtype=np.float64)
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise ValueError(f"{name} must be a finite 4x4 matrix")
    if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-8):
        raise ValueError(f"{name} has an invalid homogeneous row")
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=2e-5):
        raise ValueError(f"{name} rotation is not orthonormal")
    if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=2e-5):
        raise ValueError(f"{name} rotation is not proper")
    return transform.copy()


def invert_transform(value: np.ndarray) -> np.ndarray:
    transform = validate_transform(value)
    inverse = np.eye(4, dtype=np.float64)
    inverse[:3, :3] = transform[:3, :3].T
    inverse[:3, 3] = -inverse[:3, :3] @ transform[:3, 3]
    return inverse


def rpy_to_matrix(value: Iterable[float]) -> np.ndarray:
    """Convert fixed-axis roll, pitch, yaw to Rz(yaw) Ry(pitch) Rx(roll)."""
    roll, pitch, yaw = (float(component) for component in value)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.asarray(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def pose6_to_transform(value: Iterable[float]) -> np.ndarray:
    pose = np.asarray(list(value), dtype=np.float64)
    if pose.shape != (6,) or not np.all(np.isfinite(pose)):
        raise ValueError("pose6 must contain six finite values")
    return make_transform(rpy_to_matrix(pose[3:]), pose[:3])


def quaternion_xyzw_from_matrix(value: np.ndarray) -> np.ndarray:
    rotation = project_rotation(value)
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = 2.0 * math.sqrt(trace + 1.0)
        quaternion = np.asarray(
            [
                (rotation[2, 1] - rotation[1, 2]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
                0.25 * scale,
            ]
        )
    else:
        axis = int(np.argmax(np.diag(rotation)))
        if axis == 0:
            scale = 2.0 * math.sqrt(
                max(
                    1e-16,
                    1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2],
                )
            )
            quaternion = np.asarray(
                [
                    0.25 * scale,
                    (rotation[0, 1] + rotation[1, 0]) / scale,
                    (rotation[0, 2] + rotation[2, 0]) / scale,
                    (rotation[2, 1] - rotation[1, 2]) / scale,
                ]
            )
        elif axis == 1:
            scale = 2.0 * math.sqrt(
                max(
                    1e-16,
                    1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2],
                )
            )
            quaternion = np.asarray(
                [
                    (rotation[0, 1] + rotation[1, 0]) / scale,
                    0.25 * scale,
                    (rotation[1, 2] + rotation[2, 1]) / scale,
                    (rotation[0, 2] - rotation[2, 0]) / scale,
                ]
            )
        else:
            scale = 2.0 * math.sqrt(
                max(
                    1e-16,
                    1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1],
                )
            )
            quaternion = np.asarray(
                [
                    (rotation[0, 2] + rotation[2, 0]) / scale,
                    (rotation[1, 2] + rotation[2, 1]) / scale,
                    0.25 * scale,
                    (rotation[1, 0] - rotation[0, 1]) / scale,
                ]
            )
    quaternion /= np.linalg.norm(quaternion)
    if quaternion[3] < 0.0:
        quaternion *= -1.0
    return quaternion


def matrix_from_quaternion_xyzw(value: Iterable[float]) -> np.ndarray:
    x, y, z, w = np.asarray(list(value), dtype=np.float64)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("quaternion must be finite and nonzero")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def rotation_vector_from_matrix(value: np.ndarray) -> np.ndarray:
    quaternion = quaternion_xyzw_from_matrix(value)
    vector = quaternion[:3]
    sine_half = float(np.linalg.norm(vector))
    if sine_half < 1e-12:
        return np.zeros(3, dtype=np.float64)
    angle = 2.0 * math.atan2(sine_half, float(quaternion[3]))
    return vector * (angle / sine_half)


def rotation_distance_rad(first: np.ndarray, second: np.ndarray) -> float:
    a = quaternion_xyzw_from_matrix(first)
    b = quaternion_xyzw_from_matrix(second)
    dot = max(-1.0, min(1.0, abs(float(np.dot(a, b)))))
    return 2.0 * math.acos(dot)


def average_transforms(values: Iterable[np.ndarray]) -> np.ndarray:
    transforms = [validate_transform(value) for value in values]
    if not transforms:
        raise ValueError("at least one transform is required")
    translations = np.asarray([value[:3, 3] for value in transforms])
    quaternions = np.asarray(
        [quaternion_xyzw_from_matrix(value[:3, :3]) for value in transforms]
    )
    accumulator = sum(np.outer(value, value) for value in quaternions)
    _, eigenvectors = np.linalg.eigh(accumulator)
    quaternion = eigenvectors[:, -1]
    if quaternion[3] < 0.0:
        quaternion *= -1.0
    return make_transform(
        matrix_from_quaternion_xyzw(quaternion),
        np.median(translations, axis=0),
    )


def transform_error(first: np.ndarray, second: np.ndarray) -> tuple[float, float]:
    """Return translation metres and rotation radians between two transforms."""
    a = validate_transform(first)
    b = validate_transform(second)
    translation = float(np.linalg.norm(a[:3, 3] - b[:3, 3]))
    rotation = rotation_distance_rad(a[:3, :3], b[:3, :3])
    return translation, rotation
