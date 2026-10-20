"""T_a_b maps points expressed in b into a. All translations are metres."""

import cv2
import numpy as np


def transform(value):
    value = np.asarray(value, dtype=float)
    if value.shape != (4, 4) or not np.isfinite(value).all():
        raise ValueError("Expected a finite 4x4 rigid transform")
    r = value[:3, :3]
    if (not np.allclose(value[3], [0, 0, 0, 1], atol=1e-8, rtol=0)
            or not np.allclose(r.T @ r, np.eye(3), atol=1e-6, rtol=0)
            or not np.isclose(np.linalg.det(r), 1, atol=1e-6, rtol=0)):
        raise ValueError("Invalid SE(3) transform")
    return value.copy()


def inverse(value):
    value = transform(value)
    result = np.eye(4)
    result[:3, :3] = value[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ value[:3, 3]
    return result


def from_rt(rotation, translation):
    value = np.eye(4)
    value[:3, :3] = rotation
    value[:3, 3] = np.asarray(translation).reshape(3)
    return transform(value)


def mean_pose(values):
    values = [transform(v) for v in values]
    if not values:
        raise ValueError("No poses to average")
    u, _, vt = np.linalg.svd(sum(v[:3, :3] for v in values))
    r = u @ np.diag([1, 1, np.linalg.det(u @ vt)]) @ vt
    return from_rt(r, np.mean([v[:3, 3] for v in values], axis=0))


def pose_error(first, second):
    first, second = transform(first), transform(second)
    angle = np.linalg.norm(cv2.Rodrigues(first[:3, :3].T @ second[:3, :3])[0])
    return float(np.linalg.norm(first[:3, 3] - second[:3, 3])), float(np.degrees(angle))
