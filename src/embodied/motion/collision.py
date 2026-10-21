"""Lightweight primitive collision checks for Cartesian TCP paths.

This is a conservative workspace guard, not a replacement for link-level FCL
collision checking. It is intentionally dependency-free and can later wrap
hpp-fcl behind the same scene interface.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol, Sequence
import math

import numpy as np


class PrimitiveObstacle(Protocol):
    name: str

    def signed_distance(self, points_xyz_m: np.ndarray) -> np.ndarray: ...


@dataclass(frozen=True)
class SphereObstacle:
    center_xyz_m: np.ndarray
    radius_m: float
    name: str = "sphere"

    def __post_init__(self) -> None:
        center = np.asarray(self.center_xyz_m, dtype=np.float64)
        if center.shape != (3,) or not np.all(np.isfinite(center)):
            raise ValueError("sphere center must be a finite XYZ vector")
        if not math.isfinite(self.radius_m) or self.radius_m <= 0.0:
            raise ValueError("sphere radius must be positive and finite")
        object.__setattr__(self, "center_xyz_m", center.copy())

    def signed_distance(self, points_xyz_m: np.ndarray) -> np.ndarray:
        points = np.asarray(points_xyz_m, dtype=np.float64)
        return np.linalg.norm(points - self.center_xyz_m, axis=1) - self.radius_m


@dataclass(frozen=True)
class AabbObstacle:
    minimum_xyz_m: np.ndarray
    maximum_xyz_m: np.ndarray
    name: str = "aabb"

    def __post_init__(self) -> None:
        minimum = np.asarray(self.minimum_xyz_m, dtype=np.float64)
        maximum = np.asarray(self.maximum_xyz_m, dtype=np.float64)
        if (
            minimum.shape != (3,)
            or maximum.shape != (3,)
            or not np.all(np.isfinite(minimum))
            or not np.all(np.isfinite(maximum))
            or np.any(minimum >= maximum)
        ):
            raise ValueError("AABB bounds must be finite and min < max")
        object.__setattr__(self, "minimum_xyz_m", minimum.copy())
        object.__setattr__(self, "maximum_xyz_m", maximum.copy())

    def signed_distance(self, points_xyz_m: np.ndarray) -> np.ndarray:
        points = np.asarray(points_xyz_m, dtype=np.float64)
        outside = np.maximum(self.minimum_xyz_m - points, 0.0) + np.maximum(
            points - self.maximum_xyz_m, 0.0
        )
        outside_distance = np.linalg.norm(outside, axis=1)
        inside_distance = np.min(
            np.stack(
                [
                    points - self.minimum_xyz_m,
                    self.maximum_xyz_m - points,
                ],
                axis=2,
            ),
            axis=(1, 2),
        )
        return np.where(outside_distance > 0.0, outside_distance, -inside_distance)


@dataclass(frozen=True)
class CollisionCheck:
    safe: bool
    minimum_distance_m: float
    obstacle_name: str | None
    waypoint_index: int | None


def _swept_points(path: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Densify each segment so primitive checks cover swept motion."""
    if path.shape[0] == 1:
        return path.copy(), np.zeros(1, dtype=np.int64)
    points: list[np.ndarray] = []
    origins: list[int] = []
    for index, (start, end) in enumerate(zip(path[:-1], path[1:])):
        for fraction in np.linspace(0.0, 1.0, 33, endpoint=False):
            points.append(start + (end - start) * float(fraction))
            origins.append(index)
    points.append(path[-1].copy())
    origins.append(path.shape[0] - 2)
    return np.asarray(points, dtype=np.float64), np.asarray(origins, dtype=np.int64)


def check_tcp_path(
    path_xyz_m: np.ndarray,
    obstacles: Sequence[PrimitiveObstacle],
    *,
    clearance_m: float = 0.0,
) -> CollisionCheck:
    """Check sampled TCP points against primitive obstacles."""
    path = np.asarray(path_xyz_m, dtype=np.float64)
    if path.ndim != 2 or path.shape[1:] != (3,) or path.shape[0] < 1:
        raise ValueError("TCP path must have shape (N, 3)")
    if not np.all(np.isfinite(path)):
        raise ValueError("TCP path must be finite")
    if not math.isfinite(clearance_m) or clearance_m < 0.0:
        raise ValueError("clearance_m must be finite and non-negative")
    swept_path, origins = _swept_points(path)
    minimum_distance = math.inf
    minimum_obstacle: str | None = None
    minimum_index: int | None = None
    for obstacle in obstacles:
        distances = np.asarray(obstacle.signed_distance(swept_path), dtype=np.float64)
        if distances.shape != (swept_path.shape[0],) or not np.all(np.isfinite(distances)):
            raise ValueError("primitive obstacle returned invalid distances")
        index = int(np.argmin(distances))
        value = float(distances[index])
        if value < minimum_distance:
            minimum_distance = value
            minimum_obstacle = str(obstacle.name)
            minimum_index = int(origins[index])
    if minimum_obstacle is None:
        return CollisionCheck(True, math.inf, None, None)
    return CollisionCheck(
        safe=minimum_distance >= clearance_m,
        minimum_distance_m=minimum_distance,
        obstacle_name=minimum_obstacle,
        waypoint_index=minimum_index,
    )


__all__ = [
    "AabbObstacle",
    "CollisionCheck",
    "PrimitiveObstacle",
    "SphereObstacle",
    "check_tcp_path",
]
