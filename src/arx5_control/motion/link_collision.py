"""Conservative whole-arm collision checks using hpp-fcl/Coal primitives."""

from __future__ import annotations

from dataclasses import dataclass
import importlib
import math
import threading
from typing import Sequence

import numpy as np

from .collision import AabbObstacle, PrimitiveObstacle, SphereObstacle
from .pinocchio_backend import PinocchioKinematics


@dataclass(frozen=True)
class LinkCapsule:
    """Capsule joining two Pinocchio frames in the kinematic chain."""

    name: str
    start_frame: str
    end_frame: str
    radius_m: float

    def __post_init__(self) -> None:
        if not self.name or not self.start_frame or not self.end_frame:
            raise ValueError("link capsule names must not be empty")
        if not math.isfinite(self.radius_m) or self.radius_m <= 0.0:
            raise ValueError("link capsule radius must be positive and finite")


@dataclass(frozen=True)
class LinkCollisionCheck:
    safe: bool
    minimum_distance_m: float
    first_body: str | None
    second_body: str | None
    waypoint_index: int | None
    collision_type: str | None


def default_x5_capsules() -> tuple[LinkCapsule, ...]:
    """Conservative X5 link envelope independent of unavailable STL files."""
    return (
        LinkCapsule("base", "base_link", "joint1", 0.055),
        LinkCapsule("link1", "joint1", "joint2", 0.045),
        LinkCapsule("link2", "joint2", "joint3", 0.038),
        LinkCapsule("link3", "joint3", "joint4", 0.034),
        LinkCapsule("link4", "joint4", "joint5", 0.036),
        LinkCapsule("link5", "joint5", "joint6", 0.040),
        LinkCapsule("link6_tool", "joint6", "eef_link", 0.045),
    )


def _rotation_from_z(direction: np.ndarray) -> np.ndarray:
    """Return a rotation whose local Z axis follows ``direction``."""
    target = np.asarray(direction, dtype=np.float64)
    norm = float(np.linalg.norm(target))
    if norm <= 1e-12:
        raise ValueError("collision capsule endpoints must be distinct")
    target /= norm
    source = np.asarray([0.0, 0.0, 1.0], dtype=np.float64)
    cosine = float(np.dot(source, target))
    if cosine > 1.0 - 1e-12:
        return np.eye(3, dtype=np.float64)
    if cosine < -1.0 + 1e-12:
        return np.diag([1.0, -1.0, -1.0]).astype(np.float64)
    axis = np.cross(source, target)
    skew = np.asarray(
        [
            [0.0, -axis[2], axis[1]],
            [axis[2], 0.0, -axis[0]],
            [-axis[1], axis[0], 0.0],
        ],
        dtype=np.float64,
    )
    return np.eye(3) + skew + skew @ skew / (1.0 + cosine)


class PinocchioCollisionModel:
    """Coal capsule model with self and primitive-environment checking."""

    def __init__(
        self,
        kinematics: PinocchioKinematics,
        capsules: Sequence[LinkCapsule],
        *,
        obstacles: Sequence[PrimitiveObstacle] = (),
        clearance_m: float = 0.003,
        max_joint_step_rad: float = math.radians(0.5),
    ) -> None:
        try:
            coal = importlib.import_module("coal")
        except ImportError as exc:  # pragma: no cover - dependency-free fallback
            raise RuntimeError("hpp-fcl/Coal is not installed") from exc
        if not math.isfinite(clearance_m) or clearance_m < 0.0:
            raise ValueError("link collision clearance must be non-negative")
        if not math.isfinite(max_joint_step_rad) or max_joint_step_rad <= 0.0:
            raise ValueError("collision joint step must be positive and finite")
        capsules = tuple(capsules)
        if len(capsules) < 2:
            raise ValueError("link collision model needs at least two capsules")
        frame_names = tuple(
            dict.fromkeys(
                name
                for capsule in capsules
                for name in (capsule.start_frame, capsule.end_frame)
            )
        )
        missing = [name for name in frame_names if not kinematics.has_frame(name)]
        if missing:
            raise ValueError(f"collision frames missing from URDF: {', '.join(missing)}")

        neutral = np.zeros(kinematics.joint_dof, dtype=np.float64)
        positions = kinematics.frame_positions(neutral, frame_names)
        objects = []
        for capsule in capsules:
            length = float(
                np.linalg.norm(
                    positions[capsule.end_frame] - positions[capsule.start_frame]
                )
            )
            if length <= 1e-6:
                raise ValueError(f"collision capsule {capsule.name} has zero length")
            geometry = coal.Capsule(float(capsule.radius_m), 0.5 * length)
            objects.append(coal.CollisionObject(geometry))

        environment: list[tuple[str, object]] = []
        for obstacle in obstacles:
            if isinstance(obstacle, SphereObstacle):
                geometry = coal.Sphere(float(obstacle.radius_m))
                collision_object = coal.CollisionObject(
                    geometry,
                    np.eye(3),
                    np.asarray(obstacle.center_xyz_m, dtype=np.float64),
                )
            elif isinstance(obstacle, AabbObstacle):
                minimum = np.asarray(obstacle.minimum_xyz_m, dtype=np.float64)
                maximum = np.asarray(obstacle.maximum_xyz_m, dtype=np.float64)
                geometry = coal.Box(*(maximum - minimum).tolist())
                collision_object = coal.CollisionObject(
                    geometry,
                    np.eye(3),
                    0.5 * (minimum + maximum),
                )
            else:
                continue
            environment.append((str(obstacle.name), collision_object))

        self._coal = coal
        self._kinematics = kinematics
        self._capsules = capsules
        self._frame_names = frame_names
        self._objects = tuple(objects)
        self._environment = tuple(environment)
        self._clearance_m = float(clearance_m)
        self._max_joint_step_rad = float(max_joint_step_rad)
        self._distance_request = coal.DistanceRequest(True)
        self._distance_request.enable_signed_distance = True
        self._lock = threading.RLock()
        self._self_pairs = tuple(
            (first, second)
            for first in range(len(capsules))
            for second in range(first + 2, len(capsules))
        )

    @property
    def backend_name(self) -> str:
        return f"coal-{self._coal.__version__}"

    def _update_objects(self, joint_position: np.ndarray) -> None:
        frames = self._kinematics.frame_positions(
            joint_position,
            self._frame_names,
        )
        for capsule, collision_object in zip(self._capsules, self._objects):
            start = frames[capsule.start_frame]
            end = frames[capsule.end_frame]
            collision_object.setRotation(_rotation_from_z(end - start))
            collision_object.setTranslation(0.5 * (start + end))
            collision_object.computeAABB()

    def _distance(self, first: object, second: object) -> float:
        result = self._coal.DistanceResult()
        value = self._coal.distance(
            first,
            second,
            self._distance_request,
            result,
        )
        return float(value)

    def check_configuration(
        self,
        joint_position: np.ndarray,
        *,
        waypoint_index: int = 0,
    ) -> LinkCollisionCheck:
        with self._lock:
            self._update_objects(joint_position)
            minimum = math.inf
            bodies: tuple[str | None, str | None] = (None, None)
            collision_type: str | None = None
            for first, second in self._self_pairs:
                distance = self._distance(self._objects[first], self._objects[second])
                if distance < minimum:
                    minimum = distance
                    bodies = (
                        self._capsules[first].name,
                        self._capsules[second].name,
                    )
                    collision_type = "self"
            for link_index, collision_object in enumerate(self._objects):
                for obstacle_name, obstacle in self._environment:
                    distance = self._distance(collision_object, obstacle)
                    if distance < minimum:
                        minimum = distance
                        bodies = (self._capsules[link_index].name, obstacle_name)
                        collision_type = "environment"
            if bodies[0] is None:
                return LinkCollisionCheck(True, math.inf, None, None, None, None)
            return LinkCollisionCheck(
                safe=minimum >= self._clearance_m,
                minimum_distance_m=float(minimum),
                first_body=bodies[0],
                second_body=bodies[1],
                waypoint_index=int(waypoint_index),
                collision_type=collision_type,
            )

    def check_path(self, joint_positions: np.ndarray) -> LinkCollisionCheck:
        positions = np.asarray(joint_positions, dtype=np.float64)
        if positions.ndim != 2 or positions.shape[0] < 1:
            raise ValueError("collision joint path must have shape (N, dof)")
        if not np.all(np.isfinite(positions)):
            raise ValueError("collision joint path must be finite")
        minimum_check = LinkCollisionCheck(True, math.inf, None, None, None, None)
        for segment_index in range(max(1, positions.shape[0] - 1)):
            if positions.shape[0] == 1:
                samples = positions
            else:
                start = positions[segment_index]
                end = positions[segment_index + 1]
                count = max(
                    1,
                    int(
                        math.ceil(
                            float(np.max(np.abs(end - start)))
                            / self._max_joint_step_rad
                        )
                    ),
                )
                samples = np.asarray(
                    [
                        start + (end - start) * (step / count)
                        for step in range(count + 1)
                    ]
                )
            for local_index, sample in enumerate(samples):
                if segment_index > 0 and local_index == 0:
                    continue
                check = self.check_configuration(
                    sample,
                    waypoint_index=segment_index,
                )
                if check.minimum_distance_m < minimum_check.minimum_distance_m:
                    minimum_check = check
                if not check.safe:
                    return check
        return minimum_check


__all__ = [
    "LinkCapsule",
    "LinkCollisionCheck",
    "PinocchioCollisionModel",
    "default_x5_capsules",
]
