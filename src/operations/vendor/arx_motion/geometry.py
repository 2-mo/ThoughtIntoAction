"""Cartesian geometric paths with circular waypoint blends."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np


def normalize_quaternion_xyzw(value: np.ndarray) -> np.ndarray:
    quaternion = np.asarray(value, dtype=np.float64).copy()
    if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
        raise ValueError("orientation must be a finite XYZW quaternion")
    norm = float(np.linalg.norm(quaternion))
    if norm <= 1e-12:
        raise ValueError("orientation quaternion must not be zero")
    return quaternion / norm


def slerp_quaternion_xyzw(
    start: np.ndarray,
    target: np.ndarray,
    fraction: float,
) -> np.ndarray:
    if not math.isfinite(fraction) or not 0.0 <= fraction <= 1.0:
        raise ValueError("quaternion interpolation fraction must be in [0, 1]")
    first = normalize_quaternion_xyzw(start)
    second = normalize_quaternion_xyzw(target)
    dot = float(np.dot(first, second))
    if dot < 0.0:
        second = -second
        dot = -dot
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        return normalize_quaternion_xyzw(first + fraction * (second - first))
    angle = math.acos(dot)
    sine = math.sin(angle)
    return normalize_quaternion_xyzw(
        math.sin((1.0 - fraction) * angle) / sine * first
        + math.sin(fraction * angle) / sine * second
    )


def quaternion_distance_rad(start: np.ndarray, target: np.ndarray) -> float:
    first = normalize_quaternion_xyzw(start)
    second = normalize_quaternion_xyzw(target)
    dot = abs(float(np.dot(first, second)))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


@dataclass(frozen=True)
class CartesianWaypoint:
    """Absolute base-frame Cartesian target.

    ``blend_radius_m`` applies at this waypoint. Endpoint radii must be zero.
    A missing orientation inherits the previous waypoint orientation.
    """

    position_xyz_m: np.ndarray
    orientation_xyzw: np.ndarray | None = None
    blend_radius_m: float = 0.0
    name: str | None = None

    def __post_init__(self) -> None:
        position = np.asarray(self.position_xyz_m, dtype=np.float64).copy()
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError("waypoint position must be a finite XYZ vector")
        if not math.isfinite(self.blend_radius_m) or self.blend_radius_m < 0.0:
            raise ValueError("waypoint blend radius must be finite and non-negative")
        orientation = self.orientation_xyzw
        if orientation is not None:
            orientation = normalize_quaternion_xyzw(orientation)
        object.__setattr__(self, "position_xyz_m", position)
        object.__setattr__(self, "orientation_xyzw", orientation)


@dataclass(frozen=True)
class CartesianGeometryPath:
    """Arc-length ordered samples of one continuous Cartesian path."""

    positions_xyz_m: np.ndarray
    orientations_xyzw: np.ndarray
    distance_m: np.ndarray
    waypoint_count: int
    has_blends: bool

    def __post_init__(self) -> None:
        positions = np.asarray(self.positions_xyz_m, dtype=np.float64).copy()
        orientations = np.asarray(self.orientations_xyzw, dtype=np.float64).copy()
        distance = np.asarray(self.distance_m, dtype=np.float64).copy()
        if (
            positions.ndim != 2
            or positions.shape[0] < 2
            or positions.shape[1] != 3
            or orientations.shape != (positions.shape[0], 4)
            or distance.shape != (positions.shape[0],)
            or not np.all(np.isfinite(positions))
            or not np.all(np.isfinite(orientations))
            or not np.all(np.isfinite(distance))
            or not math.isclose(float(distance[0]), 0.0, abs_tol=1e-12)
            or np.any(np.diff(distance) <= 0.0)
        ):
            raise ValueError("Cartesian geometry samples are invalid")
        for index in range(orientations.shape[0]):
            orientations[index] = normalize_quaternion_xyzw(orientations[index])
        object.__setattr__(self, "positions_xyz_m", positions)
        object.__setattr__(self, "orientations_xyzw", orientations)
        object.__setattr__(self, "distance_m", distance)

    @property
    def length_m(self) -> float:
        return float(self.distance_m[-1])

    @property
    def progress(self) -> np.ndarray:
        return self.distance_m / self.length_m

    def sample(self, progress: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Interpolate positions and orientations at normalized arc progress."""
        values = np.asarray(progress, dtype=np.float64)
        if (
            values.ndim != 1
            or values.size < 1
            or not np.all(np.isfinite(values))
            or np.any(values < 0.0)
            or np.any(values > 1.0)
        ):
            raise ValueError("path progress must be a finite vector in [0, 1]")
        target_distance = values * self.length_m
        indices = np.searchsorted(self.distance_m, target_distance, side="right")
        indices = np.clip(indices, 1, self.distance_m.size - 1)
        before = indices - 1
        intervals = self.distance_m[indices] - self.distance_m[before]
        fractions = (target_distance - self.distance_m[before]) / intervals
        positions = (
            self.positions_xyz_m[before]
            + fractions[:, None]
            * (self.positions_xyz_m[indices] - self.positions_xyz_m[before])
        )
        orientations = np.asarray(
            [
                slerp_quaternion_xyzw(
                    self.orientations_xyzw[first],
                    self.orientations_xyzw[second],
                    float(fraction),
                )
                for first, second, fraction in zip(before, indices, fractions)
            ],
            dtype=np.float64,
        )
        at_start = values <= 0.0
        at_end = values >= 1.0
        positions[at_start] = self.positions_xyz_m[0]
        orientations[at_start] = self.orientations_xyzw[0]
        positions[at_end] = self.positions_xyz_m[-1]
        orientations[at_end] = self.orientations_xyzw[-1]
        return positions, orientations


@dataclass(frozen=True)
class _CornerBlend:
    tangent_in: np.ndarray
    tangent_out: np.ndarray
    center: np.ndarray
    normal: np.ndarray
    radius_m: float
    turn_angle_rad: float
    tangent_distance_m: float


def _resolved_orientations(
    waypoints: Sequence[CartesianWaypoint],
) -> tuple[np.ndarray, ...]:
    resolved: list[np.ndarray] = []
    for index, waypoint in enumerate(waypoints):
        orientation = waypoint.orientation_xyzw
        if orientation is None:
            if not resolved:
                raise ValueError("first Cartesian waypoint requires an orientation")
            orientation = resolved[-1]
        resolved.append(normalize_quaternion_xyzw(orientation))
    return tuple(resolved)


def _corner_blend(
    previous: np.ndarray,
    corner: np.ndarray,
    following: np.ndarray,
    radius_m: float,
) -> _CornerBlend | None:
    if radius_m <= 0.0:
        return None
    incoming_vector = corner - previous
    outgoing_vector = following - corner
    incoming_length = float(np.linalg.norm(incoming_vector))
    outgoing_length = float(np.linalg.norm(outgoing_vector))
    if incoming_length <= 1e-9 or outgoing_length <= 1e-9:
        raise ValueError("Cartesian waypoints must not repeat")
    incoming = incoming_vector / incoming_length
    outgoing = outgoing_vector / outgoing_length
    turn = math.acos(max(-1.0, min(1.0, float(np.dot(incoming, outgoing)))))
    if turn <= 1e-6:
        return None
    if turn >= math.pi - 1e-4:
        raise ValueError("180-degree Cartesian corners cannot be blended")
    tangent_distance = radius_m * math.tan(0.5 * turn)
    if tangent_distance >= min(incoming_length, outgoing_length) - 1e-9:
        raise ValueError("blend radius is too large for its adjacent segments")
    tangent_in = corner - incoming * tangent_distance
    tangent_out = corner + outgoing * tangent_distance
    interior_angle = math.pi - turn
    bisector = -incoming + outgoing
    bisector_norm = float(np.linalg.norm(bisector))
    if bisector_norm <= 1e-12:
        raise ValueError("Cartesian blend corner is degenerate")
    center = corner + bisector / bisector_norm * (
        radius_m / math.sin(0.5 * interior_angle)
    )
    start_radius = tangent_in - center
    end_radius = tangent_out - center
    normal = np.cross(start_radius, end_radius)
    normal_norm = float(np.linalg.norm(normal))
    if normal_norm <= 1e-12:
        raise ValueError("Cartesian blend plane is degenerate")
    return _CornerBlend(
        tangent_in=tangent_in,
        tangent_out=tangent_out,
        center=center,
        normal=normal / normal_norm,
        radius_m=float(radius_m),
        turn_angle_rad=turn,
        tangent_distance_m=tangent_distance,
    )


def build_cartesian_path(
    waypoints: Sequence[CartesianWaypoint],
    *,
    max_step_m: float = 0.003,
    max_orientation_step_rad: float = math.radians(2.0),
) -> CartesianGeometryPath:
    """Build line and circular-arc geometry with non-overlapping blends."""
    points = tuple(waypoints)
    if len(points) < 2:
        raise ValueError("Cartesian path needs at least two waypoints")
    if not math.isfinite(max_step_m) or max_step_m <= 0.0:
        raise ValueError("Cartesian path step must be positive and finite")
    if (
        not math.isfinite(max_orientation_step_rad)
        or max_orientation_step_rad <= 0.0
    ):
        raise ValueError("Cartesian orientation step must be positive and finite")
    if points[0].blend_radius_m > 0.0 or points[-1].blend_radius_m > 0.0:
        raise ValueError("Cartesian path endpoint blend radii must be zero")

    positions = tuple(point.position_xyz_m for point in points)
    orientations = _resolved_orientations(points)
    segment_lengths = tuple(
        float(np.linalg.norm(end - start))
        for start, end in zip(positions[:-1], positions[1:])
    )
    if any(length <= 1e-9 for length in segment_lengths):
        raise ValueError("Cartesian waypoints must not repeat")
    blends: list[_CornerBlend | None] = [None]
    blends.extend(
        _corner_blend(
            positions[index - 1],
            positions[index],
            positions[index + 1],
            points[index].blend_radius_m,
        )
        for index in range(1, len(points) - 1)
    )
    blends.append(None)
    for segment_index, segment_length in enumerate(segment_lengths):
        used_start = (
            0.0
            if blends[segment_index] is None
            else blends[segment_index].tangent_distance_m
        )
        used_end = (
            0.0
            if blends[segment_index + 1] is None
            else blends[segment_index + 1].tangent_distance_m
        )
        if used_start + used_end >= segment_length - 1e-9:
            raise ValueError("adjacent Cartesian blends overlap")

    sampled_positions: list[np.ndarray] = [positions[0].copy()]
    sampled_orientations: list[np.ndarray] = [orientations[0].copy()]
    sampled_distance: list[float] = [0.0]

    def append_curve(
        curve_positions: Sequence[np.ndarray],
        start_orientation: np.ndarray,
        end_orientation: np.ndarray,
        length_m: float,
    ) -> None:
        start_distance = sampled_distance[-1]
        count = len(curve_positions) - 1
        for index in range(1, count + 1):
            fraction = index / count
            sampled_positions.append(np.asarray(curve_positions[index]).copy())
            sampled_orientations.append(
                slerp_quaternion_xyzw(
                    start_orientation,
                    end_orientation,
                    fraction,
                )
            )
            sampled_distance.append(start_distance + length_m * fraction)

    for segment_index, segment_length in enumerate(segment_lengths):
        start_blend = blends[segment_index]
        end_blend = blends[segment_index + 1]
        start_distance = (
            0.0 if start_blend is None else start_blend.tangent_distance_m
        )
        end_distance = (
            segment_length
            if end_blend is None
            else segment_length - end_blend.tangent_distance_m
        )
        line_start = (
            positions[segment_index]
            if start_blend is None
            else start_blend.tangent_out
        )
        line_end = (
            positions[segment_index + 1]
            if end_blend is None
            else end_blend.tangent_in
        )
        start_fraction = start_distance / segment_length
        end_fraction = end_distance / segment_length
        line_start_orientation = slerp_quaternion_xyzw(
            orientations[segment_index],
            orientations[segment_index + 1],
            start_fraction,
        )
        line_end_orientation = slerp_quaternion_xyzw(
            orientations[segment_index],
            orientations[segment_index + 1],
            end_fraction,
        )
        line_length = end_distance - start_distance
        line_orientation_angle = quaternion_distance_rad(
            line_start_orientation,
            line_end_orientation,
        )
        line_count = max(
            1,
            int(math.ceil(line_length / max_step_m)),
            int(math.ceil(line_orientation_angle / max_orientation_step_rad)),
        )
        line_points = [
            line_start + (line_end - line_start) * (index / line_count)
            for index in range(line_count + 1)
        ]
        append_curve(
            line_points,
            line_start_orientation,
            line_end_orientation,
            line_length,
        )

        if end_blend is None:
            continue
        next_length = segment_lengths[segment_index + 1]
        arc_end_orientation = slerp_quaternion_xyzw(
            orientations[segment_index + 1],
            orientations[segment_index + 2],
            end_blend.tangent_distance_m / next_length,
        )
        arc_length = end_blend.radius_m * end_blend.turn_angle_rad
        arc_orientation_angle = quaternion_distance_rad(
            line_end_orientation,
            arc_end_orientation,
        )
        arc_count = max(
            2,
            int(math.ceil(arc_length / max_step_m)),
            int(math.ceil(arc_orientation_angle / max_orientation_step_rad)),
        )
        start_radius = end_blend.tangent_in - end_blend.center
        normal = end_blend.normal
        arc_points = []
        for index in range(arc_count + 1):
            angle = end_blend.turn_angle_rad * (index / arc_count)
            rotated = (
                start_radius * math.cos(angle)
                + np.cross(normal, start_radius) * math.sin(angle)
                + normal * np.dot(normal, start_radius) * (1.0 - math.cos(angle))
            )
            arc_points.append(end_blend.center + rotated)
        arc_points[-1] = end_blend.tangent_out.copy()
        append_curve(
            arc_points,
            line_end_orientation,
            arc_end_orientation,
            arc_length,
        )

    sampled_positions[-1] = positions[-1].copy()
    sampled_orientations[-1] = orientations[-1].copy()
    return CartesianGeometryPath(
        positions_xyz_m=np.asarray(sampled_positions, dtype=np.float64),
        orientations_xyzw=np.asarray(sampled_orientations, dtype=np.float64),
        distance_m=np.asarray(sampled_distance, dtype=np.float64),
        waypoint_count=len(points),
        has_blends=any(blend is not None for blend in blends),
    )


def build_circular_path(
    start: CartesianWaypoint,
    via: CartesianWaypoint,
    target: CartesianWaypoint,
    *,
    max_step_m: float = 0.003,
    max_orientation_step_rad: float = math.radians(2.0),
) -> CartesianGeometryPath:
    """Build the unique circular arc from ``start`` through ``via`` to ``target``."""
    points = (start, via, target)
    if not all(isinstance(point, CartesianWaypoint) for point in points):
        raise TypeError("circular path requires three CartesianWaypoint values")
    if not math.isfinite(max_step_m) or max_step_m <= 0.0:
        raise ValueError("Cartesian path step must be positive and finite")
    if (
        not math.isfinite(max_orientation_step_rad)
        or max_orientation_step_rad <= 0.0
    ):
        raise ValueError("Cartesian orientation step must be positive and finite")
    if any(point.blend_radius_m > 0.0 for point in points):
        raise ValueError("explicit circular path does not accept waypoint blends")

    first, middle, last = (point.position_xyz_m for point in points)
    first_delta = middle - first
    second_delta = last - first
    normal = np.cross(first_delta, second_delta)
    normal_squared = float(np.dot(normal, normal))
    if normal_squared <= 1e-16:
        raise ValueError("circular path points must be distinct and non-collinear")

    center = first + (
        np.cross(normal, first_delta) * float(np.dot(second_delta, second_delta))
        + np.cross(second_delta, normal) * float(np.dot(first_delta, first_delta))
    ) / (2.0 * normal_squared)
    radius_vector = first - center
    radius_m = float(np.linalg.norm(radius_vector))
    if not math.isfinite(radius_m) or radius_m <= 1e-9:
        raise ValueError("circular path radius is degenerate")
    unit_x = radius_vector / radius_m
    unit_normal = normal / math.sqrt(normal_squared)
    unit_y = np.cross(unit_normal, unit_x)

    def positive_angle(position: np.ndarray) -> float:
        radial = position - center
        return math.atan2(
            float(np.dot(radial, unit_y)),
            float(np.dot(radial, unit_x)),
        ) % (2.0 * math.pi)

    via_angle = positive_angle(middle)
    target_angle = positive_angle(last)
    if via_angle <= 1e-9:
        raise ValueError("circular via point is too close to the start")
    if target_angle <= via_angle + 1e-9:
        target_angle += 2.0 * math.pi
    if target_angle >= 2.0 * math.pi - 1e-9:
        raise ValueError("full-circle paths are not supported")

    orientations = _resolved_orientations(points)
    orientation_angle = quaternion_distance_rad(orientations[0], orientations[2])
    arc_length = radius_m * target_angle
    sample_count = max(
        2,
        int(math.ceil(arc_length / max_step_m)),
        int(math.ceil(orientation_angle / max_orientation_step_rad)),
    )
    fractions = np.linspace(0.0, 1.0, sample_count + 1, dtype=np.float64)
    angles = fractions * target_angle
    positions = center + radius_m * (
        np.cos(angles)[:, None] * unit_x
        + np.sin(angles)[:, None] * unit_y
    )
    positions[0] = first
    positions[-1] = last
    sampled_orientations = np.asarray(
        [
            slerp_quaternion_xyzw(
                orientations[0],
                orientations[2],
                float(fraction),
            )
            for fraction in fractions
        ],
        dtype=np.float64,
    )
    return CartesianGeometryPath(
        positions_xyz_m=positions,
        orientations_xyzw=sampled_orientations,
        distance_m=fractions * arc_length,
        waypoint_count=3,
        has_blends=False,
    )


def concatenate_cartesian_paths(
    paths: Sequence[CartesianGeometryPath],
) -> CartesianGeometryPath:
    """Join contiguous sampled paths without duplicating shared endpoints."""
    values = tuple(paths)
    if not values or not all(isinstance(path, CartesianGeometryPath) for path in values):
        raise ValueError("Cartesian path concatenation requires geometry paths")
    positions = [values[0].positions_xyz_m]
    orientations = [values[0].orientations_xyzw]
    distances = [values[0].distance_m]
    total_distance = values[0].length_m
    waypoint_count = values[0].waypoint_count
    for path in values[1:]:
        if not np.allclose(
            positions[-1][-1],
            path.positions_xyz_m[0],
            atol=1e-9,
            rtol=0.0,
        ) or quaternion_distance_rad(
            orientations[-1][-1],
            path.orientations_xyzw[0],
        ) > 1e-8:
            raise ValueError("Cartesian geometry paths are not contiguous")
        positions.append(path.positions_xyz_m[1:])
        orientations.append(path.orientations_xyzw[1:])
        distances.append(total_distance + path.distance_m[1:])
        total_distance += path.length_m
        waypoint_count += path.waypoint_count - 1
    return CartesianGeometryPath(
        positions_xyz_m=np.vstack(positions),
        orientations_xyzw=np.vstack(orientations),
        distance_m=np.concatenate(distances),
        waypoint_count=waypoint_count,
        has_blends=any(path.has_blends for path in values),
    )


__all__ = [
    "CartesianGeometryPath",
    "CartesianWaypoint",
    "build_cartesian_path",
    "build_circular_path",
    "concatenate_cartesian_paths",
    "normalize_quaternion_xyzw",
    "quaternion_distance_rad",
    "slerp_quaternion_xyzw",
]
