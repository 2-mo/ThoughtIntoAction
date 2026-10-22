"""Shared derivative scaling and sampled-trajectory analysis."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


DEFAULT_DERIVATIVE_SCALE_FLOOR = 0.10
MIN_CARTESIAN_WAYPOINTS = 20
MAX_CARTESIAN_WAYPOINTS = 400


@dataclass(frozen=True)
class SampledMotion:
    """Finite-difference dynamics for timestamped joint-space anchors."""

    waypoint_velocities: np.ndarray
    command_velocities: np.ndarray
    interval_accelerations: np.ndarray
    interval_jerks: np.ndarray

    def required_time_scale(
        self,
        *,
        velocity_limit: float | np.ndarray,
        acceleration_limit: float | np.ndarray,
        jerk_limit: float | np.ndarray,
    ) -> float:
        """Return the duration multiplier required to satisfy all limits."""
        size = self.command_velocities.shape[1]
        limits = tuple(
            _limit_vector(value, name, size)
            for value, name in (
                (velocity_limit, "velocity_limit"),
                (acceleration_limit, "acceleration_limit"),
                (jerk_limit, "jerk_limit"),
            )
        )
        velocity_ratio = max(
            _peak_ratio(self.waypoint_velocities, limits[0]),
            _peak_ratio(self.command_velocities, limits[0]),
        )
        acceleration_ratio = _peak_ratio(self.interval_accelerations, limits[1])
        jerk_ratio = _peak_ratio(self.interval_jerks, limits[2])
        return max(
            velocity_ratio,
            math.sqrt(acceleration_ratio),
            jerk_ratio ** (1.0 / 3.0),
        )


def _peak(values: np.ndarray) -> float:
    return float(np.max(np.abs(values))) if values.size else 0.0


def _limit_vector(value: float | np.ndarray, name: str, size: int) -> np.ndarray:
    limits = np.asarray(value, dtype=np.float64)
    if limits.ndim == 0:
        limits = np.full(size, float(limits), dtype=np.float64)
    elif limits.ndim == 1 and limits.size == size:
        limits = limits.copy()
    else:
        raise ValueError(f"{name} must be a scalar or a vector of length {size}")
    if not np.all(np.isfinite(limits)) or np.any(limits <= 0.0):
        raise ValueError(f"{name} must contain positive finite values")
    return limits


def _peak_ratio(values: np.ndarray, limits: np.ndarray) -> float:
    if not values.size:
        return 0.0
    return float(np.max(np.abs(values) / limits))


def derivative_scales(
    velocity_scale: float,
    *,
    floor: float = DEFAULT_DERIVATIVE_SCALE_FLOOR,
) -> tuple[float, float, float]:
    """Scale velocity exactly and bound low-speed acceleration and jerk."""
    if not math.isfinite(velocity_scale) or not 0.0 < velocity_scale <= 1.0:
        raise ValueError("velocity_scale must be in (0, 1]")
    if not math.isfinite(floor) or not 0.0 < floor <= 1.0:
        raise ValueError("derivative scale floor must be in (0, 1]")
    return (
        velocity_scale,
        max(velocity_scale**2, floor),
        max(velocity_scale**3, floor),
    )


def scaled_derivative_limits(
    max_velocity: float,
    max_acceleration: float,
    max_jerk: float,
    velocity_scale: float,
    *,
    floor: float = DEFAULT_DERIVATIVE_SCALE_FLOOR,
) -> tuple[float, float, float]:
    """Apply the shared low-speed scaling policy to physical limits."""
    limits = (max_velocity, max_acceleration, max_jerk)
    if not all(math.isfinite(value) and value > 0.0 for value in limits):
        raise ValueError("maximum derivative limits must be positive and finite")
    scales = derivative_scales(velocity_scale, floor=floor)
    return tuple(limit * scale for limit, scale in zip(limits, scales))


def analyze_sampled_motion(
    positions: np.ndarray,
    timestamps_s: np.ndarray,
    *,
    command_velocities: np.ndarray | None = None,
) -> SampledMotion:
    """Differentiate joint anchors using their real, possibly nonuniform times."""
    positions = np.asarray(positions, dtype=np.float64)
    timestamps = np.asarray(timestamps_s, dtype=np.float64)
    if positions.ndim != 2 or positions.shape[0] < 2:
        raise ValueError("positions must contain at least two joint anchors")
    if timestamps.shape != (positions.shape[0],):
        raise ValueError("timestamps must match the position anchors")
    if not np.all(np.isfinite(positions)) or not np.all(np.isfinite(timestamps)):
        raise ValueError("sampled motion must contain finite values")
    intervals = np.diff(timestamps)
    if np.any(intervals <= 0.0):
        raise ValueError("sampled motion timestamps must be strictly increasing")

    waypoint_velocities = np.diff(positions, axis=0) / intervals[:, None]
    if command_velocities is None:
        command = np.gradient(
            positions,
            timestamps,
            axis=0,
            edge_order=2 if positions.shape[0] > 2 else 1,
        )
        command[0] = 0.0
        command[-1] = 0.0
    else:
        command = np.asarray(command_velocities, dtype=np.float64)
        if command.shape != positions.shape or not np.all(np.isfinite(command)):
            raise ValueError("command velocities must match the position anchors")

    accelerations = np.diff(command, axis=0) / intervals[:, None]
    if len(accelerations) > 1:
        acceleration_intervals = 0.5 * (intervals[:-1] + intervals[1:])
        jerks = np.diff(accelerations, axis=0) / acceleration_intervals[:, None]
    else:
        jerks = np.empty((0, positions.shape[1]), dtype=np.float64)
    return SampledMotion(
        waypoint_velocities=waypoint_velocities,
        command_velocities=command,
        interval_accelerations=accelerations,
        interval_jerks=jerks,
    )
