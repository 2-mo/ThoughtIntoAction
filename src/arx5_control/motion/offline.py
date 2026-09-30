"""Offline jerk-limited joint trajectories for smooth point-to-point motion."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
import ruckig

from .interpolation import resample_quintic_hermite
from .limits import analyze_sampled_motion
from .profiles import TrajectorySample


# Long low-speed moves otherwise create tens of thousands of waypoints. The SDK
# interpolates between time-stamped points and receives each analytic velocity.
MAX_SAMPLED_WAYPOINTS = 4000


@dataclass(frozen=True)
class RuckigJointTrajectory:
    """A fully calculated trajectory that is safe to inspect before sending."""

    duration_s: float
    sample_times_s: np.ndarray
    positions: np.ndarray
    velocities: np.ndarray
    accelerations: np.ndarray

    def samples(self, start_timestamp: float) -> list[TrajectorySample]:
        if not math.isfinite(start_timestamp):
            raise ValueError("start_timestamp must be finite")
        return [
            TrajectorySample(
                timestamp=start_timestamp + float(relative_time),
                position=position.copy(),
                velocity=velocity.copy(),
            )
            for relative_time, position, velocity in zip(
                self.sample_times_s,
                self.positions,
                self.velocities,
            )
        ]


def _finite_vector(value: np.ndarray, name: str, size: int | None = None) -> np.ndarray:
    vector = np.asarray(value, dtype=np.float64).copy()
    if vector.ndim != 1 or (size is not None and vector.size != size):
        raise ValueError(f"{name} has an invalid shape")
    if not np.all(np.isfinite(vector)):
        raise ValueError(f"{name} must contain finite values")
    return vector


def _positive_limit_vector(
    value: float | np.ndarray,
    name: str,
    size: int,
) -> np.ndarray:
    """Normalize a scalar or per-joint derivative limit to one vector."""
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


def plan_ruckig_joint_trajectory(
    start_position: np.ndarray,
    target_position: np.ndarray,
    current_velocity: np.ndarray,
    *,
    current_acceleration: np.ndarray | None = None,
    min_position: np.ndarray,
    max_position: np.ndarray,
    max_velocity: float | np.ndarray,
    max_acceleration: float | np.ndarray,
    max_jerk: float | np.ndarray,
    sample_rate_hz: float,
    minimum_duration_s: float = 0.5,
) -> RuckigJointTrajectory:
    """Calculate and densely sample one synchronized, jerk-limited move."""
    start = _finite_vector(start_position, "start_position")
    target = _finite_vector(target_position, "target_position", start.size)
    velocity = _finite_vector(current_velocity, "current_velocity", start.size)
    acceleration = (
        np.zeros(start.size, dtype=np.float64)
        if current_acceleration is None
        else _finite_vector(
            current_acceleration, "current_acceleration", start.size
        )
    )
    minimum = _finite_vector(min_position, "min_position", start.size)
    maximum = _finite_vector(max_position, "max_position", start.size)
    velocity_limit = _positive_limit_vector(max_velocity, "max_velocity", start.size)
    acceleration_limit = _positive_limit_vector(
        max_acceleration, "max_acceleration", start.size
    )
    jerk_limit = _positive_limit_vector(max_jerk, "max_jerk", start.size)
    # Feedback acceleration is an estimate. Keep it inside the requested
    # envelope so noisy startup samples cannot invalidate Ruckig input.
    acceleration = np.clip(acceleration, -acceleration_limit, acceleration_limit)
    scalars = (sample_rate_hz, minimum_duration_s)
    if not all(math.isfinite(value) and value > 0.0 for value in scalars):
        raise ValueError("sample rate and minimum duration must be positive")
    if np.any(minimum >= maximum):
        raise ValueError("Ruckig position limits are invalid")
    if np.any(start < minimum) or np.any(start > maximum):
        raise ValueError("Ruckig start position is outside the software limits")
    if np.any(target < minimum) or np.any(target > maximum):
        raise ValueError("Ruckig target position is outside the software limits")
    if np.max(np.abs(target - start)) < 1e-9:
        raise ValueError("Ruckig target is already at the current position")

    degrees_of_freedom = start.size
    otg = ruckig.Ruckig(degrees_of_freedom)
    trajectory = ruckig.Trajectory(degrees_of_freedom)
    params = ruckig.InputParameter(degrees_of_freedom)
    params.current_position = start.tolist()
    params.current_velocity = velocity.tolist()
    params.current_acceleration = acceleration.tolist()
    params.target_position = target.tolist()
    params.target_velocity = [0.0] * degrees_of_freedom
    params.target_acceleration = [0.0] * degrees_of_freedom
    params.max_velocity = velocity_limit.tolist()
    params.max_acceleration = acceleration_limit.tolist()
    params.max_jerk = jerk_limit.tolist()
    params.min_position = minimum.tolist()
    params.max_position = maximum.tolist()
    params.minimum_duration = float(minimum_duration_s)
    params.control_interface = ruckig.ControlInterface.Position

    result = otg.calculate(params, trajectory)
    if result not in (ruckig.Result.Working, ruckig.Result.Finished):
        raise ValueError(f"Ruckig trajectory planning failed: {result}")
    duration_s = float(trajectory.duration)
    if not math.isfinite(duration_s) or duration_s <= 0.0:
        raise ValueError("Ruckig returned an invalid trajectory duration")

    sample_count = min(
        MAX_SAMPLED_WAYPOINTS,
        max(2, int(math.ceil(duration_s * sample_rate_hz))),
    )
    sample_times = np.linspace(
        duration_s / sample_count,
        duration_s,
        sample_count,
        dtype=np.float64,
    )
    positions = []
    velocities = []
    accelerations = []
    for sample_time in sample_times:
        position, sample_velocity, acceleration = trajectory.at_time(float(sample_time))
        positions.append(position)
        velocities.append(sample_velocity)
        accelerations.append(acceleration)

    return RuckigJointTrajectory(
        duration_s=duration_s,
        sample_times_s=sample_times,
        positions=np.asarray(positions, dtype=np.float64),
        velocities=np.asarray(velocities, dtype=np.float64),
        accelerations=np.asarray(accelerations, dtype=np.float64),
    )


def _monotone_anchor_velocities(
    positions: np.ndarray,
    times_s: np.ndarray,
) -> np.ndarray:
    """Return smooth interior velocities without overshooting reversals."""
    velocities = np.zeros_like(positions)
    if positions.shape[0] <= 2:
        return velocities
    intervals = np.diff(times_s)
    slopes = np.diff(positions, axis=0) / intervals[:, None]
    for index in range(1, positions.shape[0] - 1):
        before = slopes[index - 1]
        after = slopes[index]
        same_direction = before * after > 0.0
        before_weight = 2.0 * intervals[index] + intervals[index - 1]
        after_weight = intervals[index] + 2.0 * intervals[index - 1]
        harmonic = np.zeros_like(before)
        harmonic[same_direction] = (
            before_weight + after_weight
        ) / (
            before_weight / before[same_direction]
            + after_weight / after[same_direction]
        )
        velocities[index] = 0.5 * harmonic
    return velocities


def plan_blended_joint_trajectory(
    anchor_positions: np.ndarray,
    *,
    max_velocity: float | np.ndarray,
    max_acceleration: float | np.ndarray,
    max_jerk: float | np.ndarray,
    sample_rate_hz: float,
    minimum_duration_s: float = 0.25,
    maximum_duration_s: float = 300.0,
) -> RuckigJointTrajectory:
    """Time-parameterize exact joint anchors as one continuous C2 path."""
    positions = np.asarray(anchor_positions, dtype=np.float64).copy()
    if (
        positions.ndim != 2
        or positions.shape[0] < 2
        or not np.all(np.isfinite(positions))
    ):
        raise ValueError("anchor_positions must contain finite joint vectors")
    velocity_limit = _positive_limit_vector(
        max_velocity, "max_velocity", positions.shape[1]
    )
    acceleration_limit = _positive_limit_vector(
        max_acceleration, "max_acceleration", positions.shape[1]
    )
    jerk_limit = _positive_limit_vector(
        max_jerk, "max_jerk", positions.shape[1]
    )
    scalars = (sample_rate_hz, minimum_duration_s, maximum_duration_s)
    if not all(math.isfinite(value) and value > 0.0 for value in scalars):
        raise ValueError("trajectory timing limits must be positive and finite")
    if minimum_duration_s > maximum_duration_s:
        raise ValueError("minimum_duration_s must not exceed maximum_duration_s")
    deltas = np.diff(positions, axis=0)
    if np.any(np.max(np.abs(deltas), axis=1) < 1e-9):
        raise ValueError("adjacent joint anchors must be distinct")

    segment_durations = np.max(np.abs(deltas) / velocity_limit[None, :], axis=1)
    segment_durations = np.maximum(segment_durations, 1.0 / sample_rate_hz)
    total_duration = float(np.sum(segment_durations))
    if total_duration < minimum_duration_s:
        segment_durations *= minimum_duration_s / total_duration
    times = np.concatenate(([0.0], np.cumsum(segment_durations)))

    for _ in range(12):
        if times[-1] > maximum_duration_s:
            raise ValueError(
                "blended joint trajectory exceeds the maximum duration"
            )
        anchor_velocities = _monotone_anchor_velocities(positions, times)
        anchor_accelerations = np.zeros_like(positions)
        sample_times, sampled_positions, sampled_velocities, sampled_accelerations = (
            resample_quintic_hermite(
                times,
                positions,
                anchor_velocities,
                anchor_accelerations,
                speed_scale=1.0,
                sample_rate_hz=sample_rate_hz,
            )
        )
        dynamics = analyze_sampled_motion(
            sampled_positions,
            sample_times,
            command_velocities=sampled_velocities,
        )
        required_scale = dynamics.required_time_scale(
            velocity_limit=velocity_limit,
            acceleration_limit=acceleration_limit,
            jerk_limit=jerk_limit,
        )
        if required_scale <= 1.0 + 1e-6:
            return RuckigJointTrajectory(
                duration_s=float(sample_times[-1]),
                sample_times_s=sample_times[1:],
                positions=sampled_positions[1:],
                velocities=sampled_velocities[1:],
                accelerations=sampled_accelerations[1:],
            )
        times *= 1.02 * required_scale
    raise ValueError("unable to satisfy blended joint trajectory limits")
