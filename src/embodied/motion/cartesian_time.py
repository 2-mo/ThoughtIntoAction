"""Scalar Cartesian path time parameterization."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
import ruckig


MAX_PROGRESS_SAMPLES = 4000


@dataclass(frozen=True)
class ScalarProgressTrajectory:
    """Jerk-limited progress ``s(t)`` for a geometric path."""

    duration_s: float
    sample_times_s: np.ndarray
    progress: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray


def plan_scalar_progress(
    path_length_m: float,
    *,
    max_velocity_m_s: float,
    max_acceleration_m_s2: float,
    max_jerk_m_s3: float,
    sample_rate_hz: float,
    minimum_duration_s: float,
) -> ScalarProgressTrajectory:
    """Plan one normalized path coordinate under Cartesian derivative caps."""
    values = (
        path_length_m,
        max_velocity_m_s,
        max_acceleration_m_s2,
        max_jerk_m_s3,
        sample_rate_hz,
        minimum_duration_s,
    )
    if not all(math.isfinite(value) and value > 0.0 for value in values):
        raise ValueError("Cartesian progress limits must be positive and finite")

    otg = ruckig.Ruckig(1)
    trajectory = ruckig.Trajectory(1)
    params = ruckig.InputParameter(1)
    params.current_position = [0.0]
    params.current_velocity = [0.0]
    params.current_acceleration = [0.0]
    params.target_position = [1.0]
    params.target_velocity = [0.0]
    params.target_acceleration = [0.0]
    params.max_velocity = [max_velocity_m_s / path_length_m]
    params.max_acceleration = [max_acceleration_m_s2 / path_length_m]
    params.max_jerk = [max_jerk_m_s3 / path_length_m]
    params.minimum_duration = float(minimum_duration_s)
    params.control_interface = ruckig.ControlInterface.Position

    result = otg.calculate(params, trajectory)
    if result not in (ruckig.Result.Working, ruckig.Result.Finished):
        raise ValueError(f"Cartesian progress planning failed: {result}")
    duration_s = float(trajectory.duration)
    if not math.isfinite(duration_s) or duration_s <= 0.0:
        raise ValueError("Cartesian progress duration is invalid")

    sample_count = min(
        MAX_PROGRESS_SAMPLES,
        max(2, int(math.ceil(duration_s * sample_rate_hz))),
    )
    sample_times = np.linspace(
        duration_s / sample_count,
        duration_s,
        sample_count,
        dtype=np.float64,
    )
    progress: list[float] = []
    velocity: list[float] = []
    acceleration: list[float] = []
    for sample_time in sample_times:
        position, sample_velocity, sample_acceleration = trajectory.at_time(
            float(sample_time)
        )
        progress.append(float(position[0]))
        velocity.append(float(sample_velocity[0] * path_length_m))
        acceleration.append(float(sample_acceleration[0] * path_length_m))

    values = np.asarray(progress, dtype=np.float64)
    values[-1] = 1.0
    for index in range(values.size - 2, -1, -1):
        if values[index] >= values[index + 1]:
            values[index] = np.nextafter(values[index + 1], 0.0)
    if values[0] <= 0.0:
        values[0] = np.nextafter(values[1], 0.0)
    return ScalarProgressTrajectory(
        duration_s=duration_s,
        sample_times_s=sample_times,
        progress=values,
        velocity=np.asarray(velocity, dtype=np.float64),
        acceleration=np.asarray(acceleration, dtype=np.float64),
    )


__all__ = ["MAX_PROGRESS_SAMPLES", "ScalarProgressTrajectory", "plan_scalar_progress"]
