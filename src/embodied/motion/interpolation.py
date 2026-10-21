"""Continuous time scaling for timestamped joint trajectories."""

from __future__ import annotations

import math

import numpy as np


MAX_RESAMPLED_SAMPLES = 4000


def _dense_times_with_anchors(
    times_s: np.ndarray,
    *,
    speed_scale: float,
    sample_rate_hz: float,
    max_samples: int,
) -> np.ndarray:
    """Build a dense time grid that preserves every requested key time."""
    if times_s.size > max_samples:
        raise ValueError("max_samples is too small to preserve trajectory anchors")
    scaled_anchors = times_s / speed_scale
    durations = np.diff(scaled_anchors)
    interval_counts = np.maximum(
        1,
        np.ceil(durations * sample_rate_hz).astype(np.int64),
    )
    interval_budget = max_samples - 1
    if int(np.sum(interval_counts)) > interval_budget:
        interval_counts = np.ones(durations.size, dtype=np.int64)
        remaining = interval_budget - durations.size
        if remaining:
            shares = remaining * durations / float(np.sum(durations))
            extras = np.floor(shares).astype(np.int64)
            interval_counts += extras
            leftover = remaining - int(np.sum(extras))
            if leftover:
                order = np.argsort(-(shares - extras))
                interval_counts[order[:leftover]] += 1
    sampled = [np.asarray([0.0], dtype=np.float64)]
    for start, end, count in zip(
        scaled_anchors[:-1],
        scaled_anchors[1:],
        interval_counts,
    ):
        sampled.append(
            np.linspace(start, end, int(count) + 1, dtype=np.float64)[1:]
        )
    return np.concatenate(sampled)


def resample_cubic_hermite(
    times_s: np.ndarray,
    positions: np.ndarray,
    velocities: np.ndarray,
    *,
    speed_scale: float,
    sample_rate_hz: float,
    max_samples: int = MAX_RESAMPLED_SAMPLES,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Time-scale and densely sample a C1-continuous Hermite trajectory.

    ``times_s`` are canonical timestamps beginning at zero.  The returned
    timestamps include zero and use the same position/velocity units while
    taking ``1 / speed_scale`` times longer.
    """
    times = np.asarray(times_s, dtype=np.float64)
    point_positions = np.asarray(positions, dtype=np.float64)
    point_velocities = np.asarray(velocities, dtype=np.float64)
    if (
        times.ndim != 1
        or times.size < 2
        or point_positions.ndim != 2
        or point_positions.shape[0] != times.size
        or point_velocities.shape != point_positions.shape
        or not np.all(np.isfinite(times))
        or not np.all(np.isfinite(point_positions))
        or not np.all(np.isfinite(point_velocities))
        or not math.isclose(float(times[0]), 0.0, abs_tol=1e-12)
        or np.any(np.diff(times) <= 0.0)
    ):
        raise ValueError("Hermite trajectory arrays have invalid shapes or times")
    if not math.isfinite(speed_scale) or not 0.0 < speed_scale <= 1.0:
        raise ValueError("speed_scale must be in (0, 1]")
    if not math.isfinite(sample_rate_hz) or sample_rate_hz <= 0.0:
        raise ValueError("sample_rate_hz must be positive and finite")
    if not isinstance(max_samples, int) or max_samples < 2:
        raise ValueError("max_samples must be an integer greater than one")

    scaled_times = _dense_times_with_anchors(
        times,
        speed_scale=speed_scale,
        sample_rate_hz=sample_rate_hz,
        max_samples=max_samples,
    )
    canonical_queries = scaled_times * speed_scale
    segment_indices = np.searchsorted(times, canonical_queries, side="right")
    segment_indices = np.clip(segment_indices, 1, times.size - 1)
    before = segment_indices - 1
    after = segment_indices
    interval = times[after] - times[before]
    fraction = (canonical_queries - times[before]) / interval
    fraction = np.clip(fraction, 0.0, 1.0)

    p0 = point_positions[before]
    p1 = point_positions[after]
    v0 = point_velocities[before]
    v1 = point_velocities[after]
    u = fraction[:, None]
    dt = interval[:, None]
    h00 = 2.0 * u**3 - 3.0 * u**2 + 1.0
    h10 = u**3 - 2.0 * u**2 + u
    h01 = -2.0 * u**3 + 3.0 * u**2
    h11 = u**3 - u**2
    sampled_positions = h00 * p0 + h10 * dt * v0 + h01 * p1 + h11 * dt * v1
    canonical_velocity = (
        (6.0 * u**2 - 6.0 * u) * p0
        + (3.0 * u**2 - 4.0 * u + 1.0) * dt * v0
        + (-6.0 * u**2 + 6.0 * u) * p1
        + (3.0 * u**2 - 2.0 * u) * dt * v1
    ) / dt
    sampled_velocities = speed_scale * canonical_velocity

    sampled_positions[0] = point_positions[0]
    sampled_positions[-1] = point_positions[-1]
    sampled_velocities[0] = speed_scale * point_velocities[0]
    sampled_velocities[-1] = speed_scale * point_velocities[-1]
    for index, canonical_time in enumerate(times[1:-1], start=1):
        scaled_time = canonical_time / speed_scale
        matches = np.flatnonzero(
            np.isclose(scaled_times, scaled_time, rtol=0.0, atol=1e-12)
        )
        if matches.size:
            sample_index = int(matches[0])
            sampled_positions[sample_index] = point_positions[index]
            sampled_velocities[sample_index] = (
                speed_scale * point_velocities[index]
            )
    return scaled_times, sampled_positions, sampled_velocities


def resample_quintic_hermite(
    times_s: np.ndarray,
    positions: np.ndarray,
    velocities: np.ndarray,
    accelerations: np.ndarray,
    *,
    speed_scale: float,
    sample_rate_hz: float,
    max_samples: int = MAX_RESAMPLED_SAMPLES,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Time-scale and densely sample a C2-continuous Hermite trajectory."""
    times = np.asarray(times_s, dtype=np.float64)
    point_positions = np.asarray(positions, dtype=np.float64)
    point_velocities = np.asarray(velocities, dtype=np.float64)
    point_accelerations = np.asarray(accelerations, dtype=np.float64)
    if (
        times.ndim != 1
        or times.size < 2
        or point_positions.ndim != 2
        or point_positions.shape[0] != times.size
        or point_velocities.shape != point_positions.shape
        or point_accelerations.shape != point_positions.shape
        or not np.all(np.isfinite(times))
        or not np.all(np.isfinite(point_positions))
        or not np.all(np.isfinite(point_velocities))
        or not np.all(np.isfinite(point_accelerations))
        or not math.isclose(float(times[0]), 0.0, abs_tol=1e-12)
        or np.any(np.diff(times) <= 0.0)
    ):
        raise ValueError("Hermite trajectory arrays have invalid shapes or times")
    if not math.isfinite(speed_scale) or not 0.0 < speed_scale <= 1.0:
        raise ValueError("speed_scale must be in (0, 1]")
    if not math.isfinite(sample_rate_hz) or sample_rate_hz <= 0.0:
        raise ValueError("sample_rate_hz must be positive and finite")
    if not isinstance(max_samples, int) or max_samples < 2:
        raise ValueError("max_samples must be an integer greater than one")

    scaled_times = _dense_times_with_anchors(
        times,
        speed_scale=speed_scale,
        sample_rate_hz=sample_rate_hz,
        max_samples=max_samples,
    )
    canonical_queries = scaled_times * speed_scale
    segment_indices = np.searchsorted(times, canonical_queries, side="right")
    segment_indices = np.clip(segment_indices, 1, times.size - 1)
    before = segment_indices - 1
    after = segment_indices
    interval = times[after] - times[before]
    fraction = np.clip(
        (canonical_queries - times[before]) / interval,
        0.0,
        1.0,
    )

    p0 = point_positions[before]
    p1 = point_positions[after]
    v0 = point_velocities[before]
    v1 = point_velocities[after]
    a0 = point_accelerations[before]
    a1 = point_accelerations[after]
    u = fraction[:, None]
    h = interval[:, None]

    c0 = p0
    c1 = h * v0
    c2 = 0.5 * h**2 * a0
    displacement = p1 - (c0 + c1 + c2)
    velocity_error = h * v1 - (c1 + 2.0 * c2)
    acceleration_error = h**2 * a1 - 2.0 * c2
    c3 = 10.0 * displacement - 4.0 * velocity_error + 0.5 * acceleration_error
    c4 = -15.0 * displacement + 7.0 * velocity_error - acceleration_error
    c5 = 6.0 * displacement - 3.0 * velocity_error + 0.5 * acceleration_error

    sampled_positions = (
        c0 + c1 * u + c2 * u**2 + c3 * u**3 + c4 * u**4 + c5 * u**5
    )
    canonical_velocities = (
        c1
        + 2.0 * c2 * u
        + 3.0 * c3 * u**2
        + 4.0 * c4 * u**3
        + 5.0 * c5 * u**4
    ) / h
    canonical_accelerations = (
        2.0 * c2
        + 6.0 * c3 * u
        + 12.0 * c4 * u**2
        + 20.0 * c5 * u**3
    ) / h**2
    sampled_velocities = speed_scale * canonical_velocities
    sampled_accelerations = speed_scale**2 * canonical_accelerations

    sampled_positions[0] = point_positions[0]
    sampled_positions[-1] = point_positions[-1]
    sampled_velocities[0] = speed_scale * point_velocities[0]
    sampled_velocities[-1] = speed_scale * point_velocities[-1]
    sampled_accelerations[0] = speed_scale**2 * point_accelerations[0]
    sampled_accelerations[-1] = speed_scale**2 * point_accelerations[-1]
    for index, canonical_time in enumerate(times[1:-1], start=1):
        scaled_time = canonical_time / speed_scale
        matches = np.flatnonzero(
            np.isclose(scaled_times, scaled_time, rtol=0.0, atol=1e-12)
        )
        if matches.size:
            sample_index = int(matches[0])
            sampled_positions[sample_index] = point_positions[index]
            sampled_velocities[sample_index] = (
                speed_scale * point_velocities[index]
            )
            sampled_accelerations[sample_index] = (
                speed_scale**2 * point_accelerations[index]
            )
    return (
        scaled_times,
        sampled_positions,
        sampled_velocities,
        sampled_accelerations,
    )


__all__ = [
    "MAX_RESAMPLED_SAMPLES",
    "resample_cubic_hermite",
    "resample_quintic_hermite",
]
