"""Pure interpolation profiles and timestamped trajectory samples."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class TrajectorySample:
    """One joint-space waypoint with an absolute controller timestamp."""

    timestamp: float
    position: np.ndarray
    velocity: np.ndarray | None = None


def cosine_fraction(alpha: float) -> float:
    """Return a zero-slope interpolation fraction for alpha in [0, 1]."""
    if not 0.0 <= alpha <= 1.0:
        raise ValueError("alpha must be in [0, 1]")
    return 0.5 - 0.5 * math.cos(math.pi * alpha)


def smoothstep9_fraction(alpha: float) -> float:
    """Return a fraction with zero velocity, acceleration, and jerk at both ends."""
    if not 0.0 <= alpha <= 1.0:
        raise ValueError("alpha must be in [0, 1]")
    return alpha**5 * (
        126.0
        + alpha * (-420.0 + alpha * (540.0 + alpha * (-315.0 + 70.0 * alpha)))
    )


def smoothstep9_derivatives(alpha):
    """First three derivatives of the normalized ninth-order time law."""
    u = np.asarray(alpha)
    complement = 1. - u
    velocity = 630. * u**4 * complement**4
    acceleration = 2520. * u**3 * complement**3 * (1. - 2. * u)
    jerk = 2520. * u**2 * complement**2 * (3. * (1. - 2. * u)**2 - 2. * u * complement)
    return velocity, acceleration, jerk


def minimum_smoothstep9_duration(
    delta: np.ndarray,
    *,
    velocity_limit: float,
    acceleration_limit: float,
    jerk_limit: float,
) -> float:
    """Minimum duration for a ninth-order profile under three derivative caps."""
    delta = np.asarray(delta, dtype=np.float64)
    if delta.ndim != 1 or not np.all(np.isfinite(delta)):
        raise ValueError("delta must be a finite one-dimensional array")
    limits = (velocity_limit, acceleration_limit, jerk_limit)
    if not all(math.isfinite(value) and value > 0.0 for value in limits):
        raise ValueError("smoothstep derivative limits must be positive and finite")
    excursion = float(np.max(np.abs(delta))) if delta.size else 0.0
    if excursion == 0.0:
        return 0.0

    # Peak normalized derivatives of 126a^5 - 420a^6 + 540a^7 - 315a^8 + 70a^9.
    velocity_duration = 2.4609375 * excursion / velocity_limit
    acceleration_duration = math.sqrt(9.3719762185 * excursion / acceleration_limit)
    jerk_duration = math.cbrt(78.75 * excursion / jerk_limit)
    return max(velocity_duration, acceleration_duration, jerk_duration)


def minimum_cosine_duration(delta: np.ndarray, velocity_limit: float) -> float:
    """Minimum duration required by a cosine segment under a velocity cap."""
    delta = np.asarray(delta, dtype=np.float64)
    if delta.ndim != 1 or not np.all(np.isfinite(delta)):
        raise ValueError("delta must be a finite one-dimensional array")
    if not math.isfinite(velocity_limit) or velocity_limit <= 0.0:
        raise ValueError("velocity_limit must be positive and finite")
    return float(np.max(np.abs(delta)) * math.pi / (2.0 * velocity_limit))


def cosine_segment(
    start: np.ndarray,
    target: np.ndarray,
    start_timestamp: float,
    duration: float,
    rate_hz: float,
) -> list[TrajectorySample]:
    """Generate a dense, monotonic segment, excluding the start sample."""
    start = np.asarray(start, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    if start.ndim != 1 or target.shape != start.shape:
        raise ValueError("start and target must be one-dimensional arrays of equal shape")
    if not np.all(np.isfinite(start)) or not np.all(np.isfinite(target)):
        raise ValueError("start and target must contain only finite values")
    if not math.isfinite(start_timestamp):
        raise ValueError("start_timestamp must be finite")
    if not math.isfinite(duration) or duration <= 0.0:
        raise ValueError("duration must be positive and finite")
    if not math.isfinite(rate_hz) or rate_hz <= 0.0:
        raise ValueError("rate_hz must be positive and finite")

    step_count = max(2, int(math.ceil(duration * rate_hz)))
    delta = target - start
    return [
        TrajectorySample(
            timestamp=start_timestamp + duration * step / step_count,
            position=start + delta * cosine_fraction(step / step_count),
        )
        for step in range(1, step_count + 1)
    ]


def round_trip(
    start: np.ndarray,
    target: np.ndarray,
    start_timestamp: float,
    outward_duration: float,
    hold_duration: float,
    return_duration: float,
    rate_hz: float,
) -> list[TrajectorySample]:
    """Generate a smooth outward, hold, and return trajectory."""
    if not math.isfinite(hold_duration) or hold_duration < 0.0:
        raise ValueError("hold_duration must be non-negative and finite")
    outward = cosine_segment(
        start, target, start_timestamp, outward_duration, rate_hz
    )
    return_start = start_timestamp + outward_duration + hold_duration
    samples = list(outward)
    if hold_duration > 0.0:
        samples.append(
            TrajectorySample(
                timestamp=return_start,
                position=np.asarray(target, dtype=np.float64).copy(),
            )
        )
    samples.extend(
        cosine_segment(target, start, return_start, return_duration, rate_hz)
    )
    return samples


def via_point(
    start: np.ndarray,
    via: np.ndarray,
    target: np.ndarray,
    start_timestamp: float,
    first_duration: float,
    hold_duration: float,
    second_duration: float,
    rate_hz: float,
) -> list[TrajectorySample]:
    """Generate two smooth segments with an optional hold at the via point."""
    if not math.isfinite(hold_duration) or hold_duration < 0.0:
        raise ValueError("hold_duration must be non-negative and finite")
    samples = cosine_segment(start, via, start_timestamp, first_duration, rate_hz)
    second_start = start_timestamp + first_duration + hold_duration
    if hold_duration > 0.0:
        samples.append(
            TrajectorySample(
                timestamp=second_start,
                position=np.asarray(via, dtype=np.float64).copy(),
            )
        )
    samples.extend(
        cosine_segment(via, target, second_start, second_duration, rate_hz)
    )
    return samples
