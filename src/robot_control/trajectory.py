"""SDK-free joint target limits and quintic trajectories, shared by control and simulation."""

import numpy as np


def bounded_joint_target(
    q_actual: np.ndarray,
    q_desired: np.ndarray,
    q_min: np.ndarray,
    q_max: np.ndarray,
    max_step_rad: float,
    soft_limit_margin_rad: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Clip an absolute target to soft limits, then rate-limit from feedback."""
    q_actual = np.asarray(q_actual, dtype=float)
    q_desired = np.asarray(q_desired, dtype=float)
    q_min = np.asarray(q_min, dtype=float)
    q_max = np.asarray(q_max, dtype=float)
    if not np.all(np.isfinite(q_desired)):
        raise ValueError("Non-finite X5 joint target")

    midpoint = (q_min + q_max) / 2.0
    soft_min = np.minimum(q_min + soft_limit_margin_rad, midpoint)
    soft_max = np.maximum(q_max - soft_limit_margin_rad, midpoint)
    q_limited = np.clip(q_desired, soft_min, soft_max)
    q_command = q_actual + np.clip(
        q_limited - q_actual, -max_step_rad, max_step_rad
    )
    # If measured feedback is already just outside a hard limit (the observed
    # X5 J4 can report 1.56 while its configured maximum is 1.55), the hard
    # clamp takes precedence and always moves the target inward.
    return np.clip(q_command, q_min, q_max), q_limited


def trajectory(model, start, end, *, speed=0.15, acceleration=0.3, hz=50, margin=0.0):
    """Quintic stop-to-stop interpolation with analytic velocity/acceleration bounds."""
    if not all(np.isfinite(x) and x > 0 for x in (speed, acceleration, hz)):
        raise ValueError("Trajectory limits must be finite and positive")
    start, end = model.joints(start, margin), model.joints(end, margin)
    distance = float(np.max(np.abs(end - start)))
    duration = max(1 / hz, 1.875 * distance / speed,
                   np.sqrt((10 / np.sqrt(3)) * distance / acceleration))
    count = max(2, int(np.ceil(duration * hz)) + 1)
    times = np.arange(count, dtype=float) / hz
    u = times / times[-1]
    blend = 10 * u**3 - 15 * u**4 + 6 * u**5
    return times, start + blend[:, None] * (end - start)
