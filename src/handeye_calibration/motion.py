"""Offline sampling plans and interruptible bounded execution via an injected arm."""

import numpy as np

from robot_control.trajectory import trajectory
from robot_control.motion import move_and_settle
from handeye_calibration.solver import diversity


def validate_plan(plan, robot):
    if plan.get("schema_version") != 1 or plan.get("robot") != robot.metadata():
        raise ValueError("Plan model hash, frames or joint order differs from this robot")
    settings = plan["motion"]
    for key, maximum in (("speed", 0.15), ("acceleration", 0.5)):
        if not 0 < settings[key] <= maximum:
            raise ValueError(f"Calibration {key} must be in (0, {maximum}]")
    if settings["hz"] != 50 or not 0.05 <= settings["margin"] <= 0.2:
        raise ValueError("Use 50 Hz and a joint margin of 0.05..0.2 rad")
    center = robot.joints(plan["center"], settings["margin"])
    bounds = plan.get("eef_bounds_m")
    if bounds is not None:
        bounds = np.asarray(bounds, dtype=float)
        if bounds.shape != (3, 2) or not np.isfinite(bounds).all() or np.any(bounds[:, 0] >= bounds[:, 1]):
            raise ValueError("Invalid EEF workspace bounds")
    rows = plan["waypoints"]
    if not 14 <= len(rows) <= 80 or any(row["split"] not in {"fit", "validation"} for row in rows):
        raise ValueError("Plan requires 14..80 fit/validation waypoints")
    poses = {"fit": [], "validation": []}
    previous = center
    duration = 0.
    for row in rows:
        target = robot.joints(row["q"], settings["margin"])
        if np.max(np.abs(target - center)) > 0.4:
            raise ValueError("Sampling target >0.4 rad from reference")
        times, qs = trajectory(robot, previous, target, **settings)
        for q in qs:
            position = robot.fk(q)[:3, 3]
            if bounds is not None and (np.any(position < bounds[:, 0]) or np.any(position > bounds[:, 1])):
                raise ValueError("Trajectory leaves the configured EEF workspace")
        duration += times[-1]
        poses[row["split"]].append(robot.fk(target))
        previous = target
    diversity(poses["fit"])
    diversity(poses["validation"], minimum=4)
    return {"waypoints": len(rows), "travel_time_s": duration,
            "eef_bounds_checked": bounds is not None,
            "collision_check": "none: EEF bounds do not check links, table, wrist camera or cables"}


def make_plan(robot, center, bounds=None, step=0.3):
    if not 0.1 <= step <= 0.4:
        raise ValueError("Sampling step must be 0.1..0.4 rad")
    center = robot.joints(center, margin=0.05)
    rng = np.random.default_rng(20260908)
    rows = []
    for _ in range(5000):
        q = center + rng.uniform(-step, step, 6)
        try:
            robot.joints(q, margin=0.05)
        except ValueError:
            continue
        if any(np.linalg.norm(q - row["q"]) < 0.12 for row in rows):
            continue
        rows.append({"q": q.tolist(), "split": "fit" if len(rows) < 20 else "validation"})
        if len(rows) == 26:
            break
    plan = {"schema_version": 1, "robot": robot.metadata(), "center": center.tolist(),
            "eef_bounds_m": bounds, "motion": {"speed": 0.15, "acceleration": 0.3,
                                                "hz": 50, "margin": 0.05},
            "waypoints": rows}
    validate_plan(plan, robot)
    return plan


def execute_plan(arm, robot, plan, capture, *, move=move_and_settle):
    """Caller supplies an already-owned arm; cleanup also covers validation failure."""
    try:
        validate_plan(plan, robot)
        if plan.get("eef_bounds_m") is None:
            raise ValueError("Live automatic capture requires measured EEF workspace bounds")
        if np.max(np.abs(np.asarray(arm.snapshot()["q"]) - plan["center"])) > 0.03:
            raise ValueError("Move manually to the previewed reference pose (within 0.03 rad)")
        arm.enable()
        for index, row in enumerate(plan["waypoints"]):
            move(arm, robot, np.asarray(row["q"]), plan["motion"], bounds=plan["eef_bounds_m"])
            # A lost/ambiguous marker aborts; do not continue moving blindly.
            capture(row["split"], index)
    finally:
        arm.damp()
