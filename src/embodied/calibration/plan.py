"""Sampling plans around a taught centre pose, and their execution through the kernel.

Plans are offline joint targets checked for limits and pose diversity with the kernel's own model.
Every move is a kernel preview first: a leg the kernel refuses (limits, table plane) is skipped
without moving, and a pose where the target is not seen is skipped without blind retries.
"""
import numpy as np

from embodied.calibration.solver import diversity
from embodied.core.project import PROJECT_ROOT

KERNEL_URDF = PROJECT_ROOT / 'control/models/x5/x5_2025.urdf'
FIT, VALIDATION = 20, 6
MAX_CONSECUTIVE_MISSES = 4


def kernel_model(urdf=KERNEL_URDF):
    """The kernel's URDF with its TCP frame, for offline limits and diversity checks."""
    from embodied.core.model import RobotModel
    return RobotModel(urdf=urdf, eef='tcp_link')


def validate_plan(plan, model):
    if plan.get('kind') != 'hand_eye_plan' or plan.get('robot') != model.metadata():
        raise ValueError('Plan is not for this robot model (URDF hash, frames or joint order differ)')
    center = model.joints(plan['center'], margin=.05)
    rows = plan['waypoints']
    if not 14 <= len(rows) <= 80 or any(row['split'] not in {'fit', 'validation'} for row in rows):
        raise ValueError('Plan requires 14..80 fit/validation waypoints')
    poses = {'fit': [], 'validation': []}
    for row in rows:
        target = model.joints(row['q'], margin=.05)
        if np.max(np.abs(target - center)) > .5:
            raise ValueError('Sampling target >0.5 rad from the centre pose')
        poses[row['split']].append(model.fk(target))
    diversity(poses['fit'])
    diversity(poses['validation'], minimum=4)
    return {'waypoints': len(rows), 'max_offset_rad': float(max(np.max(np.abs(np.asarray(r['q']) - center)) for r in rows))}


def make_plan(model, center, step=.25, seed=20260908):
    """FIT + VALIDATION distinct joint targets within ±step rad of the centre."""
    if not .1 <= step <= .4:
        raise ValueError('Sampling step must be 0.1..0.4 rad')
    center = model.joints(center, margin=.05)
    rng = np.random.default_rng(seed)
    rows = []
    for _ in range(5000):
        q = center + rng.uniform(-step, step, 6)
        try:
            model.joints(q, margin=.05)
        except ValueError:
            continue
        if any(np.linalg.norm(q - row['q']) < .12 for row in rows):
            continue
        rows.append({'q': q.tolist(), 'split': 'fit' if len(rows) < FIT else 'validation'})
        if len(rows) == FIT + VALIDATION:
            break
    plan = {'schema_version': 1, 'kind': 'hand_eye_plan', 'robot': model.metadata(), 'center': center.tolist(),
            'step_rad': step, 'waypoints': rows}
    validate_plan(plan, model)
    return plan


def run_plan(arm, plan, sample, *, speed=.3, log=print):
    """Visit each waypoint, hold, sample, then return to the centre; returns the number of samples.

    `sample(split, index)` raises ValueError to skip a pose. Any other error (kernel fault, Ctrl+C)
    leaves the arm held where it is.
    """
    from embodied.calibration.arm import Refused
    misses = taken = 0
    for index, row in enumerate(plan['waypoints']):
        try:
            arm.move(row['q'], speed)
        except Refused as exc:
            log(f'waypoint {index}: kernel refused the move ({exc}); skipped')
            continue
        arm.hold()
        try:
            sample(row['split'], index)
        except ValueError as exc:
            misses += 1
            log(f'waypoint {index}: {exc}; skipped')
            if misses >= MAX_CONSECUTIVE_MISSES:
                raise RuntimeError(f'{misses} poses in a row without a usable target view; '
                                   'teach a centre pose that sees the target better') from exc
            continue
        misses, taken = 0, taken + 1
    arm.move(plan['center'], speed)
    arm.hold()
    return taken
