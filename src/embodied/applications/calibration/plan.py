"""Sampling plans around a taught centre pose, and their execution through the kernel.

Plans are offline joint targets checked for limits and pose diversity with the kernel's own model.
Every move is a kernel preview first: a leg the kernel refuses (limits, table plane) is skipped
without moving, and a pose where the target is not seen is skipped without blind retries.
"""
from itertools import combinations, product

import cv2
import numpy as np

from embodied.applications.calibration.solver import diversity
from embodied.applications.calibration.target import normalize_pixels, target_spec
from embodied.core.project import PROJECT_ROOT
from embodied.core.transforms import inverse, pose_error, transform

KERNEL_URDF = PROJECT_ROOT / 'assets/robots/x5/runtime/x5_2025.urdf'
FIT, VALIDATION = 20, 6
MAX_CONSECUTIVE_MISSES = 4


def kernel_model(urdf=KERNEL_URDF):
    """The kernel's URDF with its TCP frame, for offline limits and diversity checks."""
    from embodied.devices.robots.arx5.pinocchio_model import X5PinocchioModel
    return X5PinocchioModel(urdf=urdf, eef='tcp_link')


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
    fit = diversity(poses['fit'])
    validation = diversity(poses['validation'], minimum=4)
    if plan.get('view'):
        previous = center
        for row in [*rows, {'q': center}]:
            # shortcut: 11 joint samples screen the view; use trajectory samples for continuous visibility checks.
            if not all(target_visible(model.fk(q), plan['view'])
                       for q in np.linspace(previous, row['q'], 11)):
                raise ValueError('Planned marker/white border leaves the image margin along a MoveJ')
            previous = row['q']
    return {'waypoints': len(rows), 'max_offset_rad': float(max(np.max(np.abs(np.asarray(r['q']) - center)) for r in rows)),
            'fit': fit, 'validation': validation}


def target_visible(tcp, view):
    """Predict the whole printed square, including its white border, in the RGB view."""
    camera = view['camera']
    h = target_spec(view['target']['black_square_size_m'])['black_square_size_m'] * 2 / 3
    points = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]])
    observed = inverse(transform(tcp) @ transform(view['T_eef_camera'])) @ transform(view['T_base_target'])
    points = points @ observed[:3, :3].T + observed[:3, 3]
    if np.any(points[:, 2] <= .05) or observed[2, 2] >= 0:
        return False
    # Keep extra clearance beyond the detector's 8 px black-square gate.
    width, height = camera['width'], camera['height']
    perimeter = np.array([[24, 24], [width - 24, 24], [width - 24, height - 24], [24, height - 24]])
    # shortcut: four undistorted corners approximate curved image edges; sample the perimeter for strong distortion.
    contour = normalize_pixels(perimeter, camera).astype(np.float32)
    return all(cv2.pointPolygonTest(contour, tuple(p[:2] / p[2]), False) >= 0 for p in points)


def make_realman_plan(model, center, step=.25, *, view=None):
    """ARX-style single-axis/paired offsets, selected using RM65 TCP geometry instead of six-axis noise."""
    if not .1 <= step <= .4:
        raise ValueError('Sampling step must be 0.1..0.4 rad')
    center = model.joints(center, margin=.05)
    candidates, poses = [], []
    for count in (1, 2):
        for axes in combinations(range(6), count):
            for offsets in product((-step, -.7 * step, .7 * step, step), repeat=count):
                q = center.copy()
                q[list(axes)] += offsets
                try:
                    q = model.joints(q, margin=.05)
                except ValueError:
                    continue
                pose = model.fk(q)
                if view is not None and not target_visible(pose, view):
                    continue
                if any(distance < .001 and angle < .5 for distance, angle in
                       (pose_error(pose, previous) for previous in poses)):
                    continue
                candidates.append(q)
                poses.append(pose)
    if len(candidates) < FIT + VALIDATION:
        raise ValueError('Not enough visible/distinct RM65 targets; improve the centre view or sampling range')
    remaining = set(range(len(candidates)))

    def select(count):
        distances = np.full(len(candidates), np.inf)
        previous = model.fk(center)
        chosen = []
        for _ in range(count):
            errors = np.asarray([pose_error(previous, pose) for pose in poses]) / [.04, 20.]
            distances = np.minimum(distances, np.sum(errors**2, axis=1))
            index = max(sorted(remaining), key=lambda i: distances[i])
            remaining.remove(index)
            chosen.append(index)
            previous = poses[index]
        return chosen

    # Reserve a diverse validation set first; neither split shares a robot pose.
    validation = select(VALIDATION)
    fit = select(FIT)
    rows, previous = [], center
    for split, indices in (('fit', fit), ('validation', validation)):
        while indices:
            index = min(indices, key=lambda i: np.linalg.norm(candidates[i] - previous))
            indices.remove(index)
            previous = candidates[index]
            rows.append({'q': previous.tolist(), 'split': split})
    plan = {'schema_version': 1, 'kind': 'hand_eye_plan', 'strategy': 'realman_axis_pairs',
            'robot': model.metadata(), 'center': center.tolist(), 'step_rad': step, 'waypoints': rows}
    if view is not None:
        plan['view'] = view
    validate_plan(plan, model)
    return plan


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
    from embodied.applications.calibration.arm import Refused
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
