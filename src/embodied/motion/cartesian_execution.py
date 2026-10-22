"""Cartesian geometry and time bounds for the unified planner, outside the tick loop."""
import math
import hashlib
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation

from embodied.motion.geometry import CartesianWaypoint, build_cartesian_path
from embodied.motion.profiles import smoothstep9_fraction, smoothstep9_derivatives
from embodied.motion.cartesian_time import plan_scalar_progress
from embodied.motion.validation import finite_vector, TCP_UNITS

METRICS = {'tcp_speed': 'peak_tcp_speed_m_s', 'tcp_acceleration': 'peak_tcp_acceleration_m_s2',
           'tcp_jerk': 'peak_tcp_jerk_m_s3', 'angular_speed': 'peak_angular_speed_rad_s',
           'angular_acceleration': 'peak_angular_acceleration_rad_s2', 'angular_jerk': 'peak_angular_jerk_rad_s3'}




class CartesianSplineTrajectory:
    """One q(s) path with the same endpoint time law used by MoveL."""
    def __init__(self, curve, duration, limiting_factors):
        self.curve, self.duration = curve, float(duration)
        self.limiting_factors = limiting_factors

    def sample(self, elapsed):
        alpha = min(max(float(elapsed) / self.duration, 0.), 1.)
        fraction = smoothstep9_fraction(alpha)
        speed, _, _ = smoothstep9_derivatives(alpha)
        return self.curve(fraction), self.curve(fraction, 1) * speed / self.duration

    def describe(self):
        return {'time_law': 'smoothstep9_cartesian_spline', 'duration_s': self.duration,
                'path_parameter': 'normalized_cartesian_arc_length',
                'coefficients_sha256': hashlib.sha256(self.curve.x.tobytes() + self.curve.c.tobytes()).hexdigest()}


class TimeScaledTrajectory:
    def __init__(self, trajectory, scale):
        self.trajectory, self.scale = trajectory, float(scale)
        self.duration = trajectory.duration * self.scale

    def sample(self, elapsed):
        q, dq = self.trajectory.sample(elapsed / self.scale)
        return q, dq / self.scale

    def describe(self):
        return {'time_law': 'scaled', 'time_scale': self.scale, 'duration_s': self.duration,
                'base': self.trajectory.describe()}


def plan_path(model, start, request, joint_limits, control_hz):
    origin = np.asarray(model.fk(start), dtype=float)
    points = request.get('waypoints')
    if not isinstance(points, list) or not 1 <= len(points) <= 64:
        raise ValueError('move_path requires 1..64 base-frame poses')
    radius = request.get('blend_radius', .01)
    if isinstance(radius, bool) or not isinstance(radius, (int, float)) or not math.isfinite(radius) or radius < 0:
        raise ValueError('blend_radius must be finite and nonnegative metres')
    poses = [origin] + [finite_vector(point, 7, 'path waypoint') for point in points]
    if any(abs(np.linalg.norm(pose[3:]) - 1) > .01 for pose in poses):
        raise ValueError('Path quaternions must be normalized')
    if radius == 0 and len(poses) > 2:
        deltas = np.diff(np.asarray(poses)[:, :3], axis=0)
        direction = deltas / np.maximum(np.linalg.norm(deltas, axis=1, keepdims=True), 1e-12)
        if np.any(np.linalg.norm(np.diff(direction, axis=0), axis=1) > 1e-6):
            raise ValueError('A continuous path needs a positive blend_radius at direction changes')
    geometry = build_cartesian_path([
        CartesianWaypoint(pose[:3], pose[3:], radius if 0 < i < len(poses)-1 else 0.)
        for i, pose in enumerate(poses)])
    if len(geometry.positions_xyz_m) > 2000:
        raise ValueError('Path geometry exceeds 2000 samples; split the path')
    joints, distances, seed = [np.asarray(start).copy()], [0.], np.asarray(start).copy()
    for xyz, quaternion, distance in zip(geometry.positions_xyz_m[1:], geometry.orientations_xyzw[1:], geometry.distance_m[1:]):
        seed, converged, error = model.solve_ik(np.r_[xyz, quaternion], seed)
        if not converged or error > .001 or np.max(np.abs(seed - joints[-1])) > .1:
            raise ValueError('Path has an unreachable/discontinuous IK segment')
        if np.max(np.abs(seed - joints[-1])) > 1e-9:
            joints.append(seed.copy())
            distances.append(distance)
    if len(joints) < 2:
        raise ValueError('Path has no motion')
    # Geometry samples constrain q(s), not independent acceleration boundaries.
    # Resetting acceleration at every IK sample makes duration grow with density.
    progress = np.asarray(distances) / distances[-1]
    curve = CubicSpline(progress, np.asarray(joints), axis=0)
    alpha = np.linspace(0., 1., 2001)
    fractions = np.asarray([smoothstep9_fraction(float(u)) for u in alpha])
    ds, dds, ddds = (value[:, None] for value in smoothstep9_derivatives(alpha))
    q = curve(fractions)
    path_v, path_a, path_j = (curve(fractions, order) for order in (1, 2, 3))
    velocity = path_v * ds
    acceleration = path_a * ds**2 + path_v * dds
    jerk = path_j * ds**3 + 3. * path_a * ds * dds + path_v * ddds
    timing = {'minimum_duration': 1. / control_hz,
        'speed': float(np.max(np.abs(velocity) / np.asarray(joint_limits['joint_velocity_rad_s']))),
        'acceleration': (float(np.max(np.abs(acceleration))) / joint_limits['acceleration'])**.5,
        'jerk': (float(np.max(np.abs(jerk))) / joint_limits['jerk'])**(1/3)}
    duration = 1.05 * max(timing.values())
    limiting = [name for name, value in timing.items() if math.isclose(value, max(timing.values()), rel_tol=1e-9)]
    return (CartesianSplineTrajectory(curve, duration, limiting), alpha * duration, q,
            velocity / duration, acceleration / duration**2, jerk / duration**3)


def tcp_metrics(model, qs, times):
    poses = np.asarray([model.fk(q) for q in qs])
    xyz = poses[:, :3]
    edge = 2 if len(times) >= 3 else 1
    velocity = np.gradient(xyz, times, axis=0, edge_order=edge)
    acceleration = np.gradient(velocity, times, axis=0, edge_order=edge)
    jerk = np.gradient(acceleration, times, axis=0, edge_order=edge)
    rotations = Rotation.from_quat(poses[:, 3:])
    increments = (rotations[1:] * rotations[:-1].inv()).as_rotvec()
    angular = increments / np.diff(times)[:, None]
    angular = np.vstack([angular[0], (angular[:-1] + angular[1:]) / 2, angular[-1]])
    angular_acceleration = np.gradient(angular, times, axis=0, edge_order=edge)
    angular_jerk = np.gradient(angular_acceleration, times, axis=0, edge_order=edge)
    peaks = [float(np.max(np.linalg.norm(v, axis=1)))
             for v in (velocity, acceleration, jerk, angular, angular_acceleration, angular_jerk)]
    return {**dict(zip(METRICS.values(), peaks)),
            'path_length_m': float(np.linalg.norm(np.diff(xyz, axis=0), axis=1).sum()),
            'rotation_length_rad': float(np.linalg.norm(increments, axis=1).sum()),
            'tcp_derivative_evaluation': 'sampled_forward_kinematics'}


def apply_tcp_limits(model, trajectory, times, qs, velocity, acceleration, jerk, requested, joint_limits, control_hz):
    metrics = tcp_metrics(model, qs, times)
    factors = {'speed': float(np.max(np.abs(velocity) / np.asarray(joint_limits['joint_velocity_rad_s']))),
               'acceleration': (float(np.max(np.abs(acceleration))) / joint_limits['acceleration'])**.5,
               'jerk': (float(np.max(np.abs(jerk))) / joint_limits['jerk'])**(1/3)}
    orders = dict(zip(TCP_UNITS, (1, 2, 3, 1, 2, 3)))
    for name, limit in requested.items():
        factors[name] = (metrics[METRICS[name]] / limit)**(1 / orders[name])
    # The shared scalar planner supplies a physical lower bound for full TCP
    # translation profiles. Curvature/angular and joint checks remain explicit.
    if all(key in requested for key in ('tcp_speed', 'tcp_acceleration', 'tcp_jerk')) and metrics['path_length_m'] > 1e-9:
        progress = plan_scalar_progress(metrics['path_length_m'], max_velocity_m_s=requested['tcp_speed'],
            max_acceleration_m_s2=requested['tcp_acceleration'], max_jerk_m_s3=requested['tcp_jerk'],
            sample_rate_hz=control_hz, minimum_duration_s=1. / control_hz)
        factors['tcp_profile'] = progress.duration_s / times[-1]
    largest = max(factors.values())
    scale = max(1., largest * 1.02) if largest > 1. else 1.
    limiting = [name for name, value in factors.items() if value >= max(1., largest) * .99]
    if scale > 1:
        trajectory = TimeScaledTrajectory(trajectory, scale)
    for name, metric in METRICS.items():
        metrics[metric] /= scale**orders[name]
    return (trajectory, times * scale, qs, velocity / scale, acceleration / scale**2, jerk / scale**3,
            metrics, limiting)
