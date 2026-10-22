"""Cartesian geometry and time bounds for the unified planner, outside the tick loop."""
import math
import hashlib
import numpy as np
from scipy.interpolate import BPoly
from scipy.spatial.transform import Rotation

from embodied.motion.geometry import CartesianWaypoint, build_cartesian_path
from embodied.motion.offline import plan_blended_joint_trajectory
from embodied.motion.cartesian_time import plan_scalar_progress
from embodied.motion.validation import finite_vector

TCP_UNITS = {'tcp_speed': 'm/s', 'tcp_acceleration': 'm/s^2', 'tcp_jerk': 'm/s^3',
             'angular_speed': 'rad/s', 'angular_acceleration': 'rad/s^2', 'angular_jerk': 'rad/s^3'}
METRICS = {'tcp_speed': 'peak_tcp_speed_m_s', 'tcp_acceleration': 'peak_tcp_acceleration_m_s2',
           'tcp_jerk': 'peak_tcp_jerk_m_s3', 'angular_speed': 'peak_angular_speed_rad_s',
           'angular_acceleration': 'peak_angular_acceleration_rad_s2', 'angular_jerk': 'peak_angular_jerk_rad_s3'}


def tcp_limits(request):
    result = {}
    for name in TCP_UNITS:
        if name in request:
            value = request[name]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive ({TCP_UNITS[name]})')
            result[name] = float(value)
    return result


class QuinticTrajectory:
    """Exact piecewise quintic reconstruction: the shared planner retains anchors."""
    def __init__(self, curve, start):
        self.times = np.r_[0., curve.sample_times_s]
        self.q = np.vstack([start, curve.positions])
        self.dq = np.vstack([np.zeros_like(start), curve.velocities])
        self.ddq = np.vstack([np.zeros_like(start), curve.accelerations])
        self.curve = BPoly.from_derivatives(self.times, [[q, v, a] for q, v, a in zip(self.q, self.dq, self.ddq)])
        self.duration = float(self.times[-1])

    def sample(self, elapsed):
        t = min(max(float(elapsed), 0.), self.duration)
        return self.curve(t), self.curve(t, 1)

    def describe(self):
        return {'time_law': 'quintic_blended', 'duration_s': self.duration,
                'sample_count': len(self.times),
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
    joints, seed = [np.asarray(start).copy()], np.asarray(start).copy()
    for xyz, quaternion in zip(geometry.positions_xyz_m[1:], geometry.orientations_xyzw[1:]):
        seed, converged, error = model.solve_ik(np.r_[xyz, quaternion], seed)
        if not converged or error > .001 or np.max(np.abs(seed - joints[-1])) > .1:
            raise ValueError('Path has an unreachable/discontinuous IK segment')
        if np.max(np.abs(seed - joints[-1])) > 1e-9:
            joints.append(seed.copy())
    if len(joints) < 2:
        raise ValueError('Path has no motion')
    curve = plan_blended_joint_trajectory(np.asarray(joints),
        max_velocity=joint_limits['joint_velocity_rad_s'], max_acceleration=joint_limits['acceleration'],
        max_jerk=joint_limits['jerk'], sample_rate_hz=control_hz, minimum_duration_s=1. / control_hz,
        maximum_duration_s=60.)
    trajectory = QuinticTrajectory(curve, start)
    jerk = trajectory.curve(trajectory.times, 3)
    return trajectory, trajectory.times, trajectory.q, trajectory.dq, trajectory.ddq, jerk


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
