"""Preflight using pinned vendor offline algorithms, never a robot connection.

These local checks cannot certify controller IK, collision clearance or the
interpolated physical trajectory. The controller remains authoritative.
"""
import importlib.util
import subprocess
import sys

import numpy as np

from embodied.core.dependencies import dependency, dependency_root
from .geometry import rotation_vector


def pose_error(first, second):
    return (float(np.linalg.norm(first[:3] - second[:3])),
            float(np.linalg.norm(rotation_vector(first[3:], second[3:]))))


def load_sdk():
    """Load the pinned local algorithm package without opening a robot handle."""
    root = dependency_root('realman-api')
    revision = dependency('realman-api')['commit']
    if not (root / '.git').exists():
        raise RuntimeError('Missing offline SDK: ./run.sh fetch-dependencies --name realman-api')
    if subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True).strip() != revision:
        raise RuntimeError('RealMan offline SDK revision mismatch')
    package = root / 'Python/Robotic_Arm'
    subprocess.run(['git', '-C', str(root), 'diff', '--exit-code', 'HEAD', '--', 'Python/Robotic_Arm'],
                   check=True, capture_output=True)
    name = '_tia_realman_offline_sdk'
    if name not in sys.modules:
        spec = importlib.util.spec_from_file_location(name, package / '__init__.py',
                                                     submodule_search_locations=[str(package)])
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
    from importlib import import_module
    sdk = import_module(name + '.rm_robot_interface')
    return sdk


class OfflineModel:
    """Only Algo is instantiated; frame/limit setters modify local algorithm data."""
    def __init__(self, info):
        sdk = load_sdk()
        self.sdk = sdk
        self.algo = sdk.Algo(sdk.rm_robot_arm_model_e.RM_MODEL_RM_65_E,
                             sdk.rm_force_type_e.RM_MODEL_RM_B_E)
        # Read actual controller geometry. Wire: alpha/offset in millidegrees,
        # a/d in micrometres. SDK: degrees/metres.
        dh = np.array([info['get_DH_data'][f'joint_{i}'] for i in range(1, 7)], dtype=float)
        if dh.shape != (6, 4) or not np.isfinite(dh).all():
            raise ValueError('Invalid controller DH parameters')
        self.algo.rm_algo_set_dh(sdk.rm_dh_t(d=(dh[:, 2]/1e6).tolist(), a=(dh[:, 1]/1e6).tolist(),
                                          alpha=(dh[:, 0]/1000).tolist(), offset=(dh[:, 3]/1000).tolist()))
        for key, setter, field in (
            ('get_current_tool_frame', self.algo.rm_algo_set_toolframe, 'tool_name'),
            ('get_current_work_frame', self.algo.rm_algo_set_workframe, 'frame_name')):
            raw = info[key]
            pose = np.array(raw['pose'], dtype=float) / [1e6, 1e6, 1e6, 1000, 1000, 1000]
            setter(sdk.rm_frame_t(raw[field], tuple(pose)))
        self.lower = np.array(info['get_joint_min_pos']['min_pos'], dtype=float)/1000
        self.upper = np.array(info['get_joint_max_pos']['max_pos'], dtype=float)/1000
        if (self.lower.shape != (6,) or self.upper.shape != (6,) or not np.isfinite(self.lower).all()
                or not np.isfinite(self.upper).all() or np.any(self.lower >= self.upper)):
            raise ValueError('Invalid controller joint limits')
        self.algo.rm_algo_set_joint_min_limit(self.lower.tolist())
        self.algo.rm_algo_set_joint_max_limit(self.upper.tolist())
        self.algo.rm_algo_kin_singularity_thresholds_init()
        self.version = str(self.algo.rm_algo_version())

    def fk(self, q_deg):
        raw = self.algo.rm_algo_forward_kinematics(q_deg.tolist(), 0)
        return np.r_[raw[:3], raw[4:], raw[3]]  # Vendor wxyz -> platform xyzw.

    def solve(self, target, seed_deg):
        pose = np.r_[target[:3], target[6], target[3:6]].tolist()
        code, q = self.algo.rm_algo_inverse_kinematics(
            self.sdk.rm_inverse_kinematics_params_t(seed_deg.tolist(), pose, 0))
        return code, np.array(q, dtype=float)

    def singularity(self, q_deg):
        return self.algo.rm_algo_kin_robot_singularity_analyse(q_deg.tolist())


class JointStepTooLarge(RuntimeError):
    """Valid local IK candidate exceeds the adjacent command step bound."""


class CandidateRejected(RuntimeError):
    """No motion from this posture/target; an explicit restart may recheck it."""


class MotionCheck:
    def __init__(self, info, state, *, model=None):
        self.model = model if model is not None else OfflineModel(info)
        self.fit = self.validate_feedback(state)

    def validate_feedback(self, state):
        q = np.rad2deg(state.q)
        expected = self.model.fk(q)
        if expected.shape != (7,) or not np.isfinite(expected).all():
            raise ValueError('Offline forward kinematics returned invalid feedback')
        position, angle = pose_error(expected, state.pose)
        if position > .002 or angle > .01:
            raise RuntimeError(f'Offline model disagrees with controller: {position:.6f} m, {angle:.6f} rad')
        singularity, distance = self.model.singularity(q)
        if not np.isfinite(distance):
            raise RuntimeError('Invalid current vendor singularity feedback')
        if singularity != 0:
            raise CandidateRejected(f'Current posture near vendor singularity region: code={singularity}, shoulder_m={distance}')
        return {'fk_position_error_m': position, 'fk_angle_error_rad': angle,
                'singularity_code': singularity, 'shoulder_distance_m': distance,
                'offline_algorithm_version': self.model.version}

    def check(self, target, state, *, previous_joint_deg=None, max_joint_step_deg=None):
        self.diagnostics = {}
        self.validate_feedback(state)
        seed = np.rad2deg(state.q)
        code, solution = self.model.solve(target, seed)
        if code == 1:
            raise CandidateRejected('Offline IK rejected candidate: result=1 (no solution)')
        if code != 0 or solution.shape != (6,) or not np.isfinite(solution).all():
            raise RuntimeError(f'Offline IK rejected candidate: result={code}')
        if np.any(solution < self.model.lower) or np.any(solution > self.model.upper):
            raise RuntimeError('Offline IK candidate exceeds controller joint limits')
        position, angle = pose_error(self.model.fk(solution), target)
        if position > .001 or angle > .005:
            raise RuntimeError(f'Offline IK residual too large: {position:.6f} m, {angle:.6f} rad')
        code, distance = self.model.singularity(solution)
        if not np.isfinite(distance):
            raise RuntimeError('Invalid vendor singularity feedback')
        if code != 0:
            raise CandidateRejected(f'Candidate near vendor singularity region: code={code}, shoulder_m={distance}')
        # Conservative continuity guard, not a guarantee on actual joint velocity.
        previous = seed if previous_joint_deg is None else np.asarray(previous_joint_deg, dtype=float)
        if previous.shape != (6,) or not np.isfinite(previous).all():
            raise ValueError('Invalid previous commanded joints')
        delta = solution - previous
        step = float(np.max(np.abs(delta)))
        limits = np.ones(6) if max_joint_step_deg is None else np.asarray(max_joint_step_deg, dtype=float)
        if limits.shape != (6,) or not np.isfinite(limits).all() or np.any(limits <= 0):
            raise ValueError('Invalid per-joint command step limits')
        self.diagnostics = {
            'feedback_joint_deg': seed.tolist(),
            'previous_command_joint_deg': previous.tolist(),
            'command_delta_deg': delta.tolist(),
            'feedback_lead_deg': (solution - seed).tolist(),
            'max_change_joint': int(np.argmax(np.abs(delta))) + 1,
            'joint_step_limit_deg': limits.tolist(),
            'baseline': 'feedback' if previous_joint_deg is None else 'previous_sent_command',
            'ik_joint_deg': solution.tolist(), 'max_joint_change_deg': step,
        }
        exceeded = np.abs(delta) > limits + 1e-9
        if np.any(exceeded):
            joint = int(np.argmax(np.abs(delta) / limits))
            raise JointStepTooLarge(f'Offline IK adjacent command joint change too large: joint {joint + 1}, '
                                   f'{abs(delta[joint]):.3f} deg (limit {limits[joint]:.3f} deg)')
        return {**self.diagnostics,
                'fk_position_error_m': position, 'fk_angle_error_rad': angle,
                'singularity_code': code, 'shoulder_distance_m': distance}
