"""Shared six-axis X5 kinematics and joint targets for hardware and simulation."""
from pathlib import Path
import hashlib
import json
import sys
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from .types import ControlCommand, Mode, RobotConfig


def finite_vector(value, size, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f'{name} must contain {size} finite numbers')
    return result


def se3_error(position, rotation, target_position, target_rotation):
    """Log(current^-1 target), linear then angular, in the current local frame."""
    phi = Rotation.from_matrix(rotation.T @ target_rotation).as_rotvec()
    theta = np.linalg.norm(phi)
    x, y, z = phi
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    coefficient = 1 / 12 + theta * theta / 720 if theta < 1e-5 else (1 - .5 * theta / np.tan(.5 * theta)) / (theta * theta)
    inverse_left_jacobian = np.eye(3) - .5 * skew + coefficient * skew @ skew
    translation = inverse_left_jacobian @ (rotation.T @ (target_position - position))
    return np.concatenate((translation, phi))


class X5Model:
    def __init__(self, config: RobotConfig):
        self.config = config
        root = Path(__file__).resolve().parents[1]
        path = Path(config.model_path) if config.model_path else root / 'models/x5/x5_2025.xml'
        self.tool_path = Path(config.tool_path) if config.tool_path else root / 'config/tool.json'
        self.urdf_path = path.with_suffix('.urdf')
        manifest_path = path.with_suffix('.model.json')
        if not manifest_path.is_file():
            raise ValueError('Model manifest missing; rebuild using scripts/build_model.py')
        metadata = json.loads(manifest_path.read_text())
        expected_files = {path.name, self.urdf_path.name}
        hashes = metadata.get('sha256', {})
        if metadata.get('schema_version') != 2 or set(hashes) != expected_files:
            raise ValueError('Invalid model manifest; rebuild using scripts/build_model.py')
        for name, digest in hashes.items():
            source = path.parent / name
            if not source.is_file() or hashlib.sha256(source.read_bytes()).hexdigest() != digest:
                raise ValueError(f'Model/tool mismatch in {name}; rebuild using scripts/build_model.py')
        if (not self.tool_path.is_file()
                or hashlib.sha256(self.tool_path.read_bytes()).hexdigest() != metadata.get('tool_sha256')):
            raise ValueError('Model/tool mismatch in tool configuration; rebuild using scripts/build_model.py')
        self.tcp_link = metadata['tcp_link']
        self.calibration_status = metadata['calibration_status']
        self.model = mujoco.MjModel.from_xml_path(str(path))
        self.data = mujoco.MjData(self.model)
        self.site = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, 'tcp')
        expected_dof = 8
        if self.model.nq != expected_dof or self.model.nv != expected_dof or self.site < 0:
            raise ValueError(f'X5 model requires {expected_dof} DoF and a tcp site')
        self.lower = finite_vector(config.joint_min, 6, 'joint_min')
        self.upper = finite_vector(config.joint_max, 6, 'joint_max')
        if np.any(self.lower >= self.upper):
            raise ValueError('joint_min must be below joint_max')
        self.model.jnt_range[:6] = np.column_stack((self.lower, self.upper))
        self.model.jnt_range[6:, 1] = config.gripper_max_width / 2
        self._jacp = np.zeros((3, self.model.nv))
        self._jacr = np.zeros((3, self.model.nv))

    def _forward(self, q):
        self.data.qpos[:6] = finite_vector(q, 6, 'q')
        self.data.qvel[:] = 0
        mujoco.mj_forward(self.model, self.data)

    def fk(self, q):
        self._forward(q)
        matrix = self.data.site_xmat[self.site].reshape(3, 3)
        return np.concatenate((self.data.site_xpos[self.site].copy(), Rotation.from_matrix(matrix).as_quat()))

    def gravity(self, q):
        self._forward(q)
        return self.data.qfrc_bias[:6].copy()

    def solve_ik(self, target, q_seed, q_anchor=None):
        target = finite_vector(target, 7, 'target pose')
        if np.linalg.norm(target[3:]) < 1e-12:
            raise ValueError('target quaternion must be nonzero')
        target_rotation = Rotation.from_quat(target[3:]).as_matrix()
        q = finite_vector(q_seed, 6, 'q_seed').copy()
        anchor = None if q_anchor is None else finite_vector(q_anchor, 6, 'q_anchor')
        mu_squared = 0. if anchor is None else .005 ** 2
        regularizer = (.01 ** 2 + mu_squared) * np.eye(6)
        for _ in range(50):
            self._forward(q)
            rotation = self.data.site_xmat[self.site].reshape(3, 3)
            error = se3_error(self.data.site_xpos[self.site], rotation, target[:3], target_rotation)
            residual = float(np.linalg.norm(error))
            if residual < 1e-4 and anchor is None:
                return np.clip(q, self.lower, self.upper), True, residual
            mujoco.mj_jacSite(self.model, self.data, self._jacp, self._jacr, self.site)
            jacobian = np.vstack((rotation.T @ self._jacp[:, :6], rotation.T @ self._jacr[:, :6]))
            rhs = jacobian.T @ error
            if anchor is not None:
                rhs += mu_squared * (anchor - q)
            try:
                delta = np.linalg.solve(jacobian.T @ jacobian + regularizer, rhs)
            except np.linalg.LinAlgError:
                return np.clip(q, self.lower, self.upper), False, residual
            q = np.clip(q + delta, self.lower, self.upper)
            if anchor is not None and residual < 1e-4:
                return q, True, residual
        # Match NERO's stopping rule and diagnostics: residual is from the last
        # evaluated iterate, before its final update; finite partial q is valid.
        return q, False, residual


class JointTargetController:
    """Backend-shared mode handling. FreeDrive returns None; other modes return q."""
    def __init__(self, config: RobotConfig, kinematics: X5Model):
        self.config = config
        self.kinematics = kinematics
        self.mode = Mode.DISABLED
        self.gripper_mode = Mode.DISABLED
        self.q_target = None
        self.dq_target = np.zeros(6)
        self._joint_goal = None
        self._ik_seed = None
        self.gripper_target = config.gripper_max_width
        self.fault = ''
        self.ik_converged = True
        self.ik_residual = 0.
        self.ik_rejections = 0
        self.ik_partial_solutions = 0
        self._last_ik_warning_at = float('-inf')
        self._mode_since = None
        self._park_start = None
        self._holding = False
        self._hold_target = None

    def update(self, command: ControlCommand, q, now):
        q = finite_vector(q, 6, 'measured q')
        self.dq_target[:] = 0.0
        transition = self._mode_since is None or command.mode != self.mode or command.reset_reference
        if transition:
            self.mode = command.mode
            self._mode_since = now
            self.q_target = q.copy()
            self._joint_goal = None
            self._ik_seed = None
            self._park_start = q.copy()
            self._holding = False
            self._hold_target = None
        self.fault = ''
        self.gripper_mode = command.gripper_mode or command.mode
        if self.gripper_mode == Mode.DISABLED:
            self.gripper_target = self.config.gripper_max_width
        elif self.gripper_mode == Mode.COMMAND_FOLLOWING and command.gripper_target is not None:
            if not np.isfinite(command.gripper_target):
                self.fault = 'non-finite gripper target'
            else:
                self.gripper_target = float(np.clip(command.gripper_target, 0, self.config.gripper_max_width))
        if self.mode == Mode.DISABLED:
            delta = np.asarray(self.config.park_q) - self._park_start
            # Match the SDK home timing (at most 1 rad/s, minimum 1 s),
            # while keeping VR, feedback, cancellation and recording running.
            duration = max(1., float(np.max(np.abs(delta))) + .5)
            fraction = np.clip((now - self._mode_since) / duration, 0., 1.)
            self.q_target = np.clip(self._park_start + fraction * delta,
                                    self.kinematics.lower, self.kinematics.upper)
            if fraction < 1.0:
                self.dq_target[:] = delta / duration
            return self.q_target.copy()
        if self.mode == Mode.FREE_DRIVE:
            self.q_target = q.copy()
            return None
        # Keep the full IK goal independent of the previously sent target.
        # Sparse VR packets must not lower the 250 Hz output slew frequency.
        candidate = q if self._joint_goal is None else self._joint_goal
        if command.hold:
            if not self._holding:
                self._hold_target = q.copy()
            self._holding = True
            self._ik_seed = None
            candidate = self._hold_target
            self._joint_goal = candidate.copy()
        elif command.joint_target is not None:
            self._holding = False
            candidate = np.clip(finite_vector(command.joint_target, 6, 'joint target'),
                                self.kinematics.lower, self.kinematics.upper)
            self._joint_goal = candidate.copy()
            self._ik_seed = None
        elif command.pose_target is not None:
            self._holding = False
            try:
                solution, self.ik_converged, self.ik_residual = self.kinematics.solve_ik(command.pose_target, q if self._ik_seed is None else self._ik_seed, q)
                candidate = np.clip(finite_vector(solution, 6, 'IK output'), self.kinematics.lower, self.kinematics.upper)
                self._ik_seed = candidate.copy()
                self._joint_goal = candidate.copy()
                if not self.ik_converged:
                    self.ik_partial_solutions += 1
                    self._warn_ik(now, 'IK did not fully converge',
                                  'applying the finite joint-limited partial target')
            except (ValueError, np.linalg.LinAlgError) as error:
                # NERO rejects invalid IK without discarding the valid cached
                # goal/seed or switching mode. Before any valid goal, use q.
                self.ik_converged = False
                self.ik_residual = float('inf')
                self.ik_rejections += 1
                self._warn_ik(now, f'rejected invalid IK result ({error})',
                              'holding the last valid target')
        # Every tick, including an empty packet or rejection, walks toward the
        # complete cached target with the same bound as NERO's send path.
        step_limit = self.config.joint_step_limit
        if command.joint_step_limit is not None:
            if not np.isfinite(command.joint_step_limit) or command.joint_step_limit <= 0:
                raise ValueError('Invalid command step limit')
            step_limit = min(step_limit, command.joint_step_limit)
        delta = np.clip(candidate - self.q_target, -step_limit, step_limit)
        self.q_target = np.clip(self.q_target + delta, self.kinematics.lower, self.kinematics.upper)
        return self.q_target.copy()

    def _warn_ik(self, now, message, action):
        if now - self._last_ik_warning_at < 1.0:
            return
        print(f'arx5-vr: {message} (error={self.ik_residual}); {action}.', file=sys.stderr)
        self._last_ik_warning_at = now
