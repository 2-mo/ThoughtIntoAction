"""RealMan adapter implementing the platform RobotBackend value contract.

Feedback uses radians, metres and xyzw quaternions. Missing velocity and gripper
feedback are NaN, never synthesized as measured zero. No automatic enable/home.
"""
import json
from pathlib import Path
import threading
import time

import numpy as np
from meshcat import transformations as tf
from .geometry import rotation_vector, limited_rotation

from embodied.contracts.control import Mode, RobotState
from embodied.contracts.device_lease import DeviceLease
from .protocol import Client
from .kinematics import MotionCheck, JointStepTooLarge, CandidateRejected

DEFAULT_CONFIG = Path(__file__).resolve().parents[5] / 'configs/robots/realman/rm65-19.json'


def vector(value, size):
    a = np.asarray(value, dtype=float)
    if a.shape != (size,) or not np.isfinite(a).all():
        raise ValueError(f'Expected {size} finite values')
    return a


def decode_pose(raw):
    a = vector(raw, 6)
    return np.r_[a[:3] / 1e6, np.roll(tf.quaternion_from_euler(*(a[3:] / 1000)), -1)]


def encode_pose(pose):
    p = vector(pose, 7)
    if not np.isclose(np.linalg.norm(p[3:]), 1, atol=1e-3):
        raise ValueError('Pose quaternion must be normalized')
    return np.rint(np.r_[p[:3] * 1e6, np.asarray(tf.euler_from_quaternion(np.roll(p[3:], 1))) * 1000]).astype(int).tolist()


def load_config(path=DEFAULT_CONFIG):
    config = json.loads(Path(path).read_text())
    speed_fields = ('speed_percent',) if 'speed_percent' in config else ('linear_speed_m_s', 'angular_speed_rad_s')
    for name in ('timeout_s', 'control_hz', 'stale_s',
                 'translation_radius_m', 'rotation_radius_rad', 'position_scale', *speed_fields):
        if not np.isfinite(config[name]) or config[name] <= 0:
            raise ValueError(f'{name} must be finite and positive')
    if 'speed_percent' in config and (type(config['speed_percent']) is not int or config['speed_percent'] > 100):
        raise ValueError('speed_percent must be an integer in 1..100')
    if 'linear_speed_percent' in config and (type(config['linear_speed_percent']) is not int or
            not 1 <= config['linear_speed_percent'] <= 100 or 'speed_percent' not in config):
        raise ValueError('linear_speed_percent requires speed_percent and must be in 1..100')
    if config['control_hz'] > 50 or config['stale_s'] > .5:
        raise ValueError('This low-follow driver supports <=50 Hz and <=0.5 s timeout')
    basis = np.asarray(config['xr_to_robot_basis'], dtype=float)
    if basis.shape != (3, 3) or not np.allclose(basis.T @ basis, np.eye(3)) or not np.isclose(np.linalg.det(basis), 1):
        raise ValueError('XR basis must be a proper rotation')
    return config


class Backend:
    def __init__(self, config=None, *, allow_motion=False, client=None, defer_motion_check=False):
        self.config = dict(load_config() if config is None else config)
        self.allow_motion = allow_motion
        self.lease = DeviceLease('realman-' + self.config['host'])
        try:
            self.client = client or Client(self.config['host'], self.config['port'],
                                           self.config['timeout_s'], allow_motion=allow_motion)
        except BaseException:
            self.lease.close()
            raise
        self.lock = threading.RLock()
        self.active = False
        self.closed = False
        self.fault = ''
        self.last_command = None
        self.feedback_at = None
        self.state = None
        self.origin = None
        self.last_target = None
        self.last_joint_target = None
        self.lead_hold_since = None
        self.command_diagnostics = {}
        self.done = threading.Event()
        self.watchdog = None
        self.motion_check = None
        try:
            self.info = {k: self.client.query(k) for k in
                         ('get_arm_software_info', 'get_current_tool_frame', 'get_current_work_frame')}
            self._check_identity()
            if 'speed_percent' in self.config:
                ratio = self.config['speed_percent'] / 100
                for command, field, effective in (
                    ('get_arm_max_line_speed', 'arm_line_speed', 'linear_speed_m_s'),
                    ('get_arm_max_angular_speed', 'arm_angular_speed', 'angular_speed_rad_s')):
                    reply = self.client.query(command)
                    raw = reply.get(field)
                    if type(raw) is not int or raw <= 0:
                        raise ValueError(f'Invalid controller speed feedback: {reply}')
                    self.info[command] = reply
                    effective_ratio = self.config.get('linear_speed_percent', self.config['speed_percent']) / 100 if effective == 'linear_speed_m_s' else ratio
                    self.config[effective] = raw / 1000 * effective_ratio
                reply = self.client.query('get_joint_max_speed')
                # JSON: 0.001 RPM; convert to degrees/second, then apply the session ratio.
                raw = vector(reply.get('joint_speed'), 6)
                if np.any(raw <= 0):
                    raise ValueError(f'Invalid controller joint speeds: {reply}')
                self.info['get_joint_max_speed'] = reply
                self.config['joint_speed_deg_s'] = (raw * .006 * ratio).tolist()
            self.read_state(time.monotonic())
            self.origin = self.state.pose.copy()
            if allow_motion:
                if not defer_motion_check:
                    self.prepare_motion_check()
                self.watchdog = threading.Thread(target=self._watch, daemon=True)
                self.watchdog.start()
        except BaseException:
            self.client.close()
            self.lease.close()
            raise

    def prepare_motion_check(self):
        if self.motion_check is None:
            for key in ('get_DH_data', 'get_joint_min_pos', 'get_joint_max_pos'):
                self.info[key] = self.client.query(key)
            self.motion_check = MotionCheck(self.info, self.state)
        return self.motion_check

    def check_motion_ready(self):
        """Recheck the current feedback before an explicit standby start."""
        return self.prepare_motion_check().check(self.state.pose, self.state)

    def _check_identity(self):
        c = self.config
        if self.info['get_arm_software_info']['Product_version'] != c['expected_model']:
            raise ValueError('Unexpected RealMan model')
        for key, field, expected, pose in (
            ('get_current_tool_frame', 'tool_name', 'expected_tool', 'expected_tool_pose'),
            ('get_current_work_frame', 'frame_name', 'expected_work_frame', 'expected_work_pose')):
            if self.info[key][field] != c[expected] or self.info[key]['pose'] != c[pose]:
                raise ValueError(f'{key} differs from the deployment configuration')

    def read_state(self, now):
        if self.closed:
            raise RuntimeError('Backend closed')
        raw = self.client.query('get_current_arm_state')['arm_state']
        q = np.deg2rad(vector(raw['joint'], 6) / 1000)
        pose = decode_pose(raw['pose'])
        errors = raw['err']
        if not isinstance(errors, list):
            raise ValueError('Missing RealMan error list')
        fault = f'RealMan errors: {errors}' if any(errors) else ''
        with self.lock:
            self.feedback_at = time.monotonic()
            self.state = RobotState(q, np.full(6, np.nan), pose, float('nan'),
                                    self.feedback_at, Mode.COMMAND_FOLLOWING if self.active else Mode.DISABLED,
                                    fault or self.fault)
            return self.state

    def _watch(self):
        while not self.done.wait(.02):
            with self.lock:
                if self.active and (time.monotonic() - self.last_command > self.config['stale_s'] or
                                    time.monotonic() - self.feedback_at > self.config['stale_s']):
                    self.fault = 'Command or feedback timeout; reopen session to recover'
                    try:
                        self.stop()
                    except Exception as exc:
                        self.fault += f'; stop delivery failed: {exc}'

    def stop(self):
        with self.lock:
            if self.active:
                self.active = False
                self.client.send({'command': 'set_arm_stop'})
            self.last_target = None
            self.last_joint_target = None
            self.lead_hold_since = None

    def step(self, command, now, **options):
        try:
            self._step(command, now, **options)
        except CandidateRejected:
            # A rejected target is never sent. Keep a usable connection for
            # explicit operator re-engagement after stopping.
            self.stop()
            raise
        except Exception as exc:
            self.fault = self.fault or 'Command rejected; reopen session to recover'
            try:
                self.stop()
            except Exception as stop_error:
                exc.add_note(f'Stop delivery also failed: {stop_error}')
            raise

    def _step(self, command, now, **options):
        if not self.allow_motion:
            raise PermissionError('RealMan backend is read-only')
        with self.lock:
            if self.closed or self.fault:
                raise RuntimeError(self.fault or 'Backend closed')
            if command.gripper_target is not None or command.joint_target is not None or options:
                raise ValueError('This RealMan adapter supports Cartesian control only; gripper is unconfigured')
            if command.mode == Mode.FREE_DRIVE:
                raise ValueError('Free drive is not implemented')
            if command.mode == Mode.DISABLED or command.hold or not command.engaged:
                self.stop()
                return
            if self.state.fault or time.monotonic() - self.feedback_at > self.config['stale_s']:
                self.stop()
                raise RuntimeError(self.state.fault or 'Stale robot feedback')
            target = vector(command.pose_target, 7).copy()
            encode_pose(target)  # Validate quaternion before doing rotation arithmetic.
            c = self.config
            distance = np.linalg.norm(target[:3] - self.origin[:3])
            angle = np.linalg.norm(rotation_vector(target[3:], self.origin[3:]))
            if distance > c['translation_radius_m'] + 1e-6 or angle > c['rotation_radius_rad'] + 1e-6:
                self.stop()
                raise ValueError('Target exceeds session workspace radius')
            previous = self.last_target if self.last_target is not None else self.state.pose
            elapsed = min(max(time.monotonic() - (self.last_command or time.monotonic()), 0), 1/c['control_hz'])
            if self.last_target is None:
                elapsed = 1/c['control_hz']
            dp = target[:3] - previous[:3]
            target[:3] = previous[:3] + dp * min(1., c['linear_speed_m_s'] * elapsed / max(np.linalg.norm(dp), 1e-12))
            target[3:] = limited_rotation(target[3:], previous[3:], c['angular_speed_rad_s'] * elapsed)
            # Bound feedback lag by the configured speed and timeout in percentage mode.
            position_lead = float(np.linalg.norm(target[:3] - self.state.pose[:3]))
            angle_lead = float(np.linalg.norm(rotation_vector(target[3:], self.state.pose[3:])))
            position_limit = c['linear_speed_m_s'] * c['stale_s'] if 'speed_percent' in c else .01
            angle_limit = c['angular_speed_rad_s'] * c['stale_s'] if 'speed_percent' in c else .1
            limited = position_lead > position_limit or angle_lead > angle_limit
            self.command_diagnostics = {
                'requested_m_xyzw': vector(command.pose_target, 7).tolist(),
                'feedback_m_xyzw': self.state.pose.tolist(),
                'candidate_lead_m': position_lead, 'candidate_lead_rad': angle_lead,
                'lead_limited': limited, 'feedback_age_s': time.monotonic() - self.feedback_at,
                'position_lead_limit_m': position_limit, 'angle_lead_limit_rad': angle_limit,
            }
            if limited:
                # Retaining the previous command preserves command speed continuity.
                target = previous.copy()
                if (np.linalg.norm(target[:3] - self.state.pose[:3]) > position_limit or
                        np.linalg.norm(rotation_vector(target[3:], self.state.pose[3:])) > angle_limit):
                    raise RuntimeError('Previous command already exceeds feedback lead limits')
                if self.lead_hold_since is None:
                    self.lead_hold_since = time.monotonic()
                if time.monotonic() - self.lead_hold_since >= 1.:
                    raise RuntimeError('Following lag persisted for 1 s; stopped instead of advancing target')
            else:
                self.lead_hold_since = None
            # A fixed Cartesian speed can require larger joint steps in a
            # sensitive posture. Back off only a valid IK continuity rejection;
            # Other failures stop this command; the session decides whether
            # a rejected candidate can be resumed explicitly by the operator.
            candidate = target.copy()
            reductions = []
            for attempt in range(5):
                scale = .5 ** attempt
                target = previous.copy()
                target[:3] += (candidate[:3] - previous[:3]) * scale
                target[3:] = limited_rotation(candidate[3:], previous[3:],
                    np.linalg.norm(rotation_vector(candidate[3:], previous[3:])) * scale)
                wire_pose = encode_pose(target)
                checked_target = decode_pose(wire_pose)
                if time.monotonic() - self.feedback_at > c['stale_s']:
                    raise RuntimeError('Feedback expired during offline motion precheck')
                try:
                    joint_options = ({'max_joint_step_deg': np.maximum(
                        np.asarray(c['joint_speed_deg_s']) * elapsed, 1e-6)}
                        if 'joint_speed_deg_s' in c else {})
                    precheck = self.prepare_motion_check().check(
                        checked_target, self.state, previous_joint_deg=self.last_joint_target,
                        **joint_options)
                    break
                except JointStepTooLarge:
                    diagnostic = dict(getattr(self.motion_check, 'diagnostics', {}))
                    reductions.append(diagnostic)
                    self.command_diagnostics['precheck'] = diagnostic
                    self.command_diagnostics['joint_step_rejections'] = reductions
                    if attempt == 4:
                        raise
                except Exception:
                    self.command_diagnostics['precheck'] = getattr(self.motion_check, 'diagnostics', {})
                    raise
            self.command_diagnostics['joint_step_scale'] = scale
            self.command_diagnostics['precheck'] = precheck
            self.command_diagnostics['sent_m_xyzw'] = checked_target.tolist()
            if time.monotonic() - self.feedback_at > self.config['stale_s']:
                raise RuntimeError('Feedback expired during offline motion precheck')
            self.active = True  # A failed send may still have delivered bytes; close must attempt stop.
            self.client.send({'command': 'movep_canfd', 'pose': wire_pose, 'follow': False})
            self.last_joint_target = np.array(precheck['ik_joint_deg'], dtype=float)
            self.last_target = checked_target.copy()
            self.last_command = time.monotonic()
            self.active = True

    def telemetry(self):
        return {'backend': 'realman', 'allow_motion': self.allow_motion, 'active': self.active,
                'fault': self.fault, 'identity': self.info, 'command': self.command_diagnostics, 'gripper': {'present': None, 'configured': False},
                'velocity_feedback_available': False, 'motion_verified': False}

    def close(self):
        self.done.set()
        try:
            self.stop()
        finally:
            self.closed = True
            if self.watchdog is not None:
                self.watchdog.join(timeout=1)
            self.client.close()
            self.lease.close()
