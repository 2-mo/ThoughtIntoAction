"""Unified-worker driver: local vendor IK, six-joint transmission, EG2 positions.

No XR mapping, authority, planner or device activation lives in this module.
"""
from dataclasses import replace
import json
from pathlib import Path
import time

import numpy as np

from embodied.contracts.control import Mode
from .backend import Backend as Connection, load_config, vector
from .gripper import status, require_ready, GripperNotReady, move
from .kinematics import OfflineModel, CandidateRejected, pose_error


def data_contract(config):
    return {'schema': 'embodied.joint_vectors/1', 'representation': 'absolute_joint_position',
        'joint_names': [f'joint{i}' for i in range(1, 7)], 'joint_unit': 'rad',
        'gripper': {'name': 'eg2_nominal_width_m', 'unit': 'm', 'range': [0., config.gripper_max_width]},
        'robot_type': 'rm65_hardware_eg2_4c2_nominal_width'}


class Hardware:
    def preflight(self, config):
        from .hardware_model import planning_model
        from .kinematics import load_sdk
        planning_model(config)
        load_sdk()  # Dependency check only; never opens a robot.
        return {'ready': True, 'errors': []}

    def resources(self, config):
        return ()  # Connection owns the same per-host lease as maintenance CLI.

    def open(self, config, **options):
        return Backend(config, **options)

    def capabilities(self, config, parameters):
        return {'joint_velocity_limits_rad_s': parameters['joint_velocity_limits_rad_s'],
            'cartesian_velocity_limits': parameters['cartesian_velocity_limits'],
            'gripper_velocity_max_m_s': config.gripper_velocity_max_m_s,
            'gripper': {'present': True, 'position': True, 'grasp': False,
                'contact_detection': 'unavailable', 'width_calibration': 'nominal 70 mm stroke; uncalibrated fingertips'},
            'source': {'kind': 'controller_readback',
                'description': 'Per-axis controller speed at 50%; EG2 stroke from manufacturer specification'}}

    def snapshot(self, config):
        path = Path(config.model_path)
        return {'data_contract': data_contract(config), 'model': {
            'xml_path': str(path), 'xml': path.read_text(),
            'urdf_path': str(path.with_suffix('.urdf')), 'urdf': path.with_suffix('.urdf').read_text(),
            'joint_names': data_contract(config)['joint_names'],
            'manifest': json.loads(path.with_suffix('.model.json').read_text()),
            'tool': {'tcp_link': 'tcp_link', 'offset_m': [0, 0, config.tool_offset_m],
                'gripper': 'EG2-4C2; nominal 70 mm total stroke; fingertip zero not calibrated'}},
            'controller_policy': {'kind': 'local vendor IK then movej_canfd; no controller Cartesian IK',
                'startup': config.startup_behavior, 'shutdown': config.shutdown_behavior,
                'velocity_feedback': 'encoder position finite difference; not measured drive velocity',
                'collision_check': 'none'}}

    def shutdown(self, config, backend, **callbacks):
        backend.request_stop()
        return {'policy': 'controller-stop', 'stop_requested': True, 'damping_requested': False}


class JointModel:
    def __init__(self, vendor):
        self.vendor = vendor
        self.lower, self.upper = np.deg2rad(vendor.lower), np.deg2rad(vendor.upper)

    def fk(self, q):
        return self.vendor.fk(np.rad2deg(q))

    def solve_ik(self, pose, seed, anchor=None):
        code, q = self.vendor.solve(pose, np.rad2deg(seed))
        q = np.deg2rad(q)
        if code != 0 or q.shape != (6,) or not np.isfinite(q).all():
            raise CandidateRejected(f'Local RealMan IK rejected target: result={code}')
        if np.any(q < self.lower) or np.any(q > self.upper):
            raise CandidateRejected('Local RealMan IK exceeds joint limits')
        position_error, angle_error = pose_error(self.fk(q), pose)
        if position_error > .002 or angle_error > .01:
            raise CandidateRejected('Local RealMan IK does not reproduce requested TCP')
        return q, True, max(position_error, angle_error)


class Backend:
    def __init__(self, config, *, cancel_requested=None, controller_factory=None, connection=None, vendor=None):
        if controller_factory is not None:
            raise ValueError('RealMan hardware uses its joint controller, not ARX gains or lifecycle')
        self.config, self.cancel_requested = config, cancel_requested
        self.connection = connection or Connection(load_config(config.connection_path),
            allow_motion=True, defer_motion_check=True)
        self.previous_feedback = None
        self.last_gripper_poll = float('-inf')
        self.pending_gripper = None
        self.gripper_contact = False
        self.gripper_ready = False
        self.gripper_ready_samples = 0
        self.gripper_needs_release = False
        self.gripper_initializing = False
        self.gripper_initialize_requested = False
        self.gripper_initialization_at = None
        self.gripper_notice = None
        self.stopped = False
        # Coordinator mode acknowledgements are logical states. FreeDrive and
        # Disabled acknowledge a hold; neither enables a physical drag/Home mode.
        self.mode = self.gripper_mode = Mode.COMMAND_FOLLOWING
        self._telemetry = {}
        try:
            for key in ('get_DH_data', 'get_joint_min_pos', 'get_joint_max_pos'):
                self.connection.info[key] = self.connection.client.query(key)
            self.vendor = vendor or OfflineModel(self.connection.info)
            self.kinematics = JointModel(self.vendor)
            configured_low, configured_high = np.array(config.joint_min), np.array(config.joint_max)
            if (np.any(configured_low < self.kinematics.lower-1e-6) or
                    np.any(configured_high > self.kinematics.upper+1e-6)):
                raise ValueError('Profile joint limits exceed controller readback')
            self.kinematics.lower = configured_low
            self.kinematics.upper = configured_high
            self.velocity_limits = np.minimum(config.joint_velocity_max,
                np.deg2rad(self.connection.config['joint_speed_deg_s']))
            self.linear_speed = min(config.teleop_linear_speed_m_s, self.connection.config['linear_speed_m_s'])
            self.angular_speed = min(config.teleop_angular_speed_rad_s, self.connection.config['angular_speed_rad_s'])
            self.connection.client.allow_gripper = True
            self._poll_gripper(time.monotonic(), startup=True)
            # Reuse the existing target controller's hold, joint slew and IK
            # cache. RM-specific mode conversion prevents ARX Home/damping.
            from embodied.devices.robots.arx5.kinematics import JointTargetController
            self.controller = JointTargetController(config, self.kinematics)
            self.controller.mode = self.controller.gripper_mode = Mode.COMMAND_FOLLOWING
            self.controller.gripper_target = self.width()
            first = self.connection.read_state(time.monotonic())
            if first.fault:
                raise RuntimeError(first.fault)
            position_error, angle_error = pose_error(self.kinematics.fk(first.q), first.pose)
            if position_error > .002 or angle_error > .01:
                raise ValueError('Controller DH/TCP disagree with feedback')
            self.previous_feedback = first
            self.state = self.read_state(time.monotonic())
        except BaseException:
            self.connection.close()
            raise

    def width(self):
        if not self.gripper_ready:
            return None
        pos = self.gripper['actpos']
        if not -5 <= pos <= 1005:
            raise ValueError(f'EG2 opening outside endpoint tolerance: {pos}')
        return float(np.clip(pos, 0, 1000)/1000*self.config.gripper_max_width)

    def _poll_gripper(self, now, *, startup=False):
        self.gripper = status(self.connection.client)
        self.last_gripper_poll = now
        try:
            require_ready(self.gripper)
        except GripperNotReady:
            self.gripper_ready = False
            self.gripper_ready_samples = 0
            self.gripper_needs_release = True
            self.pending_gripper = None
            self.gripper_contact = False
        else:
            if self.gripper['mode'] not in (1, 2, 3, 4, 5, 6):
                raise ValueError(f'Unknown EG2 mode: {self.gripper}')
            if not -5 <= self.gripper['actpos'] <= 1005:
                raise ValueError(f'EG2 opening outside endpoint tolerance: {self.gripper["actpos"]}')
            if startup or self.gripper_ready:
                self.gripper_ready = True
            elif self.gripper['mode'] in (1, 2, 3, 6):
                self.gripper_ready_samples += 1
                self.gripper_ready = self.gripper_ready_samples >= 2
            else:
                self.gripper_ready_samples = 0
            if self.gripper_ready:
                self.gripper_initializing = False
        notice = ('ready' if self.gripper_ready else 'unavailable')
        if notice != self.gripper_notice:
            print(f'{self.config.name}: EG2 {notice}; feedback={self.gripper}. '
                  + ('需松开夹爪控制，再重新操作。' if self.gripper_ready and self.gripper_needs_release else
                     '机械臂保持可用；夹爪宽度未知。初始化张开请使用独立维护命令；左Y/右B长按用于机械臂回位，非零故障需先处理。'
                     if not self.gripper_ready else ''), flush=True)
            self.gripper_notice = notice

    def _initialize_gripper(self, command):
        if not command.hold or command.engaged:
            raise ValueError('Gripper initialization requires an explicit arm hold')
        if self.gripper_initializing:
            return
        try:
            reply = move(self.connection.client, 'open', speed=500, initialize_open=True)
        except GripperNotReady as exc:
            print(f'{self.config.name}: 初始化未执行；夹爪返回非零未就绪/故障状态 {exc.feedback}', flush=True)
            return
        if not reply['accepted']:
            return
        self.gripper_initializing = True
        self.gripper_initialization_at = time.monotonic()
        self.gripper_ready = False
        self.gripper_ready_samples = 0
        self.gripper_needs_release = True
        self.pending_gripper = None
        self.gripper_contact = False
        self.last_gripper_poll = float('-inf')
        print(f'{self.config.name}: 已接受操作者的EG2初始化张开请求；等待有效反馈，不自动闭合。', flush=True)

    def runtime_parameters(self):
        return {'backend': 'real', 'robot_config': self.config.__dict__,
            'data_contract': data_contract(self.config), 'controller': self.connection.info,
            'joint_velocity_limits_rad_s': self.velocity_limits.tolist(),
            'cartesian_velocity_limits': {'tcp_speed': self.linear_speed, 'angular_speed': self.angular_speed},
            'velocity_feedback': 'finite_difference_of_encoder_positions',
            'gripper_startup_ready': self.gripper_ready,
            'gripper_width': {'source': 'manufacturer_nominal_stroke', 'raw_range': [0, 1000],
                'endpoint_tolerance_raw': 5, 'stroke_m': .07, 'fingertip_calibrated': False,
                'source_url': 'https://www.inspire-robots.com/dexterous%20hands/eg2-4c-series/'}}

    def read_state(self, now):
        state = self.connection.read_state(now)
        previous = self.previous_feedback
        dt = state.received_at-previous.received_at if previous is not None else 0.
        if previous is None or dt <= 0:
            raise RuntimeError('Encoder finite difference requires two increasing feedback timestamps')
        dq = (state.q-previous.q)/dt
        self.previous_feedback = state
        interval = .08 if self.gripper_ready or self.gripper_initializing else 1.
        if now-self.last_gripper_poll >= interval:
            was_ready = self.gripper_ready
            self._poll_gripper(now)
            if self.gripper_ready and not was_ready:
                self.controller.gripper_target = self.width()
        if (self.gripper_initializing and self.gripper_initialization_at is not None
                and now-self.gripper_initialization_at > 5.):
            self.gripper_initializing = False
            print(f'{self.config.name}: 初始化后5秒未确认就绪；停止自动等待，只读复查，需操作者重新请求。', flush=True)
        self.state = replace(state, dq=dq, gripper_width=self.width(),
            mode=self.mode, gripper_mode=self.gripper_mode)
        return self.state

    def step(self, command, now, *, cancel_requested=None, feedback_validator=None):
        if self.stopped or (cancel_requested and cancel_requested()) or (
                self.cancel_requested and self.cancel_requested()):
            raise InterruptedError('RealMan control stopped')
        state = self.state
        if feedback_validator:
            feedback_validator(state)
        if state.fault or self.connection.fault or time.monotonic()-state.received_at >= .1:
            raise RuntimeError(state.fault or self.connection.fault or 'Stale RealMan feedback')
        started = time.monotonic_ns()
        if command.gripper_effort is not None:
            raise ValueError('EG2 calibrated force control is not available')
        if command.gripper_initialize_open and (not command.hold or command.engaged):
            raise ValueError('Gripper initialization requires an explicit arm hold')
        initialize = command.gripper_initialize_open and not self.gripper_initialize_requested
        self.gripper_initialize_requested = command.gripper_initialize_open
        # ARX Disabled means Home and FreeDrive means damping. Neither is an
        # implicit RealMan action: both become a measured hold here.
        holding = command.hold or command.reset_reference or command.mode != Mode.COMMAND_FOLLOWING
        self.mode, self.gripper_mode = command.mode, command.gripper_mode or command.mode
        effective = replace(command, mode=Mode.COMMAND_FOLLOWING,
            gripper_mode=Mode.COMMAND_FOLLOWING, hold=holding, reset_reference=holding,
            gripper_target=command.gripper_target if self.gripper_mode == Mode.COMMAND_FOLLOWING else None)
        if not self.gripper_ready or command.gripper_initialize_open:
            effective = replace(effective, gripper_target=None)
        elif (self.gripper_needs_release and command.hold
              and command.gripper_target == state.gripper_width):
            effective = replace(effective, gripper_target=None)
        if not holding and command.joint_target is not None:
            requested = vector(command.joint_target, 6)
            if np.any(requested < self.kinematics.lower) or np.any(requested > self.kinematics.upper):
                raise ValueError('Requested joint target exceeds controller limits')
        step_limit = self.velocity_limits/self.config.control_hz
        if effective.joint_step_limit is not None:
            step_limit = np.minimum(step_limit, np.broadcast_to(effective.joint_step_limit, (6,)))
        effective = replace(effective, joint_step_limit=step_limit)
        previous = (self.controller.q_target.copy() if self.controller.q_target is not None else state.q.copy())
        try:
            q = self.controller.update(effective, state.q, now)
        except CandidateRejected:
            self.connection.stop()
            self.controller.update(replace(effective, hold=True, reset_reference=True), state.q, now)
            self.controller.ik_converged = False
            self.controller.ik_rejections += 1
            self.controller.ik_residual = float('inf')
            q = state.q.copy()
            holding = True
        if self.controller.fault:
            raise ValueError(self.controller.fault)
        q = vector(q, 6)
        if np.any(q < self.kinematics.lower) or np.any(q > self.kinematics.upper):
            raise ValueError('Joint target outside controller limits')
        if not holding and np.any(np.abs(q-previous) > step_limit+2e-5):
            raise ValueError('Joint command exceeds per-axis cycle speed budget')
        if not holding:
            q = self._tcp_speed_bound(q, previous, planned=command.planned)
            self.controller.q_target = q.copy()
        if not holding:
            # Check the joint solution that will actually be sent. Never turn
            # a local IK rejection into a controller Cartesian command.
            code, distance = self.vendor.singularity(np.rad2deg(q))
            if not np.isfinite(distance):
                raise ValueError('Invalid vendor singularity result')
            if code:
                self.connection.stop()
                self.controller.update(replace(effective, hold=True, reset_reference=True), state.q, now)
                q, holding = state.q.copy(), True
        if holding:
            self.connection.stop()
        else:
            if (self.stopped or (cancel_requested and cancel_requested()) or
                    (self.cancel_requested and self.cancel_requested())):
                raise InterruptedError('RealMan control stopped before joint submission')
            if feedback_validator:
                feedback_validator(state)
            if time.monotonic()-state.received_at >= .1:
                raise RuntimeError('RealMan feedback expired during target calculation')
            wire = np.rint(np.rad2deg(q)*1000).astype(int).tolist()
            with self.connection.lock:
                self.connection.active = True
                self.connection.client.send({'command': 'movej_canfd', 'joint': wire, 'follow': False})
                self.connection.last_command = time.monotonic()
                self.connection.last_joint_target = np.rad2deg(q)
        if initialize:
            if (self.stopped or (cancel_requested and cancel_requested()) or
                    (self.cancel_requested and self.cancel_requested())):
                raise InterruptedError('RealMan stopped before gripper initialization')
            self._initialize_gripper(command)
        else:
            self._gripper_target(effective.gripper_target, now)
        self._telemetry = {'sent_q': q.tolist(), 'sent_dq': self.controller.dq_target.tolist(),
            'applied_kp': [0.]*6, 'applied_kd': [0.]*6, 'user_feedforward_torque': [0.]*6,
            'gripper_target': self.controller.gripper_target if self.gripper_ready and not self.gripper_needs_release else None,
            'gripper_applied_kp': 0., 'gripper_applied_kd': 0.,
            'command_started_ns': started, 'command_submitted_ns': time.monotonic_ns(),
            'feedback_velocity_source': 'encoder_finite_difference', 'gripper_raw': dict(self.gripper),
            'gripper_ready': self.gripper_ready, 'gripper_needs_release': self.gripper_needs_release}

    def _tcp_speed_bound(self, q, previous, *, planned):
        origin = self.kinematics.fk(previous)
        linear = self.linear_speed/self.config.control_hz
        angular = self.angular_speed/self.config.control_hz
        for _ in range(8):
            distance, angle = pose_error(self.kinematics.fk(q), origin)
            fraction = min(1., linear/max(distance, 1e-12), angular/max(angle, 1e-12))
            if fraction >= 1.-1e-9:
                return q
            if planned:
                raise ValueError('Planned curve exceeds controller TCP speed capability')
            q = previous + (q-previous)*fraction*.99
        raise CandidateRejected('TCP speed bound could not resolve joint step')

    def _gripper_target(self, width, now):
        if not self.gripper_ready:
            return
        if self.gripper_needs_release:
            if width is None:
                self.gripper_needs_release = False
            return  # Never replay a target that was held across an outage.
        if width is None:
            return
        maximum = self.config.gripper_max_width
        # Native ParallelMit uses float32: 70 mm becomes 0.070000000298 m.
        # Admit that encoded endpoint only, then return to the exact range.
        encoded_maximum = float(np.float32(maximum))
        if not np.isfinite(width) or not 0 <= width <= max(maximum, encoded_maximum):
            raise ValueError('Invalid EG2 nominal width target')
        raw = round(min(width, maximum)/maximum*1000)
        if self.gripper['mode'] not in (1, 2, 3, 4, 5, 6):
            raise RuntimeError(f'Unknown EG2 mode: {self.gripper}')
        if self.gripper['mode'] == 6:
            self.gripper_contact = True
            self.pending_gripper = None
        if raw > self.gripper['actpos']+3:
            self.gripper_contact = False
        if self.gripper_contact and raw < self.gripper['actpos']:
            return
        if self.gripper['mode'] in (4, 5):
            if self.pending_gripper is not None and now-self.pending_gripper[1] > 5.:
                raise RuntimeError('EG2 segment still moving after 5 s')
            return  # No queue: only the current owner target can schedule another segment.
        if self.pending_gripper is not None:
            target, sent = self.pending_gripper
            if abs(self.gripper['actpos']-target) > 3 and self.gripper['mode'] != 6:
                if now-sent > 5.:
                    raise RuntimeError('EG2 position feedback did not reach the accepted segment')
                return
            self.pending_gripper = None
        if abs(raw-self.gripper['actpos']) <= 3:
            return
        target = int(np.clip(raw, self.gripper['actpos']-100, self.gripper['actpos']+100))
        target = max(1, min(1000, target))
        reply = self.connection.client.exchange({'command': 'set_gripper_position',
            'position': target, 'block': False}, 'set_gripper')
        if reply.get('state') is not True:
            raise RuntimeError(f'EG2 position rejected: {reply}')
        self.pending_gripper = target, now

    def telemetry(self):
        return dict(self._telemetry)

    def request_stop(self):
        self.stopped = True
        self.connection.stop()

    def close(self):
        self.connection.close()
