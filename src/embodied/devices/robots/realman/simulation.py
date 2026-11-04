"""RM65 simulation plugin. Never imports the RealMan TCP backend or SDK."""
from copy import deepcopy
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import time

import mujoco
import numpy as np

from embodied.contracts.control import Mode, RobotState
from .model import Model, vector


def data_contract(config):
    return {'schema': 'embodied.joint_vectors/1', 'representation': 'absolute_joint_position',
            'joint_names': [f'joint{i}' for i in range(1, 7)], 'joint_unit': 'rad',
            'gripper': {'name': 'gripper_m', 'unit': 'm', 'range': [0., config.gripper_max_width]},
            'robot_type': 'rm65_sim_simplified_gripper'}


class Simulation:
    def snapshot(self, config):
        path = Path(config.model_path)
        return {'data_contract': data_contract(config), 'model': {
            'xml_path': str(path), 'xml': path.read_text(), 'urdf_path': str(path.with_suffix('.urdf')),
            'urdf': path.with_suffix('.urdf').read_text(), 'joint_names': data_contract(config)['joint_names'],
            'manifest': json.loads(path.with_suffix('.model.json').read_text()),
            'tool': {'tcp_link': 'tcp_link', 'offset_m': [0, 0, config.tool_offset_m],
                     'gripper': 'simplified parallel jaws; assumed range, no EG2 calibration'}},
            'controller_policy': {'kind': 'torque PD with model bias compensation',
                                  'collision_check': 'none; robot contacts disabled',
                                  'hardware_equivalence': False,
                                  'gripper': 'position only; no calibrated force or grasp feedback'}}

    def preflight(self, config):
        Model(config)
        return {'ready': True, 'errors': []}

    def resources(self, config):
        return ()

    def capabilities(self, config, parameters):
        return {'joint_velocity_limits_rad_s': list(config.joint_velocity_max),
                'gripper_velocity_max_m_s': config.gripper_velocity_max_m_s,
                'gripper': {'present': True, 'position': True, 'grasp': False,
                            'contact_detection': 'unavailable'},
                'source': {'kind': 'simulation_configuration',
                           'description': 'Conservative simulation limits; not controller readback'}}

    def open(self, config, *, cancel_requested=None, **options):
        if options:
            raise ValueError('RM65 simulation owns its PD controller; unsupported controller override')
        return Backend(config)

    def shutdown(self, config, backend, **callbacks):
        return {'policy': 'simulation-close', 'stop_requested': True}


class Backend:
    def __init__(self, config):
        self.config = config
        self.kinematics = Model(config)
        self.model = self.kinematics.model
        self.data = mujoco.MjData(self.model)
        self.data.qpos[:6] = config.park_q
        self.data.qpos[6:] = config.gripper_max_width / 2
        mujoco.mj_forward(self.model, self.data)
        ratio = 1. / config.control_hz / self.model.opt.timestep
        self.substeps = round(ratio)
        if self.substeps < 1 or not np.isclose(ratio, self.substeps):
            raise ValueError('Model timestep must divide the configured control period')
        self.mode = Mode.COMMAND_FOLLOWING
        self.holding = True
        self.target = self.data.qpos[:6].copy()
        self.width_target = config.gripper_max_width
        self._telemetry = {}
        self.closed = False
        self.state_spec = mujoco.mjtState.mjSTATE_INTEGRATION
        self.identity = {'schema': 'embodied.simulation.state', 'schema_version': 1, 'backend': 'mujoco',
                         'engine_version': mujoco.__version__, 'model_id': config.model_id,
                         'configuration_id': hashlib.sha256(json.dumps({**asdict(config), 'model_path': ''}, sort_keys=True).encode()).hexdigest(),
                         'state_spec': int(self.state_spec)}
        self.initial = self.simulation_snapshot()

    def runtime_parameters(self):
        return {'backend': 'sim', 'mujoco_version': mujoco.__version__,
                'robot_config': asdict(self.config), 'data_contract': data_contract(self.config),
                'model_runtime': {'timestep_s': self.model.opt.timestep, 'substeps': self.substeps},
                'limitations': Simulation().snapshot(self.config)['controller_policy']}

    def read_state(self, now):
        q, dq = self.data.qpos, self.data.qvel
        fault = '' if np.isfinite(q).all() and np.isfinite(dq).all() else 'non-finite simulation state'
        return RobotState(q[:6].copy(), dq[:6].copy(), self.kinematics.pose(self.data),
                          float(np.sum(q[6:])), now, self.mode, fault,
                          gripper_mode=self.mode, gripper_velocity=float(np.sum(dq[6:])))

    def step(self, command, now):
        if self.closed:
            raise RuntimeError('Simulation is closed')
        started = time.monotonic_ns()
        if command.gripper_effort is not None:
            raise ValueError('Simplified RM65 gripper supports position only')
        previous = self.target.copy()
        target = previous.copy()
        dq = np.zeros(6)
        transition = self.mode != command.mode
        holding = command.hold or command.reset_reference or command.mode in (Mode.DISABLED, Mode.FREE_DRIVE)
        if holding:
            if not self.holding or transition or command.reset_reference:
                target = self.data.qpos[:6].copy()
        elif command.joint_target is not None:
            target = vector(command.joint_target, 6, 'target').copy()
            if command.joint_velocity_target is not None:
                dq = vector(command.joint_velocity_target, 6, 'target velocity')
        elif command.pose_target is not None:
            target, ok, residual = self.kinematics.solve_ik(command.pose_target, self.data.qpos[:6])
            if not ok:
                # A streamed Quest target may leave the reachable workspace.
                # Keep the last submitted joint reference, as the existing
                # teleop backend does for rejected IK, and allow the next frame.
                target = previous
                if now - getattr(self, '_last_ik_warning_at', float('-inf')) >= 1.:
                    print(f'RM65 仿真目标逆解未收敛（残差 {residual:.6f}）；保持上一关节目标。', flush=True)
                    self._last_ik_warning_at = now
        if np.any(target < self.kinematics.lower) or np.any(target > self.kinematics.upper):
            raise ValueError('Joint target exceeds model limits')
        if np.any(np.abs(dq) > np.asarray(self.config.joint_velocity_max) * 1.000001):
            raise ValueError('Joint velocity target exceeds configured limits')
        if not command.planned and not holding:
            step = command.joint_step_limit if command.joint_step_limit is not None else self.config.joint_step_limit
            step = np.broadcast_to(np.asarray(step, dtype=float), (6,))
            if not np.isfinite(step).all() or np.any(step <= 0):
                raise ValueError('Stream step limits must be positive')
            step = np.minimum(step, np.asarray(self.config.joint_velocity_max) / self.config.control_hz)
            delta = target - previous
            if command.pose_target is not None:
                # Preserve the IK joint direction. Clipping each axis separately
                # can turn an upward TCP request into a downward first motion.
                target = previous + delta / max(1., float(np.max(np.abs(delta) / step)))
            else:
                target = previous + np.clip(delta, -step, step)
        width = self.width_target
        if command.gripper_target is not None:
            requested = float(command.gripper_target)
            if not np.isfinite(requested) or not 0 <= requested <= self.config.gripper_max_width:
                raise ValueError('Invalid simplified gripper width')
            bound = self.config.gripper_velocity_max_m_s / self.config.control_hz
            width += float(np.clip(requested - width, -bound, bound))
        self.target, self.width_target = target, width
        self.mode, self.holding = command.mode, holding
        kp, kd = np.asarray(self.config.kp), np.asarray(self.config.kd)
        if self.mode == Mode.FREE_DRIVE:
            kp, kd = np.zeros(6), np.zeros(6)
        submitted = time.monotonic_ns()
        for _ in range(self.substeps):
            torque = kp * (target - self.data.qpos[:6]) + kd * (dq - self.data.qvel[:6]) + self.data.qfrc_bias[:6]
            self.data.ctrl[:6] = np.clip(torque, -np.asarray(self.config.torque_max), self.config.torque_max)
            jaw = self.config.gripper_kp * (width / 2 - self.data.qpos[6:]) - self.config.gripper_kd * self.data.qvel[6:] + self.data.qfrc_bias[6:]
            self.data.ctrl[6:] = np.clip(jaw, -self.config.gripper_force_limit_n, self.config.gripper_force_limit_n)
            mujoco.mj_step(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        self._telemetry = {'sent_q': target.tolist(), 'sent_dq': dq.tolist(),
            'applied_kp': kp.tolist(), 'applied_kd': kd.tolist(), 'user_feedforward_torque': [0.] * 6,
            'gripper_target': width, 'gripper_applied_kp': self.config.gripper_kp,
            'gripper_applied_kd': self.config.gripper_kd,
            'command_started_ns': started, 'command_submitted_ns': submitted,
            'command_time_s': now, 'simulation_time_s': float(self.data.time),
            'sim_actuator_torque': self.data.actuator_force[:6].tolist()}

    def telemetry(self):
        return deepcopy(self._telemetry)

    def simulation_snapshot(self):
        value = np.empty(mujoco.mj_stateSize(self.model, self.state_spec))
        mujoco.mj_getState(self.model, self.data, value, self.state_spec)
        if not np.isfinite(value).all():
            raise ValueError('Non-finite simulation state')
        state = self.read_state(time.monotonic())
        bodies = {self.model.body(i).name: [*self.data.xpos[i], *self.data.xquat[i][[1, 2, 3, 0]]]
                  for i in range(1, self.model.nbody) if self.model.body(i).name}
        return {**self.identity, 'state': value.tolist(), 'observation': {
            'q': state.q.tolist(), 'dq': state.dq.tolist(), 'pose': state.pose.tolist(),
            'gripper_width': state.gripper_width, 'simulation_time_s': float(self.data.time), 'bodies': bodies}}

    def simulation_restore(self, snapshot):
        if not isinstance(snapshot, dict) or any(snapshot.get(k) != v for k, v in self.identity.items()):
            raise ValueError('Simulation snapshot identity mismatch')
        value = vector(snapshot.get('state'), mujoco.mj_stateSize(self.model, self.state_spec), 'integration state')
        candidate = mujoco.MjData(self.model)
        mujoco.mj_setState(self.model, candidate, value, self.state_spec)
        if (candidate.time < 0 or np.any(candidate.qpos < self.model.jnt_range[:, 0] - .001)
                or np.any(candidate.qpos > self.model.jnt_range[:, 1] + .001)):
            raise ValueError('Simulation snapshot position/time is outside model bounds')
        mujoco.mj_forward(self.model, candidate)
        if not np.isfinite(candidate.qacc).all():
            raise ValueError('Invalid restored simulation state')
        self.data = candidate
        self.target = self.data.qpos[:6].copy()
        self.width_target = float(np.clip(np.sum(self.data.qpos[6:]), 0, self.config.gripper_max_width))
        self.holding, self.mode = True, Mode.COMMAND_FOLLOWING
        self._telemetry = {}
        return self.simulation_snapshot()

    def simulation_reset(self):
        return self.simulation_restore(self.initial)

    def close(self):
        self.closed = True
