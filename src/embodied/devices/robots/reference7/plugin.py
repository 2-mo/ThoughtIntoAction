"""Real MuJoCo dynamics for a seven-axis, gripper-free reference robot."""
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


def data_contract():
    return {'schema': 'embodied.joint_vectors/1', 'representation': 'absolute_joint_position',
            'joint_names': [f'joint{i}' for i in range(1, 8)], 'joint_unit': 'rad',
            'gripper': None, 'robot_type': 'reference7'}


class Simulation:
    def snapshot(self, config):
        path = Path(config.model_path)
        return {'data_contract': data_contract(), 'model': {
            'xml_path': str(path), 'xml': path.read_text(), 'urdf_path': str(path.with_suffix('.urdf')),
            'urdf': path.with_suffix('.urdf').read_text(), 'joint_names': data_contract()['joint_names'],
            'manifest': {'schema_version': 1, 'model_id': config.model_id, 'tcp_link': 'tcp_link',
                         'calibration_status': 'authored reference geometry; not hardware calibrated'},
            'tool': {'tcp_link': 'tcp_link', 'gripper': None}},
            'controller_policy': {'kind': 'torque PD with model gravity compensation',
                                  'collision_check': 'none', 'hardware_equivalence': False}}

    def preflight(self, config):
        Model(config)
        return {'ready': True, 'errors': []}

    def resources(self, config):
        return ()

    def capabilities(self, config, parameters):
        return {'joint_velocity_limits_rad_s': list(config.joint_velocity_max),
                'gripper': {'present': False},
                'source': {'kind': 'simulation_configuration', 'description': 'Reference7 configured limits'}}

    def open(self, config, *, cancel_requested=None, **options):
        if options:
            raise ValueError('Reference7 owns its PD controller; unsupported controller override')
        return Backend(config)

    def shutdown(self, config, backend, **callbacks):
        return {'policy': 'simulation-close', 'stop_requested': True}


class Backend:
    def __init__(self, config):
        self.config = config
        self.kinematics = Model(config)
        self.model = self.kinematics.model
        self.data = mujoco.MjData(self.model)
        self.data.qpos[:] = config.park_q
        mujoco.mj_forward(self.model, self.data)
        ratio = 1. / config.control_hz / self.model.opt.timestep
        self.substeps = round(ratio)
        if self.substeps < 1 or not np.isclose(ratio, self.substeps):
            raise ValueError('Model timestep must divide the configured control period')
        self.mode = Mode.COMMAND_FOLLOWING
        self.holding = True
        self.target = self.data.qpos.copy()
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
                'robot_config': asdict(self.config), 'data_contract': data_contract(),
                'model_runtime': {'timestep_s': self.model.opt.timestep, 'substeps': self.substeps}}

    def read_state(self, now):
        return RobotState(self.data.qpos.copy(), self.data.qvel.copy(), self.kinematics.pose(self.data),
                          0., now, self.mode)

    def step(self, command, now):
        if self.closed:
            raise RuntimeError('Simulation is closed')
        started = time.monotonic_ns()
        previous_target = self.target.copy()
        if command.gripper_target not in (None, 0):
            raise ValueError('Reference7 has no gripper')
        dq = np.zeros(7)
        transition = self.mode != command.mode
        self.mode = command.mode
        if command.hold or command.reset_reference or self.mode == Mode.DISABLED:
            if not self.holding or transition or command.reset_reference:
                self.target = self.data.qpos.copy()
            self.holding = True
        elif command.joint_target is not None:
            self.target = vector(command.joint_target, 7, 'target').copy()
            if command.joint_velocity_target is not None:
                dq = vector(command.joint_velocity_target, 7, 'target velocity')
            self.holding = False
        elif command.pose_target is not None:
            target, ok, _ = self.kinematics.solve_ik(command.pose_target, self.data.qpos)
            if not ok:
                raise ValueError('Reference7 target is unreachable')
            self.target, self.holding = target, False
        if np.any(self.target < self.kinematics.lower) or np.any(self.target > self.kinematics.upper):
            raise ValueError('Joint target exceeds model limits')
        if np.any(np.abs(dq) > np.asarray(self.config.joint_velocity_max) * 1.000001):
            raise ValueError('Joint velocity target exceeds configured limits')
        if not command.planned and not self.holding and self.mode != Mode.FREE_DRIVE:
            step = command.joint_step_limit if command.joint_step_limit is not None else self.config.joint_step_limit
            step = np.broadcast_to(np.asarray(step, dtype=float), (7,))
            if not np.isfinite(step).all() or np.any(step <= 0):
                raise ValueError('Stream step limits must be positive')
            step = np.minimum(step, np.asarray(self.config.joint_velocity_max) / self.config.control_hz)
            self.target = previous_target + np.clip(self.target - previous_target, -step, step)
        kp, kd = np.asarray(self.config.kp), np.asarray(self.config.kd)
        if self.mode == Mode.FREE_DRIVE:
            kp, kd = np.zeros(7), np.zeros(7)
            self.target = self.data.qpos.copy()
        submitted = time.monotonic_ns()
        for _ in range(self.substeps):
            torque = kp * (self.target - self.data.qpos) + kd * (dq - self.data.qvel) + self.data.qfrc_bias
            self.data.ctrl[:] = np.clip(torque, -np.asarray(self.config.torque_max), self.config.torque_max)
            mujoco.mj_step(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        self._telemetry = {'sent_q': self.target.tolist(), 'sent_dq': dq.tolist(),
            'applied_kp': kp.tolist(), 'applied_kd': kd.tolist(), 'user_feedforward_torque': [0.] * 7,
            'gripper_target': 0., 'gripper_applied_kp': 0., 'gripper_applied_kd': 0.,
            'command_started_ns': started, 'command_submitted_ns': submitted,
            'command_time_s': now, 'simulation_time_s': float(self.data.time),
            'sim_actuator_torque': self.data.actuator_force.tolist()}

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
            'gripper_width': 0., 'simulation_time_s': float(self.data.time), 'bodies': bodies}}

    def simulation_restore(self, snapshot):
        if not isinstance(snapshot, dict) or any(snapshot.get(k) != v for k, v in self.identity.items()):
            raise ValueError('Simulation snapshot identity mismatch')
        value = vector(snapshot.get('state'), mujoco.mj_stateSize(self.model, self.state_spec), 'integration state')
        mujoco.mj_setState(self.model, self.data, value, self.state_spec)
        mujoco.mj_forward(self.model, self.data)
        self.target, self.holding, self.mode = self.data.qpos.copy(), True, Mode.COMMAND_FOLLOWING
        self._telemetry = {}
        return self.simulation_snapshot()

    def simulation_reset(self):
        return self.simulation_restore(self.initial)

    def close(self):
        self.closed = True
