"""MuJoCo torque-control backend with one 4 ms physical step per control tick."""
from copy import deepcopy
from dataclasses import asdict
import hashlib
import json
import time

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from embodied.devices.robots.arx5.kinematics import X5Model, JointTargetController
from embodied.devices.robots.arx5.profile import RobotConfig
from embodied.contracts.control import ControlCommand, RobotState, Mode


class MujocoBackend:
    def __init__(self, config: RobotConfig, *, controller_factory=JointTargetController):
        if config.control_hz != 250:
            raise ValueError('X5 control period must be 4 ms (250 Hz)')
        self.config = config
        self.kinematics = X5Model(config)
        self.model = self.kinematics.model
        self.data = mujoco.MjData(self.model)
        self.controller = controller_factory(config, self.kinematics)
        self._controller_factory = controller_factory
        self.data.qpos[:6] = config.park_q
        self.data.qpos[6:8] = config.gripper_max_width / 2
        mujoco.mj_forward(self.model, self.data)
        self._state_spec = mujoco.mjtState.mjSTATE_INTEGRATION
        self._model_id = config.model_id
        self._configuration_id = hashlib.sha256(json.dumps(
            {**asdict(config), 'model_path': '', 'tool_path': '', 'model_id': self._model_id},
            sort_keys=True).encode()).hexdigest()
        ratio = .004 / self.model.opt.timestep
        self._substeps = round(ratio)
        if self._substeps < 1 or not np.isclose(ratio, self._substeps):
            raise ValueError('model timestep must divide 4 ms exactly')
        self._closed = False
        self._jaw_bodies = {mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
                            for name in ('jaw_left', 'jaw_right')}
        self._robot_bodies = set(self.model.jnt_bodyid[:8].tolist())
        jaw_geoms = np.isin(self.model.geom_bodyid, list(self._jaw_bodies))
        self.model.geom_contype[jaw_geoms] = 1
        self.model.geom_conaffinity[jaw_geoms] = 1
        # MuJoCo caches the OR of geom masks per body for broad-phase filtering.
        self.model.body_contype[list(self._jaw_bodies)] = 1
        self.model.body_conaffinity[list(self._jaw_bodies)] = 1
        self._gripper_effort_max = float(np.min(np.abs(self.model.actuator_forcerange[6:, 0])))
        self._runtime_parameters = {
            'backend': 'sim',
            'robot_config': asdict(config),
            'mujoco_version': mujoco.__version__,
            'model_runtime': {
                'timestep_s': float(self.model.opt.timestep),
                'substeps_per_control_tick': self._substeps,
                'gravity': self.model.opt.gravity.tolist(),
                'tcp_link': self.kinematics.tcp_link,
                'active_joint_kp': list(config.kp),
                'active_joint_kd': list(config.kd),
                'active_joint_torque_limits': list(config.torque_max),
                'active_gripper_kp': 80.0,
                'active_gripper_kd': 1.0,
                'gripper_collision_policy': 'Both jaw geometries use collision mask 1 for physical contact evidence',
                'actuators_initial': [
                    {
                        'name': mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, index),
                        'gain_type': int(self.model.actuator_gaintype[index]),
                        'bias_type': int(self.model.actuator_biastype[index]),
                        'gain_parameters': self.model.actuator_gainprm[index].tolist(),
                        'bias_parameters': self.model.actuator_biasprm[index].tolist(),
                        'force_limited': bool(self.model.actuator_forcelimited[index]),
                        'force_range': self.model.actuator_forcerange[index].tolist(),
                        'control_limited': bool(self.model.actuator_ctrllimited[index]),
                        'control_range': self.model.actuator_ctrlrange[index].tolist(),
                        'gear': self.model.actuator_gear[index].tolist(),
                    }
                    for index in range(self.model.nu)
                ],
            },
            'telemetry_sources': {
                'command': 'target references from the last completed step; command_time_s is caller monotonic time; FreeDrive uses pre-step measured q and gripper width with zero gains',
                'applied_kp_kd': 'PD gains set in MuJoCo affine actuator bias; FreeDrive gains are zero',
                'user_feedforward_torque': 'zero user-supplied feedforward; excludes separately computed model gravity',
                'reported_joint_torque': 'unavailable in simulation; no SDK or hardware torque readback',
                'sim_actuator_torque': 'cached data.actuator_force[:6] after the last successful final mj_forward; simulated post-step output, including PD, model gravity and actuator force limits',
                'simulation_time_s': 'cached data.time from the same completed step as sim_actuator_torque',
                'model_runtime': 'constructor model snapshot; actuators_initial records loaded XML, active gains and joint torque limits are applied by each step',
            },
        }
        self._telemetry = dict.fromkeys((
            'command_time_s', 'applied_kp', 'applied_kd', 'sent_q', 'sent_dq',
            'gripper_target', 'gripper_applied_kp', 'gripper_applied_kd',
            'user_feedforward_torque', 'sdk_gravity_torque', 'reported_joint_torque',
            'reported_joint_torque_sdk_timestamp', 'reported_joint_torque_read_time_s',
            'sim_actuator_torque', 'simulation_time_s',
        ))
        self._telemetry['reported_joint_torque_source'] = 'unavailable_simulation'
        self._initial_snapshot = self.simulation_snapshot()

    def simulation_snapshot(self):
        """Copy complete MuJoCo integration state, not an in-flight control task.

        Called on the simulation worker. Rendering and disk I/O stay outside it.
        The engine/version/configuration identity prevents an incompatible restore.
        """
        state = np.empty(mujoco.mj_stateSize(self.model, self._state_spec))
        mujoco.mj_getState(self.model, self.data, state, self._state_spec)
        if not np.isfinite(state).all():
            raise ValueError('Cannot snapshot non-finite simulation state')
        feedback = self.read_state(time.monotonic())
        bodies = {}
        for index in range(1, self.model.nbody):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, index)
            if name:
                bodies[name] = [*self.data.xpos[index].tolist(),
                                *self.data.xquat[index][[1, 2, 3, 0]].tolist()]
        return {'schema': 'embodied.simulation.state', 'schema_version': 1,
                'backend': 'mujoco', 'engine_version': mujoco.__version__,
                'model_id': self._model_id, 'configuration_id': self._configuration_id,
                'state_spec': int(self._state_spec), 'state': state.tolist(),
                'observation': {'q': feedback.q.tolist(), 'dq': feedback.dq.tolist(),
                    'pose': feedback.pose.tolist(), 'gripper_width': feedback.gripper_width,
                    'simulation_time_s': float(self.data.time), 'bodies': bodies}}

    def _snapshot_state(self, snapshot):
        expected = {'schema': 'embodied.simulation.state', 'schema_version': 1,
                    'backend': 'mujoco', 'engine_version': mujoco.__version__,
                    'model_id': self._model_id, 'configuration_id': self._configuration_id,
                    'state_spec': int(self._state_spec)}
        if not isinstance(snapshot, dict) or any(snapshot.get(k) != v for k, v in expected.items()):
            raise ValueError('Simulation snapshot model, configuration or engine version mismatch')
        value = snapshot.get('state')
        size = mujoco.mj_stateSize(self.model, self._state_spec)
        if (not isinstance(value, list) or len(value) != size
                or any(type(v) not in (int, float) for v in value)):
            raise ValueError('Invalid simulation integration state')
        state = np.asarray(value, dtype=float)
        if not np.isfinite(state).all():
            raise ValueError('Simulation state must be finite')
        return state

    def simulation_restore(self, snapshot):
        """Restore physics into a new hold reference; never resume old motion."""
        state = self._snapshot_state(snapshot)  # Validate fully before mutation.
        mujoco.mj_setState(self.model, self.data, state, self._state_spec)
        mujoco.mj_forward(self.model, self.data)
        self.controller = self._controller_factory(self.config, self.kinematics)
        self.controller.update(ControlCommand(Mode.COMMAND_FOLLOWING, hold=True,
            gripper_target=float(self.data.qpos[6:8].sum()), reset_reference=True),
            self.data.qpos[:6], time.monotonic())
        for key in self._telemetry:
            if key != 'reported_joint_torque_source':
                self._telemetry[key] = None

    def simulation_reset(self):
        self.simulation_restore(self._initial_snapshot)

    def render_snapshot(self, snapshot, *, camera=-1, width=640, height=480):
        """Offline RGB rendering from recorded physics; not exposed on the tick RPC.

        Use a separate backend instance in the control Python environment. The
        current physics state is untouched; named cameras come from the model.
        """
        state = self._snapshot_state(snapshot)
        if any(type(v) is not int or not 1 <= v <= 4096 for v in (width, height)):
            raise ValueError('Render dimensions must be integers in 1..4096')
        data = mujoco.MjData(self.model)
        mujoco.mj_setState(self.model, data, state, self._state_spec)
        mujoco.mj_forward(self.model, data)
        with mujoco.Renderer(self.model, height=height, width=width) as renderer:
            renderer.update_scene(data, camera=camera)
            return renderer.render().copy()

    def runtime_parameters(self) -> dict:
        """Return the captured model settings without changing simulation state."""
        return deepcopy(self._runtime_parameters)

    def telemetry(self) -> dict:
        """Return completed-step diagnostics, never live MuJoCo views."""
        return deepcopy(self._telemetry)

    def read_state(self, now):
        site = self.kinematics.site
        pose = np.concatenate((self.data.site_xpos[site], Rotation.from_matrix(self.data.site_xmat[site].reshape(3, 3)).as_quat()))
        fault = self.controller.fault
        if not np.isfinite(self.data.qpos).all():
            fault = 'non-finite simulation state'
        return RobotState(q=self.data.qpos[:6].copy(), dq=self.data.qvel[:6].copy(), pose=pose,
                          gripper_width=float(np.clip(self.data.qpos[6:8].sum(), 0, self.config.gripper_max_width)),
                          gripper_velocity=float(self.data.qvel[6:8].sum()),
                          gripper_effort=float(np.mean(self.data.actuator_force[6:8])),
                          gripper_contact=self._gripper_contact(),
                          received_at=now, mode=self.controller.mode, fault=fault, gripper_mode=self.controller.gripper_mode)

    def _gripper_contact(self):
        touched = set()
        for contact in self.data.contact[:self.data.ncon]:
            a, b = (int(self.model.geom_bodyid[g]) for g in (contact.geom1, contact.geom2))
            if contact.dist <= 0:
                if a in self._jaw_bodies and b not in self._robot_bodies:
                    touched.add(a)
                if b in self._jaw_bodies and a not in self._robot_bodies:
                    touched.add(b)
        return touched == self._jaw_bodies

    def step(self, command, now):
        if self._closed:
            raise RuntimeError('simulation backend is closed')
        effort = command.gripper_effort
        if effort is not None and (command.mode != Mode.COMMAND_FOLLOWING or not np.isfinite(effort)
                                  or not 0 <= effort <= self._gripper_effort_max):
            raise ValueError('Gripper closing effort exceeds the simulated actuator capability')
        target = self.controller.update(command, self.data.qpos[:6], now)
        kp = np.zeros(6) if target is None else np.asarray(self.config.kp)
        kd = np.zeros(6) if target is None else np.asarray(self.config.kd)
        self.model.actuator_biasprm[:6, 1] = -kp
        self.model.actuator_biasprm[:6, 2] = -kd
        self.model.actuator_forcerange[:6] = np.column_stack((-np.asarray(self.config.torque_max), self.config.torque_max))
        gripper_enabled = self.controller.gripper_mode != Mode.FREE_DRIVE
        self.model.actuator_biasprm[6:, 1] = -80 if gripper_enabled else 0
        self.model.actuator_biasprm[6:, 2] = -1 if gripper_enabled else 0
        if effort is not None:
            self.model.actuator_biasprm[6:, 1:3] = 0
        sent = {
            'command_started_ns': time.monotonic_ns(),
            'command_time_s': float(now),
            'applied_kp': kp.tolist(),
            'applied_kd': kd.tolist(),
            'sent_q': (self.data.qpos[:6] if target is None else target).tolist(),
            'sent_dq': self.controller.dq_target.tolist(),
            'gripper_target': float(self.controller.gripper_target if gripper_enabled else
                                    np.clip(self.data.qpos[6:8].sum(), 0, self.config.gripper_max_width)),
            'gripper_applied_kp': 80.0 if gripper_enabled and effort is None else 0.0,
            'gripper_applied_kd': 1.0 if gripper_enabled and effort is None else 0.0,
            'user_feedforward_torque': [0.0] * 6,
        }
        for _ in range(self._substeps):
            # Affine bias supplies -kp*q-kd*dq; ctrl adds both desired terms.
            gravity = self.kinematics.gravity(self.data.qpos[:6])
            self.data.ctrl[:6] = gravity if target is None else gravity + kp * target + kd * self.controller.dq_target
            self.data.ctrl[6:] = 80 * self.controller.gripper_target / 2 if gripper_enabled else 0
            if effort is not None:
                self.data.ctrl[6:] = -effort
            mujoco.mj_step(self.model, self.data)
        sent['command_submitted_ns'] = time.monotonic_ns()
        # mj_step leaves Cartesian fields from the pre-integration state.
        mujoco.mj_forward(self.model, self.data)
        sent['sim_actuator_torque'] = self.data.actuator_force[:6].tolist()
        sent['simulation_time_s'] = float(self.data.time)
        self._telemetry.update(sent)

    def close(self):
        self._closed = True
