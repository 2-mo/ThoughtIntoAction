#!/usr/bin/env python3
"""Offline regression checks. Every CAN controller constructor is replaced."""
from dataclasses import replace
from pathlib import Path
import json
import signal
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import numpy as np
import mujoco
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / 'src'), str(ROOT)]
import arx5_interface as sdk
from embodied.devices.robots.arx5.kinematics import JointTargetController, X5Model
from embodied.devices.robots.arx5.real import RealBackend
from embodied.control.settings import load_settings
from embodied.devices.robots.arx5.profile import RobotConfig
from embodied.contracts.control import ControlCommand, Mode


class FakeHardware:
    def __init__(self, robot, control, interface):
        self.robot, self.control = robot, control
        self.state = sdk.JointState(6)
        self.state.pos()[:] = [0, 1.1, 1.3, 0, 0, 0]
        self.state.gripper_pos = .041
        self.state.timestamp = .020
        self.gain = sdk.Gain(6)
        self.command = None
        self.sent = []
        self.damping_calls = 0

    def get_joint_state(self):
        return self.state

    def get_timestamp(self):
        return .030

    def set_joint_cmd(self, command):
        self.command = command

    def set_gain(self, gain):
        self.gain = gain

    def send_recv_once(self):
        self.sent.append((self.command, self.gain))

    def get_joint_cmd(self):
        return self.command

    def get_gain(self):
        return self.gain

    def set_to_damping(self):
        self.damping_calls += 1


class NativeInterfaceChecks(unittest.TestCase):
    def setUp(self):
        self.patch = patch.object(sdk, 'Arx5JointController', FakeHardware)
        self.patch.start()
        self.addCleanup(self.patch.stop)

    def backend(self, **kwargs):
        backend = RealBackend(load_settings().robot, **kwargs)
        self.addCleanup(backend.close)
        return backend

    def test_calibrated_inertials_and_fingertip_fk(self):
        config = load_settings().robot
        model = X5Model(config)
        source = ET.parse(ROOT / 'assets/robots/x5/runtime/x5_2025.source.urdf').getroot()
        actual = ET.parse(model.urdf_path).getroot()
        self.assertEqual(float(actual.find("link[@name='link6']/inertial/mass").get('value')), .481)
        for name in ['base_link'] + [f'link{i}' for i in range(1, 7)]:
            for field in ('origin', 'mass', 'inertia'):
                key = f"link[@name='{name}']/inertial/{field}"
                self.assertEqual(source.find(key).attrib, actual.find(key).attrib)
        np.testing.assert_allclose(model.model.site('tcp').pos, [0, 0, 0])
        fixed = actual.find("joint[@name='tool_tcp_fixed']/origin")
        np.testing.assert_allclose(np.fromstring(fixed.get('xyz'), sep=' '), [.145, 0, 0])
        solver = sdk.Arx5Solver(str(model.urdf_path), 6, model.lower, model.upper,
                               'base_link', 'tcp_link', np.array([0., 0., -9.807]))
        reference = sdk.Arx5Solver(str(ROOT / 'assets/robots/x5/runtime/x5_2025.source.urdf'), 6,
                                  model.lower, model.upper, 'base_link', 'eef_link',
                                  np.array([0., 0., -9.807]))
        for q in (np.zeros(6), np.array([.2, 1.1, 1.3, -.3, .4, -.2]),
                  np.array([-.4, .6, 1.7, .8, -.5, .3])):
            np.testing.assert_allclose(solver.forward_kinematics(q), reference.forward_kinematics(q), atol=1e-10)
            np.testing.assert_allclose(model.fk(q)[:3], solver.forward_kinematics(q)[:3], atol=1e-9)
            np.testing.assert_allclose(Rotation.from_quat(model.fk(q)[3:]).as_matrix(),
                                       Rotation.from_euler('xyz', solver.forward_kinematics(q)[3:]).as_matrix(), atol=1e-9)
            np.testing.assert_allclose(solver.inverse_dynamics(q, np.zeros(6), np.zeros(6)),
                                       reference.inverse_dynamics(q, np.zeros(6), np.zeros(6)), atol=1e-10)
            # The two 1 g visual jaws belong only to the simulator.
            model_gravity = model.gravity(q)
            jaws = np.zeros(6)
            for name in ('jaw_left', 'jaw_right'):
                body = model.model.body(name).id
                jacobian = np.zeros((3, model.model.nv))
                mujoco.mj_jacBodyCom(model.model, model.data, jacobian, None, body)
                jaws -= jacobian[:, :6].T @ (model.model.body_mass[body] * model.model.opt.gravity)
            np.testing.assert_allclose(model_gravity - jaws,
                                       solver.inverse_dynamics(q, np.zeros(6), np.zeros(6)), atol=1e-9)

    def test_grasp_effort_uses_sdk_units_feedback_and_clears_position_gains(self):
        backend = self.backend()
        hardware = backend._hardware
        hardware.state.gripper_vel = -.002
        hardware.state.gripper_torque = .2
        state = backend.read_state(10.)
        self.assertEqual(state.gripper_velocity, -.002)
        self.assertEqual(state.gripper_effort, .2)
        self.assertIsNone(state.gripper_contact)
        backend.step(ControlCommand(Mode.COMMAND_FOLLOWING, gripper_effort=.2), 10.)
        packet, gain = hardware.sent[-1]
        self.assertEqual(packet.gripper_torque, .2)  # Negative open readout -> positive closes.
        self.assertEqual((gain.gripper_kp, gain.gripper_kd), (0., 0.))
        with self.assertRaises(ValueError):
            backend.step(ControlCommand(Mode.COMMAND_FOLLOWING, gripper_effort=backend._gripper_effort_max + 1), 10.004)
        backend.step(ControlCommand(Mode.COMMAND_FOLLOWING, gripper_target=.04), 10.008)
        self.assertEqual(hardware.sent[-1][0].gripper_torque, 0.)
        self.assertGreater(hardware.sent[-1][1].gripper_kp, 0.)

    def test_native_gravity_and_unscaled_gains(self):
        backend = self.backend()
        hardware = backend._hardware
        self.assertTrue(hardware.control.gravity_compensation)
        self.assertFalse(hardware.control.background_send_recv)
        self.assertEqual(hardware.robot.gripper_open_readout, -3.4)
        self.assertEqual(hardware.robot.gripper_width, .082)
        backend.step(ControlCommand(Mode.COMMAND_FOLLOWING), 10.)
        packet, gain = hardware.sent[-1]
        np.testing.assert_array_equal(packet.pos(), hardware.state.pos())
        np.testing.assert_array_equal(packet.vel(), np.zeros(6))
        np.testing.assert_array_equal(packet.torque(), np.zeros(6))
        np.testing.assert_array_equal(gain.kp(), backend.config.kp)
        np.testing.assert_array_equal(gain.kd(), backend.config.kd)
        self.assertEqual(len(hardware.sent), 1)
        backend.step(ControlCommand(Mode.DISABLED), 10.004)
        np.testing.assert_allclose(hardware.command.vel(), -hardware.state.pos() / 1.8)
        self.assertEqual(hardware.command.timestamp, 0.)

    def test_extra_feedforward_is_not_gravity(self):
        backend = self.backend()
        torque = np.array([.1, -.2, .3, -.1, .2, -.05])
        backend.step(ControlCommand(Mode.COMMAND_FOLLOWING), 10., feedforward_torque=torque)
        np.testing.assert_array_equal(backend._hardware.command.torque(), torque)
        for invalid in ([0] * 5, [float('nan')] * 6, [1000] * 6):
            with self.assertRaises(ValueError):
                backend.step(ControlCommand(Mode.COMMAND_FOLLOWING), 10.004, feedforward_torque=invalid)
        self.assertEqual(len(backend._hardware.sent), 1)

    def test_zero_home_ramp_and_no_velocity_after_home(self):
        config = load_settings().robot
        np.testing.assert_array_equal(config.park_q, np.zeros(6))
        controller = JointTargetController(config, X5Model(config))
        measured = np.array([.2, 1.1, 1.3, 0., .2, 0.])
        np.testing.assert_array_equal(controller.update(ControlCommand(Mode.DISABLED), measured, 10.), measured)
        np.testing.assert_allclose(controller.dq_target, -measured / 1.8)
        np.testing.assert_allclose(controller.update(ControlCommand(Mode.DISABLED), measured, 12.), np.zeros(6))
        np.testing.assert_array_equal(controller.dq_target, np.zeros(6))

    def test_planned_velocity_reaches_sdk_and_resets_without_changing_gains(self):
        backend = self.backend()
        hardware = backend._hardware
        measured = np.array(hardware.state.pos()).copy()
        velocity = np.array([.03, -.02, .01, -.01, .02, -.03])
        follow = ControlCommand(Mode.COMMAND_FOLLOWING, joint_target=measured + velocity*.004,
                                joint_velocity_target=velocity, joint_step_limit=.002)
        for replacement in (ControlCommand(Mode.COMMAND_FOLLOWING, hold=True),
                            replace(follow, reset_reference=True),
                            ControlCommand(Mode.FREE_DRIVE),
                            ControlCommand(Mode.COMMAND_FOLLOWING, joint_target=measured),
                            ControlCommand(Mode.COMMAND_FOLLOWING, pose_target=backend.kinematics.fk(measured))):
            with self.subTest(replacement=replacement):
                backend.step(ControlCommand(Mode.COMMAND_FOLLOWING, hold=True), 10.)
                backend.step(follow, 10.004)
                np.testing.assert_allclose(hardware.command.vel(), velocity)
                np.testing.assert_array_equal(hardware.gain.kp(), backend.config.kp)
                np.testing.assert_array_equal(hardware.gain.kd(), backend.config.kd)
                backend.step(replacement, 10.008)
                np.testing.assert_array_equal(hardware.command.vel(), np.zeros(6))

    def test_backend_sdk_capability_drives_planned_packet_without_second_slew(self):
        from embodied.control.motion_limits import MotionLimits
        config = replace(load_settings().robot, joint_velocity_max=(99.,) * 6)
        backend = RealBackend(config)
        self.addCleanup(backend.close)
        hardware = backend._hardware
        from embodied.devices.robots.arx5.plugin import Hardware
        limits = MotionLimits.load(config, backend='real',
                                   capabilities=Hardware().capabilities(config, backend.runtime_parameters()))
        np.testing.assert_array_equal(limits.joint_velocity_limits, hardware.robot.joint_vel_max)
        self.assertFalse(np.array_equal(limits.joint_velocity_limits, config.joint_velocity_max))
        measured = np.array(hardware.state.pos()).copy()
        goal = measured + [.04, -.04, .04, -.04, .04, -.04]
        velocity = np.array([.6, -.7, .8, -.9, 1., -1.1])
        backend.step(ControlCommand(Mode.COMMAND_FOLLOWING, hold=True), 10.)
        follow = ControlCommand(Mode.COMMAND_FOLLOWING, joint_target=goal,
            joint_velocity_target=velocity, joint_step_limit=limits.step_limits({'speed': 0}), planned=True)
        backend.step(follow, 10.004)
        np.testing.assert_array_equal(hardware.command.pos(), goal)
        np.testing.assert_array_equal(hardware.command.vel(), velocity)
        np.testing.assert_array_equal(hardware.gain.kp(), config.kp)
        np.testing.assert_array_equal(hardware.gain.kd(), config.kd)
        count = len(hardware.sent)
        for invalid in (replace(follow, joint_target=backend.kinematics.upper + .001),
                        replace(follow, joint_velocity_target=limits.joint_velocity_limits + .001),
                        replace(follow, joint_velocity_target=None),
                        replace(follow, joint_step_limit=None)):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                backend.step(invalid, 10.008)
        self.assertEqual(len(hardware.sent), count)

    def test_per_axis_slew_uses_configured_period_but_planned_q_dq_stay_exact(self):
        from embodied.control.motion_limits import MotionLimits
        caps = np.array([.6, .8, 1., 1.2, 1.4, 1.6])
        for hz in (100, 500):
            config = replace(load_settings().robot, control_hz=hz, joint_velocity_max=tuple(caps))
            limits = MotionLimits.load(config)
            measured = np.array([0., .6, 1., 0., .2, 0.])
            goal, velocity = measured + .05, caps * .8
            for planned in (False, True):
                with self.subTest(hz=hz, planned=planned):
                    controller = JointTargetController(config, X5Model(config))
                    controller.update(ControlCommand(Mode.COMMAND_FOLLOWING, hold=True), measured, 10.)
                    command = ControlCommand(Mode.COMMAND_FOLLOWING, joint_target=goal,
                        joint_velocity_target=velocity, joint_step_limit=limits.step_limits({'speed': 0}),
                        planned=planned)
                    actual = controller.update(command, measured, 10. + 1 / hz)
                    np.testing.assert_allclose(actual, goal if planned else measured + caps / hz)
                    np.testing.assert_allclose(controller.dq_target, velocity if planned else np.zeros(6))

    def test_managed_worker_passes_active_backend_capabilities_before_any_command(self):
        from embodied.control import worker
        from embodied.devices.robots.arx5 import real
        made, captured = [], []
        def construct(config, **kwargs):
            backend = RealBackend(config, joint_velocity_limit_rad_s=.7, **kwargs)
            made.append((backend, backend._hardware))
            return backend
        def capture_gateway(*args, **kwargs):
            captured.append(kwargs['motion_limits'])
            raise RuntimeError('offline capability captured')
        with tempfile.TemporaryDirectory(prefix='arx5-offline-capability-') as folder:
            directory = Path(folder) / 'run'
            directory.mkdir()
            (directory / 'runtime.json').write_text(json.dumps({'native': None}))
            with patch.object(real, 'RealBackend', side_effect=construct), \
                 patch.object(real, 'preflight', return_value={'ready': True}), \
                 patch('embodied.contracts.device_lease.DeviceLease'), \
                 patch('embodied.control.gateway.Gateway', side_effect=capture_gateway), \
                 patch.object(worker, 'RobotBridge') as bridge, \
                 self.assertRaisesRegex(RuntimeError, 'offline capability captured'):
                worker.run('real', 'arx5-offline', None, directory, managed=True)
            bridge.assert_not_called()
        self.assertEqual(len(made), 1)
        self.assertEqual(len(captured), 1)
        np.testing.assert_allclose(captured[0].resolve({'speed': 0})['joint_velocity_rad_s'], [.7] * 6)
        self.assertEqual(captured[0].describe()['source']['kind'], 'sdk_runtime_configuration')
        self.assertEqual(made[0][1].sent, [])
        self.assertTrue(made[0][0]._closed)

    def test_planned_velocity_is_zero_on_clipped_axes_and_invalid_values_never_send(self):
        backend = self.backend()
        hardware = backend._hardware
        measured = np.array(hardware.state.pos()).copy()
        backend.step(ControlCommand(Mode.COMMAND_FOLLOWING, hold=True), 10.)
        goal = measured + [.1, -.1, .0001, 0, 0, 0]
        velocity = np.array([.03, -.03, .025, 0, 0, 0])
        backend.step(ControlCommand(Mode.COMMAND_FOLLOWING, joint_target=goal,
                     joint_velocity_target=velocity, joint_step_limit=.002), 10.004)
        np.testing.assert_allclose(hardware.command.pos(), measured + [.002, -.002, .0001, 0, 0, 0])
        np.testing.assert_allclose(hardware.command.vel(), [0, 0, .025, 0, 0, 0])
        controller = backend.controller
        for boundary, direction in ((backend.kinematics.upper, 1.), (backend.kinematics.lower, -1.)):
            controller.update(ControlCommand(Mode.COMMAND_FOLLOWING, hold=True, reset_reference=True), boundary, 11.)
            for overshoot in (0., .1):
                controller.update(ControlCommand(Mode.COMMAND_FOLLOWING, joint_target=boundary+direction*overshoot,
                                  joint_velocity_target=np.full(6, direction*.03)), boundary, 11.004)
                np.testing.assert_array_equal(controller.dq_target, np.zeros(6))
        count = len(hardware.sent)
        for velocity in (np.full(6, float('nan')), np.zeros(5), np.full(6, .501)):
            with self.assertRaises(ValueError):
                backend.step(ControlCommand(Mode.COMMAND_FOLLOWING, joint_target=measured,
                             joint_velocity_target=velocity, joint_step_limit=.002), 12.)
        self.assertEqual(len(hardware.sent), count)

    def test_manual_gravity_retains_zero_damping_phase(self):
        backend = self.backend(automatic_gravity=False)
        self.assertFalse(backend._hardware.control.gravity_compensation)
        command = ControlCommand(Mode.FREE_DRIVE, gripper_mode=Mode.FREE_DRIVE)
        backend.step(command, 10., damping_only=True)
        np.testing.assert_array_equal(backend._hardware.command.torque(), np.zeros(6))
        backend.step(command, 10.004)
        q = backend._hardware.state.pos()
        gravity = backend._gravity_solver.inverse_dynamics(q, np.zeros(6), np.zeros(6))
        np.testing.assert_allclose(backend._hardware.command.torque(), gravity)

    def test_cancelled_step_never_sends_and_stop_rejects_targets(self):
        backend = self.backend()
        with self.assertRaises(InterruptedError):
            backend.step(ControlCommand(Mode.DISABLED), 10., cancel_requested=lambda: True)
        self.assertEqual(backend._hardware.sent, [])
        hardware = backend._hardware
        backend.request_stop()
        with self.assertRaises(InterruptedError):
            backend.step(ControlCommand(Mode.DISABLED), 10.004)
        self.assertEqual(hardware.sent, [])
        self.assertEqual(hardware.damping_calls, 1)

    def test_real_launch_accepts_vr_and_rejects_demo(self):
        from embodied.control.app import validate_run
        validate_run('real', 'vr', True)
        with self.assertRaises(ValueError):
            validate_run('real', 'demo', True)

    def test_configurable_home_is_preserved(self):
        config = replace(RobotConfig(), park_q=(0, 1.2, 1.4, 0, 0, 0))
        controller = JointTargetController(config, X5Model(config))
        controller.update(ControlCommand(Mode.DISABLED), np.zeros(6), 10.)
        np.testing.assert_allclose(controller.update(ControlCommand(Mode.DISABLED), np.zeros(6), 12.), config.park_q)

    def test_real_worker_records_and_damps_on_exit_or_fault(self):
        from embodied.control import worker
        from embodied.devices.robots.arx5 import real
        for failure in (False, True):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory(prefix='arx5-offline-') as folder:
                made = []
                def construct(config, **kwargs):
                    backend = RealBackend(config, **kwargs)
                    made.append((backend, backend._hardware))
                    return backend
                polls = []
                def poll(now):
                    polls.append(now)
                    if len(polls) == 2:
                        if failure:
                            made[0][1].state.timestamp = float('nan')
                        else:
                            signal.raise_signal(signal.SIGTERM)
                    return ControlCommand(Mode.COMMAND_FOLLOWING)
                bridge = SimpleNamespace(poll_commands=poll, publish_state=lambda *args: None, close=lambda: None)
                directory = Path(folder) / 'run'
                with patch.object(real, 'RealBackend', side_effect=construct), \
                     patch.object(real, 'preflight', return_value={'ready': True}), \
                     patch.object(worker, 'RobotBridge', return_value=bridge):
                    if failure:
                        with self.assertRaises(RuntimeError):
                            worker.run('real', 'arx5-offline', None, directory)
                    else:
                        worker.run('real', 'arx5-offline', None, directory)
                backend, hardware = made[0]
                self.assertTrue(backend._closed)
                self.assertEqual(hardware.damping_calls, 1)
                self.assertEqual(len(hardware.sent), 1)
                report = json.loads((directory / 'report.json').read_text())
                self.assertTrue(report['damping_requested'])
                self.assertFalse(report['park_completed'])
                self.assertTrue(report['tuning_recording']['complete'])
                self.assertEqual(bool(report['error']), failure)


if __name__ == '__main__':
    with patch.object(sdk, 'Arx5CartesianController', side_effect=AssertionError('CAN construction forbidden')):
        unittest.main(verbosity=2)
