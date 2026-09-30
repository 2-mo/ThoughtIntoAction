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

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import arx5_interface as sdk
from arx5_vr.model import JointTargetController, X5Model
from arx5_vr.real import RealBackend
from arx5_vr.settings import load_settings
from arx5_vr.types import ControlCommand, Mode, RobotConfig


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
        source = ET.parse(ROOT / 'models/x5/x5_2025.source.urdf').getroot()
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
        reference = sdk.Arx5Solver(str(ROOT / 'models/x5/x5_2025.source.urdf'), 6,
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
        from arx5_vr.app import validate_run
        validate_run('real', 'vr', True)
        with self.assertRaises(ValueError):
            validate_run('real', 'demo', True)

    def test_configurable_home_is_preserved(self):
        config = replace(RobotConfig(), park_q=(0, 1.2, 1.4, 0, 0, 0))
        controller = JointTargetController(config, X5Model(config))
        controller.update(ControlCommand(Mode.DISABLED), np.zeros(6), 10.)
        np.testing.assert_allclose(controller.update(ControlCommand(Mode.DISABLED), np.zeros(6), 12.), config.park_q)

    def test_real_worker_records_and_damps_on_exit_or_fault(self):
        from arx5_vr import real, worker
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
