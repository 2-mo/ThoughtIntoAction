"""Explicit TCP bounds, a single continuous path, and feedback-based grasping."""
import json
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation

from embodied.contracts.control import RobotState
from embodied.control.commands import CommandEngine, MotionPlanner
from embodied.control.grasp import Grasp


class CartesianModel:
    lower, upper = np.full(6, -3.), np.full(6, 3.)

    def fk(self, q):
        return np.r_[np.asarray(q)[:3] + [0., 0., .3], Rotation.from_rotvec(q[3:]).as_quat()]

    def solve_ik(self, pose, seed):
        return np.r_[np.asarray(pose)[:3] - [0., 0., .3], Rotation.from_quat(pose[3:]).as_rotvec()], True, 0.


class ControlPrimitiveTests(unittest.TestCase):
    def setUp(self):
        self.config = SimpleNamespace(park_q=(0.,)*6, model_plugin='test.cartesian', control_hz=250,
            gripper_max_width=.08, joint_velocity_max=(2.,)*6)
        with patch('embodied.contracts.registry.Registry.create', return_value=CartesianModel()):
            self.planner = MotionPlanner(self.config, 'fixture')
        self.snapshot = {'q': [0.]*6, 'gripper_width': .08, 'epoch': 0, 'owner': None}
        self.cap = {'present': True, 'grasp': True, 'effort_max': 1.,
                    'effort_unit': 'motor_Nm', 'contact_detection': 'effort_stall'}

    def state(self, now, *, width=.04, velocity=0., effort=.3, contact=None):
        return RobotState(np.zeros(6), np.zeros(6), self.planner.model.fk(np.zeros(6)), width, now,
                          gripper_velocity=velocity, gripper_effort=effort, gripper_contact=contact)

    def plan(self, **request):
        return self.planner.plan({'speed': 0, **request}, self.snapshot, 1.)

    def test_tcp_units_bound_actual_trajectory_and_change_identity(self):
        base = dict(kind='movel', pose=[.1, 0., .3, 0., 0., 0., 1.])
        original = self.plan(**base)
        limited = self.plan(**base, tcp_speed=.02, tcp_acceleration=.1, tcp_jerk=.5)
        self.assertGreater(limited['times'][-1], original['times'][-1])
        for field, maximum in [('peak_tcp_speed_m_s', .02), ('peak_tcp_acceleration_m_s2', .1), ('peak_tcp_jerk_m_s3', .5)]:
            self.assertLessEqual(limited['metrics'][field], maximum)
        self.assertTrue(any(key.startswith('tcp_') for key in limited['metrics']['limiting_factors']))
        self.assertEqual(limited['limits']['joint_velocity_rad_s'], [2.]*6)
        a, b = self.plan(**base, tcp_speed=5.), self.plan(**base, tcp_speed=6.)
        np.testing.assert_array_equal(a['times'], b['times'])
        self.assertNotEqual(a['plan_id'], b['plan_id'])
        positions = np.asarray([limited['_trajectory'].sample(t)[0] for t in limited['times']])
        peak = np.max(np.linalg.norm(np.gradient(positions[:, :3], limited['times'], axis=0), axis=1))
        self.assertLessEqual(peak, .02)

    def test_pure_rotation_uses_angular_not_linear_speed(self):
        pose = [0., 0., .3, *Rotation.from_rotvec([0., 0., .2]).as_quat()]
        plan = self.plan(kind='movel', pose=pose, angular_speed=.05)
        self.assertLessEqual(plan['metrics']['peak_angular_speed_rad_s'], .05)
        self.assertAlmostEqual(plan['metrics']['path_length_m'], 0.)
        self.assertIn('angular_speed', plan['metrics']['limiting_factors'])

    def test_path_is_one_nonstopping_trajectory_and_fits_rpc(self):
        plan = self.plan(kind='move_path', waypoints=[[.03, 0., .3, 0., 0., 0., 1.],
            [.03, .03, .3, 0., 0., 0., 1.]], blend_radius=.005, tcp_speed=.04)
        trajectory, duration = plan['_trajectory'], plan['times'][-1]
        samples = [trajectory.sample(t) for t in np.linspace(duration*.05, duration*.95, 300)]
        self.assertGreater(min(np.linalg.norm(dq) for q, dq in samples), 1e-6)
        np.testing.assert_allclose(trajectory.sample(duration)[1], np.zeros(6), atol=1e-10)
        public = {k: v.tolist() if isinstance(v, np.ndarray) else v for k, v in plan.items() if not k.startswith('_')}
        self.assertLess(len(json.dumps(public)), 1_000_000)
        engine = CommandEngine('r', self.config, 'fixture')
        state = self.state(1., width=.08)
        lease = engine.request({'version': 1, 'runtime_id': 'r', 'action': 'acquire', 'source': 'test'}, state, 1.)
        engine.next(state, 1.)
        operation = engine.request({'version': 1, 'runtime_id': 'r', 'action': 'execute',
            **{k: lease[k] for k in ('token', 'epoch')}, 'sequence': 1, '_plan': plan}, state, 1.)
        engine.expires = 100.
        command = engine.next(state, 1. + duration/2)
        self.assertGreater(np.linalg.norm(command.joint_velocity_target), 1e-6)
        self.assertEqual(engine.operation['operation_id'], operation['operation_id'])
        self.assertEqual(len(engine.operations), 1)
        self.assertFalse(engine.settling.samples)

    def test_contact_requires_fresh_sustained_feedback_and_real_physics_evidence(self):
        for physical in (False, True):
            cap = {**self.cap, 'contact_detection': 'physics_contact'}
            grasp = Grasp({'effort': .3, 'timeout': 1.}, self.state(0., width=.08), cap, 0.)
            for t in np.arange(.1, 1.01, .05):
                grasp.update(self.state(float(t), contact=physical), float(t))
                if grasp.result:
                    break
            self.assertEqual(grasp.result['confirmed'], physical)
            self.assertEqual(grasp.result['stop_reason'], 'contact' if physical else 'timeout')
        grasp = Grasp({'effort': .3}, self.state(0., width=.08), self.cap, 0.)
        grasp.update(self.state(.3), .3)
        grasp.update(self.state(.6), .6)  # missing feedback cannot count as sustained contact
        self.assertIsNone(grasp.result)
        with self.assertRaisesRegex(ValueError, 'stale'):
            grasp.update(self.state(.6), .8)
        with self.assertRaisesRegex(ValueError, 'finite'):
            Grasp({'effort': .3}, self.state(0., effort=None), self.cap, 0.)

    def test_empty_close_is_not_contact_and_no_gripper_is_rejected(self):
        grasp = Grasp({'effort': .3}, self.state(0., width=.08), self.cap, 0.)
        grasp.update(self.state(.4, width=0.), .4)
        self.assertEqual(grasp.result['stop_reason'], 'minimum_width')
        self.assertFalse(grasp.result['confirmed'])
        with self.assertRaisesRegex(ValueError, 'does not support'):
            Grasp({'effort': .3}, self.state(0.), {'present': False}, 0.)

    def test_grasp_contact_effort_persists_until_cancel(self):
        engine = CommandEngine('r', self.config, 'fixture', capabilities={'gripper': self.cap})
        lease = engine.request({'version': 1, 'runtime_id': 'r', 'action': 'acquire', 'source': 'test'}, self.state(0., width=.08), 0.)
        envelope = {'version': 1, 'runtime_id': 'r', **{k: lease[k] for k in ('token', 'epoch')}}
        engine.next(self.state(0.), 0.)
        operation = engine.request({**envelope, 'action': 'grasp', 'sequence': 1, 'effort': .3}, self.state(0., width=.08), 0.)
        engine.expires = 10.
        for t in np.arange(.3, .7, .05):
            command = engine.next(self.state(float(t)), float(t))
        result = engine.operations[operation['operation_id']]['result']
        self.assertTrue(result['confirmed'])
        self.assertEqual(command.gripper_effort, .3)
        engine.request({**envelope, 'action': 'cancel', 'sequence': 2}, self.state(.8), .8)
        self.assertIsNone(engine.next(self.state(.8), .8).gripper_effort)

class PhysicsGraspTests(unittest.TestCase):
    def test_x5_blended_path_timing_is_stable_across_geometry_sample_density(self):
        from embodied.control.settings import load_settings
        from embodied.motion import cartesian_execution
        config = load_settings().robot
        planner = MotionPlanner(config, config.model_id)
        start = np.asarray(config.park_q)
        first = planner.model.fk(start)
        first[2] += .02
        second = first.copy()
        second[1] += .02
        request = dict(kind='move_path', waypoints=[first.tolist(), second.tolist()], blend_radius=.005,
                       speed=.5, acceleration=2., jerk=10., tcp_speed=.1, angular_speed=.5)
        snapshot = dict(q=start.tolist(), gripper_width=.082, epoch=0, owner=None)
        geometry_builder = cartesian_execution.build_cartesian_path
        durations = []
        for step in (.0015, .003, .005):
            with patch.object(cartesian_execution, 'build_cartesian_path',
                              side_effect=lambda points: geometry_builder(points, max_step_m=step)):
                plan = planner.plan(request, snapshot, 0.)
            durations.append(plan['times'][-1])
            self.assertEqual(plan['limits']['joint_velocity_rad_s'], [.5] * 6)
            self.assertEqual(plan['limits']['tcp'], {'tcp_speed': .1, 'angular_speed': .5})
            for name, cap in [('peak_velocity_rad_s', .5), ('peak_acceleration_rad_s2', 2.),
                              ('peak_jerk_rad_s3', 10.), ('peak_tcp_speed_m_s', .1),
                              ('peak_angular_speed_rad_s', .5)]:
                self.assertLessEqual(plan['metrics'][name], cap)
            self.assertIn('jerk', plan['metrics']['limiting_factors'])
        # Line/arc curvature and numerical IK still vary with approximation
        # density. They must not introduce a new acceleration cycle at every sample.
        self.assertLess(max(durations) / min(durations), 1.2)
        self.assertLess(max(durations), 6.)

    def test_actual_mujoco_contacts_confirm_only_an_obstructed_close(self):
        import xml.etree.ElementTree as ET
        from pathlib import Path
        import mujoco
        from embodied.contracts.control import ControlCommand, Mode
        from embodied.control.settings import load_settings
        from embodied.devices.robots.arx5.kinematics import X5Model
        from embodied.devices.robots.arx5.simulation import MujocoBackend
        from embodied.devices.robots.arx5.plugin import Simulation
        config = load_settings().robot
        for obstruction in (False, True):
            with self.subTest(obstruction=obstruction):
                kinematics = X5Model(config)
                if obstruction:
                    path = Path(config.model_path)
                    tree = ET.parse(path)
                    tree.find('compiler').set('meshdir', str(path.parent / 'meshes'))
                    kinematics.data.qpos[:6] = config.park_q
                    mujoco.mj_forward(kinematics.model, kinematics.data)
                    link = kinematics.model.body('link6').id
                    rotation = kinematics.data.xmat[link].reshape(3, 3)
                    xyz = kinematics.data.xpos[link] + rotation @ np.array([.11, 0., 0.])
                    quaternion = Rotation.from_matrix(rotation).as_quat()[[3, 0, 1, 2]]
                    body = ET.SubElement(tree.find('worldbody'), 'body', name='fixture',
                        pos=' '.join(map(str, xyz)), quat=' '.join(map(str, quaternion)))
                    ET.SubElement(body, 'geom', type='box', size='.025 .012 .01', contype='1', conaffinity='1')
                    kinematics.model = mujoco.MjModel.from_xml_string(ET.tostring(tree.getroot(), encoding='unicode'))
                    kinematics.data = mujoco.MjData(kinematics.model)
                with patch('embodied.devices.robots.arx5.simulation.X5Model', return_value=kinematics):
                    backend = MujocoBackend(config)
                try:
                    capability = Simulation().capabilities(config, backend.runtime_parameters())['gripper']
                    grasp = Grasp({'effort': .1, 'timeout': 2.}, backend.read_state(0.), capability, 0.)
                    for tick in range(501):
                        now = tick / config.control_hz
                        state = backend.read_state(now)
                        effort = grasp.update(state, now)
                        if grasp.result:
                            break
                        backend.step(ControlCommand(Mode.COMMAND_FOLLOWING, hold=True,
                            gripper_target=state.gripper_width, gripper_effort=effort), now)
                    self.assertIsNotNone(grasp.result)
                    self.assertEqual(grasp.result['confirmed'], obstruction, grasp.result)
                    self.assertEqual(grasp.result['stop_reason'], 'contact' if obstruction else 'minimum_width')
                    self.assertEqual(backend.telemetry()['gripper_applied_kp'], 0.)
                finally:
                    backend.close()


if __name__ == '__main__':
    unittest.main()
