"""RM65 nominal kinematics, shared planner/physics and atomic scene restore."""
from copy import deepcopy
import json
import unittest
from dataclasses import replace

import numpy as np

from embodied.contracts.paths import PROJECT_ROOT
from embodied.contracts.registry import Registry
from embodied.contracts.control import ControlCommand, Mode
from embodied.control.settings import load_settings
from embodied.control.commands import MotionPlanner


class SimulationTests(unittest.TestCase):
    def setUp(self):
        self.config = load_settings(PROJECT_ROOT / 'configs/robots/realman/sim.json').robot
        self.driver = Registry().create('robot', 'realman.sim', runtime='control')
        self.backend = self.driver.open(self.config)
        self.addCleanup(self.backend.close)

    def test_model_matches_pinned_vendor_nominal_fk_and_metadata(self):
        # RM_API2 a810b9fdea29, Algo(RM65,B), Lq=(0,0,.14,0,0,0), Base=identity.
        fixtures = [([0]*6, [-5.4376965e-8,0,.9905000329, -4.3711388e-8,0,1,0]),
            ([14.162,14.94,21.428,-15.776,88.912,36.789],
             [-.4200631678,-.0263769459,.4992551804,.8507217169,-.2321489006,-.4639005661,.0847108588]),
            ([10,20,-30,40,50,-60], [-.1564460248,-.1695856452,.89658916,-.165183112,-.3244271874,.9254686236,.1047346517])]
        from scipy.spatial.transform import Rotation
        for degrees, expected in fixtures:
            actual = self.backend.kinematics.fk(np.deg2rad(degrees))
            self.assertLess(np.linalg.norm(actual[:3] - expected[:3]), 1e-5)
            error = Rotation.from_quat(actual[3:]) * Rotation.from_quat(expected[3:]).inv()
            self.assertLess(error.magnitude(), 2e-5)
        parameters = self.driver.snapshot(self.config)
        self.assertEqual(parameters['model']['manifest']['tcp_link'], 'tcp_link')
        self.assertEqual(len(parameters['data_contract']['joint_names']), 6)
        self.assertFalse(self.driver.capabilities(self.config, None)['gripper']['grasp'])
        self.assertFalse(self.config.hardware_plugin)
        json.dumps(parameters, allow_nan=False)
        with self.assertRaisesRegex(ValueError, 'configuration changed'):
            replace(self.config, tool_offset_m=.15).model_id

    def test_shared_movej_movel_follow_in_physics(self):
        planner = MotionPlanner(self.config, self.config.model_id)
        for kind in ('movej', 'movel'):
            state = self.backend.read_state(0.)
            if kind == 'movej':
                goal = state.q.copy(); goal[0] += .1
                request = {'kind': kind, 'q': goal, 'speed': .2}
            else:
                goal = state.pose.copy(); goal[0] += .01
                request = {'kind': kind, 'pose': goal, 'speed': .2, 'tcp_speed': .02}
            plan = planner.plan(request, {'q': state.q, 'gripper_width': .04, 'epoch': 0, 'owner': None}, 0.)
            for i in range(int((plan['times'][-1] + 1) * self.config.control_hz)):
                t = i / self.config.control_hz
                q, dq = plan['_trajectory'].sample(t)
                self.backend.step(ControlCommand(Mode.COMMAND_FOLLOWING, joint_target=q,
                    joint_velocity_target=dq, planned=True), t)
            measured = self.backend.read_state(t)
            self.assertLess(float(np.max(abs(measured.q - plan['q'][-1]))), .003)
            self.assertGreater(self.backend.data.time, 0)
        self.assertLess(np.linalg.norm(measured.pose[:3] - goal[:3]), .001)

    def test_gripper_stream_rate_and_atomic_snapshot_restore(self):
        initial = self.backend.simulation_snapshot()
        q = self.backend.read_state(0.).q
        self.backend.step(ControlCommand(Mode.COMMAND_FOLLOWING, joint_target=q+.1, gripper_target=.02), 0.)
        np.testing.assert_allclose(self.backend.telemetry()['sent_q'], q + .005)
        self.assertAlmostEqual(self.backend.telemetry()['gripper_target'], .0398)
        for i in range(250):
            self.backend.step(ControlCommand(Mode.COMMAND_FOLLOWING, hold=True, gripper_target=.02), i*.01)
        self.assertAlmostEqual(self.backend.read_state(0.).gripper_width, .02, places=4)
        saved = self.backend.simulation_snapshot()
        for mutate in ('model', 'nan', 'position'):
            bad = deepcopy(saved)
            if mutate == 'model': bad['model_id'] = 'wrong'
            if mutate == 'nan': bad['state'][0] = float('nan')
            if mutate == 'position': bad['state'][1] = 100  # integration state starts with time then qpos
            with self.assertRaises(ValueError):
                self.backend.simulation_restore(bad)
            np.testing.assert_array_equal(self.backend.simulation_snapshot()['state'], saved['state'])
        self.backend.simulation_restore(initial)
        np.testing.assert_array_equal(self.backend.simulation_snapshot()['state'], initial['state'])
        self.backend.simulation_restore(saved)
        self.backend.simulation_reset()
        np.testing.assert_array_equal(self.backend.simulation_snapshot()['state'], initial['state'])
        with self.assertRaisesRegex(ValueError, 'position only'):
            self.backend.step(ControlCommand(Mode.COMMAND_FOLLOWING, gripper_effort=1), 0)

    def test_unreachable_stream_target_holds_and_later_valid_target_recovers(self):
        state = self.backend.read_state(0.)
        previous = self.backend.target.copy()
        unreachable = state.pose.copy()
        unreachable[0] = 10.
        self.backend.step(ControlCommand(Mode.COMMAND_FOLLOWING, pose_target=unreachable,
                                         gripper_target=.02), 0.)
        np.testing.assert_array_equal(self.backend.telemetry()['sent_q'], previous)
        self.assertLess(self.backend.telemetry()['gripper_target'], .04)
        self.assertFalse(self.backend.closed)
        goal = previous.copy()
        goal[0] += .02
        reachable = self.backend.kinematics.fk(goal)
        for i in range(30):
            self.backend.step(ControlCommand(Mode.COMMAND_FOLLOWING, pose_target=reachable), .01*(i+1))
        self.assertGreater(self.backend.read_state(.3).q[0], previous[0]+.005)

    def test_upward_pose_stream_preserves_joint_direction_under_rate_limit(self):
        import mujoco
        # Captured Quest posture: independent joint clipping moves the TCP down
        # even for an upward target with unchanged orientation.
        q = np.array([.2846434944, .0987155586, 1.1136884228,
                      -.9803576667, 1.0650679170, 1.3575574727])
        self.backend.data.qpos[:6] = q
        self.backend.target = q.copy()
        mujoco.mj_forward(self.backend.model, self.backend.data)
        initial = self.backend.read_state(0.).pose.copy()
        goal = initial.copy()
        goal[2] += .05
        heights = []
        bound = np.minimum(self.config.joint_step_limit,
                           np.asarray(self.config.joint_velocity_max) / self.config.control_hz)
        for i in range(200):
            previous = self.backend.target.copy()
            self.backend.step(ControlCommand(Mode.COMMAND_FOLLOWING, pose_target=goal), i*.01)
            self.assertTrue(np.all(abs(self.backend.target-previous) <= bound+1e-12))
            heights.append(self.backend.read_state(i*.01).pose[2])
        self.assertGreaterEqual(min(heights), initial[2]-1e-5)
        self.assertAlmostEqual(heights[-1], goal[2], delta=.001)


if __name__ == '__main__':
    unittest.main()
