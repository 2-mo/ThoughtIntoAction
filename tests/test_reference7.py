"""Independent MuJoCo model/driver, common planner, state restore and wire shape."""
import json
import unittest

import numpy as np

from embodied.contracts.registry import Registry
from embodied.contracts.paths import PROJECT_ROOT
from embodied.contracts.control import ControlCommand, Mode
from embodied.control.commands import MotionPlanner
from embodied.control.settings import load_settings
from embodied.data.episode.contract import runtime_contract, vector


class Reference7Tests(unittest.TestCase):
    def setUp(self):
        self.config = load_settings(PROJECT_ROOT / 'configs/robots/reference7/robot.json').robot
        self.driver = Registry().create('robot', self.config.simulation_plugin, runtime='control')
        self.backend = self.driver.open(self.config)
        self.addCleanup(self.backend.close)

    def test_registered_model_has_seven_axes_and_no_fake_gripper(self):
        self.assertTrue(self.driver.preflight(self.config)['ready'])
        parameters = self.driver.snapshot(self.config)
        contract = runtime_contract({'joint_count': 7, 'data_contract': parameters['data_contract']})
        self.assertIsNone(contract['gripper'])
        self.assertEqual(vector({'q': [0.] * 7, 'gripper_width': 0.}, contract), [0.] * 7)
        self.assertEqual(self.config.control_hz, 100)
        self.assertEqual(self.driver.capabilities(self.config, None)['gripper'], {'present': False})
        self.assertFalse(self.config.hardware_plugin)
        json.dumps(parameters, allow_nan=False)

    def test_shared_movej_movel_and_continuous_path_execute_in_physics_then_restore(self):
        initial = self.backend.simulation_snapshot()
        planner = MotionPlanner(self.config, self.config.model_id)
        for kind in ('movej', 'movel', 'move_path'):
            state = self.backend.read_state(0.)
            if kind == 'movej':
                goal = state.q.copy(); goal[-1] += .15
                request = {'kind': kind, 'q': goal, 'speed': .5}
            elif kind == 'movel':
                pose = state.pose.copy(); pose[0] += .005
                request = {'kind': kind, 'pose': pose, 'speed': .5, 'tcp_speed': .03}
            else:
                first = state.pose.copy(); first[0] += .005
                second = first.copy(); second[1] += .005
                request = {'kind': kind, 'waypoints': [first.tolist(), second.tolist()],
                           'blend_radius': .001, 'speed': .5, 'tcp_speed': .03}
            plan = planner.plan(request, {'q': state.q, 'gripper_width': 0., 'epoch': 0, 'owner': None}, 0.)
            for i in range(int((plan['times'][-1] + 1.) * self.config.control_hz)):
                t = i / self.config.control_hz
                q, dq = plan['_trajectory'].sample(t)
                self.backend.step(ControlCommand(Mode.COMMAND_FOLLOWING, joint_target=q,
                    joint_velocity_target=dq, planned=True), t)
            measured = self.backend.read_state(t)
            self.assertLess(float(np.max(abs(measured.q - plan['q'][-1]))), .005)
            self.assertEqual(len(self.backend.telemetry()['sent_q']), 7)
            self.assertGreater(float(self.backend.data.time), 0)
        self.backend.simulation_restore(initial)
        np.testing.assert_allclose(self.backend.read_state(0.).q, self.config.park_q)
        self.assertEqual(self.backend.telemetry(), {})
        altered = {**initial, 'model_id': 'different'}
        with self.assertRaisesRegex(ValueError, 'identity'):
            self.backend.simulation_restore(altered)
        json.dumps(self.backend.simulation_snapshot(), allow_nan=False)

    def test_unplanned_stream_respects_profile_velocity_and_rejects_gripper(self):
        q = self.backend.read_state(0.).q
        self.backend.step(ControlCommand(Mode.COMMAND_FOLLOWING, joint_target=q + .1), 0.)
        np.testing.assert_allclose(self.backend.telemetry()['sent_q'], q + .015)
        with self.assertRaisesRegex(ValueError, 'no gripper'):
            self.backend.step(ControlCommand(Mode.COMMAND_FOLLOWING, gripper_target=.02), .01)


if __name__ == '__main__':
    unittest.main()
