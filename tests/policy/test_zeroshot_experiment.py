import json
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
from arm_service_fixture import ServiceFixture
from legacy.robot_control.client import ArmClient
from embodied.devices.robots.arx5.pinocchio_model import X5PinocchioModel
from embodied.core.presets import DEFAULT_GRASP_READY_JOINTS
from embodied.policy.pi05_zeroshot.real import MOTION, move_pregrasp
from embodied.policy.pi05_zeroshot.experiment import require_fresh, step_target, execute_step


class ExperimentTests(unittest.TestCase):
    def prediction(self, q, delta=.1):
        return {'state_physical': [*q, .078], 'first_action_x5_experimental': [*(np.array(q)+delta), .01],
                'rgb_monotonic_ns': time.monotonic_ns(), 'arm_service_id': 'test',
                'state_clipped_dimensions': [], 'state_quantile_excess': 0.}

    def test_joint_and_cartesian_bounds(self):
        robot = X5PinocchioModel()
        tcp = X5PinocchioModel(eef='eef_link')
        q = list(DEFAULT_GRASP_READY_JOINTS)
        target, plan = step_target(robot, tcp, {'actual_pos': q}, self.prediction(q))
        self.assertLessEqual(np.max(np.abs(target-q)), .020000001)
        self.assertLessEqual(np.linalg.norm(tcp.fk(target)[:3,3]-tcp.fk(q)[:3,3]), .01)
        self.assertLess(plan['scale'], 1)
        with self.assertRaises(ValueError):
            step_target(robot, tcp, {'actual_pos': q}, self.prediction(q, 2))

    def test_stale_or_changed_observation_rejected(self):
        q = list(DEFAULT_GRASP_READY_JOINTS)
        state = {'actual_pos': q, 'actual_vel': [0]*6}
        arm = SimpleNamespace(service_id='test')
        pred = self.prediction(q)
        require_fresh(arm, pred, state)
        pred['rgb_monotonic_ns'] -= 3_000_000_000
        with self.assertRaises(ValueError):
            require_fresh(arm, pred, state)
        pred = self.prediction(q)
        with self.assertRaises(ValueError):
            require_fresh(arm, pred, {**state, 'actual_pos': (np.array(q)+.02).tolist()})

    def test_single_step_real_rpc_preserves_gripper_and_hold(self):
        fixture = ServiceFixture(table_safety=True)
        self.addCleanup(fixture.close)
        arm = ArmClient(fixture.robot, path=fixture.path, hardware=False, actuate=True,
                        speed=.15, motion=MOTION, input_name='test')
        self.addCleanup(arm.close)
        state = move_pregrasp(arm)
        pred = self.prediction(state['actual_pos'], .02)
        pred['arm_service_id'] = arm.service_id
        with tempfile.TemporaryDirectory() as tmp:
            execute_step(arm, pred, Path(tmp))
            result = json.loads((Path(tmp)/'execution.json').read_text())
        self.assertEqual(result['status'], 'completed')
        self.assertEqual(result['final_mode'], 'hold')
        self.assertIsNone(fixture.service.owner)
        self.assertLessEqual(np.max(np.abs(result['actual_delta_rad'])), .021)
        self.assertGreater(np.max(np.abs(result['actual_delta_rad'])), .005)
        self.assertAlmostEqual(arm.snapshot()['gripper_actual_pos'], state['gripper_actual_pos'], places=4)


if __name__ == '__main__':
    unittest.main()
