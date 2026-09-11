import json
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'tests'))
from arm_service_fixture import ServiceFixture
from robot_control.client import ArmClient
from real import MOTION, move_pregrasp
from rollout import bounded_target, control_loop, check_state_quantile


class QuantileTests(unittest.TestCase):
    def test_closed_gripper_and_threshold_boundary(self):
        with self.assertRaises(ValueError):
            check_state_quantile(.6505498836051953, .5)
        for excess in (.6505498836051953, 1.038, 1.1):
            check_state_quantile(excess, 1.1)
        for excess in (1.10001, float('nan'), float('inf'), -.1):
            with self.subTest(excess=excess), self.assertRaises(ValueError):
                check_state_quantile(excess, 1.1)
        for limit in (0, -1, float('nan'), float('inf')):
            with self.subTest(limit=limit), self.assertRaises(ValueError):
                check_state_quantile(0, limit)


class RolloutTests(unittest.TestCase):
    def setUp(self):
        self.fixture = ServiceFixture(table_safety=True)
        self.addCleanup(self.fixture.close)
        self.arm = ArmClient(self.fixture.robot, path=self.fixture.path, hardware=False,
                             actuate=True, speed=.2, motion=MOTION, input_name='rollout-test')
        self.addCleanup(self.arm.close)

    def test_targets_bound_joints_and_gripper_speed(self):
        state = self.arm.snapshot()
        target, grip, clipped = bounded_target(self.arm.robot, [-100]*6+[0], state, .078, .078, .03, 1/30)
        self.assertLessEqual(np.max(np.abs(target-state['actual_pos'])), .080000001)
        self.assertAlmostEqual(grip, .077)
        self.assertTrue(clipped)
        with self.assertRaises(ValueError):
            bounded_target(self.arm.robot, [np.nan]*7, state, .078, .078, .03, 1/30)

    def test_multiple_replans_gripper_and_time_limit_hold(self):
        move_pregrasp(self.arm)
        self.arm.start_control()
        arm = self.arm
        captured = []
        def capture(a, camera, path):
            state = a.snapshot()
            obs = {'state_physical': [*state['actual_pos'],state['gripper_actual_pos']],
                   'rgb_monotonic_ns': time.monotonic_ns(), 'arm_service_id': a.service_id}
            captured.append(obs)
            (path/'observation.json').write_text(json.dumps(obs))
            return obs
        class Worker:
            digest = 'test'
            def predict(self, path, prompt, check):
                check()
                obs = captured[-1]
                action = np.array(obs['state_physical'])
                action[0] += .02
                action[-1] = 0.
                np.savez(path/'prediction.npz', actions_x5_experimental=np.tile(action,(50,1)))
                return {**obs, 'state_quantile_excess': .6505498836051953, 'inference_seconds': .01}
        with tempfile.TemporaryDirectory() as tmp, patch('rollout.capture_owned', capture):
            result = control_loop(arm, Worker(), None, Path(tmp), 'test', seconds=.65, chunk_steps=3, grip_speed=.03,
                                  state_quantile_limit=1.1)
            rows = [json.loads(x) for x in (Path(tmp)/'commands.jsonl').read_text().splitlines()]
        self.assertGreaterEqual(result['cycles'], 2)
        self.assertEqual(result['status'], 'time_limit')
        self.assertEqual(result['state_quantile_limit'], 1.1)
        self.assertGreaterEqual(result['commands'], 6)
        self.assertGreater(len({x['rgb_monotonic_ns'] for x in captured}), 1)
        self.assertLess(rows[-1]['gripper_m'], rows[0]['gripper_m'])
        self.assertIsNone(self.fixture.service.owner)
        self.assertTrue(self.fixture.service.holding)

    def test_policy_failure_stops_and_holds(self):
        move_pregrasp(self.arm)
        self.arm.start_control()
        class Worker:
            digest = 'test'
            def predict(self, *args, **kwargs):
                raise RuntimeError('policy failure')
        with tempfile.TemporaryDirectory() as tmp, patch('rollout.capture_owned', return_value={}):
            with self.assertRaisesRegex(RuntimeError, 'policy failure'):
                control_loop(self.arm, Worker(), None, Path(tmp), 'test', seconds=1, chunk_steps=3, grip_speed=.03)
            self.assertEqual(json.loads((Path(tmp)/'rollout.json').read_text())['status'], 'failed')
        self.assertIsNone(self.fixture.service.owner)
        self.assertTrue(self.fixture.service.holding)


if __name__ == '__main__':
    unittest.main()
