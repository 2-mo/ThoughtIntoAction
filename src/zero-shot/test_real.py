"""Exercise the real RPC control path with a simulated arm, never hardware."""
import sys
from pathlib import Path
import unittest
import time
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tests'))
from real import MOTION, move_pregrasp, stop_and_hold
from robot_control.client import ArmClient
from arm_service_fixture import ServiceFixture


class PregraspTests(unittest.TestCase):
    def setUp(self):
        self.fixture = ServiceFixture(table_safety=True)
        self.addCleanup(self.fixture.close)
        self.arm = ArmClient(self.fixture.robot, path=self.fixture.path, hardware=False,
                             actuate=True, speed=.15, motion=MOTION, input_name='zero-shot-test')
        self.addCleanup(self.arm.close)

    def test_arrival_releases_lease_and_keeps_hold(self):
        state = move_pregrasp(self.arm)
        np.testing.assert_allclose(state['actual_pos'], [0, 1.5, 1.45, -1.52, 0, 0], atol=.03)
        self.assertIsNone(self.fixture.service.owner)
        self.assertTrue(self.fixture.service.holding)
        self.arm.close()
        self.assertTrue(self.fixture.service.holding)

    def test_timeout_stops_and_holds(self):
        with self.assertRaisesRegex(RuntimeError, '未到达'):
            move_pregrasp(self.arm, timeout=0)
        self.assertIsNone(self.fixture.service.owner)
        self.assertTrue(self.fixture.service.holding)

    def test_pregrasp_opens_closed_gripper_before_return(self):
        self.arm.start_control()
        self.arm.set_joint_positions(self.arm.snapshot()['actual_pos'], profile='preset')
        self.arm.set_catch_pos(.01)
        until = time.monotonic()+3
        while self.arm.snapshot()['gripper_actual_pos'] > .013 and time.monotonic() < until:
            time.sleep(.02)
        self.assertLess(self.arm.snapshot()['gripper_actual_pos'], .013)
        stop_and_hold(self.arm)
        state = move_pregrasp(self.arm, open_gripper=True)
        self.assertAlmostEqual(state['gripper_actual_pos'], .078, delta=.003)
        np.testing.assert_allclose(state['actual_pos'], [0, 1.5, 1.45, -1.52, 0, 0], atol=.03)
        self.assertIsNone(self.fixture.service.owner)
        self.assertTrue(self.fixture.service.holding)

    def test_cannot_take_other_controller_lease(self):
        other = ArmClient(self.fixture.robot, path=self.fixture.path, hardware=False,
                          actuate=True, input_name='quest')
        self.addCleanup(other.close)
        other.start_control()
        self.arm.snapshot()
        with self.assertRaisesRegex(RuntimeError, '占用'):
            move_pregrasp(self.arm)
        self.assertEqual(self.fixture.service.owner['input'], 'quest')
        self.assertIsNone(self.arm.token)


if __name__ == '__main__':
    unittest.main()
