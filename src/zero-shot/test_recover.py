from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'tests'))
from arm_service_fixture import ServiceFixture
from robot_control.client import ArmClient
from recover import HOME_SPEED, HOME_MOTION, home, hold, move_pregrasp, damping
import recover


class RecoveryTests(unittest.TestCase):
    def setUp(self):
        self.fixture=ServiceFixture(table_safety=True)
        self.addCleanup(self.fixture.close)
        self.arm=ArmClient(self.fixture.robot,path=self.fixture.path,hardware=False,actuate=True,
                           input_name='terminal-recovery',speed=HOME_SPEED,motion=HOME_MOTION)
        self.addCleanup(self.arm.close)

    def test_home_via_service_and_hold(self):
        initial=self.arm.snapshot()['gripper_actual_pos']
        state=home(self.arm,timeout=20)
        np.testing.assert_allclose(state['actual_pos'],0,atol=.03)
        self.assertAlmostEqual(state['gripper_actual_pos'], min(initial,self.arm.settings['gripper_open_limit_m']),places=4)
        self.assertTrue(self.fixture.service.holding)
        self.assertIsNone(self.fixture.service.owner)

    def test_hold_from_damping(self):
        before=self.arm.snapshot()['actual_pos']
        state=hold(self.arm)
        np.testing.assert_allclose(state['actual_pos'],before,atol=.01)
        self.assertTrue(self.fixture.service.holding)

    def test_damping_and_back_to_hold(self):
        hold(self.arm)
        damping(self.arm)
        self.assertEqual(self.arm.info['mode'], 'damping')
        self.assertIsNone(self.fixture.service.owner)
        self.assertIsNone(self.arm.token)
        hold(self.arm)
        self.assertEqual(self.arm.info['mode'], 'hold')
        self.assertIsNone(self.fixture.service.owner)

    def test_damping_is_idempotent(self):
        with patch.object(self.arm, 'start_control') as acquire:
            damping(self.arm)
            damping(self.arm)
        acquire.assert_not_called()

    def test_damping_rejects_foreign_owner(self):
        other = ArmClient(self.fixture.robot, path=self.fixture.path, hardware=False,
                          actuate=True, input_name='quest')
        self.addCleanup(other.close)
        other.start_control()
        with self.assertRaisesRegex(RuntimeError, 'quest'):
            damping(self.arm)
        self.assertEqual(self.fixture.service.owner['input'], 'quest')

    def test_pregrasp_with_collection_settings(self):
        state=move_pregrasp(self.arm,open_gripper=True)
        np.testing.assert_allclose(state['actual_pos'],[0,1.5,1.45,-1.52,0,0],atol=.03)
        self.assertAlmostEqual(state['gripper_actual_pos'],.078,delta=.003)
        self.assertTrue(self.fixture.service.holding)
        self.assertIsNone(self.fixture.service.owner)

    def test_reject_foreign_owner(self):
        other=ArmClient(self.fixture.robot,path=self.fixture.path,hardware=False,actuate=True,input_name='quest')
        self.addCleanup(other.close)
        other.start_control()
        with self.assertRaisesRegex(RuntimeError,'quest'):
            hold(self.arm)
        self.assertEqual(self.fixture.service.owner['input'],'quest')

    def test_status_is_read_only_and_does_not_stop_policy(self):
        other = ArmClient(self.fixture.robot, path=self.fixture.path, hardware=False,
                          actuate=True, input_name='quest')
        self.addCleanup(other.close)
        other.start_control()
        with patch.object(sys, 'argv', ['recover.py', '--status', '--virtual',
                                      '--socket', str(self.fixture.path)]), \
                patch.object(recover, 'stop_policy_processes') as stop:
            recover.main()
        stop.assert_not_called()
        self.assertEqual(self.fixture.service.owner['input'], 'quest')


if __name__=='__main__':unittest.main()
