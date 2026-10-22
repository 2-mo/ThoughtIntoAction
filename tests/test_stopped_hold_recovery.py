import time
import unittest
from unittest.mock import Mock
import numpy as np
from legacy.robot_control.service import ArmService
from legacy.robot_control.online_motion import OnlineMotion
from embodied.devices.robots.arx5.pinocchio_model import X5PinocchioModel
from embodied.core.project import load_config


class StoppedHoldRecoveryTests(unittest.TestCase):
    def make_service(self):
        robot=X5PinocchioModel()
        backend=Mock()
        backend.robot_config.joint_vel_max=np.ones(6)*5
        s=ArmService(robot,backend,Mock(),backend='simulated',hardware=False,
                     settings=load_config()['arm_service'])
        q=np.array([0,1.5,1.45,-1.52,0,0.])
        s.motion=OnlineMotion(q+.1,np.zeros(6),np.ones(6)*5,.15,s.settings,1/240,robot.lower,robot.upper)
        s.motion.stop();s.motion.finished=True
        s.owner={'input':'pi05-sft','deadline':time.monotonic()-5,'speed':.15}
        s.release_after_stop=True
        now=time.monotonic_ns()
        s.state={'actual_pos':q.tolist(),'actual_vel':[0.]*6,'host_monotonic_ns':now}
        for age in range(600, -1, -4):
            s.sample_history.append({**s.state,'host_monotonic_ns':now-age*1_000_000})
        return s,q

    def recover(self,s):
        return s.request({'op':'recover_hold','service_id':s.service_id,
                          'sent_monotonic_ns':time.monotonic_ns(),'expected_owner':'pi05-sft'})

    def test_reanchor_without_disabling_torque(self):
        s,q=self.make_service()
        result=self.recover(s)
        self.assertTrue(result['recovered'])
        self.assertAlmostEqual(result['previous_tracking_error_rad'],.1)
        np.testing.assert_allclose(s.motion.input.current_position,q)
        self.assertTrue(s.holding)
        self.assertIsNone(s.owner)
        s.arm.set_control_enabled.assert_not_called()

    def test_reject_live_foreign_moving_and_incomplete_stops(self):
        for case in ('live','foreign','moving','unfinished','not_stopping','drifting','nonfinite'):
            with self.subTest(case=case):
                s,_=self.make_service()
                if case=='live':s.owner['deadline']=time.monotonic()+5
                if case=='foreign':s.owner['input']='quest'
                if case=='moving':s.sample_history[-1]['actual_vel']=[.1]*6
                if case=='unfinished':s.motion.finished=False
                if case=='not_stopping':
                    s.release_after_stop=False
                    s.owner['deadline']=time.monotonic()+5
                if case=='drifting':s.sample_history[-1]['actual_pos']=[0.]*6
                if case=='nonfinite':s.sample_history[-1]['actual_pos']=[float('nan')]*6
                with self.assertRaises(ValueError):self.recover(s)
                self.assertIsNotNone(s.owner)
                s.arm.set_control_enabled.assert_not_called()

if __name__=='__main__':unittest.main()
