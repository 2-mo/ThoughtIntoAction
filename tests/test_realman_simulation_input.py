"""Synthetic Quest inputs verify the simulation gateway path without an XR device."""
from copy import deepcopy
import unittest

from embodied.devices.robots.realman.simulation_quest import mapping_config, run


class Clock:
    def __init__(self): self.time = 0.
    def now(self): return self.time
    def wait(self, duration): self.time += duration


class Session:
    def __init__(self):
        self.parameters = {'effective_robot_config': {'gripper_max_width': .04, 'gripper_velocity_max_m_s': .02}}
        self.commands = []
    def status(self): return {'pose': [-.4, 0, .5, 0, 0, 0, 1], 'gripper_width': .03}
    def command(self, action, **fields): self.commands.append((action, deepcopy(fields)))


class XR:
    def __init__(self, clock, stale=False): self.clock, self.stale = clock, stale
    @property
    def index(self): return round(self.clock.time / .04)
    def get_time_stamp_ns(self): return 1 if self.stale else self.index+1
    def get_right_grip(self): return .9 if self.index in (1, 2) else 0.
    def get_B_button(self): return not self.stale and self.index >= 5
    def get_right_controller_pose(self): return [.1 if self.index == 2 else 0., 0, 0, 0, 0, 0, 1]
    def get_A_button(self): return False
    def get_right_trigger(self): return .9 if self.index == 4 else 0.


class InputTests(unittest.TestCase):
    def test_release_clutch_axis_speed_and_gripper_then_B_hold(self):
        clock, session = Clock(), Session()
        report = run(session, XR(clock), mapping_config(), 1., now=clock.now, wait=clock.wait)
        targets = [fields['pose'] for action,fields in session.commands if action == 'cartesian']
        self.assertEqual(len(targets), 2)
        self.assertAlmostEqual(targets[-1][0], -.4)
        self.assertAlmostEqual(targets[-1][1], .004)
        self.assertAlmostEqual(targets[-1][2], .5)
        gripper = [fields['gripper_width'] for action,fields in session.commands if action == 'gripper']
        self.assertEqual(len(gripper), 1)
        self.assertLess(gripper[0], .03)
        self.assertEqual(session.commands[-1][0], 'hold')
        self.assertTrue(report['simulation_only'])

    def test_stalled_timestamp_stops_and_holds(self):
        clock, session = Clock(), Session()
        with self.assertRaisesRegex(RuntimeError, 'expired'):
            run(session, XR(clock, stale=True), mapping_config(), 1., now=clock.now, wait=clock.wait)
        self.assertEqual(session.commands[-1][0], 'hold')
        self.assertLess(clock.time, .4)

    def test_invalid_pose_keeps_original_error_and_holds(self):
        clock, session, xr = Clock(), Session(), None
        xr = XR(clock)
        xr.get_right_controller_pose = lambda: [float('nan')] * 7
        with self.assertRaisesRegex(ValueError, 'finite'):
            run(session, xr, mapping_config(), 1., now=clock.now, wait=clock.wait)
        self.assertEqual(session.commands[-1][0], 'hold')


if __name__ == '__main__':
    unittest.main()
