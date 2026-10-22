import unittest
import numpy as np
from embodied.core.safety import TablePlane
from embodied.devices.robots.arx5.pinocchio_model import X5PinocchioModel
from legacy.robot_control.online_motion import OnlineMotion, DEFAULTS


class TableSafetyTests(unittest.TestCase):
    def test_zero_margin_allows_contact_and_retains_horizontal_motion(self):
        plane = TablePlane(0, 0)
        np.testing.assert_allclose(plane.clamp([.2, -.1, -.3]), [.2, -.1, 0])
        np.testing.assert_allclose(plane.clamp([.3, .1, .05]), [.3, .1, .05])
        np.testing.assert_allclose(plane.clamp([0, 0, 0]), [0, 0, 0])
        self.assertAlmostEqual(TablePlane(.1, .02).minimum_z, .12)
        for z, margin in [(float('nan'), 0), (0, -.1), (0, float('inf'))]:
            with self.assertRaises(ValueError): TablePlane(z, margin)

    def test_all_profiles_reject_below_table_without_mutating_previous_plan(self):
        robot = X5PinocchioModel(eef='eef_link')
        planner = OnlineMotion([0]*6, [0]*6, [3]*6, 3,
            {**DEFAULTS, 'joint_soft_limit_margin_rad': .05}, .02, robot.lower, robot.upper)
        before = planner.snapshot()
        for profile in ('teleop', 'home', 'preset'):
            self.assertFalse(planner.path_above_floor([0, 1.5, 0, 0, 0, 0], profile, robot, 0))
            self.assertTrue(planner.path_above_floor([0, .5, 0, 0, 0, 0], profile, robot, 0))
            self.assertEqual(planner.snapshot(), before)

    def test_path_check_inspects_interior_even_when_endpoints_are_safe(self):
        class MidpointObstacle:
            def fk(self, q):
                pose = np.eye(4)
                pose[2, 3] = -.1 if .2 < q[0] < .4 else .1
                return pose
        planner = OnlineMotion([0]*6, [0]*6, [3]*6, 3,
            {**DEFAULTS, 'joint_soft_limit_margin_rad': .05}, .02, [-3]*6, [3]*6)
        self.assertFalse(planner.path_above_floor([.6, 0, 0, 0, 0, 0], 'home', MidpointObstacle(), 0))
