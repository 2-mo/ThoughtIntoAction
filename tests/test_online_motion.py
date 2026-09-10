import unittest
import numpy as np
from robot_control.online_motion import OnlineMotion, DEFAULTS


class OnlineMotionTests(unittest.TestCase):
    def planner(self):
        return OnlineMotion(np.zeros(6), np.zeros(6), [3.]*6, 2.,
                            {**DEFAULTS, 'joint_soft_limit_margin_rad': .05}, .01, [-3.]*6, [3.]*6)

    def check_step(self, planner, previous):
        q, v = planner.step()
        a = np.array(planner.input.current_acceleration)
        self.assertLessEqual(abs(v).max(), max(planner.input.max_velocity) + 1e-7)
        self.assertLessEqual(abs(a).max(), max(planner.input.max_acceleration) + 1e-7)
        self.assertLessEqual(abs(a-previous).max()/.01, max(planner.input.max_jerk) + 1e-6)
        return a

    def test_changing_targets_preserves_velocity_acceleration_and_jerk(self):
        planner = self.planner()
        a = np.zeros(6)
        for i in range(600):
            if i < 150:
                planner.target([.8 + .1*np.sin(i/20)]*6)
            elif i == 150:
                planner.target([-.5]*6)
            a = self.check_step(planner, a)
        self.assertTrue(planner.finished)
        np.testing.assert_allclose(planner.input.current_position, [-.5]*6, atol=1e-7)
        np.testing.assert_allclose(planner.input.current_velocity, 0, atol=1e-7)
        np.testing.assert_allclose(a, 0, atol=1e-7)

    def test_stop_discards_old_target_and_brakes_to_zero_then_can_resume(self):
        planner = self.planner()
        planner.target([2.]*6)
        a = np.zeros(6)
        for _ in range(60):
            a = self.check_step(planner, a)
        before = np.array(planner.input.current_position)
        planner.stop()
        for _ in range(250):
            a = self.check_step(planner, a)
        self.assertTrue(planner.finished)
        self.assertGreater(planner.input.current_position[0], before[0])
        self.assertLess(planner.input.current_position[0], 1.5)
        np.testing.assert_allclose(planner.input.current_velocity, 0, atol=1e-8)
        np.testing.assert_allclose(a, 0, atol=1e-8)
        planner.target([0.]*6)
        for _ in range(400):
            a = self.check_step(planner, a)
        np.testing.assert_allclose(planner.input.current_position, 0, atol=1e-7)

    def test_uncapped_teleop_and_preset_keep_home_speed_limit(self):
        hardware = np.array([1., 2., 3., 4., 5., 6.])
        for speed, expected in ((0., hardware), (2.5, np.minimum(hardware, 2.5))):
            planner = OnlineMotion(np.zeros(6), np.zeros(6), hardware, speed,
                {**DEFAULTS, 'preset_speed_rad_s': 0., 'home_speed_rad_s': .6,
                 'joint_soft_limit_margin_rad': .05}, .01, [-3.]*6, [3.]*6)
            for profile in ('teleop', 'preset', 'home', 'teleop'):
                planner.configure(profile)
                np.testing.assert_allclose(planner.input.max_velocity,
                    np.minimum(expected, .6) if profile == 'home' else expected)

    def test_collection_profile_reduces_following_error_with_bounded_motion(self):
        errors = []
        for acceleration, jerk in ((6., 30.), (12., 120.)):
            planner = OnlineMotion(np.zeros(6), np.zeros(6), [5.]*6, 0,
                {**DEFAULTS, 'joint_soft_limit_margin_rad': .05,
                 'max_joint_acceleration_rad_s2': acceleration,
                 'max_joint_jerk_rad_s3': jerk}, .02, [-3.]*6, [3.]*6)
            previous = np.zeros(6)
            samples = []
            for i in range(500):
                target = .2 * np.sin(2 * np.pi * .5 * i * .02)
                planner.target([target]*6)
                q, velocity = planner.step()
                actual_acceleration = np.array(planner.input.current_acceleration)
                self.assertLessEqual(np.max(np.abs(velocity)), 5. + 1e-7)
                self.assertLessEqual(np.max(np.abs(actual_acceleration)), acceleration + 1e-7)
                self.assertLessEqual(np.max(np.abs(actual_acceleration - previous)) / .02, jerk + 1e-6)
                previous = actual_acceleration
                if i > 100:
                    samples.append(q[0] - target)
            errors.append(np.sqrt(np.mean(np.square(samples))))
        self.assertLess(errors[1], .75 * errors[0])

    def test_two_stage_home_repeated_targets_keep_midpoint_and_final_speed(self):
        start = np.array([2., -1., .5, 0., .3, 0.])
        planner = OnlineMotion(start, np.zeros(6), [5.]*6, 0.,
            {**DEFAULTS, 'home_speed_rad_s': 2., 'home_final_speed_rad_s': .75,
             'home_acceleration_rad_s2': 2., 'home_jerk_rad_s3': 16.,
             'joint_soft_limit_margin_rad': .05}, .01, [-3.]*6, [3.]*6)
        phases = set()
        for _ in range(2000):
            planner.target(np.zeros(6), 'home')
            phase = planner.home_phase
            phases.add(phase)
            q, v = planner.step()
            self.assertLessEqual(np.max(np.abs(v)), (2. if phase == 'first' else .75) + 1e-7)
            if phase == 'first' and planner.home_phase == 'final':
                np.testing.assert_allclose(q, start / 2, atol=1e-8)
                np.testing.assert_allclose(v, 0, atol=1e-8)
            if planner.finished:
                break
        self.assertTrue(planner.finished)
        self.assertEqual(phases, {'first', 'final'})
        np.testing.assert_allclose(q, 0, atol=1e-8)
        planner.stop()
        self.assertIsNone(planner.home_phase)
        planner.target([.2]*6, 'home')
        np.testing.assert_allclose(planner._home_midpoint, [.1]*6, atol=1e-8)

    def test_two_stage_floor_check_covers_second_half_without_mutating_motion(self):
        from types import SimpleNamespace
        planner = OnlineMotion([2.]*6, np.zeros(6), [5.]*6, 0.,
            {**DEFAULTS, 'home_speed_rad_s': 2., 'home_final_speed_rad_s': .75,
             'joint_soft_limit_margin_rad': .05}, .01, [-3.]*6, [3.]*6)
        seen = []
        def fk(q):
            seen.append(q[0])
            result = np.eye(4)
            result[2, 3] = -1. if q[0] < .5 else 1.
            return result
        before = planner.snapshot()
        self.assertFalse(planner.path_above_floor([0.]*6, 'home', SimpleNamespace(fk=fk), 0.))
        self.assertLess(min(seen), .5)
        self.assertEqual(planner.snapshot(), before)

    def test_home_has_independent_limits(self):
        planner = self.planner()
        planner.target([.5]*6, 'home')
        a = np.zeros(6)
        for _ in range(600):
            a = self.check_step(planner, a)
        self.assertEqual(planner.input.max_velocity, [.3]*6)
        self.assertEqual(planner.input.max_acceleration, [.5]*6)
        self.assertEqual(planner.input.max_jerk, [2.]*6)
        self.assertTrue(planner.finished)

    def test_switch_to_home_brakes_with_previous_limits_before_slow_profile(self):
        planner = self.planner()
        planner.target([1.]*6)
        for _ in range(40):
            planner.step()
        planner.target([0.]*6, 'home')
        self.assertEqual(planner.profile, 'teleop')
        self.assertTrue(planner.stopping)
        previous_a = np.array(planner.input.current_acceleration)
        for _ in range(1000):
            planner.target([0.]*6, 'home')
            previous_a = self.check_step(planner, previous_a)
        self.assertEqual(planner.profile, 'home')
        self.assertTrue(planner.finished)
        np.testing.assert_allclose(planner.input.current_position, 0, atol=1e-7)

    def test_double_speed_home_halves_rest_to_rest_duration(self):
        def duration(speed, acceleration, jerk):
            planner = OnlineMotion([.8, -.4, .2, 0, .1, 0], np.zeros(6), [3.]*6, 3.,
                {**DEFAULTS, 'joint_soft_limit_margin_rad': .05,
                 'home_speed_rad_s': speed, 'home_acceleration_rad_s2': acceleration,
                 'home_jerk_rad_s3': jerk}, .02, [-3.]*6, [3.]*6)
            planner.target([0.]*6, 'home')
            for count in range(1, 1000):
                planner.step()
                if planner.finished:
                    np.testing.assert_allclose(planner.input.current_position, 0, atol=1e-7)
                    return count * .02
            self.fail('Home trajectory did not finish')
        self.assertAlmostEqual(duration(.6, 2., 16.), duration(.3, .5, 2.) / 2, delta=.03)
