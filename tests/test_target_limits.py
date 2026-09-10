"""Cartesian target rates tested without a device or wall-clock sleeps."""
import unittest

import numpy as np
import meshcat.transformations as tf

from quest_teleop.target_limits import CartesianTargetLimiter, rotvec


class TargetLimitTests(unittest.TestCase):
    def test_translation_norm_and_unlimited_rotation(self):
        limiter = CartesianTargetLimiter(.25, 0)
        limiter.step([0]*3, [0]*3, now=0, period=.02, reset=True)
        pos, rotation = limiter.step([1, 1, 0], [0, 0, 2], now=.02, period=.02)
        np.testing.assert_allclose(pos, np.array([1, 1, 0]) * .005 / np.sqrt(2))
        np.testing.assert_allclose(rotation, [0, 0, 2])

    def test_rotation_rate_and_unlimited_translation(self):
        limiter = CartesianTargetLimiter(0, 2)
        limiter.step([0]*3, [0]*3, now=0, period=.02, reset=True)
        pos, rotation = limiter.step([1, 2, 3], [1, 1, 0], now=.02, period=.02)
        np.testing.assert_allclose(pos, [1, 2, 3])
        np.testing.assert_allclose(rotation, np.array([1, 1, 0]) * .04 / np.sqrt(2))

    def test_rotation_takes_short_path_across_pi(self):
        limiter = CartesianTargetLimiter(0, 0)
        limiter.step([0]*3, [0, 0, np.deg2rad(179)], now=0, period=.02)
        previous = limiter.orientation.copy()
        limiter.angular_speed = 1
        limiter.step([0]*3, [0, 0, np.deg2rad(-179)], now=.02, period=.02)
        delta = rotvec(tf.quaternion_multiply(limiter.orientation, tf.quaternion_conjugate(previous)))
        np.testing.assert_allclose(delta, [0, 0, .02], atol=1e-10)

    def test_stall_does_not_accumulate_allowance_and_reset_drops_backlog(self):
        limiter = CartesianTargetLimiter(1, 1)
        limiter.step([0]*3, [0]*3, now=0, period=.02, reset=True)
        pos, rotation = limiter.step([1, 0, 0], [0, 1, 0], now=10, period=.02)
        np.testing.assert_allclose(pos, [.02, 0, 0])
        np.testing.assert_allclose(rotation, [0, .02, 0])
        pos, rotation = limiter.step([1, 0, 0], [0, 1, 0], now=11, period=.02, reset=True)
        np.testing.assert_allclose(pos, 0)
        np.testing.assert_allclose(rotation, 0)
        pos, rotation = limiter.step([-.1, 0, 0], [0, -.1, 0], now=11.02, period=.02)
        np.testing.assert_allclose(pos, [-.02, 0, 0])
        np.testing.assert_allclose(rotation, [0, -.02, 0])

    def test_invalid_rates(self):
        for value in (-1, float('nan'), float('inf')):
            for rates in ((value, 0), (0, value)):
                with self.assertRaises(ValueError):
                    CartesianTargetLimiter(*rates)
