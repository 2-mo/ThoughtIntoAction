"""Same fast trajectory in MuJoCo, with/without inertial feedforward; no CAN."""
from dataclasses import replace
import unittest

import numpy as np

from embodied.contracts.control import ControlCommand, Mode
from embodied.control.motion_limits import MotionLimits
from embodied.control.settings import load_settings
from embodied.core.presets import DEFAULT_GRASP_READY_JOINTS
from embodied.devices.robots.arx5.simulation import MujocoBackend
from embodied.motion.offline import plan_ruckig_joint_trajectory


class DynamicsFeedforwardTests(unittest.TestCase):
    def run_move(self, start, goal, *, feedforward):
        config = replace(load_settings().robot, park_q=tuple(start))
        backend = MujocoBackend(config)
        policy = MotionLimits.load(config).resolve({})
        try:
            curve = plan_ruckig_joint_trajectory(np.array(start), np.array(goal), np.zeros(6),
                min_position=backend.kinematics.lower, max_position=backend.kinematics.upper,
                max_velocity=policy['joint_velocity_rad_s'], max_acceleration=policy['acceleration'],
                max_jerk=policy['jerk'], sample_rate_hz=250, minimum_duration_s=.004)
            samples = []
            for tick in range(500):
                now = tick*.004
                q, dq, ddq = curve.at_time(now)
                backend.step(ControlCommand(Mode.COMMAND_FOLLOWING,
                    joint_target=q, joint_velocity_target=dq,
                    joint_acceleration_target=ddq if feedforward else None,
                    joint_step_limit=np.array(policy['joint_velocity_rad_s'])/250, planned=True), now)
                state = backend.read_state(now)
                samples.append((state.q, state.dq))
                # Feedforward must never retime or offset the reference.
                np.testing.assert_array_equal(backend.telemetry()['sent_q'], q)
                np.testing.assert_array_equal(backend.telemetry()['sent_dq'], dq)
            qs = np.array([q for q, _ in samples])
            velocities = np.array([dq for _, dq in samples])
            direction = np.sign(np.array(goal)-start)
            overshoot = np.maximum(0., np.max((qs-goal)*direction, axis=0))
            return curve.duration_s, overshoot, np.max(np.abs(velocities)), qs[-1]
        finally:
            backend.close()

    def test_inertia_feedforward_reduces_overshoot_at_unchanged_speed(self):
        home, ready = np.zeros(6), np.array(DEFAULT_GRASP_READY_JOINTS)
        baseline = self.run_move(home, ready, feedforward=False)
        corrected = self.run_move(home, ready, feedforward=True)
        self.assertEqual(corrected[0], baseline[0])
        self.assertLess(corrected[0], .7)
        self.assertGreater(np.rad2deg(baseline[1][1]), 2.)  # Reproduces the J2 issue.
        self.assertLess(np.rad2deg(np.max(corrected[1])), .25)
        self.assertGreater(corrected[2], 4.8)
        np.testing.assert_allclose(corrected[3], ready, atol=np.deg2rad(.1))
        reverse = self.run_move(ready, home, feedforward=True)
        self.assertLess(np.rad2deg(np.max(reverse[1])), .25)
        self.assertGreater(reverse[2], 4.8)
        np.testing.assert_allclose(reverse[3], home, atol=np.deg2rad(.1))


if __name__ == '__main__':
    unittest.main()
