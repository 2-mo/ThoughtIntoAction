from __future__ import annotations

import unittest

import numpy as np

from embodied.motion import analyze_sampled_motion
from embodied.motion.offline import plan_blended_joint_trajectory


class BlendedJointTrajectoryTest(unittest.TestCase):
    def test_preserves_keypoints_without_stopping_at_each_one(self) -> None:
        anchors = np.asarray(
            [
                [0.0, 0.0, 0.0],
                [0.4, 0.2, -0.1],
                [0.8, 0.1, -0.2],
            ],
            dtype=np.float64,
        )
        velocity_limit = np.full(3, 1.0, dtype=np.float64)
        acceleration_limit = np.full(3, 2.0, dtype=np.float64)
        jerk_limit = np.full(3, 10.0, dtype=np.float64)

        trajectory = plan_blended_joint_trajectory(
            anchors,
            max_velocity=velocity_limit,
            max_acceleration=acceleration_limit,
            max_jerk=jerk_limit,
            sample_rate_hz=100.0,
        )

        keypoint_matches = np.flatnonzero(
            np.all(
                np.isclose(
                    trajectory.positions,
                    anchors[1],
                    rtol=0.0,
                    atol=1e-12,
                ),
                axis=1,
            )
        )
        self.assertEqual(keypoint_matches.size, 1)
        keypoint_index = int(keypoint_matches[0])
        self.assertGreater(abs(trajectory.velocities[keypoint_index, 0]), 1e-6)
        self.assertTrue(np.allclose(trajectory.positions[-1], anchors[-1]))
        self.assertTrue(np.allclose(trajectory.velocities[-1], 0.0))

        sampled_motion = analyze_sampled_motion(
            np.vstack((anchors[0], trajectory.positions)),
            np.concatenate(([0.0], trajectory.sample_times_s)),
            command_velocities=np.vstack(
                (np.zeros(anchors.shape[1]), trajectory.velocities)
            ),
        )
        self.assertLessEqual(
            sampled_motion.required_time_scale(
                velocity_limit=velocity_limit,
                acceleration_limit=acceleration_limit,
                jerk_limit=jerk_limit,
            ),
            1.0 + 1e-6,
        )


if __name__ == "__main__":
    unittest.main()
