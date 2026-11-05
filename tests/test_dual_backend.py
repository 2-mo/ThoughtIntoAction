"""Composite dual-arm backend: 12-joint feedback and per-arm command splitting."""
import unittest

import numpy as np

from embodied.contracts.control import ControlCommand, Mode, RobotState
from embodied.devices.robots.arx5.dual import DualBackend, combined_mode
from embodied.devices.robots.arx5.profile import load_dual_profile
from embodied.contracts.paths import PROJECT_ROOT


class FakeArm:
    def __init__(self, offset):
        self.offset = offset
        self.commands = []
        self.stopped = closed = False

    def read_state(self, now):
        return RobotState(
            q=np.full(6, float(self.offset)), dq=np.zeros(6),
            pose=np.array([self.offset, 0, 0, 0, 0, 0, 1.0]),
            gripper_width=0.01 * self.offset, received_at=now,
            mode=Mode.COMMAND_FOLLOWING, gripper_mode=Mode.COMMAND_FOLLOWING,
        )

    def step(self, command, now, **options):
        self.commands.append((command, now))

    def telemetry(self):
        return {'offset': self.offset}

    def runtime_parameters(self):
        return {'offset': self.offset}

    def request_stop(self):
        self.stopped = True

    def close(self):
        self.closed = True


class DualBackendTests(unittest.TestCase):
    def setUp(self):
        import json
        values = json.loads((PROJECT_ROOT / "configs/robots/arx5/dual.json").read_text())
        self.profile = load_dual_profile(values, PROJECT_ROOT / "configs/robots/arx5")
        self.arms = [FakeArm(1), FakeArm(2)]
        self.backend = DualBackend(self.profile, self.arms)

    def test_feedback_concatenates_both_arms(self):
        state = self.backend.read_state(1.0)
        self.assertEqual(state.q.tolist(), [1.0] * 6 + [2.0] * 6)
        self.assertEqual(state.poses[1][0], 2.0)
        self.assertEqual(state.gripper_widths, (0.01, 0.02))
        self.assertEqual(state.mode, Mode.COMMAND_FOLLOWING)

    def test_mode_is_the_most_restrictive_arm(self):
        self.assertEqual(combined_mode([Mode.COMMAND_FOLLOWING, Mode.FREE_DRIVE]), Mode.FREE_DRIVE)
        self.assertEqual(combined_mode([Mode.FREE_DRIVE, Mode.DISABLED]), Mode.DISABLED)

    def test_pose_targets_are_routed_per_arm(self):
        left = np.full(7, 0.1)
        right = np.full(7, 0.2)
        self.backend.step(ControlCommand(mode=Mode.COMMAND_FOLLOWING, pose_targets=(left, right)), 1.0)
        self.assertTrue(np.allclose(self.arms[0].commands[-1][0].pose_target, left))
        self.assertTrue(np.allclose(self.arms[1].commands[-1][0].pose_target, right))

    def test_joint_targets_split_in_half(self):
        target = np.arange(12, dtype=float)
        self.backend.step(ControlCommand(mode=Mode.COMMAND_FOLLOWING, joint_target=target), 1.0)
        self.assertEqual(self.arms[0].commands[-1][0].joint_target.tolist(), list(range(6)))
        self.assertEqual(self.arms[1].commands[-1][0].joint_target.tolist(), list(range(6, 12)))

    def test_stop_and_close_reach_both_arms(self):
        self.backend.request_stop()
        self.backend.close()
        self.assertTrue(all(arm.stopped and arm.closed for arm in self.arms))


if __name__ == "__main__":
    unittest.main()
