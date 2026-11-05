"""Dual-arm profile: two complete single-arm profiles on distinct CAN interfaces."""
import json
import unittest
from pathlib import Path

from embodied.devices.robots.arx5.profile import load_dual_profile
from embodied.contracts.paths import PROJECT_ROOT


class DualProfileTests(unittest.TestCase):
    directory = PROJECT_ROOT / "configs/robots/arx5"

    def load(self, values):
        return load_dual_profile(values, self.directory)

    def test_repository_dual_profile_has_two_arms(self):
        values = json.loads((self.directory / "dual.json").read_text())
        profile = self.load(values)
        # left arm is wired to can1, right to can0 (verified against the bench)
        self.assertEqual([arm.can_interface for arm in profile.arms], ["can1", "can0"])
        self.assertEqual(profile.left.joint_count, 6)
        self.assertEqual(profile.joint_count, 12)

    def test_duplicate_interface_is_rejected(self):
        with self.assertRaises(ValueError):
            self.load({"version": 1, "arms": [
                {"id": "left", "can_interface": "can0"},
                {"id": "right", "can_interface": "can0"},
            ]})

    def test_missing_or_duplicate_ids_are_rejected(self):
        for arms in (
            [{"id": "left", "can_interface": "can0"}],
            [{"id": "left", "can_interface": "can0"}, {"id": "left", "can_interface": "can1"}],
            [{"can_interface": "can0"}, {"id": "right", "can_interface": "can1"}],
            [{"id": "middle", "can_interface": "can0"}, {"id": "right", "can_interface": "can1"}],
        ):
            with self.assertRaises(ValueError):
                self.load({"version": 1, "arms": arms})

    def test_describe_arms_lists_each_side(self):
        values = json.loads((self.directory / "dual.json").read_text())
        arms = self.load(values).describe_arms()
        self.assertEqual([arm["id"] for arm in arms], ["arm1", "arm2"])
        self.assertEqual([arm["side"] for arm in arms], ["left", "right"])
        self.assertEqual([arm["can_interface"] for arm in arms], ["can1", "can0"])
        self.assertEqual([len(arm["joint_min"]) for arm in arms], [6, 6])
        self.assertEqual(len(arms[0]["park_q"]), 6)

    def test_unknown_keys_are_rejected(self):
        with self.assertRaises(ValueError):
            self.load({"version": 1, "arms": [], "mode": "dual"})
        with self.assertRaises(ValueError):
            self.load({"version": 1, "arms": [
                {"id": "left", "can_interface": "can0", "joints": 6},
                {"id": "right", "can_interface": "can1"},
            ]})


if __name__ == "__main__":
    unittest.main()
