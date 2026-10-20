from __future__ import annotations

import unittest

from legacy.arx5_control.gripper_calibration import DEFAULT_GRIPPER_CALIBRATION_PATH
from legacy.arx5_control.payload_profiles import DEFAULT_PAYLOAD_ROOT
from embodied.core.arx5_paths import CALIBRATIONS_ROOT, DATASETS_ROOT, OUTPUTS_ROOT, RUNTIME_ROOT
from embodied.data.groot.raw_episode import DEFAULT_RAW_ROOT


class PersistentDataPathTest(unittest.TestCase):
    def test_visible_local_directories_are_not_dot_directories(self) -> None:
        for root in (CALIBRATIONS_ROOT, DATASETS_ROOT, OUTPUTS_ROOT, RUNTIME_ROOT):
            self.assertFalse(root.name.startswith("."))

    def test_calibrations_are_outside_runtime_directory(self) -> None:
        self.assertEqual(
            DEFAULT_GRIPPER_CALIBRATION_PATH,
            CALIBRATIONS_ROOT / "gripper" / "calibration.json",
        )
        self.assertEqual(DEFAULT_PAYLOAD_ROOT, CALIBRATIONS_ROOT / "payload")
        self.assertNotEqual(CALIBRATIONS_ROOT, RUNTIME_ROOT)

    def test_collection_outputs_are_outside_runtime_directory(self) -> None:
        self.assertEqual(DEFAULT_RAW_ROOT, DATASETS_ROOT / "data-coll")
        self.assertNotEqual(DATASETS_ROOT, RUNTIME_ROOT)


if __name__ == "__main__":
    unittest.main()
