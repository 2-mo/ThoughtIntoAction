from __future__ import annotations

import unittest

from arx5_backend.features import feature_catalog


class CommandCatalogTest(unittest.TestCase):
    def test_each_public_command_resolves_to_one_kernel_action(self) -> None:
        catalog = feature_catalog()
        names = {
            (feature["name"], command["name"])
            for feature in catalog.describe()
            for command in feature["commands"]
        }
        self.assertEqual(len(names), 45)
        self.assertIn(("arm", "preview-pose-live"), names)
        self.assertEqual(
            catalog.resolve("arm", "emergency-damping").kernel_action,
            "emergency",
        )
        features = {feature["name"]: feature for feature in catalog.describe()}
        self.assertEqual(features["teleoperation"]["streams"], ["target"])

    def test_kernel_action_maps_back_to_public_name(self) -> None:
        catalog = feature_catalog()
        self.assertEqual(
            catalog.public_name("handeye_validate"),
            ("calibration", "hand-eye-validate"),
        )


if __name__ == "__main__":
    unittest.main()
