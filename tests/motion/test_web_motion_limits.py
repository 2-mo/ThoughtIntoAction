from __future__ import annotations

import unittest

from embodied.applications.console.kernel.control import (
    WEB_MOTION_SAFETY,
    velocity_scale_from_params,
)


class WebMotionLimitsTest(unittest.TestCase):
    def test_full_speed_uses_hardware_limit_scale(self) -> None:
        self.assertEqual(velocity_scale_from_params({"speed_percent": 100}), 1.0)
        self.assertEqual(WEB_MOTION_SAFETY.max_commanded_velocity_rad_s, 5.5)
        self.assertEqual(
            WEB_MOTION_SAFETY.max_commanded_cartesian_velocity_m_s,
            0.60,
        )

    def test_linear_paths_use_tolerant_guide_spacing(self) -> None:
        self.assertEqual(WEB_MOTION_SAFETY.cartesian_geometry_step_m, 0.006)
        self.assertGreater(WEB_MOTION_SAFETY.cartesian_orientation_step_rad, 0.0)


if __name__ == "__main__":
    unittest.main()
