from __future__ import annotations

import unittest

from embodied.tasks.cube.config import load_config
from embodied.tasks.cube.planner import (
    maximum_plausible_contact_width_m,
)


class CubeGripperGeometryTest(unittest.TestCase):
    def test_plausible_contact_width_accepts_cube_diagonal(self) -> None:
        config = load_config()

        maximum_width_m = maximum_plausible_contact_width_m(config)

        self.assertAlmostEqual(maximum_width_m, 0.043)
        self.assertLess(maximum_width_m, config.motion.precontact_width_m)


if __name__ == "__main__":
    unittest.main()
