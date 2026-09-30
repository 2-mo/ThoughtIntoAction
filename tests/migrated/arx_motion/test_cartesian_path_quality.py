from __future__ import annotations

import math
import unittest

import numpy as np

from arx5_control.cartesian import CartesianPlanner, CartesianPlanningError


class CartesianPathQualityTest(unittest.TestCase):
    def test_rejects_discontinuous_ik_branch_jump(self) -> None:
        planner = object.__new__(CartesianPlanner)
        planner._joint_dof = 6
        planner._max_ik_step_rad = math.radians(12.0)
        positions = np.zeros((4, 6), dtype=np.float64)
        positions[2:, 4] = math.radians(30.0)

        with self.assertRaisesRegex(CartesianPlanningError, "joint 5 jumps"):
            planner._check_joint_path_continuity(positions)

    def test_accepts_continuous_ik_path(self) -> None:
        planner = object.__new__(CartesianPlanner)
        planner._joint_dof = 6
        planner._max_ik_step_rad = math.radians(12.0)
        positions = np.zeros((5, 6), dtype=np.float64)
        positions[:, 2] = np.linspace(0.0, math.radians(20.0), 5)

        planner._check_joint_path_continuity(positions)


if __name__ == "__main__":
    unittest.main()
