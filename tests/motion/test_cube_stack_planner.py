from __future__ import annotations

from types import SimpleNamespace
import unittest

from embodied.console.features.cube.stack import speed_partitioned_motion_runs
from embodied.tasks.cube.config import load_config
from embodied.tasks.cube.planner import (
    Action,
    build_stack_actions,
    consecutive_move_runs,
    motion_plan_runs,
)


class CubeStackPlannerTest(unittest.TestCase):
    def test_stack_splits_fast_transit_from_slow_precision_motion(self) -> None:
        orientation = (0.0, 0.0, 0.0, 1.0)
        actions = (
            Action("move", "pick approach", (0.3, 0.0, 0.2), orientation, "smooth"),
            Action(
                "move",
                "vertical grasp descent",
                (0.3, 0.0, 0.08),
                orientation,
                "linear",
            ),
            Action("gripper", "close", gripper_width_m=0.03),
            Action("move", "vertical lift", (0.3, 0.0, 0.2), orientation, "linear"),
            Action(
                "move",
                "continuous transfer above stack target",
                (0.4, 0.0, 0.2),
                orientation,
                "smooth",
            ),
            Action(
                "move",
                "vertical stack descent",
                (0.4, 0.0, 0.1),
                orientation,
                "linear",
            ),
            Action("gripper", "release", gripper_width_m=0.08),
            Action(
                "move",
                "vertical retreat after stack",
                (0.4, 0.0, 0.2),
                orientation,
                "linear",
            ),
        )

        runs = speed_partitioned_motion_runs(
            actions,
            transit_velocity_scale=0.55,
            precision_velocity_scale=0.18,
        )

        self.assertEqual([start for start, _, _ in runs], [0, 1, 3, 5, 7])
        self.assertEqual([len(run) for _, run, _ in runs], [1, 1, 2, 1, 1])
        self.assertEqual(
            [scale for _, _, scale in runs],
            [0.55, 0.18, 0.55, 0.18, 0.55],
        )

    def test_stack_groups_motion_around_tool_barriers(self) -> None:
        config = load_config()
        orientation = (0.0, 0.0, 0.0, 1.0)
        pick_actions = (
            Action("gripper", "open", gripper_width_m=0.08),
            Action(
                "move",
                "approach",
                target_xyz_m=(0.30, 0.00, 0.20),
                orientation_xyzw=orientation,
                path_mode="smooth",
            ),
            Action(
                "move",
                "grasp",
                target_xyz_m=(0.30, 0.00, 0.08),
                orientation_xyzw=orientation,
                path_mode="linear",
            ),
            Action("gripper", "precontact", gripper_width_m=0.05),
            Action("force_close", "contact"),
            Action("wait", "settle", wait_s=0.1),
            Action(
                "move",
                "vertical lift",
                target_xyz_m=(0.30, 0.00, 0.20),
                orientation_xyzw=orientation,
                path_mode="linear",
            ),
        )
        target = SimpleNamespace(
            base_xyz_m=(
                config.table.center_x_m,
                config.table.center_y_m,
                config.table.top_z_m + config.cube_edge_m / 2.0,
            )
        )

        actions = build_stack_actions(pick_actions, target, config)
        runs = consecutive_move_runs(actions)

        self.assertEqual([len(run) for _, run in runs], [2, 3, 1])
        self.assertEqual(runs[0][1][0].blend_radius_m, 0.0)
        self.assertEqual(runs[0][1][-1].blend_radius_m, 0.0)
        self.assertEqual(runs[1][1][0].blend_radius_m, 0.0)
        self.assertEqual(runs[1][1][1].blend_radius_m, 0.0)
        self.assertEqual(runs[1][1][-1].blend_radius_m, 0.0)

        planned_runs = motion_plan_runs(actions)
        self.assertEqual(
            [tuple(action.path_mode for action in run) for _, run in planned_runs],
            [
                ("smooth", "linear"),
                ("linear", "smooth", "linear"),
                ("linear",),
            ],
        )
        transfer = next(
            action
            for action in actions
            if action.label == "continuous transfer above stack target"
        )
        self.assertEqual(transfer.path_mode, "smooth")


if __name__ == "__main__":
    unittest.main()
