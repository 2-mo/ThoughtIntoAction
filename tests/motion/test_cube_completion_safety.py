from __future__ import annotations

from types import SimpleNamespace
import unittest

import numpy as np

from legacy.arx5_control import ControllerMode
from embodied.console.kernel.control import ControlKernel
from embodied.console.features.cube.stack import execute_cube_stack
from embodied.tasks.cube.planner import Action


class FakeParkController:
    def __init__(self) -> None:
        self.mode = ControllerMode.ACTIVE
        self.safety = SimpleNamespace(minimum_motion_duration_s=2.0)
        self.state = SimpleNamespace(
            joint_position=np.zeros(6, dtype=np.float64),
            gripper_position=0.050,
        )
        self.held = False
        self.damped = False

    def get_state(self):
        return self.state

    def minimum_gripper_duration(self, delta_m: float) -> float:
        return abs(delta_m) * 10.0

    def move_gripper(self, target_m: float, duration_s: float):
        self.state.gripper_position = target_m
        return SimpleNamespace(target_m=target_m, duration_s=duration_s)

    def hold_current(self):
        self.held = True
        return self.state

    def damping(self):
        self.damped = True
        raise AssertionError("normal park must not enter damping")


class CubeCompletionSafetyTest(unittest.TestCase):
    def test_park_restores_reference_gripper_and_holds(self) -> None:
        kernel = ControlKernel("can0")
        controller = FakeParkController()
        kernel.controller = controller
        kernel.reference_joint_position = tuple(0.0 for _ in range(6))
        kernel.reference_gripper_position = 0.025
        kernel.park_preview = SimpleNamespace(
            target_joint_position=np.zeros(6, dtype=np.float64),
            start_joint_position=np.zeros(6, dtype=np.float64),
        )
        kernel.park_preview_velocity_scale = 0.1

        result = kernel.make_operation("park", {"speed_percent": 10})()

        self.assertAlmostEqual(controller.state.gripper_position, kernel.reference_gripper_position)
        self.assertTrue(controller.held)
        self.assertFalse(controller.damped)
        self.assertAlmostEqual(result.target_m, kernel.reference_gripper_position)

    def test_recoverable_cube_failure_keeps_active_hold(self) -> None:
        controller = FakeParkController()

        ControlKernel._hold_after_cube_failure(controller, "failure")

        self.assertTrue(controller.held)
        self.assertFalse(controller.damped)

    def test_preplanned_stack_executes_frozen_phases_then_holds(self) -> None:
        controller = FakeParkController()
        controller.safety.terminal_position_tolerance_rad = np.deg2rad(1.0)
        executed = []
        controller.execute_motion_plan = lambda plan, **kwargs: (
            executed.append(plan)
            or SimpleNamespace(final_error_rad=np.zeros(6, dtype=np.float64))
        )
        controller.force_close_gripper = lambda *args, **kwargs: SimpleNamespace(
            stop_reason="contact",
            final_width_m=0.030,
            max_abs_torque_nm=0.2,
            elapsed_s=0.5,
        )
        motion = SimpleNamespace(
            velocity_scale=0.1,
            closing_torque_nm=0.2,
            minimum_gripper_width_m=0.0,
            gripper_timeout_s=5.0,
            contact_hold_s=0.2,
            precontact_width_m=0.050,
        )
        preview = SimpleNamespace(
            actions=(
                Action("move", "approach"),
                Action("force_close", "contact"),
                Action("move", "transfer"),
                Action("gripper", "release", gripper_width_m=0.080),
            ),
            phases=(
                SimpleNamespace(start_index=0, action_count=1, plan="approach"),
                SimpleNamespace(start_index=2, action_count=1, plan="transfer"),
            ),
            return_plan="return",
            config=SimpleNamespace(cube_edge_m=0.030, motion=motion),
        )

        result = execute_cube_stack(controller, preview)

        self.assertEqual(executed, ["approach", "transfer", "return"])
        self.assertAlmostEqual(controller.state.gripper_position, 0.0)
        self.assertTrue(controller.held)
        self.assertEqual(result["mode"], "active")

    def test_stack_callback_runs_after_place_and_before_return(self) -> None:
        controller = FakeParkController()
        events: list[str] = []
        controller.execute_motion_plan = lambda plan, **kwargs: (
            events.append(str(plan))
            or SimpleNamespace(final_error_rad=np.zeros(6, dtype=np.float64))
        )
        preview = SimpleNamespace(
            actions=(Action("move", "place"),),
            phases=(SimpleNamespace(start_index=0, action_count=1, plan="place"),),
            return_plan="return",
            config=SimpleNamespace(
                cube_edge_m=0.030,
                motion=SimpleNamespace(velocity_scale=0.1),
            ),
        )

        execute_cube_stack(
            controller,
            preview,
            before_return=lambda: events.append("stop-recording"),
        )

        self.assertEqual(events, ["place", "stop-recording", "return"])


if __name__ == "__main__":
    unittest.main()
