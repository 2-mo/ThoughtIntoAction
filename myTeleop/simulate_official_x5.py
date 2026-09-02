#!/usr/bin/env python3
"""Simulate the stock XRoboToolkit ARX control pipeline with an X5 model."""

import time
from copy import deepcopy
from pathlib import Path

import arx5_interface as arx5
import numpy as np
import tyro

from teleop_official_x5 import (
    DEFAULT_X5_URDF_PATH,
    OfficialSingleX5Controller,
    bounded_joint_target,
)
from xrobotoolkit_teleop.hardware.arx_r5_teleop_controller import (
    DEFAULT_ARX_R5_MANIPULATOR_CONFIG,
)


class SimulatedOfficialX5Adapter:
    """Minimal in-memory replacement for the official ARX hardware interface."""

    def __init__(
        self,
        initial_q: np.ndarray,
        gripper_width: float,
        control_rate_hz: int,
        max_joint_speed_rad_s: float,
        joint_soft_limit_margin_rad: float,
        q_min: np.ndarray,
        q_max: np.ndarray,
    ) -> None:
        self._q = np.asarray(initial_q, dtype=float).copy()
        self._dq = np.zeros(6, dtype=float)
        self._gripper = gripper_width
        self._dt = 1.0 / control_rate_hz
        self._max_step_rad = max_joint_speed_rad_s / control_rate_hz
        self._command_gap_reset_s = 3.0 / control_rate_hz
        self._soft_limit_margin_rad = joint_soft_limit_margin_rad
        self._q_min = np.asarray(q_min, dtype=float)
        self._q_max = np.asarray(q_max, dtype=float)
        self._previous_q_desired = self._q.copy()
        self._last_diagnostic_time = 0.0
        self._peak_ik_step = 0.0
        self._last_position_command_time = 0.0

    def get_joint_positions(self) -> np.ndarray:
        return np.concatenate((self._q.copy(), [self._gripper]))

    def get_joint_velocities(self) -> np.ndarray:
        return np.concatenate((self._dq.copy(), [0.0]))

    def set_joint_positions(self, positions: np.ndarray) -> bool:
        target = np.asarray(positions, dtype=float)
        if target.shape != (6,):
            raise ValueError(f"Expected six X5 joints, got {target.shape}")
        now = time.monotonic()
        if now - self._last_position_command_time > self._command_gap_reset_s:
            self._previous_q_desired = target.copy()
            self._peak_ik_step = 0.0
        self._last_position_command_time = now
        desired_step = float(np.max(np.abs(target - self._previous_q_desired)))
        self._peak_ik_step = max(self._peak_ik_step, desired_step)
        self._previous_q_desired = target.copy()
        q_next, q_limited = bounded_joint_target(
            q_actual=self._q,
            q_desired=target,
            q_min=self._q_min,
            q_max=self._q_max,
            max_step_rad=self._max_step_rad,
            soft_limit_margin_rad=self._soft_limit_margin_rad,
        )
        self._dq = (q_next - self._q) / self._dt
        self._q = q_next
        if now - self._last_diagnostic_time >= 0.2:
            print(
                "OFFICIAL SIM "
                f"q_actual={np.round(self._q, 4)} "
                f"q_des={np.round(target, 4)} "
                f"peak_ik_step={self._peak_ik_step:.4f} "
                f"tracking_error={np.max(np.abs(q_limited - self._q)):.4f}"
            )
            self._last_diagnostic_time = now
            self._peak_ik_step = 0.0
        return True

    def set_catch_pos(self, position: float) -> bool:
        self._gripper = float(position)
        return True

    def set_to_damping(self) -> None:
        self._dq.fill(0.0)


class OfficialSingleX5Simulation(OfficialSingleX5Controller):
    """Use the same stock controller as the X5 baseline without opening CAN."""

    def __init__(self, *args, initial_q: np.ndarray, **kwargs) -> None:
        self._simulation_initial_q = np.asarray(initial_q, dtype=float)
        self._simulation_robot_config = (
            arx5.RobotConfigFactory.get_instance().get_config("X5")
        )
        super().__init__(*args, **kwargs)

    def _robot_setup(self) -> None:
        if getattr(self, "_x5_setup_done", False):
            return
        self.arm_controllers = {
            name: SimulatedOfficialX5Adapter(
                initial_q=self._simulation_initial_q,
                gripper_width=self._x5_gripper_width,
                control_rate_hz=self._x5_control_rate_hz,
                max_joint_speed_rad_s=self._x5_max_joint_speed_rad_s,
                joint_soft_limit_margin_rad=self._x5_joint_soft_limit_margin_rad,
                q_min=np.asarray(self._simulation_robot_config.joint_pos_min),
                q_max=np.asarray(self._simulation_robot_config.joint_pos_max),
            )
            for name in self.can_ports
        }
        self._x5_setup_done = True
        print("Official XRoboToolkit pipeline + virtual X5 backend.")

    def _shutdown_robot(self) -> None:
        for controller in self.arm_controllers.values():
            controller.set_to_damping()
        print("Virtual X5 stopped.")


def main(
    robot_urdf_path: str = DEFAULT_X5_URDF_PATH,
    scale_factor: float = 0.1,
    control_mode: str = "pose",
    control_rate_hz: int = 50,
    max_joint_speed_rad_s: float = 0.2,
    joint_soft_limit_margin_rad: float = 0.05,
    gripper_width: float = 0.082,
    initial_q: tuple[float, float, float, float, float, float] = (
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ),
    visualize_placo: bool = True,
) -> None:
    """Run the official ARX teleop logic against an in-memory X5."""
    if not Path(robot_urdf_path).is_file():
        raise FileNotFoundError(robot_urdf_path)
    if control_mode not in {"position", "pose"}:
        raise ValueError("control_mode must be 'position' or 'pose'")
    if control_rate_hz <= 0 or max_joint_speed_rad_s <= 0.0:
        raise ValueError("control rate and joint speed must be positive")
    if joint_soft_limit_margin_rad < 0.0:
        raise ValueError("joint soft-limit margin cannot be negative")

    manipulator_config = deepcopy(DEFAULT_ARX_R5_MANIPULATOR_CONFIG)
    manipulator_config["right_arm"]["control_mode"] = control_mode
    manipulator_config["right_arm"]["gripper_config"]["open_pos"] = [
        gripper_width
    ]
    manipulator_config["right_arm"]["gripper_config"]["close_pos"] = [0.0]

    controller = OfficialSingleX5Simulation(
        robot_urdf_path=robot_urdf_path,
        manipulator_config=manipulator_config,
        can_ports={"right_arm": "simulation"},
        scale_factor=scale_factor,
        control_rate_hz=control_rate_hz,
        visualize_placo=visualize_placo,
        enable_camera=False,
        enable_log_data=False,
        log_dir=str(Path(__file__).resolve().parent / "logs"),
        actuate=False,
        preview_time_s=0.05,
        gripper_open_readout=-3.4,
        gripper_width=gripper_width,
        max_joint_speed_rad_s=max_joint_speed_rad_s,
        joint_soft_limit_margin_rad=joint_soft_limit_margin_rad,
        initial_q=np.asarray(initial_q, dtype=float),
    )
    controller.run()


if __name__ == "__main__":
    tyro.cli(main)
