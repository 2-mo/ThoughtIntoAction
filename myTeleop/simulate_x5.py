#!/usr/bin/env python3
"""Quest-driven X5 simulation using the same pipeline as real hardware."""

from copy import deepcopy
from pathlib import Path

import arx5_interface as arx5
import numpy as np
import tyro

from teleop_single_x5 import (
    DEFAULT_X5_MANIPULATOR_CONFIG,
    DEFAULT_X5_URDF_PATH,
    SingleX5TeleopController,
)


class SimulatedX5Interface:
    """In-memory joint backend with the same interface as the physical X5."""

    def __init__(
        self,
        initial_q: np.ndarray,
        gripper_width: float,
        dt: float,
        response_rate_rad_s: float,
    ) -> None:
        self.robot_config = arx5.RobotConfigFactory.get_instance().get_config("X5")
        self.robot_config.gripper_width = gripper_width
        self._q = np.asarray(initial_q, dtype=float).copy()
        self._dq = np.zeros_like(self._q)
        self._gripper = gripper_width
        self._gripper_velocity = 0.0
        self._dt = dt
        self._response_rate_rad_s = response_rate_rad_s

    def get_joint_positions(self) -> np.ndarray:
        return np.concatenate((self._q.copy(), [self._gripper]))

    def get_joint_velocities(self) -> np.ndarray:
        return np.concatenate((self._dq.copy(), [self._gripper_velocity]))

    def get_position_target(self) -> np.ndarray:
        return self._q.copy()

    def set_command(
        self,
        positions: np.ndarray | None = None,
        gripper_position: float | None = None,
    ) -> None:
        if positions is not None:
            target = np.asarray(positions, dtype=float)
            step = np.clip(
                target - self._q,
                -self._response_rate_rad_s * self._dt,
                self._response_rate_rad_s * self._dt,
            )
            self._dq = step / self._dt
            self._q += step
        else:
            self._dq.fill(0.0)

        if gripper_position is not None:
            target_gripper = float(
                np.clip(gripper_position, 0.0, self.robot_config.gripper_width)
            )
            self._gripper_velocity = (target_gripper - self._gripper) / self._dt
            self._gripper = target_gripper
        else:
            self._gripper_velocity = 0.0

    def set_to_damping(self) -> None:
        self._dq.fill(0.0)
        self._gripper_velocity = 0.0


class X5SimulationController(SingleX5TeleopController):
    """Replace only the physical CAN backend; keep the control path identical."""

    def __init__(self, *args, initial_q: np.ndarray, **kwargs) -> None:
        self.initial_q = np.asarray(initial_q, dtype=float)
        self.simulation_response_rate_rad_s = float(
            kwargs.pop("simulation_response_rate_rad_s")
        )
        self.simulation_dt = 1.0 / float(kwargs["control_rate_hz"])
        super().__init__(*args, **kwargs)

    def _robot_setup(self) -> None:
        if getattr(self, "_single_arm_hardware_initialized", False):
            return
        self.arm_controllers = {
            arm_name: SimulatedX5Interface(
                initial_q=self.initial_q,
                gripper_width=self.gripper_width,
                dt=self.simulation_dt,
                response_rate_rad_s=self.simulation_response_rate_rad_s,
            )
            for arm_name in self.can_ports
        }
        self._single_arm_hardware_initialized = True

        for arm_name, controller in self.arm_controllers.items():
            config = self.manipulator_config[arm_name]
            if "gripper_config" in config:
                joint_name = config["gripper_config"]["joint_names"][0]
                self.gripper_pos_target[arm_name][joint_name] = float(
                    controller.get_joint_positions()[-1]
                )
        print("Virtual ARX X5 initialized; CAN and physical motors are not used.")

    def _shutdown_robot(self) -> None:
        for controller in self.arm_controllers.values():
            controller.set_to_damping()
        print("Virtual ARX X5 stopped.")


def main(
    robot_urdf_path: str = DEFAULT_X5_URDF_PATH,
    scale_factor: float = 0.1,
    rotation_scale_factor: float = 0.5,
    control_mode: str = "pose",
    control_rate_hz: int = 50,
    preview_time_s: float = 0.05,
    max_joint_speed_rad_s: float = 0.5,
    max_ik_step_rad: float = 0.35,
    joint_soft_limit_margin_rad: float = 0.1,
    simulation_response_rate_rad_s: float = 0.5,
    initial_q: tuple[float, float, float, float, float, float] = (
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ),
    xr_stale_timeout_s: float = 0.25,
    xr_settle_samples: int = 3,
    max_xr_step_m: float = 0.08,
    max_xr_step_rad: float = 1.0,
    gripper_width: float = 0.082,
    telemetry_log: bool = True,
    log_dir: str = str(Path(__file__).resolve().parent / "logs"),
) -> None:
    """Run the hardware-equivalent control pipeline on an in-memory X5."""
    if control_mode not in {"position", "pose"}:
        raise ValueError("control_mode must be 'position' or 'pose'")
    if control_rate_hz <= 0:
        raise ValueError("control_rate_hz must be positive")
    if simulation_response_rate_rad_s <= 0.0:
        raise ValueError("simulation response rate must be positive")

    manipulator_config = deepcopy(DEFAULT_X5_MANIPULATOR_CONFIG)
    manipulator_config["right_arm"]["control_mode"] = control_mode
    manipulator_config["right_arm"]["gripper_config"]["open_pos"] = [
        gripper_width
    ]
    manipulator_config["right_arm"]["gripper_config"]["close_pos"] = [0.0]

    controller = X5SimulationController(
        robot_urdf_path=robot_urdf_path,
        manipulator_config=manipulator_config,
        can_ports={"right_arm": "simulation"},
        arm_model="X5",
        gripper_open_readout=-3.4,
        gripper_width=gripper_width,
        preview_time_s=preview_time_s,
        max_joint_speed_rad_s=max_joint_speed_rad_s,
        max_ik_step_rad=max_ik_step_rad,
        joint_soft_limit_margin_rad=joint_soft_limit_margin_rad,
        xr_stale_timeout_s=xr_stale_timeout_s,
        xr_settle_samples=xr_settle_samples,
        max_xr_step_m=max_xr_step_m,
        max_xr_step_rad=max_xr_step_rad,
        rotation_scale_factor=rotation_scale_factor,
        dry_run=False,
        telemetry_log=telemetry_log,
        telemetry_source="simulation",
        telemetry_log_dir=log_dir,
        initial_q=np.asarray(initial_q, dtype=float),
        simulation_response_rate_rad_s=simulation_response_rate_rad_s,
        scale_factor=scale_factor,
        control_rate_hz=control_rate_hz,
        visualize_placo=True,
        enable_camera=False,
        enable_log_data=False,
        log_dir=log_dir,
    )
    controller.run()


if __name__ == "__main__":
    tyro.cli(main)
