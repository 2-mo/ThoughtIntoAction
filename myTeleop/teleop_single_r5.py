#!/usr/bin/env python3
"""Single-arm Quest teleoperation using the maintained ARX5 SDK."""

from copy import deepcopy
from pathlib import Path
from typing import Optional

import arx5_interface as arx5
import numpy as np
import tyro

from xrobotoolkit_teleop.hardware.arx_r5_teleop_controller import (
    ARXR5TeleopController,
    DEFAULT_ARX_R5_MANIPULATOR_CONFIG,
    DEFAULT_ARX_R5_URDF_PATH,
)


class ARX5Interface:
    """Small adapter from XRoboToolkit's interface to arx5-interface."""

    def __init__(
        self,
        can_port: str,
        model: str,
        gripper_open_readout: float,
        gripper_width: float,
    ) -> None:
        self.robot_config = arx5.RobotConfigFactory.get_instance().get_config(model)
        self.robot_config.gripper_open_readout = gripper_open_readout
        self.robot_config.gripper_width = gripper_width
        self.controller_config = arx5.ControllerConfigFactory.get_instance().get_config(
            "joint_controller", self.robot_config.joint_dof
        )
        self.controller_config.background_send_recv = True
        self.arm = arx5.Arx5JointController(
            self.robot_config, self.controller_config, can_port
        )
        self.arm.set_log_level(arx5.LogLevel.INFO)

        # Hold the pose measured during initialization; do not move home.
        gain = arx5.Gain(
            self.controller_config.default_kp,
            self.controller_config.default_kd,
            self.controller_config.default_gripper_kp,
            self.controller_config.default_gripper_kd,
        )
        self.arm.set_gain(gain)

    def get_joint_positions(self) -> np.ndarray:
        state = self.arm.get_joint_state()
        return np.concatenate((state.pos().copy(), [state.gripper_pos]))

    def get_joint_velocities(self) -> np.ndarray:
        state = self.arm.get_joint_state()
        return np.concatenate((state.vel().copy(), [state.gripper_vel]))

    def set_command(
        self,
        positions: Optional[np.ndarray] = None,
        gripper_position: Optional[float] = None,
    ) -> None:
        command = self.arm.get_joint_cmd()
        if positions is not None:
            target = np.asarray(positions, dtype=float)
            if target.shape != (self.robot_config.joint_dof,) or not np.all(
                np.isfinite(target)
            ):
                raise ValueError(f"Invalid ARX joint target: {target}")
            command.pos()[:] = target
        if gripper_position is not None:
            command.gripper_pos = float(
                np.clip(gripper_position, 0.0, self.robot_config.gripper_width)
            )

        # get_joint_cmd() carries the timestamp of the previously interpolated
        # command. Reset it so the SDK schedules this command with its default
        # preview time instead of rejecting an already-expired timestamp.
        command.timestamp = 0.0
        self.arm.set_joint_cmd(command)

    def set_joint_positions(self, positions: np.ndarray) -> None:
        self.set_command(positions=positions)

    def set_catch_pos(self, position: float) -> None:
        self.set_command(gripper_position=position)

    def set_to_damping(self) -> None:
        self.arm.set_to_damping()


class SingleArmTeleopController(ARXR5TeleopController):
    """Use XRoboToolkit IK with one maintained-SDK ARX5 backend."""

    def __init__(
        self,
        *args,
        arm_model: str,
        gripper_open_readout: float,
        gripper_width: float,
        **kwargs,
    ) -> None:
        self.arm_model = arm_model
        self.gripper_open_readout = gripper_open_readout
        self.gripper_width = gripper_width
        super().__init__(*args, **kwargs)

    def _robot_setup(self) -> None:
        if getattr(self, "_single_arm_hardware_initialized", False):
            return
        self.arm_controllers = {
            arm_name: ARX5Interface(
                can_port=can_port,
                model=self.arm_model,
                gripper_open_readout=self.gripper_open_readout,
                gripper_width=self.gripper_width,
            )
            for arm_name, can_port in self.can_ports.items()
        }
        self._single_arm_hardware_initialized = True
        print("ARX5 initialized at its current pose (no automatic homing).")

    def _shutdown_robot(self) -> None:
        for controller in self.arm_controllers.values():
            controller.set_to_damping()
        print("ARX5 switched to damping mode.")

    def _send_command(self) -> None:
        """Send one timestamp-consistent arm and gripper command per cycle."""
        for arm_name, controller in self.arm_controllers.items():
            positions = None
            if self.active.get(arm_name, False):
                positions = self.placo_robot.state.q[
                    self.placo_arm_joint_slice[arm_name]
                ].copy()

            gripper_position = None
            if "gripper_config" in self.manipulator_config[arm_name]:
                gripper_config = self.manipulator_config[arm_name]["gripper_config"]
                joint_name = gripper_config["joint_names"][0]
                gripper_position = self.gripper_pos_target[arm_name][joint_name]

            controller.set_command(
                positions=positions,
                gripper_position=gripper_position,
            )


def main(
    robot_urdf_path: str = DEFAULT_ARX_R5_URDF_PATH,
    can_port: str = "can0",
    arm_model: str = "X5",
    gripper_open_readout: float = -3.4,
    gripper_width: float = 0.082,
    scale_factor: float = 1.0,
    control_rate_hz: int = 50,
    visualize_placo: bool = False,
    enable_camera: bool = False,
    enable_log_data: bool = False,
    log_dir: str = str(Path(__file__).resolve().parent / "logs"),
) -> None:
    manipulator_config = deepcopy(DEFAULT_ARX_R5_MANIPULATOR_CONFIG)
    manipulator_config["right_arm"]["gripper_config"]["open_pos"] = [gripper_width]
    manipulator_config["right_arm"]["gripper_config"]["close_pos"] = [0.0]

    controller = SingleArmTeleopController(
        robot_urdf_path=robot_urdf_path,
        manipulator_config=manipulator_config,
        can_ports={"right_arm": can_port},
        arm_model=arm_model,
        gripper_open_readout=gripper_open_readout,
        gripper_width=gripper_width,
        scale_factor=scale_factor,
        control_rate_hz=control_rate_hz,
        visualize_placo=visualize_placo,
        enable_camera=enable_camera,
        enable_log_data=enable_log_data,
        log_dir=log_dir,
    )
    controller.run()


if __name__ == "__main__":
    tyro.cli(main)
