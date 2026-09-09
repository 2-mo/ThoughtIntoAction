"""Experimental Quest teleop through the optional ARX X5-2025 backend."""

from copy import deepcopy
from pathlib import Path

import tyro

from robot_core.project import LOG_DIR
from robot_control.hardware import DEFAULT_X5_URDF_PATH
from quest_teleop.base_controller import DEFAULT_X5_MANIPULATOR_CONFIG
from quest_teleop.controller import X5TeleopController


def main(
    robot_urdf_path: str = DEFAULT_X5_URDF_PATH,
    can_port: str = "can0",
    scale_factor: float = 0.1,
    control_mode: str = "pose",
    control_rate_hz: int = 50,
    gripper_open_readout: float = -3.4,
    gripper_width: float = 0.082,
    max_joint_speed_rad_s: float = 0.1,
    joint_soft_limit_margin_rad: float = 0.05,
    actuate: bool = False,
    telemetry: str = "local",
) -> None:
    """Run XRoboToolkit with the official X5-2025 hardware implementation."""
    if actuate:
        raise RuntimeError(
            "ARX X5-2025 POSITION_CONTROL(5) is disabled for XR: isolated "
            "tests showed uncommanded multi-joint drift. Use this entry in "
            "read-only mode only."
        )
    if not Path(robot_urdf_path).is_file():
        raise FileNotFoundError(robot_urdf_path)
    if control_mode not in {"position", "pose"}:
        raise ValueError("control_mode must be 'position' or 'pose'")

    manipulator_config = deepcopy(DEFAULT_X5_MANIPULATOR_CONFIG)
    manipulator_config["right_arm"]["control_mode"] = control_mode
    manipulator_config["right_arm"]["gripper_config"]["open_pos"] = [
        gripper_width
    ]
    manipulator_config["right_arm"]["gripper_config"]["close_pos"] = [0.0]

    controller = X5TeleopController(
        backend="arx2025",
        robot_urdf_path=robot_urdf_path,
        manipulator_config=manipulator_config,
        can_ports={"right_arm": can_port},
        scale_factor=scale_factor,
        control_rate_hz=control_rate_hz,
        telemetry=telemetry,
        enable_camera=False,
        enable_log_data=False,
        log_dir=str(LOG_DIR),
        actuate=actuate,
        preview_time_s=0.0,
        gripper_open_readout=gripper_open_readout,
        gripper_width=gripper_width,
        max_joint_speed_rad_s=max_joint_speed_rad_s,
        joint_soft_limit_margin_rad=joint_soft_limit_margin_rad,
    )
    controller.run()


if __name__ == "__main__":
    tyro.cli(main)
