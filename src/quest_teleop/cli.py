"""CLI entry points for the project X5 controller and A/B presets."""

import argparse
from copy import deepcopy
from pathlib import Path
import sys
from typing import Literal

import tyro

from robot_core.project import LOG_DIR
from robot_core.project import X5_2025_URDF
from .base_controller import DEFAULT_X5_MANIPULATOR_CONFIG
from .controller import X5TeleopController
from .presets import DEFAULT_HOME_JOINTS, DEFAULT_GRASP_READY_JOINTS, PresetSingleX5Controller


def baseline_main(
    robot_urdf_path: str = str(X5_2025_URDF),
    scale_factor: float = 0.1,
    operator_yaw_deg: float = 0.0,
    rotation_frame: Literal["base", "tool", "aligned", "calibrated"] = "base",
    max_eef_linear_speed_m_s: float = 0.25,
    max_eef_angular_speed_rad_s: float = 0.0,
    control_mode: str = "pose",
    control_link: Literal["link6", "eef_link"] = "eef_link",
    control_rate_hz: int = 50,
    max_joint_speed_rad_s: float = 0.2,
    max_joint_acceleration_rad_s2: float = 2.,
    max_joint_jerk_rad_s3: float = 10.,
    max_command_lead_rad: float = 0.15,
    preview_time_s: float = 0.01,
    teleop_preview_time_s: float = .01,
    joint_soft_limit_margin_rad: float = 0.05,
    actuate: bool = False,
    service_socket: str | None = None,
    enable_session: bool = False,
    input_plugin: str = "quest",
) -> None:
    """Run the project-owned Quest control path on one X5."""
    if not Path(robot_urdf_path).is_file():
        raise FileNotFoundError(robot_urdf_path)
    if control_mode not in {"position", "pose"}:
        raise ValueError("control_mode must be 'position' or 'pose'")
    if control_rate_hz <= 0 or max_joint_speed_rad_s < 0.0:
        raise ValueError("control rate and joint speed must be positive")

    manipulator_config = deepcopy(DEFAULT_X5_MANIPULATOR_CONFIG)
    if control_link not in {"link6", "eef_link"}:
        raise ValueError("control_link must be link6 or eef_link")
    manipulator_config["right_arm"]["link_name"] = control_link
    manipulator_config["right_arm"]["control_mode"] = control_mode
    manipulator_config["right_arm"]["gripper_config"]["open_pos"] = [
        0.082
    ]
    manipulator_config["right_arm"]["gripper_config"]["close_pos"] = [0.0]

    controller = X5TeleopController(
        robot_urdf_path=robot_urdf_path,
        manipulator_config=manipulator_config,
        can_ports={"right_arm": "service"},
        scale_factor=scale_factor,
        operator_yaw_deg=operator_yaw_deg,
        rotation_frame=rotation_frame,
        max_eef_linear_speed_m_s=max_eef_linear_speed_m_s,
        max_eef_angular_speed_rad_s=max_eef_angular_speed_rad_s,
        control_rate_hz=control_rate_hz,
        enable_camera=False,
        enable_log_data=False,
        log_dir=str(LOG_DIR),
        actuate=actuate, service_socket=service_socket, input_plugin=input_plugin,
        max_joint_speed_rad_s=max_joint_speed_rad_s,
        max_joint_acceleration_rad_s2=max_joint_acceleration_rad_s2, max_joint_jerk_rad_s3=max_joint_jerk_rad_s3,
        max_command_lead_rad=max_command_lead_rad, preview_time_s=preview_time_s,
        teleop_preview_time_s=teleop_preview_time_s,
        joint_soft_limit_margin_rad=joint_soft_limit_margin_rad,
        dynamics_urdf_path=robot_urdf_path,
        enable_session=enable_session,
    )
    controller.run()


def presets_main(
    robot_urdf_path: str = str(X5_2025_URDF),
    scale_factor: float = 0.1,
    operator_yaw_deg: float = 0.0,
    rotation_frame: Literal["base", "tool", "aligned", "calibrated"] = "base",
    max_eef_linear_speed_m_s: float = 0.25,
    max_eef_angular_speed_rad_s: float = 0.0,
    control_mode: str = "pose",
    control_link: Literal["link6", "eef_link"] = "eef_link",
    control_rate_hz: int = 50,
    home_joints: tuple[float, float, float, float, float, float] = DEFAULT_HOME_JOINTS,
    grasp_ready_joints: tuple[float, float, float, float, float, float] = DEFAULT_GRASP_READY_JOINTS,
    collection_task: str = "X5 遥操采集",
    home_hold_s: float = 1.0,
    home_jerk_rad_s3: float = 2.,
    preset_jerk_rad_s3: float = 4.,
    home_speed_rad_s: float = .3,
    home_acceleration_rad_s2: float = .5,
    preset_speed_rad_s: float = .5,
    preset_acceleration_rad_s2: float = 1.,
    preset_tolerance_rad: float = 0.03,
    max_joint_speed_rad_s: float = 0.2,
    max_joint_acceleration_rad_s2: float = 2.,
    max_joint_jerk_rad_s3: float = 10.,
    max_command_lead_rad: float = 0.15,
    preview_time_s: float = 0.01,
    teleop_preview_time_s: float = .01,
    joint_soft_limit_margin_rad: float = 0.05,
    actuate: bool = False,
    service_socket: str | None = None,
    enable_session: bool = False,
    input_plugin: str = "quest",
) -> None:
    """Run X5 collection: A start/save, B preset/hold Home; stick manages task and discard."""
    if not Path(robot_urdf_path).is_file():
        raise FileNotFoundError(robot_urdf_path)
    if control_mode not in {"position", "pose"}:
        raise ValueError("control_mode must be 'position' or 'pose'")
    if control_rate_hz <= 0 or max_joint_speed_rad_s < 0.0:
        raise ValueError("control rate and joint speed must be positive")

    manipulator_config = deepcopy(DEFAULT_X5_MANIPULATOR_CONFIG)
    if control_link not in {"link6", "eef_link"}:
        raise ValueError("control_link must be link6 or eef_link")
    manipulator_config["right_arm"]["link_name"] = control_link
    manipulator_config["right_arm"]["control_mode"] = control_mode
    manipulator_config["right_arm"]["gripper_config"]["open_pos"] = [
        0.082
    ]
    manipulator_config["right_arm"]["gripper_config"]["close_pos"] = [0.0]

    controller = PresetSingleX5Controller(
        robot_urdf_path=robot_urdf_path,
        manipulator_config=manipulator_config,
        can_ports={"right_arm": "service"},
        scale_factor=scale_factor,
        operator_yaw_deg=operator_yaw_deg,
        rotation_frame=rotation_frame,
        max_eef_linear_speed_m_s=max_eef_linear_speed_m_s,
        max_eef_angular_speed_rad_s=max_eef_angular_speed_rad_s,
        control_rate_hz=control_rate_hz,
        enable_camera=False,
        enable_log_data=False,
        log_dir=str(LOG_DIR),
        actuate=actuate, service_socket=service_socket, input_plugin=input_plugin,
        max_joint_speed_rad_s=max_joint_speed_rad_s,
        max_joint_acceleration_rad_s2=max_joint_acceleration_rad_s2, max_joint_jerk_rad_s3=max_joint_jerk_rad_s3,
        max_command_lead_rad=max_command_lead_rad, preview_time_s=preview_time_s,
        teleop_preview_time_s=teleop_preview_time_s,
        joint_soft_limit_margin_rad=joint_soft_limit_margin_rad,
        home_joints=home_joints, collection_task=collection_task,
        grasp_ready_joints=grasp_ready_joints,
        home_hold_s=home_hold_s,
        home_jerk_rad_s3=home_jerk_rad_s3, preset_jerk_rad_s3=preset_jerk_rad_s3,
        home_speed_rad_s=home_speed_rad_s, home_acceleration_rad_s2=home_acceleration_rad_s2,
        preset_speed_rad_s=preset_speed_rad_s, preset_acceleration_rad_s2=preset_acceleration_rad_s2,
        preset_tolerance_rad=preset_tolerance_rad,
        dynamics_urdf_path=robot_urdf_path,
        enable_session=enable_session,
    )
    controller.run()


def main(argv=None):
    arguments = list(sys.argv[1:] if argv is None else argv)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("baseline", "presets"))
    if not arguments or arguments[0] in {"-h", "--help"}:
        parser.print_help()
        return
    mode = parser.parse_args(arguments[:1]).mode
    entry = {"baseline": baseline_main, "presets": presets_main}[mode]
    tyro.cli(entry, args=arguments[1:])


if __name__ == "__main__":
    main()
