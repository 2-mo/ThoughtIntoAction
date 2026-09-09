"""CLI entry points for the project X5 controller and A/B presets."""

import argparse
from copy import deepcopy
from pathlib import Path
import sys

import tyro

from robot_core.project import LOG_DIR
from robot_core.project import X5_2025_URDF
from .base_controller import DEFAULT_X5_MANIPULATOR_CONFIG
from .controller import X5TeleopController
from .presets import DEFAULT_HOME_JOINTS, DEFAULT_GRASP_READY_JOINTS, PresetSingleX5Controller


def baseline_main(
    robot_urdf_path: str = str(X5_2025_URDF),
    scale_factor: float = 0.1,
    control_mode: str = "pose",
    control_rate_hz: int = 50,
    max_joint_speed_rad_s: float = 0.2,
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
    if control_rate_hz <= 0 or max_joint_speed_rad_s <= 0.0:
        raise ValueError("control rate and joint speed must be positive")

    manipulator_config = deepcopy(DEFAULT_X5_MANIPULATOR_CONFIG)
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
        control_rate_hz=control_rate_hz,
        enable_camera=False,
        enable_log_data=False,
        log_dir=str(LOG_DIR),
        actuate=actuate, service_socket=service_socket, input_plugin=input_plugin,
        max_joint_speed_rad_s=max_joint_speed_rad_s,
        dynamics_urdf_path=robot_urdf_path,
        enable_session=enable_session,
    )
    controller.run()


def presets_main(
    robot_urdf_path: str = str(X5_2025_URDF),
    scale_factor: float = 0.1,
    control_mode: str = "pose",
    control_rate_hz: int = 50,
    home_joints: tuple[float, float, float, float, float, float] = DEFAULT_HOME_JOINTS,
    grasp_ready_joints: tuple[float, float, float, float, float, float] = DEFAULT_GRASP_READY_JOINTS,
    collection_task: str = "X5 遥操采集",
    home_hold_s: float = 1.0,
    preset_tolerance_rad: float = 0.03,
    max_joint_speed_rad_s: float = 0.2,
    actuate: bool = False,
    service_socket: str | None = None,
    enable_session: bool = False,
    input_plugin: str = "quest",
) -> None:
    """Run X5 collection: A preset then start, B save/hold Home; stick manages task and discard."""
    if not Path(robot_urdf_path).is_file():
        raise FileNotFoundError(robot_urdf_path)
    if control_mode not in {"position", "pose"}:
        raise ValueError("control_mode must be 'position' or 'pose'")
    if control_rate_hz <= 0 or max_joint_speed_rad_s <= 0.0:
        raise ValueError("control rate and joint speed must be positive")

    manipulator_config = deepcopy(DEFAULT_X5_MANIPULATOR_CONFIG)
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
        control_rate_hz=control_rate_hz,
        enable_camera=False,
        enable_log_data=False,
        log_dir=str(LOG_DIR),
        actuate=actuate, service_socket=service_socket, input_plugin=input_plugin,
        max_joint_speed_rad_s=max_joint_speed_rad_s,
        home_joints=home_joints, collection_task=collection_task,
        grasp_ready_joints=grasp_ready_joints,
        home_hold_s=home_hold_s,
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
