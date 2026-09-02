#!/usr/bin/env python3
"""Quest button presets layered on the validated official X5 teleop backend."""

import threading
import time
from copy import deepcopy
from pathlib import Path

import numpy as np
import tyro

from teleop_official_x5 import DEFAULT_X5_URDF_PATH, OfficialSingleX5Controller
from xrobotoolkit_teleop.hardware.arx_r5_teleop_controller import (
    DEFAULT_ARX_R5_MANIPULATOR_CONFIG,
)


DEFAULT_HOME_JOINTS = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
# With the Stanford X5 kinematic chain used by X5_2025_hybrid.urdf, this
# configuration points the eef/gripper x-axis approximately straight down
# while keeping the arm forward of the base. Treat it as a starting preset and
# override it after validating the exact hardware setup if needed.
DEFAULT_GRASP_READY_JOINTS = (0.0, 1.50, 1.45, -1.52, 0.0, 0.0)


class PresetSingleX5Controller(OfficialSingleX5Controller):
    """Add A=grasp-ready and long-B=home without changing normal teleop."""

    def __init__(
        self,
        *args,
        home_joints: tuple[float, float, float, float, float, float],
        grasp_ready_joints: tuple[float, float, float, float, float, float],
        home_hold_s: float,
        preset_tolerance_rad: float,
        **kwargs,
    ) -> None:
        if home_hold_s <= 0.0:
            raise ValueError("home_hold_s must be positive")
        if preset_tolerance_rad <= 0.0:
            raise ValueError("preset_tolerance_rad must be positive")

        self._home_joints = np.asarray(home_joints, dtype=float)
        self._grasp_ready_joints = np.asarray(grasp_ready_joints, dtype=float)
        self._home_hold_s = float(home_hold_s)
        self._preset_tolerance_rad = float(preset_tolerance_rad)

        self._preset_lock = threading.Lock()
        self._preset_target: np.ndarray | None = None
        self._preset_name: str | None = None
        self._prev_a_button = False
        self._b_pressed_since: float | None = None
        self._b_fired = False

        super().__init__(*args, **kwargs)
        self._validate_preset("HOME", self._home_joints)
        self._validate_preset("GRASP_READY", self._grasp_ready_joints)

        print("Quest preset controls:")
        print("  A: move to GRASP_READY (Grip must be released)")
        print(f"  B: hold for {self._home_hold_s:.1f}s to move HOME (Grip must be released)")
        print("  Pressing Grip during a preset cancels the preset immediately.")

    def _validate_preset(self, name: str, target: np.ndarray) -> None:
        if target.shape != (6,) or not np.all(np.isfinite(target)):
            raise ValueError(f"{name} preset must contain six finite joint values")
        controller = next(iter(self.arm_controllers.values()))
        q_min = np.asarray(controller.robot_config.joint_pos_min, dtype=float)
        q_max = np.asarray(controller.robot_config.joint_pos_max, dtype=float)
        if np.any(target < q_min) or np.any(target > q_max):
            raise ValueError(
                f"{name} preset is outside X5 joint limits: target={target}, "
                f"min={q_min}, max={q_max}"
            )

    def _reset_teleop_reference(self) -> None:
        """Force the next Grip activation to establish a fresh XR/robot anchor."""
        for arm_name in self.manipulator_config:
            self.active[arm_name] = False
            self.ref_ee_xyz[arm_name] = None
            self.ref_ee_quat[arm_name] = None
            self.ref_controller_xyz[arm_name] = None
            self.ref_controller_quat[arm_name] = None

    def _start_preset(self, name: str, target: np.ndarray) -> None:
        self._reset_teleop_reference()
        with self._preset_lock:
            self._preset_name = name
            self._preset_target = target.copy()
        print(f"PRESET {name} started: q={np.round(target, 4)}")

    def _cancel_preset(self, reason: str) -> None:
        with self._preset_lock:
            if self._preset_target is None:
                return
            name = self._preset_name
            self._preset_target = None
            self._preset_name = None
        self._reset_teleop_reference()
        print(f"PRESET {name} cancelled: {reason}")

    def _finish_preset(self, expected_name: str) -> None:
        with self._preset_lock:
            if self._preset_name != expected_name:
                return
            self._preset_target = None
            self._preset_name = None
        self._reset_teleop_reference()
        print(f"PRESET {expected_name} reached")

    def _pre_ik_update(self) -> None:
        """Read Quest A/B and update the preset state machine."""
        a_pressed = bool(self.xr_client.get_button_state_by_name("A"))
        b_pressed = bool(self.xr_client.get_button_state_by_name("B"))
        grip = float(
            self.xr_client.get_key_value_by_name(
                self.manipulator_config["right_arm"]["control_trigger"]
            )
        )
        grip_active = grip > 0.9
        now = time.monotonic()

        if grip_active:
            self._cancel_preset("Grip activated")
            self._b_pressed_since = None
            self._b_fired = False
        else:
            if a_pressed and not self._prev_a_button:
                self._start_preset("GRASP_READY", self._grasp_ready_joints)

            if b_pressed:
                if self._b_pressed_since is None:
                    self._b_pressed_since = now
                elif (
                    not self._b_fired
                    and now - self._b_pressed_since >= self._home_hold_s
                ):
                    self._start_preset("HOME", self._home_joints)
                    self._b_fired = True
            else:
                self._b_pressed_since = None
                self._b_fired = False

        self._prev_a_button = a_pressed

    def _send_command(self) -> None:
        """Preset targets take priority; otherwise preserve stock teleoperation."""
        with self._preset_lock:
            preset_target = (
                None if self._preset_target is None else self._preset_target.copy()
            )
            preset_name = self._preset_name

        for arm_name, controller in self.arm_controllers.items():
            if preset_target is not None:
                controller.set_joint_positions(preset_target)
                q_actual = controller.get_joint_positions()[:6]
                error = float(np.max(np.abs(q_actual - preset_target)))
                if error <= self._preset_tolerance_rad and preset_name is not None:
                    self._finish_preset(preset_name)
            elif self.active.get(arm_name, False):
                q_des = self.placo_robot.state.q[
                    self.placo_arm_joint_slice[arm_name]
                ].copy()
                controller.set_joint_positions(q_des)

            if "gripper_config" in self.manipulator_config[arm_name]:
                gripper_config = self.manipulator_config[arm_name]["gripper_config"]
                joint_name = gripper_config["joint_names"][0]
                gripper_target = self.gripper_pos_target[arm_name][joint_name]
                controller.set_catch_pos(gripper_target)


def main(
    robot_urdf_path: str = DEFAULT_X5_URDF_PATH,
    can_port: str = "can0",
    scale_factor: float = 0.1,
    control_mode: str = "pose",
    control_rate_hz: int = 50,
    preview_time_s: float = 0.05,
    gripper_open_readout: float = -3.4,
    gripper_width: float = 0.082,
    gripper_force_hold: bool = True,
    gripper_contact_torque: float = 0.60,
    gripper_hold_torque: float = 0.30,
    gripper_contact_cycles: int = 1,
    gripper_release_margin_m: float = 0.004,
    home_joints: tuple[float, float, float, float, float, float] = DEFAULT_HOME_JOINTS,
    grasp_ready_joints: tuple[float, float, float, float, float, float] = DEFAULT_GRASP_READY_JOINTS,
    home_hold_s: float = 1.0,
    preset_tolerance_rad: float = 0.03,
    max_joint_speed_rad_s: float = 0.2,
    joint_soft_limit_margin_rad: float = 0.05,
    actuate: bool = False,
    visualize_placo: bool = True,
) -> None:
    """Run X5 teleoperation with Quest A/B preset-pose controls."""
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

    controller = PresetSingleX5Controller(
        robot_urdf_path=robot_urdf_path,
        manipulator_config=manipulator_config,
        can_ports={"right_arm": can_port},
        scale_factor=scale_factor,
        control_rate_hz=control_rate_hz,
        visualize_placo=visualize_placo,
        enable_camera=False,
        enable_log_data=False,
        log_dir=str(Path(__file__).resolve().parent / "logs"),
        actuate=actuate,
        preview_time_s=preview_time_s,
        gripper_open_readout=gripper_open_readout,
        gripper_width=gripper_width,
        max_joint_speed_rad_s=max_joint_speed_rad_s,
        joint_soft_limit_margin_rad=joint_soft_limit_margin_rad,
        gripper_force_hold=gripper_force_hold,
        gripper_contact_torque=gripper_contact_torque,
        gripper_hold_torque=gripper_hold_torque,
        gripper_contact_cycles=gripper_contact_cycles,
        gripper_release_margin_m=gripper_release_margin_m,
        home_joints=home_joints,
        grasp_ready_joints=grasp_ready_joints,
        home_hold_s=home_hold_s,
        preset_tolerance_rad=preset_tolerance_rad,
        dynamics_urdf_path=robot_urdf_path,
    )
    controller.run()


if __name__ == "__main__":
    tyro.cli(main)
