#!/usr/bin/env python3
"""XRoboToolkit's stock ARX teleop pipeline with a minimal X5 SDK adapter."""

import time
from copy import deepcopy
from pathlib import Path

import arx5_interface as arx5
import numpy as np
import tyro

from xrobotoolkit_teleop.hardware.arx_r5_teleop_controller import (
    ARXR5TeleopController,
    DEFAULT_ARX_R5_MANIPULATOR_CONFIG,
)


DEFAULT_X5_URDF_PATH = str(
    Path(__file__).resolve().parent.parent
    / "arx5-sdk"
    / "models"
    / "X5_2025_hybrid.urdf"
)


def bounded_joint_target(
    q_actual: np.ndarray,
    q_desired: np.ndarray,
    q_min: np.ndarray,
    q_max: np.ndarray,
    max_step_rad: float,
    soft_limit_margin_rad: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Clip an absolute target to soft limits, then rate-limit from feedback."""
    q_actual = np.asarray(q_actual, dtype=float)
    q_desired = np.asarray(q_desired, dtype=float)
    q_min = np.asarray(q_min, dtype=float)
    q_max = np.asarray(q_max, dtype=float)
    if not np.all(np.isfinite(q_desired)):
        raise ValueError("Non-finite X5 joint target")

    midpoint = (q_min + q_max) / 2.0
    soft_min = np.minimum(q_min + soft_limit_margin_rad, midpoint)
    soft_max = np.maximum(q_max - soft_limit_margin_rad, midpoint)
    q_limited = np.clip(q_desired, soft_min, soft_max)
    q_command = q_actual + np.clip(
        q_limited - q_actual, -max_step_rad, max_step_rad
    )
    # If measured feedback is already just outside a hard limit (the observed
    # X5 J4 can report 1.56 while its configured maximum is 1.55), the hard
    # clamp takes precedence and always moves the target inward.
    return np.clip(q_command, q_min, q_max), q_limited


class OfficialX5HardwareAdapter:
    """Expose the stock ARXR5Interface API through the X5-compatible SDK."""

    def __init__(
        self,
        can_port: str,
        actuate: bool,
        preview_time_s: float,
        gripper_open_readout: float,
        gripper_width: float,
        control_rate_hz: int,
        max_joint_speed_rad_s: float,
        joint_soft_limit_margin_rad: float,
        gripper_force_hold: bool,
        gripper_contact_torque: float,
        gripper_hold_torque: float,
        gripper_contact_cycles: int,
        gripper_release_margin_m: float,
        dynamics_urdf_path: str = DEFAULT_X5_URDF_PATH,
    ) -> None:
        self.actuate = actuate
        self.robot_config = arx5.RobotConfigFactory.get_instance().get_config("X5")
        if not Path(dynamics_urdf_path).is_file():
            raise FileNotFoundError(dynamics_urdf_path)
        # Keep Stanford's transparent joint controller and hardware config,
        # but compute inverse-dynamics gravity from ARX's official X5-2025
        # inertial model.
        self.robot_config.urdf_path = dynamics_urdf_path
        self.robot_config.gripper_open_readout = gripper_open_readout
        self.robot_config.gripper_width = gripper_width
        if abs(gripper_open_readout) < 1e-6:
            raise ValueError("gripper_open_readout must be non-zero")
        if gripper_contact_cycles < 1:
            raise ValueError("gripper_contact_cycles must be at least 1")
        if gripper_release_margin_m < 0.0:
            raise ValueError("gripper_release_margin_m cannot be negative")
        if not 0.0 < gripper_hold_torque < gripper_contact_torque:
            raise ValueError(
                "gripper_hold_torque must be positive and below gripper_contact_torque"
            )
        if gripper_contact_torque >= self.robot_config.gripper_torque_max:
            raise ValueError(
                "gripper_contact_torque must stay below the SDK gripper torque limit "
                f"({self.robot_config.gripper_torque_max:.3f})"
            )

        self._max_step_rad = max_joint_speed_rad_s / control_rate_hz
        self._max_command_lead_rad = 0.05
        self._command_gap_reset_s = 3.0 / control_rate_hz
        self._soft_limit_margin_rad = joint_soft_limit_margin_rad

        self._gripper_force_hold_enabled = gripper_force_hold
        self._gripper_contact_torque = gripper_contact_torque
        self._gripper_hold_torque = gripper_hold_torque
        self._gripper_contact_cycles = gripper_contact_cycles
        self._gripper_release_margin_m = gripper_release_margin_m
        self._gripper_closing_torque_sign = -float(np.sign(gripper_open_readout))
        self._gripper_mode = "position"
        self._gripper_contact_count = 0
        self._gripper_latched_position = 0.0

        config = arx5.ControllerConfigFactory.get_instance().get_config(
            "joint_controller", self.robot_config.joint_dof
        )
        config.background_send_recv = True
        config.default_preview_time = preview_time_s
        self._position_gripper_kp = float(config.default_gripper_kp)
        self._position_gripper_kd = float(config.default_gripper_kd)

        self.arm = arx5.Arx5JointController(self.robot_config, config, can_port)
        self.arm.set_log_level(arx5.LogLevel.INFO)

        state = self.arm.get_joint_state()
        self._q_target = state.pos().copy()
        self._previous_q_desired = self._q_target.copy()
        self._gripper_target = float(
            np.clip(state.gripper_pos, 0.0, self.robot_config.gripper_width)
        )
        self._gripper_latched_position = self._gripper_target

        if actuate:
            # Arx5JointController starts in damping mode (kp == 0). In that
            # mode its internal loop deliberately replaces every position
            # command with measured feedback, so position targets cannot move
            # the arm until the normal joint-controller gains are enabled.
            gain = arx5.Gain(
                config.default_kp,
                config.default_kd,
                config.default_gripper_kp,
                config.default_gripper_kd,
            )
            self.arm.set_gain(gain)
            # Let the 500 Hz SDK loop publish feedback after the gain switch,
            # then use that stabilized pose as the command trajectory origin.
            time.sleep(0.1)
            state = self.arm.get_joint_state()
            self._q_target = state.pos().copy()
            self._previous_q_desired = self._q_target.copy()
            self._gripper_target = float(
                np.clip(state.gripper_pos, 0.0, self.robot_config.gripper_width)
            )
            self._gripper_latched_position = self._gripper_target
        else:
            self.arm.set_to_damping()

        self._last_diagnostic_time = 0.0
        self._peak_ik_step = 0.0
        self._last_position_command_time = 0.0

    def get_joint_positions(self) -> np.ndarray:
        state = self.arm.get_joint_state()
        position = self._validated_position(state)
        return np.concatenate((position, [state.gripper_pos]))

    def get_joint_velocities(self) -> np.ndarray:
        state = self.arm.get_joint_state()
        return np.concatenate((state.vel().copy(), [state.gripper_vel]))

    def get_runtime_diagnostics(self) -> dict[str, np.ndarray | float | str]:
        """Snapshot feedback and the SDK's current output command safely.

        The numpy views returned by pybind are owned by their C++ objects, so
        keep both JointState instances alive until every array is copied.
        """
        state = self.arm.get_joint_state()
        command = self.arm.get_joint_cmd()
        gain = self.arm.get_gain()
        return {
            "actual_pos": self._validated_position(state),
            "actual_vel": state.vel().copy(),
            "actual_torque": state.torque().copy(),
            "sdk_cmd_pos": command.pos().copy(),
            "sdk_cmd_vel": command.vel().copy(),
            "sdk_cmd_torque": command.torque().copy(),
            "kp": gain.kp().copy(),
            "kd": gain.kd().copy(),
            "gripper_actual_pos": float(state.gripper_pos),
            "gripper_actual_vel": float(state.gripper_vel),
            "gripper_actual_torque": float(state.gripper_torque),
            "gripper_mode": self._gripper_mode,
            "feedback_timestamp": float(state.timestamp),
            "command_timestamp": float(command.timestamp),
            "controller_timestamp": float(self.arm.get_timestamp()),
        }

    def _validated_position(self, state: arx5.JointState) -> np.ndarray:
        """Copy feedback while its owning C++ JointState is still alive."""
        position = state.pos().copy()
        q_min = np.asarray(self.robot_config.joint_pos_min, dtype=float)
        q_max = np.asarray(self.robot_config.joint_pos_max, dtype=float)
        tolerance_rad = 0.2
        valid = (
            position.shape == (self.robot_config.joint_dof,)
            and np.all(np.isfinite(position))
            and np.all(position >= q_min - tolerance_rad)
            and np.all(position <= q_max + tolerance_rad)
        )
        if not valid:
            if self.actuate:
                self.arm.set_to_damping()
            raise RuntimeError(f"Invalid X5 joint feedback; command blocked: {position}")
        return position

    def set_joint_positions(self, positions: np.ndarray) -> bool:
        target = np.asarray(positions, dtype=float)
        if target.shape != (self.robot_config.joint_dof,):
            raise ValueError(f"Expected six X5 joints, got {target.shape}")
        # Keep the returned JointState alive until after copying pos(). The
        # pybind pos() array references storage owned by JointState; chaining
        # get_joint_state().pos().copy() can read freed memory.
        state = self.arm.get_joint_state()
        q_actual = self._validated_position(state)
        now = time.monotonic()
        if now - self._last_position_command_time > self._command_gap_reset_s:
            self._previous_q_desired = target.copy()
            self._peak_ik_step = 0.0
            self._q_target = q_actual.copy()
        self._last_position_command_time = now
        desired_step = float(np.max(np.abs(target - self._previous_q_desired)))
        self._peak_ik_step = max(self._peak_ik_step, desired_step)
        self._previous_q_desired = target.copy()
        self._q_target, q_limited = bounded_joint_target(
            # Rate-limit the command trajectory, not measured feedback. Using
            # q_actual here would keep the target only one tiny step ahead of
            # a lagging motor, eliminating the position error needed by PD.
            q_actual=self._q_target,
            q_desired=target,
            q_min=np.asarray(self.robot_config.joint_pos_min),
            q_max=np.asarray(self.robot_config.joint_pos_max),
            max_step_rad=self._max_step_rad,
            soft_limit_margin_rad=self._soft_limit_margin_rad,
        )
        self._q_target = np.clip(
            self._q_target,
            q_actual - self._max_command_lead_rad,
            q_actual + self._max_command_lead_rad,
        )
        self._q_target = np.clip(
            self._q_target,
            np.asarray(self.robot_config.joint_pos_min),
            np.asarray(self.robot_config.joint_pos_max),
        )
        if now - self._last_diagnostic_time >= 0.2:
            ik_backlog = float(np.max(np.abs(q_limited - q_actual)))
            command_tracking_error = float(np.max(np.abs(self._q_target - q_actual)))
            print(
                "OFFICIAL BASELINE "
                f"q_actual={np.round(q_actual, 4)} "
                f"q_des={np.round(target, 4)} "
                f"q_safe={np.round(self._q_target, 4)} "
                f"peak_ik_step={self._peak_ik_step:.4f} "
                f"ik_backlog={ik_backlog:.4f} "
                f"command_tracking_error={command_tracking_error:.4f}"
            )
            self._last_diagnostic_time = now
            self._peak_ik_step = 0.0
        # XRoboToolkit calls set_catch_pos() immediately after this method.
        # Defer submission so the arm and gripper are combined into exactly
        # one SDK waypoint per 50 Hz control cycle.
        return True

    def _set_gripper_force_hold(self, state: arx5.JointState) -> None:
        if self._gripper_mode == "force_hold":
            return
        gain = self.arm.get_gain()
        gain.gripper_kp = 0.0
        gain.gripper_kd = 0.0
        self.arm.set_gain(gain)
        self._gripper_mode = "force_hold"
        self._gripper_contact_count = 0
        self._gripper_latched_position = float(
            np.clip(state.gripper_pos, 0.0, self.robot_config.gripper_width)
        )
        self._gripper_target = self._gripper_latched_position
        print(
            "GRIPPER POSITION -> FORCE_HOLD "
            f"pos={self._gripper_latched_position:.4f} "
            f"torque={float(state.gripper_torque):+.3f} "
            f"hold={self._gripper_closing_torque_sign * self._gripper_hold_torque:+.3f}"
        )

    def _set_gripper_position_mode(self) -> None:
        if self._gripper_mode == "position":
            return
        gain = self.arm.get_gain()
        gain.gripper_kp = self._position_gripper_kp
        gain.gripper_kd = self._position_gripper_kd
        self.arm.set_gain(gain)
        self._gripper_mode = "position"
        self._gripper_contact_count = 0
        print("GRIPPER FORCE_HOLD -> POSITION")

    def set_catch_pos(self, position: float) -> bool:
        requested = float(np.clip(position, 0.0, self.robot_config.gripper_width))

        if not self.actuate or not self._gripper_force_hold_enabled:
            self._gripper_target = requested
            self._submit()
            return True

        state = self.arm.get_joint_state()
        actual_pos = float(
            np.clip(state.gripper_pos, 0.0, self.robot_config.gripper_width)
        )

        if self._gripper_mode == "force_hold":
            # A trigger release moves the requested width back toward open.
            # Once it exceeds the contact width by a small margin, restore
            # position control and let the user's trigger open the gripper.
            if requested > self._gripper_latched_position + self._gripper_release_margin_m:
                self._set_gripper_position_mode()
                self._gripper_target = requested
            else:
                # Position is ignored while gripper kp == 0, but keeping it at
                # measured feedback makes the eventual mode transition smooth.
                self._gripper_target = actual_pos
        else:
            self._gripper_target = requested
            closing_intent = requested < actual_pos - 0.001
            closing_torque = (
                float(state.gripper_torque) * self._gripper_closing_torque_sign
            )
            if closing_intent and closing_torque >= self._gripper_contact_torque:
                self._gripper_contact_count += 1
            else:
                self._gripper_contact_count = 0

            if self._gripper_contact_count >= self._gripper_contact_cycles:
                self._set_gripper_force_hold(state)

        self._submit()
        return True

    def _submit(self) -> None:
        if not self.actuate:
            return
        command = arx5.JointState(self.robot_config.joint_dof)
        command.pos()[:] = self._q_target
        command.vel()[:] = 0.0
        command.torque()[:] = 0.0
        command.gripper_pos = self._gripper_target
        command.gripper_vel = 0.0
        command.gripper_torque = (
            self._gripper_closing_torque_sign * self._gripper_hold_torque
            if self._gripper_mode == "force_hold"
            else 0.0
        )
        command.timestamp = 0.0
        self.arm.set_joint_cmd(command)

    def set_to_damping(self) -> None:
        self.arm.set_to_damping()


class OfficialSingleX5Controller(ARXR5TeleopController):
    """Keep XRoboToolkit control logic; replace only model/device backend."""

    def __init__(
        self,
        *args,
        actuate: bool,
        preview_time_s: float,
        gripper_open_readout: float,
        gripper_width: float,
        max_joint_speed_rad_s: float,
        joint_soft_limit_margin_rad: float,
        gripper_force_hold: bool,
        gripper_contact_torque: float,
        gripper_hold_torque: float,
        gripper_contact_cycles: int,
        gripper_release_margin_m: float,
        dynamics_urdf_path: str = DEFAULT_X5_URDF_PATH,
        **kwargs,
    ) -> None:
        self._x5_actuate = actuate
        self._x5_preview_time_s = preview_time_s
        self._x5_gripper_open_readout = gripper_open_readout
        self._x5_gripper_width = gripper_width
        self._x5_max_joint_speed_rad_s = max_joint_speed_rad_s
        self._x5_joint_soft_limit_margin_rad = joint_soft_limit_margin_rad
        self._x5_gripper_force_hold = gripper_force_hold
        self._x5_gripper_contact_torque = gripper_contact_torque
        self._x5_gripper_hold_torque = gripper_hold_torque
        self._x5_gripper_contact_cycles = gripper_contact_cycles
        self._x5_gripper_release_margin_m = gripper_release_margin_m
        self._x5_dynamics_urdf_path = dynamics_urdf_path
        self._x5_control_rate_hz = int(kwargs["control_rate_hz"])
        super().__init__(*args, **kwargs)

    def _robot_setup(self) -> None:
        if getattr(self, "_x5_setup_done", False):
            return
        self.arm_controllers = {
            name: OfficialX5HardwareAdapter(
                can_port=port,
                actuate=self._x5_actuate,
                preview_time_s=self._x5_preview_time_s,
                gripper_open_readout=self._x5_gripper_open_readout,
                gripper_width=self._x5_gripper_width,
                control_rate_hz=self._x5_control_rate_hz,
                max_joint_speed_rad_s=self._x5_max_joint_speed_rad_s,
                joint_soft_limit_margin_rad=self._x5_joint_soft_limit_margin_rad,
                gripper_force_hold=self._x5_gripper_force_hold,
                gripper_contact_torque=self._x5_gripper_contact_torque,
                gripper_hold_torque=self._x5_gripper_hold_torque,
                gripper_contact_cycles=self._x5_gripper_contact_cycles,
                gripper_release_margin_m=self._x5_gripper_release_margin_m,
                dynamics_urdf_path=self._x5_dynamics_urdf_path,
            )
            for name, port in self.can_ports.items()
        }
        self._x5_setup_done = True
        mode = "ACTUATION ENABLED" if self._x5_actuate else "READ-ONLY/DAMPING"
        print(f"Official XRoboToolkit pipeline + X5 backend: {mode}")
        print("Automatic homing is disabled for this baseline.")

    def _shutdown_robot(self) -> None:
        for controller in self.arm_controllers.values():
            controller.set_to_damping()
        print("X5 switched to damping mode.")

    def run(self) -> None:
        try:
            super().run()
        finally:
            self.xr_client.close()


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
    max_joint_speed_rad_s: float = 0.2,
    joint_soft_limit_margin_rad: float = 0.05,
    actuate: bool = False,
    visualize_placo: bool = True,
) -> None:
    """Run the stock XRoboToolkit ARX control path on one X5."""
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

    controller = OfficialSingleX5Controller(
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
        dynamics_urdf_path=robot_urdf_path,
    )
    controller.run()


if __name__ == "__main__":
    tyro.cli(main)
