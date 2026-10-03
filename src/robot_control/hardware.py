"""Shared Stanford X5 adapter; importing it does not open CAN or XR.

Extracted from the current teleop backend, not a desktop hardware implementation.
"""

import time
from pathlib import Path

import arx5_interface as arx5
import numpy as np

from robot_core.device_lease import DeviceLease
from robot_core.project import X5_2025_URDF
from .trajectory import bounded_joint_target, validate_motion_limits

DEFAULT_X5_URDF_PATH = str(X5_2025_URDF)


class OfficialX5HardwareAdapter:
    """Expose joint, gripper and feedback operations through the X5-compatible SDK."""

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
        max_command_lead_rad: float = 0.05,
        gripper_open_limit_m: float = 0.078,
        gripper_force_hold: bool = True,
        gripper_contact_torque: float = 0.60,
        gripper_hold_torque: float = 0.30,
        gripper_contact_cycles: int = 1,
        gripper_release_margin_m: float = 0.004,
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
        if getattr(arx5, "quest_gripper_turns_version", None) != 1:
            raise RuntimeError("夹爪启动修复 SDK 未加载；请使用项目启动脚本，参见 docs/gripper-startup.md")
        self.robot_config.gripper_startup_turns = True
        if not np.isfinite(gripper_open_limit_m) or not 0 < gripper_open_limit_m <= gripper_width:
            raise ValueError("gripper_open_limit_m must be positive and no greater than gripper_width")
        self._gripper_open_limit_m = gripper_open_limit_m
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

        validate_motion_limits(max_joint_speed_rad_s, max_command_lead_rad, preview_time_s, joint_soft_limit_margin_rad)
        self._preview_time_s = preview_time_s
        self._max_command_lead_rad = max_command_lead_rad
        hardware_speed = np.asarray(self.robot_config.joint_vel_max)
        self._max_step_rad = (hardware_speed if max_joint_speed_rad_s == 0 else
                              np.minimum(hardware_speed, max_joint_speed_rad_s)) / control_rate_hz
        self._command_gap_reset_s = 3.0 / control_rate_hz
        self._soft_limit_margin_rad = joint_soft_limit_margin_rad
        self._submission_sequence = 0
        self._last_submission = None
        self._requested_joints = None
        self._requested_monotonic_ns = None
        self._requested_gripper_pos = None

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
        self._position_gain = arx5.Gain(config.default_kp, config.default_kd,
                                        config.default_gripper_kp, config.default_gripper_kd)
        self._control_rate_hz = control_rate_hz
        self._position_gripper_kp = float(config.default_gripper_kp)
        self._position_gripper_kd = float(config.default_gripper_kd)

        self._motor_feedback = None
        self._device_lease = DeviceLease(can_port)
        try:
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
        except BaseException:
            self.close()
            raise

        from .motor_feedback import MotorFeedback
        self._motor_feedback = MotorFeedback(can_port, self.robot_config.motor_id,
            [t.name for t in self.robot_config.motor_type], self.robot_config.gripper_motor_id,
            self.robot_config.gripper_motor_type.name)

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
        diagnostics = self._motor_feedback.snapshot() if self._motor_feedback else None
        motors = diagnostics['motors'] if diagnostics else []
        return {
            "joint_current_a": [m['current_a'] for m in motors[:6]] if motors else [None] * 6,
            "gripper_current_a": motors[6]['current_a'] if motors else None,
            "feedback_valid": [m['valid'] for m in motors] if motors else [False] * 7,
            "motor_feedback_monotonic_ns": [m['host_received_monotonic_ns'] for m in motors] if motors else [None] * 7,
            "motor_load_raw": [m['load_raw'] for m in motors] if motors else [None] * 7,
            "motor_diagnostics": diagnostics,
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
            "gripper_sdk_cmd_pos": float(command.gripper_pos),
            "gripper_sdk_cmd_vel": float(command.gripper_vel),
            "gripper_sdk_cmd_torque": float(command.gripper_torque),
            "gripper_kp": float(gain.gripper_kp),
            "gripper_kd": float(gain.gripper_kd),
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

    def get_recording_snapshot(self) -> dict:
        """Separate input target, accepted Python waypoint and SDK interpolated output."""
        diagnostics = self.get_runtime_diagnostics()
        result = {k: v.tolist() if isinstance(v, np.ndarray) else v for k, v in diagnostics.items()}
        result.update({"requested_pos": self._requested_joints,
                       "requested_monotonic_ns": self._requested_monotonic_ns,
                       "requested_gripper_pos": self._requested_gripper_pos,
                       "submission": self._last_submission, "actuate": self.actuate,
                       "motion_diagnostics": getattr(self, "_motion_diagnostics", None)})
        return result

    def set_planned_joint_state(self, positions, velocity):
        """Consume the service trajectory without a second clipping/interpolation layer."""
        q, dq = np.asarray(positions), np.asarray(velocity)
        if q.shape != (6,) or dq.shape != (6,) or not np.isfinite([q, dq]).all():
            raise ValueError("Invalid planned joint state")
        if np.any(q < np.asarray(self.robot_config.joint_pos_min) - 1e-8) or np.any(q > np.asarray(self.robot_config.joint_pos_max) + 1e-8):
            raise ValueError("Planned waypoint outside hard limits")
        self._q_target = q.copy()
        self._planned_velocity = dq.copy()
        self._requested_joints = q.tolist()
        self._requested_monotonic_ns = time.monotonic_ns()

    def set_joint_positions(self, positions: np.ndarray) -> bool:
        self._planned_velocity = None
        target = np.asarray(positions, dtype=float)
        if target.shape != (self.robot_config.joint_dof,):
            raise ValueError(f"Expected six X5 joints, got {target.shape}")
        # Keep the returned JointState alive until after copying pos(). The
        # pybind pos() array references storage owned by JointState; chaining
        # get_joint_state().pos().copy() can read freed memory.
        state = self.arm.get_joint_state()
        q_actual = self._validated_position(state)
        self._requested_joints = target.tolist()
        self._requested_monotonic_ns = time.monotonic_ns()
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
        if self._max_command_lead_rad > 0:
            self._q_target = np.clip(self._q_target, q_actual - self._max_command_lead_rad,
                                     q_actual + self._max_command_lead_rad)
        self._q_target = np.clip(
            self._q_target,
            np.asarray(self.robot_config.joint_pos_min),
            np.asarray(self.robot_config.joint_pos_max),
        )
        if now - self._last_diagnostic_time >= 0.2:
            ik_backlog = float(np.max(np.abs(q_limited - q_actual)))
            command_tracking_error = float(np.max(np.abs(self._q_target - q_actual)))
            self._motion_diagnostics = {
                "sample_monotonic_ns": time.monotonic_ns(),
                "q_actual": q_actual.tolist(), "q_des": target.tolist(),
                "q_safe": self._q_target.tolist(),
                "peak_ik_step": self._peak_ik_step,
                "ik_backlog": ik_backlog,
                "command_tracking_error": command_tracking_error,
            }
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
        requested = float(np.clip(position, 0.0, self._gripper_open_limit_m))
        self._requested_gripper_pos = requested

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
        command.vel()[:] = self._planned_velocity if getattr(self, "_planned_velocity", None) is not None else 0.0
        command.torque()[:] = 0.0
        command.gripper_pos = self._gripper_target
        command.gripper_vel = 0.0
        command.gripper_torque = (
            self._gripper_closing_torque_sign * self._gripper_hold_torque
            if self._gripper_mode == "force_hold"
            else 0.0
        )
        command.timestamp = float(self.arm.get_timestamp()) + self._preview_time_s
        self.arm.set_joint_cmd(command)
        self._submission_sequence += 1
        self._last_submission = {
            "sequence": self._submission_sequence, "host_monotonic_ns": time.monotonic_ns(),
            "pos": command.pos().copy().tolist(), "vel": command.vel().copy().tolist(),
            "torque": command.torque().copy().tolist(), "gripper_pos": float(command.gripper_pos),
            "gripper_vel": float(command.gripper_vel), "gripper_torque": float(command.gripper_torque),
            "timestamp_argument": float(command.timestamp), "gripper_mode": self._gripper_mode,
        }

    def set_to_damping(self) -> None:
        self._planned_velocity = None
        self.actuate = False
        self.arm.set_to_damping()

    def set_control_enabled(self, enabled: bool) -> None:
        """Service-owned transition, seeded from feedback; never home or open the gripper."""
        if not enabled:
            self.set_to_damping()
            return
        state = self.arm.get_joint_state()
        self._planned_velocity = None
        self._q_target = self._validated_position(state)
        self._previous_q_desired = self._q_target.copy()
        self._gripper_target = float(np.clip(state.gripper_pos, 0., self.robot_config.gripper_width))
        self._gripper_latched_position = self._gripper_target
        self._gripper_mode = "position"
        self._gripper_contact_count = 0
        self._last_position_command_time = 0.
        self._requested_joints = self._requested_monotonic_ns = None
        self._requested_gripper_pos = self._last_submission = None
        self.actuate = True
        self._submit()
        self.arm.set_gain(self._position_gain)

    def set_motion_limit(self, speed: float, *, max_command_lead_rad=None,
                         preview_time_s=None, joint_soft_limit_margin_rad=None) -> None:
        lead = self._max_command_lead_rad if max_command_lead_rad is None else max_command_lead_rad
        preview = self._preview_time_s if preview_time_s is None else preview_time_s
        margin = self._soft_limit_margin_rad if joint_soft_limit_margin_rad is None else joint_soft_limit_margin_rad
        validate_motion_limits(speed, lead, preview, margin)
        hardware_speed = np.asarray(self.robot_config.joint_vel_max)
        self._max_step_rad = (hardware_speed if speed == 0 else np.minimum(hardware_speed, speed)) / self._control_rate_hz
        self._max_command_lead_rad, self._preview_time_s, self._soft_limit_margin_rad = lead, preview, margin


    def close(self) -> None:
        """Release native CAN ownership before releasing the cooperative lease."""
        arm = getattr(self, "arm", None)
        try:
            if arm is not None:
                arm.set_to_damping()
        finally:
            if getattr(self, "_motor_feedback", None) is not None:
                self._motor_feedback.close()
            self.arm = None
            del arm
            lease = getattr(self, "_device_lease", None)
            if lease is not None:
                lease.close()

    def __del__(self):
        try:
            self.close()
        except Exception:
            # Destructors cannot report errors reliably during interpreter shutdown.
            pass
