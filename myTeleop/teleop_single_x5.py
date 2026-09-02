#!/usr/bin/env python3
"""Single-arm Quest teleoperation for ARX X5 with bounded commands."""

import threading
import time
from copy import deepcopy
from pathlib import Path
from typing import Optional

import arx5_interface as arx5
import meshcat.transformations as tf
import numpy as np
import tyro

from xrobotoolkit_teleop.hardware.arx_r5_teleop_controller import (
    ARXR5TeleopController as BaseARXTeleopController,
    DEFAULT_ARX_R5_MANIPULATOR_CONFIG as DEFAULT_X5_MANIPULATOR_CONFIG,
)
from xrobotoolkit_teleop.utils.geometry import apply_delta_pose

try:
    from .telemetry_logger import TelemetryCSVLogger, add_vector
except ImportError:
    from telemetry_logger import TelemetryCSVLogger, add_vector


DEFAULT_X5_URDF_PATH = (
    "/home/r2/miniconda3/envs/xrobot312/lib/python3.12/"
    "site-packages/arx5_interface/models/X5.urdf"
)


class ARX5Interface:
    """Adapter from XRoboToolkit's controller API to arx5-interface."""

    def __init__(
        self,
        can_port: str,
        model: str,
        gripper_open_readout: float,
        gripper_width: float,
        preview_time_s: float,
        actuate: bool,
    ) -> None:
        self.actuate = actuate
        self.robot_config = arx5.RobotConfigFactory.get_instance().get_config(model)
        self.robot_config.gripper_open_readout = gripper_open_readout
        self.robot_config.gripper_width = gripper_width
        self.controller_config = arx5.ControllerConfigFactory.get_instance().get_config(
            "joint_controller", self.robot_config.joint_dof
        )
        self.controller_config.background_send_recv = True
        # The SDK default is zero, which turns every update into an immediate
        # fixed target. The official teleop example uses a future waypoint.
        self.controller_config.default_preview_time = preview_time_s

        self.arm = arx5.Arx5JointController(
            self.robot_config, self.controller_config, can_port
        )
        self.arm.set_log_level(arx5.LogLevel.INFO)

        initial_state = self.arm.get_joint_state()
        self._position_target = initial_state.pos().copy()
        self._gripper_target = float(
            np.clip(initial_state.gripper_pos, 0.0, self.robot_config.gripper_width)
        )

        if actuate:
            gain = arx5.Gain(
                self.controller_config.default_kp,
                self.controller_config.default_kd,
                self.controller_config.default_gripper_kp,
                self.controller_config.default_gripper_kd,
            )
            self.arm.set_gain(gain)
        else:
            self.arm.set_to_damping()

    def get_joint_positions(self) -> np.ndarray:
        state = self.arm.get_joint_state()
        return np.concatenate((state.pos().copy(), [state.gripper_pos]))

    def get_joint_velocities(self) -> np.ndarray:
        state = self.arm.get_joint_state()
        return np.concatenate((state.vel().copy(), [state.gripper_vel]))

    def get_position_target(self) -> np.ndarray:
        return self._position_target.copy()

    def set_command(
        self,
        positions: Optional[np.ndarray] = None,
        gripper_position: Optional[float] = None,
    ) -> None:
        if not self.actuate:
            return

        if positions is not None:
            target = np.asarray(positions, dtype=float)
            expected_shape = (self.robot_config.joint_dof,)
            if target.shape != expected_shape or not np.all(np.isfinite(target)):
                raise ValueError(
                    f"Invalid ARX joint target {target}; expected {expected_shape}"
                )
            self._position_target = target.copy()

        if gripper_position is not None:
            self._gripper_target = float(
                np.clip(gripper_position, 0.0, self.robot_config.gripper_width)
            )

        # Never reuse get_joint_cmd() here. Its torque already contains the
        # SDK's gravity compensation, so feeding it back would accumulate that
        # torque on every Python control cycle.
        command = arx5.JointState(self.robot_config.joint_dof)
        command.pos()[:] = self._position_target
        command.vel()[:] = 0.0
        command.torque()[:] = 0.0
        command.gripper_pos = self._gripper_target
        command.gripper_vel = 0.0
        command.gripper_torque = 0.0

        # The configured preview time makes timestamp=0 a future interpolated
        # waypoint instead of an immediate fixed target.
        command.timestamp = 0.0
        self.arm.set_joint_cmd(command)

    def set_to_damping(self) -> None:
        self.arm.set_to_damping()


class SingleX5TeleopController(BaseARXTeleopController):
    """Relative-clutch X5 teleoperation with XR and joint-command validation."""

    def __init__(
        self,
        *args,
        arm_model: str,
        gripper_open_readout: float,
        gripper_width: float,
        preview_time_s: float,
        max_joint_speed_rad_s: float,
        max_ik_step_rad: float,
        joint_soft_limit_margin_rad: float,
        xr_stale_timeout_s: float,
        xr_settle_samples: int,
        max_xr_step_m: float,
        max_xr_step_rad: float,
        rotation_scale_factor: float,
        dry_run: bool,
        telemetry_log: bool,
        telemetry_source: str,
        telemetry_log_dir: str,
        **kwargs,
    ) -> None:
        self.arm_model = arm_model
        self.gripper_open_readout = gripper_open_readout
        self.gripper_width = gripper_width
        self.preview_time_s = preview_time_s
        self.max_joint_speed_rad_s = max_joint_speed_rad_s
        self.max_ik_step_rad = max_ik_step_rad
        self.joint_soft_limit_margin_rad = joint_soft_limit_margin_rad
        self.xr_stale_timeout_s = xr_stale_timeout_s
        self.xr_settle_samples = max(1, xr_settle_samples)
        self.max_xr_step_m = max_xr_step_m
        self.max_xr_step_rad = max_xr_step_rad
        self.rotation_scale_factor = rotation_scale_factor
        self.dry_run = dry_run
        self.telemetry_source = telemetry_source

        self._state_lock = threading.RLock()
        self._last_xr_timestamp_ns = 0
        self._last_xr_update_time = 0.0
        self._candidate_pose: dict[str, np.ndarray] = {}
        self._candidate_samples: dict[str, int] = {}
        self._last_follow_pose: dict[str, np.ndarray] = {}
        self._clutch_latched: dict[str, bool] = {}
        self._last_warning_time: dict[str, float] = {}
        self._last_diagnostic_time = 0.0
        self._latest_xr_pose: dict[str, np.ndarray] = {}
        self._latest_grip: dict[str, float] = {}
        self._actual_eef_pose: dict[str, np.ndarray] = {}
        self._ik_eef_pose: dict[str, np.ndarray] = {}
        self._xr_delta_world: dict[str, np.ndarray] = {}
        self._last_accepted_ik_target: dict[str, np.ndarray] = {}
        self._telemetry_logger: Optional[TelemetryCSVLogger] = None

        super().__init__(*args, **kwargs)
        if telemetry_log:
            self._telemetry_logger = TelemetryCSVLogger(
                telemetry_log_dir, telemetry_source
            )
            print(f"Telemetry log: {self._telemetry_logger.path}")

    def run(self) -> None:
        """Run the upstream hardware threads and close the native XR client."""
        try:
            super().run()
        finally:
            try:
                self.xr_client.close()
            finally:
                if self._telemetry_logger is not None:
                    self._telemetry_logger.close()

    def _robot_setup(self) -> None:
        if getattr(self, "_single_arm_hardware_initialized", False):
            return

        self.arm_controllers = {
            arm_name: ARX5Interface(
                can_port=can_port,
                model=self.arm_model,
                gripper_open_readout=self.gripper_open_readout,
                gripper_width=self.gripper_width,
                preview_time_s=self.preview_time_s,
                actuate=not self.dry_run,
            )
            for arm_name, can_port in self.can_ports.items()
        }
        self._single_arm_hardware_initialized = True

        # Do not command the gripper open as soon as the process starts.
        for arm_name, controller in self.arm_controllers.items():
            current = controller.get_joint_positions()
            config = self.manipulator_config[arm_name]
            if "gripper_config" in config:
                joint_name = config["gripper_config"]["joint_names"][0]
                self.gripper_pos_target[arm_name][joint_name] = float(
                    np.clip(current[-1], 0.0, self.gripper_width)
                )

        mode = "DRY-RUN damping mode" if self.dry_run else "bounded command mode"
        print(f"ARX X5 initialized at its current pose ({mode}).")

    def _shutdown_robot(self) -> None:
        for controller in self.arm_controllers.values():
            controller.set_to_damping()
        print("ARX X5 switched to damping mode.")

    def _control_thread(self, stop_event: threading.Event) -> None:
        """Guarantee damping even when command generation raises an exception."""
        try:
            while not stop_event.is_set():
                started = time.monotonic()
                self._send_command()
                elapsed = time.monotonic() - started
                time.sleep(max(0.0, 1.0 / self.control_rate_hz - elapsed))
        finally:
            self._shutdown_robot()
            print("Control loop has stopped.")

    def _update_robot_state(self) -> None:
        with self._state_lock:
            super()._update_robot_state()

    def _update_gripper_target(self) -> None:
        with self._state_lock:
            super()._update_gripper_target()

    def _xr_is_fresh(self) -> bool:
        timestamp_ns = int(self.xr_client.get_timestamp_ns())
        now = time.monotonic()
        if timestamp_ns > 0 and timestamp_ns != self._last_xr_timestamp_ns:
            self._last_xr_timestamp_ns = timestamp_ns
            self._last_xr_update_time = now
        return (
            timestamp_ns > 0
            and self._last_xr_update_time > 0.0
            and now - self._last_xr_update_time <= self.xr_stale_timeout_s
        )

    @staticmethod
    def _normalize_xyzw(quaternion: np.ndarray) -> Optional[np.ndarray]:
        quaternion = np.asarray(quaternion, dtype=float)
        if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
            return None
        norm = float(np.linalg.norm(quaternion))
        if norm < 0.5:
            return None
        return quaternion / norm

    def _validated_xr_pose(self, pose: np.ndarray) -> Optional[np.ndarray]:
        pose = np.asarray(pose, dtype=float)
        if pose.shape != (7,) or not np.all(np.isfinite(pose)):
            return None
        if not self._xr_is_fresh() or np.linalg.norm(pose[:3]) > 10.0:
            return None
        quaternion = self._normalize_xyzw(pose[3:7])
        if quaternion is None:
            return None
        result = pose.copy()
        result[3:7] = quaternion
        return result

    @staticmethod
    def _pose_as_xyzw(position: np.ndarray, quaternion_wxyz: np.ndarray) -> np.ndarray:
        quaternion_wxyz = np.asarray(quaternion_wxyz, dtype=float)
        return np.concatenate(
            (np.asarray(position, dtype=float), quaternion_wxyz[[1, 2, 3, 0]])
        )

    @staticmethod
    def _quaternion_distance_rad(q1_xyzw: np.ndarray, q2_xyzw: np.ndarray) -> float:
        q1 = q1_xyzw / np.linalg.norm(q1_xyzw)
        q2 = q2_xyzw / np.linalg.norm(q2_xyzw)
        dot = float(np.clip(abs(float(np.dot(q1, q2))), -1.0, 1.0))
        return 2.0 * float(np.arccos(dot))

    def _pose_step_safe(
        self, arm_name: str, previous: np.ndarray, current: np.ndarray
    ) -> bool:
        translation_step = float(np.linalg.norm(current[:3] - previous[:3]))
        if translation_step > self.max_xr_step_m:
            self._warn_throttled(
                f"{arm_name}_xr_xyz",
                f"Rejecting XR translation jump: {translation_step:.3f} m",
            )
            return False

        rotation_step = self._quaternion_distance_rad(
            previous[3:7], current[3:7]
        )
        if rotation_step > self.max_xr_step_rad:
            self._warn_throttled(
                f"{arm_name}_xr_rot",
                f"Rejecting XR rotation jump: {rotation_step:.3f} rad",
            )
            return False
        return True

    def _pose_settled(self, arm_name: str, pose: np.ndarray) -> bool:
        previous = self._candidate_pose.get(arm_name)
        if previous is None or not self._pose_step_safe(arm_name, previous, pose):
            self._candidate_pose[arm_name] = pose.copy()
            self._candidate_samples[arm_name] = 1
        else:
            self._candidate_pose[arm_name] = pose.copy()
            self._candidate_samples[arm_name] += 1
        return self._candidate_samples[arm_name] >= self.xr_settle_samples

    def _release_clutch(self, arm_name: str, announce: bool = False) -> None:
        was_latched = self._clutch_latched.get(arm_name, False)
        self.active[arm_name] = False
        self._clutch_latched[arm_name] = False
        self.ref_ee_xyz[arm_name] = None
        self.ref_ee_quat[arm_name] = None
        self.ref_controller_xyz[arm_name] = None
        self.ref_controller_quat[arm_name] = None
        self._candidate_pose.pop(arm_name, None)
        self._candidate_samples.pop(arm_name, None)
        self._last_follow_pose.pop(arm_name, None)
        self._xr_delta_world.pop(arm_name, None)
        if announce and was_latched:
            print(f"{arm_name}: clutch released.")

    def _latch_clutch(
        self, arm_name: str, config: dict, xr_pose: np.ndarray
    ) -> None:
        self.placo_robot.update_kinematics()
        ee_xyz, ee_quat = self._get_link_pose(config["link_name"])
        self.ref_ee_xyz[arm_name] = ee_xyz.copy()
        self.ref_ee_quat[arm_name] = np.asarray(ee_quat, dtype=float).copy()
        self.ref_controller_xyz[arm_name] = None
        self.ref_controller_quat[arm_name] = None
        self._process_xr_pose(xr_pose, arm_name)

        if self.effector_control_mode[arm_name] == "position":
            self.effector_task[arm_name].target_world = ee_xyz.copy()
        else:
            target_pose = tf.quaternion_matrix(ee_quat)
            target_pose[:3, 3] = ee_xyz
            self.effector_task[arm_name].T_world_frame = target_pose

        self.active[arm_name] = True
        self._clutch_latched[arm_name] = True
        self._last_follow_pose[arm_name] = xr_pose.copy()
        dof = self.arm_controllers[arm_name].robot_config.joint_dof
        self._last_accepted_ik_target[arm_name] = self.arm_controllers[
            arm_name
        ].get_joint_positions()[:dof].copy()
        print(f"{arm_name}: clutch latched at the current robot pose.")

    def _update_ik(self) -> None:
        with self._state_lock:
            # Refresh feedback inside the same critical section as the solve.
            super()._update_robot_state()
            self.placo_robot.update_kinematics()

            for arm_name, config in self.manipulator_config.items():
                ee_xyz, ee_quat = self._get_link_pose(config["link_name"])
                self._actual_eef_pose[arm_name] = self._pose_as_xyzw(
                    ee_xyz, ee_quat
                )

            for arm_name, config in self.manipulator_config.items():
                grip = float(
                    self.xr_client.get_key_value_by_name(config["control_trigger"])
                )
                raw_xr_pose = np.asarray(
                    self.xr_client.get_pose_by_name(config["pose_source"]),
                    dtype=float,
                )
                self._latest_grip[arm_name] = grip
                self._latest_xr_pose[arm_name] = raw_xr_pose.copy()
                if grip <= 0.9:
                    self._release_clutch(arm_name, announce=True)
                    continue

                xr_pose = self._validated_xr_pose(raw_xr_pose)
                if xr_pose is None:
                    self._release_clutch(arm_name)
                    self._warn_throttled(
                        f"{arm_name}_xr_invalid",
                        "Grip held, but XR data is invalid or stale; command blocked.",
                    )
                    continue

                if not self._clutch_latched.get(arm_name, False):
                    if not self._pose_settled(arm_name, xr_pose):
                        self.active[arm_name] = False
                        continue
                    self._latch_clutch(arm_name, config, xr_pose)
                    continue

                previous = self._last_follow_pose.get(arm_name)
                if previous is not None and not self._pose_step_safe(
                    arm_name, previous, xr_pose
                ):
                    self._release_clutch(arm_name)
                    self._warn_throttled(
                        f"{arm_name}_relatch",
                        f"{arm_name}: XR jump detected; release and press Grip again.",
                    )
                    continue
                self._last_follow_pose[arm_name] = xr_pose.copy()

                delta_xyz, delta_rot = self._process_xr_pose(xr_pose, arm_name)
                scaled_delta_rot = delta_rot * self.rotation_scale_factor
                self._xr_delta_world[arm_name] = np.concatenate(
                    (delta_xyz.copy(), scaled_delta_rot.copy())
                )
                if self.effector_control_mode[arm_name] == "position":
                    self.effector_task[arm_name].target_world = (
                        self.ref_ee_xyz[arm_name] + delta_xyz
                    )
                else:
                    target_xyz, target_quat = apply_delta_pose(
                        self.ref_ee_xyz[arm_name],
                        self.ref_ee_quat[arm_name],
                        delta_xyz,
                        scaled_delta_rot,
                    )
                    target_pose = tf.quaternion_matrix(target_quat)
                    target_pose[:3, 3] = target_xyz
                    self.effector_task[arm_name].T_world_frame = target_pose

            # The upstream controller solves even while every clutch is open.
            # Keep q exactly equal to hardware feedback until an arm is active.
            if not any(self.active.values()):
                self._ik_eef_pose = {
                    name: pose.copy() for name, pose in self._actual_eef_pose.items()
                }
                return

            try:
                self.solver.solve(True)
            except RuntimeError as exc:
                self._warn_throttled("ik_failure", f"IK solver failed: {exc}")

            for arm_name, config in self.manipulator_config.items():
                ee_xyz, ee_quat = self._get_link_pose(config["link_name"])
                self._ik_eef_pose[arm_name] = self._pose_as_xyzw(ee_xyz, ee_quat)

    def _send_command(self) -> None:
        with self._state_lock:
            xr_fresh = self._xr_is_fresh()

            for arm_name, controller in self.arm_controllers.items():
                positions = None
                submitted_positions = None
                rejection_reason = ""
                command_sent = False
                joint_state = controller.get_joint_positions()
                q_now = joint_state[: controller.robot_config.joint_dof]
                q_des = self.placo_robot.state.q[
                    self.placo_arm_joint_slice[arm_name]
                ].copy()
                tracking_error = float(np.max(np.abs(q_des - q_now)))
                previous_ik = self._last_accepted_ik_target.get(arm_name)
                ik_step = (
                    float(np.max(np.abs(q_des - previous_ik)))
                    if previous_ik is not None
                    else 0.0
                )
                active = self.active.get(arm_name, False)

                if self.dry_run:
                    now = time.monotonic()
                    if now - self._last_diagnostic_time >= 0.2:
                        print(
                            f"DRY-RUN active={active} "
                            f"tracking_error={tracking_error:.4f} "
                            f"ik_step={ik_step:.4f} q_now={np.round(q_now, 4)} "
                            f"q_ik={np.round(q_des, 4)}"
                        )
                        self._last_diagnostic_time = now
                elif active and not xr_fresh:
                    rejection_reason = "xr_stale"
                elif active:
                    if not np.all(np.isfinite(q_des)):
                        rejection_reason = "non_finite_ik"
                        self._warn_throttled("invalid_q", "Rejecting non-finite IK target.")
                    else:
                        q_min = np.asarray(controller.robot_config.joint_pos_min)
                        q_max = np.asarray(controller.robot_config.joint_pos_max)
                        margin = self.joint_soft_limit_margin_rad
                        q_soft_min = np.minimum(q_min + margin, (q_min + q_max) / 2.0)
                        q_soft_max = np.maximum(q_max - margin, (q_min + q_max) / 2.0)
                        q_des = np.clip(q_des, q_soft_min, q_soft_max)
                        previous_ik = self._last_accepted_ik_target.get(arm_name)
                        ik_step = (
                            float(np.max(np.abs(q_des - previous_ik)))
                            if previous_ik is not None
                            else 0.0
                        )
                        if ik_step > self.max_ik_step_rad:
                            rejection_reason = "ik_jump"

                    if rejection_reason == "ik_jump":
                        self._warn_throttled(
                            "ik_jump",
                            f"Rejecting IK target step: {ik_step:.3f} rad; "
                            "release and press Grip again.",
                        )
                        self._release_clutch(arm_name)

                    if not rejection_reason:
                        self._last_accepted_ik_target[arm_name] = q_des.copy()
                        error = q_des - q_now
                        max_step = self.max_joint_speed_rad_s / self.control_rate_hz
                        positions = q_now + np.clip(error, -max_step, max_step)

                gripper_position = None
                if "gripper_config" in self.manipulator_config[arm_name]:
                    gripper_config = self.manipulator_config[arm_name][
                        "gripper_config"
                    ]
                    joint_name = gripper_config["joint_names"][0]
                    gripper_position = self.gripper_pos_target[arm_name][joint_name]

                if not self.dry_run and xr_fresh and not rejection_reason:
                    controller.set_command(
                        positions=positions, gripper_position=gripper_position
                    )
                    command_sent = positions is not None or gripper_position is not None
                    if command_sent:
                        submitted_positions = controller.get_position_target()

                self._write_telemetry(
                    arm_name=arm_name,
                    xr_fresh=xr_fresh,
                    command_sent=command_sent,
                    rejection_reason=rejection_reason,
                    q_actual=q_now,
                    q_ik=q_des,
                    q_command=submitted_positions,
                    command_velocity=(
                        np.zeros_like(q_now) if command_sent else None
                    ),
                    command_torque=(np.zeros_like(q_now) if command_sent else None),
                    gripper_actual=float(joint_state[-1]),
                    gripper_target=gripper_position,
                    max_joint_error=tracking_error,
                    max_ik_step=ik_step,
                )

    def _write_telemetry(
        self,
        *,
        arm_name: str,
        xr_fresh: bool,
        command_sent: bool,
        rejection_reason: str,
        q_actual: np.ndarray,
        q_ik: np.ndarray,
        q_command: Optional[np.ndarray],
        command_velocity: Optional[np.ndarray],
        command_torque: Optional[np.ndarray],
        gripper_actual: float,
        gripper_target: Optional[float],
        max_joint_error: float,
        max_ik_step: float,
    ) -> None:
        if self._telemetry_logger is None:
            return

        row = {
            "host_time_ns": time.time_ns(),
            "xr_timestamp_ns": self._last_xr_timestamp_ns,
            "source": self.telemetry_source,
            "active": int(self.active.get(arm_name, False)),
            "xr_fresh": int(xr_fresh),
            "command_sent": int(command_sent),
            "rejection_reason": rejection_reason,
            "right_grip": self._latest_grip.get(arm_name, 0.0),
            "right_trigger": float(
                self.xr_client.get_key_value_by_name("right_trigger")
            ),
            "gripper_actual_m": gripper_actual,
            "gripper_target_m": "" if gripper_target is None else gripper_target,
            "gripper_command_velocity": 0.0 if command_sent else "",
            "gripper_command_torque": 0.0 if command_sent else "",
            "max_joint_error_rad": max_joint_error,
            "max_ik_step_rad": max_ik_step,
        }
        add_vector(row, "xr", self._latest_xr_pose.get(arm_name))
        add_vector(row, "eef_actual", self._actual_eef_pose.get(arm_name))
        add_vector(row, "eef_ik", self._ik_eef_pose.get(arm_name))
        add_vector(row, "xr_delta_world", self._xr_delta_world.get(arm_name))
        add_vector(row, "q_actual", q_actual)
        add_vector(row, "q_ik", q_ik)
        add_vector(row, "q_command", q_command)
        add_vector(row, "q_command_velocity", command_velocity)
        add_vector(row, "q_command_torque", command_torque)
        self._telemetry_logger.write(row)

    def _warn_throttled(
        self, key: str, message: str, interval_s: float = 1.0
    ) -> None:
        now = time.monotonic()
        if now - self._last_warning_time.get(key, 0.0) >= interval_s:
            print(message)
            self._last_warning_time[key] = now


def main(
    robot_urdf_path: str = DEFAULT_X5_URDF_PATH,
    can_port: str = "can0",
    arm_model: str = "X5",
    gripper_open_readout: float = -3.4,
    gripper_width: float = 0.082,
    scale_factor: float = 0.1,
    rotation_scale_factor: float = 0.5,
    control_mode: str = "pose",
    control_rate_hz: int = 50,
    preview_time_s: float = 0.05,
    max_joint_speed_rad_s: float = 0.5,
    max_ik_step_rad: float = 0.35,
    joint_soft_limit_margin_rad: float = 0.1,
    xr_stale_timeout_s: float = 0.25,
    xr_settle_samples: int = 3,
    max_xr_step_m: float = 0.08,
    max_xr_step_rad: float = 1.0,
    dry_run: bool = False,
    telemetry_log: bool = True,
    visualize_placo: bool = False,
    enable_camera: bool = False,
    enable_log_data: bool = False,
    log_dir: str = str(Path(__file__).resolve().parent / "logs"),
) -> None:
    """Run bounded X5 teleoperation; use --dry-run to suppress motor commands."""
    if control_mode not in {"position", "pose"}:
        raise ValueError("control_mode must be 'position' or 'pose'")
    if control_rate_hz <= 0 or preview_time_s <= 0.0:
        raise ValueError("control_rate_hz and preview_time_s must be positive")
    if max_joint_speed_rad_s <= 0.0 or max_ik_step_rad <= 0.0:
        raise ValueError("joint speed and IK step limits must be positive")
    if joint_soft_limit_margin_rad < 0.0:
        raise ValueError("joint soft-limit margin cannot be negative")
    if not Path(robot_urdf_path).is_file():
        raise FileNotFoundError(f"Robot URDF does not exist: {robot_urdf_path}")

    manipulator_config = deepcopy(DEFAULT_X5_MANIPULATOR_CONFIG)
    manipulator_config["right_arm"]["control_mode"] = control_mode
    manipulator_config["right_arm"]["gripper_config"]["open_pos"] = [
        gripper_width
    ]
    manipulator_config["right_arm"]["gripper_config"]["close_pos"] = [0.0]

    print(f"X5 URDF: {robot_urdf_path}")
    control_description = (
        "XYZ with orientation held"
        if control_mode == "pose" and rotation_scale_factor == 0.0
        else control_mode
    )
    print(
        f"Mode: {'DRY-RUN' if dry_run else 'HARDWARE'}, control={control_description}, "
        f"scale={scale_factor}, joint_speed={max_joint_speed_rad_s:.3f} rad/s, "
        f"rotation_scale={rotation_scale_factor}, preview={preview_time_s:.3f} s"
    )
    print(
        "Quest mapping: forward/back -> robot X, left/right -> robot Y, "
        "up/down -> robot Z."
    )

    controller = SingleX5TeleopController(
        robot_urdf_path=robot_urdf_path,
        manipulator_config=manipulator_config,
        can_ports={"right_arm": can_port},
        arm_model=arm_model,
        gripper_open_readout=gripper_open_readout,
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
        dry_run=dry_run,
        telemetry_log=telemetry_log,
        telemetry_source="dry_run" if dry_run else "hardware",
        telemetry_log_dir=log_dir,
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
