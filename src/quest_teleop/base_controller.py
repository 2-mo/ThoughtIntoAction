"""X5 feedback/command bridge over XRoboToolkit's generic Quest/IK loop.

Adapted from XR Robotics (MIT), revision 79e5cb8a56e3455515ce1b476e993c764ec58739.
See LICENSE.XRoboToolkit and docs/x5-controller.md for provenance.
Input/telemetry lifecycles are owned here; arm construction uses the selected backend.
"""

from copy import deepcopy
import threading
import time
from typing import Dict

import numpy as np

from xrobotoolkit_teleop.common.base_hardware_teleop_controller import HardwareTeleopController
from xrobotoolkit_teleop.utils.geometry import R_HEADSET_TO_WORLD
from robot_core.plugins import Registry
from robot_core.model import RobotModel

DEFAULT_X5_MANIPULATOR_CONFIG = {
    "right_arm": {
        "link_name": "link6",
        "pose_source": "right_controller",
        "control_trigger": "right_grip",
        "gripper_config": {
            "type": "parallel",
            "gripper_trigger": "right_trigger",
            "joint_names": ["joint7"],
            "open_pos": [0.082],
            "close_pos": [0.0],
        },
    },
}


class X5TeleopBase(HardwareTeleopController):
    """Shared six-joint mapping and component lifecycles, without automatic homing."""

    def __init__(
        self,
        robot_urdf_path: str,
        manipulator_config: dict,
        can_ports: Dict[str, str],
        R_headset_world: np.ndarray = R_HEADSET_TO_WORLD,
        scale_factor: float = 1.0,
        telemetry: str = "none",
        input_plugin: str = "quest",
        control_rate_hz: int = 50,
        enable_log_data: bool = False,
        log_dir: str = "outputs/logs",
        log_freq: float = 50,
        enable_camera: bool = False,
        camera_serial_dict: Dict[str, str] | None = None,
        camera_width: int | None = None,
        camera_height: int | None = None,
        camera_fps: int | None = None,
        enable_camera_depth: bool = False,
        enable_camera_compression: bool = True,
        camera_jpg_quality: int = 85,
    ):
        # Legacy --enable-camera uses this workstation's configured wrist camera.
        # The main presets controller keeps this off and shares RGB via TeleopSession.
        camera = {}
        if enable_camera and camera_serial_dict is None:
            from robot_core.project import load_config
            camera = load_config()["cameras"]["wrist_rgb"]
            if not camera["enabled"] or not camera["serial"]:
                raise ValueError("Enable and configure cameras.wrist_rgb before opening RGB")
            camera_serial_dict = {"wrist_rgb": camera["serial"]}
        if enable_camera and (not camera_serial_dict or not all(camera_serial_dict.values())):
            raise ValueError("Camera serial numbers must be explicit")
        self.can_ports = dict(can_ports)
        self.camera_serial_dict = dict(camera_serial_dict or {})
        self.camera_serial_to_name = {serial: name for name, serial in self.camera_serial_dict.items()}
        self.camera_width = camera_width if camera_width is not None else camera.get("width", 640)
        self.camera_height = camera_height if camera_height is not None else camera.get("height", 480)
        camera_fps = camera_fps if camera_fps is not None else camera.get("fps", 30)
        self.enable_camera_depth = enable_camera_depth
        self.enable_camera_compression = enable_camera_compression
        self.camera_jpg_quality = camera_jpg_quality
        # Own initialization so input/backend/telemetry factories are never patched globally.
        # Field layout and IK behavior follow the pinned upstream MIT implementation.
        self.robot_urdf_path = robot_urdf_path
        self.manipulator_config = deepcopy(manipulator_config)
        self.floating_base, self.q_init = False, None
        self.R_headset_world, self.scale_factor = R_headset_world, scale_factor
        self.dt = 1.0 / control_rate_hz
        self.control_rate_hz, self.log_freq = control_rate_hz, log_freq
        self.enable_log_data, self.log_dir = enable_log_data, log_dir
        self.enable_camera, self.camera_fps = enable_camera, camera_fps
        self.camera_interface = None
        self._start_time = 0
        self._prev_b_button_state = self._is_logging = False
        self._stop_event = threading.Event()
        self._state_lock = threading.RLock()
        self._closed = False
        self.telemetry = self.xr_client = None
        self.arm_controllers = {}
        for field in ("ref_ee_xyz", "ref_ee_quat", "ref_controller_xyz", "ref_controller_quat"):
            setattr(self, field, {name: None for name in manipulator_config})
        for field in ("effector_task", "effector_control_mode", "active", "gripper_pos_target",
                      "motion_tracker_task", "ref_tracker_xyz", "ref_robot_xyz"):
            setattr(self, field, {})
        for name, config in manipulator_config.items():
            if "gripper_config" in config:
                grip = config["gripper_config"]
                self.gripper_pos_target[name] = dict(zip(grip["joint_names"], grip["open_pos"]))
        registry = Registry()
        # Validate all names before constructing any device.
        input_factory = registry.load("input", input_plugin)
        telemetry_factory = registry.load("telemetry", telemetry)
        try:
            if enable_log_data:
                from xrobotoolkit_teleop.common.data_logger import DataLogger
                self.data_logger = DataLogger(log_dir=log_dir)
            backend = getattr(self, "_backend_name", "stanford")
            if not getattr(self, "_service_mode", False):
                self.telemetry = telemetry_factory(robot=RobotModel(robot_urdf_path), source={
                    "kind": "hardware" if registry.spec("arm", backend).hardware else "virtual",
                    "backend": backend, "input": input_plugin, "controller": type(self).__name__,
                })
            self.xr_client = input_factory()
            from .inputs import InputFreshness
            timestamp = getattr(self.xr_client, "get_timestamp_ns", None)
            self._input_freshness = InputFreshness(timestamp) if callable(timestamp) else None
            self._input_ready = self._input_freshness is None
            self._robot_setup()
            self._placo_setup()
            self._update_robot_state()
            self.placo_robot.update_kinematics()
            self.sync_end_effector_poses_to_placo_tasks()
        except BaseException:
            self.close()
            raise

    def _ik_thread(self, stop_event):
        try:
            while not stop_event.is_set():
                start = time.monotonic()
                with self._state_lock:
                    self._input_ready = False
                    if getattr(self, "_input_paused", False):
                        ready = False
                    elif getattr(self, "_resume_pending", False):
                        ready = self._resume_input_ready()
                    else:
                        ready = self._input_freshness is None or self._input_freshness.ready()
                    if ready:
                        processing_started = time.monotonic_ns()
                        self._update_robot_state()
                        self._update_gripper_target()
                        self._pre_ik_update()
                        self._update_ik()
                        processing_finished = time.monotonic_ns()
                        self._input_timing = {"sample_monotonic_ns": processing_finished,
                                              "duration_ms": (processing_finished - processing_started) / 1e6}
                        self._input_ready = True
                stop_event.wait(max(0., self.dt - (time.monotonic() - start)))
        except Exception as exc:
            self._thread_error = exc
            raise
        finally:
            stop_event.set()
            self._stop_event.set()

    def _control_thread(self, stop_event):
        try:
            while not stop_event.is_set():
                start = time.monotonic()
                with self._state_lock:
                    if self._input_ready:
                        self._send_command()
                stop_event.wait(max(0., self.dt - (time.monotonic() - start)))
        except Exception as exc:
            self._thread_error = exc
            raise
        finally:
            stop_event.set()
            self._stop_event.set()
            self._shutdown_robot()

    def _should_keep_running(self):
        if getattr(self, "_service_mode", False):
            # Probe on the main thread too: a failed IK/control thread must not
            # leave the application running as if its service were still connected.
            next(iter(self.arm_controllers.values())).get_joint_positions()
            return not self._stop_event.is_set()
        # Copy state into the publisher buffer; no rendering or network waits here.
        with self._state_lock:
            name, arm = next(iter(self.arm_controllers.items()))
            actual = arm.get_joint_positions()[:6].copy()
            target = self.placo_robot.state.q[self.placo_arm_joint_slice[name]].copy()
            if getattr(self, "_preset_target", None) is not None:
                target = self._preset_target.copy()
        self.telemetry.update(actual, target)
        return not self._stop_event.is_set()

    def close(self):
        if self._closed:
            return
        self._closed = True
        try:
            self._shutdown_robot()
        finally:
            # Close every created component, including after partial construction.
            from contextlib import ExitStack
            with ExitStack() as cleanup:
                for component in [self.telemetry, self.xr_client, *self.arm_controllers.values()]:
                    close = getattr(component, "close", None)
                    if close is not None:
                        cleanup.callback(close)

    def run(self):
        try:
            if getattr(self, "_service_mode", False):
                print("Control client ready: connected to arm-service", flush=True)
            super().run()
            error = getattr(self, "_thread_error", None)
            if error is not None:
                raise RuntimeError(f"Teleoperation worker failed: {error}") from error
        finally:
            self.close()

    def _placo_setup(self):
        super()._placo_setup()
        self.placo_arm_joint_slice: Dict[str, slice] = {}
        for arm_name, config in self.manipulator_config.items():
            ee_link_name = config["link_name"]
            arm_prefix = ee_link_name.replace("link6", "")
            arm_joint_names = [f"{arm_prefix}joint{i}" for i in range(1, 7)]
            self.placo_arm_joint_slice[arm_name] = slice(
                self.placo_robot.get_joint_offset(arm_joint_names[0]),
                self.placo_robot.get_joint_offset(arm_joint_names[-1]) + 1,
            )

    def _initialize_camera(self):
        if self.enable_camera:
            from xrobotoolkit_teleop.hardware.interface.realsense import RealSenseCameraInterface
            print("Initializing camera...")
            try:
                self.camera_interface = RealSenseCameraInterface(
                    width=self.camera_width,
                    height=self.camera_height,
                    fps=self.camera_fps,
                    serial_numbers=list(self.camera_serial_dict.values()),
                    enable_depth=self.enable_camera_depth,
                    enable_compression=self.enable_camera_compression,
                    jpg_quality=self.camera_jpg_quality,
                )
                self.camera_interface.start()
                print("Camera initialized successfully.")
            except Exception as e:
                print(f"Error initializing camera: {e}")
                self.camera_interface = None

    def _update_robot_state(self):
        """Reads current joint states from the arms and updates Placo."""
        for arm_name, controller in self.arm_controllers.items():
            q_slice = self.placo_arm_joint_slice[arm_name]
            self.placo_robot.state.q[q_slice] = controller.get_joint_positions()[:6]

    def _bounded_ik_command(self, arm_name):
        """Saturate normal IK requests at joint limits without dropping control."""
        slot = self.placo_arm_joint_slice[arm_name]
        requested = self.placo_robot.state.q[slot].copy()
        if requested.shape != (6,) or not np.isfinite(requested).all():
            raise ValueError("IK returned invalid joint data")
        config = self.arm_controllers[arm_name].robot_config
        bounded = np.clip(requested, config.joint_pos_min, config.joint_pos_max)
        indices = np.flatnonzero(np.abs(requested - bounded) > 1e-8)
        self._ik_limit_report = {"sample_monotonic_ns": time.monotonic_ns(),
                                 "joints": [int(i) + 1 for i in indices]}
        self.placo_robot.state.q[slot] = bounded
        return bounded

    def _send_command(self):
        """Sends the solved joint targets to the hardware controllers."""
        self._ik_limit_report = None
        for arm_name, controller in self.arm_controllers.items():
            if self.active.get(arm_name, False):
                q_des = self._bounded_ik_command(arm_name)
                controller.set_joint_positions(q_des)

            if "gripper_config" in self.manipulator_config[arm_name]:
                gripper_config = self.manipulator_config[arm_name]["gripper_config"]
                joint_name = gripper_config["joint_names"][0]
                gripper_target = self.gripper_pos_target[arm_name][joint_name]
                controller.set_catch_pos(gripper_target)

    def _get_robot_state_for_logging(self) -> Dict:
        """Returns a dictionary of robot-specific data for logging."""
        return {
            "qpos": {arm: c.get_joint_positions() for arm, c in self.arm_controllers.items()},
            "qvel": {arm: c.get_joint_velocities() for arm, c in self.arm_controllers.items()},
            "qpos_des": {
                arm: self.placo_robot.state.q[self.placo_arm_joint_slice[arm]].copy()
                for arm in self.arm_controllers
            },
            "gripper_target": {
                arm: (
                    self.gripper_pos_target[arm].copy()
                    if "gripper_config" in self.manipulator_config[arm]
                    else None
                )
                for arm in self.arm_controllers
            },
        }

    def _get_camera_frame_for_logging(self) -> Dict:
        """Returns a dictionary of camera frames for logging with camera names as keys."""
        if not self.camera_interface:
            return {}

        # Use compressed frames for logging to reduce file size
        if self.camera_interface.enable_compression:
            frames_by_serial = self.camera_interface.get_compressed_frames()
        else:
            # Fallback to regular frames for compatibility
            frames_by_serial = self.camera_interface.get_frames()

        if not frames_by_serial:
            return {}

        # Convert from serial number keys to camera name keys
        frames_by_name = {}
        for serial, frames in frames_by_serial.items():
            camera_name = self.camera_serial_to_name.get(
                serial, serial
            )  # Fallback to serial if name not found
            frames_by_name[camera_name] = frames

        return frames_by_name
