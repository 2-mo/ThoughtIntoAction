"""Project-owned Quest X5 controller; importing it never creates a CAN controller."""

import signal
import time
from copy import deepcopy

from robot_core.project import X5_2025_URDF
from robot_core.plugins import Registry

from .base_controller import X5TeleopBase


class X5TeleopController(X5TeleopBase):
    """Project X5 controller: shared backend, bounded commands and recording session."""

    def __init__(
        self,
        *args,
        actuate: bool,
        preview_time_s: float = .03,
        gripper_open_readout: float = -3.4,
        gripper_width: float = .082,
        max_joint_speed_rad_s: float = .5,
        joint_soft_limit_margin_rad: float = .05,
        gripper_force_hold: bool = True,
        gripper_contact_torque: float = 0.60,
        gripper_hold_torque: float = 0.30,
        gripper_contact_cycles: int = 1,
        gripper_release_margin_m: float = 0.004,
        dynamics_urdf_path: str = str(X5_2025_URDF),
        backend: str = "service",
        service_socket: str | None = None,
        initial_q=None,
        simulation_mode: bool = False,
        enable_session: bool = False,
        **kwargs,
    ) -> None:
        registry = Registry()
        self._service_mode = backend == "service"
        self._service_socket = service_socket
        self._simulation_mode = simulation_mode
        self._input_name = kwargs.get("input_plugin", "quest")
        spec = registry.spec("arm", backend, allow_hardware=not simulation_mode)
        input_spec = registry.spec("input", kwargs.get("input_plugin", "quest"))
        registry.spec("telemetry", kwargs.get("telemetry", "none"))
        if (simulation_mode or spec.simulation_only) and actuate:
            raise ValueError("Simulation does not accept --actuate")
        if (spec.hardware or (self._service_mode and not simulation_mode)) and input_spec.simulation_only:
            raise ValueError("A simulation-only input cannot drive a hardware backend")
        if enable_session and (simulation_mode or "recording" not in spec.capabilities):
            raise ValueError(f"Backend {backend!r} does not support real-data recording")
        if spec.hardware and initial_q is not None:
            raise ValueError("initial_q is only available for simulation")
        if "experimental" in spec.capabilities and actuate:
            raise ValueError("Experimental ARX position actuation remains disabled")
        self._arm_factory = registry.load("arm", backend, allow_hardware=not simulation_mode)
        self._initial_q = initial_q
        self._backend_name = backend
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
        self._enable_session = enable_session
        self._session = None
        self._input_paused = False
        self._resume_pending = False
        self._session_control = {
            "actuate": actuate, "control_rate_hz": self._x5_control_rate_hz,
            "preview_time_s": preview_time_s, "max_joint_speed_rad_s": max_joint_speed_rad_s,
            "joint_soft_limit_margin_rad": joint_soft_limit_margin_rad,
            "gripper_width_m": gripper_width, "gripper_open_readout": gripper_open_readout,
            "gripper_force_hold": gripper_force_hold, "gripper_contact_torque": gripper_contact_torque,
            "gripper_hold_torque": gripper_hold_torque, "scale_factor": kwargs["scale_factor"],
            "manipulator_config": deepcopy(kwargs["manipulator_config"]),
            "backend": backend, "input_plugin": kwargs.get("input_plugin", "quest"),
            "telemetry": kwargs.get("telemetry", "none"), "controller": type(self).__name__,
        }
        super().__init__(*args, **kwargs)

    def _robot_setup(self) -> None:
        if getattr(self, "_x5_setup_done", False):
            return
        for name, port in self.can_ports.items():
            settings = {
                "can_port": port,
                "actuate": self._x5_actuate,
                "preview_time_s": self._x5_preview_time_s,
                "gripper_open_readout": self._x5_gripper_open_readout,
                "gripper_width": self._x5_gripper_width,
                "control_rate_hz": self._x5_control_rate_hz,
                "max_joint_speed_rad_s": self._x5_max_joint_speed_rad_s,
                "joint_soft_limit_margin_rad": self._x5_joint_soft_limit_margin_rad,
                "gripper_force_hold": self._x5_gripper_force_hold,
                "gripper_contact_torque": self._x5_gripper_contact_torque,
                "gripper_hold_torque": self._x5_gripper_hold_torque,
                "gripper_contact_cycles": self._x5_gripper_contact_cycles,
                "gripper_release_margin_m": self._x5_gripper_release_margin_m,
                "dynamics_urdf_path": self._x5_dynamics_urdf_path,
            }
            if self._service_mode:
                settings.update(service_socket=self._service_socket, hardware=not self._simulation_mode,
                                actuate=self._x5_actuate or self._simulation_mode, input_name=self._input_name)
            self.arm_controllers[name] = self._arm_factory(settings=settings, initial_q=self._initial_q)
            if self._service_mode:
                arm = self.arm_controllers[name]
                self._session_control.update(
                    service_settings=dict(arm.settings),
                    requested_max_joint_speed_rad_s=self._x5_max_joint_speed_rad_s,
                    max_joint_speed_rad_s=self._x5_max_joint_speed_rad_s,
                    gripper_width_m=arm.settings["gripper_width"],
                    gripper_open_limit_m=arm.settings["gripper_open_limit_m"],
                    gripper_open_readout=arm.settings["gripper_open_readout"],
                    preview_time_s=arm.settings["preview_time_s"],
                    joint_soft_limit_margin_rad=arm.settings["joint_soft_limit_margin_rad"],
                )
                self._session_control.update(backend=arm.info["backend"], transport="arm-service",
                                             service_id=arm.service_id)
                self.manipulator_config[name]["gripper_config"]["open_pos"] = [arm.settings["gripper_open_limit_m"]]
                self.gripper_pos_target[name]["joint7"] = float(arm.get_joint_positions()[6])
        self._session_control["manipulator_config"] = deepcopy(self.manipulator_config)
        self._x5_setup_done = True
        mode = "ACTUATION ENABLED" if self._x5_actuate else "DAMPING / VIRTUAL"
        print(f"Quest X5 controller: {mode}")
        print("Automatic homing is disabled.")

    def _shutdown_robot(self) -> None:
        for controller in self.arm_controllers.values():
            try:
                controller.set_to_damping()
            except RuntimeError as exc:
                print(f"Control release: {exc}")
        print("X5 control release requested; arm service continues feedback."
              if getattr(self, "_service_mode", False) else "X5 damping requested.")

    def _send_command(self) -> None:
        super()._send_command()
        self._publish_runtime()

    def _publish_runtime(self, preset_name=None) -> None:
        if self._session is None:
            return
        adapter = self.arm_controllers["right_arm"]
        packet = adapter.get_recording_snapshot()
        packet["source"] = ("preset" if preset_name else self._session_control["input_plugin"]
                            if self.active.get("right_arm", False) else "hold")
        packet["preset_name"] = preset_name
        packet["timing"] = {
            "arm_command": getattr(adapter, "command_timing", None) if getattr(self, "_service_mode", False) else None,
            "input_processing": getattr(self, "_input_timing", None) if self._input_name == "quest" else None,
        }
        packet["joint_limit"] = getattr(self, "_ik_limit_report", None)
        if self._input_name == "quest":
            # Sample diagnostics at 10 Hz; never let optional display data stop control.
            now = time.monotonic_ns()
            previous = getattr(self, "_monitor_input", None)
            if previous is None or now - previous["sample_monotonic_ns"] >= 100_000_000:
                from .inputs import controller_snapshot
                self._monitor_input = controller_snapshot(self.xr_client, now)
            packet["input"] = self._monitor_input
        self._session.publish(packet)

    def set_input_paused(self, paused):
        """Session RPC thread; serialize against IK and command submission."""
        if not self._service_mode:
            raise RuntimeError("Input pause requires the arm-service backend")
        with self._state_lock:
            self._input_ready = False
            self._input_paused = True
            self._resume_pending = False
            for arm in self.arm_controllers.values():
                arm.set_to_damping()
            cancel = getattr(self, "_cancel_preset", None)
            if cancel:
                cancel("Headset pause")
            for name in self.active:
                self.active[name] = False
                for field in ("ref_ee_xyz", "ref_ee_quat", "ref_controller_xyz", "ref_controller_quat"):
                    getattr(self, field)[name] = None
            if not paused:
                from .inputs import InputFreshness
                self._input_freshness = InputFreshness(self.xr_client.get_timestamp_ns)
                self._resume_pending = True
                self._input_paused = False
            return {"input_paused": self._input_paused, "waiting_for_fresh_input": self._resume_pending}

    def _resume_input_ready(self):
        """Wait for a new stream and released buttons before rebuilding anchors."""
        from .inputs import InputFreshness
        try:
            ready = self._input_freshness.ready()
        except RuntimeError:
            self._input_freshness = InputFreshness(self.xr_client.get_timestamp_ns)
            return False
        if not ready:
            return False
        if any(self.xr_client.get_key_value_by_name(c["control_trigger"]) > .9
               for c in self.manipulator_config.values()):
            return False
        if any(self.xr_client.get_button_state_by_name(b) for b in ("A", "B", "right_axis_click")):
            return False
        self._update_robot_state()
        self.placo_robot.update_kinematics()
        self.sync_end_effector_poses_to_placo_tasks()
        reset_buttons = getattr(self, "_reset_buttons", None)
        if reset_buttons:
            reset_buttons()
        self._resume_pending = False
        return True

    def run(self) -> None:
        previous_handler = signal.getsignal(signal.SIGTERM)
        def interrupted(signum, frame):
            raise KeyboardInterrupt("SIGTERM")
        signal.signal(signal.SIGTERM, interrupted)
        try:
            if self._enable_session:
                from robot_core.project import load_config
                from robot_core.model import RobotModel
                from quest_teleop.runtime.session import TeleopSession
                model = RobotModel(self._x5_dynamics_urdf_path)
                self._session = TeleopSession(load_config(), model.metadata(), self._session_control)
                self._session.input_pause = self.set_input_paused
                if hasattr(self, "_collection"):
                    self._session.collection_status = self._collection_status
                self._session.start()
                print(f"Local session: {self._session.socket_path}; use ./run.sh session status / record start")
            super().run()
        finally:
            try:
                if self._session is not None:
                    self._session.close()
            finally:
                try:
                    self.close()
                finally:
                    signal.signal(signal.SIGTERM, previous_handler)
