"""Project-owned Quest X5 controller; importing it never creates a CAN controller."""

import signal
import time
import numpy as np
from copy import deepcopy

from embodied.devices.robots.arx5.paths import X5_2025_URDF
from embodied.contracts.registry import Registry

from legacy.quest_teleop.base_controller import X5TeleopBase
from legacy.quest_teleop.arming import DownHoldGate
from legacy.quest_teleop.frames import R_CONTROLLER_TO_TOOL, R_CONTROLLER_TO_DOWN_TOOL


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
        max_command_lead_rad: float = .15,
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
        teleop_preview_time_s: float = .01,
        max_joint_acceleration_rad_s2: float = 2.,
        max_joint_jerk_rad_s3: float = 10.,
        home_speed_rad_s: float = .3,
        home_final_speed_rad_s: float = 0.,
        home_acceleration_rad_s2: float = .5,
        home_jerk_rad_s3: float = 2.,
        preset_speed_rad_s: float = .5,
        preset_acceleration_rad_s2: float = 1.,
        preset_jerk_rad_s3: float = 4.,
        **kwargs,
    ) -> None:
        self._trajectory_settings = dict(teleop_preview_time_s=teleop_preview_time_s, max_joint_acceleration_rad_s2=max_joint_acceleration_rad_s2,
            max_joint_jerk_rad_s3=max_joint_jerk_rad_s3, home_speed_rad_s=home_speed_rad_s,
            home_final_speed_rad_s=home_final_speed_rad_s,
            home_acceleration_rad_s2=home_acceleration_rad_s2, home_jerk_rad_s3=home_jerk_rad_s3,
            preset_speed_rad_s=preset_speed_rad_s, preset_acceleration_rad_s2=preset_acceleration_rad_s2,
            preset_jerk_rad_s3=preset_jerk_rad_s3)
        import math
        if not all(type(v) in (int, float) and math.isfinite(v)
                   and (v > 0 or (key in {"preset_speed_rad_s", "home_final_speed_rad_s"} and v == 0))
                   for key, v in self._trajectory_settings.items()):
            raise ValueError("Trajectory limits must be finite and positive; preset speed 0 uses the session limit, final Home speed 0 disables staging")
        registry = Registry()
        self._service_mode = backend == "service"
        self._service_socket = service_socket
        self._simulation_mode = simulation_mode
        self._input_name = kwargs.get("input_plugin", "quest")
        self._control_enabled = self._input_name != "quest"
        self._activation_gate = DownHoldGate()
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
        self._x5_max_command_lead_rad = max_command_lead_rad
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
            "trajectory_settings": self._trajectory_settings,
            "actuate": actuate, "control_rate_hz": self._x5_control_rate_hz,
            "preview_time_s": preview_time_s, "max_joint_speed_rad_s": max_joint_speed_rad_s,
            "max_command_lead_rad": max_command_lead_rad,
            "joint_soft_limit_margin_rad": joint_soft_limit_margin_rad,
            "gripper_width_m": gripper_width, "gripper_open_readout": gripper_open_readout,
            "gripper_force_hold": gripper_force_hold, "gripper_contact_torque": gripper_contact_torque,
            "gripper_hold_torque": gripper_hold_torque, "scale_factor": kwargs["scale_factor"],
            "operator_yaw_deg": kwargs.get("operator_yaw_deg", 0.0),
            "rotation_frame": kwargs.get("rotation_frame", "base"),
            "max_eef_linear_speed_m_s": kwargs.get("max_eef_linear_speed_m_s", .25),
            "max_eef_angular_speed_rad_s": kwargs.get("max_eef_angular_speed_rad_s", 0.0),
            "manipulator_config": deepcopy(kwargs["manipulator_config"]),
            "backend": backend, "input_plugin": kwargs.get("input_plugin", "quest"),
            "telemetry": kwargs.get("telemetry", "none"), "controller": type(self).__name__,
        }
        super().__init__(*args, **kwargs)
        self._session_control["quest_to_base_rotation"] = self.R_headset_world.tolist()
        if self.rotation_frame == "tool":
            self._session_control["controller_to_tool_rotation"] = R_CONTROLLER_TO_TOOL.tolist()
        elif self.rotation_frame == "aligned":
            self._session_control["controller_to_tool_rotation"] = R_CONTROLLER_TO_DOWN_TOOL.tolist()
            self._session_control["rotation_alignment_reference"] = "controller_forward_gripper_down"
            self._session_control["position_frame"] = "aligned_horizontal"
            print("位姿参考：首次握住 Grip 前，先让夹爪处于朝下的预抓取姿态，手柄水平朝前。平移与旋转一起校准，上下保持竖直；后续松握 Grip 时保留参考。", flush=True)
        elif self.rotation_frame == "calibrated":
            self._session_control["position_frame"] = "measured_quest_to_base"
            print("请在待机状态输入 c，按提示移动手柄标定方向；标定后才可启用控制。控制点为夹指尖端中心（URDF eef_link）。", flush=True)
        heading = "measured by c calibration" if self.rotation_frame == "calibrated" else "calibrated at first Grip (fixed yaw unused)" if self.rotation_frame == "aligned" else f"{self.operator_yaw_deg:+g} deg about base Z"
        print(f"Operator heading: {heading}; rotation frame: {self.rotation_frame}", flush=True)
        if not self._control_enabled:
            print("待机：不发送机械臂或夹爪命令。松开扳机和按键，右摇杆先回中，再持续下拨 1 秒启用控制；每次切换后需回中。", flush=True)

    def _control_gate_ready(self):
        if self._input_name != "quest":
            return True
        released = not any(self.xr_client.get_button_state_by_name(b)
                           for b in ("A", "B", "right_axis_click"))
        released = released and all(self.xr_client.get_key_value_by_name(key) < .1
                                    for key in ("right_grip", "right_trigger"))
        if self._activation_gate.update(self.xr_client.get_joystick_state("right"),
                                        time.monotonic(), released=released):
            if self.rotation_frame == "calibrated" and any(basis is None for basis in self._rotation_alignment.values()):
                print("尚未完成方向标定；请在遥操终端输入 c。", flush=True)
                return False
            if self._control_enabled:
                self._control_enabled = self._input_ready = False
                cancel = getattr(self, "_cancel_preset", None)
                if cancel:
                    cancel("Control standby")
                for name, arm in self.arm_controllers.items():
                    self.active[name] = False
                    for field in ("ref_ee_xyz", "ref_ee_quat", "ref_controller_xyz", "ref_controller_quat"):
                        getattr(self, field)[name] = None
                    arm.stop_control(release=True)
                print("正在平滑停车，随后保持待机；不发送机械臂或夹爪命令。右摇杆回中再下拨 1 秒启用控制。", flush=True)
                self._publish_runtime()
                return False
            for arm in self.arm_controllers.values():
                arm.snapshot()
                if arm.info.get("mode") == "stopping":
                    print("仍在平滑停车，停稳后右摇杆回中再下拨 1 秒启用。", flush=True)
                    return False
            self._update_robot_state()
            self.placo_robot.update_kinematics()
            self.sync_end_effector_poses_to_placo_tasks()
            for name, arm in self.arm_controllers.items():
                self.active[name] = False
                for field in ("ref_ee_xyz", "ref_ee_quat", "ref_controller_xyz", "ref_controller_quat"):
                    getattr(self, field)[name] = None
                config = self.manipulator_config[name].get("gripper_config")
                if config:
                    self.gripper_pos_target[name][config["joint_names"][0]] = float(arm.get_joint_positions()[6])
            reset = getattr(self, "_reset_buttons", None)
            if reset:
                reset()
            self._control_enabled = True
            print("控制已启用；握住 Grip 开始遥操，右摇杆回中再下拨 1 秒返回待机。", flush=True)
            self._publish_runtime()
            return False
        if not self._control_enabled:
            self._publish_runtime()
        return self._control_enabled

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
                "max_command_lead_rad": self._x5_max_command_lead_rad,
                "joint_soft_limit_margin_rad": self._x5_joint_soft_limit_margin_rad,
                "gripper_force_hold": self._x5_gripper_force_hold,
                "gripper_contact_torque": self._x5_gripper_contact_torque,
                "gripper_hold_torque": self._x5_gripper_hold_torque,
                "gripper_contact_cycles": self._x5_gripper_contact_cycles,
                "gripper_release_margin_m": self._x5_gripper_release_margin_m,
                "dynamics_urdf_path": self._x5_dynamics_urdf_path,
            }
            if self._service_mode:
                settings.update(trajectory_settings=self._trajectory_settings, service_socket=self._service_socket, hardware=not self._simulation_mode,
                                actuate=self._x5_actuate or self._simulation_mode, input_name=self._input_name)
            self.arm_controllers[name] = self._arm_factory(settings=settings, initial_q=self._initial_q)
            if self._service_mode:
                arm = self.arm_controllers[name]
                self._session_control.update(
                    service_settings=dict(arm.settings),
                    requested_max_joint_speed_rad_s=self._x5_max_joint_speed_rad_s,
                    effective_joint_speed_limits_rad_s=[v if self._x5_max_joint_speed_rad_s == 0 else min(v, self._x5_max_joint_speed_rad_s)
                                                       for v in arm.info["hardware_joint_speed_limits_rad_s"]],
                    max_joint_speed_rad_s=self._x5_max_joint_speed_rad_s,
                    gripper_width_m=arm.settings["gripper_width"],
                    gripper_open_limit_m=arm.settings["gripper_open_limit_m"],
                    gripper_open_readout=arm.settings["gripper_open_readout"],
                    preview_time_s=self._x5_preview_time_s,
                    max_command_lead_rad=self._x5_max_command_lead_rad,
                    joint_soft_limit_margin_rad=self._x5_joint_soft_limit_margin_rad,
                )
                from embodied.core.safety import TablePlane
                safety = arm.info.get("safety")
                if safety is None:
                    raise RuntimeError("机械臂服务缺少统一安全配置，请重启机械臂服务")
                self._table_plane = TablePlane(safety["table_z_m"] if safety["table_enabled"] else None,
                                               safety["table_clearance_m"])
                self._session_control["arm_safety"] = dict(safety)
                self._session_control.update(backend=arm.info["backend"], transport="arm-service",
                                             service_id=arm.service_id)
                self.manipulator_config[name]["gripper_config"]["open_pos"] = [arm.settings["gripper_open_limit_m"]]
                self.gripper_pos_target[name]["joint7"] = float(arm.get_joint_positions()[6])
        self._session_control["manipulator_config"] = deepcopy(self.manipulator_config)
        self._x5_setup_done = True
        mode = "ACTUATION AUTHORIZED / STANDBY" if self._x5_actuate else "DAMPING / VIRTUAL"
        print(f"Quest X5 controller: {mode}")
        print("Automatic homing is disabled.")
        print(f"Motion settings: speed={self._x5_max_joint_speed_rad_s:g} rad/s (0=SDK limits), "
              f"lead={self._x5_max_command_lead_rad:g} rad (0=off), "
              f"preview={self._x5_preview_time_s:g}s, margin={self._x5_joint_soft_limit_margin_rad:g} rad")

    def _shutdown_robot(self) -> None:
        for controller in self.arm_controllers.values():
            try:
                if getattr(self, '_service_mode', False):
                    controller.stop_control(release=True)
                else:
                    controller.set_to_damping()
            except RuntimeError as exc:
                print(f"Control release: {exc}")
        print("X5 stop-and-hold requested; arm service continues feedback."
              if getattr(self, "_service_mode", False) else "X5 damping requested.")

    def _send_command(self) -> None:
        if not self._control_enabled:
            self._publish_runtime()
            return
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
        packet["control_enabled"] = self._control_enabled
        if self._table_plane.minimum_z is not None:
            packet["table_safety"] = {"minimum_z_m": self._table_plane.minimum_z,
                "blocked": bool((packet.get("table_safety") or {}).get("blocked", False))}
        replay = getattr(self, "_robot_replay", None)
        feedback = getattr(self, "button_feedback", None)
        packet["button_feedback"] = feedback() if feedback else None
        packet["quest_to_base_rotation"] = self.R_headset_world.tolist()
        packet["robot_replay"] = replay.status() if replay else {
            "phase": "loading" if getattr(self, "_replay_loading", False) else "idle",
            "error": getattr(self, "_replay_error", None)}
        packet["loop_rates"] = {**packet.get("loop_rates", {}), "ik": self._ik_rate.snapshot()}
        packet["timing"] = {
            "arm_command": getattr(adapter, "command_timing", None) if getattr(self, "_service_mode", False) else None,
            "input_processing": getattr(self, "_input_timing", None) if self._input_name == "quest" else None,
        }
        packet["joint_limit"] = getattr(self, "_ik_limit_report", None)
        if self.rotation_frame in {"aligned", "calibrated"}:
            # Snapshot the map: a later pause/calibration must not rewrite an
            # already queued recording sample's interpretation of orientation.
            packet["rotation_alignment"] = dict(self._rotation_alignment)
            packet["position_alignment"] = dict(self._position_alignment)
        if self._input_name == "quest":
            # Sample display diagnostics at 10 Hz.
            now = time.monotonic_ns()
            previous = getattr(self, "_monitor_input", None)
            if previous is None or now - previous["sample_monotonic_ns"] >= 100_000_000:
                from legacy.quest_teleop.inputs import controller_snapshot
                self._monitor_input = controller_snapshot(self.xr_client, now)
            packet["input"] = self._monitor_input
        # Read-only orientation comparison: use the actual Cartesian IK task,
        # rather than reconstructing a desired pose from joint commands.
        basis = self._rotation_alignment.get("right_arm")
        task = self.effector_task.get("right_arm")
        packet["orientation_preview"] = {
            "basis": (basis if self.rotation_frame in {"aligned", "calibrated"}
                      else self.R_headset_world.tolist() if self.rotation_frame == "base" else None),
            "mode": self.rotation_frame,
            "input_ready": not (getattr(self, "_input_paused", False) or getattr(self, "_resume_pending", False)),
            "active": bool(self._control_enabled and self.active.get("right_arm", False)),
            "target": np.asarray(task.T_world_frame).flatten(order="F").tolist()
                      if task is not None and hasattr(task, "T_world_frame") else None,
        }
        self._session.publish(packet)

    def set_input_paused(self, paused):
        """Session RPC thread; serialize against IK and command submission."""
        if not self._service_mode:
            raise RuntimeError("Input pause requires the arm-service backend")
        with self._state_lock:
            self._input_ready = False
            self._control_enabled = False
            self._activation_gate.reset()
            self.reset_rotation_alignment()
            if self.rotation_frame == "calibrated" and not paused:
                print("恢复后请在待机状态输入 c 重新标定方向。", flush=True)
            if self.rotation_frame == "aligned" and not paused:
                print("恢复后需重建位姿参考：先让夹爪回到朝下的预抓取姿态，手柄水平朝前，再握住 Grip。", flush=True)
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
                from legacy.quest_teleop.inputs import InputFreshness
                self._input_freshness = InputFreshness(self.xr_client.get_timestamp_ns)
                self._resume_pending = True
                self._input_paused = False
            return {"input_paused": self._input_paused, "waiting_for_fresh_input": self._resume_pending}

    def _resume_input_ready(self):
        """Wait for a new stream and released buttons before rebuilding anchors."""
        from legacy.quest_teleop.inputs import InputFreshness
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

    def set_frame_calibration(self, rotation, session_id):
        """Apply a measured coordinate map only while the arm is in standby."""
        from legacy.quest_teleop.frame_calibration import validate_rotation
        basis = validate_rotation(rotation).tolist()
        with self._state_lock:
            if self.rotation_frame not in {"base", "calibrated"}:
                raise RuntimeError("快速对齐需要 base 坐标模式")
            if self._session is None or self._session.id != session_id:
                raise RuntimeError("遥操会话已改变，请重新标定")
            if self._control_enabled or self._input_paused or self._resume_pending:
                raise RuntimeError("标定需要追踪正常的待机状态")
            if self._session.recording is not None:
                raise RuntimeError("请先结束录制再标定")
            for arm in self.arm_controllers.values():
                arm.snapshot()
                if arm.info.get("mode") == "stopping":
                    raise RuntimeError("请等待机械臂停稳再标定")
            if self.rotation_frame == "base":
                from legacy.quest_teleop.inputs import controller_snapshot
                from legacy.quest_teleop.frame_calibration import validate_standby_status
                validate_standby_status({"session_id": session_id, "state_age_s": 0.,
                    "recording": None, "state": {"control_enabled": False,
                    "input": controller_snapshot(self.xr_client, time.monotonic_ns())}})
                if not self._input_freshness.ready():
                    raise RuntimeError("请等待 Quest 数据恢复")
                self.R_headset_world = np.asarray(basis)
            self._session_control["quest_to_base_rotation"] = basis
            for name in self.manipulator_config:
                self._rotation_alignment[name] = basis
                self._position_alignment[name] = basis
                for field in ("ref_ee_xyz", "ref_ee_quat", "ref_controller_xyz", "ref_controller_quat"):
                    getattr(self, field)[name] = None
            self._publish_runtime()
            return {"rotation": basis, "session_id": session_id}

    def run(self) -> None:
        previous_handler = signal.getsignal(signal.SIGTERM)
        def interrupted(signum, frame):
            raise KeyboardInterrupt("SIGTERM")
        signal.signal(signal.SIGTERM, interrupted)
        try:
            if self._enable_session:
                from embodied.core.project import load_config
                from embodied.devices.robots.arx5.pinocchio_model import X5PinocchioModel
                from legacy.quest_teleop.runtime.session import TeleopSession
                model = X5PinocchioModel(self._x5_dynamics_urdf_path, eef=self.manipulator_config["right_arm"]["link_name"])
                if self._service_mode:
                    arm = self.arm_controllers["right_arm"]
                    if arm.info.get("sample_protocol") != 1:
                        raise RuntimeError("逐周期录制需要新版机械臂服务，请重启 scripts/shortcuts/启动机械臂服务.sh")
                    self._session_control["recording_rate_hz"] = arm.info["settings"]["control_rate_hz"]
                self._session = TeleopSession(load_config(), model.metadata(), self._session_control)
                self._session.input_pause = self.set_input_paused
                self._session.frame_calibration = self.set_frame_calibration
                if hasattr(self, "_collection"):
                    self._session.collection_status = self._collection_status
                if self._service_mode:
                    self._session.record_samples = arm.recording_samples
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
