"""Explicit hardware backend using one Stanford SDK exchange per 4 ms tick.

Importing this module or calling ``preflight`` never opens a CAN socket.
Constructing ``RealBackend`` does: the SDK enables motors during construction.
"""

import importlib
from importlib.metadata import PackageNotFoundError, version
import platform
import os
import time
from copy import deepcopy
from dataclasses import asdict
from pathlib import Path

import numpy as np
from .types import ControlCommand, Mode, RobotConfig, RobotState


_ROBOT_CONFIG_FIELDS = (
    "robot_model", "joint_pos_min", "joint_pos_max", "joint_vel_max",
    "joint_torque_max", "ee_vel_max", "gripper_vel_max", "gripper_torque_max",
    "gripper_width", "gripper_open_readout", "joint_dof", "motor_id", "motor_type",
    "gripper_motor_id", "gripper_motor_type", "gravity_vector", "base_link_name",
    "eef_link_name", "urdf_path",
)
_CONTROLLER_CONFIG_FIELDS = (
    "controller_type", "default_kp", "default_kd", "default_gripper_kp",
    "default_gripper_kd", "over_current_cnt_max", "controller_dt",
    "gravity_compensation", "background_send_recv", "shutdown_to_passive",
    "interpolation_method", "default_preview_time",
)


def _json_value(value):
    """Copy SDK configuration values without exposing live NumPy storage."""
    if isinstance(value, np.ndarray):
        return _json_value(value.tolist())
    if isinstance(value, np.generic):
        return _json_value(value.item())
    if isinstance(value, dict):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_value(item) for item in value]
    if isinstance(value, float):
        return value if np.isfinite(value) else None
    if value is None or isinstance(value, (str, int, bool)):
        return value
    # SDK MotorType is a pybind enum, not Python's enum.Enum.
    return str(value)


def _config_snapshot(config, fields):
    result = {}
    unavailable = []
    for name in fields:
        try:
            result[name] = _json_value(getattr(config, name))
        except (AttributeError, RuntimeError, TypeError, ValueError):
            result[name] = None
            unavailable.append(name)
    result["unavailable_fields"] = unavailable
    return result


def _positive_limit(value, factory_maximum, name):
    """Accept a scalar limit that can only tighten every SDK factory bound."""
    try:
        limit = np.asarray(value, dtype=float)
        maximum = np.asarray(factory_maximum, dtype=float)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be a finite positive scalar within the SDK factory limit") from error
    if (limit.shape != () or not np.isfinite(limit) or limit <= 0
            or not np.isfinite(maximum).all() or not np.all(limit <= maximum)):
        raise ValueError(f"{name} must be a finite positive scalar within the SDK factory limit")
    return float(limit)


def preflight(config: RobotConfig) -> dict:
    """Read Linux interface metadata and import the SDK, without connecting it."""
    result = {
        "platform": platform.system(),
        "interface": config.can_interface,
        "interface_exists": False,
        "is_can": False,
        "interface_up": False,
        "sdk_available": False,
        "ready": False,
        "errors": [],
    }
    interface = config.can_interface
    if not interface or Path(interface).name != interface or interface in (".", ".."):
        result["errors"].append("invalid CAN interface name")
    elif result["platform"] != "Linux":
        result["errors"].append("real backend requires Linux SocketCAN")
    else:
        path = Path("/sys/class/net") / interface
        result["interface_exists"] = path.is_dir()
        if path.is_dir():
            try:
                # Linux ARPHRD_CAN is 280; IFF_UP is bit zero.
                result["is_can"] = int((path / "type").read_text().strip()) == 280
                result["interface_up"] = bool(int((path / "flags").read_text().strip(), 16) & 1)
            except (OSError, ValueError) as error:
                result["errors"].append(str(error))
        else:
            result["errors"].append(f"CAN interface {interface!r} does not exist")
    try:
        importlib.import_module("arx5_interface")
        result['control_api'] = 'Arx5JointController.set_joint_cmd'
        result['gravity_compensation'] = 'native SDK; calibrated URDF; no extra scaling'
        result["sdk_available"] = True
    except (ImportError, OSError, RuntimeError, ValueError, KeyError) as error:
        result["errors"].append(f"arx5_interface import failed: {error}")
    result["ready"] = bool(
        result["platform"] == "Linux"
        and result["interface_exists"]
        and result["is_can"]
        and result["interface_up"]
        and result["sdk_available"]
    )
    return result


class RealBackend:
    """AC one X5 2025 adapter. Only explicit real-mode startup constructs it."""

    def __init__(
        self, config: RobotConfig, *,
        joint_velocity_limit_rad_s: float | None = None,
        gripper_speed_m_s: float | None = None,
        gripper_torque_limit_nm: float | None = None,
        gripper_gains: tuple[float, float] | None = None,
        automatic_gravity: bool = True,
        controller_factory=None,
        cancel_requested=None,
    ):
        if config.control_hz != 250:
            raise ValueError("X5 control period must be 4 ms (250 Hz)")
        if config.variant != "2025":
            raise ValueError("this backend requires the AC one X5 2025 profile")
        for name in ("kp", "kd", "torque_max"):
            value = np.asarray(getattr(config, name), dtype=float)
            if value.shape != (6,) or not np.isfinite(value).all() or np.any(value < 0):
                raise ValueError(f"{name} must contain six finite nonnegative values")
        if not np.isfinite(config.gripper_open_readout) or config.gripper_open_readout == 0:
            raise ValueError("gripper_open_readout must be finite and nonzero")
        if not np.isfinite(config.gripper_max_width) or config.gripper_max_width <= 0:
            raise ValueError("gripper_max_width must be finite and positive")

        # Finish local geometry/configuration checks before constructing hardware.
        from .model import JointTargetController, X5Model

        self.config = config
        self.kinematics = X5Model(config)
        self.controller = (controller_factory or JointTargetController)(config, self.kinematics)
        sdk = importlib.import_module("arx5_interface")
        self._sdk = sdk
        self._automatic_gravity = bool(automatic_gravity)
        robot = sdk.RobotConfigFactory.get_instance().get_config("X5")
        if os.environ.get('QUEST_ARX_GRIPPER_TURNS') == '1':
            if getattr(sdk, 'quest_gripper_turns_version', None) != 1:
                raise RuntimeError('Python 3.10 gripper startup patch required; run ./run.sh control build-sdk')
            robot.gripper_startup_turns = True
        else:
            # Upstream behaviour: close the gripper before powering the arm (control/README.md).
            # The platform entry ./run.sh control start --backend real enables the local patch.
            import sys
            print("Gripper startup patch off (upstream behaviour); use ./run.sh control start --backend real to enable it.",
                  file=sys.stderr, flush=True)
        control = sdk.ControllerConfigFactory.get_instance().get_config("joint_controller", 6)
        factory_defaults = {
            "robot": _config_snapshot(robot, _ROBOT_CONFIG_FIELDS),
            "controller": _config_snapshot(control, _CONTROLLER_CONFIG_FIELDS),
        }
        # Retain the same X5 motor mapping, units and gravity vector as the
        # operator-tested Stanford examples. Do not map to a different codec.
        # These optional bench limits never widen the factory envelope. Validate
        # before any hardware construction, including gripper constructor gains.
        if joint_velocity_limit_rad_s is not None:
            joint_velocity_limit_rad_s = _positive_limit(
                joint_velocity_limit_rad_s, robot.joint_vel_max, "joint_velocity_limit_rad_s")
        if gripper_speed_m_s is not None:
            gripper_speed_m_s = _positive_limit(
                gripper_speed_m_s, robot.gripper_vel_max, "gripper_speed_m_s")
        if gripper_torque_limit_nm is not None:
            gripper_torque_limit_nm = _positive_limit(
                gripper_torque_limit_nm, robot.gripper_torque_max, "gripper_torque_limit_nm")
        if gripper_gains is not None:
            try:
                gripper_gains = np.asarray(gripper_gains, dtype=float)
            except (TypeError, ValueError) as error:
                raise ValueError("gripper_gains must be two finite nonnegative values within SDK defaults") from error
            maximum = np.array([control.default_gripper_kp, control.default_gripper_kd])
            if (gripper_gains.shape != (2,) or not np.isfinite(gripper_gains).all()
                    or np.any(gripper_gains < 0) or not np.isfinite(maximum).all()
                    or np.any(gripper_gains > maximum)):
                raise ValueError("gripper_gains must be two finite nonnegative values within SDK defaults")
        robot.urdf_path = str(self.kinematics.urdf_path)
        robot.base_link_name = "base_link"
        robot.eef_link_name = self.kinematics.tcp_link
        robot.joint_pos_min = np.asarray(config.joint_min, dtype=float)
        robot.joint_pos_max = np.asarray(config.joint_max, dtype=float)
        robot.joint_torque_max = np.minimum(np.asarray(config.torque_max, dtype=float), robot.joint_torque_max)
        self._sdk_torque_ceiling = np.asarray(robot.joint_torque_max, dtype=float).copy()
        robot.gripper_width = config.gripper_max_width
        robot.gripper_open_readout = config.gripper_open_readout
        if joint_velocity_limit_rad_s is not None:
            robot.joint_vel_max = np.full(6, joint_velocity_limit_rad_s)
        if gripper_speed_m_s is not None:
            robot.gripper_vel_max = gripper_speed_m_s
        if gripper_torque_limit_nm is not None:
            robot.gripper_torque_max = gripper_torque_limit_nm
        control.controller_dt = 0.004
        control.background_send_recv = False
        # VR uses native automatic gravity exactly once. The staged bench can
        # opt out so its initial feedback-only phase has no gravity torque.
        control.gravity_compensation = self._automatic_gravity
        self._gravity_solver = None if self._automatic_gravity else sdk.Arx5Solver(
            robot.urdf_path, 6, np.asarray(robot.joint_pos_min), np.asarray(robot.joint_pos_max),
            robot.base_link_name, robot.eef_link_name, np.asarray(robot.gravity_vector))
        control.default_preview_time = 0.0
        control.default_kp = np.asarray(config.kp, dtype=float)
        control.default_kd = np.asarray(config.kd, dtype=float)
        if gripper_gains is not None:
            control.default_gripper_kp, control.default_gripper_kd = map(float, gripper_gains)
        self._gripper_kp = control.default_gripper_kp
        self._gripper_kd = control.default_gripper_kd
        self._last_sdk_timestamp = None
        self._received_at = float("-inf")
        self._closed = False
        self._stopped = False
        self._damping_requested = False
        self._motor_ids = list(robot.motor_id)
        self._gripper_motor_id = int(robot.gripper_motor_id)
        try:
            sdk_version = version("arx5-interface")
        except PackageNotFoundError:
            sdk_version = getattr(sdk, "__version__", None)
        self._runtime_parameters = {
            "backend": "stanford",
            "hardware_profile": "AC one 2025; identified by operator, not firmware discovery",
            "robot_config": _json_value(asdict(config)),
            "sdk_version": _json_value(sdk_version),
            "gripper_startup_turns": bool(getattr(robot, 'gripper_startup_turns', False)),
            "control_api": "Arx5JointController.set_joint_cmd; native position, velocity, extra feedforward torque",
            "gravity_api": ("native automatic gravity; packet torque excludes gravity" if self._automatic_gravity
                            else "Stanford Arx5Solver.inverse_dynamics(q,0,0); unscaled; automatic gravity disabled for staged bench"),
            "shutdown_gain_policy": "Stanford default_kd, without additional scaling",
            "sdk_configuration_source": "factory and overridden constructor configuration; not hardware readback",
            "sdk_factory_defaults": factory_defaults,
            "sdk_runtime": {
                "robot": _config_snapshot(robot, _ROBOT_CONFIG_FIELDS),
                "controller": _config_snapshot(control, _CONTROLLER_CONFIG_FIELDS),
            },
            "telemetry_sources": {
                "command": "SDK packet and gains from the last successfully completed send_recv_once; command_time_s is caller monotonic time",
                "applied_kp_kd": "values passed to SDK; not a readback of final clipped CAN output",
                "user_feedforward_torque": "additional torque supplied by the caller, excluding gravity; zero by default",
                "sdk_gravity_torque": "automatic mode: SDK output torque minus supplied feedforward (may include clipping); bench: unscaled inverse_dynamics(q,0,0)",
                "pose": "FK of the same six-joint feedback sample at tcp_link (fingertip midpoint)",
                "reported_joint_torque": "cached JointState.torque() from the latest existing read_state call, with SDK sample timestamp and caller read time; step reads before sending, not after; raw SDK-reported value, not independently calibrated torque",
                "sdk_raw_feedback": "six arm positions/velocities/torques and gripper position/velocity/torque from public JointState; gripper position and velocity are metres and metres/second, not motor radians",
                "unavailable": "public Python API does not expose raw motor current, temperature, voltage, per-motor error codes or per-motor reception timestamps",
                "timestamp": "SDK aggregate cache update time, not proof that all seven motors returned fresh CAN frames",
                "sdk_output_command": "get_joint_cmd/get_gain are SDK software values, not motor acknowledgements",
            },
            "firmware_torque_scaling_verified": False,
        }
        self._telemetry = dict.fromkeys((
            "command_time_s", "applied_kp", "applied_kd", "sent_q", "sent_dq",
            "gripper_target", "gripper_applied_kp", "gripper_applied_kd",
            "user_feedforward_torque", "sdk_gravity_torque", "reported_joint_torque",
            "reported_joint_torque_sdk_timestamp", "reported_joint_torque_read_time_s",
            "sim_actuator_torque", "simulation_time_s",
            "sdk_raw_feedback", "sdk_output_command", "sdk_gain_readback",
        ))
        self._telemetry["reported_joint_torque_source"] = "unavailable_not_read"
        self._telemetry['automatic_sdk_gravity'] = self._automatic_gravity
        if cancel_requested is not None and cancel_requested():
            raise InterruptedError("cancelled before SDK construction")
        self._hardware = sdk.Arx5JointController(robot, control, config.can_interface)

    def runtime_parameters(self) -> dict:
        """Return constructor snapshots without querying hardware."""
        return deepcopy(self._runtime_parameters)

    def telemetry(self) -> dict:
        """Return cached diagnostics without additional SDK calls."""
        return deepcopy(self._telemetry)

    def _require_open(self):
        if self._closed:
            raise RuntimeError("real backend is closed")
        return self._hardware

    def read_state(self, now: float) -> RobotState:
        hardware = self._require_open()
        sample = hardware.get_joint_state()
        q = np.asarray(sample.pos(), dtype=float).copy()
        dq = np.asarray(sample.vel(), dtype=float).copy()
        width = float(sample.gripper_pos)
        fault = ""
        valid = q.shape == (6,) and dq.shape == (6,)
        valid = valid and np.isfinite(q).all() and np.isfinite(dq).all() and np.isfinite(width)
        if valid:
            pose = self.kinematics.fk(q)
        else:
            fault = "invalid SDK joint feedback"
            pose = np.full(7, np.nan)

        stamp = float(sample.timestamp)
        torque = None
        raw_torque = None
        try:
            torque_accessor = getattr(sample, "torque", None)
            if not callable(torque_accessor):
                source = "unavailable_missing_api"
            else:
                value = np.asarray(torque_accessor(), dtype=float)
                raw_torque = value.tolist()
                if value.shape == (6,) and np.isfinite(value).all():
                    torque = value.tolist()
                    source = "sdk_joint_state_torque_cached_read_state"
                else:
                    source = "unavailable_invalid_value"
        except Exception:
            # Diagnostics must not make an otherwise valid control read fail.
            source = "unavailable_invalid_value"
        self._telemetry.update({
            "reported_joint_torque": torque,
            "reported_joint_torque_source": source,
            "reported_joint_torque_sdk_timestamp": _json_value(stamp),
            "reported_joint_torque_read_time_s": _json_value(float(now)),
            "host_planner_fault": self.controller.fault,
            "feedback_freshness_verified": False,
            "sdk_raw_feedback": _json_value({
                "motor_ids": self._motor_ids,
                "q": q, "dq": dq, "torque": raw_torque,
                "gripper": {"motor_id": self._gripper_motor_id,
                            "pos_m": width,
                            "vel_m_s": getattr(sample, "gripper_vel", None),
                            "torque_sdk": getattr(sample, "gripper_torque", None)},
                "sdk_timestamp_s": stamp, "local_read_monotonic_s": now,
                "current": None, "temperature": None, "voltage": None,
                "error_codes": None, "motor_reception_timestamps": None,
            }),
        })
        sdk_now = float(hardware.get_timestamp())
        if not np.isfinite(stamp) or stamp <= 0 or not np.isfinite(sdk_now):
            fault = "missing or invalid SDK feedback timestamp"
        elif stamp > sdk_now + 0.004:
            fault = "SDK feedback timestamp is in the future"
        elif self._last_sdk_timestamp is not None and stamp < self._last_sdk_timestamp:
            fault = "SDK feedback timestamp regressed"
        elif stamp != self._last_sdk_timestamp and valid:
            # Translate SDK-relative clock to caller's monotonic clock. Repeated
            # polls of the same sample never advance the receipt timestamp.
            self._received_at = now - max(0.0, sdk_now - stamp)
            self._last_sdk_timestamp = stamp
        return RobotState(
            q=q,
            dq=dq,
            pose=pose,
            gripper_width=width,
            received_at=self._received_at,
            mode=Mode.DISABLED if self._stopped else self.controller.mode,
            gripper_mode=Mode.FREE_DRIVE if self._stopped else self.controller.gripper_mode,
            fault=fault,
        )

    def step(self, command: ControlCommand, now: float, *, cancel_requested=None,
             feedback_validator=None, damping_only=False, feedforward_torque=None):
        """Submit one tick, checking cancellation before each SDK command.

        Cancellation prevents the next call; it cannot interrupt a C++ SDK
        send that is already executing. The caller performs damping shutdown.
        """
        def check_cancelled():
            if self._stopped:
                raise InterruptedError("stop latched; ordinary targets are rejected")
            if cancel_requested is not None and cancel_requested():
                raise InterruptedError("control step cancelled")

        check_cancelled()
        hardware = self._require_open()
        extra_torque = np.zeros(6) if feedforward_torque is None else np.asarray(feedforward_torque, dtype=float)
        if (extra_torque.shape != (6,) or not np.isfinite(extra_torque).all()
                or np.any(np.abs(extra_torque) > self._sdk_torque_ceiling)):
            raise ValueError('feedforward_torque must contain six finite values within SDK torque limits')
        if damping_only and (self._automatic_gravity or np.any(extra_torque)):
            raise ValueError('damping-only phase requires automatic_gravity=False and zero feedforward')
        state = self.read_state(now)
        if feedback_validator is not None:
            feedback_validator(state)
        check_cancelled()
        if state.fault:
            raise RuntimeError(f"cannot command invalid feedback: {state.fault}")
        if damping_only and (command.mode is not Mode.FREE_DRIVE
                             or command.gripper_mode is not Mode.FREE_DRIVE):
            raise ValueError('damping feedback phase cannot include position or gripper control')
        target = self.controller.update(command, state.q, now)
        check_cancelled()
        gain = self._sdk.Gain(6)
        if damping_only:
            gain.kd()[:] = self.config.kd  # Same native damping as initialization/exit.
        if target is not None:
            gain.kp()[:] = self.config.kp
            gain.kd()[:] = self.config.kd
        if self.controller.gripper_mode != Mode.FREE_DRIVE:
            gain.gripper_kp = self._gripper_kp
            gain.gripper_kd = self._gripper_kd
        packet = self._sdk.JointState(6)
        packet.pos()[:] = state.q if target is None else target
        packet.vel()[:] = self.controller.dq_target
        gravity = None
        packet_torque = extra_torque.copy()
        if not self._automatic_gravity and not damping_only:
            gravity = np.asarray(self._gravity_solver.inverse_dynamics(
                state.q, np.zeros(6), np.zeros(6)), dtype=float)
            if gravity.shape != (6,) or not np.isfinite(gravity).all():
                raise RuntimeError('Stanford gravity solver returned invalid torque')
            packet_torque += gravity
        if np.any(np.abs(packet_torque) > self._sdk_torque_ceiling):
            raise RuntimeError('feedforward plus gravity exceeds the configured SDK torque bound')
        packet.torque()[:] = packet_torque
        packet.gripper_pos = (
            state.gripper_width
            if self.controller.gripper_mode == Mode.FREE_DRIVE
            else self.controller.gripper_target
        )
        packet.gripper_vel = 0.0
        packet.gripper_torque = 0.0
        packet.timestamp = 0.0  # With zero preview, replace the current waypoint.
        sent = {
            "command_time_s": _json_value(float(now)),
            "applied_kp": gain.kp().tolist(),
            "applied_kd": gain.kd().tolist(),
            "sent_q": packet.pos().tolist(),
            "sent_dq": packet.vel().tolist(),
            "gripper_target": float(packet.gripper_pos),
            "gripper_applied_kp": float(gain.gripper_kp),
            "gripper_applied_kd": float(gain.gripper_kd),
            "user_feedforward_torque": extra_torque.tolist(),
            "sdk_gravity_torque": None if gravity is None else gravity.tolist(),
            "control_phase": "damping-feedback" if damping_only else "gravity-enabled",
            "automatic_sdk_gravity": self._automatic_gravity,
        }
        check_cancelled()
        sent["command_started_ns"] = time.monotonic_ns()
        hardware.set_joint_cmd(packet)
        check_cancelled()
        hardware.set_gain(gain)
        check_cancelled()
        hardware.send_recv_once()
        sent["command_submitted_ns"] = time.monotonic_ns()
        sent["sdk_readback_ns"] = 0
        self._telemetry.update(sent)
        # These getters copy software state; they do not issue extra CAN traffic.
        try:
            output = hardware.get_joint_cmd()
            actual_gain = hardware.get_gain()
            self._telemetry["sdk_readback_ns"] = time.monotonic_ns()
            if self._automatic_gravity:
                self._telemetry['sdk_gravity_torque'] = _json_value(np.asarray(output.torque()) - extra_torque)
            self._telemetry.update(_json_value({
                "sdk_output_command": {
                    "q": output.pos(), "dq": output.vel(), "torque": output.torque(),
                    "gripper_pos_m": output.gripper_pos,
                    "gripper_vel_m_s": output.gripper_vel,
                    "gripper_torque_sdk": output.gripper_torque,
                    "timestamp_s": output.timestamp,
                },
                "sdk_gain_readback": {
                    "kp": actual_gain.kp(), "kd": actual_gain.kd(),
                    "gripper_kp": actual_gain.gripper_kp,
                    "gripper_kd": actual_gain.gripper_kd,
                },
            }))
        except Exception as error:
            self._telemetry["sdk_output_read_error"] = f"{type(error).__name__}: {error}"

    def request_stop(self):
        """Latch normal targets and invoke the original SDK damping API.

        set_to_damping clears feedforward and position gains. It does not turn
        automatic gravity off; the SDK destructor does that during close().
        """
        self._stopped = True
        if not self._closed and not self._damping_requested:
            self._hardware.set_to_damping()
            self._damping_requested = True
            self._telemetry["stop_latched"] = True
            gain = self._hardware.get_gain()
            self._telemetry["sdk_gain_readback"] = _json_value({
                "kp": gain.kp(), "kd": gain.kd(),
                "gripper_kp": gain.gripper_kp, "gripper_kd": gain.gripper_kd,
            })

    def close(self):
        """Release SDK ownership; SDK applies damping, with no added home request."""
        if self._closed:
            return
        self._closed = True
        # SDK has no explicit close. X5 destruction enters its own approximately
        # two-second passive/damping phase; no extra home command is issued here.
        self._hardware = None

    def close_to_damping(self):
        """Request SDK damping and release ownership without parking or homing.

        SDK destruction applies its approximately two-second damping shutdown.
        This is not an immediate motor disable. Release ownership even when
        the explicit damping request fails; callers still receive that error.
        """
        if self._closed:
            return
        try:
            self._hardware.set_to_damping()
        finally:
            self.close()
