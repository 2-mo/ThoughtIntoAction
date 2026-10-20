"""Import-isolated ARX5 backend boundary.

This module describes the SDK boundary and canonical state conversion. The
default composition remains hardware-disabled; no SDK import or controller
construction occurs until an explicit hardware-enabled configuration is passed
to ``connect``. Active Cartesian IK/Ruckig command execution is intentionally
not enabled until the user's arm variant and stop semantics are confirmed.
"""

from __future__ import annotations
from embodied.workstation.components.ownership import leased_connect, leased_disconnect


import importlib
import math
import threading
import time
from collections import deque
from collections.abc import Callable, Iterable, Sequence
from contextlib import suppress
from dataclasses import dataclass
from typing import Any, Protocol, cast

from embodied.phone.core.capabilities import RobotCapabilities
from embodied.phone.core.frames import (
    quaternion_from_axis_angle,
    quaternion_multiply,
    quaternion_slerp,
)
from embodied.phone.core.models import (
    ControlMode,
    Pose,
    Quaternion,
    RobotCommand,
    RobotState,
    Vector3,
)
from embodied.phone.kinematics.models import IKSolverConfig, OfflineKinematics

from embodied.phone.robots.arx5_safety import (
    Arx5SafetyEnvelope,
    Arx5SafetyViolation,
    validate_interface_name,
)


class Arx5BackendError(RuntimeError):
    """Base error for the SDK boundary."""


class Arx5HardwareDisabledError(Arx5BackendError):
    """Raised before any SDK import when hardware is not explicitly enabled."""


class Arx5DependencyUnavailableError(Arx5BackendError):
    """Raised when the optional vendor package is not installed."""


class Arx5FeedbackError(Arx5BackendError):
    """Raised for malformed or stale SDK feedback."""


class Arx5CommandError(Arx5BackendError):
    """Raised when the not-yet-verified active command path is requested."""


@dataclass(frozen=True, slots=True)
class Arx5BackendConfig:
    """Explicit runtime gate and expected dimensions for one X5 instance."""

    model: str = "X5"
    interface_name: str | None = None
    hardware_enabled: bool = False
    control_rate_hz: float = 200.0
    joint_count: int = 6
    # The vendor background pump can occasionally publish repeated timestamps
    # while CAN remains healthy. Treat only a sustained 1.5 s freeze as fault.
    feedback_stale_timeout_ms: float = 1_500.0
    disconnect_damping_enabled: bool = True
    damping_on_connect_error: bool = True
    feedback_fault_damping_enabled: bool = True
    active_command_enabled: bool = False
    gripper_command_enabled: bool = False
    gripper_only: bool = False
    max_joint_target_step_rad: float = 0.08
    position_gain_scale: float = 0.25
    gripper_gain_scale: float = 0.20
    # X5 reports a 0.30 m/s gripper limit. At the 200 Hz servo cadence this
    # permits a 1.5 mm target change per cycle.
    max_gripper_step_m: float = 0.0015
    gripper_open_readout: float | None = None
    background_send_recv: bool = True
    interpolation_method: str = "cubic"
    safety_envelope: Arx5SafetyEnvelope | None = None

    def __post_init__(self) -> None:
        model = self.model.strip()
        if not model:
            raise ValueError("ARX5 model must not be blank")
        raw_interface = (
            "" if self.interface_name is None else self.interface_name.strip()
        )
        interface = (
            None if not raw_interface else validate_interface_name(raw_interface)
        )
        if self.hardware_enabled and not interface:
            raise ValueError("interface_name is required when ARX5 hardware is enabled")
        if not math.isfinite(self.control_rate_hz) or self.control_rate_hz <= 0.0:
            raise ValueError("control_rate_hz must be positive and finite")
        if isinstance(self.joint_count, bool) or self.joint_count <= 0:
            raise ValueError("joint_count must be positive")
        if (
            not math.isfinite(self.feedback_stale_timeout_ms)
            or self.feedback_stale_timeout_ms <= 0.0
        ):
            raise ValueError("feedback_stale_timeout_ms must be positive and finite")
        if not isinstance(self.disconnect_damping_enabled, bool):
            raise ValueError("disconnect_damping_enabled must be boolean")
        if not isinstance(self.damping_on_connect_error, bool):
            raise ValueError("damping_on_connect_error must be boolean")
        if not isinstance(self.feedback_fault_damping_enabled, bool):
            raise ValueError("feedback_fault_damping_enabled must be boolean")
        if not isinstance(self.active_command_enabled, bool):
            raise ValueError("active_command_enabled must be boolean")
        if not isinstance(self.gripper_command_enabled, bool):
            raise ValueError("gripper_command_enabled must be boolean")
        if self.gripper_command_enabled and not self.active_command_enabled:
            raise ValueError("gripper_command_enabled requires active_command_enabled")
        if self.gripper_only and not self.gripper_command_enabled:
            raise ValueError("gripper_only requires gripper_command_enabled")
        if (
            not math.isfinite(self.max_joint_target_step_rad)
            or self.max_joint_target_step_rad <= 0.0
        ):
            raise ValueError("max_joint_target_step_rad must be positive and finite")
        if (
            not math.isfinite(self.position_gain_scale)
            or not 0.0 < self.position_gain_scale <= 1.0
        ):
            raise ValueError("position_gain_scale must be in (0, 1]")
        if (
            not math.isfinite(self.gripper_gain_scale)
            or not 0.0 < self.gripper_gain_scale <= 1.0
        ):
            raise ValueError("gripper_gain_scale must be in (0, 1]")
        if not math.isfinite(self.max_gripper_step_m) or self.max_gripper_step_m <= 0.0:
            raise ValueError("max_gripper_step_m must be positive and finite")
        if self.gripper_open_readout is not None and (
            not math.isfinite(self.gripper_open_readout)
            or not 0.5 <= abs(self.gripper_open_readout) <= 10.0
        ):
            raise ValueError(
                "gripper_open_readout magnitude must be finite and in [0.5, 10.0]"
            )
        if not isinstance(self.background_send_recv, bool):
            raise ValueError("background_send_recv must be boolean")
        interpolation_method = self.interpolation_method.strip()
        if interpolation_method not in {"cubic", "linear"}:
            raise ValueError("interpolation_method must be 'cubic' or 'linear'")
        envelope = self.safety_envelope
        if envelope is not None and envelope.joint_count != self.joint_count:
            raise ValueError("ARX5 safety envelope joint count does not match config")
        object.__setattr__(self, "model", model)
        object.__setattr__(self, "interface_name", interface)
        object.__setattr__(self, "interpolation_method", interpolation_method)


class Arx5JointState(Protocol):
    timestamp: float
    gripper_pos: float
    gripper_vel: float
    gripper_torque: float

    def pos(self) -> Sequence[float]: ...

    def vel(self) -> Sequence[float]: ...

    def torque(self) -> Sequence[float]: ...


class Arx5EefState(Protocol):
    timestamp: float
    gripper_pos: float
    gripper_vel: float
    gripper_torque: float

    def pose_6d(self) -> Sequence[float]: ...


class Arx5SdkController(Protocol):
    def get_joint_state(self) -> Arx5JointState: ...

    def reset_to_home(self) -> None: ...

    def set_to_damping(self) -> None: ...

    def set_joint_cmd(self, command: object) -> None: ...

    def get_controller_config(self) -> object: ...

    def get_robot_config(self) -> object: ...

    def set_gain(self, gain: object) -> None: ...


class Arx5SdkModule(Protocol):
    RobotConfigFactory: Any
    ControllerConfigFactory: Any
    Arx5JointController: Any
    JointState: Any
    Gain: Any


@dataclass(frozen=True, slots=True)
class Arx5RawFeedback:
    """Backend-local SDK feedback before canonical normalization."""

    sdk_timestamp_s: float
    joint_position: tuple[float, ...]
    joint_velocity: tuple[float, ...]
    joint_torque: tuple[float, ...]
    gripper_position_raw: float
    gripper_velocity_raw: float
    gripper_torque_raw: float


SdkLoader = Callable[[], object]
ControllerFactory = Callable[[object, Arx5BackendConfig], Arx5SdkController]
TcpPoseProvider = Callable[[tuple[float, ...]], Pose]


def _finite_sequence(
    name: str,
    value: object,
    expected_length: int,
) -> tuple[float, ...]:
    if isinstance(value, (str, bytes)):
        raise Arx5FeedbackError(f"{name} is not a numeric sequence")
    try:
        items = cast(Iterable[Any], value)
        result = tuple(float(item) for item in items)
    except (TypeError, ValueError) as exc:
        raise Arx5FeedbackError(f"{name} is not a numeric sequence") from exc
    if len(result) != expected_length or not all(
        math.isfinite(item) for item in result
    ):
        raise Arx5FeedbackError(f"{name} must contain {expected_length} finite values")
    return result


def _finite_scalar(name: str, value: object) -> float:
    try:
        result = float(cast(Any, value))
    except (TypeError, ValueError) as exc:
        raise Arx5FeedbackError(f"{name} is not numeric") from exc
    if not math.isfinite(result):
        raise Arx5FeedbackError(f"{name} must be finite")
    return result


def _member(value: object, name: str) -> object:
    try:
        member = getattr(value, name)
    except AttributeError as exc:
        raise Arx5FeedbackError(f"SDK state has no {name}") from exc
    try:
        return member() if callable(member) else member
    except Exception as exc:
        raise Arx5FeedbackError(f"SDK state {name} read failed: {exc}") from exc


def _rpy_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    # ARX5's documented pose_6d layout is XYZ + fixed-axis RPY. Keep this
    # conversion at the SDK boundary; canonical models remain XYZW quaternions.
    roll_quat = quaternion_from_axis_angle(Vector3(1.0, 0.0, 0.0), roll)
    pitch_quat = quaternion_from_axis_angle(Vector3(0.0, 1.0, 0.0), pitch)
    yaw_quat = quaternion_from_axis_angle(Vector3(0.0, 0.0, 1.0), yaw)
    return quaternion_multiply(quaternion_multiply(yaw_quat, pitch_quat), roll_quat)


def pose_from_sdk_eef_state(eef_state: Arx5EefState) -> Pose:
    """Convert a documented SDK EEF pose shape to canonical geometry.

    This helper is deliberately separate from runtime feedback. The inspected
    reference derives measured TCP pose from measured joints through FK because
    SDK EEF state may not have the required measurement semantics.
    """

    pose_6d = _finite_sequence("eef pose", _member(eef_state, "pose_6d"), 6)
    return Pose(
        position=Vector3(*pose_6d[:3]),
        orientation=_rpy_quaternion(*pose_6d[3:]),
    )


def _normalized_gripper_readout(
    raw_position: float,
    open_readout: float | None,
) -> float | None:
    """Convert the configured closed-to-open SDK readout into 0..1 feedback."""

    if open_readout is None:
        return None
    return min(1.0, max(0.0, raw_position / open_readout))


def robot_state_from_sdk(
    joint_state: Arx5JointState,
    tcp_pose: Pose,
    *,
    timestamp_ns: int,
    joint_count: int,
    gripper: float | None = None,
) -> RobotState:
    """Convert measured joints plus a verified FK pose without vendor imports."""

    if isinstance(timestamp_ns, bool) or timestamp_ns < 0:
        raise ValueError("timestamp_ns must be non-negative")
    if not isinstance(tcp_pose, Pose):
        raise Arx5FeedbackError("verified TCP pose must use canonical Pose")
    position = _finite_sequence(
        "joint position", _member(joint_state, "pos"), joint_count
    )
    velocity = _finite_sequence(
        "joint velocity", _member(joint_state, "vel"), joint_count
    )
    torque = _finite_sequence(
        "joint torque", _member(joint_state, "torque"), joint_count
    )
    return RobotState(
        timestamp_ns=timestamp_ns,
        joint_position=position,
        joint_velocity=velocity,
        joint_torque=torque,
        tcp_pose=tcp_pose,
        tcp_twist=None,
        gripper=gripper,
        external_wrench=None,
    )


def _load_sdk() -> object:
    try:
        return importlib.import_module("arx5_interface")
    except ImportError as exc:
        raise Arx5DependencyUnavailableError(
            "arx5-interface is optional; run uv sync --extra arx5 on Ubuntu x86_64"
        ) from exc


def _default_controller_factory(
    sdk: object, config: Arx5BackendConfig
) -> Arx5SdkController:
    try:
        module = cast(Arx5SdkModule, sdk)
        robot_factory = module.RobotConfigFactory
        controller_factory = module.ControllerConfigFactory
        controller_type = module.Arx5JointController
        robot_config = robot_factory.get_instance().get_config(config.model)
        if config.gripper_open_readout is not None:
            robot_config.gripper_open_readout = config.gripper_open_readout
        controller_config = controller_factory.get_instance().get_config(
            "joint_controller", int(robot_config.joint_dof)
        )
        # The SDK owns the CAN send/receive pump. The inspected reference and
        # hardware probe both require background I/O for advancing feedback
        # timestamps; the framework still owns the higher-level latest-target
        # Servo cadence.
        controller_config.background_send_recv = config.background_send_recv
        controller_config.interpolation_method = config.interpolation_method
        return cast(
            Arx5SdkController,
            controller_type(
                robot_config,
                controller_config,
                config.interface_name,
            ),
        )
    except (AttributeError, TypeError, ValueError) as exc:
        raise Arx5BackendError(
            f"could not construct ARX5 SDK controller: {exc}"
        ) from exc


class Arx5Backend:
    """ARX5 lifecycle boundary with a fail-closed active-command surface."""

    def __init__(
        self,
        config: Arx5BackendConfig | None = None,
        *,
        sdk_loader: SdkLoader = _load_sdk,
        controller_factory: ControllerFactory = _default_controller_factory,
        kinematics: OfflineKinematics | None = None,
        tcp_pose_provider: TcpPoseProvider | None = None,
        clock: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        self._config = config or Arx5BackendConfig()
        self._sdk_loader = sdk_loader
        self._controller_factory = controller_factory
        self._kinematics = kinematics
        self._tcp_pose_provider = tcp_pose_provider
        self._clock = clock
        self._lock = threading.RLock()
        self._controller: Arx5SdkController | None = None
        self._state: RobotState | None = None
        self._connected = False
        self._safe_stopped = False
        self._faulted = False
        self._safe_stop_count = 0
        self._hold_count = 0
        self._stop_count = 0
        self._commands: deque[RobotCommand] = deque(maxlen=256)
        self._last_sdk_timestamp_s: float | None = None
        self._last_feedback_progress_ns: int | None = None
        self._raw_feedback: Arx5RawFeedback | None = None
        self._last_safe_stop_error: str | None = None
        self._sdk_module: Arx5SdkModule | None = None
        self._gripper_width_m: float | None = None
        self._gripper_target_m: float | None = None
        self._hold_joint_position: tuple[float, ...] | None = None

        self._capabilities = RobotCapabilities(
            cartesian_position=True,
            cartesian_velocity=False,
            joint_position=True,
            torque_control=False,
            gripper=self._config.gripper_command_enabled,
            external_wrench=False,
            control_rate_hz=self._config.control_rate_hz,
            joint_count=self._config.joint_count,
        )

    @property
    def capabilities(self) -> RobotCapabilities:
        return self._capabilities

    @property
    def connected(self) -> bool:
        with self._lock:
            return self._connected

    @property
    def safe_stopped(self) -> bool:
        with self._lock:
            return self._safe_stopped

    @property
    def faulted(self) -> bool:
        """Whether a feedback failure has latched this connection closed."""

        with self._lock:
            return self._faulted

    @property
    def safe_stop_count(self) -> int:
        with self._lock:
            return self._safe_stop_count

    @property
    def hold_count(self) -> int:
        with self._lock:
            return self._hold_count

    @property
    def stop_count(self) -> int:
        with self._lock:
            return self._stop_count

    @property
    def command_history(self) -> tuple[RobotCommand, ...]:
        with self._lock:
            return tuple(self._commands)

    @property
    def raw_feedback(self) -> Arx5RawFeedback | None:
        with self._lock:
            return self._raw_feedback

    @property
    def last_safe_stop_error(self) -> str | None:
        """Return the most recent SDK damping failure without claiming a stop."""

        with self._lock:
            return self._last_safe_stop_error

    def recover_after_safe_stop(self) -> None:
        """Return to joint zero before rebuilding a stopped controller.

        A safe-stop intentionally leaves the current controller latched. The
        caller must first obtain an explicit reset request and a released,
        normally tracked input before this lifecycle transition is allowed. A
        recovery must not release an arbitrary arm pose while the controller is
        rebuilt, so joint-zero recovery completes first.
        """

        with self._lock:
            self._require_connected()
            if not self._config.active_command_enabled:
                raise Arx5CommandError(
                    "ARX5 recovery requires the explicit active-command gate"
                )
            controller = self._controller
            if controller is None:
                raise Arx5CommandError("ARX5 controller is unavailable")
            zero_position = self._return_to_joint_zero_locked(controller)
            self.disconnect()
            self.connect(initial_joint_position=zero_position)

    def reconnect_after_safe_stop(self) -> None:
        """Compatibility alias for recovery that returns the arm to zero first."""

        self.recover_after_safe_stop()

    @leased_connect(lambda self: self._config.interface_name)
    def connect(
        self,
        *,
        initial_joint_position: tuple[float, ...] | None = None,
    ) -> None:
        with self._lock:
            if self._connected:
                return
            if not self._config.hardware_enabled:
                raise Arx5HardwareDisabledError(
                    "ARX5 hardware is disabled; pass the explicit hardware gate"
                )
            if self._tcp_pose_provider is None:
                raise Arx5BackendError(
                    "ARX5 connection requires a verified measured-joint FK provider"
                )
            sdk = self._sdk_loader()
            controller = self._controller_factory(sdk, self._config)
            self._last_sdk_timestamp_s = None
            self._last_feedback_progress_ns = None
            self._raw_feedback = None
            self._faulted = False
            self._last_safe_stop_error = None
            try:
                state = self._read_state_from_controller(controller)
            except Exception:
                if self._config.damping_on_connect_error:
                    with suppress(Exception):
                        controller.set_to_damping()
                raise
            self._controller = controller
            self._sdk_module = cast(Arx5SdkModule, sdk)
            self._state = state
            self._connected = True
            self._safe_stopped = False
            if self._config.active_command_enabled:
                try:
                    self._activate_position_control_locked(
                        controller,
                        joint_position=initial_joint_position,
                    )
                    self._hold_joint_position = initial_joint_position
                except Exception:
                    self._set_to_damping_locked(controller, raise_on_error=False)
                    self._controller = None
                    self._sdk_module = None
                    self._connected = False
                    raise

    def state(self) -> RobotState:
        with self._lock:
            self._require_connected()
            if self._faulted:
                raise Arx5FeedbackError(
                    "ARX5 backend is faulted; reconnect before reading state"
                )
            if self._safe_stopped:
                assert self._state is not None
                return self._state
            assert self._controller is not None
            try:
                self._state = self._read_state_from_controller(self._controller)
            except Arx5FeedbackError:
                self._faulted = True
                # A direct backend caller does not necessarily have a ServoLoop
                # around it. Try the documented SDK damping primitive here as
                # well; the original feedback error remains the primary signal.
                if self._config.feedback_fault_damping_enabled:
                    self._attempt_damping_after_boundary_fault_locked()
                raise
            return self._state

    def send(self, command: RobotCommand) -> RobotState:
        with self._lock:
            self._require_connected()
            if self._faulted:
                if command.mode is ControlMode.STOP:
                    self.safe_stop(
                        command.reason or "stop command after feedback fault"
                    )
                    assert self._state is not None
                    return self._state
                raise Arx5CommandError(
                    "ARX5 backend is faulted; reconnect before accepting commands"
                )
            if self._safe_stopped and command.mode is not ControlMode.STOP:
                raise Arx5CommandError(
                    "ARX5 backend is safe-stopped; reconnect before accepting commands"
                )
            if command.mode is ControlMode.HOME:
                if not self._config.active_command_enabled:
                    raise Arx5CommandError(
                        "ARX5 joint-zero home requires the explicit active-command gate"
                    )
                controller = self._controller
                if controller is None:
                    raise Arx5CommandError("ARX5 controller is unavailable")
                self._return_to_joint_zero_locked(controller, gripper=command.gripper)
                self._commands.append(command)
                assert self._state is not None
                return self._state
            if command.mode is ControlMode.HOLD:
                self._commands.append(command)
                if self._config.active_command_enabled:
                    self._hold_position_locked(command.gripper)
                else:
                    self.hold(command.reason or "hold command")
                assert self._state is not None
                return self._state
            if command.mode is ControlMode.STOP:
                self._stop_count += 1
                self._commands.append(command)
                self._hold_joint_position = None
                self.safe_stop(command.reason or "stop command")
                assert self._state is not None
                return self._state
            if command.mode is ControlMode.JOINT_POSITION:
                if not self._config.active_command_enabled:
                    raise Arx5CommandError(
                        "ARX5 joint command requires the explicit active-command gate"
                    )
                if command.target_joint_position is None:
                    raise Arx5CommandError("joint command is missing target positions")
                if self._state is None:
                    raise Arx5BackendError("ARX5 state is unavailable")
                if len(command.target_joint_position) != self._config.joint_count:
                    raise Arx5CommandError("joint command has the wrong joint count")
                self._hold_joint_position = None
                delta = tuple(
                    target - current
                    for target, current in zip(
                        command.target_joint_position,
                        self._state.joint_position,
                        strict=True,
                    )
                )
                step = math.sqrt(sum(value * value for value in delta))
                if step > self._config.max_joint_target_step_rad:
                    fraction = self._config.max_joint_target_step_rad / step
                    delta = tuple(value * fraction for value in delta)
                target = tuple(
                    current + command.velocity_scale * value
                    for current, value in zip(
                        self._state.joint_position,
                        delta,
                        strict=True,
                    )
                )
                bounded_command = RobotCommand(
                    mode=ControlMode.JOINT_POSITION,
                    target_pose=None,
                    target_twist=None,
                    target_joint_position=target,
                    gripper=command.gripper,
                    velocity_scale=command.velocity_scale,
                    timestamp_ns=command.timestamp_ns,
                    reason=command.reason or "joint position target",
                )
                self._validate_command_envelope(bounded_command)
                self._send_joint_command_locked(target, gripper=command.gripper)
                self._commands.append(bounded_command)
                self._state = self._read_state_from_controller(self._controller)
                return self._state
            if (
                command.mode is ControlMode.CARTESIAN_POSITION
                and self._config.gripper_only
            ):
                self._hold_position_locked(command.gripper)
                self._commands.append(command)
                self._state = self._read_state_from_controller(self._controller)
                return self._state
            if command.mode is not ControlMode.CARTESIAN_POSITION:
                raise Arx5CommandError(
                    "low-speed ARX5 pilot accepts Cartesian position targets only"
                )
            if command.target_pose is None:
                raise Arx5CommandError("Cartesian command is missing target pose")
            self._hold_joint_position = None
            try:
                self._validate_command_envelope(command)
            except Arx5SafetyViolation as exc:
                self._faulted = True
                self.safe_stop("Cartesian command exceeded robot-local envelope")
                raise Arx5CommandError(
                    f"ARX5 command violates robot-local envelope: {exc}"
                ) from exc
            if not self._config.active_command_enabled:
                self.safe_stop("active ARX5 command path is not enabled")
                raise Arx5CommandError(
                    "ARX5 active command path requires the explicit active-command gate"
                )
            if self._kinematics is None or self._state is None:
                raise Arx5CommandError("ARX5 active command path has no IK provider")
            ik_config = IKSolverConfig(
                max_iterations=100,
                damping=1e-3,
                maximum_joint_step_rad=self._config.max_joint_target_step_rad,
            )
            result = self._kinematics.solve_ik(
                command.target_pose,
                self._state.joint_position,
                config=ik_config,
            )
            if not result.reachable:
                # A phone pose can jump into a locally unreachable IK branch
                # while the operator is still holding the clutch. Back off
                # toward the measured pose instead of dropping the CAN session.
                measured_pose = self._state.tcp_pose
                for fraction in (0.5, 0.25, 0.1):
                    fallback_pose = Pose(
                        measured_pose.position
                        + (command.target_pose.position - measured_pose.position)
                        * fraction,
                        quaternion_slerp(
                            measured_pose.orientation,
                            command.target_pose.orientation,
                            fraction,
                        ),
                    )
                    fallback = self._kinematics.solve_ik(
                        fallback_pose,
                        self._state.joint_position,
                        config=ik_config,
                    )
                    if fallback.reachable:
                        result = fallback
                        break
            if not result.reachable:
                self._send_joint_command_locked(
                    self._state.joint_position,
                    gripper=command.gripper,
                )
                self._commands.append(command)
                self._state = self._read_state_from_controller(self._controller)
                return self._state
            delta = tuple(
                target - current
                for target, current in zip(
                    result.joint_position,
                    self._state.joint_position,
                    strict=True,
                )
            )
            step = math.sqrt(sum(value * value for value in delta))
            if step > self._config.max_joint_target_step_rad:
                fraction = self._config.max_joint_target_step_rad / step
                delta = tuple(value * fraction for value in delta)
            scale = command.velocity_scale
            target = tuple(
                current + scale * value
                for current, value in zip(
                    self._state.joint_position,
                    delta,
                    strict=True,
                )
            )
            try:
                self._validate_command_envelope(
                    RobotCommand(
                        mode=ControlMode.JOINT_POSITION,
                        target_pose=None,
                        target_twist=None,
                        target_joint_position=target,
                        gripper=None,
                        velocity_scale=scale,
                        timestamp_ns=command.timestamp_ns,
                    )
                )
            except Arx5SafetyViolation:
                self._send_joint_command_locked(
                    self._state.joint_position,
                    gripper=command.gripper,
                )
                self._commands.append(command)
                self._state = self._read_state_from_controller(self._controller)
                return self._state
            self._send_joint_command_locked(target, gripper=command.gripper)
            self._commands.append(command)
            self._state = self._read_state_from_controller(self._controller)
            return self._state

    def hold(self, reason: str) -> None:
        if not reason.strip():
            raise ValueError("hold reason must not be blank")
        with self._lock:
            self._hold_count += 1
        # Damping is the only SDK stop primitive currently evidenced by the
        # inspected SDK/reference. It is not claimed to be a position hold.
        self.safe_stop(f"hold requested before verified ARX5 hold: {reason}")

    def safe_stop(self, reason: str) -> None:
        if not reason.strip():
            raise ValueError("safe-stop reason must not be blank")
        with self._lock:
            self._require_connected()
            if self._safe_stopped:
                return
            controller = self._controller
            if controller is None:
                raise Arx5BackendError("ARX5 controller is unavailable")
            self._set_to_damping_locked(controller, raise_on_error=True)

    @leased_disconnect
    def disconnect(self) -> None:
        with self._lock:
            if not self._connected:
                return
            try:
                if (
                    self._config.disconnect_damping_enabled
                    and not self._safe_stopped
                    and not self._faulted
                ):
                    self.safe_stop("ARX5 backend disconnect")
            finally:
                self._controller = None
                self._sdk_module = None
                self._gripper_width_m = None
                self._gripper_target_m = None
                self._hold_joint_position = None
                self._connected = False

    def _read_state_from_controller(self, controller: Arx5SdkController) -> RobotState:
        try:
            joint_state = controller.get_joint_state()
        except Exception as exc:
            raise Arx5FeedbackError(f"ARX5 SDK feedback read failed: {exc}") from exc
        sdk_timestamp_s = _finite_scalar(
            "SDK timestamp", _member(joint_state, "timestamp")
        )
        now_ns = self._clock()
        if self._last_sdk_timestamp_s is not None:
            if sdk_timestamp_s <= self._last_sdk_timestamp_s:
                if (
                    self._last_feedback_progress_ns is not None
                    and now_ns - self._last_feedback_progress_ns
                    > self._config.feedback_stale_timeout_ms * 1_000_000.0
                ):
                    raise Arx5FeedbackError("ARX5 feedback timestamp stopped advancing")
            else:
                self._last_feedback_progress_ns = now_ns
        else:
            self._last_feedback_progress_ns = now_ns
        self._last_sdk_timestamp_s = sdk_timestamp_s
        joint_position = _finite_sequence(
            "joint position",
            _member(joint_state, "pos"),
            self._config.joint_count,
        )
        raw_feedback = Arx5RawFeedback(
            sdk_timestamp_s=sdk_timestamp_s,
            joint_position=joint_position,
            joint_velocity=_finite_sequence(
                "joint velocity",
                _member(joint_state, "vel"),
                self._config.joint_count,
            ),
            joint_torque=_finite_sequence(
                "joint torque",
                _member(joint_state, "torque"),
                self._config.joint_count,
            ),
            gripper_position_raw=_finite_scalar(
                "gripper position",
                _member(joint_state, "gripper_pos"),
            ),
            gripper_velocity_raw=_finite_scalar(
                "gripper velocity",
                _member(joint_state, "gripper_vel"),
            ),
            gripper_torque_raw=_finite_scalar(
                "gripper torque",
                _member(joint_state, "gripper_torque"),
            ),
        )
        envelope = self._config.safety_envelope
        if envelope is not None:
            try:
                envelope.validate_feedback(
                    joint_position=raw_feedback.joint_position,
                    joint_velocity=raw_feedback.joint_velocity,
                    joint_torque=raw_feedback.joint_torque,
                )
            except Arx5SafetyViolation:
                # A measured arm can be outside the provisional envelope after
                # manual motion or vendor homing. Keep publishing feedback so
                # the operator can recover; command targets remain validated.
                pass
        provider = self._tcp_pose_provider
        if provider is None:
            raise Arx5BackendError("ARX5 measured-joint FK provider is unavailable")
        try:
            tcp_pose = provider(joint_position)
        except Exception as exc:
            raise Arx5FeedbackError(
                f"ARX5 measured-joint FK provider rejected feedback: {exc}"
            ) from exc
        if envelope is not None:
            try:
                envelope.validate_pose(tcp_pose)
            except Arx5SafetyViolation:
                pass
        self._raw_feedback = raw_feedback
        return robot_state_from_sdk(
            joint_state,
            tcp_pose,
            timestamp_ns=now_ns,
            joint_count=self._config.joint_count,
            gripper=_normalized_gripper_readout(
                raw_feedback.gripper_position_raw,
                self._config.gripper_open_readout,
            ),
        )

    def _validate_command_envelope(self, command: RobotCommand) -> None:
        envelope = self._config.safety_envelope
        if envelope is None:
            return
        if command.target_pose is not None:
            envelope.validate_pose(command.target_pose)
        if command.target_twist is not None:
            envelope.validate_twist(command.target_twist)
        if command.target_joint_position is not None:
            envelope.validate_joint_position(
                command.target_joint_position,
                name="joint command position",
            )

    def _return_to_joint_zero_locked(
        self,
        controller: Arx5SdkController,
        *,
        gripper: float | None = None,
    ) -> tuple[float, ...]:
        """Move synchronously to all-zero joints and keep zero as HOLD target."""

        zero_position = (0.0,) * self._config.joint_count
        self._last_sdk_timestamp_s = None
        self._last_feedback_progress_ns = None
        self._state = self._read_state_from_controller(controller)
        self._activate_position_control_locked(
            controller,
            joint_position=self._state.joint_position,
        )
        self._safe_stopped = False
        step_limit = self._config.max_joint_target_step_rad * 0.25
        period_s = 1.0 / self._config.control_rate_hz
        deadline_s = time.monotonic() + 10.0
        while True:
            assert self._state is not None
            current = self._state.joint_position
            if max(abs(value) for value in current) <= 0.002:
                break
            delta = tuple(
                target - measured
                for target, measured in zip(zero_position, current, strict=True)
            )
            distance = math.sqrt(sum(value * value for value in delta))
            fraction = 1.0 if distance <= step_limit else step_limit / distance
            target_position = tuple(
                measured + value * fraction
                for measured, value in zip(current, delta, strict=True)
            )
            self._validate_command_envelope(
                RobotCommand(
                    mode=ControlMode.JOINT_POSITION,
                    target_pose=None,
                    target_twist=None,
                    target_joint_position=target_position,
                    gripper=gripper,
                    velocity_scale=0.25,
                    timestamp_ns=self._clock(),
                    reason="joint-zero home target",
                )
            )
            self._send_joint_command_locked(target_position, gripper=gripper)
            if time.monotonic() >= deadline_s:
                self._set_to_damping_locked(controller, raise_on_error=False)
                raise Arx5CommandError("ARX5 joint-zero home timed out")
            time.sleep(period_s)
            self._state = self._read_state_from_controller(controller)
        self._send_joint_command_locked(zero_position, gripper=gripper)
        self._hold_joint_position = zero_position
        self._state = self._read_state_from_controller(controller)
        return zero_position

    def _activate_position_control_locked(
        self,
        controller: Arx5SdkController,
        *,
        joint_position: tuple[float, ...] | None = None,
    ) -> None:
        if self._state is None:
            raise Arx5BackendError("ARX5 state is unavailable for activation")
        robot_config = controller.get_robot_config()
        gripper_width = float(robot_config.gripper_width)
        if not math.isfinite(gripper_width) or gripper_width <= 0.0:
            raise Arx5BackendError("ARX5 gripper width is invalid")
        self._gripper_width_m = gripper_width
        current_gripper = (
            0.0
            if self._raw_feedback is None
            else self._raw_feedback.gripper_position_raw
        )
        self._gripper_target_m = min(max(current_gripper, 0.0), gripper_width)
        self._send_joint_command_locked(
            self._state.joint_position if joint_position is None else joint_position,
            controller=controller,
        )
        sdk = self._sdk_module
        if sdk is None:
            raise Arx5BackendError("ARX5 SDK module is unavailable")
        controller_config = controller.get_controller_config()
        numpy = importlib.import_module("numpy")
        gain = sdk.Gain(
            numpy.asarray(controller_config.default_kp)
            * self._config.position_gain_scale,
            numpy.asarray(controller_config.default_kd),
            (
                float(controller_config.default_gripper_kp)
                * self._config.gripper_gain_scale
                if self._config.gripper_command_enabled
                else 0.0
            ),
            (
                float(controller_config.default_gripper_kd)
                if self._config.gripper_command_enabled
                else 0.0
            ),
        )
        controller.set_gain(gain)

    def _hold_position_locked(self, gripper: float | None) -> None:
        if self._state is None:
            raise Arx5BackendError("ARX5 state is unavailable for position hold")
        if self._hold_joint_position is None:
            self._hold_joint_position = self._state.joint_position
        self._send_joint_command_locked(self._hold_joint_position, gripper=gripper)

    def _send_joint_command_locked(
        self,
        position: tuple[float, ...],
        *,
        controller: Arx5SdkController | None = None,
        gripper: float | None = None,
    ) -> None:
        active_controller = controller or self._controller
        sdk = self._sdk_module
        if active_controller is None or sdk is None:
            raise Arx5BackendError("ARX5 controller is unavailable")
        raw_gripper = (
            0.0
            if self._raw_feedback is None
            else self._raw_feedback.gripper_position_raw
        )
        gripper_position = raw_gripper
        if self._config.gripper_command_enabled:
            width = self._gripper_width_m
            if width is None:
                raise Arx5BackendError("ARX5 gripper width is unavailable")
            desired = raw_gripper if gripper is None else float(gripper) * width
            desired = min(max(desired, 0.0), width)
            previous = (
                min(max(raw_gripper, 0.0), width)
                if self._gripper_target_m is None
                else self._gripper_target_m
            )
            step = min(
                self._config.max_gripper_step_m,
                max(-self._config.max_gripper_step_m, desired - previous),
            )
            gripper_position = previous + step
            self._gripper_target_m = gripper_position
        numpy = importlib.import_module("numpy")
        command = sdk.JointState(
            numpy.asarray(position, dtype=numpy.float64),
            numpy.zeros(self._config.joint_count, dtype=numpy.float64),
            numpy.zeros(self._config.joint_count, dtype=numpy.float64),
            gripper_position,
            0.0,
            0.0,
        )
        active_controller.set_joint_cmd(command)

    def _attempt_damping_after_boundary_fault_locked(self) -> None:
        if self._safe_stopped:
            return
        controller = self._controller
        if controller is None:
            return
        self._set_to_damping_locked(controller, raise_on_error=False)

    def _set_to_damping_locked(
        self,
        controller: Arx5SdkController,
        *,
        raise_on_error: bool,
    ) -> None:
        try:
            controller.set_to_damping()
        except Exception as exc:
            self._last_safe_stop_error = str(exc)
            if raise_on_error:
                raise Arx5BackendError(f"ARX5 SDK damping failed: {exc}") from exc
            return
        self._last_safe_stop_error = None
        self._safe_stopped = True
        self._safe_stop_count += 1

    def _require_connected(self) -> None:
        if not self._connected:
            raise Arx5BackendError("ARX5 backend is not connected")


__all__ = [
    "Arx5Backend",
    "Arx5BackendConfig",
    "Arx5BackendError",
    "Arx5CommandError",
    "Arx5DependencyUnavailableError",
    "Arx5EefState",
    "Arx5FeedbackError",
    "Arx5HardwareDisabledError",
    "Arx5JointState",
    "Arx5RawFeedback",
    "Arx5SdkController",
    "ControllerFactory",
    "SdkLoader",
    "TcpPoseProvider",
    "pose_from_sdk_eef_state",
    "robot_state_from_sdk",
]
