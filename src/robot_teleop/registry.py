"""Explicit component registry and hardware-free composition root."""

from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

from robot_teleop.config import TeleopProfile
from robot_teleop.controllers.cartesian_position import CartesianPositionController
from robot_teleop.core.models import TeleopInput, Vector3
from robot_teleop.core.pipeline import TeleopPipeline
from robot_teleop.core.ports import TeleopInputSource, Transport
from robot_teleop.inputs.arkit import ArkitInputSource
from robot_teleop.inputs.mock import MockInputSource
from robot_teleop.inputs.replay import ReplayInputSource
from robot_teleop.kinematics.config import load_model_config
from robot_teleop.kinematics.pinocchio import PinocchioKinematics
from robot_teleop.policies.no_compliance import NoCompliancePolicy
from robot_teleop.policies.relative_pose import RelativePosePolicy
from robot_teleop.protocol.packet import EStopResetRequestPacket
from robot_teleop.robots.arx5 import Arx5Backend, Arx5BackendConfig
from robot_teleop.robots.arx5_safety import Arx5SafetyEnvelope
from robot_teleop.robots.mock import MockRobotBackend
from robot_teleop.runtime.application import TeleopApplication
from robot_teleop.runtime.debug_dashboard import DebugDashboardServer, DebugTelemetry
from robot_teleop.runtime.estop_reset import EStopResetGate
from robot_teleop.runtime.latest_value import LatestValue
from robot_teleop.runtime.mapping_settings import MappingSettingsStore
from robot_teleop.runtime.metrics import MetricsWorker, RuntimeMetrics
from robot_teleop.runtime.receiver import InputReceiver
from robot_teleop.runtime.servo_loop import ServoLoop, ServoLoopConfig
from robot_teleop.runtime.watchdog import Watchdog
from robot_teleop.transports.mock import MockTransport
from robot_teleop.transports.udp import UdpTransport

INPUTS: dict[str, type[object]] = {
    "arkit": ArkitInputSource,
    "mock": MockInputSource,
    "replay": ReplayInputSource,
}

# Live ARX5 pilot workspace: keep only the base-plane lower bound for TCP Z.
# X/Y and upper Z remain intentionally broad so this boundary does not behave
# like a tabletop clearance margin during grasping tests.
_ARX5_BASE_PLANE_WORKSPACE_MINIMUM_M = Vector3(-10.0, -10.0, 0.0)
_ARX5_BASE_PLANE_WORKSPACE_MAXIMUM_M = Vector3(10.0, 10.0, 10.0)

TRANSPORTS: dict[str, type[object]] = {
    "mock": MockTransport,
    "udp": UdpTransport,
}

POLICIES: dict[str, type[object]] = {
    "relative-pose": RelativePosePolicy,
}

ROBOTS: dict[str, type[object]] = {
    "arx5": Arx5Backend,
    "mock": MockRobotBackend,
}

CONTROLLERS: dict[str, type[object]] = {
    "cartesian-position": CartesianPositionController,
}


class ComponentUnavailableError(ValueError):
    pass


class HardwareEnableRequiredError(PermissionError):
    pass


@dataclass(frozen=True, slots=True)
class ApplicationBundle:
    application: TeleopApplication
    input_source: TeleopInputSource
    transport: Transport | None
    robot: MockRobotBackend | Arx5Backend
    metrics: RuntimeMetrics
    mapping_settings: MappingSettingsStore
    profile: TeleopProfile
    hardware_enabled: bool = False


def _require_registered(
    name: str, registry: dict[str, type[object]], kind: str
) -> None:
    if name not in registry:
        raise ComponentUnavailableError(f"{kind} component is unavailable: {name}")


def create_application(
    profile: TeleopProfile,
    *,
    enable_hardware: bool,
    robot_interface: str | None = None,
    allow_active_command: bool = False,
    allow_gripper_command: bool = False,
    gripper_only: bool = False,
    gripper_open_readout: float | None = None,
    allow_unverified_safety_envelope: bool = False,
    metrics_sample_capacity: int = 4096,
    debug_web_host: str | None = None,
    debug_web_port: int = 8080,
) -> ApplicationBundle:
    if profile.robot_name != "mock" and not enable_hardware:
        raise HardwareEnableRequiredError(
            "non-mock robots require the explicit --enable-hardware flag"
        )
    _require_registered(profile.input_name, INPUTS, "input")
    _require_registered(profile.transport_name, TRANSPORTS, "transport")
    _require_registered(profile.policy_name, POLICIES, "policy")
    _require_registered(profile.robot_name, ROBOTS, "robot")
    _require_registered(profile.controller_name, CONTROLLERS, "controller")
    metrics = RuntimeMetrics(sample_capacity=metrics_sample_capacity)
    debug_telemetry = DebugTelemetry()
    mapping_settings = MappingSettingsStore(profile.mapping)
    source: TeleopInputSource
    transport_component: Transport | None = None
    estop_reset_gate: EStopResetGate | None = None
    reset_acknowledger: Callable[[EStopResetRequestPacket], None] | None = None
    if profile.input_name == "mock":
        if profile.transport_name != "mock":
            raise ComponentUnavailableError("mock input requires mock transport")
        source = MockInputSource()
    elif profile.input_name == "arkit":
        if profile.transport_name != "udp":
            raise ComponentUnavailableError("arkit input requires udp transport")
        if profile.udp is None:
            raise ComponentUnavailableError("udp transport requires network config")
        transport = UdpTransport(
            profile.udp,
            event_callback=metrics.record_protocol_event,
        )
        estop_reset_gate = EStopResetGate()
        arkit_source = ArkitInputSource(
            transport,
            event_callback=metrics.record_protocol_event,
            settings_callback=mapping_settings.update,
            estop_reset_request_callback=estop_reset_gate.record_request,
        )
        source = arkit_source
        reset_acknowledger = arkit_source.acknowledge_estop_reset
        transport_component = transport
    else:
        raise ComponentUnavailableError(
            "the replay input requires explicit samples through the Python API"
        )

    if profile.robot_name == "mock":
        robot: MockRobotBackend | Arx5Backend = MockRobotBackend()
    elif profile.robot_name == "arx5":
        if robot_interface is None or not robot_interface.strip():
            raise ComponentUnavailableError(
                "arx5 requires an explicit --robot-interface value"
            )
        model_config = load_model_config(
            Path(__file__).resolve().parents[2] / "configs/phone/robots/arx5.yaml"
        )
        kinematics = PinocchioKinematics(model_config.model)
        safety_envelope = None
        if allow_gripper_command and not allow_active_command:
            raise ComponentUnavailableError(
                "ARX5 gripper command requires --allow-active-command"
            )
        if allow_active_command:
            if not allow_unverified_safety_envelope:
                raise ComponentUnavailableError(
                    "ARX5 active command requires --allow-unverified-safety-envelope"
                )
            limits = model_config.model.joint_limits
            safety_envelope = Arx5SafetyEnvelope(
                position_lower_rad=limits.lower_rad,
                position_upper_rad=limits.upper_rad,
                max_velocity_rad_s=limits.velocity_max_rad_s,
                max_acceleration_rad_s2=(0.70,) * limits.joint_count,
                max_jerk_rad_s3=(3.0,) * limits.joint_count,
                max_torque_nm=(30.0, 40.0, 30.0, 15.0, 10.0, 10.0),
                workspace_minimum_m=_ARX5_BASE_PLANE_WORKSPACE_MINIMUM_M,
                workspace_maximum_m=_ARX5_BASE_PLANE_WORKSPACE_MAXIMUM_M,
                tcp_linear_velocity_max_m_s=(
                    profile.target_safety.maximum_linear_velocity_m_s
                ),
                tcp_angular_velocity_max_rad_s=(
                    profile.target_safety.maximum_angular_velocity_rad_s
                ),
                evidence_source=(
                    "fast operator default using arx5.yaml joint velocity limits "
                    "and arkit_arx5 profile Cartesian limits"
                ),
            )
        robot = Arx5Backend(
            Arx5BackendConfig(
                hardware_enabled=True,
                interface_name=robot_interface,
                active_command_enabled=allow_active_command,
                gripper_command_enabled=allow_gripper_command,
                gripper_only=gripper_only,
                gripper_open_readout=gripper_open_readout,
                safety_envelope=safety_envelope,
            ),
            kinematics=kinematics,
            tcp_pose_provider=kinematics.forward_pose,
        )
    else:
        raise ComponentUnavailableError(
            f"robot component is registered but has no composition rule: "
            f"{profile.robot_name}"
        )
    latest: LatestValue[TeleopInput] = LatestValue()
    receiver = InputReceiver(
        source,
        latest,
        metrics,
        poll_interval_s=profile.receiver_poll_interval_s,
        on_input_accepted=(
            lambda input_state: (
                debug_telemetry.record_input(input_state),
                None
                if estop_reset_gate is None
                else estop_reset_gate.observe_input(input_state),
            )[-1]
        ),
    )
    pipeline = TeleopPipeline(
        policy=RelativePosePolicy(
            profile.mapping,
            mapping_provider=mapping_settings.load,
        ),
        compliance=NoCompliancePolicy(),
        input_capabilities=source.capabilities,
        safety_limits=profile.target_safety,
    )
    servo = ServoLoop(
        latest=latest,
        pipeline=pipeline,
        controller=CartesianPositionController(robot.capabilities),
        backend=robot,
        watchdog=Watchdog(profile.watchdog),
        metrics=metrics,
        config=ServoLoopConfig(
            rate_hz=profile.rate_hz,
            state_stale_timeout_ms=profile.state_stale_timeout_ms,
        ),
        telemetry=debug_telemetry,
    )
    debug_server = None
    if debug_web_host is not None:
        debug_server = DebugDashboardServer(
            debug_telemetry,
            state_provider=lambda: servo.state.value,
            metrics_provider=lambda: metrics.report(),
            host=debug_web_host,
            port=debug_web_port,
        )
    application = TeleopApplication(
        receiver=receiver,
        servo=servo,
        metrics_worker=MetricsWorker(metrics, interval_s=0.1),
        backend=robot,
        estop_reset_gate=estop_reset_gate,
        reset_acknowledger=reset_acknowledger,
        reset_backend=(
            robot.rearm_after_safe_stop
            if estop_reset_gate is not None and isinstance(robot, MockRobotBackend)
            else (
                robot.recover_after_safe_stop
                if estop_reset_gate is not None and isinstance(robot, Arx5Backend)
                else None
            )
        ),
        debug_telemetry=debug_telemetry,
        debug_server=debug_server,
    )
    return ApplicationBundle(
        application=application,
        input_source=source,
        transport=transport_component,
        robot=robot,
        metrics=metrics,
        mapping_settings=mapping_settings,
        profile=profile,
        hardware_enabled=profile.robot_name != "mock" and enable_hardware,
    )
