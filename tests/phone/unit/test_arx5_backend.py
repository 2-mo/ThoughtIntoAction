from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass

import pytest

from embodied.phone.core.frames import quaternion_angular_distance
from embodied.phone.core.models import ControlMode, Pose, RobotCommand, Vector3
from embodied.phone.robots.arx5 import (
    Arx5Backend,
    Arx5BackendConfig,
    Arx5BackendError,
    Arx5CommandError,
    Arx5FeedbackError,
    Arx5HardwareDisabledError,
    pose_from_sdk_eef_state,
    robot_state_from_sdk,
)
from embodied.phone.robots.arx5_safety import Arx5SafetyEnvelope


@dataclass
class FakeJointState:
    timestamp: float = 1.0
    gripper_pos: float = 0.01
    gripper_vel: float = 0.0
    gripper_torque: float = 0.0
    position: tuple[float, ...] = (0.0, 0.1, 0.2, 0.0, 0.0, 0.0)
    velocity: tuple[float, ...] = (0.0,) * 6
    torque_values: tuple[float, ...] = (0.0,) * 6
    fail_position_read: bool = False

    def pos(self) -> tuple[float, ...]:
        if self.fail_position_read:
            raise RuntimeError("injected position read failure")
        return self.position

    def vel(self) -> tuple[float, ...]:
        return self.velocity

    def torque(self) -> tuple[float, ...]:
        return self.torque_values


@dataclass
class FakeEefState:
    timestamp: float = 1.0
    gripper_pos: float = 0.01
    gripper_vel: float = 0.0
    gripper_torque: float = 0.0
    pose: tuple[float, ...] = (0.4, 0.0, 0.3, 0.0, 0.0, math.pi / 2.0)

    def pose_6d(self) -> tuple[float, ...]:
        return self.pose


@dataclass
class FakeRobotConfig:
    gripper_width: float = 0.088


@dataclass
class FakeControllerConfig:
    default_kp: tuple[float, ...] = (1.0,) * 6
    default_kd: tuple[float, ...] = (0.1,) * 6
    default_gripper_kp: float = 1.0
    default_gripper_kd: float = 0.1


class FakeSdk:
    class JointState:
        def __init__(
            self,
            position: object,
            velocity: object,
            torque: object,
            gripper_pos: float,
            gripper_vel: float,
            gripper_torque: float,
        ) -> None:
            self.position = tuple(float(value) for value in position)
            self.velocity = tuple(float(value) for value in velocity)
            self.torque = tuple(float(value) for value in torque)
            self.gripper_pos = float(gripper_pos)
            self.gripper_vel = float(gripper_vel)
            self.gripper_torque = float(gripper_torque)

    class Gain:
        def __init__(
            self,
            kp: object,
            kd: object,
            gripper_kp: float,
            gripper_kd: float,
        ) -> None:
            self.kp = tuple(float(value) for value in kp)
            self.kd = tuple(float(value) for value in kd)
            self.gripper_kp = float(gripper_kp)
            self.gripper_kd = float(gripper_kd)


class FakeController:
    def __init__(
        self,
        *,
        advance_timestamp: bool = True,
        fail_damping: bool = False,
        fail_joint_read: bool = False,
        home_position: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    ) -> None:
        self.joint = FakeJointState()
        self.eef = FakeEefState()
        self.advance_timestamp = advance_timestamp
        self.fail_damping = fail_damping
        self.fail_joint_read = fail_joint_read
        self.home_position = home_position
        self.damping_calls = 0
        self.home_calls = 0
        self.gains: list[object] = []
        self.joint_commands: list[object] = []

    def get_joint_state(self) -> FakeJointState:
        if self.fail_joint_read:
            raise RuntimeError("injected joint read failure")
        if self.advance_timestamp:
            self.joint.timestamp += 0.01
        return self.joint

    def set_to_damping(self) -> None:
        self.damping_calls += 1
        if self.fail_damping:
            raise RuntimeError("injected damping failure")

    def reset_to_home(self) -> None:
        self.home_calls += 1
        self.joint.position = self.home_position
        self.eef.pose = (0.35, 0.0, 0.25, 0.0, 0.0, 0.0)

    def get_home_pose(self) -> tuple[float, ...]:
        return self.home_position

    def get_robot_config(self) -> FakeRobotConfig:
        return FakeRobotConfig()

    def get_controller_config(self) -> FakeControllerConfig:
        return FakeControllerConfig()

    def set_gain(self, gain: object) -> None:
        self.gains.append(gain)

    def set_joint_cmd(self, command: object) -> None:
        self.joint_commands.append(command)
        position = getattr(command, "position", None)
        if position is not None:
            self.joint.position = tuple(float(value) for value in position)


def fake_factory(
    controller: FakeController,
) -> Callable[[object, Arx5BackendConfig], FakeController]:
    def factory(_sdk: object, _config: Arx5BackendConfig) -> FakeController:
        return controller

    return factory


def test_backend_config_records_sdk_io_mode() -> None:
    config = Arx5BackendConfig(
        hardware_enabled=True,
        interface_name="fake0",
        background_send_recv=True,
        interpolation_method="cubic",
    )

    assert config.background_send_recv is True
    assert config.interpolation_method == "cubic"

    with pytest.raises(ValueError, match="interpolation_method"):
        Arx5BackendConfig(interpolation_method="nearest")

    with pytest.raises(ValueError, match="simple CAN name"):
        Arx5BackendConfig(interface_name="../../can0")

    with pytest.raises(ValueError, match="gripper_open_readout"):
        Arx5BackendConfig(gripper_open_readout=0.1)


def arx5_envelope() -> Arx5SafetyEnvelope:
    return Arx5SafetyEnvelope(
        position_lower_rad=(-1.0,) * 6,
        position_upper_rad=(1.0,) * 6,
        max_velocity_rad_s=(1.0,) * 6,
        max_acceleration_rad_s2=(2.0,) * 6,
        max_jerk_rad_s3=(3.0,) * 6,
        max_torque_nm=(4.0,) * 6,
        workspace_minimum_m=Vector3(0.2, -0.2, 0.2),
        workspace_maximum_m=Vector3(0.6, 0.2, 0.5),
        tcp_linear_velocity_max_m_s=0.1,
        tcp_angular_velocity_max_rad_s=0.2,
        singularity_warn=0.2,
        singularity_stop=0.1,
        evidence_source="fake test envelope",
    )


def test_disabled_backend_rejects_before_loading_sdk() -> None:
    loader_calls = 0

    def loader() -> object:
        nonlocal loader_calls
        loader_calls += 1
        return object()

    backend = Arx5Backend(sdk_loader=loader)

    with pytest.raises(Arx5HardwareDisabledError):
        backend.connect()

    assert loader_calls == 0
    assert backend.connected is False


def test_state_conversion_keeps_sdk_boundary_and_xyzw_orientation() -> None:
    pose = pose_from_sdk_eef_state(FakeEefState())
    state = robot_state_from_sdk(
        FakeJointState(),
        pose,
        timestamp_ns=123,
        joint_count=6,
    )

    assert state.timestamp_ns == 123
    assert state.joint_position == pytest.approx((0.0, 0.1, 0.2, 0.0, 0.0, 0.0))

    assert state.tcp_pose.position.as_tuple() == pytest.approx((0.4, 0.0, 0.3))
    expected = (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))
    assert state.tcp_pose.orientation.as_tuple() == pytest.approx(expected)

    # The canonical quaternion is a unit quaternion and the conversion is not
    # accidentally interpreted as wxyz.
    assert (
        quaternion_angular_distance(
            state.tcp_pose.orientation,
            type(state.tcp_pose.orientation)(*expected),
        )
        < 1e-9
    )


def test_backend_exposes_calibrated_gripper_feedback() -> None:
    controller = FakeController()
    controller.joint.gripper_pos = -1.62423
    backend = Arx5Backend(
        Arx5BackendConfig(
            hardware_enabled=True,
            interface_name="fake0",
            gripper_open_readout=-3.24846,
        ),
        sdk_loader=lambda: FakeSdk(),
        controller_factory=fake_factory(controller),
        tcp_pose_provider=lambda _joint_position: pose_from_sdk_eef_state(
            controller.eef
        ),
    )

    backend.connect()

    assert backend.state().gripper == pytest.approx(0.5)


def test_state_conversion_uses_documented_rpy_composition_order() -> None:
    roll, pitch, yaw = (0.3, -0.2, 0.4)
    state = pose_from_sdk_eef_state(
        FakeEefState(pose=(0.4, 0.0, 0.3, roll, pitch, yaw))
    )
    half_roll = roll * 0.5
    half_pitch = pitch * 0.5
    half_yaw = yaw * 0.5
    expected = (
        math.sin(half_roll) * math.cos(half_pitch) * math.cos(half_yaw)
        - math.cos(half_roll) * math.sin(half_pitch) * math.sin(half_yaw),
        math.cos(half_roll) * math.sin(half_pitch) * math.cos(half_yaw)
        + math.sin(half_roll) * math.cos(half_pitch) * math.sin(half_yaw),
        math.cos(half_roll) * math.cos(half_pitch) * math.sin(half_yaw)
        - math.sin(half_roll) * math.sin(half_pitch) * math.cos(half_yaw),
        math.cos(half_roll) * math.cos(half_pitch) * math.cos(half_yaw)
        + math.sin(half_roll) * math.sin(half_pitch) * math.sin(half_yaw),
    )

    assert state.orientation.as_tuple() == pytest.approx(expected)


def test_fake_sdk_lifecycle_is_idempotent_and_active_path_fails_safe() -> None:
    controller = FakeController()
    backend = Arx5Backend(
        Arx5BackendConfig(hardware_enabled=True, interface_name="fake0"),
        sdk_loader=lambda: object(),
        controller_factory=fake_factory(controller),
        tcp_pose_provider=lambda _joint_position: pose_from_sdk_eef_state(
            controller.eef
        ),
    )
    backend.connect()
    backend.connect()
    assert backend.connected is True
    assert backend.state().joint_position[2] == pytest.approx(0.2)

    command = RobotCommand(
        mode=ControlMode.CARTESIAN_POSITION,
        target_pose=backend.state().tcp_pose,
        target_twist=None,
        target_joint_position=None,
        gripper=None,
        velocity_scale=1.0,
        timestamp_ns=1,
    )
    with pytest.raises(Arx5CommandError, match="active-command gate"):
        backend.send(command)
    assert backend.safe_stopped is True
    assert backend.safe_stop_count == 1
    assert controller.damping_calls == 1

    backend.disconnect()
    backend.disconnect()
    assert backend.connected is False


def test_feedback_timestamp_stall_is_rejected() -> None:
    controller = FakeController(advance_timestamp=False)
    clock_values = iter((0, 200_000_000))
    backend = Arx5Backend(
        Arx5BackendConfig(
            hardware_enabled=True,
            interface_name="fake0",
            feedback_stale_timeout_ms=100.0,
        ),
        sdk_loader=lambda: object(),
        controller_factory=fake_factory(controller),
        tcp_pose_provider=lambda _joint_position: pose_from_sdk_eef_state(
            controller.eef
        ),
        clock=lambda: next(clock_values),
    )
    backend.connect()

    with pytest.raises(Arx5FeedbackError, match="stopped advancing"):
        backend.state()

    assert backend.safe_stopped is True
    assert backend.safe_stop_count == 1
    assert backend.faulted is True
    assert controller.damping_calls == 1
    with pytest.raises(Arx5CommandError, match="faulted"):
        backend.send(
            RobotCommand(
                mode=ControlMode.HOLD,
                target_pose=None,
                target_twist=None,
                target_joint_position=None,
                gripper=None,
                velocity_scale=0.0,
                timestamp_ns=2,
            )
        )


def test_home_moves_to_joint_zero_without_sdk_reset() -> None:
    controller = FakeController()
    backend = Arx5Backend(
        Arx5BackendConfig(
            hardware_enabled=True,
            interface_name="fake0",
            active_command_enabled=True,
            control_rate_hz=10_000.0,
            feedback_stale_timeout_ms=100.0,
            max_joint_target_step_rad=1.0,
        ),
        sdk_loader=FakeSdk,
        controller_factory=fake_factory(controller),
        tcp_pose_provider=lambda _joint_position: pose_from_sdk_eef_state(
            controller.eef
        ),
    )
    backend.connect()
    assert len(controller.gains) == 1

    state = backend.send(
        RobotCommand(
            mode=ControlMode.HOME,
            target_pose=None,
            target_twist=None,
            target_joint_position=None,
            gripper=None,
            velocity_scale=0.0,
            timestamp_ns=1,
        )
    )

    assert controller.home_calls == 0
    assert len(controller.gains) == 2
    assert len(controller.joint_commands) >= 2
    assert state.joint_position == pytest.approx((0.0,) * 6)
    assert backend.safe_stopped is False
    assert backend.faulted is False
    backend.send(
        RobotCommand(
            mode=ControlMode.HOLD,
            target_pose=None,
            target_twist=None,
            target_joint_position=None,
            gripper=None,
            velocity_scale=0.0,
            timestamp_ns=2,
        )
    )
    assert backend.safe_stopped is False
    assert controller.joint_commands[-1].position == pytest.approx((0.0,) * 6)


def test_recovery_returns_joint_zero_before_reconnecting_a_safe_stopped_arm() -> None:
    controller = FakeController()
    backend = Arx5Backend(
        Arx5BackendConfig(
            hardware_enabled=True,
            interface_name="fake0",
            active_command_enabled=True,
            control_rate_hz=10_000.0,
            max_joint_target_step_rad=1.0,
        ),
        sdk_loader=FakeSdk,
        controller_factory=fake_factory(controller),
        tcp_pose_provider=lambda _joint_position: pose_from_sdk_eef_state(
            controller.eef
        ),
    )
    backend.connect()
    backend.safe_stop("test E-stop")

    backend.recover_after_safe_stop()

    assert controller.home_calls == 0
    assert controller.damping_calls == 2
    assert controller.joint_commands[-1].position == pytest.approx((0.0,) * 6)
    assert backend.connected is True
    assert backend.safe_stopped is False


def test_active_joint_target_moves_toward_zero_without_cartesian_ik() -> None:
    controller = FakeController()
    backend = Arx5Backend(
        Arx5BackendConfig(
            hardware_enabled=True,
            interface_name="fake0",
            active_command_enabled=True,
        ),
        sdk_loader=FakeSdk,
        controller_factory=fake_factory(controller),
        tcp_pose_provider=lambda _joint_position: pose_from_sdk_eef_state(
            controller.eef
        ),
    )
    backend.connect()

    state = backend.send(
        RobotCommand(
            mode=ControlMode.JOINT_POSITION,
            target_pose=None,
            target_twist=None,
            target_joint_position=(0.0,) * 6,
            gripper=None,
            velocity_scale=0.25,
            timestamp_ns=1,
        )
    )

    assert 0.0 < state.joint_position[1] < 0.1
    assert 0.0 < state.joint_position[2] < 0.2
    assert backend.safe_stopped is False


def test_feedback_envelope_violation_does_not_latch_control() -> None:
    controller = FakeController()
    backend = Arx5Backend(
        Arx5BackendConfig(
            hardware_enabled=True,
            interface_name="fake0",
            safety_envelope=arx5_envelope(),
        ),
        sdk_loader=lambda: object(),
        controller_factory=fake_factory(controller),
        tcp_pose_provider=lambda _joint_position: pose_from_sdk_eef_state(
            controller.eef
        ),
    )
    backend.connect()
    controller.joint.velocity = (1.1, 0.0, 0.0, 0.0, 0.0, 0.0)

    state = backend.state()

    assert state.joint_velocity[0] == pytest.approx(1.1)
    assert controller.damping_calls == 0
    assert backend.safe_stopped is False
    assert backend.faulted is False


def test_sdk_feedback_read_failure_latches_and_damps() -> None:
    controller = FakeController()
    backend = Arx5Backend(
        Arx5BackendConfig(hardware_enabled=True, interface_name="fake0"),
        sdk_loader=lambda: object(),
        controller_factory=fake_factory(controller),
        tcp_pose_provider=lambda _joint_position: pose_from_sdk_eef_state(
            controller.eef
        ),
    )
    backend.connect()
    controller.fail_joint_read = True

    with pytest.raises(Arx5FeedbackError, match="SDK feedback read failed"):
        backend.state()

    assert backend.faulted is True
    assert backend.safe_stopped is True
    assert controller.damping_calls == 1


def test_sdk_feedback_member_failure_latches_and_damps() -> None:
    controller = FakeController()
    backend = Arx5Backend(
        Arx5BackendConfig(hardware_enabled=True, interface_name="fake0"),
        sdk_loader=lambda: object(),
        controller_factory=fake_factory(controller),
        tcp_pose_provider=lambda _joint_position: pose_from_sdk_eef_state(
            controller.eef
        ),
    )
    backend.connect()
    controller.joint.fail_position_read = True

    with pytest.raises(Arx5FeedbackError, match="pos read failed"):
        backend.state()

    assert backend.faulted is True
    assert backend.safe_stopped is True
    assert controller.damping_calls == 1


def test_fk_feedback_failure_latches_and_damps() -> None:
    controller = FakeController()
    fail_fk = False

    def provider(_joint_position: tuple[float, ...]) -> Pose:
        if fail_fk:
            raise RuntimeError("injected FK failure")
        return pose_from_sdk_eef_state(controller.eef)

    backend = Arx5Backend(
        Arx5BackendConfig(hardware_enabled=True, interface_name="fake0"),
        sdk_loader=lambda: object(),
        controller_factory=fake_factory(controller),
        tcp_pose_provider=provider,
    )
    backend.connect()
    fail_fk = True

    with pytest.raises(Arx5FeedbackError, match="FK provider rejected feedback"):
        backend.state()

    assert backend.faulted is True
    assert backend.safe_stopped is True
    assert controller.damping_calls == 1


def test_command_envelope_violation_damps_before_active_path_rejection() -> None:
    controller = FakeController()
    backend = Arx5Backend(
        Arx5BackendConfig(
            hardware_enabled=True,
            interface_name="fake0",
            safety_envelope=arx5_envelope(),
        ),
        sdk_loader=lambda: object(),
        controller_factory=fake_factory(controller),
        tcp_pose_provider=lambda _joint_position: pose_from_sdk_eef_state(
            controller.eef
        ),
    )
    backend.connect()
    orientation = backend.state().tcp_pose.orientation
    unsafe = RobotCommand(
        mode=ControlMode.CARTESIAN_POSITION,
        target_pose=Pose(Vector3(0.7, 0.0, 0.3), orientation),
        target_twist=None,
        target_joint_position=None,
        gripper=None,
        velocity_scale=1.0,
        timestamp_ns=1,
    )

    with pytest.raises(Arx5CommandError, match="robot-local envelope"):
        backend.send(unsafe)

    assert backend.safe_stopped is True
    assert backend.safe_stop_count == 1
    assert backend.faulted is True
    assert controller.damping_calls == 1


def test_feedback_fault_records_damping_failure_without_claiming_safe_stop() -> None:
    controller = FakeController(advance_timestamp=False, fail_damping=True)
    clock_values = iter((0, 200_000_000))
    backend = Arx5Backend(
        Arx5BackendConfig(
            hardware_enabled=True,
            interface_name="fake0",
            feedback_stale_timeout_ms=100.0,
        ),
        sdk_loader=lambda: object(),
        controller_factory=fake_factory(controller),
        tcp_pose_provider=lambda _joint_position: pose_from_sdk_eef_state(
            controller.eef
        ),
        clock=lambda: next(clock_values),
    )
    backend.connect()

    with pytest.raises(Arx5FeedbackError, match="stopped advancing"):
        backend.state()

    assert backend.safe_stopped is False
    assert backend.safe_stop_count == 0
    assert backend.faulted is True
    assert backend.last_safe_stop_error == "injected damping failure"
    assert controller.damping_calls == 1
    backend.disconnect()
    assert controller.damping_calls == 1


def test_enabled_backend_requires_measured_joint_fk_before_loading_sdk() -> None:
    loader_calls = 0

    def loader() -> object:
        nonlocal loader_calls
        loader_calls += 1
        return object()

    backend = Arx5Backend(
        Arx5BackendConfig(hardware_enabled=True, interface_name="fake0"),
        sdk_loader=loader,
    )

    with pytest.raises(Arx5BackendError, match="FK provider"):
        backend.connect()

    assert loader_calls == 0


def test_state_only_disconnect_can_skip_damping_command() -> None:
    controller = FakeController()
    backend = Arx5Backend(
        Arx5BackendConfig(
            hardware_enabled=True,
            interface_name="fake0",
            disconnect_damping_enabled=False,
            damping_on_connect_error=False,
        ),
        sdk_loader=lambda: object(),
        controller_factory=fake_factory(controller),
        tcp_pose_provider=lambda _joint_position: pose_from_sdk_eef_state(
            controller.eef
        ),
    )

    backend.connect()
    raw = backend.raw_feedback
    assert raw is not None
    assert raw.gripper_position_raw == pytest.approx(0.01)
    backend.disconnect()

    assert backend.connected is False
    assert backend.safe_stop_count == 0
    assert controller.damping_calls == 0
