import math

import pytest

from embodied.phone.core.capabilities import InputCapabilities
from embodied.phone.core.frames import (
    quaternion_angular_distance,
    quaternion_from_axis_angle,
)
from embodied.phone.core.models import (
    ControlMode,
    Pose,
    Quaternion,
    RobotState,
    RobotTarget,
    TeleopInput,
    TrackingQuality,
    Vector3,
)
from embodied.phone.core.safety import (
    InputSafetyLimits,
    InputSafetyValidator,
    SafetyViolation,
    TargetSafetyGate,
    TargetSafetyLimits,
)

CAPABILITIES = InputCapabilities(
    pose_6dof=True,
    linear_velocity=False,
    angular_velocity=False,
    analog_gripper=False,
    haptics=False,
    dual_hand=False,
)
ZERO_POSITION = Vector3(0.0, 0.0, 0.0)
IDENTITY = Quaternion(0.0, 0.0, 0.0, 1.0)


def make_input(
    sequence: int,
    *,
    session_id: int = 1,
    timestamp_ns: int | None = None,
    position: Vector3 = ZERO_POSITION,
    orientation: Quaternion = IDENTITY,
    tracking: TrackingQuality = TrackingQuality.NORMAL,
) -> TeleopInput:
    return TeleopInput(
        sequence=sequence,
        session_id=session_id,
        timestamp_ns=sequence if timestamp_ns is None else timestamp_ns,
        pose=Pose(position, orientation),
        linear_velocity=None,
        angular_velocity=None,
        clutch_active=False,
        recenter_requested=False,
        estop_requested=False,
        gripper=None,
        tracking_quality=tracking,
        source_name="mock",
    )


def make_robot(pose: Pose) -> RobotState:
    return RobotState(
        timestamp_ns=1,
        joint_position=(0.0,) * 6,
        joint_velocity=(0.0,) * 6,
        joint_torque=None,
        tcp_pose=pose,
        tcp_twist=None,
        gripper=None,
        external_wrench=None,
    )


def make_position_target(pose: Pose) -> RobotTarget:
    return RobotTarget(
        mode=ControlMode.CARTESIAN_POSITION,
        target_pose=pose,
        target_twist=None,
        target_joint_position=None,
        gripper=None,
        velocity_scale=1.0,
        timestamp_ns=1,
    )


def test_input_validator_rejects_duplicate_and_out_of_order_sequence() -> None:
    validator = InputSafetyValidator(InputSafetyLimits(), CAPABILITIES)
    validator.validate(make_input(2))

    with pytest.raises(SafetyViolation, match="sequence"):
        validator.validate(make_input(2, timestamp_ns=3))
    with pytest.raises(SafetyViolation, match="sequence"):
        validator.validate(make_input(1, timestamp_ns=4))


def test_input_validator_resets_ordering_for_new_session() -> None:
    validator = InputSafetyValidator(InputSafetyLimits(), CAPABILITIES)
    validator.validate(make_input(50, session_id=1))

    validator.validate(make_input(0, session_id=2, timestamp_ns=0))


def test_input_validator_accepts_uint32_sequence_wrap() -> None:
    validator = InputSafetyValidator(InputSafetyLimits(), CAPABILITIES)
    validator.validate(make_input(0xFFFFFFFF, timestamp_ns=10))

    validator.validate(make_input(0, timestamp_ns=20))


def test_input_validator_rejects_position_and_rotation_jumps() -> None:
    validator = InputSafetyValidator(
        InputSafetyLimits(
            maximum_position_jump_m=0.1,
            maximum_rotation_jump_rad=0.2,
        ),
        CAPABILITIES,
    )
    validator.validate(make_input(1))

    with pytest.raises(SafetyViolation, match="position jump"):
        validator.validate(make_input(2, position=Vector3(0.2, 0.0, 0.0)))

    validator.reset()
    validator.validate(make_input(1))
    rotated = quaternion_from_axis_angle(Vector3(0.0, 0.0, 1.0), math.pi / 2.0)
    with pytest.raises(SafetyViolation, match="rotation jump"):
        validator.validate(make_input(2, orientation=rotated))


def test_input_validator_rebases_after_limited_tracking() -> None:
    validator = InputSafetyValidator(
        InputSafetyLimits(maximum_position_jump_m=0.1),
        CAPABILITIES,
    )
    validator.validate(make_input(1))
    validator.validate(
        make_input(
            2,
            position=Vector3(1.0, 0.0, 0.0),
            tracking=TrackingQuality.LIMITED,
        )
    )
    validator.validate(make_input(3, position=Vector3(1.0, 0.0, 0.0)))


def test_input_validator_enforces_capability_claims() -> None:
    validator = InputSafetyValidator(InputSafetyLimits(), CAPABILITIES)
    unexpected_velocity = TeleopInput(
        sequence=1,
        session_id=1,
        timestamp_ns=1,
        pose=Pose(Vector3(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
        linear_velocity=Vector3(1.0, 0.0, 0.0),
        angular_velocity=None,
        clutch_active=False,
        recenter_requested=False,
        estop_requested=False,
        gripper=None,
        tracking_quality=TrackingQuality.NORMAL,
        source_name="mock",
    )

    with pytest.raises(SafetyViolation, match="linear velocity"):
        validator.validate(unexpected_velocity)


def test_target_gate_limits_new_targets_and_does_not_chase_stale_input() -> None:
    reference = Pose(ZERO_POSITION, IDENTITY)
    desired = Pose(
        Vector3(0.2, 0.0, 0.0),
        quaternion_from_axis_angle(Vector3(0.0, 0.0, 1.0), 0.4),
    )
    gate = TargetSafetyGate(
        TargetSafetyLimits(
            maximum_target_step_m=0.1,
            maximum_rotation_step_rad=0.2,
            maximum_linear_velocity_m_s=0.5,
            maximum_angular_velocity_rad_s=0.5,
        )
    )
    robot = make_robot(reference)
    gate.reset(reference)

    limited = gate.validate(
        make_position_target(desired),
        robot,
        dt=0.1,
        is_new_input=True,
    )
    assert limited.target_pose is not None
    assert limited.target_pose.position.x == pytest.approx(0.05)
    assert quaternion_angular_distance(
        limited.target_pose.orientation, IDENTITY
    ) == pytest.approx(0.05)

    held = gate.validate(
        make_position_target(desired),
        robot,
        dt=0.005,
        is_new_input=False,
    )
    assert held.target_pose == limited.target_pose


def test_target_gate_rejects_workspace_even_when_motion_is_rate_limited() -> None:
    reference = Pose(Vector3(0.4, 0.0, 0.3), IDENTITY)
    gate = TargetSafetyGate(
        TargetSafetyLimits(
            maximum_target_step_m=0.05,
            maximum_rotation_step_rad=0.2,
            maximum_linear_velocity_m_s=0.1,
            maximum_angular_velocity_rad_s=0.5,
            workspace_minimum_m=Vector3(0.0, -0.5, 0.0),
            workspace_maximum_m=Vector3(0.5, 0.5, 0.8),
        )
    )
    gate.reset(reference)

    with pytest.raises(SafetyViolation, match="workspace"):
        gate.validate(
            make_position_target(Pose(Vector3(0.8, 0.0, 0.3), IDENTITY)),
            make_robot(reference),
            dt=0.1,
        )
