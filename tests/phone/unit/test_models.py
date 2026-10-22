import math
from dataclasses import FrozenInstanceError

import pytest

from embodied.devices.phone.core.models import (
    ControlMode,
    Pose,
    Quaternion,
    RobotState,
    RobotTarget,
    TeleopInput,
    TrackingQuality,
    Vector3,
)

IDENTITY = Quaternion(0.0, 0.0, 0.0, 1.0)
ORIGIN = Vector3(0.0, 0.0, 0.0)


def test_vector_rejects_non_finite_component() -> None:
    with pytest.raises(ValueError, match="finite"):
        Vector3(math.nan, 0.0, 0.0)


def test_quaternion_is_normalized_in_xyzw_order() -> None:
    quaternion = Quaternion(0.0, 0.0, 0.0, 2.0)

    assert quaternion == IDENTITY
    assert quaternion.as_tuple() == (0.0, 0.0, 0.0, 1.0)


def test_quaternion_rejects_zero_norm() -> None:
    with pytest.raises(ValueError, match="norm"):
        Quaternion(0.0, 0.0, 0.0, 0.0)


def test_teleop_input_is_immutable() -> None:
    input_state = TeleopInput(
        sequence=1,
        session_id=2,
        timestamp_ns=3,
        pose=Pose(ORIGIN, IDENTITY),
        linear_velocity=None,
        angular_velocity=None,
        clutch_active=False,
        recenter_requested=False,
        estop_requested=False,
        gripper=None,
        tracking_quality=TrackingQuality.NORMAL,
        source_name="mock",
    )

    with pytest.raises(FrozenInstanceError):
        input_state.sequence = 4  # type: ignore[misc]


def test_robot_state_requires_consistent_joint_lengths() -> None:
    with pytest.raises(ValueError, match="joint_velocity"):
        RobotState(
            timestamp_ns=1,
            joint_position=(0.0, 0.0),
            joint_velocity=(0.0,),
            joint_torque=None,
            tcp_pose=Pose(ORIGIN, IDENTITY),
            tcp_twist=None,
            gripper=None,
            external_wrench=None,
        )


def test_cartesian_position_target_requires_pose() -> None:
    with pytest.raises(ValueError, match="target_pose"):
        RobotTarget(
            mode=ControlMode.CARTESIAN_POSITION,
            target_pose=None,
            target_twist=None,
            target_joint_position=None,
            gripper=None,
            velocity_scale=1.0,
            timestamp_ns=1,
        )
