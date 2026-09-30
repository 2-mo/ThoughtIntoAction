import math

import pytest

from robot_teleop.core.frames import (
    quaternion_angular_distance,
    quaternion_from_axis_angle,
    quaternion_multiply,
)
from robot_teleop.core.mapping import AxisMapping, MappingOutputFrame, PoseMapping
from robot_teleop.core.models import (
    Pose,
    Quaternion,
    RobotState,
    TeleopInput,
    TrackingQuality,
    Vector3,
)
from robot_teleop.policies.relative_pose import RelativePosePolicy


def _input(position: Vector3, orientation: Quaternion) -> TeleopInput:
    return TeleopInput(
        sequence=1,
        session_id=1,
        timestamp_ns=1,
        pose=Pose(position, orientation),
        linear_velocity=None,
        angular_velocity=None,
        clutch_active=True,
        recenter_requested=False,
        estop_requested=False,
        gripper=None,
        tracking_quality=TrackingQuality.NORMAL,
        source_name="test",
    )


def _robot(orientation: Quaternion) -> RobotState:
    return RobotState(
        timestamp_ns=1,
        joint_position=(0.0,) * 6,
        joint_velocity=(0.0,) * 6,
        joint_torque=None,
        tcp_pose=Pose(Vector3(0.4, -0.1, 0.3), orientation),
        tcp_twist=None,
        gripper=None,
        external_wrench=None,
    )


def test_base_frame_maps_cardinal_phone_motion_independent_of_tool_attitude() -> None:
    mapping = PoseMapping(
        axes=AxisMapping(robot_from_input=(1, 0, 2), signs=(1, -1, 1)),
        output_frame=MappingOutputFrame.BASE,
    )
    policy = RelativePosePolicy(mapping)
    robot_orientation = quaternion_from_axis_angle(Vector3(0.0, 1.0, 0.0), math.pi / 2)
    robot = _robot(robot_orientation)
    reference = _input(Vector3(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
    policy.reset(reference, robot)

    target = policy.update(
        _input(
            Vector3(0.1, 0.2, 0.3),
            quaternion_from_axis_angle(Vector3(0.0, 0.0, 1.0), 0.2),
        ),
        robot,
        dt=0.01,
    )

    assert target.target_pose is not None
    assert target.target_pose.position.as_tuple() == pytest.approx((0.6, -0.2, 0.6))
    expected_orientation = quaternion_multiply(
        quaternion_from_axis_angle(Vector3(0.0, 0.0, 1.0), 0.2),
        robot_orientation,
    )
    orientation_error = quaternion_angular_distance(
        target.target_pose.orientation,
        expected_orientation,
    )
    assert (
        orientation_error < 1e-9
    )


def test_tool_frame_keeps_the_previous_relative_tool_behavior() -> None:
    mapping = PoseMapping(output_frame=MappingOutputFrame.TOOL)
    policy = RelativePosePolicy(mapping)
    robot_orientation = quaternion_from_axis_angle(Vector3(0.0, 1.0, 0.0), math.pi / 2)
    robot = _robot(robot_orientation)
    reference = _input(Vector3(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
    policy.reset(reference, robot)

    target = policy.update(
        _input(Vector3(0.1, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
        robot,
        dt=0.01,
    )

    assert target.target_pose is not None
    assert target.target_pose.position.as_tuple() == pytest.approx((0.4, -0.1, 0.2))
