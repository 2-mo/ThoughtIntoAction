import math
from dataclasses import replace

import pytest

from embodied.devices.phone.core.frames import (
    quaternion_angular_distance,
    quaternion_from_axis_angle,
    quaternion_multiply,
)
from embodied.devices.phone.core.mapping import AxisMapping, MappingOutputFrame, PoseMapping
from embodied.devices.phone.core.models import (
    Pose,
    Quaternion,
    RobotState,
    TeleopInput,
    TrackingQuality,
    Vector3,
)
from embodied.devices.phone.policies.relative_pose import RelativePosePolicy


def test_bounded_rotation_reverses_without_saved_excess_and_resets_on_clutch():
    policy = RelativePosePolicy(PoseMapping(), max_position_lead_m=.05, max_rotation_lead_rad=.35)
    robot = _robot(Quaternion(0, 0, 0, 1))
    reference = _input(Vector3(0, 0, 0), Quaternion(0, 0, 0, 1))
    policy.reset(reference, robot)
    first = policy.update(_input(Vector3(0, 0, 0), quaternion_from_axis_angle(Vector3(0, 0, 1), 1.)), robot, 0)
    assert quaternion_angular_distance(robot.tcp_pose.orientation, first.target_pose.orientation) == pytest.approx(.35)
    reverse = policy.update(_input(Vector3(0, 0, 0), quaternion_from_axis_angle(Vector3(0, 0, 1), .9)), robot, 0)
    assert quaternion_angular_distance(robot.tcp_pose.orientation, reverse.target_pose.orientation) == pytest.approx(.25)
    policy.reset(reference, robot)
    target = policy.update(reference, robot, 0)
    assert quaternion_angular_distance(target.target_pose.orientation, robot.tcp_pose.orientation) < 1e-8


def test_lead_bound_does_not_limit_total_travel_when_robot_follows():
    policy = RelativePosePolicy(PoseMapping(), max_position_lead_m=.05)
    robot = _robot(Quaternion(0, 0, 0, 1))
    policy.reset(_input(Vector3(0, 0, 0), Quaternion(0, 0, 0, 1)), robot)
    for step in range(1, 41):
        target = policy.update(_input(Vector3(step * .01, 0, 0), Quaternion(0, 0, 0, 1)), robot, 0)
        assert not policy.limited
        robot = replace(robot, tcp_pose=target.target_pose)
    assert robot.tcp_pose.position.x == pytest.approx(.8)


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
