import math
from dataclasses import replace

import pytest

from embodied.devices.phone.core.frames import (
    quaternion_angular_distance,
    quaternion_from_axis_angle,
    quaternion_multiply,
    rotate_vector,
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
from embodied.devices.inputs.phone import phone_aligned


@pytest.mark.parametrize('phone_axis, expected', [((1, 0, 0), (0, -1, 0)), ((0, 1, 0), (0, 0, 1)), ((0, 0, -1), (1, 0, 0))])
def test_aligned_pregrasp_maps_phone_right_up_forward_to_base(phone_axis, expected):
    policy = RelativePosePolicy(phone_aligned())
    robot = _robot(quaternion_from_axis_angle(Vector3(0, 1, 0), math.pi / 2))
    # ARKit's initial frame need not be upright or have the current phone heading.
    phone_q = quaternion_from_axis_angle(Vector3(1, 1, 1), .8)
    origin = _input(Vector3(0, 0, 0), phone_q)
    policy.reset(origin, robot)
    movement = rotate_vector(phone_q, Vector3(*phone_axis)) * .03
    target = policy.update(_input(movement, phone_q), robot, 0)
    assert (target.target_pose.position - robot.tcp_pose.position).as_tuple() == pytest.approx(tuple(.03*v for v in expected))
    assert quaternion_angular_distance(target.target_pose.orientation, robot.tcp_pose.orientation) < 1e-8


def test_aligned_reference_survives_clutch_release_and_tool_rotation_without_jumps():
    policy = RelativePosePolicy(phone_aligned())
    robot = _robot(quaternion_from_axis_angle(Vector3(0, 1, 0), math.pi / 2))
    origin = _input(Vector3(0, 0, 0), Quaternion(0, 0, 0, 1))
    policy.reset(origin, robot)
    alignment = policy.rotation_alignment
    new_robot = _robot(quaternion_multiply(quaternion_from_axis_angle(Vector3(0, 0, 1), .7), robot.tcp_pose.orientation))
    new_phone = _input(Vector3(.2, .1, .3), quaternion_from_axis_angle(Vector3(0, 1, 0), .7))
    policy.reset(new_phone, new_robot)
    assert policy.rotation_alignment == alignment
    same = policy.update(new_phone, new_robot, 0).target_pose
    assert same.position == new_robot.tcp_pose.position
    assert quaternion_angular_distance(same.orientation, new_robot.tcp_pose.orientation) < 1e-8
    moved = replace(new_phone, pose=Pose(new_phone.pose.position + Vector3(.03, 0, 0), new_phone.pose.orientation))
    assert (policy.update(moved, new_robot, 0).target_pose.position - same.position).as_tuple() == pytest.approx((0, -.03, 0))
    turn = replace(new_phone, pose=Pose(new_phone.pose.position, quaternion_multiply(
        quaternion_from_axis_angle(Vector3(0, 1, 0), .1), new_phone.pose.orientation)))
    expected = quaternion_multiply(quaternion_from_axis_angle(Vector3(0, 0, 1), .1), new_robot.tcp_pose.orientation)
    assert quaternion_angular_distance(policy.update(turn, new_robot, 0).target_pose.orientation, expected) < 1e-8
    policy.clear_alignment()
    policy.reset(new_phone, new_robot)
    assert policy.rotation_alignment is not None


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
