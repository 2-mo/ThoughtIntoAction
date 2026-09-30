import math

import pytest

from robot_teleop.core.frames import (
    compose_pose,
    inverse_pose,
    quaternion_angular_distance,
    quaternion_from_axis_angle,
    rotate_vector,
    scale_quaternion_rotation,
)
from robot_teleop.core.mapping import AxisMapping, PoseMapping
from robot_teleop.core.models import Pose, Quaternion, Vector3

IDENTITY = Quaternion(0.0, 0.0, 0.0, 1.0)


def assert_vector(actual: Vector3, expected: tuple[float, float, float]) -> None:
    assert actual.as_tuple() == pytest.approx(expected, abs=1e-9)


def test_pose_inverse_round_trip() -> None:
    pose = Pose(
        Vector3(1.0, -2.0, 0.5),
        quaternion_from_axis_angle(Vector3(0.0, 0.0, 1.0), math.pi / 2.0),
    )

    identity = compose_pose(inverse_pose(pose), pose)

    assert_vector(identity.position, (0.0, 0.0, 0.0))
    assert quaternion_angular_distance(identity.orientation, IDENTITY) < 1e-9


def test_relative_translation_is_expressed_in_reference_frame() -> None:
    reference = Pose(
        Vector3(1.0, 0.0, 0.0),
        quaternion_from_axis_angle(Vector3(0.0, 0.0, 1.0), math.pi / 2.0),
    )
    current = compose_pose(reference, Pose(Vector3(1.0, 0.0, 0.0), IDENTITY))

    delta = compose_pose(inverse_pose(reference), current)

    assert_vector(delta.position, (1.0, 0.0, 0.0))


def test_rotation_scaling_uses_shortest_path() -> None:
    half_turn = quaternion_from_axis_angle(Vector3(0.0, 0.0, 1.0), math.pi)

    halfway = scale_quaternion_rotation(half_turn, 0.5)
    rotated = rotate_vector(halfway, Vector3(1.0, 0.0, 0.0))

    assert_vector(rotated, (0.0, 1.0, 0.0))


def test_axis_mapping_matches_profile_example() -> None:
    axes = AxisMapping(
        robot_from_input=(2, 0, 1),
        signs=(-1, -1, 1),
    )

    mapped = axes.map_vector(Vector3(1.0, 2.0, 3.0))

    assert_vector(mapped, (-3.0, -1.0, 2.0))


def test_orientation_mapping_conjugates_the_axis_frame() -> None:
    axes = AxisMapping(
        robot_from_input=(2, 0, 1),
        signs=(-1, -1, 1),
    )
    mapping = PoseMapping(axes=axes, position_scale=0.5, rotation_scale=1.0)
    input_rotation = quaternion_from_axis_angle(Vector3(1.0, 0.0, 0.0), math.pi / 2.0)

    mapped = mapping.map_delta(Pose(Vector3(2.0, 0.0, 0.0), input_rotation))
    rotated_z = rotate_vector(mapped.orientation, Vector3(0.0, 0.0, 1.0))

    assert_vector(mapped.position, (0.0, -1.0, 0.0))
    assert_vector(rotated_z, (-1.0, 0.0, 0.0))
