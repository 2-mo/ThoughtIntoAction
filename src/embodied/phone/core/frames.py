"""Small canonical rigid-transform and quaternion math surface."""

from __future__ import annotations

import math

from embodied.phone.core.models import Pose, Quaternion, Vector3

Matrix3 = tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]

IDENTITY_QUATERNION = Quaternion(0.0, 0.0, 0.0, 1.0)


def dot(first: Vector3, second: Vector3) -> float:
    return first.x * second.x + first.y * second.y + first.z * second.z


def normalized_vector(value: Vector3) -> Vector3:
    norm = value.norm()
    if norm <= 1e-12:
        raise ValueError("Vector norm must be positive")
    return value * (1.0 / norm)


def quaternion_conjugate(value: Quaternion) -> Quaternion:
    return Quaternion(-value.x, -value.y, -value.z, value.w)


def quaternion_multiply(first: Quaternion, second: Quaternion) -> Quaternion:
    ax, ay, az, aw = first.as_tuple()
    bx, by, bz, bw = second.as_tuple()
    return Quaternion(
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def rotate_vector(rotation: Quaternion, value: Vector3) -> Vector3:
    matrix = quaternion_to_matrix(rotation)
    x, y, z = value.as_tuple()
    return Vector3(
        matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z,
        matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z,
        matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z,
    )


def quaternion_from_axis_angle(axis: Vector3, angle_rad: float) -> Quaternion:
    if not math.isfinite(angle_rad):
        raise ValueError("angle_rad must be finite")
    unit = normalized_vector(axis)
    half = angle_rad / 2.0
    scale = math.sin(half)
    return Quaternion(unit.x * scale, unit.y * scale, unit.z * scale, math.cos(half))


def quaternion_angular_distance(first: Quaternion, second: Quaternion) -> float:
    product = sum(
        a * b for a, b in zip(first.as_tuple(), second.as_tuple(), strict=True)
    )
    return 2.0 * math.acos(min(1.0, max(-1.0, abs(product))))


def quaternion_slerp(
    first: Quaternion,
    second: Quaternion,
    fraction: float,
) -> Quaternion:
    if not math.isfinite(fraction) or not 0.0 <= fraction <= 1.0:
        raise ValueError("fraction must be finite and in [0, 1]")
    first_values = first.as_tuple()
    second_values = second.as_tuple()
    cosine = sum(a * b for a, b in zip(first_values, second_values, strict=True))
    if cosine < 0.0:
        second_values = (
            -second_values[0],
            -second_values[1],
            -second_values[2],
            -second_values[3],
        )
        cosine = -cosine
    cosine = min(1.0, max(-1.0, cosine))
    if cosine > 0.9995:
        values = tuple(
            first_value + fraction * (second_value - first_value)
            for first_value, second_value in zip(
                first_values, second_values, strict=True
            )
        )
        return Quaternion(*values)
    angle = math.acos(cosine)
    sine = math.sin(angle)
    first_scale = math.sin((1.0 - fraction) * angle) / sine
    second_scale = math.sin(fraction * angle) / sine
    values = tuple(
        first_scale * first_value + second_scale * second_value
        for first_value, second_value in zip(first_values, second_values, strict=True)
    )
    return Quaternion(*values)


def scale_quaternion_rotation(value: Quaternion, scale: float) -> Quaternion:
    return quaternion_slerp(IDENTITY_QUATERNION, value, scale)


def compose_pose(first: Pose, second: Pose) -> Pose:
    return Pose(
        first.position + rotate_vector(first.orientation, second.position),
        quaternion_multiply(first.orientation, second.orientation),
    )


def inverse_pose(value: Pose) -> Pose:
    inverse_orientation = quaternion_conjugate(value.orientation)
    return Pose(
        rotate_vector(inverse_orientation, -value.position),
        inverse_orientation,
    )


def relative_pose(reference: Pose, current: Pose) -> Pose:
    return compose_pose(inverse_pose(reference), current)


def quaternion_to_matrix(value: Quaternion) -> Matrix3:
    x, y, z, w = value.as_tuple()
    return (
        (
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ),
        (
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ),
        (
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ),
    )


def quaternion_from_matrix(matrix: Matrix3) -> Quaternion:
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        return Quaternion(
            (matrix[2][1] - matrix[1][2]) / scale,
            (matrix[0][2] - matrix[2][0]) / scale,
            (matrix[1][0] - matrix[0][1]) / scale,
            0.25 * scale,
        )
    diagonal = (matrix[0][0], matrix[1][1], matrix[2][2])
    index = max(range(3), key=diagonal.__getitem__)
    if index == 0:
        scale = math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0
        return Quaternion(
            0.25 * scale,
            (matrix[0][1] + matrix[1][0]) / scale,
            (matrix[0][2] + matrix[2][0]) / scale,
            (matrix[2][1] - matrix[1][2]) / scale,
        )
    if index == 1:
        scale = math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0
        return Quaternion(
            (matrix[0][1] + matrix[1][0]) / scale,
            0.25 * scale,
            (matrix[1][2] + matrix[2][1]) / scale,
            (matrix[0][2] - matrix[2][0]) / scale,
        )
    scale = math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0
    return Quaternion(
        (matrix[0][2] + matrix[2][0]) / scale,
        (matrix[1][2] + matrix[2][1]) / scale,
        0.25 * scale,
        (matrix[1][0] - matrix[0][1]) / scale,
    )


def matrix_multiply(first: Matrix3, second: Matrix3) -> Matrix3:
    return tuple(
        tuple(
            sum(first[row][inner] * second[inner][column] for inner in range(3))
            for column in range(3)
        )
        for row in range(3)
    )  # type: ignore[return-value]


def transpose_matrix(value: Matrix3) -> Matrix3:
    return tuple(tuple(value[column][row] for column in range(3)) for row in range(3))  # type: ignore[return-value]
