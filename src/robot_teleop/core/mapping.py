"""Configurable signed-axis and scale mapping for relative poses."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

from robot_teleop.core.frames import (
    Matrix3,
    matrix_multiply,
    quaternion_from_matrix,
    quaternion_to_matrix,
    scale_quaternion_rotation,
    transpose_matrix,
)
from robot_teleop.core.models import Pose, Vector3


class MappingOutputFrame(Enum):
    """Frame in which a mapped phone delta is interpreted by the policy."""

    TOOL = "tool"
    BASE = "base"


@dataclass(frozen=True, slots=True)
class AxisMapping:
    """Map output XYZ from input indices with one sign per output axis."""

    robot_from_input: tuple[int, int, int] = (0, 1, 2)
    signs: tuple[int, int, int] = (1, 1, 1)

    def __post_init__(self) -> None:
        if sorted(self.robot_from_input) != [0, 1, 2]:
            raise ValueError("robot_from_input must be a permutation of (0, 1, 2)")
        if any(sign not in (-1, 1) for sign in self.signs):
            raise ValueError("axis signs must be either -1 or 1")

    @property
    def matrix(self) -> Matrix3:
        rows: list[tuple[float, float, float]] = []
        for source_index, sign in zip(self.robot_from_input, self.signs, strict=True):
            row = [0.0, 0.0, 0.0]
            row[source_index] = float(sign)
            rows.append((row[0], row[1], row[2]))
        return (rows[0], rows[1], rows[2])

    def map_vector(self, value: Vector3) -> Vector3:
        source = value.as_tuple()
        return Vector3(
            self.signs[0] * source[self.robot_from_input[0]],
            self.signs[1] * source[self.robot_from_input[1]],
            self.signs[2] * source[self.robot_from_input[2]],
        )


@dataclass(frozen=True, slots=True)
class MappingScales:
    position_scale: float = 1.0
    rotation_scale: float = 1.0

    def __post_init__(self) -> None:
        position_scale = float(self.position_scale)
        rotation_scale = float(self.rotation_scale)
        if not math.isfinite(position_scale) or position_scale < 0.0:
            raise ValueError("position_scale must be non-negative and finite")
        if not math.isfinite(rotation_scale) or not 0.0 <= rotation_scale <= 1.0:
            raise ValueError("rotation_scale must be finite and in [0, 1]")
        object.__setattr__(self, "position_scale", position_scale)
        object.__setattr__(self, "rotation_scale", rotation_scale)


@dataclass(frozen=True, slots=True)
class PoseMapping:
    axes: AxisMapping = AxisMapping()
    position_scale: float = 1.0
    rotation_scale: float = 1.0
    output_frame: MappingOutputFrame = MappingOutputFrame.TOOL

    def __post_init__(self) -> None:
        scales = MappingScales(self.position_scale, self.rotation_scale)
        object.__setattr__(self, "position_scale", scales.position_scale)
        object.__setattr__(self, "rotation_scale", scales.rotation_scale)
        object.__setattr__(self, "output_frame", MappingOutputFrame(self.output_frame))

    @property
    def scales(self) -> MappingScales:
        return MappingScales(self.position_scale, self.rotation_scale)

    def with_scales(self, scales: MappingScales) -> PoseMapping:
        return PoseMapping(
            axes=self.axes,
            position_scale=scales.position_scale,
            rotation_scale=scales.rotation_scale,
            output_frame=self.output_frame,
        )

    def map_delta(self, value: Pose) -> Pose:
        axes = self.axes.matrix
        input_rotation = quaternion_to_matrix(value.orientation)
        mapped_rotation = matrix_multiply(
            matrix_multiply(axes, input_rotation),
            transpose_matrix(axes),
        )
        return Pose(
            self.axes.map_vector(value.position) * self.position_scale,
            scale_quaternion_rotation(
                quaternion_from_matrix(mapped_rotation),
                self.rotation_scale,
            ),
        )
