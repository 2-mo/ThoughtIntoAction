"""Capabilities used for composition without concrete-type branching."""

import math
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class InputCapabilities:
    pose_6dof: bool
    linear_velocity: bool
    angular_velocity: bool
    analog_gripper: bool
    haptics: bool
    dual_hand: bool


@dataclass(frozen=True, slots=True)
class RobotCapabilities:
    cartesian_position: bool
    cartesian_velocity: bool
    joint_position: bool
    torque_control: bool
    gripper: bool
    external_wrench: bool
    control_rate_hz: float
    joint_count: int

    def __post_init__(self) -> None:
        if not math.isfinite(self.control_rate_hz) or self.control_rate_hz <= 0.0:
            raise ValueError("control_rate_hz must be positive and finite")
        if isinstance(self.joint_count, bool) or self.joint_count <= 0:
            raise ValueError("joint_count must be positive")
