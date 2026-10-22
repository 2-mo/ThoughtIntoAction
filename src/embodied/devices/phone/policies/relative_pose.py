"""Relative 6DoF pose policy with reference-based clutch continuity."""

from collections.abc import Callable
import math

from embodied.devices.phone.core.frames import (compose_pose, quaternion_multiply, relative_pose,
    quaternion_conjugate, quaternion_angular_distance, quaternion_slerp, IDENTITY_QUATERNION)
from embodied.devices.phone.core.mapping import MappingOutputFrame, PoseMapping
from embodied.devices.phone.core.models import (
    ControlMode,
    Pose,
    RobotState,
    RobotTarget,
    TeleopInput,
    TrackingQuality,
    Vector3,
    hold_target,
)


class RelativePosePolicy:
    def __init__(
        self,
        mapping: PoseMapping,
        *,
        mapping_provider: Callable[[], PoseMapping] | None = None,
        max_position_lead_m: float | None = None,
        max_rotation_lead_rad: float | None = None,
    ) -> None:
        self._mapping_provider = mapping_provider or (lambda: mapping)
        self._active_mapping = mapping
        self._input_reference: Pose | None = None
        self._robot_reference: Pose | None = None
        for bound in (max_position_lead_m, max_rotation_lead_rad):
            if bound is not None and (not math.isfinite(bound) or bound <= 0):
                raise ValueError('Pose lead bounds must be positive and finite')
        self.max_position_lead_m = max_position_lead_m
        self.max_rotation_lead_rad = max_rotation_lead_rad
        self._position_correction = Vector3(0, 0, 0)
        self._rotation_correction = IDENTITY_QUATERNION
        self.limited = False
        self.position_lead_m = self.rotation_lead_rad = 0.

    def reset(self, input_state: TeleopInput, robot_state: RobotState) -> None:
        self._active_mapping = self._mapping_provider()
        self._input_reference = input_state.pose
        self._robot_reference = robot_state.tcp_pose
        self._position_correction = Vector3(0, 0, 0)
        self._rotation_correction = IDENTITY_QUATERNION
        self.limited = False
        self.position_lead_m = self.rotation_lead_rad = 0.

    def update(
        self,
        input_state: TeleopInput,
        robot_state: RobotState,
        dt: float,
    ) -> RobotTarget:
        del dt
        if (
            not input_state.clutch_active
            or input_state.tracking_quality is not TrackingQuality.NORMAL
        ):
            return hold_target(input_state.timestamp_ns, gripper=input_state.gripper)
        if self._input_reference is None or self._robot_reference is None:
            raise RuntimeError("RelativePosePolicy must be reset before update")
        input_delta = relative_pose(self._input_reference, input_state.pose)
        mapped_delta = self._active_mapping.map_delta(input_delta)
        if self._active_mapping.output_frame is MappingOutputFrame.BASE:
            # Keep cardinal hand motions aligned with the table/base frame even
            # when the initial tool attitude is pitched or rolled.
            target_pose = Pose(
                self._robot_reference.position + mapped_delta.position,
                quaternion_multiply(
                    mapped_delta.orientation,
                    self._robot_reference.orientation,
                ),
            )
        else:
            target_pose = compose_pose(self._robot_reference, mapped_delta)
        # Bound tracking error, not speed. Discard unreachable displacement from
        # the reference so reversing the hand responds immediately after saturation.
        raw = target_pose
        position = raw.position + self._position_correction
        orientation = quaternion_multiply(self._rotation_correction, raw.orientation)
        delta = position - robot_state.tcp_pose.position
        distance = delta.norm()
        angle = quaternion_angular_distance(robot_state.tcp_pose.orientation, orientation)
        self.limited = False
        if self.max_position_lead_m is not None and distance > self.max_position_lead_m:
            position = robot_state.tcp_pose.position + delta * (self.max_position_lead_m / distance)
            self._position_correction = position - raw.position
            self.limited = True
        if self.max_rotation_lead_rad is not None and angle > self.max_rotation_lead_rad:
            orientation = quaternion_slerp(robot_state.tcp_pose.orientation, orientation,
                                          self.max_rotation_lead_rad / angle)
            self._rotation_correction = quaternion_multiply(orientation, quaternion_conjugate(raw.orientation))
            self.limited = True
        self.position_lead_m = (position - robot_state.tcp_pose.position).norm()
        self.rotation_lead_rad = quaternion_angular_distance(robot_state.tcp_pose.orientation, orientation)
        target_pose = Pose(position, orientation)
        return RobotTarget(
            mode=ControlMode.CARTESIAN_POSITION,
            target_pose=target_pose,
            target_twist=None,
            target_joint_position=None,
            gripper=input_state.gripper,
            velocity_scale=1.0,
            timestamp_ns=input_state.timestamp_ns,
        )
