"""Relative 6DoF pose policy with reference-based clutch continuity."""

from collections.abc import Callable

from embodied.devices.phone.core.frames import compose_pose, quaternion_multiply, relative_pose
from embodied.devices.phone.core.mapping import MappingOutputFrame, PoseMapping
from embodied.devices.phone.core.models import (
    ControlMode,
    Pose,
    RobotState,
    RobotTarget,
    TeleopInput,
    TrackingQuality,
    hold_target,
)


class RelativePosePolicy:
    def __init__(
        self,
        mapping: PoseMapping,
        *,
        mapping_provider: Callable[[], PoseMapping] | None = None,
    ) -> None:
        self._mapping_provider = mapping_provider or (lambda: mapping)
        self._active_mapping = mapping
        self._input_reference: Pose | None = None
        self._robot_reference: Pose | None = None

    def reset(self, input_state: TeleopInput, robot_state: RobotState) -> None:
        self._active_mapping = self._mapping_provider()
        self._input_reference = input_state.pose
        self._robot_reference = robot_state.tcp_pose

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
        return RobotTarget(
            mode=ControlMode.CARTESIAN_POSITION,
            target_pose=target_pose,
            target_twist=None,
            target_joint_position=None,
            gripper=input_state.gripper,
            velocity_scale=1.0,
            timestamp_ns=input_state.timestamp_ns,
        )
