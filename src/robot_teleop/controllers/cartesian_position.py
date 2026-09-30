"""Phase 0 pass-through Cartesian position controller."""

from robot_teleop.core.capabilities import RobotCapabilities
from robot_teleop.core.models import ControlMode, RobotCommand, RobotState, RobotTarget


class UnsupportedControlModeError(RuntimeError):
    pass


class CartesianPositionController:
    def __init__(self, capabilities: RobotCapabilities) -> None:
        if not capabilities.cartesian_position:
            raise ValueError("robot does not support Cartesian position control")
        self._capabilities = capabilities

    def reset(self, state: RobotState) -> None:
        del state

    def update(
        self,
        target: RobotTarget,
        state: RobotState,
        dt: float,
    ) -> RobotCommand:
        del state, dt
        if target.mode not in (
            ControlMode.CARTESIAN_POSITION,
            ControlMode.JOINT_POSITION,
            ControlMode.HOLD,
            ControlMode.STOP,
            ControlMode.HOME,
        ):
            raise UnsupportedControlModeError(
                f"Cartesian position controller cannot execute {target.mode.value}"
            )
        return RobotCommand(
            mode=target.mode,
            target_pose=target.target_pose,
            target_twist=target.target_twist,
            target_joint_position=target.target_joint_position,
            gripper=target.gripper,
            velocity_scale=target.velocity_scale,
            timestamp_ns=target.timestamp_ns,
            reason=(
                "pipeline hold"
                if target.mode is ControlMode.HOLD
                else "pipeline stop"
                if target.mode is ControlMode.STOP
                else "pipeline home"
                if target.mode is ControlMode.HOME
                else None
            ),
        )
