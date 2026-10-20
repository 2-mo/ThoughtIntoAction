"""Single target-filter extension point; Phase 0 defaults to disabled."""

from robot_teleop.core.models import RobotState, RobotTarget


class IdentityTargetFilter:
    def reset(self, state: RobotState) -> None:
        del state

    def apply(
        self,
        target: RobotTarget,
        state: RobotState,
        dt: float,
    ) -> RobotTarget:
        del state, dt
        return target
