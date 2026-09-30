"""Default policy that leaves position-servo targets unchanged."""

from robot_teleop.core.models import RobotState, RobotTarget


class NoCompliancePolicy:
    def modify(
        self,
        target: RobotTarget,
        state: RobotState,
        dt: float,
    ) -> RobotTarget:
        del state, dt
        return target
