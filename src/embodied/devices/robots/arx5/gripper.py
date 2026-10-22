"""Position goals with bounded total motor effort, including the PD contribution.

The Stanford SDK expresses DM J4310 feedback/feedforward in estimated motor Nm
and converts to/from motor current with 0.424. Native MIT gains act before that
conversion. This is not a calibrated fingertip force in newtons.
"""
import math


class GripperPositionServo:
    MOTOR_TORQUE_CONSTANT = .424

    def __init__(self, *, open_readout, width, kp, kd, effort_limit, speed, dt):
        values = (open_readout, width, kp, kd, effort_limit, speed, dt)
        if (not all(math.isfinite(v) for v in values) or open_readout == 0
                or min(width, effort_limit, speed, dt) <= 0 or min(kp, kd) < 0):
            raise ValueError('Invalid ARX gripper position servo configuration')
        self.radians_per_metre = open_readout / width
        self.kp, self.kd = kp * self.MOTOR_TORQUE_CONSTANT, kd * self.MOTOR_TORQUE_CONSTANT
        self.limit, self.step = effort_limit, speed * dt
        self.reference = None
        self.limited = False

    def reset(self):
        self.reference = None
        self.limited = False

    def update(self, target, actual, velocity):
        if not all(math.isfinite(v) for v in (target, actual, velocity)):
            raise ValueError('Gripper position servo requires finite feedback and target')
        if self.reference is None:
            self.reference = actual
        # Keep the SDK's configured width rate. Do not accumulate a position goal
        # through an object: the reference stays within the effort envelope.
        self.reference += max(-self.step, min(self.step, target - self.reference))
        stiffness = self.kp * abs(self.radians_per_metre)
        if stiffness > 0:
            lead = self.limit / stiffness
            self.reference = max(actual - lead, min(actual + lead, self.reference))
        requested = (self.kp * (self.reference - actual) - self.kd * velocity) * self.radians_per_metre
        effort = max(-self.limit, min(self.limit, requested))
        self.limited = abs(requested) >= self.limit - 1e-9
        return effort
