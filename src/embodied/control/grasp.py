"""Bounded grasp progression on existing feedback; no sleeps, SDK calls or I/O."""
import math

from embodied.control.motion_limits import positive_number


class Grasp:
    def __init__(self, request, state, capabilities, now):
        if not capabilities.get('present') or not capabilities.get('grasp'):
            raise ValueError('The connected device does not support grasp')
        self.capabilities = capabilities
        self.effort = positive_number(request.get('effort'), 'effort')
        if self.effort > capabilities['effort_max']:
            raise ValueError('Requested effort exceeds the device capability')
        self.minimum_width = positive_number(request.get('minimum_width', 0.), 'minimum_width', zero_allowed=True)
        if self.minimum_width > state.gripper_width:
            raise ValueError('minimum_width exceeds current gripper width')
        self.timeout = positive_number(request.get('timeout', 5.), 'timeout')
        self.contact_time = positive_number(request.get('contact_time', .2), 'contact_time')
        if self.timeout > 60 or self.contact_time >= self.timeout:
            raise ValueError('Require contact_time < timeout <= 60 seconds')
        self.started, self.stall_since, self.last_feedback = now, None, None
        self.start_width = state.gripper_width
        self.arm_q = state.q.copy()
        self.ramp_s = .3
        self.result = None
        self._feedback(state, now)

    def _feedback(self, state, now):
        values = (state.gripper_width, state.gripper_velocity, state.gripper_effort, state.received_at)
        if any(value is None or not math.isfinite(value) for value in values):
            raise ValueError('Grasp requires finite width, velocity and effort feedback')
        if not 0 <= now - state.received_at < .1:
            raise ValueError('Grasp feedback is stale')

    def update(self, state, now):
        self._feedback(state, now)
        elapsed = now - self.started
        ramp = .5 - .5 * math.cos(math.pi * min(1., elapsed / self.ramp_s))
        reason = None
        if state.gripper_width <= self.minimum_width + .001:
            reason = 'minimum_width'
        elif elapsed >= self.timeout:
            reason = 'timeout'
        else:
            # SDK hardware uses effort + sustained stall above the travel floor.
            # A simulator additionally supplies actual collision evidence.
            physical_contact = (state.gripper_contact is True if self.capabilities.get('contact_detection') == 'physics_contact'
                                else state.gripper_contact is not False)
            contact = (elapsed >= self.ramp_s and abs(state.gripper_velocity) <= .002
                       and abs(state.gripper_effort) >= self.effort * .2 and physical_contact
                       and self.start_width - state.gripper_width >= .0005)
            if self.last_feedback is not None and not 0 < state.received_at - self.last_feedback <= .1:
                contact = False
            self.stall_since = (now if self.stall_since is None else self.stall_since) if contact else None
            if self.stall_since is not None and now - self.stall_since >= self.contact_time:
                reason = 'contact'
        self.last_feedback = state.received_at
        if reason:
            self.result = {'stop_reason': reason, 'confirmed': reason == 'contact',
                           'actual_width_m': float(state.gripper_width),
                           'actual_effort': float(state.gripper_effort),
                           'effort_unit': self.capabilities['effort_unit'],
                           'contact_detection': self.capabilities['contact_detection'],
                           'elapsed_s': elapsed}
        return self.effort * ramp
