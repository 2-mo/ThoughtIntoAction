"""Pure relative-pose and clutch rules shared by hardware and simulated RM65."""
import numpy as np
from .geometry import matrix, quaternion, limited_rotation, rotation_vector


def vector(value, count):
    result = np.asarray(value, dtype=float)
    if result.shape != (count,) or not np.isfinite(result).all():
        raise ValueError(f"Expected {count} finite pose values")
    return result


class RelativeTarget:
    def __init__(self, config, *, position_only=False):
        self.config = config
        self.position_only = position_only
        self.basis = np.asarray(config['xr_to_robot_basis'])
        self.reference = None
        self.robot_reference = None
        self.origin = None
        self.limits = []

    def reset(self):
        self.reference = None

    def update(self, controller, robot_pose):
        controller = vector(controller, 7)
        if not np.isclose(np.linalg.norm(controller[3:]), 1, atol=.01):
            raise ValueError('Invalid tracked controller quaternion')
        if self.reference is None:
            self.reference = controller.copy()
            self.robot_reference = robot_pose.copy()
        if self.origin is None:
            self.origin = robot_pose.copy()
        self.limits = []
        target = self.robot_reference.copy()
        delta = self.basis @ (controller[:3] - self.reference[:3]) * self.config['position_scale']
        target[:3] += delta
        if not self.position_only:
            rotation = self.basis @ matrix(controller[3:]) @ matrix(self.reference[3:]).T @ self.basis.T
            target[3:] = quaternion(rotation @ matrix(self.robot_reference[3:]))
        offset = target[:3] - self.origin[:3]
        radius = self.config['translation_radius_m']
        if np.linalg.norm(offset) > radius:
            self.limits.append('translation_boundary')
            target[:3] = self.origin[:3] + offset * radius / np.linalg.norm(offset)
            # Discard only the clipped translation, so reversing the hand moves
            # inward immediately. Rotation keeps its independent reference.
            self.reference[:3] = controller[:3]
            self.robot_reference[:3] = target[:3]
        if np.linalg.norm(rotation_vector(target[3:], self.origin[3:])) > self.config['rotation_radius_rad']:
            self.limits.append('rotation_boundary')
            target[3:] = limited_rotation(target[3:], self.origin[3:], self.config['rotation_radius_rad'])
            self.reference[3:] = controller[3:]
            self.robot_reference[3:] = target[3:]
        return target


class GripGate:
    """Release to arm, squeeze to engage, release below 0.2 to disengage."""
    def __init__(self):
        self.armed = False
        self.following = False

    def update(self, grip, fresh):
        if not fresh:
            self.armed = self.following = False
        elif grip < .2:
            self.armed = True
            self.following = False
        elif grip > .8 and self.armed:
            self.following = True
        return self.following


class XRClock:
    def __init__(self, stale_s):
        self.stale_s = stale_s
        self.previous = 0
        self.updated_at = float('-inf')

    def fresh(self, timestamp, now):
        rollback = timestamp < self.previous
        if timestamp > self.previous:
            self.updated_at = now
        self.previous = timestamp
        return timestamp > 0 and not rollback and now - self.updated_at < self.stale_s
