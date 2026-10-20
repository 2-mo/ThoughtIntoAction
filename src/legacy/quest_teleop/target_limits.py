"""Independent rates for Cartesian IK targets; no device access."""
import math
import numpy as np
import meshcat.transformations as tf


def quaternion(rotvec):
    angle = np.linalg.norm(rotvec)
    return tf.quaternion_about_axis(angle, rotvec / angle) if angle > 1e-12 else np.array([1., 0., 0., 0.])


def rotvec(quat):
    q = np.array(quat, dtype=float)
    q /= np.linalg.norm(q)
    if q[0] < 0:
        q = -q
    length = np.linalg.norm(q[1:])
    return q[1:] * (2 * math.atan2(length, q[0]) / length) if length > 1e-12 else np.zeros(3)


class CartesianTargetLimiter:
    def __init__(self, linear_speed, angular_speed):
        for value in (linear_speed, angular_speed):
            if not math.isfinite(value) or value < 0:
                raise ValueError("End-effector target speeds must be finite and nonnegative; 0 disables that limit")
        self.linear_speed, self.angular_speed = linear_speed, angular_speed
        self.position = np.zeros(3)
        self.orientation = np.array([1., 0., 0., 0.])
        self.last_time = None

    def step(self, position, rotation, *, now, period, reset=False):
        if reset:
            self.position = np.zeros(3)
            self.orientation = np.array([1., 0., 0., 0.])
            self.last_time = now
            return self.position.copy(), np.zeros(3)
        dt = min(period, max(0., now - self.last_time)) if self.last_time is not None else 0.
        self.last_time = now
        change = np.asarray(position) - self.position
        distance = np.linalg.norm(change)
        if self.linear_speed > 0 and distance > self.linear_speed * dt:
            change *= self.linear_speed * dt / distance
        self.position = self.position + change
        desired = quaternion(np.asarray(rotation))
        change_rot = rotvec(tf.quaternion_multiply(desired, tf.quaternion_conjugate(self.orientation)))
        angle = np.linalg.norm(change_rot)
        if self.angular_speed > 0 and angle > self.angular_speed * dt:
            change_rot *= self.angular_speed * dt / angle
        self.orientation = tf.quaternion_multiply(quaternion(change_rot), self.orientation)
        self.orientation /= np.linalg.norm(self.orientation)
        return self.position.copy(), rotvec(self.orientation)
