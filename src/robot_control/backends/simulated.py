"""Bounded virtual X5 feedback using the project URDF; imports no hardware or Quest SDK."""

from types import SimpleNamespace
import threading
import time

import numpy as np

from robot_control.trajectory import bounded_joint_target


class SimulatedX5:
    def __init__(self, robot, *, initial_q=None, gripper_width=.082,
                 control_rate_hz=50, max_joint_speed_rad_s=.2, joint_soft_limit_margin_rad=.05):
        if control_rate_hz <= 0 or max_joint_speed_rad_s <= 0 or gripper_width <= 0:
            raise ValueError("Control rate, speed and gripper width must be positive")
        if not np.isfinite([control_rate_hz, max_joint_speed_rad_s, gripper_width,
                            joint_soft_limit_margin_rad]).all() or joint_soft_limit_margin_rad < 0:
            raise ValueError("Invalid simulation limits")
        self._q = robot.joints(np.zeros(6) if initial_q is None else initial_q)
        self._dq = np.zeros(6)
        self._last_move = 0.
        self._gripper = gripper_width
        self._gripper_width = gripper_width
        self._dt = 1. / control_rate_hz
        self._max_step = max_joint_speed_rad_s / control_rate_hz
        self._margin = joint_soft_limit_margin_rad
        self.robot_config = SimpleNamespace(joint_pos_min=robot.lower.copy(),
                                             joint_pos_max=robot.upper.copy(), joint_dof=6)
        self._lock = threading.Lock()
        self.closed = False

    def get_joint_positions(self):
        with self._lock:
            return np.r_[self._q, self._gripper]

    def get_joint_velocities(self):
        with self._lock:
            return np.r_[self._dq if time.monotonic() - self._last_move < self._dt else np.zeros(6), 0.]

    def set_joint_positions(self, positions):
        positions = np.asarray(positions, dtype=float)
        if positions.shape != (6,):
            raise ValueError("Expected six simulated joint targets")
        with self._lock:
            if self.closed:
                raise RuntimeError("Simulated arm is closed")
            next_q, _ = bounded_joint_target(
                q_actual=self._q, q_desired=positions,
                q_min=self.robot_config.joint_pos_min, q_max=self.robot_config.joint_pos_max,
                max_step_rad=self._max_step, soft_limit_margin_rad=self._margin,
            )
            self._dq = (next_q - self._q) / self._dt
            self._q = next_q
            self._last_move = time.monotonic()
        return True

    def set_catch_pos(self, position):
        if not np.isfinite(position):
            raise ValueError("Gripper target must be finite")
        with self._lock:
            if self.closed:
                raise RuntimeError("Simulated arm is closed")
            self._gripper = float(np.clip(position, 0., self._gripper_width))
        return True

    def set_to_damping(self):
        with self._lock:
            self._dq.fill(0.)

    def set_control_enabled(self, enabled):
        self.set_to_damping()

    def set_motion_limit(self, speed):
        self._max_step = speed * self._dt

    def close(self):
        with self._lock:
            self._dq.fill(0.)
            self.closed = True
