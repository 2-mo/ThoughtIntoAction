"""Calibration leases control through the independent arm service."""

import time

import numpy as np


class CalibrationArm:
    def __init__(self, robot, service_socket=None):
        self.service_socket, self.robot = service_socket, robot
        self.backend = None
        self._last_timestamp = None
        self._last_progress = time.monotonic()

    def __enter__(self):
        from legacy.robot_control.client import ArmClient
        self.backend = ArmClient(self.robot, path=self.service_socket, hardware=True,
                                 actuate=False, input_name="handeye", speed=.15)
        try:
            self.backend.start_control()
            self.gripper = float(self.backend.get_joint_positions()[6])
            self.snapshot()
            return self
        except BaseException:
            self.backend.close()
            self.backend = None
            raise

    def snapshot(self):
        diagnostics = self.backend.get_runtime_diagnostics()
        q = self.robot.joints(diagnostics["actual_pos"])
        dq = np.asarray(diagnostics["actual_vel"])
        stamp = diagnostics["feedback_timestamp"]
        if (not np.isfinite(dq).all() or not np.isfinite(stamp)
                or not np.isfinite(diagnostics["gripper_actual_pos"])):
            raise RuntimeError("Non-finite arm feedback")
        now = time.monotonic()
        if self._last_timestamp is None or stamp > self._last_timestamp:
            self._last_timestamp, self._last_progress = stamp, now
        elif stamp < self._last_timestamp or now - self._last_progress > 0.25:
            raise RuntimeError("Stale or regressed arm feedback timestamp")
        # SDK timestamps prove the controller loop is progressing; they are
        # not per-motor CAN receive timestamps and cannot prove motor freshness.
        return {"q": q.tolist(), "dq": dq.tolist(), "gripper_m": float(diagnostics["gripper_actual_pos"]),
                "sdk_timestamp": stamp, "host_monotonic_ns": diagnostics["host_monotonic_ns"]}

    def enable(self):
        state = self.snapshot()
        if np.max(np.abs(state["dq"])) > 0.02:
            raise ValueError("Reference pose is moving; cannot enable position hold")
        # The service seeds gains/command from its own fresh feedback.
        backend = self.backend
        backend.set_to_damping()
        self.gripper = state["gripper_m"]
        backend.actuate = True
        backend.start_control()

    def command(self, q):
        self.backend.set_joint_positions(self.robot.joints(q, margin=0.05))
        # set_joint_positions alone deliberately does NOT submit a command.
        self.backend.set_catch_pos(self.gripper)

    def damp(self):
        if self.backend is not None:
            self.backend.set_to_damping()
            self.backend.actuate = False

    def __exit__(self, *exc):
        if self.backend is not None:
            self.backend.close()
            self.backend = None
