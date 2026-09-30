"""Standalone robot bridge and synthetic XR publisher for native control."""
from __future__ import annotations

import threading
import time

import numpy as np

from arx5_vr.types import ControlCommand, Mode, RobotState, XRFrame
from .services import create_node, drain_latest, make_publisher, make_subscriber, open_service, send, validate_prefix
from .types import DeviceChannelMode, JointVector15, ParallelMitCommand2, ParallelVector2, Pose7, XrControllerInput

_FROM_WIRE = {0: Mode.DISABLED, 2: Mode.FREE_DRIVE, 3: Mode.COMMAND_FOLLOWING}
_TO_WIRE = {mode: wire for wire, mode in _FROM_WIRE.items()}


class RobotBridge:
    """Consume native commands and publish backend feedback at the caller's rate."""

    def __init__(self, prefix: str, *, commands=True, feedback=True):
        self.commands_enabled, self.feedback_enabled = commands, feedback
        self.prefix = validate_prefix(prefix)
        self._node = create_node()
        self._publishers, self._subscribers = {}, {}
        self._last_feedback_at = float("-inf")
        self._modes = {"arm": Mode.DISABLED, "gripper": Mode.DISABLED}
        for channel in self._modes:
            if commands:
                self._subscribers[channel] = make_subscriber(open_service(
                    self._node, f"{prefix}/robot/{channel}/control/mode", DeviceChannelMode, mode=True))
            if feedback:
                self._publishers[channel] = make_publisher(open_service(
                    self._node, f"{prefix}/robot/{channel}/info/mode", DeviceChannelMode, mode=True))
        for key, suffix, kind in [
            ("pose", "arm/commands/end_pose", Pose7),
            ("grip", "gripper/commands/parallel_mit", ParallelMitCommand2),
        ]:
            if commands:
                self._subscribers[key] = make_subscriber(open_service(self._node, f"{prefix}/robot/{suffix}", kind))
        for key, suffix, kind in [
            ("pose", "arm/states/end_effector_pose", Pose7),
            ("q", "arm/states/joint_position", JointVector15),
            ("dq", "arm/states/joint_velocity", JointVector15),
            ("width", "gripper/states/parallel_position", ParallelVector2),
        ]:
            if feedback:
                self._publishers[key] = make_publisher(open_service(self._node, f"{prefix}/robot/{suffix}", kind))

    def poll_commands(self, now: float) -> ControlCommand:
        if not self.commands_enabled:
            raise RuntimeError("This bridge only publishes feedback")
        previous_arm_mode = self._modes["arm"]
        for channel in self._modes:
            value = drain_latest(self._subscribers[channel])
            if value is not None:
                self._modes[channel] = _FROM_WIRE.get(value.value, Mode.DISABLED)
        pose = drain_latest(self._subscribers["pose"])
        grip = drain_latest(self._subscribers["grip"])
        if previous_arm_mode is not Mode.COMMAND_FOLLOWING and self._modes["arm"] is Mode.COMMAND_FOLLOWING:
            # NERO clears pending arm commands on CF entry before its first
            # feedback-anchored tick. The gripper has no corresponding entry
            # flush, so its independent command must remain available.
            pose = None
        target = np.array(pose.values, dtype=float) if pose is not None else None
        width = float(grip.position[0]) if grip is not None and grip.len == 1 else None
        if target is not None and (not np.isfinite(target).all() or np.linalg.norm(target[3:]) < 1e-8):
            target = None
        if width is not None and not np.isfinite(width):
            width = None
        return ControlCommand(mode=self._modes["arm"], pose_target=target,
                              gripper_target=width, gripper_mode=self._modes["gripper"],
                              engaged=target is not None)

    def publish_state(self, state: RobotState, now: float) -> None:
        if not self.feedback_enabled:
            raise RuntimeError("This bridge only consumes commands")
        for channel, mode in (("arm", state.mode), ("gripper", state.gripper_mode or state.mode)):
            send(self._publishers[channel], DeviceChannelMode.of(_TO_WIRE[mode]))
        age = now - state.received_at
        if (state.fault or not np.isfinite(age) or age < 0 or age >= .100
                or state.received_at <= self._last_feedback_at):
            return
        self._last_feedback_at = state.received_at
        timestamp = max(0, time.time_ns() // 1000 - int(age * 1_000_000))
        send(self._publishers["pose"], Pose7.from_values(timestamp, list(state.pose)))
        send(self._publishers["q"], JointVector15.from_values(timestamp, list(state.q)))
        send(self._publishers["dq"], JointVector15.from_values(timestamp, list(state.dq)))
        send(self._publishers["width"], ParallelVector2.from_values(timestamp, [state.gripper_width]))

    def close(self) -> None:
        self._subscribers.clear()
        self._publishers.clear()
        self._node = None


class XRPublisher:
    """Publish already-transformed synthetic XR frames for the real native router.

    A lock prevents interleaved callers. The two IPC samples share a timestamp;
    iceoryx2 does not offer a cross-topic atomic transaction. Native routing
    validates the matching pose/input pair before enabling motion.
    """

    def __init__(self, prefix: str):
        validate_prefix(prefix)
        self._node = create_node()
        self._lock = threading.Lock()
        self._input = make_publisher(open_service(self._node, f"{prefix}/xr/right_controller/states/input", XrControllerInput))
        self._pose = make_publisher(open_service(self._node, f"{prefix}/xr/right_controller/states/pose", Pose7))

    def publish(self, frame: XRFrame) -> None:
        with self._lock:
            timestamp = time.time_ns() // 1000
            pose = np.asarray(frame.pose, dtype=float) if frame.pose is not None else None
            valid = (frame.valid and frame.connected and pose is not None and pose.shape == (7,)
                     and np.isfinite(pose).all() and np.linalg.norm(pose[3:]) > 1e-8)
            inp = XrControllerInput()
            inp.timestamp_us, inp.sequence = timestamp, frame.sequence
            inp.connection_state, inp.tracking_valid = (1 if frame.connected else 2), int(valid)
            inp.buttons_bitset = (4 if frame.primary else 0) | (8 if frame.secondary else 0)
            inp.trigger = float(np.clip(frame.trigger, 0, 1))
            inp.grip = float(np.clip(frame.grip, 0, 1))
            send(self._input, inp)
            if valid:
                send(self._pose, Pose7.from_values(timestamp, list(pose)))

    def close(self) -> None:
        self._input = self._pose = self._node = None
