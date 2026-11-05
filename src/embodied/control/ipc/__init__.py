"""Standalone robot bridge and synthetic XR publisher for native control."""
from __future__ import annotations

import threading
import time

import numpy as np

from embodied.contracts.control import ControlCommand, Mode, RobotState, XRFrame
from embodied.control.ipc.services import create_node, drain_latest, make_publisher, make_subscriber, open_service, send, validate_prefix
from embodied.control.ipc.types import DeviceChannelMode, JointVector15, ParallelMitCommand2, ParallelVector2, Pose7, XrControllerInput

_FROM_WIRE = {0: Mode.DISABLED, 2: Mode.FREE_DRIVE, 3: Mode.COMMAND_FOLLOWING}
_TO_WIRE = {mode: wire for wire, mode in _FROM_WIRE.items()}


class RobotBridge:
    """Consume native commands and publish backend feedback at the caller's rate."""

    def __init__(self, prefix: str, *, commands=True, feedback=True, arms: int = 1):
        self.commands_enabled, self.feedback_enabled = commands, feedback
        self.prefix = validate_prefix(prefix)
        self.arms = max(1, int(arms))
        # Channel names stay "arm"/"gripper" for the first arm so single-arm
        # topics are unchanged; further arms use arm2/gripper2.
        self._names = [("arm" if index == 0 else f"arm{index + 1}",
                        "gripper" if index == 0 else f"gripper{index + 1}")
                       for index in range(self.arms)]
        self._node = create_node()
        self._publishers, self._subscribers = {}, {}
        self._last_feedback_at = float("-inf")
        self._modes = {name: Mode.DISABLED for pair in self._names for name in pair}
        self._mode_seen = {name: False for pair in self._names for name in pair}
        for arm_name, grip_name in self._names:
            for channel in (arm_name, grip_name):
                if commands:
                    self._subscribers[channel] = make_subscriber(open_service(
                        self._node, f"{prefix}/robot/{channel}/control/mode", DeviceChannelMode, mode=True))
                if feedback:
                    self._publishers[channel] = make_publisher(open_service(
                        self._node, f"{prefix}/robot/{channel}/info/mode", DeviceChannelMode, mode=True))
            if commands:
                self._subscribers[f"pose:{arm_name}"] = make_subscriber(open_service(
                    self._node, f"{prefix}/robot/{arm_name}/commands/end_pose", Pose7))
                self._subscribers[f"grip:{grip_name}"] = make_subscriber(open_service(
                    self._node, f"{prefix}/robot/{grip_name}/commands/parallel_mit", ParallelMitCommand2))
            if feedback:
                for key, suffix, kind, channel in [
                    ("pose", "states/end_effector_pose", Pose7, arm_name),
                    ("q", "states/joint_position", JointVector15, arm_name),
                    ("dq", "states/joint_velocity", JointVector15, arm_name),
                    ("width", "states/parallel_position", ParallelVector2, grip_name),
                ]:
                    self._publishers[f"{key}:{channel}"] = make_publisher(open_service(
                        self._node, f"{prefix}/robot/{channel}/{suffix}", kind))

    def poll_commands(self, now: float, arm: int = 0) -> ControlCommand:
        if not self.commands_enabled:
            raise RuntimeError("This bridge only publishes feedback")
        arm_name, grip_name = self._names[arm]
        previous_arm_mode = self._modes[arm_name]
        for channel in (arm_name, grip_name):
            value = drain_latest(self._subscribers[channel])
            if value is not None:
                self._modes[channel] = _FROM_WIRE.get(value.value, Mode.DISABLED)
                self._mode_seen[channel] = True
        pose = drain_latest(self._subscribers[f"pose:{arm_name}"])
        grip = drain_latest(self._subscribers[f"grip:{grip_name}"])
        if previous_arm_mode is not Mode.COMMAND_FOLLOWING and self._modes[arm_name] is Mode.COMMAND_FOLLOWING:
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
        return ControlCommand(mode=self._modes[arm_name], pose_target=target,
                              gripper_target=width, gripper_mode=self._modes[grip_name],
                              engaged=target is not None)

    def modes_seen(self, arm: int = 0) -> bool:
        """True once the coordinator has actually commanded this arm's mode."""
        return all(self._mode_seen[name] for name in self._names[arm])

    def publish_state(self, state: RobotState, now: float) -> None:
        if not self.feedback_enabled:
            raise RuntimeError("This bridge only consumes commands")
        poses = state.poses or (state.pose,)
        widths = state.gripper_widths or (state.gripper_width,)
        half = len(state.q) // max(self.arms, 1)
        for arm, (arm_name, grip_name) in enumerate(self._names):
            mode = state.mode
            send(self._publishers[arm_name], DeviceChannelMode.of(_TO_WIRE[mode]))
            send(self._publishers[grip_name], DeviceChannelMode.of(_TO_WIRE[state.gripper_mode or mode]))
        age = now - state.received_at
        if (state.fault or not np.isfinite(age) or age < 0 or age >= .100
                or state.received_at <= self._last_feedback_at):
            return
        self._last_feedback_at = state.received_at
        timestamp = max(0, time.time_ns() // 1000 - int(age * 1_000_000))
        for arm, (arm_name, grip_name) in enumerate(self._names):
            pose = poses[arm] if arm < len(poses) else poses[0]
            width = widths[arm] if arm < len(widths) else widths[0]
            q = state.q[arm * half:(arm + 1) * half] if self.arms > 1 else state.q
            dq = state.dq[arm * half:(arm + 1) * half] if self.arms > 1 else state.dq
            send(self._publishers[f"pose:{arm_name}"], Pose7.from_values(timestamp, list(pose)))
            send(self._publishers[f"q:{arm_name}"], JointVector15.from_values(timestamp, list(q)))
            send(self._publishers[f"dq:{arm_name}"], JointVector15.from_values(timestamp, list(dq)))
            send(self._publishers[f"width:{grip_name}"], ParallelVector2.from_values(timestamp, [float(width)]))

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
