"""Fixed-size, non-blocking telemetry. This channel never accepts commands."""

import ctypes as c
import math
import time

from arx5_vr.ipc.services import create_node, make_publisher, make_subscriber, send, validate_prefix

VERSION = 2
CAPACITY = 4096
FEEDBACK = 1
COMMAND = 2
POSE_TARGET = 4
TERMINAL = 8
FAULT = 16
IK_GOAL = 32
SDK_OUTPUT = 64
MODES = {"disabled": 0, "free-drive": 2, "command-following": 3}
PHASES = {"control": 1, "shutdown-parking": 2, "terminal": 3}


class RobotTickV1(c.Structure):
    _fields_ = [
        ("version", c.c_uint32), ("size", c.c_uint32), ("runtime_id", c.c_char * 48),
        ("sequence", c.c_uint64), ("dropped", c.c_uint64), ("flags", c.c_uint32),
        ("phase", c.c_uint32), ("mode", c.c_int32), ("gripper_mode", c.c_int32),
        ("command_mode", c.c_int32), ("command_gripper_mode", c.c_int32),
        ("tick_ns", c.c_uint64), ("feedback_read_ns", c.c_uint64),
        ("feedback_received_ns", c.c_uint64), ("command_started_ns", c.c_uint64),
        ("command_submitted_ns", c.c_uint64), ("sdk_readback_ns", c.c_uint64),
        ("completed_ns", c.c_uint64),
        ("q", c.c_double * 6), ("dq", c.c_double * 6), ("pose", c.c_double * 7),
        ("gripper_width", c.c_double), ("pose_target", c.c_double * 7),
        ("joint_goal", c.c_double * 6), ("sent_q", c.c_double * 6),
        ("sent_dq", c.c_double * 6), ("kp", c.c_double * 6), ("kd", c.c_double * 6),
        ("feedforward", c.c_double * 6), ("gripper_target", c.c_double),
        ("gripper_kp", c.c_double), ("gripper_kd", c.c_double),
        ("ik_residual", c.c_double), ("ik_converged", c.c_uint32),
        ("ik_rejections", c.c_uint32), ("ik_partial_solutions", c.c_uint32),
        ("sdk_q", c.c_double * 6), ("sdk_dq", c.c_double * 6),
        ("sdk_torque", c.c_double * 6), ("sdk_gripper_target", c.c_double),
        ("error", c.c_char * 192),
    ]

    @classmethod
    def type_name(cls):
        return "QuestRobotTickV1"


class RobotTickV2(c.Structure):
    _fields_ = RobotTickV1._fields_ + [
        ('source', c.c_char * 32), ('owner_epoch', c.c_uint64),
        ('source_sequence', c.c_int64), ('operation_id', c.c_char * 72),
    ]

    @classmethod
    def type_name(cls):
        return 'QuestRobotTickV2'


def layout():
    return {"version": VERSION, "size": c.sizeof(RobotTickV2), "alignment": c.alignment(RobotTickV2),
            "fields": {name: {"offset": getattr(RobotTickV2, name).offset, "size": c.sizeof(kind)}
                       for name, kind in RobotTickV2._fields_}}


def service(node, prefix):
    import iceoryx2 as iox
    validate_prefix(prefix)
    return (node.service_builder(iox.ServiceName.new(f"{prefix}/telemetry/robot_tick/v2"))
            .publish_subscribe(RobotTickV2).enable_safe_overflow(True)
            .history_size(CAPACITY).subscriber_max_buffer_size(CAPACITY)
            .max_publishers(1).max_subscribers(8).max_nodes(9).open_or_create())


def _mode(value):
    return MODES[value.value]


class TickPublisher:
    def __init__(self, runtime_id, source="quest"):
        self.runtime_id = runtime_id
        self.source = source
        self.node = create_node()
        self.publisher = make_publisher(service(self.node, runtime_id))
        self.sequence = self.dropped = 0
        self.last_error = None

    def _new(self, phase):
        packet = RobotTickV2()
        packet.version, packet.size = VERSION, c.sizeof(RobotTickV2)
        packet.runtime_id = self.runtime_id.encode()
        packet.sequence, packet.dropped = self.sequence, self.dropped
        self.sequence += 1
        packet.phase = PHASES[phase]
        packet.source = self.source.encode()[:31]
        packet.source_sequence = -1
        return packet

    def _send(self, packet):
        try:
            send(self.publisher, packet)
        except Exception as exc:
            self.dropped += 1
            self.last_error = f"{type(exc).__name__}: {exc}"

    def publish(self, state, command, backend, tick, read_ns, phase, completed, error=None, context=None):
        # Arrays are copied into the message before returning to the caller.
        packet = self._new(phase)
        try:
            if context:
                packet.source = context['source'].encode()[:31]
                packet.owner_epoch = context['epoch']
                packet.source_sequence = context.get('source_sequence', -1)
                packet.operation_id = context.get('operation_id', '').encode()[:71]
                if phase == 'control' and context.get('phase') != 'teleop':
                    packet.phase = 4  # Owned operation/hold, excluded from ordinary teleop training.
            packet.tick_ns = int(tick * 1_000_000_000)
            packet.feedback_read_ns = read_ns
            packet.feedback_received_ns = max(0, int(state.received_at * 1_000_000_000))
            packet.flags = FEEDBACK
            packet.q[:], packet.dq[:], packet.pose[:] = state.q, state.dq, state.pose
            packet.gripper_width = state.gripper_width
            packet.mode, packet.gripper_mode = _mode(state.mode), _mode(state.gripper_mode or state.mode)
            packet.command_mode = _mode(command.mode)
            packet.command_gripper_mode = _mode(command.gripper_mode or command.mode)
            if command.pose_target is not None:
                packet.flags |= POSE_TARGET
                packet.pose_target[:] = command.pose_target
            controller = backend.controller
            if controller._joint_goal is not None:
                packet.flags |= IK_GOAL
                packet.joint_goal[:] = controller._joint_goal
            packet.ik_residual = controller.ik_residual
            packet.ik_converged = controller.ik_converged
            packet.ik_rejections = controller.ik_rejections
            packet.ik_partial_solutions = controller.ik_partial_solutions
            if completed:
                info = backend.telemetry()
                packet.flags |= COMMAND
                for destination, key in (("sent_q", "sent_q"), ("sent_dq", "sent_dq"),
                                         ("kp", "applied_kp"), ("kd", "applied_kd"),
                                         ("feedforward", "user_feedforward_torque")):
                    getattr(packet, destination)[:] = info[key]
                packet.gripper_target = info["gripper_target"]
                packet.gripper_kp, packet.gripper_kd = info["gripper_applied_kp"], info["gripper_applied_kd"]
                packet.command_started_ns = info["command_started_ns"]
                packet.command_submitted_ns = info["command_submitted_ns"]
                packet.sdk_readback_ns = info.get("sdk_readback_ns", 0)
                if output := info.get("sdk_output_command"):
                    packet.flags |= SDK_OUTPUT
                    packet.sdk_q[:], packet.sdk_dq[:], packet.sdk_torque[:] = output["q"], output["dq"], output["torque"]
                    packet.sdk_gripper_target = output["gripper_pos_m"]
            if error or state.fault:
                packet.flags |= FAULT
                packet.error = str(error or state.fault).encode()[:191]
            packet.completed_ns = time.monotonic_ns()
            self._send(packet)
        except Exception as exc:
            self.dropped += 1
            self.last_error = f"{type(exc).__name__}: {exc}"

    def finish(self, error=None):
        packet = self._new("terminal")
        packet.flags = TERMINAL | (FAULT if error else 0)
        packet.error = str(error or "").encode()[:191]
        packet.completed_ns = time.monotonic_ns()
        self._send(packet)

    def status(self):
        return {"submitted": self.sequence, "dropped": self.dropped, "error": self.last_error}

    def close(self):
        self.publisher = self.node = None


class TickSubscriber:
    def __init__(self, runtime_id):
        self.node = create_node()
        self.subscriber = make_subscriber(service(self.node, runtime_id))

    def receive(self):
        sample = self.subscriber.receive()
        if sample is None:
            return None
        packet = RobotTickV2.from_buffer_copy(bytes(sample.payload().contents))
        if packet.version != VERSION or packet.size != c.sizeof(RobotTickV2):
            raise ValueError("RobotTickV2 ABI mismatch")
        return packet

    def close(self):
        self.subscriber = self.node = None


def to_dict(packet):
    """Serialize only in the observer; unavailable groups stay null."""
    def array(value):
        return [float(x) if math.isfinite(x) else None for x in value]
    result = {"type": "tick", "version": VERSION, "runtime_id": packet.runtime_id.decode(),
              "sequence": packet.sequence, "producer_dropped": packet.dropped,
              "phase": packet.phase, "terminal": bool(packet.flags & TERMINAL),
              "error": packet.error.decode(errors="replace") or None,
              "timing": {name: getattr(packet, name) for name in (
                  "tick_ns", "feedback_read_ns", "feedback_received_ns", "command_started_ns",
                  "command_submitted_ns", "sdk_readback_ns", "completed_ns")},
              "feedback": None, "command": None, "sdk_output": None, "source": packet.source.decode(),
              "source_sequence": packet.source_sequence if packet.source_sequence >= 0 else None,
              "owner_epoch": packet.owner_epoch, "operation_id": packet.operation_id.decode() or None}
    if packet.flags & FEEDBACK:
        result["feedback"] = {"q": array(packet.q), "dq": array(packet.dq), "pose": array(packet.pose),
                              "gripper_width": packet.gripper_width,
                              "mode": packet.mode, "gripper_mode": packet.gripper_mode}
        result["target"] = {"pose": array(packet.pose_target) if packet.flags & POSE_TARGET else None,
                            "joint_goal": array(packet.joint_goal) if packet.flags & IK_GOAL else None,
                            "ik_converged": bool(packet.ik_converged),
                            "ik_residual": packet.ik_residual if math.isfinite(packet.ik_residual) else None,
                            "ik_rejections": packet.ik_rejections, "ik_partial_solutions": packet.ik_partial_solutions}
    if packet.flags & COMMAND:
        result["command"] = {"q": array(packet.sent_q), "dq": array(packet.sent_dq),
                             "kp": array(packet.kp), "kd": array(packet.kd),
                             "feedforward": array(packet.feedforward), "gripper_width": packet.gripper_target,
                             "gripper_kp": packet.gripper_kp, "gripper_kd": packet.gripper_kd,
                             "mode": packet.command_mode, "gripper_mode": packet.command_gripper_mode,
                             "submitted": True}
    if packet.flags & SDK_OUTPUT:
        result["sdk_output"] = {"q": array(packet.sdk_q), "dq": array(packet.sdk_dq),
                                "torque": array(packet.sdk_torque), "gripper_width": packet.sdk_gripper_target}
    return result
