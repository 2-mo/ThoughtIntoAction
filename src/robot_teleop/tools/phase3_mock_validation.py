"""Exercise the full ARKit-format UDP path against MockRobot only."""

from __future__ import annotations

import argparse
import math
import socket
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass, replace
from pathlib import Path
from typing import cast

from robot_teleop.config import TeleopProfile, load_profile
from robot_teleop.core.frames import (
    compose_pose,
    quaternion_angular_distance,
    quaternion_from_axis_angle,
    quaternion_multiply,
)
from robot_teleop.core.mapping import MappingOutputFrame
from robot_teleop.core.models import Pose, Quaternion, TrackingQuality, Vector3
from robot_teleop.core.state_machine import TeleopState
from robot_teleop.protocol.packet import (
    EStopResetAckPacket,
    EStopResetRequestPacket,
    Packet,
    PingPacket,
    PongPacket,
    PosePacket,
    SettingsAckPacket,
    SettingsPacket,
    SourceType,
    decode_packet,
    encode_packet,
)
from robot_teleop.registry import ApplicationBundle, create_application
from robot_teleop.transports.udp import UdpTransport

from ._common import print_report, validate_positive
from .phase3_mock_session import PHASE3_METRICS_SAMPLE_CAPACITY

ROOT = Path(__file__).resolve().parents[3]
IDENTITY = Quaternion(0.0, 0.0, 0.0, 1.0)


@dataclass(frozen=True, slots=True)
class Phase3MockValidationConfig:
    """Finite local scenario settings; hardware is deliberately not configurable."""

    duration_s: float = 2.0
    stream_rate_hz: float = 60.0
    burst_count: int = 128
    release_hold_budget_ms: float = 250.0
    timeout_s: float = 2.0

    def __post_init__(self) -> None:
        validate_positive("duration_s", self.duration_s)
        validate_positive("stream_rate_hz", self.stream_rate_hz)
        validate_positive("release_hold_budget_ms", self.release_hold_budget_ms)
        validate_positive("timeout_s", self.timeout_s)
        if isinstance(self.burst_count, bool) or self.burst_count < 2:
            raise ValueError("burst_count must be an integer of at least two")


class _ArkitUdpClient:
    def __init__(self, address: tuple[str, int], *, session_id: int) -> None:
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._address = address
        self._session_id = session_id
        self._pose_sequence = 0
        self._settings_sequence = 0
        self._ping_sequence = 0
        self._reset_sequence = 0
        self._last_timestamp_ns = 0

    @property
    def session_id(self) -> int:
        return self._session_id

    def close(self) -> None:
        self._socket.close()

    def begin_session(self, session_id: int) -> None:
        if session_id == self._session_id:
            raise ValueError(
                "new scenario session must differ from the current session"
            )
        self._session_id = session_id
        self._pose_sequence = 0
        self._settings_sequence = 0
        self._ping_sequence = 0
        self._reset_sequence = 0

    def next_timestamp_ns(self) -> int:
        self._last_timestamp_ns = max(time.monotonic_ns(), self._last_timestamp_ns + 1)
        return self._last_timestamp_ns

    def send_pose(
        self,
        *,
        position: Vector3,
        orientation: Quaternion = IDENTITY,
        clutch_active: bool,
        recenter_requested: bool = False,
        estop_requested: bool = False,
        tracking_quality: TrackingQuality = TrackingQuality.NORMAL,
        session_id: int | None = None,
        sequence: int | None = None,
    ) -> PosePacket:
        packet_session_id = self._session_id if session_id is None else session_id
        packet_sequence = self._pose_sequence if sequence is None else sequence
        if sequence is None:
            self._pose_sequence = (self._pose_sequence + 1) & 0xFFFFFFFF
        packet = PosePacket(
            source_type=SourceType.ARKIT,
            sequence=packet_sequence,
            session_id=packet_session_id,
            client_time_ns=self.next_timestamp_ns(),
            pose=Pose(position, orientation),
            linear_velocity=None,
            angular_velocity=None,
            connected=True,
            clutch_active=clutch_active,
            recenter_requested=recenter_requested,
            estop_requested=estop_requested,
            gripper=0.5,
            tracking_quality=tracking_quality,
        )
        self._send(packet)
        return packet

    def send_settings(
        self,
        *,
        position_scale: float,
        rotation_scale: float,
    ) -> SettingsPacket:
        packet = SettingsPacket(
            source_type=SourceType.ARKIT,
            sequence=self._settings_sequence,
            session_id=self._session_id,
            client_time_ns=self.next_timestamp_ns(),
            position_scale=position_scale,
            rotation_scale=rotation_scale,
        )
        self._settings_sequence = (self._settings_sequence + 1) & 0xFFFFFFFF
        self._send(packet)
        return packet

    def send_ping(self) -> PingPacket:
        packet = PingPacket(
            source_type=SourceType.ARKIT,
            sequence=self._ping_sequence,
            session_id=self._session_id,
            client_time_ns=self.next_timestamp_ns(),
        )
        self._ping_sequence = (self._ping_sequence + 1) & 0xFFFFFFFF
        self._send(packet)
        return packet

    def send_reset_request(self) -> EStopResetRequestPacket:
        packet = EStopResetRequestPacket(
            source_type=SourceType.ARKIT,
            sequence=self._reset_sequence,
            session_id=self._session_id,
            client_time_ns=self.next_timestamp_ns(),
        )
        self._reset_sequence = (self._reset_sequence + 1) & 0xFFFFFFFF
        self._send(packet)
        return packet

    def send_malformed(self) -> None:
        self._socket.sendto(b"\x00", self._address)

    def receive_matching(
        self,
        predicate: Callable[[Packet], bool],
        *,
        timeout_s: float,
        description: str,
    ) -> Packet:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            remaining_s = max(0.001, deadline - time.monotonic())
            self._socket.settimeout(min(0.05, remaining_s))
            try:
                payload = self._socket.recvfrom(2_048)[0]
            except TimeoutError:
                continue
            try:
                packet = decode_packet(payload, expected_source=SourceType.ARKIT)
            except ValueError:
                continue
            if predicate(packet):
                return packet
        raise RuntimeError(f"timed out waiting for {description}")

    def receives_no_match(
        self,
        predicate: Callable[[Packet], bool],
        *,
        duration_s: float,
    ) -> bool:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            remaining_s = max(0.001, deadline - time.monotonic())
            self._socket.settimeout(min(0.02, remaining_s))
            try:
                payload = self._socket.recvfrom(2_048)[0]
            except TimeoutError:
                continue
            try:
                packet = decode_packet(payload, expected_source=SourceType.ARKIT)
            except ValueError:
                continue
            if predicate(packet):
                return False
        return True

    def _send(self, packet: Packet) -> None:
        self._socket.sendto(encode_packet(packet), self._address)


def _wait_until(
    description: str,
    predicate: Callable[[], bool],
    *,
    timeout_s: float,
) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.001)
    raise RuntimeError(f"timed out waiting for {description}")


def _require(condition: bool, description: str) -> None:
    if not condition:
        raise RuntimeError(f"Phase 3 Mock validation failed: {description}")


def _metric_count(
    report: dict[str, object],
    category: str,
    name: str,
) -> int:
    raw_category = report.get(category)
    if not isinstance(raw_category, dict):
        raise RuntimeError(f"missing metric category: {category}")
    counts = cast(dict[str, object], raw_category)
    value = counts.get(name, 0)
    if isinstance(value, bool) or not isinstance(value, int):
        raise RuntimeError(f"invalid metric count: {category}.{name}")
    return value


def _pose_matches(first: Pose, second: Pose, *, tolerance: float = 2e-6) -> bool:
    return (
        first.position - second.position
    ).norm() <= tolerance and quaternion_angular_distance(
        first.orientation, second.orientation
    ) <= tolerance


def _expected_mapped_pose(
    *,
    bundle: ApplicationBundle,
    reference_pose: Pose,
    input_delta: Vector3,
    input_orientation: Quaternion,
) -> Pose:
    mapping = bundle.mapping_settings.load()
    mapped_delta = mapping.map_delta(Pose(input_delta, input_orientation))
    if mapping.output_frame is MappingOutputFrame.BASE:
        return Pose(
            reference_pose.position + mapped_delta.position,
            quaternion_multiply(mapped_delta.orientation, reference_pose.orientation),
        )
    return compose_pose(reference_pose, mapped_delta)


def _wait_for_latest(
    bundle: ApplicationBundle,
    packet: PosePacket,
    *,
    timeout_s: float,
) -> None:
    def matches() -> bool:
        snapshot = bundle.application.latest_input.load()
        return (
            snapshot is not None
            and snapshot.value.session_id == packet.session_id
            and snapshot.value.sequence == packet.sequence
        )

    _wait_until(
        f"input session {packet.session_id} sequence {packet.sequence}",
        matches,
        timeout_s=timeout_s,
    )


def _profile(config: Phase3MockValidationConfig) -> TeleopProfile:
    profile = load_profile(ROOT / "configs/phone/profiles/arkit_mock.yaml")
    if profile.udp is None:
        raise RuntimeError("ARKit Mock profile must define UDP settings")
    return replace(
        profile,
        udp=replace(
            profile.udp,
            bind_host="127.0.0.1",
            bind_port=0,
            receive_timeout_s=0.02,
        ),
        watchdog=replace(
            profile.watchdog,
            hold_packet_age_ms=100.0,
            brake_packet_age_ms=200.0,
            stop_packet_age_ms=500.0,
        ),
        state_stale_timeout_ms=2_000.0,
    )


def run_phase3_mock_validation(
    config: Phase3MockValidationConfig | None = None,
) -> dict[str, object]:
    """Run the finite Phase 3 loopback scenario without enabling hardware."""
    validation = config or Phase3MockValidationConfig()
    bundle = create_application(
        _profile(validation),
        enable_hardware=False,
        metrics_sample_capacity=PHASE3_METRICS_SAMPLE_CAPACITY,
    )
    _require(not bundle.hardware_enabled, "Mock validation must not enable hardware")
    transport = bundle.transport
    if not isinstance(transport, UdpTransport):
        raise RuntimeError("Phase 3 Mock validation requires UDP transport")
    client: _ArkitUdpClient | None = None
    checks: dict[str, bool] = {}
    release_to_hold_ms = 0.0
    local_rtt_ms = 0.0
    stream_sent = 0
    final_state = TeleopState.DISCONNECTED
    metrics_report: dict[str, object] = {}
    robot_summary: dict[str, object] = {}
    started_at = time.monotonic()

    try:
        bundle.application.start()
        client = _ArkitUdpClient(transport.local_address, session_id=101)

        origin = Vector3(0.0, 0.0, 0.0)
        baseline = client.send_pose(position=origin, clutch_active=False)
        _wait_for_latest(bundle, baseline, timeout_s=validation.timeout_s)
        initial_robot_pose = bundle.robot.state().tcp_pose

        settings = client.send_settings(position_scale=0.5, rotation_scale=0.5)
        settings_ack = client.receive_matching(
            lambda packet: isinstance(packet, SettingsAckPacket),
            timeout_s=validation.timeout_s,
            description="settings acknowledgement",
        )
        _require(isinstance(settings_ack, SettingsAckPacket), "settings ACK type")
        _require(
            settings_ack.sequence == settings.sequence
            and settings_ack.session_id == settings.session_id
            and settings_ack.client_time_ns == settings.client_time_ns,
            "settings ACK does not match the request",
        )
        _wait_until(
            "applied mapping scales",
            lambda: (
                bundle.mapping_settings.load().position_scale == 0.5
                and bundle.mapping_settings.load().rotation_scale == 0.5
            ),
            timeout_s=validation.timeout_s,
        )

        ping_started_ns = time.monotonic_ns()
        ping = client.send_ping()
        pong = client.receive_matching(
            lambda packet: isinstance(packet, PongPacket),
            timeout_s=validation.timeout_s,
            description="pong",
        )
        local_rtt_ms = (time.monotonic_ns() - ping_started_ns) / 1_000_000.0
        if not isinstance(pong, PongPacket):
            raise RuntimeError("Phase 3 Mock validation failed: pong type")
        _require(
            pong.sequence == ping.sequence
            and pong.session_id == ping.session_id
            and pong.client_time_ns == ping.client_time_ns
            and pong.server_send_time_ns >= pong.server_receive_time_ns,
            "pong does not match the ping",
        )

        input_delta = Vector3(0.0006, 0.0004, 0.0008)
        input_orientation = quaternion_from_axis_angle(Vector3(1.0, 0.0, 0.0), 0.004)
        activation = client.send_pose(position=origin, clutch_active=True)
        _wait_for_latest(bundle, activation, timeout_s=validation.timeout_s)
        _wait_until(
            "initial clutch activation",
            lambda: bundle.application.state is TeleopState.ACTIVE,
            timeout_s=validation.timeout_s,
        )
        motion = client.send_pose(
            position=input_delta,
            orientation=input_orientation,
            clutch_active=True,
        )
        _wait_for_latest(bundle, motion, timeout_s=validation.timeout_s)
        expected_motion_pose = _expected_mapped_pose(
            bundle=bundle,
            reference_pose=initial_robot_pose,
            input_delta=input_delta,
            input_orientation=input_orientation,
        )
        _wait_until(
            "mapped Cartesian pose",
            lambda: _pose_matches(bundle.robot.state().tcp_pose, expected_motion_pose),
            timeout_s=validation.timeout_s,
        )
        checks["mapping_axes_scales_orientation"] = _pose_matches(
            bundle.robot.state().tcp_pose,
            expected_motion_pose,
        )
        _require(
            checks["mapping_axes_scales_orientation"], "axis or orientation mapping"
        )

        stream_start = time.monotonic()
        stream_samples = max(
            1, math.ceil(validation.duration_s * validation.stream_rate_hz)
        )
        latest_position = input_delta
        for index in range(stream_samples):
            deadline = stream_start + index / validation.stream_rate_hz
            wait_s = deadline - time.monotonic()
            if wait_s > 0.0:
                time.sleep(wait_s)
            phase = 2.0 * math.pi * index / max(1, stream_samples)
            latest_position = Vector3(
                input_delta.x + 0.00004 * math.sin(phase),
                input_delta.y + 0.00003 * math.cos(phase),
                input_delta.z,
            )
            packet = client.send_pose(
                position=latest_position,
                orientation=input_orientation,
                clutch_active=True,
            )
            stream_sent += 1
        _wait_for_latest(bundle, packet, timeout_s=validation.timeout_s)
        expected_stream_pose = _expected_mapped_pose(
            bundle=bundle,
            reference_pose=initial_robot_pose,
            input_delta=latest_position,
            input_orientation=input_orientation,
        )
        _wait_until(
            "final streamed Cartesian pose",
            lambda: _pose_matches(
                bundle.robot.state().tcp_pose,
                expected_stream_pose,
            ),
            timeout_s=validation.timeout_s,
        )

        pose_before_recenter = bundle.robot.state().tcp_pose
        recenter = client.send_pose(
            position=latest_position,
            orientation=input_orientation,
            clutch_active=True,
            recenter_requested=True,
        )
        _wait_for_latest(bundle, recenter, timeout_s=validation.timeout_s)
        _wait_until(
            "recenter continuity",
            lambda: _pose_matches(
                bundle.robot.state().tcp_pose,
                pose_before_recenter,
            ),
            timeout_s=validation.timeout_s,
        )
        pose_after_recenter = bundle.robot.state().tcp_pose
        recenter_released = client.send_pose(
            position=latest_position,
            orientation=input_orientation,
            clutch_active=True,
        )
        _wait_for_latest(bundle, recenter_released, timeout_s=validation.timeout_s)
        _wait_until(
            "recenter release continuity",
            lambda: _pose_matches(
                bundle.robot.state().tcp_pose,
                pose_after_recenter,
            ),
            timeout_s=validation.timeout_s,
        )
        checks["clutch_recenter_continuity"] = _pose_matches(
            pose_before_recenter,
            pose_after_recenter,
        ) and _pose_matches(pose_after_recenter, bundle.robot.state().tcp_pose)
        _require(checks["clutch_recenter_continuity"], "recenter changed the target")

        holds_before_release = bundle.robot.hold_count
        release_started_ns = time.monotonic_ns()
        release = client.send_pose(
            position=latest_position,
            orientation=input_orientation,
            clutch_active=False,
        )
        _wait_for_latest(bundle, release, timeout_s=validation.timeout_s)
        _wait_until(
            "clutch release hold",
            lambda: bundle.robot.hold_count > holds_before_release,
            timeout_s=validation.timeout_s,
        )
        release_to_hold_ms = (time.monotonic_ns() - release_started_ns) / 1_000_000.0
        _require(
            release_to_hold_ms <= validation.release_hold_budget_ms,
            "clutch release exceeded the local Mock hold budget",
        )

        reactivation_reference = bundle.robot.state().tcp_pose
        reactivation = client.send_pose(
            position=latest_position,
            orientation=input_orientation,
            clutch_active=True,
        )
        _wait_for_latest(bundle, reactivation, timeout_s=validation.timeout_s)
        _wait_until(
            "clutch reactivation",
            lambda: bundle.application.state is TeleopState.ACTIVE,
            timeout_s=validation.timeout_s,
        )
        time.sleep(0.02)
        _require(
            _pose_matches(bundle.robot.state().tcp_pose, reactivation_reference),
            "clutch reactivation changed the target",
        )

        limited = client.send_pose(
            position=latest_position,
            orientation=input_orientation,
            clutch_active=True,
            tracking_quality=TrackingQuality.LIMITED,
        )
        _wait_for_latest(bundle, limited, timeout_s=validation.timeout_s)
        _wait_until(
            "degraded tracking state",
            lambda: bundle.application.state is TeleopState.DEGRADED,
            timeout_s=validation.timeout_s,
        )
        unavailable = client.send_pose(
            position=latest_position,
            orientation=input_orientation,
            clutch_active=False,
            tracking_quality=TrackingQuality.UNAVAILABLE,
        )
        _wait_for_latest(bundle, unavailable, timeout_s=validation.timeout_s)
        _wait_until(
            "tracking-loss hold",
            lambda: bundle.application.state is TeleopState.HOLD,
            timeout_s=validation.timeout_s,
        )
        recovered_release = client.send_pose(
            position=latest_position,
            orientation=input_orientation,
            clutch_active=False,
        )
        _wait_for_latest(bundle, recovered_release, timeout_s=validation.timeout_s)
        time.sleep(0.02)
        recovered = client.send_pose(
            position=latest_position,
            orientation=input_orientation,
            clutch_active=True,
        )
        _wait_for_latest(bundle, recovered, timeout_s=validation.timeout_s)
        _wait_until(
            "post-tracking clutch activation",
            lambda: bundle.application.state is TeleopState.ACTIVE,
            timeout_s=validation.timeout_s,
        )

        session_reference = bundle.robot.state().tcp_pose
        retired_session_id = client.session_id
        client.begin_session(202)
        session_release = client.send_pose(
            position=latest_position,
            orientation=input_orientation,
            clutch_active=False,
        )
        _wait_for_latest(bundle, session_release, timeout_s=validation.timeout_s)
        _wait_until(
            "new-session hold",
            lambda: bundle.application.state is TeleopState.HOLD,
            timeout_s=validation.timeout_s,
        )
        session_activation = client.send_pose(
            position=latest_position,
            orientation=input_orientation,
            clutch_active=True,
        )
        _wait_for_latest(bundle, session_activation, timeout_s=validation.timeout_s)
        _wait_until(
            "new-session activation",
            lambda: bundle.application.state is TeleopState.ACTIVE,
            timeout_s=validation.timeout_s,
        )
        time.sleep(0.02)
        session_continuity = _pose_matches(
            bundle.robot.state().tcp_pose,
            session_reference,
        )

        client.send_pose(
            position=latest_position,
            orientation=input_orientation,
            clutch_active=False,
            session_id=retired_session_id,
            sequence=9_999,
        )
        _wait_until(
            "retired-session rejection",
            lambda: (
                _metric_count(
                    bundle.metrics.report(),
                    "protocol_events",
                    "sequence.reject.retired_session",
                )
                >= 1
            ),
            timeout_s=validation.timeout_s,
        )
        checks["tracking_and_session_recovery"] = session_continuity
        _require(checks["tracking_and_session_recovery"], "recovery changed the target")

        client.send_malformed()
        _wait_until(
            "malformed packet rejection",
            lambda: (
                _metric_count(
                    bundle.metrics.report(), "protocol_events", "packet.reject.length"
                )
                >= 1
            ),
            timeout_s=validation.timeout_s,
        )
        client.send_pose(
            position=latest_position,
            orientation=input_orientation,
            clutch_active=True,
            sequence=session_activation.sequence,
        )
        _wait_until(
            "duplicate sequence rejection",
            lambda: (
                _metric_count(
                    bundle.metrics.report(),
                    "protocol_events",
                    "sequence.reject.duplicate",
                )
                >= 1
            ),
            timeout_s=validation.timeout_s,
        )
        client.send_pose(
            position=latest_position,
            orientation=input_orientation,
            clutch_active=True,
            sequence=0,
        )
        _wait_until(
            "out-of-order sequence rejection",
            lambda: (
                _metric_count(
                    bundle.metrics.report(),
                    "protocol_events",
                    "sequence.reject.out_of_order",
                )
                >= 1
            ),
            timeout_s=validation.timeout_s,
        )
        rejected_jump = client.send_pose(
            position=Vector3(1.0, 1.0, 1.0),
            orientation=input_orientation,
            clutch_active=True,
        )
        _wait_until(
            "input jump rejection",
            lambda: (
                _metric_count(
                    bundle.metrics.report(),
                    "input_rejections",
                    "input position jump exceeds the limit",
                )
                == 1
            ),
            timeout_s=validation.timeout_s,
        )
        post_jump = client.send_pose(
            position=latest_position,
            orientation=input_orientation,
            clutch_active=True,
        )
        _wait_for_latest(bundle, post_jump, timeout_s=validation.timeout_s)
        _require(
            rejected_jump.sequence + 1 == post_jump.sequence,
            "scenario jump sequence was not consecutive",
        )
        checks["invalid_input_rejection"] = True

        burst_start_sequence = post_jump.sequence + 1
        burst_packet = post_jump
        for index in range(validation.burst_count):
            burst_packet = client.send_pose(
                position=Vector3(
                    latest_position.x + (index + 1) * 0.000001,
                    latest_position.y,
                    latest_position.z,
                ),
                orientation=input_orientation,
                clutch_active=True,
            )
        _wait_for_latest(bundle, burst_packet, timeout_s=validation.timeout_s)
        _wait_until(
            "latest-only burst accounting",
            lambda: (
                _metric_count(
                    bundle.metrics.report(), "protocol_events", "udp.superseded"
                )
                > 0
            ),
            timeout_s=validation.timeout_s,
        )
        latest_snapshot = bundle.application.latest_input.load()
        checks["latest_only_burst"] = (
            latest_snapshot is not None
            and latest_snapshot.value.sequence == burst_packet.sequence
            and burst_packet.sequence - burst_start_sequence + 1
            == validation.burst_count
        )
        _require(checks["latest_only_burst"], "burst did not retain the latest pose")
        latest_position = burst_packet.pose.position

        estop = client.send_pose(
            position=latest_position,
            orientation=input_orientation,
            clutch_active=False,
            estop_requested=True,
        )
        _wait_for_latest(bundle, estop, timeout_s=validation.timeout_s)
        _wait_until(
            "E-stop state",
            lambda: bundle.application.state is TeleopState.ESTOP,
            timeout_s=validation.timeout_s,
        )
        _wait_until(
            "stopped E-stop servo",
            lambda: bundle.application.worker_count == 2,
            timeout_s=validation.timeout_s,
        )
        _require(bundle.robot.safe_stopped, "E-stop did not safe-stop MockRobot")
        reset_request = client.send_reset_request()
        reset_release = client.send_pose(
            position=latest_position,
            orientation=input_orientation,
            clutch_active=False,
            estop_requested=False,
        )
        _wait_for_latest(bundle, reset_release, timeout_s=validation.timeout_s)
        _wait_until(
            "safe E-stop release proof",
            lambda: bundle.application.estop_reset_ready,
            timeout_s=validation.timeout_s,
        )
        no_ack_before_local_reset = client.receives_no_match(
            lambda packet: isinstance(packet, EStopResetAckPacket),
            duration_s=0.05,
        )
        local_reset_completed = bundle.application.reset_estop()
        reset_ack = client.receive_matching(
            lambda packet: isinstance(packet, EStopResetAckPacket),
            timeout_s=validation.timeout_s,
            description="E-stop reset acknowledgement",
        )
        _require(isinstance(reset_ack, EStopResetAckPacket), "reset ACK type")
        _wait_until(
            "restarted servo after local reset",
            lambda: (
                bundle.application.worker_count == 3
                and bundle.application.state is TeleopState.READY
            ),
            timeout_s=validation.timeout_s,
        )
        checks["explicit_estop_reset"] = (
            no_ack_before_local_reset
            and local_reset_completed
            and reset_ack
            == EStopResetAckPacket(
                source_type=SourceType.ARKIT,
                sequence=reset_request.sequence,
                session_id=reset_request.session_id,
                client_time_ns=reset_request.client_time_ns,
            )
            and not bundle.robot.safe_stopped
        )
        _require(checks["explicit_estop_reset"], "E-stop reset was not explicit")

        reset_activation = client.send_pose(
            position=latest_position,
            orientation=input_orientation,
            clutch_active=True,
        )
        _wait_for_latest(bundle, reset_activation, timeout_s=validation.timeout_s)
        _wait_until(
            "post-reset clutch activation",
            lambda: bundle.application.state is TeleopState.ACTIVE,
            timeout_s=validation.timeout_s,
        )
        safe_stops_before_watchdog = bundle.robot.safe_stop_count
        _wait_until(
            "watchdog safe stop",
            lambda: bundle.robot.safe_stop_count > safe_stops_before_watchdog,
            timeout_s=validation.timeout_s,
        )
        metrics_report = bundle.metrics.report()
        checks["watchdog_safe_stop"] = (
            bundle.application.state is TeleopState.FAULT
            and _metric_count(metrics_report, "watchdog_stages", "hold") > 0
            and _metric_count(metrics_report, "watchdog_stages", "brake") > 0
            and _metric_count(metrics_report, "watchdog_stages", "safe_stop") >= 1
        )
        _require(checks["watchdog_safe_stop"], "watchdog did not reach safe stop")
        final_state = bundle.application.state
        robot_summary = {
            "safe_stop_count": bundle.robot.safe_stop_count,
            "hold_count": bundle.robot.hold_count,
            "stop_count": bundle.robot.stop_count,
            "command_history_count": len(bundle.robot.command_history),
            "command_history_capacity": 256,
            "safe_stopped": bundle.robot.safe_stopped,
        }
    finally:
        try:
            bundle.application.stop()
        finally:
            if client is not None:
                client.close()

    elapsed_s = time.monotonic() - started_at
    workers_stopped = (
        bundle.application.worker_count == 0 and not bundle.robot.connected
    )
    _require(workers_stopped, "Mock validation did not stop every worker")
    return {
        "hardware_enabled": False,
        "duration_s": elapsed_s,
        "stream_duration_requested_s": validation.duration_s,
        "stream_rate_hz_requested": validation.stream_rate_hz,
        "stream_pose_sent": stream_sent,
        "local_loopback_rtt_ms": local_rtt_ms,
        "clutch_release_to_hold_ms": release_to_hold_ms,
        "checks": checks,
        "metrics": metrics_report,
        "robot": robot_summary,
        "final_state": final_state.value,
        "workers_stopped": workers_stopped,
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--stream-rate-hz", type=float, default=60.0)
    parser.add_argument("--burst-count", type=int, default=128)
    parser.add_argument("--release-hold-budget-ms", type=float, default=250.0)
    parser.add_argument("--timeout", type=float, default=2.0)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    report = run_phase3_mock_validation(
        Phase3MockValidationConfig(
            duration_s=args.duration,
            stream_rate_hz=args.stream_rate_hz,
            burst_count=args.burst_count,
            release_hold_budget_ms=args.release_hold_budget_ms,
            timeout_s=args.timeout,
        )
    )
    print_report(report)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
