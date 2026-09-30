"""ARKit protocol adapter producing canonical teleoperation input."""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from typing import Protocol, runtime_checkable

from robot_teleop.core.capabilities import InputCapabilities
from robot_teleop.core.mapping import MappingScales
from robot_teleop.core.models import TeleopInput, TrackingQuality
from robot_teleop.core.ports import Transport
from robot_teleop.protocol.packet import (
    EStopResetAckPacket,
    EStopResetRequestPacket,
    Packet,
    PacketDecodeError,
    PingPacket,
    PongPacket,
    PosePacket,
    SettingsAckPacket,
    SettingsPacket,
    SourceType,
    decode_packet,
    encode_packet,
)
from robot_teleop.protocol.sequence import SequenceError, SequenceTracker

type EventCallback = Callable[[str, int], None]
type SettingsCallback = Callable[[MappingScales], MappingScales]
type EStopResetRequestCallback = Callable[[EStopResetRequestPacket], bool]


@runtime_checkable
class BatchReceivingTransport(Protocol):
    def receive_batch(self) -> tuple[bytes, ...]: ...


ARKIT_CAPABILITIES = InputCapabilities(
    pose_6dof=True,
    linear_velocity=True,
    angular_velocity=True,
    analog_gripper=True,
    haptics=True,
    dual_hand=False,
)


class ArkitPacketDecoder:
    @property
    def capabilities(self) -> InputCapabilities:
        return ARKIT_CAPABILITIES

    def decode(self, payload: bytes) -> Packet:
        return decode_packet(
            payload,
            expected_source=SourceType.ARKIT,
            capabilities=self.capabilities,
        )


class ArkitInputSource:
    def __init__(
        self,
        transport: Transport,
        *,
        decoder: ArkitPacketDecoder | None = None,
        event_callback: EventCallback | None = None,
        settings_callback: SettingsCallback | None = None,
        estop_reset_request_callback: EStopResetRequestCallback | None = None,
        clock: Callable[[], int] | None = None,
        retired_session_capacity: int = 16,
    ) -> None:
        self._transport = transport
        self._decoder = decoder or ArkitPacketDecoder()
        self._event_callback = event_callback
        self._settings_callback = settings_callback
        self._estop_reset_request_callback = estop_reset_request_callback
        self._clock = clock or time.monotonic_ns
        self._sequences = SequenceTracker(
            retired_session_capacity=retired_session_capacity
        )
        self._settings_sequences = SequenceTracker(retired_session_capacity=1)
        self._estop_reset_sequences = SequenceTracker(retired_session_capacity=1)
        self._active_pose_session_id: int | None = None
        self._lock = threading.Lock()
        self._running = False

    @property
    def capabilities(self) -> InputCapabilities:
        return self._decoder.capabilities

    def _emit(self, event: str, count: int = 1) -> None:
        callback = self._event_callback
        if callback is not None and count > 0:
            callback(event, count)

    def start(self) -> None:
        with self._lock:
            if self._running:
                return
            self._sequences.reset()
            self._settings_sequences.reset()
            self._estop_reset_sequences.reset()
            self._active_pose_session_id = None
        self._transport.start()
        with self._lock:
            self._running = True

    def stop(self) -> None:
        with self._lock:
            self._running = False
        self._transport.stop()

    def _is_running(self) -> bool:
        with self._lock:
            return self._running

    def _handle_settings(self, packet: SettingsPacket) -> None:
        if packet.session_id != self._active_pose_session_id:
            self._emit("settings.reject.session")
            return
        callback = self._settings_callback
        if callback is None:
            self._emit("settings.reject.unavailable")
            return
        try:
            self._settings_sequences.observe(
                session_id=packet.session_id,
                sequence=packet.sequence,
                timestamp_ns=packet.client_time_ns,
            )
        except SequenceError as exc:
            self._emit(f"settings.reject.{exc.code.value}")
            return
        try:
            applied = callback(
                MappingScales(packet.position_scale, packet.rotation_scale)
            )
        except ValueError:
            self._emit("settings.reject.value")
            return
        self._emit("packet.settings")
        self._transport.send(
            encode_packet(
                SettingsAckPacket(
                    source_type=packet.source_type,
                    sequence=packet.sequence,
                    session_id=packet.session_id,
                    client_time_ns=packet.client_time_ns,
                    position_scale=applied.position_scale,
                    rotation_scale=applied.rotation_scale,
                )
            )
        )
        self._emit("packet.settings_ack_sent")

    def acknowledge_estop_reset(self, request: EStopResetRequestPacket) -> None:
        """Send the exact acknowledgement after a local reset completes."""
        self._transport.send(
            encode_packet(
                EStopResetAckPacket(
                    source_type=request.source_type,
                    sequence=request.sequence,
                    session_id=request.session_id,
                    client_time_ns=request.client_time_ns,
                )
            )
        )
        self._emit("packet.estop_reset_ack_sent")

    def _handle_estop_reset_request(self, packet: EStopResetRequestPacket) -> None:
        if packet.session_id != self._active_pose_session_id:
            self._emit("estop_reset.reject.session")
            return
        try:
            self._estop_reset_sequences.observe(
                session_id=packet.session_id,
                sequence=packet.sequence,
                timestamp_ns=packet.client_time_ns,
            )
        except SequenceError as exc:
            if exc.code.value == "duplicate":
                self._emit("estop_reset.request.duplicate")
                callback = self._estop_reset_request_callback
                if callback is not None and callback(packet):
                    self.acknowledge_estop_reset(packet)
                    self._emit("packet.estop_reset_ack_retried")
                return
            self._emit(f"estop_reset.reject.{exc.code.value}")
            return
        callback = self._estop_reset_request_callback
        if callback is None:
            self._emit("estop_reset.reject.unavailable")
            return
        if callback(packet):
            self.acknowledge_estop_reset(packet)
            self._emit("packet.estop_reset_ack_retried")
        self._emit("packet.estop_reset_request")

    def latest(self) -> TeleopInput | None:
        if not self._is_running():
            return None
        if isinstance(self._transport, BatchReceivingTransport):
            payloads = self._transport.receive_batch()
        else:
            payload = self._transport.receive()
            payloads = () if payload is None else (payload,)
        if not payloads:
            return None
        latest_input: TeleopInput | None = None
        for payload in payloads:
            if not self._is_running():
                break
            input_state = self._handle_payload(payload, self._clock())
            if input_state is None:
                continue
            if latest_input is not None:
                self._emit("udp.superseded")
            latest_input = input_state
        return latest_input

    def _handle_payload(
        self,
        payload: bytes,
        received_at_ns: int,
    ) -> TeleopInput | None:
        try:
            packet = self._decoder.decode(payload)
        except PacketDecodeError as exc:
            self._emit(f"packet.reject.{exc.code.value}")
            return None

        if isinstance(packet, PingPacket):
            self._emit("packet.ping")
            send_time_ns = self._clock()
            self._transport.send(
                encode_packet(
                    PongPacket(
                        source_type=packet.source_type,
                        sequence=packet.sequence,
                        session_id=packet.session_id,
                        client_time_ns=packet.client_time_ns,
                        server_receive_time_ns=received_at_ns,
                        server_send_time_ns=send_time_ns,
                    )
                )
            )
            self._emit("packet.pong_sent")
            return None
        if isinstance(packet, PongPacket):
            self._emit("packet.pong")
            return None
        if isinstance(packet, SettingsPacket):
            self._handle_settings(packet)
            return None
        if isinstance(packet, SettingsAckPacket):
            self._emit("packet.settings_ack")
            return None
        if isinstance(packet, EStopResetRequestPacket):
            self._handle_estop_reset_request(packet)
            return None
        if isinstance(packet, EStopResetAckPacket):
            self._emit("estop_reset.reject.direction")
            return None

        assert isinstance(packet, PosePacket)
        try:
            observation = self._sequences.observe(
                session_id=packet.session_id,
                sequence=packet.sequence,
                timestamp_ns=packet.client_time_ns,
            )
        except SequenceError as exc:
            self._emit(f"sequence.reject.{exc.code.value}")
            return None
        if packet.session_id != self._active_pose_session_id:
            self._settings_sequences.reset()
            self._estop_reset_sequences.reset()
            self._active_pose_session_id = packet.session_id
        self._emit("packet.pose")
        self._emit("sequence.loss", observation.lost_packets)
        if observation.session_changed:
            self._emit("sequence.session_changed")
        tracking = (
            packet.tracking_quality if packet.connected else TrackingQuality.UNAVAILABLE
        )
        return TeleopInput(
            sequence=packet.sequence,
            session_id=packet.session_id,
            timestamp_ns=packet.client_time_ns,
            pose=packet.pose,
            linear_velocity=packet.linear_velocity,
            angular_velocity=packet.angular_velocity,
            clutch_active=packet.clutch_active,
            recenter_requested=packet.recenter_requested,
            estop_requested=packet.estop_requested,
            gripper=packet.gripper,
            tracking_quality=tracking,
            source_name="arkit",
            home_requested=packet.home_requested,
            joint_zero_requested=packet.joint_zero_requested,
        )
