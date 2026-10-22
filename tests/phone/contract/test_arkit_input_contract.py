import math
import struct
import threading
from collections.abc import Callable, Iterator

from embodied.devices.phone.core.mapping import MappingScales
from embodied.devices.phone.core.models import (
    Pose,
    Quaternion,
    TrackingQuality,
    Vector3,
)
from embodied.devices.phone.inputs.arkit import (
    ARKIT_CAPABILITIES,
    ArkitInputSource,
    ArkitPacketDecoder,
)
from embodied.devices.phone.protocol.packet import (
    PingPacket,
    PongPacket,
    PosePacket,
    SettingsAckPacket,
    SettingsPacket,
    SourceType,
    decode_packet,
    encode_packet,
)
from embodied.devices.phone.transports.mock import MockTransport


class BatchMockTransport:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._running = False
        self._batch: tuple[bytes, ...] = ()
        self.sent: list[bytes] = []

    def start(self) -> None:
        with self._lock:
            self._running = True

    def inject_batch(self, *payloads: bytes) -> None:
        with self._lock:
            assert self._running
            self._batch = tuple(payloads)

    def receive(self) -> bytes | None:
        raise AssertionError("batch source must use receive_batch")

    def receive_batch(self) -> tuple[bytes, ...]:
        with self._lock:
            if not self._running:
                return ()
            batch = self._batch
            self._batch = ()
            return batch

    def send(self, payload: bytes) -> None:
        with self._lock:
            assert self._running
            self.sent.append(bytes(payload))

    def stop(self) -> None:
        with self._lock:
            self._running = False
            self._batch = ()


def packet(
    sequence: int,
    *,
    session_id: int = 7,
    timestamp_ns: int | None = None,
    tracking: TrackingQuality = TrackingQuality.NORMAL,
    clutch: bool = False,
    position_x: float = 0.0,
    linear_velocity: Vector3 | None = None,
    angular_velocity: Vector3 | None = None,
    gripper: float | None = None,
) -> PosePacket:
    return PosePacket(
        source_type=SourceType.ARKIT,
        sequence=sequence,
        session_id=session_id,
        client_time_ns=sequence * 10 if timestamp_ns is None else timestamp_ns,
        pose=Pose(
            Vector3(position_x, 0.0, 0.0),
            Quaternion(0.0, 0.0, 0.0, 1.0),
        ),
        linear_velocity=linear_velocity,
        angular_velocity=angular_velocity,
        connected=True,
        clutch_active=clutch,
        recenter_requested=False,
        estop_requested=False,
        gripper=gripper,
        tracking_quality=tracking,
    )


def source_with_events(
    *,
    clock: Iterator[int] | None = None,
    settings_callback: Callable[[MappingScales], MappingScales] | None = None,
) -> tuple[ArkitInputSource, MockTransport, dict[str, int]]:
    events: dict[str, int] = {}

    def record(event: str, count: int) -> None:
        events[event] = events.get(event, 0) + count

    transport = MockTransport()
    source = ArkitInputSource(
        transport,
        event_callback=record,
        clock=(lambda: next(clock)) if clock is not None else None,
        settings_callback=settings_callback,
    )
    return source, transport, events


def test_arkit_decoder_is_separate_and_checks_expected_source() -> None:
    decoder = ArkitPacketDecoder()

    decoded = decoder.decode(encode_packet(packet(1)))

    assert isinstance(decoded, PosePacket)
    assert decoder.capabilities == ARKIT_CAPABILITIES


def test_arkit_pose_converts_to_canonical_input() -> None:
    source, transport, events = source_with_events()
    source.start()
    try:
        transport.inject_received(
            encode_packet(
                packet(
                    1,
                    timestamp_ns=123,
                    clutch=True,
                    position_x=0.25,
                    linear_velocity=Vector3(1.0, 2.0, 3.0),
                    angular_velocity=Vector3(4.0, 5.0, 6.0),
                    gripper=0.75,
                )
            )
        )

        result = source.latest()

        assert result is not None
        assert result.sequence == 1
        assert result.session_id == 7
        assert result.timestamp_ns == 123
        assert result.pose.position.x == 0.25
        assert result.linear_velocity == Vector3(1.0, 2.0, 3.0)
        assert result.angular_velocity == Vector3(4.0, 5.0, 6.0)
        assert result.clutch_active
        assert result.gripper == 0.75
        assert result.tracking_quality is TrackingQuality.NORMAL
        assert result.source_name == "arkit"
        assert source.capabilities == ARKIT_CAPABILITIES
        assert events["packet.pose"] == 1
    finally:
        source.stop()


def test_optional_fields_and_tracking_loss_are_preserved() -> None:
    source, transport, _ = source_with_events()
    source.start()
    try:
        transport.inject_received(
            encode_packet(packet(1, tracking=TrackingQuality.UNAVAILABLE))
        )

        result = source.latest()

        assert result is not None
        assert result.tracking_quality is TrackingQuality.UNAVAILABLE
        assert result.linear_velocity is None
        assert result.angular_velocity is None
        assert result.gripper is None
    finally:
        source.stop()


def test_ping_gets_immediate_pong_without_pose_ack() -> None:
    source, transport, events = source_with_events(clock=iter((1_000, 1_010)))
    source.start()
    try:
        transport.inject_received(
            encode_packet(
                PingPacket(
                    source_type=SourceType.ARKIT,
                    sequence=9,
                    session_id=7,
                    client_time_ns=500,
                )
            )
        )

        assert source.latest() is None
        response = transport.latest_sent
        assert response is not None
        pong = decode_packet(response)
        assert pong == PongPacket(
            source_type=SourceType.ARKIT,
            sequence=9,
            session_id=7,
            client_time_ns=500,
            server_receive_time_ns=1_000,
            server_send_time_ns=1_010,
        )
        assert events["packet.ping"] == 1
        assert events["packet.pong_sent"] == 1
    finally:
        source.stop()


def test_batch_preserves_control_packets_and_publishes_only_latest_pose() -> None:
    applied: list[MappingScales] = []
    events: dict[str, int] = {}

    def record(event: str, count: int) -> None:
        events[event] = events.get(event, 0) + count

    def apply_settings(scales: MappingScales) -> MappingScales:
        applied.append(scales)
        return scales

    transport = BatchMockTransport()
    source = ArkitInputSource(
        transport,
        event_callback=record,
        settings_callback=apply_settings,
    )
    settings = SettingsPacket(
        source_type=SourceType.ARKIT,
        sequence=0,
        session_id=7,
        client_time_ns=15,
        position_scale=0.75,
        rotation_scale=0.5,
    )
    ping = PingPacket(
        source_type=SourceType.ARKIT,
        sequence=0,
        session_id=7,
        client_time_ns=25,
    )
    source.start()
    try:
        transport.inject_batch(
            encode_packet(packet(0, timestamp_ns=10)),
            encode_packet(settings),
            encode_packet(packet(1, timestamp_ns=20)),
            encode_packet(ping),
        )

        result = source.latest()

        assert result is not None
        assert result.sequence == 1
        assert applied == [MappingScales(0.75, 0.5)]
        assert len(transport.sent) == 2
        assert isinstance(decode_packet(transport.sent[0]), SettingsAckPacket)
        assert isinstance(decode_packet(transport.sent[1]), PongPacket)
        assert events["packet.pose"] == 2
        assert events["packet.settings"] == 1
        assert events["packet.ping"] == 1
        assert events["udp.superseded"] == 1
        assert "sequence.loss" not in events
    finally:
        source.stop()


def test_settings_require_active_pose_session_and_are_acknowledged() -> None:
    applied: list[MappingScales] = []

    def apply_settings(scales: MappingScales) -> MappingScales:
        applied.append(scales)
        return scales

    source, transport, events = source_with_events(settings_callback=apply_settings)
    settings = SettingsPacket(
        source_type=SourceType.ARKIT,
        sequence=3,
        session_id=7,
        client_time_ns=30,
        position_scale=0.75,
        rotation_scale=0.5,
    )
    source.start()
    try:
        transport.inject_received(encode_packet(settings))
        assert source.latest() is None
        assert applied == []
        assert events["settings.reject.session"] == 1

        transport.inject_received(encode_packet(packet(1, timestamp_ns=10)))
        assert source.latest() is not None
        transport.inject_received(encode_packet(settings))
        assert source.latest() is None

        assert applied == [MappingScales(0.75, 0.5)]
        response = transport.latest_sent
        assert response is not None
        assert decode_packet(response) == SettingsAckPacket(
            source_type=SourceType.ARKIT,
            sequence=3,
            session_id=7,
            client_time_ns=30,
            position_scale=0.75,
            rotation_scale=0.5,
        )
        assert events["packet.settings"] == 1
        assert events["packet.settings_ack_sent"] == 1

        transport.inject_received(encode_packet(settings))
        assert source.latest() is None
        assert events["settings.reject.duplicate"] == 1

        transport.inject_received(
            encode_packet(packet(0, session_id=8, timestamp_ns=40))
        )
        assert source.latest() is not None
        transport.inject_received(encode_packet(settings))
        assert source.latest() is None
        assert events["settings.reject.session"] == 2
    finally:
        source.stop()


def test_malformed_packet_is_counted_and_source_recovers() -> None:
    source, transport, events = source_with_events()
    source.start()
    try:
        malformed = bytearray(encode_packet(packet(1)))
        struct.pack_into("<f", malformed, 36, math.nan)
        transport.inject_received(bytes(malformed))
        assert source.latest() is None
        assert events["packet.reject.non_finite"] == 1

        transport.inject_received(encode_packet(packet(2)))
        assert source.latest() is not None
    finally:
        source.stop()


def test_sequence_gap_duplicate_out_of_order_and_session_retirement() -> None:
    source, transport, events = source_with_events()
    source.start()
    try:
        transport.inject_received(encode_packet(packet(10)))
        assert source.latest() is not None
        transport.inject_received(encode_packet(packet(13)))
        assert source.latest() is not None
        assert events["sequence.loss"] == 2

        transport.inject_received(encode_packet(packet(13, timestamp_ns=140)))
        assert source.latest() is None
        assert events["sequence.reject.duplicate"] == 1

        transport.inject_received(encode_packet(packet(12, timestamp_ns=150)))
        assert source.latest() is None
        assert events["sequence.reject.out_of_order"] == 1

        transport.inject_received(encode_packet(packet(0, session_id=8)))
        assert source.latest() is not None
        assert events["sequence.session_changed"] == 1

        transport.inject_received(encode_packet(packet(14, session_id=7)))
        assert source.latest() is None
        assert events["sequence.reject.retired_session"] == 1
    finally:
        source.stop()


def test_stop_is_idempotent_and_prevents_updates() -> None:
    source, transport, _ = source_with_events()
    source.start()
    source.start()
    source.stop()
    source.stop()

    assert source.latest() is None
    assert transport.receive() is None
