import math
import struct
from dataclasses import replace

import pytest

from robot_teleop.core.capabilities import InputCapabilities
from robot_teleop.core.models import (
    Pose,
    Quaternion,
    TrackingQuality,
    Vector3,
)
from robot_teleop.protocol.flags import PacketFlags
from robot_teleop.protocol.packet import (
    EStopResetAckPacket,
    EStopResetRequestPacket,
    PacketDecodeError,
    PacketErrorCode,
    PacketType,
    PingPacket,
    PongPacket,
    PosePacket,
    SettingsAckPacket,
    SettingsPacket,
    SourceType,
    decode_packet,
    encode_packet,
)
from robot_teleop.protocol.version import (
    ESTOP_RESET_PACKET_SIZE,
    HEADER_STRUCT,
    PING_PACKET_SIZE,
    PONG_PACKET_SIZE,
    POSE_PACKET_SIZE,
    SETTINGS_PACKET_SIZE,
)


def pose_packet(**changes: object) -> PosePacket:
    packet = PosePacket(
        source_type=SourceType.ARKIT,
        sequence=0x10203040,
        session_id=0x50607080,
        client_time_ns=0x0102030405060708,
        pose=Pose(
            position=Vector3(0.25, -0.5, 1.0),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        ),
        linear_velocity=Vector3(0.125, -0.25, 0.5),
        angular_velocity=Vector3(-0.125, 0.25, -0.5),
        connected=True,
        clutch_active=True,
        recenter_requested=False,
        estop_requested=False,
        gripper=0.75,
        tracking_quality=TrackingQuality.NORMAL,
    )
    return replace(packet, **changes)


def mutate(data: bytes, offset: int, fmt: str, value: object) -> bytes:
    result = bytearray(data)
    struct.pack_into(fmt, result, offset, value)
    return bytes(result)


def assert_rejected(
    data: bytes,
    code: PacketErrorCode,
) -> None:
    with pytest.raises(PacketDecodeError) as error:
        decode_packet(data)
    assert error.value.code is code


def test_pose_round_trip_and_header_bytes_are_exact() -> None:
    packet = pose_packet()

    encoded = encode_packet(packet)
    decoded = decode_packet(encoded)

    assert len(encoded) == POSE_PACKET_SIZE == 96
    assert encoded[:4] == b"RTLP"
    assert HEADER_STRUCT.unpack_from(encoded) == (
        0x504C5452,
        1,
        1,
        96,
        PacketType.POSE,
        SourceType.ARKIT,
        0,
        int(
            PacketFlags.CONNECTED
            | PacketFlags.CLUTCH_ACTIVE
            | PacketFlags.TRACKING_VALID
            | PacketFlags.GRIPPER_VALID
            | PacketFlags.LINEAR_VELOCITY_VALID
            | PacketFlags.ANGULAR_VELOCITY_VALID
        ),
        0x10203040,
        0x50607080,
        0x0102030405060708,
    )
    assert decoded == packet
    assert isinstance(decoded, PosePacket)
    assert decoded.pose.orientation.as_tuple() == (0.0, 0.0, 0.0, 1.0)


def test_optional_pose_values_round_trip_as_absent() -> None:
    packet = pose_packet(
        linear_velocity=None,
        angular_velocity=None,
        gripper=None,
        clutch_active=False,
        tracking_quality=TrackingQuality.UNAVAILABLE,
    )

    assert decode_packet(encode_packet(packet)) == packet


def test_ping_and_pong_round_trip_with_fixed_sizes() -> None:
    ping = PingPacket(
        source_type=SourceType.ARKIT,
        sequence=7,
        session_id=11,
        client_time_ns=13,
    )
    pong = PongPacket(
        source_type=SourceType.ARKIT,
        sequence=7,
        session_id=11,
        client_time_ns=13,
        server_receive_time_ns=17,
        server_send_time_ns=19,
    )

    assert len(encode_packet(ping)) == PING_PACKET_SIZE == 36
    assert len(encode_packet(pong)) == PONG_PACKET_SIZE == 52
    assert decode_packet(encode_packet(ping)) == ping
    assert decode_packet(encode_packet(pong)) == pong


def test_estop_reset_request_and_ack_round_trip_with_fixed_sizes() -> None:
    request = EStopResetRequestPacket(
        source_type=SourceType.ARKIT,
        sequence=17,
        session_id=11,
        client_time_ns=23,
    )
    acknowledgement = EStopResetAckPacket(
        source_type=SourceType.ARKIT,
        sequence=17,
        session_id=11,
        client_time_ns=23,
    )

    assert len(encode_packet(request)) == ESTOP_RESET_PACKET_SIZE == 36
    assert len(encode_packet(acknowledgement)) == ESTOP_RESET_PACKET_SIZE == 36
    assert decode_packet(encode_packet(request)) == request
    assert decode_packet(encode_packet(acknowledgement)) == acknowledgement


def test_protocol_v1_0_packets_remain_decodable_after_minor_extension() -> None:
    legacy = mutate(encode_packet(pose_packet()), 6, "<H", 0)

    decoded = decode_packet(legacy)

    assert isinstance(decoded, PosePacket)
    assert decoded.sequence == 0x10203040


def test_settings_and_ack_round_trip_with_fixed_sizes() -> None:
    settings = SettingsPacket(
        source_type=SourceType.ARKIT,
        sequence=17,
        session_id=11,
        client_time_ns=23,
        position_scale=0.75,
        rotation_scale=0.5,
    )
    acknowledgement = SettingsAckPacket(
        source_type=SourceType.ARKIT,
        sequence=17,
        session_id=11,
        client_time_ns=23,
        position_scale=0.75,
        rotation_scale=0.5,
    )

    assert len(encode_packet(settings)) == SETTINGS_PACKET_SIZE == 44
    assert len(encode_packet(acknowledgement)) == SETTINGS_PACKET_SIZE == 44
    assert decode_packet(encode_packet(settings)) == settings
    assert decode_packet(encode_packet(acknowledgement)) == acknowledgement


@pytest.mark.parametrize(
    ("offset", "value", "code"),
    [
        (36, math.nan, PacketErrorCode.NON_FINITE),
        (36, -0.1, PacketErrorCode.FIELD_RANGE),
        (40, -0.1, PacketErrorCode.FIELD_RANGE),
        (40, 1.1, PacketErrorCode.FIELD_RANGE),
    ],
)
def test_invalid_settings_values_are_rejected(
    offset: int,
    value: float,
    code: PacketErrorCode,
) -> None:
    settings = encode_packet(
        SettingsPacket(
            source_type=SourceType.ARKIT,
            sequence=1,
            session_id=2,
            client_time_ns=3,
            position_scale=0.7,
            rotation_scale=0.8,
        )
    )

    assert_rejected(mutate(settings, offset, "<f", value), code)


@pytest.mark.parametrize(
    ("data", "code"),
    [
        (b"", PacketErrorCode.LENGTH),
        (encode_packet(pose_packet())[:-1], PacketErrorCode.SIZE),
        (
            mutate(encode_packet(pose_packet()), 0, "<I", 0),
            PacketErrorCode.MAGIC,
        ),
        (
            mutate(encode_packet(pose_packet()), 4, "<H", 2),
            PacketErrorCode.VERSION,
        ),
        (
            mutate(encode_packet(pose_packet()), 6, "<H", 2),
            PacketErrorCode.VERSION,
        ),
        (
            mutate(encode_packet(pose_packet()), 8, "<H", 95),
            PacketErrorCode.SIZE,
        ),
        (
            mutate(encode_packet(pose_packet()), 10, "<H", 99),
            PacketErrorCode.TYPE,
        ),
        (
            mutate(encode_packet(pose_packet()), 12, "<H", 99),
            PacketErrorCode.SOURCE,
        ),
        (
            mutate(encode_packet(pose_packet()), 14, "<H", 1),
            PacketErrorCode.RESERVED,
        ),
        (
            mutate(encode_packet(pose_packet()), 16, "<I", 1 << 31),
            PacketErrorCode.FLAGS,
        ),
        (
            mutate(encode_packet(pose_packet()), 24, "<I", 0),
            PacketErrorCode.FIELD_RANGE,
        ),
        (
            mutate(encode_packet(pose_packet()), 36, "<f", math.nan),
            PacketErrorCode.NON_FINITE,
        ),
        (
            mutate(encode_packet(pose_packet()), 48, "<f", 0.5),
            PacketErrorCode.QUATERNION,
        ),
        (
            mutate(encode_packet(pose_packet()), 93, "<B", 1),
            PacketErrorCode.RESERVED,
        ),
    ],
)
def test_invalid_packet_classes_are_rejected(
    data: bytes,
    code: PacketErrorCode,
) -> None:
    assert_rejected(data, code)


def test_tracking_flag_and_state_must_agree() -> None:
    unavailable = encode_packet(
        pose_packet(
            clutch_active=False,
            tracking_quality=TrackingQuality.UNAVAILABLE,
        )
    )

    invalid = mutate(
        unavailable,
        16,
        "<I",
        int(PacketFlags.CONNECTED | PacketFlags.TRACKING_VALID),
    )

    assert_rejected(invalid, PacketErrorCode.TRACKING)


def test_expected_source_is_checked_at_adapter_boundary() -> None:
    encoded = mutate(encode_packet(pose_packet()), 12, "<H", SourceType.MOCK)

    decoded = decode_packet(encoded)
    assert isinstance(decoded, PosePacket)
    assert decoded.source_type is SourceType.MOCK
    with pytest.raises(PacketDecodeError) as error:
        decode_packet(encoded, expected_source=SourceType.ARKIT)
    assert error.value.code is PacketErrorCode.SOURCE


def test_validity_flags_must_fit_declared_source_capabilities() -> None:
    pose_only = InputCapabilities(
        pose_6dof=True,
        linear_velocity=False,
        angular_velocity=False,
        analog_gripper=False,
        haptics=False,
        dual_hand=False,
    )

    with pytest.raises(PacketDecodeError) as error:
        decode_packet(encode_packet(pose_packet()), capabilities=pose_only)

    assert error.value.code is PacketErrorCode.CAPABILITY


def test_clutch_requires_connection_and_valid_tracking() -> None:
    packet = encode_packet(
        pose_packet(
            connected=False,
            clutch_active=False,
            tracking_quality=TrackingQuality.UNAVAILABLE,
        )
    )
    invalid = mutate(packet, 16, "<I", int(PacketFlags.CLUTCH_ACTIVE))

    assert_rejected(invalid, PacketErrorCode.FLAGS)


def test_valid_gripper_must_be_in_unit_interval() -> None:
    invalid = mutate(encode_packet(pose_packet()), 88, "<f", 1.5)

    assert_rejected(invalid, PacketErrorCode.GRIPPER)


def test_ping_rejects_pose_flags() -> None:
    ping = encode_packet(
        PingPacket(
            source_type=SourceType.ARKIT,
            sequence=1,
            session_id=2,
            client_time_ns=3,
        )
    )
    invalid = mutate(ping, 16, "<I", int(PacketFlags.CONNECTED))

    assert_rejected(invalid, PacketErrorCode.FLAGS)


@pytest.mark.parametrize(
    "packet",
    [
        EStopResetRequestPacket(
            source_type=SourceType.ARKIT,
            sequence=1,
            session_id=2,
            client_time_ns=3,
        ),
        EStopResetAckPacket(
            source_type=SourceType.ARKIT,
            sequence=1,
            session_id=2,
            client_time_ns=3,
        ),
    ],
)
def test_estop_reset_packets_reject_pose_flags(packet: object) -> None:
    assert isinstance(packet, (EStopResetRequestPacket, EStopResetAckPacket))
    invalid = mutate(encode_packet(packet), 16, "<I", int(PacketFlags.ESTOP))

    assert_rejected(invalid, PacketErrorCode.FLAGS)
