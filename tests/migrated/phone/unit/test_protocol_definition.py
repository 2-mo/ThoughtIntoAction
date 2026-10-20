import struct

from embodied.phone.protocol.flags import PacketFlags
from embodied.phone.protocol.version import (
    ESTOP_RESET_PACKET_SIZE,
    HEADER_FORMAT,
    HEADER_SIZE,
    MAGIC,
    MAGIC_BYTES,
    PING_PACKET_SIZE,
    PONG_PACKET_FORMAT,
    PONG_PACKET_SIZE,
    POSE_PACKET_FORMAT,
    POSE_PACKET_SIZE,
    PROTOCOL_MAJOR,
    PROTOCOL_MINOR,
    SETTINGS_PACKET_FORMAT,
    SETTINGS_PACKET_SIZE,
)


def test_protocol_layout_is_fixed_and_little_endian() -> None:
    assert HEADER_FORMAT.startswith("<")
    assert POSE_PACKET_FORMAT.startswith("<")
    assert PONG_PACKET_FORMAT.startswith("<")
    assert SETTINGS_PACKET_FORMAT.startswith("<")
    assert struct.calcsize(HEADER_FORMAT) == HEADER_SIZE == 36
    assert struct.calcsize(POSE_PACKET_FORMAT) == POSE_PACKET_SIZE == 96
    assert PING_PACKET_SIZE == HEADER_SIZE == 36
    assert ESTOP_RESET_PACKET_SIZE == HEADER_SIZE == 36
    assert struct.calcsize(PONG_PACKET_FORMAT) == PONG_PACKET_SIZE == 52
    assert struct.calcsize(SETTINGS_PACKET_FORMAT) == SETTINGS_PACKET_SIZE == 44
    assert struct.pack("<I", MAGIC) == MAGIC_BYTES == b"RTLP"
    assert (PROTOCOL_MAJOR, PROTOCOL_MINOR) == (1, 1)


def test_required_flags_have_stable_bits() -> None:
    assert PacketFlags.CONNECTED == 1 << 0
    assert PacketFlags.CLUTCH_ACTIVE == 1 << 1
    assert PacketFlags.RECENTER == 1 << 2
    assert PacketFlags.ESTOP == 1 << 3
    assert PacketFlags.TRACKING_VALID == 1 << 4
    assert PacketFlags.GRIPPER_VALID == 1 << 5
    assert PacketFlags.LINEAR_VELOCITY_VALID == 1 << 6
    assert PacketFlags.ANGULAR_VELOCITY_VALID == 1 << 7
