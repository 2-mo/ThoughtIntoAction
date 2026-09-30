"""Canonical protocol v1 packets shared by Python and Swift tests."""

from __future__ import annotations

import hashlib
from typing import Final

from robot_teleop.core.models import (
    Pose,
    Quaternion,
    TrackingQuality,
    Vector3,
)
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
    encode_packet,
)
from robot_teleop.protocol.version import (
    HEADER_STRUCT,
    PROTOCOL_MAJOR,
    PROTOCOL_MINOR,
)

GOLDEN_POSE: Final = PosePacket(
    source_type=SourceType.ARKIT,
    sequence=0x10203040,
    session_id=0x50607080,
    client_time_ns=0x0102030405060708,
    pose=Pose(
        position=Vector3(0.25, -0.5, 1.0),
        orientation=Quaternion(0.5, 0.5, 0.5, -0.5),
    ),
    linear_velocity=Vector3(0.125, -0.25, 0.5),
    angular_velocity=Vector3(-0.125, 0.25, -0.5),
    connected=True,
    clutch_active=True,
    recenter_requested=True,
    estop_requested=False,
    gripper=0.75,
    tracking_quality=TrackingQuality.NORMAL,
)

GOLDEN_PING: Final = PingPacket(
    source_type=SourceType.ARKIT,
    sequence=0xAABBCCDD,
    session_id=0x50607080,
    client_time_ns=0x1112131415161718,
)

GOLDEN_PONG: Final = PongPacket(
    source_type=SourceType.ARKIT,
    sequence=0xAABBCCDD,
    session_id=0x50607080,
    client_time_ns=0x1112131415161718,
    server_receive_time_ns=0x2122232425262728,
    server_send_time_ns=0x3132333435363738,
)

GOLDEN_SETTINGS: Final = SettingsPacket(
    source_type=SourceType.ARKIT,
    sequence=0x0A0B0C0D,
    session_id=0x50607080,
    client_time_ns=0x4142434445464748,
    position_scale=0.75,
    rotation_scale=0.5,
)

GOLDEN_SETTINGS_ACK: Final = SettingsAckPacket(
    source_type=SourceType.ARKIT,
    sequence=0x0A0B0C0D,
    session_id=0x50607080,
    client_time_ns=0x4142434445464748,
    position_scale=0.75,
    rotation_scale=0.5,
)

GOLDEN_ESTOP_RESET_REQUEST: Final = EStopResetRequestPacket(
    source_type=SourceType.ARKIT,
    sequence=0xD0D1D2D3,
    session_id=0x50607080,
    client_time_ns=0x5152535455565758,
)

GOLDEN_ESTOP_RESET_ACK: Final = EStopResetAckPacket(
    source_type=SourceType.ARKIT,
    sequence=0xD0D1D2D3,
    session_id=0x50607080,
    client_time_ns=0x5152535455565758,
)

GOLDEN_PACKETS: Final[dict[str, Packet]] = {
    "pose_v1.bin": GOLDEN_POSE,
    "ping_v1.bin": GOLDEN_PING,
    "pong_v1.bin": GOLDEN_PONG,
    "settings_v1.bin": GOLDEN_SETTINGS,
    "settings_ack_v1.bin": GOLDEN_SETTINGS_ACK,
    "estop_reset_request_v1.bin": GOLDEN_ESTOP_RESET_REQUEST,
    "estop_reset_ack_v1.bin": GOLDEN_ESTOP_RESET_ACK,
}


def _hex_u32(value: int) -> str:
    return f"0x{value:08x}"


def _hex_u64(value: int) -> str:
    return f"0x{value:016x}"


def _packet_fields(packet: Packet, data: bytes) -> dict[str, object]:
    header = HEADER_STRUCT.unpack_from(data)
    fields: dict[str, object] = {
        "packet_type": header[4],
        "source_type": header[5],
        "flags": _hex_u32(header[7]),
        "sequence": _hex_u32(packet.sequence),
        "session_id": _hex_u32(packet.session_id),
        "client_time_ns": _hex_u64(packet.client_time_ns),
    }
    if isinstance(packet, PosePacket):
        fields.update(
            {
                "position": list(packet.pose.position.as_tuple()),
                "quaternion_xyzw": list(packet.pose.orientation.as_tuple()),
                "linear_velocity": list(packet.linear_velocity.as_tuple())
                if packet.linear_velocity is not None
                else None,
                "angular_velocity": list(packet.angular_velocity.as_tuple())
                if packet.angular_velocity is not None
                else None,
                "gripper": packet.gripper,
                "tracking_state": 2,
            }
        )
    if isinstance(packet, PongPacket):
        fields.update(
            {
                "server_receive_time_ns": _hex_u64(packet.server_receive_time_ns),
                "server_send_time_ns": _hex_u64(packet.server_send_time_ns),
            }
        )
    if isinstance(packet, (SettingsPacket, SettingsAckPacket)):
        fields.update(
            {
                "position_scale": packet.position_scale,
                "rotation_scale": packet.rotation_scale,
            }
        )
    return fields


def golden_manifest() -> dict[str, object]:
    messages: dict[str, object] = {}
    for filename, packet in GOLDEN_PACKETS.items():
        data = encode_packet(packet)
        messages[filename] = {
            "size": len(data),
            "sha256": hashlib.sha256(data).hexdigest(),
            "fields": _packet_fields(packet, data),
        }
    return {
        "protocol": {
            "major": PROTOCOL_MAJOR,
            "minor": PROTOCOL_MINOR,
            "byte_order": "little-endian",
            "quaternion_order": "xyzw",
        },
        "messages": messages,
    }
