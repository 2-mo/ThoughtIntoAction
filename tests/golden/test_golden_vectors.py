import hashlib
import json
import struct
from pathlib import Path

from embodied.phone.protocol.golden import GOLDEN_PACKETS, golden_manifest
from embodied.phone.protocol.packet import PosePacket, decode_packet, encode_packet

GOLDEN_DIR = Path(__file__).parent


def test_checked_in_manifest_matches_canonical_definition() -> None:
    manifest = json.loads((GOLDEN_DIR / "manifest.json").read_text(encoding="utf-8"))

    assert manifest == golden_manifest()


def test_all_cross_language_message_types_have_vectors() -> None:
    assert set(GOLDEN_PACKETS) == {
        "pose_v1.bin",
        "ping_v1.bin",
        "pong_v1.bin",
        "settings_v1.bin",
        "settings_ack_v1.bin",
        "estop_reset_request_v1.bin",
        "estop_reset_ack_v1.bin",
    }


def test_checked_in_bytes_match_encoder_and_decoder() -> None:
    manifest = golden_manifest()

    for filename, packet in GOLDEN_PACKETS.items():
        data = (GOLDEN_DIR / filename).read_bytes()
        expected = manifest["messages"][filename]

        assert data == encode_packet(packet)
        assert decode_packet(data) == packet
        assert len(data) == expected["size"]
        assert hashlib.sha256(data).hexdigest() == expected["sha256"]


def test_pose_vector_freezes_xyzw_order() -> None:
    data = (GOLDEN_DIR / "pose_v1.bin").read_bytes()
    packet = GOLDEN_PACKETS["pose_v1.bin"]

    assert isinstance(packet, PosePacket)
    assert struct.unpack_from("<4f", data, 48) == (0.5, 0.5, 0.5, -0.5)
    assert packet.pose.orientation.as_tuple() == (0.5, 0.5, 0.5, -0.5)
