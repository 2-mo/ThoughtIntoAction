"""Strict semantic codec for the fixed UDP wire messages."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import IntEnum, StrEnum

from embodied.phone.core.capabilities import InputCapabilities
from embodied.phone.core.models import (
    Pose,
    Quaternion,
    TrackingQuality,
    Vector3,
)
from embodied.phone.protocol.flags import KNOWN_PACKET_FLAGS, PacketFlags
from embodied.phone.protocol.version import (
    ESTOP_RESET_PACKET_SIZE,
    HEADER_SIZE,
    HEADER_STRUCT,
    MAGIC,
    MIN_PROTOCOL_MINOR,
    PING_PACKET_SIZE,
    PONG_PACKET_SIZE,
    PONG_STRUCT,
    POSE_PACKET_SIZE,
    POSE_STRUCT,
    PROTOCOL_MAJOR,
    PROTOCOL_MINOR,
    SETTINGS_PACKET_SIZE,
    SETTINGS_STRUCT,
)

UINT32_MAX = (1 << 32) - 1
UINT64_MAX = (1 << 64) - 1
QUATERNION_NORM_MIN = 0.95
QUATERNION_NORM_MAX = 1.05
ZERO_RESERVED = b"\x00\x00\x00"


class PacketType(IntEnum):
    POSE = 1
    PING = 2
    PONG = 3
    SETTINGS = 4
    SETTINGS_ACK = 5
    ESTOP_RESET_REQUEST = 6
    ESTOP_RESET_ACK = 7


class SourceType(IntEnum):
    UNKNOWN = 0
    ARKIT = 1
    VR = 2
    SPACEMOUSE = 3
    HAND_TRACKING = 4
    TEACHING = 5
    REPLAY = 6
    MOCK = 7


class TrackingState(IntEnum):
    UNAVAILABLE = 0
    LIMITED = 1
    NORMAL = 2


class PacketErrorCode(StrEnum):
    LENGTH = "length"
    MAGIC = "magic"
    VERSION = "version"
    SIZE = "size"
    TYPE = "type"
    SOURCE = "source"
    RESERVED = "reserved"
    FLAGS = "flags"
    FIELD_RANGE = "field_range"
    NON_FINITE = "non_finite"
    QUATERNION = "quaternion"
    TRACKING = "tracking"
    CAPABILITY = "capability"
    GRIPPER = "gripper"


class PacketDecodeError(ValueError):
    """A rejected datagram with a stable metric-friendly reason code."""

    def __init__(self, code: PacketErrorCode, detail: str) -> None:
        self.code = code
        self.detail = detail
        super().__init__(f"{code.value}: {detail}")


def _uint(name: str, value: int, maximum: int, *, nonzero: bool = False) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{name} must be an integer")
    if value < int(nonzero) or value > maximum:
        qualifier = "nonzero " if nonzero else ""
        raise ValueError(f"{name} must be a {qualifier}unsigned wire integer")
    return value


def _packet_identity(
    source_type: SourceType,
    sequence: int,
    session_id: int,
    client_time_ns: int,
) -> tuple[SourceType, int, int, int]:
    source = SourceType(source_type)
    return (
        source,
        _uint("sequence", sequence, UINT32_MAX),
        _uint("session_id", session_id, UINT32_MAX, nonzero=True),
        _uint("client_time_ns", client_time_ns, UINT64_MAX),
    )


@dataclass(frozen=True, slots=True)
class PosePacket:
    source_type: SourceType
    sequence: int
    session_id: int
    client_time_ns: int
    pose: Pose
    linear_velocity: Vector3 | None
    angular_velocity: Vector3 | None
    connected: bool
    clutch_active: bool
    recenter_requested: bool
    estop_requested: bool
    gripper: float | None
    tracking_quality: TrackingQuality
    home_requested: bool = False
    joint_zero_requested: bool = False

    def __post_init__(self) -> None:
        source, sequence, session_id, client_time_ns = _packet_identity(
            self.source_type,
            self.sequence,
            self.session_id,
            self.client_time_ns,
        )
        if source is SourceType.UNKNOWN:
            raise ValueError("Pose packet source must identify an input device")
        tracking = TrackingQuality(self.tracking_quality)
        if self.clutch_active and (
            not self.connected or tracking is TrackingQuality.UNAVAILABLE
        ):
            raise ValueError("clutch requires a connected, valid tracking state")
        gripper = self.gripper
        if gripper is not None:
            gripper = float(gripper)
            if not math.isfinite(gripper) or not 0.0 <= gripper <= 1.0:
                raise ValueError("gripper must be finite and in [0, 1]")
        object.__setattr__(self, "source_type", source)
        object.__setattr__(self, "sequence", sequence)
        object.__setattr__(self, "session_id", session_id)
        object.__setattr__(self, "client_time_ns", client_time_ns)
        object.__setattr__(self, "tracking_quality", tracking)
        object.__setattr__(self, "gripper", gripper)


@dataclass(frozen=True, slots=True)
class PingPacket:
    source_type: SourceType
    sequence: int
    session_id: int
    client_time_ns: int

    def __post_init__(self) -> None:
        values = _packet_identity(
            self.source_type,
            self.sequence,
            self.session_id,
            self.client_time_ns,
        )
        object.__setattr__(self, "source_type", values[0])
        object.__setattr__(self, "sequence", values[1])
        object.__setattr__(self, "session_id", values[2])
        object.__setattr__(self, "client_time_ns", values[3])


@dataclass(frozen=True, slots=True)
class PongPacket:
    source_type: SourceType
    sequence: int
    session_id: int
    client_time_ns: int
    server_receive_time_ns: int
    server_send_time_ns: int

    def __post_init__(self) -> None:
        values = _packet_identity(
            self.source_type,
            self.sequence,
            self.session_id,
            self.client_time_ns,
        )
        receive_time = _uint(
            "server_receive_time_ns",
            self.server_receive_time_ns,
            UINT64_MAX,
        )
        send_time = _uint(
            "server_send_time_ns",
            self.server_send_time_ns,
            UINT64_MAX,
        )
        if send_time < receive_time:
            raise ValueError("server send time must not precede receive time")
        object.__setattr__(self, "source_type", values[0])
        object.__setattr__(self, "sequence", values[1])
        object.__setattr__(self, "session_id", values[2])
        object.__setattr__(self, "client_time_ns", values[3])
        object.__setattr__(self, "server_receive_time_ns", receive_time)
        object.__setattr__(self, "server_send_time_ns", send_time)


@dataclass(frozen=True, slots=True)
class EStopResetRequestPacket:
    """Phone request for a locally approved E-stop reset."""

    source_type: SourceType
    sequence: int
    session_id: int
    client_time_ns: int

    def __post_init__(self) -> None:
        values = _packet_identity(
            self.source_type,
            self.sequence,
            self.session_id,
            self.client_time_ns,
        )
        if values[0] is SourceType.UNKNOWN:
            raise ValueError(
                "E-stop reset request source must identify an input device"
            )
        object.__setattr__(self, "source_type", values[0])
        object.__setattr__(self, "sequence", values[1])
        object.__setattr__(self, "session_id", values[2])
        object.__setattr__(self, "client_time_ns", values[3])


@dataclass(frozen=True, slots=True)
class EStopResetAckPacket:
    """Ubuntu acknowledgement emitted after an explicit local reset."""

    source_type: SourceType
    sequence: int
    session_id: int
    client_time_ns: int

    def __post_init__(self) -> None:
        values = _packet_identity(
            self.source_type,
            self.sequence,
            self.session_id,
            self.client_time_ns,
        )
        if values[0] is SourceType.UNKNOWN:
            raise ValueError(
                "E-stop reset acknowledgement source must identify an input device"
            )
        object.__setattr__(self, "source_type", values[0])
        object.__setattr__(self, "sequence", values[1])
        object.__setattr__(self, "session_id", values[2])
        object.__setattr__(self, "client_time_ns", values[3])


def _settings_values(
    position_scale: float, rotation_scale: float
) -> tuple[float, float]:
    position = float(position_scale)
    rotation = float(rotation_scale)
    if not math.isfinite(position) or not math.isfinite(rotation):
        raise ValueError("mapping scales must be finite")
    if position < 0.0:
        raise ValueError("position scale must be non-negative")
    if not 0.0 <= rotation <= 1.0:
        raise ValueError("rotation scale must be in [0, 1]")
    return position, rotation


@dataclass(frozen=True, slots=True)
class SettingsPacket:
    source_type: SourceType
    sequence: int
    session_id: int
    client_time_ns: int
    position_scale: float
    rotation_scale: float

    def __post_init__(self) -> None:
        values = _packet_identity(
            self.source_type,
            self.sequence,
            self.session_id,
            self.client_time_ns,
        )
        if values[0] is SourceType.UNKNOWN:
            raise ValueError("Settings packet source must identify an input device")
        scales = _settings_values(self.position_scale, self.rotation_scale)
        object.__setattr__(self, "source_type", values[0])
        object.__setattr__(self, "sequence", values[1])
        object.__setattr__(self, "session_id", values[2])
        object.__setattr__(self, "client_time_ns", values[3])
        object.__setattr__(self, "position_scale", scales[0])
        object.__setattr__(self, "rotation_scale", scales[1])


@dataclass(frozen=True, slots=True)
class SettingsAckPacket:
    source_type: SourceType
    sequence: int
    session_id: int
    client_time_ns: int
    position_scale: float
    rotation_scale: float

    def __post_init__(self) -> None:
        values = _packet_identity(
            self.source_type,
            self.sequence,
            self.session_id,
            self.client_time_ns,
        )
        if values[0] is SourceType.UNKNOWN:
            raise ValueError("SettingsAck packet source must identify an input device")
        scales = _settings_values(self.position_scale, self.rotation_scale)
        object.__setattr__(self, "source_type", values[0])
        object.__setattr__(self, "sequence", values[1])
        object.__setattr__(self, "session_id", values[2])
        object.__setattr__(self, "client_time_ns", values[3])
        object.__setattr__(self, "position_scale", scales[0])
        object.__setattr__(self, "rotation_scale", scales[1])


type Packet = (
    PosePacket
    | PingPacket
    | PongPacket
    | SettingsPacket
    | SettingsAckPacket
    | EStopResetRequestPacket
    | EStopResetAckPacket
)


def _tracking_state(quality: TrackingQuality) -> TrackingState:
    return {
        TrackingQuality.UNAVAILABLE: TrackingState.UNAVAILABLE,
        TrackingQuality.LIMITED: TrackingState.LIMITED,
        TrackingQuality.NORMAL: TrackingState.NORMAL,
    }[quality]


def _tracking_quality(state: TrackingState) -> TrackingQuality:
    return {
        TrackingState.UNAVAILABLE: TrackingQuality.UNAVAILABLE,
        TrackingState.LIMITED: TrackingQuality.LIMITED,
        TrackingState.NORMAL: TrackingQuality.NORMAL,
    }[state]


def _pose_flags(packet: PosePacket) -> PacketFlags:
    flags = PacketFlags(0)
    if packet.connected:
        flags |= PacketFlags.CONNECTED
    if packet.clutch_active:
        flags |= PacketFlags.CLUTCH_ACTIVE
    if packet.recenter_requested:
        flags |= PacketFlags.RECENTER
    if packet.home_requested:
        flags |= PacketFlags.HOME
    if packet.joint_zero_requested:
        flags |= PacketFlags.JOINT_ZERO
    if packet.estop_requested:
        flags |= PacketFlags.ESTOP
    if packet.tracking_quality is not TrackingQuality.UNAVAILABLE:
        flags |= PacketFlags.TRACKING_VALID
    if packet.gripper is not None:
        flags |= PacketFlags.GRIPPER_VALID
    if packet.linear_velocity is not None:
        flags |= PacketFlags.LINEAR_VELOCITY_VALID
    if packet.angular_velocity is not None:
        flags |= PacketFlags.ANGULAR_VELOCITY_VALID
    return flags


def _header_values(
    packet_type: PacketType,
    size: int,
    source_type: SourceType,
    flags: int,
    sequence: int,
    session_id: int,
    client_time_ns: int,
) -> tuple[int, ...]:
    return (
        MAGIC,
        PROTOCOL_MAJOR,
        PROTOCOL_MINOR,
        size,
        packet_type,
        source_type,
        0,
        flags,
        sequence,
        session_id,
        client_time_ns,
    )


def encode_packet(packet: Packet) -> bytes:
    if isinstance(packet, PosePacket):
        linear = packet.linear_velocity or Vector3(0.0, 0.0, 0.0)
        angular = packet.angular_velocity or Vector3(0.0, 0.0, 0.0)
        gripper = packet.gripper if packet.gripper is not None else 0.0
        return POSE_STRUCT.pack(
            *_header_values(
                PacketType.POSE,
                POSE_PACKET_SIZE,
                packet.source_type,
                int(_pose_flags(packet)),
                packet.sequence,
                packet.session_id,
                packet.client_time_ns,
            ),
            *packet.pose.position.as_tuple(),
            *packet.pose.orientation.as_tuple(),
            *linear.as_tuple(),
            *angular.as_tuple(),
            gripper,
            _tracking_state(packet.tracking_quality),
            ZERO_RESERVED,
        )
    if isinstance(packet, PingPacket):
        return HEADER_STRUCT.pack(
            *_header_values(
                PacketType.PING,
                PING_PACKET_SIZE,
                packet.source_type,
                0,
                packet.sequence,
                packet.session_id,
                packet.client_time_ns,
            )
        )
    if isinstance(packet, PongPacket):
        return PONG_STRUCT.pack(
            *_header_values(
                PacketType.PONG,
                PONG_PACKET_SIZE,
                packet.source_type,
                0,
                packet.sequence,
                packet.session_id,
                packet.client_time_ns,
            ),
            packet.server_receive_time_ns,
            packet.server_send_time_ns,
        )
    if isinstance(packet, (EStopResetRequestPacket, EStopResetAckPacket)):
        packet_type = (
            PacketType.ESTOP_RESET_REQUEST
            if isinstance(packet, EStopResetRequestPacket)
            else PacketType.ESTOP_RESET_ACK
        )
        return HEADER_STRUCT.pack(
            *_header_values(
                packet_type,
                ESTOP_RESET_PACKET_SIZE,
                packet.source_type,
                0,
                packet.sequence,
                packet.session_id,
                packet.client_time_ns,
            )
        )
    if isinstance(packet, (SettingsPacket, SettingsAckPacket)):
        packet_type = (
            PacketType.SETTINGS
            if isinstance(packet, SettingsPacket)
            else PacketType.SETTINGS_ACK
        )
        return SETTINGS_STRUCT.pack(
            *_header_values(
                packet_type,
                SETTINGS_PACKET_SIZE,
                packet.source_type,
                0,
                packet.sequence,
                packet.session_id,
                packet.client_time_ns,
            ),
            packet.position_scale,
            packet.rotation_scale,
        )
    raise TypeError(f"unsupported packet object: {type(packet).__name__}")


def _error(code: PacketErrorCode, detail: str) -> PacketDecodeError:
    return PacketDecodeError(code, detail)


def _decode_header(data: bytes) -> tuple[PacketType, SourceType, int, int, int, int]:
    if len(data) < HEADER_SIZE:
        raise _error(PacketErrorCode.LENGTH, "datagram is shorter than the header")
    (
        magic,
        major,
        minor,
        declared_size,
        raw_packet_type,
        raw_source_type,
        reserved,
        raw_flags,
        sequence,
        session_id,
        client_time_ns,
    ) = HEADER_STRUCT.unpack_from(data)
    if magic != MAGIC:
        raise _error(PacketErrorCode.MAGIC, "unexpected magic")
    if major != PROTOCOL_MAJOR or not MIN_PROTOCOL_MINOR <= minor <= PROTOCOL_MINOR:
        raise _error(PacketErrorCode.VERSION, "unsupported protocol version")
    try:
        packet_type = PacketType(raw_packet_type)
    except ValueError as exc:
        raise _error(PacketErrorCode.TYPE, "unknown packet type") from exc
    expected_size = {
        PacketType.POSE: POSE_PACKET_SIZE,
        PacketType.PING: PING_PACKET_SIZE,
        PacketType.PONG: PONG_PACKET_SIZE,
        PacketType.SETTINGS: SETTINGS_PACKET_SIZE,
        PacketType.SETTINGS_ACK: SETTINGS_PACKET_SIZE,
        PacketType.ESTOP_RESET_REQUEST: ESTOP_RESET_PACKET_SIZE,
        PacketType.ESTOP_RESET_ACK: ESTOP_RESET_PACKET_SIZE,
    }[packet_type]
    if declared_size != expected_size or len(data) != expected_size:
        raise _error(PacketErrorCode.SIZE, "declared or actual packet size is invalid")
    try:
        source_type = SourceType(raw_source_type)
    except ValueError as exc:
        raise _error(PacketErrorCode.SOURCE, "unknown source type") from exc
    if (
        packet_type
        in (
            PacketType.POSE,
            PacketType.SETTINGS,
            PacketType.SETTINGS_ACK,
            PacketType.ESTOP_RESET_REQUEST,
            PacketType.ESTOP_RESET_ACK,
        )
        and source_type is SourceType.UNKNOWN
    ):
        raise _error(PacketErrorCode.SOURCE, "packet source is not an input device")
    if reserved != 0:
        raise _error(PacketErrorCode.RESERVED, "header reserved field is nonzero")
    if raw_flags & ~KNOWN_PACKET_FLAGS:
        raise _error(PacketErrorCode.FLAGS, "unknown flag bits are set")
    if packet_type is not PacketType.POSE and raw_flags:
        raise _error(PacketErrorCode.FLAGS, "control flags are invalid for this type")
    if session_id == 0:
        raise _error(PacketErrorCode.FIELD_RANGE, "session ID must be nonzero")
    return (
        packet_type,
        source_type,
        raw_flags,
        sequence,
        session_id,
        client_time_ns,
    )


def _decode_pose(
    data: bytes,
    source_type: SourceType,
    raw_flags: int,
    sequence: int,
    session_id: int,
    client_time_ns: int,
    capabilities: InputCapabilities | None,
) -> PosePacket:
    unpacked = POSE_STRUCT.unpack(data)
    raw_values = unpacked[11:25]
    if not all(math.isfinite(value) for value in raw_values):
        raise _error(PacketErrorCode.NON_FINITE, "Pose contains NaN or infinity")
    if unpacked[26] != ZERO_RESERVED:
        raise _error(PacketErrorCode.RESERVED, "Pose reserved bytes are nonzero")

    position_values = raw_values[0:3]
    quaternion_values = raw_values[3:7]
    linear_values = raw_values[7:10]
    angular_values = raw_values[10:13]
    raw_gripper = raw_values[13]
    quaternion_norm = math.sqrt(sum(value * value for value in quaternion_values))
    if not QUATERNION_NORM_MIN <= quaternion_norm <= QUATERNION_NORM_MAX:
        raise _error(PacketErrorCode.QUATERNION, "raw quaternion norm is invalid")

    try:
        tracking_state = TrackingState(unpacked[25])
    except ValueError as exc:
        raise _error(PacketErrorCode.TRACKING, "unknown tracking state") from exc
    flags = PacketFlags(raw_flags)
    if capabilities is not None:
        unsupported = (
            not capabilities.pose_6dof
            or (
                bool(flags & PacketFlags.LINEAR_VELOCITY_VALID)
                and not capabilities.linear_velocity
            )
            or (
                bool(flags & PacketFlags.ANGULAR_VELOCITY_VALID)
                and not capabilities.angular_velocity
            )
            or (
                bool(flags & PacketFlags.GRIPPER_VALID)
                and not capabilities.analog_gripper
            )
        )
        if unsupported:
            raise _error(
                PacketErrorCode.CAPABILITY,
                "validity flags exceed declared source capabilities",
            )
    tracking_valid = bool(flags & PacketFlags.TRACKING_VALID)
    if tracking_valid != (tracking_state is not TrackingState.UNAVAILABLE):
        raise _error(PacketErrorCode.TRACKING, "tracking flag and state disagree")
    connected = bool(flags & PacketFlags.CONNECTED)
    clutch_active = bool(flags & PacketFlags.CLUTCH_ACTIVE)
    if clutch_active and (not connected or not tracking_valid):
        raise _error(
            PacketErrorCode.FLAGS,
            "clutch requires connected and valid tracking flags",
        )
    gripper = raw_gripper if flags & PacketFlags.GRIPPER_VALID else None
    if gripper is not None and not 0.0 <= gripper <= 1.0:
        raise _error(PacketErrorCode.GRIPPER, "gripper is outside [0, 1]")

    return PosePacket(
        source_type=source_type,
        sequence=sequence,
        session_id=session_id,
        client_time_ns=client_time_ns,
        pose=Pose(
            position=Vector3(*position_values),
            orientation=Quaternion(*quaternion_values),
        ),
        linear_velocity=(
            Vector3(*linear_values)
            if flags & PacketFlags.LINEAR_VELOCITY_VALID
            else None
        ),
        angular_velocity=(
            Vector3(*angular_values)
            if flags & PacketFlags.ANGULAR_VELOCITY_VALID
            else None
        ),
        connected=connected,
        clutch_active=clutch_active,
        recenter_requested=bool(flags & PacketFlags.RECENTER),
        estop_requested=bool(flags & PacketFlags.ESTOP),
        gripper=gripper,
        tracking_quality=_tracking_quality(tracking_state),
        home_requested=bool(flags & PacketFlags.HOME),
        joint_zero_requested=bool(flags & PacketFlags.JOINT_ZERO),
    )


def decode_packet(
    data: bytes,
    *,
    expected_source: SourceType | None = None,
    capabilities: InputCapabilities | None = None,
) -> Packet:
    packet_type, source, flags, sequence, session_id, client_time_ns = _decode_header(
        data
    )
    if expected_source is not None and source is not SourceType(expected_source):
        raise _error(PacketErrorCode.SOURCE, "packet came from an unexpected source")
    if packet_type is PacketType.POSE:
        return _decode_pose(
            data,
            source,
            flags,
            sequence,
            session_id,
            client_time_ns,
            capabilities,
        )
    if packet_type is PacketType.PING:
        return PingPacket(source, sequence, session_id, client_time_ns)

    if packet_type in (
        PacketType.ESTOP_RESET_REQUEST,
        PacketType.ESTOP_RESET_ACK,
    ):
        reset_packet_class = (
            EStopResetRequestPacket
            if packet_type is PacketType.ESTOP_RESET_REQUEST
            else EStopResetAckPacket
        )
        return reset_packet_class(source, sequence, session_id, client_time_ns)

    if packet_type in (PacketType.SETTINGS, PacketType.SETTINGS_ACK):
        unpacked = SETTINGS_STRUCT.unpack(data)
        position_scale = unpacked[-2]
        rotation_scale = unpacked[-1]
        if not math.isfinite(position_scale) or not math.isfinite(rotation_scale):
            raise _error(
                PacketErrorCode.NON_FINITE, "Settings contains NaN or infinity"
            )
        if position_scale < 0.0 or not 0.0 <= rotation_scale <= 1.0:
            raise _error(PacketErrorCode.FIELD_RANGE, "Settings scale is invalid")
        settings_packet_class = (
            SettingsPacket if packet_type is PacketType.SETTINGS else SettingsAckPacket
        )
        return settings_packet_class(
            source_type=source,
            sequence=sequence,
            session_id=session_id,
            client_time_ns=client_time_ns,
            position_scale=position_scale,
            rotation_scale=rotation_scale,
        )

    unpacked = PONG_STRUCT.unpack(data)
    receive_time = unpacked[-2]
    send_time = unpacked[-1]
    if send_time < receive_time:
        raise _error(
            PacketErrorCode.FIELD_RANGE,
            "server send time precedes receive time",
        )
    return PongPacket(
        source_type=source,
        sequence=sequence,
        session_id=session_id,
        client_time_ns=client_time_ns,
        server_receive_time_ns=receive_time,
        server_send_time_ns=send_time,
    )
