"""Native control wire ABI, adapted from the NERO driver ctypes definitions.

Names, field order, alignment, and numeric discriminants match the bundled
Rust executables. These definitions have no runtime dependency on Rollio.
"""
from __future__ import annotations
import ctypes
from typing import ClassVar
MAX_DOF = 15
MAX_PARALLEL = 2
MAX_POSE = 7

class JointVector15(ctypes.Structure):
    """Mirror of Rust `JointVector15` (`repr(C)`).

    Layout: `u64 timestamp_us` (0..8), `u32 len` (8..12), 4-byte padding,
    `f64[15] values` (16..136). Total 136 bytes, alignment 8.
    """

    _fields_: ClassVar[list[tuple[str, type]]] = [
        ("timestamp_us", ctypes.c_uint64),
        ("len", ctypes.c_uint32),
        ("values", ctypes.c_double * MAX_DOF),
    ]

    @classmethod
    def type_name(cls) -> str:
        return "JointVector15"

    @classmethod
    def from_values(cls, timestamp_us: int, values: list[float]) -> JointVector15:
        msg = cls()
        msg.timestamp_us = int(timestamp_us) & 0xFFFFFFFFFFFFFFFF
        n = min(len(values), MAX_DOF)
        msg.len = n
        for i in range(n):
            msg.values[i] = float(values[i])
        return msg


class JointMitCommand15(ctypes.Structure):
    """Mirror of Rust `JointMitCommand15` (`repr(C)`).

    Layout: `u64 timestamp_us`, `u32 len`, 4-byte padding, then five
    `f64[15]` arrays (position, velocity, effort, kp, kd). Total 616 bytes.
    """

    _fields_: ClassVar[list[tuple[str, type]]] = [
        ("timestamp_us", ctypes.c_uint64),
        ("len", ctypes.c_uint32),
        ("position", ctypes.c_double * MAX_DOF),
        ("velocity", ctypes.c_double * MAX_DOF),
        ("effort", ctypes.c_double * MAX_DOF),
        ("kp", ctypes.c_double * MAX_DOF),
        ("kd", ctypes.c_double * MAX_DOF),
    ]

    @classmethod
    def type_name(cls) -> str:
        return "JointMitCommand15"


class Pose7(ctypes.Structure):
    """Mirror of Rust `Pose7` (`repr(C)`).

    Layout: `u64 timestamp_us` (0..8), `f64[7] values` (8..64). Total 64 bytes.
    Values are `[x, y, z, qx, qy, qz, qw]` with the quaternion in xyzw order.
    """

    _fields_: ClassVar[list[tuple[str, type]]] = [
        ("timestamp_us", ctypes.c_uint64),
        ("values", ctypes.c_double * MAX_POSE),
    ]

    @classmethod
    def type_name(cls) -> str:
        return "Pose7"

    @classmethod
    def from_values(cls, timestamp_us: int, values: list[float]) -> Pose7:
        msg = cls()
        msg.timestamp_us = int(timestamp_us) & 0xFFFFFFFFFFFFFFFF
        for i in range(min(len(values), MAX_POSE)):
            msg.values[i] = float(values[i])
        return msg


class ParallelVector2(ctypes.Structure):
    """Mirror of Rust `ParallelVector2` (`repr(C)`).

    Layout: `u64 timestamp_us`, `u32 len`, 4-byte padding, `f64[2] values`.
    Total 32 bytes.
    """

    _fields_: ClassVar[list[tuple[str, type]]] = [
        ("timestamp_us", ctypes.c_uint64),
        ("len", ctypes.c_uint32),
        ("values", ctypes.c_double * MAX_PARALLEL),
    ]

    @classmethod
    def type_name(cls) -> str:
        return "ParallelVector2"

    @classmethod
    def from_values(cls, timestamp_us: int, values: list[float]) -> ParallelVector2:
        msg = cls()
        msg.timestamp_us = int(timestamp_us) & 0xFFFFFFFFFFFFFFFF
        n = min(len(values), MAX_PARALLEL)
        msg.len = n
        for i in range(n):
            msg.values[i] = float(values[i])
        return msg


class ParallelMitCommand2(ctypes.Structure):
    """Mirror of Rust `ParallelMitCommand2` (`repr(C)`).

    Layout: `u64 timestamp_us`, `u32 len`, 4-byte padding, then five
    `f64[2]` arrays (position, velocity, effort, kp, kd). Total 96 bytes.
    """

    _fields_: ClassVar[list[tuple[str, type]]] = [
        ("timestamp_us", ctypes.c_uint64),
        ("len", ctypes.c_uint32),
        ("position", ctypes.c_double * MAX_PARALLEL),
        ("velocity", ctypes.c_double * MAX_PARALLEL),
        ("effort", ctypes.c_double * MAX_PARALLEL),
        ("kp", ctypes.c_double * MAX_PARALLEL),
        ("kd", ctypes.c_double * MAX_PARALLEL),
    ]

    @classmethod
    def type_name(cls) -> str:
        return "ParallelMitCommand2"


# ---------------------------------------------------------------------------
# Enums (modelled as a single-field Structure so iceoryx2's get_type_name +
# ctypes.sizeof / alignment plumbing accepts them).
# ---------------------------------------------------------------------------


class DeviceChannelMode(ctypes.Structure):
    """Mirror of Rust `#[repr(C)] enum DeviceChannelMode` (single i32 discriminant).

    The Rust enum has no payload variants, so its `repr(C)` representation is
    a plain `c_int` on every platform we run on (x86_64 / aarch64 Linux,
    macOS). Size 4, alignment 4.
    """

    _fields_: ClassVar[list[tuple[str, type]]] = [
        ("value", ctypes.c_int32),
    ]

    @classmethod
    def type_name(cls) -> str:
        return "DeviceChannelMode"

    @classmethod
    def of(cls, discriminant: int) -> DeviceChannelMode:
        msg = cls()
        msg.value = int(discriminant)
        return msg


class XrControllerInput(ctypes.Structure):
    """40-byte native XR input; the pose shares timestamp_us."""
    _fields_ = [
        ("timestamp_us", ctypes.c_uint64),
        ("sequence", ctypes.c_uint64),
        ("connection_state", ctypes.c_uint8),
        ("tracking_valid", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8 * 2),
        ("buttons_bitset", ctypes.c_uint32),
        ("trigger", ctypes.c_float),
        ("grip", ctypes.c_float),
        ("joystick_x", ctypes.c_float),
        ("joystick_y", ctypes.c_float),
    ]

    @classmethod
    def type_name(cls) -> str:
        return "XrControllerInput"
