"""Stable v1 pose packet capability and state bits."""

from enum import IntFlag


class PacketFlags(IntFlag):
    CONNECTED = 1 << 0
    CLUTCH_ACTIVE = 1 << 1
    RECENTER = 1 << 2
    ESTOP = 1 << 3
    TRACKING_VALID = 1 << 4
    GRIPPER_VALID = 1 << 5
    LINEAR_VELOCITY_VALID = 1 << 6
    ANGULAR_VELOCITY_VALID = 1 << 7
    HOME = 1 << 8
    JOINT_ZERO = 1 << 9


KNOWN_PACKET_FLAGS = sum(int(flag) for flag in PacketFlags)
