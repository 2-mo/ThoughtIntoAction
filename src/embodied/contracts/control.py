"""Device-independent feedback and command values."""

from dataclasses import dataclass
from enum import Enum
import numpy as np


class Mode(str, Enum):
    DISABLED = "disabled"
    FREE_DRIVE = "free-drive"
    COMMAND_FOLLOWING = "command-following"


@dataclass(frozen=True)
class XRFrame:
    sequence: int
    sdk_timestamp_ns: int
    received_at: float
    pose: np.ndarray | None
    trigger: float = 0.0
    grip: float = 0.0
    primary: bool = False
    secondary: bool = False
    connected: bool = True
    valid: bool = True
    error: str = ""


@dataclass
class RobotState:
    q: np.ndarray
    dq: np.ndarray
    pose: np.ndarray
    gripper_width: float
    received_at: float
    mode: Mode = Mode.DISABLED
    fault: str = ""
    gripper_mode: Mode | None = None


@dataclass
class ControlCommand:
    mode: Mode
    pose_target: np.ndarray | None = None
    gripper_target: float | None = None
    hold: bool = False
    engaged: bool = False
    reason: str = ""
    gripper_mode: Mode | None = None
    joint_target: np.ndarray | None = None
    joint_step_limit: float | np.ndarray | None = None
    reset_reference: bool = False
    joint_velocity_target: np.ndarray | None = None
    planned: bool = False
