"""Hardware-independent contracts for the public motion API."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from enum import Enum
import math
from typing import Any

import numpy as np

from arx5_control.motion.geometry import normalize_quaternion_xyzw


def _vector(value: np.ndarray, size: int, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64).copy()
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain {size} finite values")
    result.setflags(write=False)
    return result


def quaternion_multiply_xyzw(first: np.ndarray, second: np.ndarray) -> np.ndarray:
    """Compose two XYZW rotations, applying ``second`` then ``first``."""
    ax, ay, az, aw = normalize_quaternion_xyzw(first)
    bx, by, bz, bw = normalize_quaternion_xyzw(second)
    return normalize_quaternion_xyzw(
        np.asarray(
            [
                aw * bx + ax * bw + ay * bz - az * by,
                aw * by - ax * bz + ay * bw + az * bx,
                aw * bz + ax * by - ay * bx + az * bw,
                aw * bw - ax * bx - ay * by - az * bz,
            ],
            dtype=np.float64,
        )
    )


def rotate_vector_xyzw(orientation: np.ndarray, vector: np.ndarray) -> np.ndarray:
    quaternion = normalize_quaternion_xyzw(orientation)
    xyz = _vector(vector, 3, "vector")
    q_xyz = quaternion[:3]
    cross = np.cross(q_xyz, xyz)
    return xyz + 2.0 * (quaternion[3] * cross + np.cross(q_xyz, cross))


@dataclass(frozen=True)
class Pose:
    """Rigid pose represented by metres and an XYZW quaternion."""

    position_xyz_m: np.ndarray
    orientation_xyzw: np.ndarray

    def __post_init__(self) -> None:
        position = _vector(self.position_xyz_m, 3, "pose position")
        orientation = normalize_quaternion_xyzw(self.orientation_xyzw)
        orientation.setflags(write=False)
        object.__setattr__(self, "position_xyz_m", position)
        object.__setattr__(self, "orientation_xyzw", orientation)

    @classmethod
    def identity(cls) -> Pose:
        return cls(np.zeros(3), np.asarray([0.0, 0.0, 0.0, 1.0]))

    def compose(self, child: Pose) -> Pose:
        """Return this parent pose composed with a child-frame pose."""
        if not isinstance(child, Pose):
            raise TypeError("child must be a Pose")
        return Pose(
            self.position_xyz_m
            + rotate_vector_xyzw(self.orientation_xyzw, child.position_xyz_m),
            quaternion_multiply_xyzw(
                self.orientation_xyzw,
                child.orientation_xyzw,
            ),
        )

    def inverse(self) -> Pose:
        inverse_orientation = self.orientation_xyzw.copy()
        inverse_orientation[:3] *= -1.0
        return Pose(
            -rotate_vector_xyzw(inverse_orientation, self.position_xyz_m),
            inverse_orientation,
        )


@dataclass(frozen=True)
class ToolCenterPoint:
    """Active TCP pose relative to the controller's flange/eef link."""

    name: str = "flange"
    pose_in_flange: Pose = field(default_factory=Pose.identity)

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError("TCP name must not be empty")
        if not isinstance(self.pose_in_flange, Pose):
            raise TypeError("pose_in_flange must be a Pose")


@dataclass(frozen=True)
class Payload:
    """Load attached to the active TCP, following ABB ``loaddata`` semantics."""

    name: str = "no_payload"
    mass_kg: float = 0.0
    center_of_mass_tcp_m: np.ndarray = field(
        default_factory=lambda: np.zeros(3, dtype=np.float64)
    )
    principal_inertia_kg_m2: np.ndarray = field(
        default_factory=lambda: np.zeros(3, dtype=np.float64)
    )

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError("payload name must not be empty")
        if not math.isfinite(self.mass_kg) or self.mass_kg < 0.0:
            raise ValueError("payload mass must be finite and non-negative")
        center = _vector(
            self.center_of_mass_tcp_m,
            3,
            "payload center of mass",
        )
        inertia = _vector(
            self.principal_inertia_kg_m2,
            3,
            "payload principal inertia",
        )
        if np.any(inertia < 0.0):
            raise ValueError("payload principal inertia must be non-negative")
        object.__setattr__(self, "center_of_mass_tcp_m", center)
        object.__setattr__(self, "principal_inertia_kg_m2", inertia)


@dataclass(frozen=True)
class ReferenceFrame:
    """Fixed command frame pose relative to the robot base frame."""

    name: str = "base"
    pose_in_base: Pose = field(default_factory=Pose.identity)

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError("reference frame name must not be empty")
        if not isinstance(self.pose_in_base, Pose):
            raise TypeError("pose_in_base must be a Pose")


@dataclass(frozen=True)
class MotionContext:
    """Controller-owned tool, load, frame, and global speed settings."""

    tcp: ToolCenterPoint = field(default_factory=ToolCenterPoint)
    payload: Payload = field(default_factory=Payload)
    reference_frame: ReferenceFrame = field(default_factory=ReferenceFrame)
    speed_override: float = 1.0
    revision: int = 0

    def __post_init__(self) -> None:
        if not isinstance(self.tcp, ToolCenterPoint):
            raise TypeError("tcp must be a ToolCenterPoint")
        if not isinstance(self.payload, Payload):
            raise TypeError("payload must be a Payload")
        if not isinstance(self.reference_frame, ReferenceFrame):
            raise TypeError("reference_frame must be a ReferenceFrame")
        if not math.isfinite(self.speed_override) or not 0.0 < self.speed_override <= 1.0:
            raise ValueError("speed override must be in (0, 1]")
        if isinstance(self.revision, bool) or not isinstance(self.revision, int):
            raise ValueError("motion context revision must be an integer")
        if self.revision < 0:
            raise ValueError("motion context revision must be non-negative")

    def effective_velocity_scale(self, command_scale: float) -> float:
        if not math.isfinite(command_scale) or not 0.0 < command_scale <= 1.0:
            raise ValueError("command velocity scale must be in (0, 1]")
        return float(command_scale) * self.speed_override

    def updated(self, **changes: object) -> MotionContext:
        """Return a new context and advance the plan invalidation revision."""
        if "revision" in changes:
            raise ValueError("motion context revision is controller-owned")
        return replace(self, revision=self.revision + 1, **changes)

    def command_pose_to_base(self, pose: Pose) -> Pose:
        if not isinstance(pose, Pose):
            raise TypeError("command pose must be a Pose")
        return self.reference_frame.pose_in_base.compose(pose)

    def tcp_target_to_flange(self, pose: Pose) -> Pose:
        """Resolve a command-frame TCP target into a base-frame flange pose."""
        base_tcp = self.command_pose_to_base(pose)
        return base_tcp.compose(self.tcp.pose_in_flange.inverse())

    def flange_pose_to_tcp(self, pose_in_base: Pose) -> Pose:
        if not isinstance(pose_in_base, Pose):
            raise TypeError("flange pose must be a Pose")
        return pose_in_base.compose(self.tcp.pose_in_flange)


class CartesianPathMode(str, Enum):
    LINEAR = "movel"
    CIRCULAR = "movec"


class MotionType(str, Enum):
    MOVEJ = "movej"
    MOVEL = "movel"
    MOVEC = "movec"
    MOVE_PATH = "move_path"


@dataclass(frozen=True)
class CartesianPathSegment:
    """One validated Cartesian segment for :meth:`ArmController.move_path`."""

    target: Pose
    mode: CartesianPathMode = CartesianPathMode.LINEAR
    via: Pose | None = None
    blend_radius_m: float = 0.0
    name: str | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.target, Pose):
            raise TypeError("path target must be a Pose")
        try:
            mode = CartesianPathMode(self.mode)
        except (TypeError, ValueError) as exc:
            raise ValueError("path mode must be movel or movec") from exc
        if mode == CartesianPathMode.CIRCULAR and not isinstance(self.via, Pose):
            raise ValueError("movec path segment requires a via Pose")
        if mode == CartesianPathMode.LINEAR and self.via is not None:
            raise ValueError("movel path segment must not define a via pose")
        if not math.isfinite(self.blend_radius_m) or self.blend_radius_m < 0.0:
            raise ValueError("path blend radius must be finite and non-negative")
        if self.name is not None and (
            not isinstance(self.name, str) or not self.name.strip()
        ):
            raise ValueError("path segment name must not be empty")
        object.__setattr__(self, "mode", mode)


@dataclass(frozen=True)
class MotionPlan:
    """Fully time-parameterized and validated motion ready for execution."""

    motion_type: MotionType
    context: MotionContext
    start_joint_position: np.ndarray
    target_joint_position: np.ndarray
    joint_positions: np.ndarray
    joint_velocities: np.ndarray
    sample_times_s: np.ndarray
    path_xyz_m: np.ndarray
    duration_s: float
    backend_plan: Any = field(repr=False, compare=False)
    target_tcp_pose_in_base: Pose | None = None
    start_tcp_pose_in_base: Pose | None = None
    unrestricted: bool = False

    def __post_init__(self) -> None:
        try:
            motion_type = MotionType(self.motion_type)
        except (TypeError, ValueError) as exc:
            raise ValueError("unsupported motion plan type") from exc
        if not isinstance(self.context, MotionContext):
            raise TypeError("motion plan context must be a MotionContext")
        start = np.asarray(self.start_joint_position, dtype=np.float64).copy()
        target = np.asarray(self.target_joint_position, dtype=np.float64).copy()
        positions = np.asarray(self.joint_positions, dtype=np.float64).copy()
        velocities = np.asarray(self.joint_velocities, dtype=np.float64).copy()
        times = np.asarray(self.sample_times_s, dtype=np.float64).copy()
        path = np.asarray(self.path_xyz_m, dtype=np.float64).copy()
        if (
            start.ndim != 1
            or target.shape != start.shape
            or start.size < 1
            or positions.ndim != 2
            or positions.shape[1:] != start.shape
            or velocities.shape != positions.shape
            or times.shape != (positions.shape[0],)
            or positions.shape[0] < 1
            or path.ndim != 2
            or path.shape[0] < 2
            or path.shape[1] != 3
            or not np.all(np.isfinite(start))
            or not np.all(np.isfinite(target))
            or not np.all(np.isfinite(positions))
            or not np.all(np.isfinite(velocities))
            or not np.all(np.isfinite(times))
            or not np.all(np.isfinite(path))
            or times[0] <= 0.0
            or np.any(np.diff(times) <= 0.0)
        ):
            raise ValueError("motion plan samples are invalid")
        if (
            not math.isfinite(self.duration_s)
            or self.duration_s <= 0.0
            or not math.isclose(float(times[-1]), self.duration_s, abs_tol=1e-9)
        ):
            raise ValueError("motion plan duration does not match its samples")
        if self.target_tcp_pose_in_base is not None and not isinstance(
            self.target_tcp_pose_in_base,
            Pose,
        ):
            raise TypeError("motion plan TCP target must be a Pose")
        if self.start_tcp_pose_in_base is not None and not isinstance(
            self.start_tcp_pose_in_base,
            Pose,
        ):
            raise TypeError("motion plan TCP start must be a Pose")
        if not isinstance(self.unrestricted, bool):
            raise TypeError("motion plan unrestricted flag must be boolean")
        for array in (start, target, positions, velocities, times, path):
            array.setflags(write=False)
        object.__setattr__(self, "motion_type", motion_type)
        object.__setattr__(self, "start_joint_position", start)
        object.__setattr__(self, "target_joint_position", target)
        object.__setattr__(self, "joint_positions", positions)
        object.__setattr__(self, "joint_velocities", velocities)
        object.__setattr__(self, "sample_times_s", times)
        object.__setattr__(self, "path_xyz_m", path)

    @property
    def max_joint_delta_rad(self) -> float:
        return float(
            np.max(np.abs(self.target_joint_position - self.start_joint_position))
        )

    @property
    def path_mode(self) -> str:
        return self.motion_type.value

    @property
    def plan(self) -> Any:
        """Cartesian backend plan retained while result consumers migrate."""
        direct = getattr(self.backend_plan, "plan", None)
        if direct is not None:
            return direct
        return getattr(self.backend_plan, "cartesian_plan", None)


__all__ = [
    "CartesianPathMode",
    "CartesianPathSegment",
    "MotionContext",
    "MotionPlan",
    "MotionType",
    "Payload",
    "Pose",
    "ReferenceFrame",
    "ToolCenterPoint",
    "quaternion_multiply_xyzw",
    "rotate_vector_xyzw",
]
