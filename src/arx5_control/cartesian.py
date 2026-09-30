"""Cartesian planning helpers built on the official ARX5 KDL solver.

This module never opens a CAN interface.  It converts a small XYZ displacement
into a checked joint-space target; the hardware controller remains responsible
for trajectory generation, feedback monitoring, and emergency damping.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
from typing import Sequence

import arx5_interface as arx5
import numpy as np

from .motion.jacobian import (
    JacobianDiagnostics,
    numerical_jacobian,
    singularity_speed_scale,
)
from .motion.link_collision import (
    LinkCapsule,
    LinkCollisionCheck,
    PinocchioCollisionModel,
    default_x5_capsules,
)
from .motion.pinocchio_backend import PinocchioKinematics
from .motion.collision import (
    CollisionCheck,
    PrimitiveObstacle,
    check_tcp_path,
)


class CartesianPlanningError(RuntimeError):
    """Raised when an XYZ request cannot be converted into a safe target."""


@dataclass(frozen=True)
class CartesianPlan:
    """A fully checked, non-executing Cartesian motion preview."""

    start_pose_6d: np.ndarray
    target_pose_6d: np.ndarray
    solved_pose_6d: np.ndarray
    start_joint_position: np.ndarray
    target_joint_position: np.ndarray
    position_error_m: float
    orientation_error_rad: float
    singularity_speed_scale: float = 1.0
    minimum_jacobian_singular_value: float = math.inf

    @property
    def delta_xyz_m(self) -> np.ndarray:
        return self.target_pose_6d[:3] - self.start_pose_6d[:3]

    @property
    def max_joint_delta_rad(self) -> float:
        return float(
            np.max(np.abs(self.target_joint_position - self.start_joint_position))
        )


@dataclass(frozen=True)
class CartesianPathPlan:
    """A checked sequence of continuous IK solutions for one XYZ path."""

    final_plan: CartesianPlan
    fractions: np.ndarray
    joint_positions: np.ndarray


def _wrapped_angle_error(actual: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Return component-wise RPY error wrapped to [-pi, pi]."""
    return (actual - target + math.pi) % (2.0 * math.pi) - math.pi


def normalize_quaternion_xyzw(value: np.ndarray) -> np.ndarray:
    """Return one finite unit quaternion in browser-friendly XYZW order."""
    quaternion = np.asarray(value, dtype=np.float64).copy()
    if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
        raise CartesianPlanningError("target orientation quaternion is invalid")
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-9:
        raise CartesianPlanningError("target orientation quaternion is zero")
    return quaternion / norm


def quaternion_xyzw_to_rpy(value: np.ndarray) -> np.ndarray:
    """Convert XYZW quaternion to fixed-axis roll, pitch, yaw radians."""
    x, y, z, w = normalize_quaternion_xyzw(value)
    roll = math.atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    )
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sin_pitch)
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return np.asarray([roll, pitch, yaw], dtype=np.float64)


def rpy_to_quaternion_xyzw(value: np.ndarray) -> np.ndarray:
    """Convert fixed-axis roll, pitch, yaw radians to XYZW quaternion."""
    rpy = np.asarray(value, dtype=np.float64)
    if rpy.shape != (3,) or not np.all(np.isfinite(rpy)):
        raise CartesianPlanningError("RPY orientation is invalid")
    roll, pitch, yaw = 0.5 * rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return normalize_quaternion_xyzw(
        np.asarray(
            [
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
                cr * cp * cy + sr * sp * sy,
            ],
            dtype=np.float64,
        )
    )


def _slerp_quaternion_xyzw(
    start: np.ndarray,
    target: np.ndarray,
    fraction: float,
) -> np.ndarray:
    """Shortest-arc quaternion interpolation with stable near-zero handling."""
    first = normalize_quaternion_xyzw(start)
    second = normalize_quaternion_xyzw(target)
    dot = float(np.dot(first, second))
    if dot < 0.0:
        second = -second
        dot = -dot
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        return normalize_quaternion_xyzw(first + fraction * (second - first))
    angle = math.acos(dot)
    sine = math.sin(angle)
    return normalize_quaternion_xyzw(
        math.sin((1.0 - fraction) * angle) / sine * first
        + math.sin(fraction * angle) / sine * second
    )


class CartesianPlanner:
    """Small-step XYZ planner using ``arx5_interface.Arx5Solver``.

    The current joint position is always used as the IK seed.  This favours the
    continuous local solution and avoids randomly switching to a different arm
    configuration during beginner-oriented Cartesian moves.
    """

    def __init__(
        self,
        robot_config: arx5.RobotConfig,
        *,
        joint_limit_margin_rad: float,
        max_axis_motion_m: float,
        max_motion_m: float,
        max_joint_delta_rad: float,
        position_tolerance_m: float,
        orientation_tolerance_rad: float,
        jacobian_step_rad: float = 1e-5,
        singularity_warn: float = 0.030,
        singularity_stop: float = 0.005,
        singularity_min_speed_scale: float = 0.25,
        max_ik_step_rad: float = math.radians(12.0),
        tcp_obstacles: Sequence[PrimitiveObstacle] = (),
        tcp_clearance_m: float = 0.0,
        use_pinocchio: bool = True,
        enable_link_collision: bool = False,
        link_collision_clearance_m: float = 0.003,
        link_collision_joint_step_rad: float = math.radians(0.5),
        link_collision_capsules: Sequence[LinkCapsule] | None = None,
    ) -> None:
        self._joint_min = np.asarray(
            robot_config.joint_pos_min, dtype=np.float64
        ).copy()
        self._joint_max = np.asarray(
            robot_config.joint_pos_max, dtype=np.float64
        ).copy()
        self._joint_dof = int(robot_config.joint_dof)
        self._joint_limit_margin_rad = float(joint_limit_margin_rad)
        self._max_axis_motion_m = float(max_axis_motion_m)
        self._max_motion_m = float(max_motion_m)
        self._max_joint_delta_rad = float(max_joint_delta_rad)
        self._position_tolerance_m = float(position_tolerance_m)
        self._orientation_tolerance_rad = float(orientation_tolerance_rad)
        self._jacobian_step_rad = float(jacobian_step_rad)
        self._singularity_warn = float(singularity_warn)
        self._singularity_stop = float(singularity_stop)
        self._singularity_min_speed_scale = float(singularity_min_speed_scale)
        self._max_ik_step_rad = float(max_ik_step_rad)
        self._tcp_obstacles = tuple(tcp_obstacles)
        self._tcp_clearance_m = float(tcp_clearance_m)
        self._kinematics: PinocchioKinematics | None = None
        self._link_collision: PinocchioCollisionModel | None = None
        if (
            not math.isfinite(self._jacobian_step_rad)
            or self._jacobian_step_rad <= 0.0
            or not math.isfinite(self._singularity_warn)
            or not math.isfinite(self._singularity_stop)
            or self._singularity_warn <= self._singularity_stop
            or self._singularity_stop <= 0.0
            or not 0.0 < self._singularity_min_speed_scale <= 1.0
            or not math.isfinite(self._max_ik_step_rad)
            or self._max_ik_step_rad <= 0.0
            or not math.isfinite(self._tcp_clearance_m)
            or self._tcp_clearance_m < 0.0
            or not math.isfinite(link_collision_clearance_m)
            or link_collision_clearance_m < 0.0
            or not math.isfinite(link_collision_joint_step_rad)
            or link_collision_joint_step_rad <= 0.0
        ):
            raise ValueError("invalid Cartesian Jacobian singularity configuration")
        self._solver = arx5.Arx5Solver(
            robot_config.urdf_path,
            robot_config.joint_dof,
            robot_config.joint_pos_min,
            robot_config.joint_pos_max,
            robot_config.base_link_name,
            robot_config.eef_link_name,
            robot_config.gravity_vector,
        )
        if use_pinocchio:
            self._kinematics = PinocchioKinematics.try_create(
                robot_config.urdf_path,
                robot_config.joint_dof,
                robot_config.eef_link_name,
            )
        if enable_link_collision:
            if self._kinematics is None:
                raise ValueError(
                    "link collision requires Pinocchio; run uv sync --only-group control"
                )
            capsules = (
                default_x5_capsules()
                if link_collision_capsules is None
                else tuple(link_collision_capsules)
            )
            self._link_collision = PinocchioCollisionModel(
                self._kinematics,
                capsules,
                obstacles=self._tcp_obstacles,
                clearance_m=float(link_collision_clearance_m),
                max_joint_step_rad=float(link_collision_joint_step_rad),
            )

    @property
    def kinematics_backend(self) -> str:
        if self._kinematics is None:
            return "arx-sdk-numerical"
        return self._kinematics.backend_name

    @property
    def collision_backend(self) -> str:
        if self._link_collision is None:
            return "tcp-primitives"
        return self._link_collision.backend_name

    def plan_relative_xyz(
        self,
        start_pose_6d: np.ndarray,
        start_joint_position: np.ndarray,
        delta_xyz_m: np.ndarray,
        *,
        seed_joint_position: np.ndarray | None = None,
        target_orientation_xyzw: np.ndarray | None = None,
        unrestricted: bool = False,
    ) -> CartesianPlan:
        """Plan a base-frame XYZ displacement while preserving current RPY."""
        start_pose = np.asarray(start_pose_6d, dtype=np.float64).copy()
        start_joint = np.asarray(start_joint_position, dtype=np.float64).copy()
        seed_joint = (
            start_joint.copy()
            if seed_joint_position is None
            else np.asarray(seed_joint_position, dtype=np.float64).copy()
        )
        delta = np.asarray(delta_xyz_m, dtype=np.float64).copy()
        if start_pose.shape != (6,) or not np.all(np.isfinite(start_pose)):
            raise CartesianPlanningError("current end-effector pose is invalid")
        if start_joint.shape != (self._joint_dof,) or not np.all(
            np.isfinite(start_joint)
        ):
            raise CartesianPlanningError("current joint position is invalid")
        if seed_joint.shape != start_joint.shape or not np.all(np.isfinite(seed_joint)):
            raise CartesianPlanningError("IK seed joint position is invalid")
        if delta.shape != (3,) or not np.all(np.isfinite(delta)):
            raise CartesianPlanningError("XYZ displacement must be three finite values")
        if (
            not unrestricted
            and np.max(np.abs(delta)) > self._max_axis_motion_m + 1e-12
        ):
            raise CartesianPlanningError(
                "each XYZ axis is limited to "
                f"+/-{1000.0 * self._max_axis_motion_m:.0f} mm per move"
            )
        distance = float(np.linalg.norm(delta))
        target_orientation = (
            None
            if target_orientation_xyzw is None
            else normalize_quaternion_xyzw(target_orientation_xyzw)
        )
        orientation_distance = 0.0
        if target_orientation is not None:
            start_orientation = rpy_to_quaternion_xyzw(start_pose[3:])
            orientation_distance = 2.0 * math.acos(
                max(-1.0, min(1.0, abs(float(np.dot(
                    start_orientation,
                    target_orientation,
                )))))
            )
        if distance < 0.0001 and orientation_distance < math.radians(0.05):
            raise CartesianPlanningError("target pose is too close to the current pose")
        if not unrestricted and distance > self._max_motion_m + 1e-12:
            raise CartesianPlanningError(
                "combined XYZ displacement is limited to "
                f"{1000.0 * self._max_motion_m:.0f} mm per move"
            )

        start_singularity_scale, start_minimum = self._path_singularity_scale(
            start_joint.reshape(1, -1)
        )

        target_pose = start_pose.copy()
        target_pose[:3] += delta
        if target_orientation is not None:
            target_pose[3:] = quaternion_xyzw_to_rpy(target_orientation)
        target_joint, solved_pose, position_error, orientation_error = (
            self._checked_inverse_kinematics(
                target_pose,
                seed_joint_position=seed_joint,
                reference_joint_position=start_joint,
                enforce_joint_delta=not unrestricted,
            )
        )
        target_singularity_scale, target_minimum = self._path_singularity_scale(
            target_joint.reshape(1, -1)
        )
        self._check_joint_path_collisions(np.vstack([start_joint, target_joint]))

        return CartesianPlan(
            start_pose_6d=start_pose,
            target_pose_6d=target_pose,
            solved_pose_6d=solved_pose,
            start_joint_position=start_joint,
            target_joint_position=target_joint,
            position_error_m=position_error,
            orientation_error_rad=orientation_error,
            singularity_speed_scale=min(start_singularity_scale, target_singularity_scale),
            minimum_jacobian_singular_value=min(start_minimum, target_minimum),
        )

    def plan_relative_xyz_path(
        self,
        start_pose_6d: np.ndarray,
        start_joint_position: np.ndarray,
        delta_xyz_m: np.ndarray,
        fractions: np.ndarray,
        *,
        target_orientation_xyzw: np.ndarray | None = None,
        unrestricted: bool = False,
    ) -> CartesianPathPlan:
        """Solve all points of a pre-interpolated straight XYZ path.

        Each solution seeds the next one, while every waypoint remains checked
        against the original joint pose and the same limits as a normal preview.
        """
        start_pose = np.asarray(start_pose_6d, dtype=np.float64).copy()
        start_joint = np.asarray(start_joint_position, dtype=np.float64).copy()
        delta = np.asarray(delta_xyz_m, dtype=np.float64).copy()
        fractions = np.asarray(fractions, dtype=np.float64).copy()

        # Reuse the endpoint planner for request shape/range checks and to fail
        # quickly when even the final target has no safe IK solution.
        self.plan_relative_xyz(
            start_pose,
            start_joint,
            delta,
            target_orientation_xyzw=target_orientation_xyzw,
            unrestricted=unrestricted,
        )
        if (
            fractions.ndim != 1
            or fractions.size < 2
            or not np.all(np.isfinite(fractions))
            or fractions[0] <= 0.0
            or not math.isclose(float(fractions[-1]), 1.0, abs_tol=1e-12)
            or np.any(np.diff(fractions) <= 0.0)
            or np.any(fractions > 1.0)
        ):
            raise CartesianPlanningError(
                "path fractions must be strictly increasing in (0, 1] and end at 1"
            )

        seed = start_joint.copy()
        joint_positions: list[np.ndarray] = []
        solved_pose = start_pose.copy()
        position_error = 0.0
        orientation_error = 0.0
        start_orientation = rpy_to_quaternion_xyzw(start_pose[3:])
        target_orientation = (
            start_orientation
            if target_orientation_xyzw is None
            else normalize_quaternion_xyzw(target_orientation_xyzw)
        )
        for fraction in fractions:
            target_pose = start_pose.copy()
            target_pose[:3] += delta * float(fraction)
            target_pose[3:] = quaternion_xyzw_to_rpy(
                _slerp_quaternion_xyzw(
                    start_orientation,
                    target_orientation,
                    float(fraction),
                )
            )
            seed, solved_pose, position_error, orientation_error = (
                self._checked_inverse_kinematics(
                    target_pose,
                    seed_joint_position=seed,
                    reference_joint_position=start_joint,
                    enforce_joint_delta=not unrestricted,
                )
            )
            joint_positions.append(seed.copy())

        final_target_pose = start_pose.copy()
        final_target_pose[:3] += delta
        final_target_pose[3:] = quaternion_xyzw_to_rpy(target_orientation)
        sampled_joints = np.vstack([start_joint, np.stack(joint_positions)])
        self._check_joint_path_continuity(sampled_joints)
        path_singularity_scale, path_minimum = self._path_singularity_scale(
            sampled_joints
        )
        self._check_joint_path_collisions(sampled_joints)
        final_plan = replace(
            CartesianPlan(
            start_pose_6d=start_pose,
            target_pose_6d=final_target_pose,
            solved_pose_6d=solved_pose,
            start_joint_position=start_joint,
            target_joint_position=joint_positions[-1].copy(),
            position_error_m=position_error,
            orientation_error_rad=orientation_error,
            ),
            singularity_speed_scale=path_singularity_scale,
            minimum_jacobian_singular_value=path_minimum,
        )
        return CartesianPathPlan(
            final_plan=final_plan,
            fractions=fractions,
            joint_positions=np.stack(joint_positions),
        )

    def plan_pose_path(
        self,
        start_pose_6d: np.ndarray,
        start_joint_position: np.ndarray,
        target_poses_6d: np.ndarray,
        fractions: np.ndarray,
        *,
        unrestricted: bool = False,
    ) -> CartesianPathPlan:
        """Solve an arbitrary pre-sampled Cartesian geometry path."""
        start_pose = np.asarray(start_pose_6d, dtype=np.float64).copy()
        start_joint = np.asarray(start_joint_position, dtype=np.float64).copy()
        poses = np.asarray(target_poses_6d, dtype=np.float64).copy()
        fractions = np.asarray(fractions, dtype=np.float64).copy()
        if start_pose.shape != (6,) or not np.all(np.isfinite(start_pose)):
            raise CartesianPlanningError("current end-effector pose is invalid")
        if start_joint.shape != (self._joint_dof,) or not np.all(
            np.isfinite(start_joint)
        ):
            raise CartesianPlanningError("current joint position is invalid")
        if (
            poses.ndim != 2
            or poses.shape[0] < 2
            or poses.shape[1] != 6
            or not np.all(np.isfinite(poses))
            or fractions.shape != (poses.shape[0],)
            or not np.all(np.isfinite(fractions))
            or fractions[0] <= 0.0
            or not math.isclose(float(fractions[-1]), 1.0, abs_tol=1e-12)
            or np.any(np.diff(fractions) <= 0.0)
            or np.any(fractions > 1.0)
        ):
            raise CartesianPlanningError("sampled Cartesian pose path is invalid")
        displacements = poses[:, :3] - start_pose[:3]
        if (
            not unrestricted
            and (
                float(np.max(np.linalg.norm(displacements, axis=1)))
                > self._max_motion_m + 1e-12
                or float(np.max(np.abs(displacements)))
                > self._max_axis_motion_m + 1e-12
            )
        ):
            raise CartesianPlanningError(
                "Cartesian path excursion is limited to "
                f"{1000.0 * self._max_motion_m:.0f} mm"
            )

        seed = start_joint.copy()
        joint_positions: list[np.ndarray] = []
        solved_pose = start_pose.copy()
        position_error = 0.0
        orientation_error = 0.0
        for target_pose in poses:
            seed, solved_pose, position_error, orientation_error = (
                self._checked_inverse_kinematics(
                    target_pose,
                    seed_joint_position=seed,
                    reference_joint_position=start_joint,
                    enforce_joint_delta=not unrestricted,
                )
            )
            joint_positions.append(seed.copy())

        sampled_joints = np.vstack([start_joint, np.stack(joint_positions)])
        self._check_joint_path_continuity(sampled_joints)
        path_singularity_scale, path_minimum = self._path_singularity_scale(
            sampled_joints
        )
        self._check_joint_path_collisions(sampled_joints)
        final_plan = CartesianPlan(
            start_pose_6d=start_pose,
            target_pose_6d=poses[-1].copy(),
            solved_pose_6d=solved_pose,
            start_joint_position=start_joint,
            target_joint_position=joint_positions[-1].copy(),
            position_error_m=position_error,
            orientation_error_rad=orientation_error,
            singularity_speed_scale=path_singularity_scale,
            minimum_jacobian_singular_value=path_minimum,
        )
        return CartesianPathPlan(
            final_plan=final_plan,
            fractions=fractions,
            joint_positions=np.stack(joint_positions),
        )

    def check_tcp_collision(self, joint_positions: np.ndarray) -> CollisionCheck:
        """Check a sampled TCP path when primitive obstacles are configured."""
        if not self._tcp_obstacles:
            return CollisionCheck(True, math.inf, None, None)
        path_xyz = self.forward_kinematics_path(joint_positions)[:, :3]
        return check_tcp_path(
            path_xyz,
            self._tcp_obstacles,
            clearance_m=self._tcp_clearance_m,
        )

    def check_link_collision(
        self, joint_positions: np.ndarray
    ) -> LinkCollisionCheck:
        """Check link/self and link/environment collision along a joint path."""
        if self._link_collision is None:
            return LinkCollisionCheck(True, math.inf, None, None, None, None)
        try:
            return self._link_collision.check_path(joint_positions)
        except (TypeError, ValueError, RuntimeError) as exc:
            raise CartesianPlanningError(
                f"link collision evaluation failed: {exc}"
            ) from exc

    def _check_joint_path_collisions(self, joint_positions: np.ndarray) -> None:
        tcp_collision = self.check_tcp_collision(joint_positions)
        if not tcp_collision.safe:
            raise CartesianPlanningError(
                "TCP path collides with "
                f"{tcp_collision.obstacle_name} at waypoint "
                f"{tcp_collision.waypoint_index}"
            )
        link_collision = self.check_link_collision(joint_positions)
        if not link_collision.safe:
            raise CartesianPlanningError(
                f"{link_collision.collision_type} collision between "
                f"{link_collision.first_body} and {link_collision.second_body} "
                f"near waypoint {link_collision.waypoint_index} "
                f"(distance {1000.0 * link_collision.minimum_distance_m:.1f} mm)"
            )

    def forward_kinematics_path(self, joint_positions: np.ndarray) -> np.ndarray:
        """Resolve a checked batch of joint samples to six-dimensional poses."""
        positions = np.asarray(joint_positions, dtype=np.float64)
        if (
            positions.ndim != 2
            or positions.shape[0] < 1
            or positions.shape[1] != self._joint_dof
            or not np.all(np.isfinite(positions))
        ):
            raise CartesianPlanningError("joint trajectory has an invalid shape")
        if self._kinematics is not None:
            try:
                poses = self._kinematics.forward_kinematics_path(positions)
            except (TypeError, ValueError, RuntimeError) as exc:
                raise CartesianPlanningError(
                    f"Pinocchio forward kinematics failed: {exc}"
                ) from exc
        else:
            poses = np.asarray(
                [self._solver.forward_kinematics(position) for position in positions],
                dtype=np.float64,
            )
        if poses.shape != (positions.shape[0], 6) or not np.all(np.isfinite(poses)):
            raise CartesianPlanningError("trajectory forward kinematics failed")
        return poses

    def jacobian_diagnostics(
        self, joint_position: np.ndarray
    ) -> JacobianDiagnostics:
        """Return FK-derived Jacobian metrics for one joint position."""
        try:
            if self._kinematics is not None:
                return self._kinematics.jacobian_diagnostics(joint_position)
            return numerical_jacobian(
                self._solver.forward_kinematics,
                np.asarray(joint_position, dtype=np.float64),
                step_rad=self._jacobian_step_rad,
            )
        except (TypeError, ValueError, RuntimeError) as exc:
            raise CartesianPlanningError(
                f"Jacobian evaluation failed: {exc}"
            ) from exc

    def _path_singularity_scale(
        self, joint_positions: np.ndarray
    ) -> tuple[float, float]:
        positions = np.asarray(joint_positions, dtype=np.float64)
        if (
            positions.ndim != 2
            or positions.shape[1] != self._joint_dof
            or positions.shape[0] < 1
            or not np.all(np.isfinite(positions))
        ):
            raise CartesianPlanningError("Jacobian path has an invalid joint shape")
        minimum = math.inf
        scale = 1.0
        for position in positions:
            diagnostics = self.jacobian_diagnostics(position)
            minimum = min(minimum, diagnostics.minimum_singular_value)
            current_scale = singularity_speed_scale(
                diagnostics.minimum_singular_value,
                warn_threshold=self._singularity_warn,
                stop_threshold=self._singularity_stop,
                minimum_scale=self._singularity_min_speed_scale,
            )
            if current_scale <= 0.0:
                raise CartesianPlanningError(
                    "Cartesian path approaches a Jacobian singularity "
                    f"(minimum singular value {diagnostics.minimum_singular_value:.4f})"
                )
            scale = min(scale, current_scale)
        return float(scale), float(minimum)

    def _check_joint_path_continuity(self, joint_positions: np.ndarray) -> None:
        positions = np.asarray(joint_positions, dtype=np.float64)
        if positions.ndim != 2 or positions.shape[1] != self._joint_dof:
            raise CartesianPlanningError("IK path has an invalid joint shape")
        steps = np.abs(np.diff(positions, axis=0))
        if not steps.size:
            return
        maximum_step = float(np.max(steps))
        if maximum_step <= self._max_ik_step_rad + 1e-12:
            return
        waypoint, joint = np.unravel_index(int(np.argmax(steps)), steps.shape)
        raise CartesianPlanningError(
            "IK path is discontinuous near waypoint "
            f"{waypoint + 1}: joint {joint + 1} jumps "
            f"{math.degrees(maximum_step):.1f} deg"
        )

    def _checked_inverse_kinematics(
        self,
        target_pose: np.ndarray,
        *,
        seed_joint_position: np.ndarray,
        reference_joint_position: np.ndarray,
        enforce_joint_delta: bool = True,
    ) -> tuple[np.ndarray, np.ndarray, float, float]:
        def validate_solution(
            candidate: np.ndarray,
        ) -> tuple[np.ndarray, np.ndarray, float, float]:
            target_joint = np.asarray(candidate, dtype=np.float64).copy()
            if target_joint.shape != reference_joint_position.shape or not np.all(
                np.isfinite(target_joint)
            ):
                raise CartesianPlanningError(
                    "inverse kinematics returned an invalid target"
                )

            lower = self._joint_min + self._joint_limit_margin_rad
            upper = self._joint_max - self._joint_limit_margin_rad
            outside = (target_joint < lower) | (target_joint > upper)
            if np.any(outside):
                joint = int(np.flatnonzero(outside)[0])
                raise CartesianPlanningError(
                    f"IK target for joint {joint + 1} is too close to a software limit"
                )

            joint_delta = target_joint - reference_joint_position
            if (
                enforce_joint_delta
                and np.max(np.abs(joint_delta)) > self._max_joint_delta_rad + 1e-12
            ):
                joint = int(np.argmax(np.abs(joint_delta)))
                raise CartesianPlanningError(
                    "XYZ target would move "
                    f"joint {joint + 1} by {math.degrees(joint_delta[joint]):+.1f} deg; "
                    "the Cartesian limit is "
                    f"+/-{math.degrees(self._max_joint_delta_rad):.0f} deg"
                )

            solved_pose = np.asarray(
                self._solver.forward_kinematics(target_joint), dtype=np.float64
            ).copy()
            if solved_pose.shape != (6,) or not np.all(np.isfinite(solved_pose)):
                raise CartesianPlanningError("forward-kinematics verification failed")
            position_error = float(
                np.linalg.norm(solved_pose[:3] - target_pose[:3])
            )
            orientation_error = float(
                np.linalg.norm(
                    _wrapped_angle_error(solved_pose[3:], target_pose[3:])
                )
            )
            if position_error > self._position_tolerance_m:
                raise CartesianPlanningError(
                    "IK position residual is too large: "
                    f"{1000.0 * position_error:.2f} mm"
                )
            if orientation_error > self._orientation_tolerance_rad:
                raise CartesianPlanningError(
                    "IK orientation residual is too large: "
                    f"{math.degrees(orientation_error):.2f} deg"
                )
            return target_joint, solved_pose, position_error, orientation_error

        status, candidate = self._solver.inverse_kinematics(
            target_pose, seed_joint_position
        )
        status = int(status)
        validation_error: CartesianPlanningError | None = None
        if status == 0:
            try:
                return validate_solution(candidate)
            except CartesianPlanningError as exc:
                validation_error = exc

        multi_trial_ik = getattr(self._solver, "multi_trial_ik", None)
        if callable(multi_trial_ik):
            fallback_status, fallback_candidate = multi_trial_ik(
                target_pose,
                seed_joint_position,
                8,
            )
            fallback_status = int(fallback_status)
            if fallback_status == 0:
                try:
                    return validate_solution(fallback_candidate)
                except CartesianPlanningError as exc:
                    validation_error = exc
            status = fallback_status

        if validation_error is not None:
            raise validation_error
        status_name = self._solver.get_ik_status_name(status)
        if status == -100 and str(status_name).startswith("Unknown IK status"):
            status_name = "solver did not converge"
        raise CartesianPlanningError(
            f"inverse kinematics failed: {status_name} ({status})"
        )
