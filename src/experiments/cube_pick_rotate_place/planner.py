"""Geometry, preflight, and execution for the cube manipulation sequence.

This module is the single source of truth for cube grasp geometry. Both the
terminal experiment and the web control service plan against it, so a tabletop
bound or gripper orientation fixed here applies to every entry point.

It deliberately avoids OpenCV and pyrealsense2 so the control-only ``.venv``
can import it; ``CubeObservation`` is a typing-only import for that reason.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
import math
import time
from typing import TYPE_CHECKING, Any, Callable

import numpy as np

from arx5_control import CartesianPathSegment, Pose

from arx5_control.cartesian import normalize_quaternion_xyzw
from hand_eye_calibration.core.transforms import quaternion_xyzw_from_matrix, rpy_to_matrix

from .config import ExperimentConfig, MotionConfig

if TYPE_CHECKING:
    # Perception pulls in OpenCV. Keeping it behind TYPE_CHECKING lets the
    # control-only .venv import this planner without a vision stack.
    from .perception import CubeObservation


# Oblique side-grasp profiles as (downward tilt deg, jaw-axis roll deg). A
# perfectly horizontal EEF commonly places X5 J5 near its +/-90 deg limit, so
# these vary entry tilt and jaw roll while keeping the lower finger clear of
# the tabletop.
SIDE_GRASP_PROFILES: tuple[tuple[int, int], ...] = (
    (25, 0),
    (45, 0),
    (62, 0),
    (45, 15),
    (45, -15),
)

# Side clearance is at least this far back from the cube centre before the
# horizontal grasp segment starts.
MINIMUM_SIDE_CLEARANCE_M = 0.055
MINIMUM_CONTACT_CLOSURE_M = 0.001
CUBE_DIAGONAL_CONTACT_MARGIN_M = 0.001


class CubePlanError(ValueError):
    """A planner rejection carrying a stable code for UI localization.

    The message stays English for logs and IK failure summaries. Callers that
    surface text to an operator map ``code`` to their own wording instead of
    duplicating the geometry check.
    """

    def __init__(self, code: str, message: str, **context: Any) -> None:
        super().__init__(message)
        self.code = code
        self.context = context


def maximum_plausible_contact_width_m(config: ExperimentConfig) -> float:
    diagonal_limit = (
        math.sqrt(2.0) * config.cube_edge_m
        + CUBE_DIAGONAL_CONTACT_MARGIN_M
    )
    closure_limit = (
        config.motion.precontact_width_m - MINIMUM_CONTACT_CLOSURE_M
    )
    return min(diagonal_limit, closure_limit)


@dataclass(frozen=True)
class Action:
    kind: str
    label: str
    target_xyz_m: tuple[float, float, float] | None = None
    orientation_xyzw: tuple[float, float, float, float] | None = None
    path_mode: str | None = None
    blend_radius_m: float = 0.0
    gripper_width_m: float | None = None
    wait_s: float | None = None


@dataclass(frozen=True)
class PreflightStep:
    label: str
    target_joint_position_rad: tuple[float, ...]
    maximum_joint_delta_rad: float
    minimum_tcp_z_m: float
    # Resolved TCP polyline for this segment, used by the web 3D preview.
    # Excluded from comparison because ndarray equality is not a bool.
    path_xyz_m: np.ndarray = field(
        default_factory=lambda: np.zeros((0, 3), dtype=np.float64),
        compare=False,
    )


@dataclass(frozen=True)
class GraspCandidate:
    """One orientation hypothesis to try during preflight."""

    name: str
    grasp_mode: str
    orientation_xyzw: np.ndarray
    approach_xy: tuple[float, float] | None = None


def top_down_orientation_xyzw(gripper_axis_yaw_rad: float) -> np.ndarray:
    """Point EEF +X down while aligning the jaw closing axis (+Y) in XY."""
    yaw = float(gripper_axis_yaw_rad)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    rotation = np.asarray(
        [
            [0.0, cosine, sine],
            [0.0, sine, -cosine],
            [-1.0, 0.0, 0.0],
        ],
        dtype=np.float64,
    )
    return quaternion_xyzw_from_matrix(rotation)


def side_grasp_orientation_xyzw(
    approach_angle_rad: float,
    downward_tilt_rad: float = 0.0,
    jaw_roll_rad: float = 0.0,
) -> np.ndarray:
    """Aim tool +X from above the side with an optional jaw-axis roll."""
    cosine = math.cos(float(approach_angle_rad))
    sine = math.sin(float(approach_angle_rad))
    tilt_cosine = math.cos(float(downward_tilt_rad))
    tilt_sine = math.sin(float(downward_tilt_rad))
    roll_cosine = math.cos(float(jaw_roll_rad))
    roll_sine = math.sin(float(jaw_roll_rad))
    tool_axis = np.asarray(
        [tilt_cosine * cosine, tilt_cosine * sine, -tilt_sine],
        dtype=np.float64,
    )
    level_jaw_axis = np.asarray([-sine, cosine, 0.0], dtype=np.float64)
    level_up_axis = np.asarray(
        [tilt_sine * cosine, tilt_sine * sine, tilt_cosine],
        dtype=np.float64,
    )
    jaw_axis = roll_cosine * level_jaw_axis + roll_sine * level_up_axis
    up_axis = -roll_sine * level_jaw_axis + roll_cosine * level_up_axis
    rotation = np.column_stack((tool_axis, jaw_axis, up_axis))
    return quaternion_xyzw_from_matrix(rotation)


def top_down_orientation_candidates(
    yaw_rad: float,
    current_pose_6d: np.ndarray,
) -> tuple[tuple[str, np.ndarray], ...]:
    """Return top-down gripper poses equivalent for a square target."""
    current_pose = np.asarray(current_pose_6d, dtype=np.float64)
    angles: list[tuple[str, float]] = []
    if current_pose.shape == (6,) and np.all(np.isfinite(current_pose)):
        current_rotation = rpy_to_matrix(current_pose[3:])
        current_jaw = current_rotation[:, 1]
        if float(np.linalg.norm(current_jaw[:2])) > 1e-6:
            current_yaw = math.atan2(float(current_jaw[1]), float(current_jaw[0]))
            angles.extend(
                (
                    ("current_wrist", current_yaw),
                    ("current_wrist_quarter_turn", current_yaw + math.pi / 2.0),
                )
            )
    angles.extend(
        (f"square_{quarter * 90}", float(yaw_rad) + quarter * math.pi / 2.0)
        for quarter in range(4)
    )
    candidates: list[tuple[str, np.ndarray]] = []
    seen: set[tuple[float, ...]] = set()
    for label, angle in angles:
        quaternion = top_down_orientation_xyzw(angle)
        key = tuple(np.round(quaternion, 6))
        inverse_key = tuple(np.round(-quaternion, 6))
        if key in seen or inverse_key in seen:
            continue
        seen.add(key)
        candidates.append((label, quaternion))
    return tuple(candidates)


def grasp_orientation_candidates(
    observation: CubeObservation,
    current_pose_6d: np.ndarray,
) -> tuple[GraspCandidate, ...]:
    """Enumerate top-down then oblique side grasps for one observed cube."""
    candidates: list[GraspCandidate] = [
        GraspCandidate(name=label, grasp_mode="top", orientation_xyzw=orientation)
        for label, orientation in top_down_orientation_candidates(
            observation.yaw_rad,
            current_pose_6d,
        )
    ]
    pose = np.asarray(current_pose_6d, dtype=np.float64)
    cube_xy = np.asarray(observation.base_xyz_m[:2], dtype=np.float64)
    toward_cube = cube_xy - pose[:2]
    if float(np.linalg.norm(toward_cube)) < 1e-6:
        base_angle = math.atan2(float(cube_xy[1]), float(cube_xy[0]))
    else:
        base_angle = math.atan2(float(toward_cube[1]), float(toward_cube[0]))
    for tilt_deg, jaw_roll_deg in SIDE_GRASP_PROFILES:
        for quarter in range(4):
            angle = base_angle + quarter * math.pi / 2.0
            candidates.append(
                GraspCandidate(
                    name=f"side_{quarter * 90}_tilt_{tilt_deg}_roll_{jaw_roll_deg}",
                    grasp_mode="side",
                    orientation_xyzw=side_grasp_orientation_xyzw(
                        angle,
                        math.radians(tilt_deg),
                        math.radians(jaw_roll_deg),
                    ),
                    approach_xy=(math.cos(angle), math.sin(angle)),
                )
            )
    return tuple(candidates)


def point_inside_table(config: ExperimentConfig, x_m: float, y_m: float) -> bool:
    """Report whether a base-frame XY point keeps a full cube on the tabletop."""
    table = config.table
    delta_x = x_m - table.center_x_m
    delta_y = y_m - table.center_y_m
    cosine = math.cos(table.yaw_rad)
    sine = math.sin(table.yaw_rad)
    local_x = cosine * delta_x + sine * delta_y
    local_y = -sine * delta_x + cosine * delta_y
    margin = config.cube_edge_m / 2.0
    return (
        abs(local_x) <= table.width_m / 2.0 - margin
        and abs(local_y) <= table.depth_m / 2.0 - margin
    )


def grasp_tcp_z_m(config: ExperimentConfig) -> float:
    """Return the commanded grasp TCP height, rejecting sub-tabletop targets."""
    grasp_z = (
        config.table.top_z_m
        + config.cube_edge_m / 2.0
        + config.motion.tcp_z_offset_m
    )
    if grasp_z <= config.table.top_z_m + 0.001:
        raise CubePlanError(
            "grasp_below_table",
            "tcp-z-offset would put the grasp TCP at or below the tabletop",
        )
    return grasp_z


def minimum_initial_tcp_z_m(config: ExperimentConfig) -> float:
    """Return the TCP height an automatic approach requires before it starts."""
    return config.table.top_z_m + config.motion.minimum_initial_clearance_m


def build_pick_actions(
    observation: CubeObservation,
    config: ExperimentConfig,
    orientation_xyzw: np.ndarray | None = None,
    *,
    grasp_mode: str = "top",
    approach_xy: tuple[float, float] | None = None,
) -> tuple[Action, ...]:
    """Build the approach, grasp, and lift chain for one cube.

    The sequence ends holding the cube above the table so callers can chain a
    place, a stack, or an operator decision onto it.
    """
    motion = config.motion
    pick_x, pick_y, _ = observation.base_xyz_m
    grasp_z = grasp_tcp_z_m(config)
    approach_z = grasp_z + motion.approach_height_m
    lift_z = grasp_z + motion.lift_height_m
    orientation = (
        top_down_orientation_xyzw(observation.yaw_rad)
        if orientation_xyzw is None
        else normalize_quaternion_xyzw(np.asarray(orientation_xyzw, dtype=np.float64))
    )
    initial_orientation = tuple(float(value) for value in orientation)
    grasp_tail = (
        Action(
            kind="gripper",
            label="fast close to precontact",
            gripper_width_m=motion.precontact_width_m,
        ),
        Action(kind="force_close", label="torque-limited grasp"),
        Action(kind="wait", label="grasp settle", wait_s=motion.settle_s),
        Action(
            kind="move",
            label="vertical lift",
            target_xyz_m=(pick_x, pick_y, lift_z),
            orientation_xyzw=initial_orientation,
            path_mode="linear",
        ),
    )
    if grasp_mode == "side":
        if approach_xy is None:
            raise CubePlanError(
                "side_missing_approach",
                "side grasp requires a horizontal approach direction",
            )
        approach = np.asarray(approach_xy, dtype=np.float64)
        approach /= max(float(np.linalg.norm(approach)), 1e-9)
        clearance = max(MINIMUM_SIDE_CLEARANCE_M, config.cube_edge_m * 2.0)
        pregrasp_x = pick_x - float(approach[0]) * clearance
        pregrasp_y = pick_y - float(approach[1]) * clearance
        if not point_inside_table(config, pregrasp_x, pregrasp_y):
            raise CubePlanError(
                "side_pregrasp_outside_table",
                "side pre-approach point is outside the configured tabletop",
            )
        transit_z = max(approach_z, lift_z)
        return (
            Action(
                kind="gripper",
                label="open before side approach",
                gripper_width_m=motion.pregrasp_width_m,
            ),
            Action(
                kind="move",
                label="side pre-approach above table",
                target_xyz_m=(pregrasp_x, pregrasp_y, transit_z),
                orientation_xyzw=initial_orientation,
                path_mode="smooth",
            ),
            Action(
                kind="move",
                label="side approach descent",
                target_xyz_m=(pregrasp_x, pregrasp_y, grasp_z),
                orientation_xyzw=initial_orientation,
                path_mode="linear",
            ),
            Action(
                kind="move",
                label="horizontal side grasp",
                target_xyz_m=(pick_x, pick_y, grasp_z),
                orientation_xyzw=initial_orientation,
                path_mode="linear",
            ),
            *grasp_tail,
        )
    if grasp_mode != "top":
        raise CubePlanError(
            "unsupported_grasp_mode",
            f"unsupported grasp mode: {grasp_mode}",
            grasp_mode=grasp_mode,
        )
    return (
        Action(
            kind="gripper",
            label="open before approach",
            gripper_width_m=motion.pregrasp_width_m,
        ),
        Action(
            kind="move",
            label="pick approach",
            target_xyz_m=(pick_x, pick_y, approach_z),
            orientation_xyzw=initial_orientation,
            path_mode="smooth",
        ),
        Action(
            kind="move",
            label="vertical grasp descent",
            target_xyz_m=(pick_x, pick_y, grasp_z),
            orientation_xyzw=initial_orientation,
            path_mode="linear",
        ),
        *grasp_tail,
    )


def build_action_plan(
    observation: CubeObservation,
    config: ExperimentConfig,
) -> tuple[Action, ...]:
    """Build a place-at-offset plan; the default offset is zero."""
    motion = config.motion
    pick_x, pick_y, _ = observation.base_xyz_m
    place_x = pick_x + motion.place_offset_x_m
    place_y = pick_y + motion.place_offset_y_m
    if not point_inside_table(config, place_x, place_y):
        raise CubePlanError(
            "place_outside_table",
            "place target is outside the configured tabletop",
        )
    grasp_z = grasp_tcp_z_m(config)
    lift_z = grasp_z + motion.lift_height_m
    placed_orientation = tuple(
        float(value)
        for value in top_down_orientation_xyzw(
            observation.yaw_rad + motion.rotation_rad
        )
    )
    return (
        *build_pick_actions(observation, config),
        Action(
            kind="move",
            label="rotate and transfer above place",
            target_xyz_m=(place_x, place_y, lift_z),
            orientation_xyzw=placed_orientation,
            path_mode="smooth",
        ),
        Action(
            kind="move",
            label="vertical place descent",
            target_xyz_m=(place_x, place_y, grasp_z),
            orientation_xyzw=placed_orientation,
            path_mode="linear",
        ),
        Action(
            kind="gripper",
            label="release cube",
            gripper_width_m=motion.release_width_m,
        ),
        Action(kind="wait", label="release settle", wait_s=motion.settle_s),
        Action(
            kind="move",
            label="vertical retreat",
            target_xyz_m=(place_x, place_y, lift_z),
            orientation_xyzw=placed_orientation,
            path_mode="linear",
        ),
        Action(kind="damping", label="enter damping"),
    )


def build_stack_actions(
    pick_actions: tuple[Action, ...],
    target: CubeObservation,
    config: ExperimentConfig,
) -> tuple[Action, ...]:
    """Build one pick-to-stack graph with motion runs split by tool barriers."""
    if not pick_actions or pick_actions[-1].label != "vertical lift":
        raise CubePlanError(
            "invalid_pick_prefix",
            "stack planning requires a pick sequence ending in vertical lift",
        )
    target_x, target_y, _ = target.base_xyz_m
    if not point_inside_table(config, target_x, target_y):
        raise CubePlanError(
            "stack_target_outside_table",
            "stack target is outside the configured tabletop",
        )
    source_move = next(
        (
            action
            for action in reversed(pick_actions)
            if action.kind == "move" and action.orientation_xyzw is not None
        ),
        None,
    )
    if source_move is None:
        raise CubePlanError(
            "invalid_pick_orientation",
            "pick sequence has no resolved grasp orientation",
        )
    orientation = source_move.orientation_xyzw
    place_z = (
        config.table.top_z_m
        + 1.5 * config.cube_edge_m
        + config.motion.tcp_z_offset_m
    )
    hover_z = place_z + config.motion.lift_height_m
    blend_radius = min(
        config.cube_edge_m / 4.0,
        config.motion.lift_height_m / 6.0,
    )
    pick_prefix = list(pick_actions)
    for index, action in enumerate(pick_prefix[:-1]):
        next_action = pick_prefix[index + 1]
        if action.path_mode == "linear" and next_action.path_mode == "linear":
            pick_prefix[index] = replace(action, blend_radius_m=blend_radius)
    return (
        *pick_prefix,
        Action(
            kind="move",
            label="continuous transfer above stack target",
            target_xyz_m=(float(target_x), float(target_y), hover_z),
            orientation_xyzw=orientation,
            path_mode="smooth",
        ),
        Action(
            kind="move",
            label="vertical stack descent",
            target_xyz_m=(float(target_x), float(target_y), place_z),
            orientation_xyzw=orientation,
            path_mode="linear",
        ),
        Action(
            kind="gripper",
            label="release stacked cube",
            gripper_width_m=config.motion.release_width_m,
        ),
        Action(kind="wait", label="release settle", wait_s=config.motion.settle_s),
        Action(
            kind="move",
            label="vertical retreat after stack",
            target_xyz_m=(float(target_x), float(target_y), hover_z),
            orientation_xyzw=orientation,
            path_mode="linear",
        ),
    )


def consecutive_move_runs(
    actions: tuple[Action, ...],
) -> tuple[tuple[int, tuple[Action, ...]], ...]:
    """Return arm-motion runs separated by a gripper or wait barrier."""
    runs: list[tuple[int, tuple[Action, ...]]] = []
    index = 0
    while index < len(actions):
        if actions[index].kind != "move":
            index += 1
            continue
        start = index
        while index < len(actions) and actions[index].kind == "move":
            index += 1
        runs.append((start, actions[start:index]))
    return tuple(runs)


def motion_plan_runs(
    actions: tuple[Action, ...],
) -> tuple[tuple[int, tuple[Action, ...]], ...]:
    """Return continuous key-pose runs separated by tool-action barriers."""
    runs = consecutive_move_runs(actions)
    for _, run in runs:
        for action in run:
            if action.path_mode not in {"smooth", "linear"}:
                raise RuntimeError(
                    f"move action {action.label!r} has an invalid path mode"
                )
    return runs


def validate_initial_pose(
    eef_pose_6d: np.ndarray,
    config: ExperimentConfig,
) -> None:
    pose = np.asarray(eef_pose_6d, dtype=np.float64)
    if pose.shape != (6,) or not np.all(np.isfinite(pose)):
        raise CubePlanError("invalid_tcp_pose", "controller returned an invalid TCP pose")
    required_z = minimum_initial_tcp_z_m(config)
    if float(pose[2]) < required_z:
        raise CubePlanError(
            "initial_tcp_too_low",
            "initial TCP is too low for automatic approach: "
            f"z={1000.0 * pose[2]:.1f} mm, require at least "
            f"{1000.0 * required_z:.1f} mm",
            clearance_m=config.motion.minimum_initial_clearance_m,
            required_z_m=required_z,
        )


def blended_linear_actions(
    actions: tuple[Action, ...],
    start_index: int,
) -> tuple[Action, ...]:
    """Return one explicitly blended run beginning at ``start_index``."""
    first = actions[start_index]
    if (
        first.kind != "move"
        or first.path_mode != "linear"
        or first.blend_radius_m <= 0.0
    ):
        return ()
    end_index = start_index
    while (
        end_index < len(actions)
        and actions[end_index].kind == "move"
        and actions[end_index].path_mode == "linear"
    ):
        end_index += 1
    run = actions[start_index:end_index]
    if len(run) < 2:
        raise RuntimeError(
            f"move action {first.label!r} defines a blend without a following "
            "linear move"
        )
    if run[-1].blend_radius_m > 0.0:
        raise RuntimeError("the final action in a blended run must have zero radius")
    return run


def cartesian_path_for_actions(
    actions: tuple[Action, ...],
) -> tuple[CartesianPathSegment, ...]:
    """Convert a validated linear action run into one controller path."""
    segments: list[CartesianPathSegment] = []
    for action in actions:
        if action.target_xyz_m is None or action.orientation_xyzw is None:
            raise RuntimeError(f"move action {action.label!r} is incomplete")
        segments.append(
            CartesianPathSegment(
                target=Pose(
                    np.asarray(action.target_xyz_m, dtype=np.float64),
                    np.asarray(action.orientation_xyzw, dtype=np.float64),
                ),
                blend_radius_m=action.blend_radius_m,
                name=action.label,
            )
        )
    return tuple(segments)


def preflight_action_plan(
    controller: Any,
    actions: tuple[Action, ...],
) -> tuple[PreflightStep, ...]:
    """Resolve every Cartesian segment in sequence without sending commands."""
    previews: list[PreflightStep] = []
    index = 0
    while index < len(actions):
        action = actions[index]
        if action.kind != "move":
            index += 1
            continue
        blended = blended_linear_actions(actions, index)
        if blended:
            plan = controller.plan_move_path(
                cartesian_path_for_actions(blended),
                unrestricted=True,
            )
            label = f"{blended[0].label} / {blended[-1].label}"
            index += len(blended)
        else:
            if action.target_xyz_m is None or action.orientation_xyzw is None:
                raise RuntimeError(f"move action {action.label!r} is incomplete")
            target = Pose(
                np.asarray(action.target_xyz_m, dtype=np.float64),
                np.asarray(action.orientation_xyzw, dtype=np.float64),
            )
            if action.path_mode == "smooth":
                plan = controller.plan_movej(target, allow_large_motion=True)
            elif action.path_mode == "linear":
                plan = controller.plan_movel(target, unrestricted=True)
            else:
                raise RuntimeError(
                    f"move action {action.label!r} has an invalid path mode"
                )
            label = action.label
            index += 1
        endpoint = plan.plan
        if endpoint is None:
            raise RuntimeError(f"move action {label!r} has no Cartesian preview")
        tcp_path = np.asarray(plan.path_xyz_m, dtype=np.float64)
        previews.append(
            PreflightStep(
                label=label,
                target_joint_position_rad=tuple(
                    float(value) for value in plan.target_joint_position
                ),
                maximum_joint_delta_rad=float(
                    plan.max_joint_delta_rad
                ),
                minimum_tcp_z_m=float(np.min(tcp_path[:, 2])),
                path_xyz_m=tcp_path.copy(),
            )
        )
    return tuple(previews)


def execute_action_plan(
    controller: Any,
    actions: tuple[Action, ...],
    motion: MotionConfig,
    *,
    sleep_fn: Callable[[float], None] = time.sleep,
    output: Callable[[str], None] = print,
) -> None:
    """Execute an already preflighted plan, checking each segment again online."""
    index = 0
    while index < len(actions):
        action = actions[index]
        output(f"[{index + 1:02d}/{len(actions):02d}] {action.label}")
        if action.kind == "move":
            blended = blended_linear_actions(actions, index)
            if blended:
                output(
                    f"    blended through {len(blended)} waypoints, "
                    f"speed={100 * motion.velocity_scale:.0f}%"
                )
                controller.move_path(
                    cartesian_path_for_actions(blended),
                    velocity_scale=motion.velocity_scale,
                    unrestricted=True,
                )
                index += len(blended)
                continue
            if action.target_xyz_m is None or action.orientation_xyzw is None:
                raise RuntimeError(f"move action {action.label!r} is incomplete")
            target = np.asarray(action.target_xyz_m, dtype=np.float64)
            quaternion = np.asarray(action.orientation_xyzw, dtype=np.float64)
            output(
                f"    target=({1000 * action.target_xyz_m[0]:.1f}, "
                f"{1000 * action.target_xyz_m[1]:.1f}, "
                f"{1000 * action.target_xyz_m[2]:.1f}) mm, "
                f"speed={100 * motion.velocity_scale:.0f}%"
            )
            pose = Pose(target, quaternion)
            if action.path_mode == "smooth":
                controller.movej(
                    pose,
                    velocity_scale=motion.velocity_scale,
                    allow_large_motion=True,
                )
            elif action.path_mode == "linear":
                controller.movel(
                    pose,
                    velocity_scale=motion.velocity_scale,
                    unrestricted=True,
                )
            else:
                raise RuntimeError(
                    f"move action {action.label!r} has an invalid path mode"
                )
        elif action.kind == "gripper":
            if action.gripper_width_m is None:
                raise RuntimeError(f"gripper action {action.label!r} has no width")
            state = controller.get_state()
            required = controller.minimum_gripper_duration(
                action.gripper_width_m - state.gripper_position
            )
            controller.move_gripper(
                action.gripper_width_m,
                max(motion.gripper_duration_s, required),
            )
        elif action.kind == "force_close":
            result = controller.force_close_gripper(
                motion.closing_torque_nm,
                minimum_width_m=motion.minimum_gripper_width_m,
                timeout_s=motion.gripper_timeout_s,
                contact_hold_s=motion.contact_hold_s,
                position_hold_after=True,
                torque_hold_after=True,
            )
            maximum_contact_width_m = maximum_plausible_contact_width_m(config)
            if (
                result.stop_reason != "contact"
                or result.final_width_m > maximum_contact_width_m
            ):
                raise RuntimeError(
                    "grasp contact was not confirmed; refusing to lift "
                    f"(stop_reason={result.stop_reason}, "
                    f"width={1000.0 * result.final_width_m:.1f} mm, "
                    f"maximum={1000.0 * maximum_contact_width_m:.1f} mm)"
                )
            output(
                f"    contact at {1000.0 * result.final_width_m:.1f} mm, "
                f"peak motor torque {result.max_abs_torque_nm:.3f} N.m"
            )
        elif action.kind == "wait":
            sleep_fn(float(action.wait_s or 0.0))
        elif action.kind == "damping":
            controller.damping()
        else:
            raise RuntimeError(f"unknown experiment action: {action.kind}")
        index += 1
