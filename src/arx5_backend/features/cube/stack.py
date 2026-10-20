from __future__ import annotations

from dataclasses import dataclass, replace
import math
import time
from typing import Any, Callable

import numpy as np

from arx5_control import (
    ArmSnapshot,
    CartesianWaypointTrajectoryPreview,
    ControlError,
    JointTrajectoryPreview,
    MotionSettleError,
    MotionResult,
    MotionPlan,
    Pose,
)
from experiments.cube_pick_rotate_place.config import ExperimentConfig
from experiments.cube_pick_rotate_place.planner import (
    Action,
    build_stack_actions,
    maximum_plausible_contact_width_m,
    motion_plan_runs,
)
from arx5_backend.features.gripper import HOME_GRIPPER_WIDTH_M


CUBE_STACK_PREVIEW_TIMEOUT_S = 600.0
PRECISION_MOTION_LABELS = frozenset(
    {
        "side approach descent",
        "horizontal side grasp",
        "vertical grasp descent",
        "vertical stack descent",
    }
)


@dataclass(frozen=True)
class StackPhase:
    start_index: int
    action_count: int
    plan: MotionPlan
    labels: tuple[str, ...]


@dataclass(frozen=True)
class CubeStackPlan:
    actions: tuple[Action, ...]
    phases: tuple[StackPhase, ...]
    return_plan: MotionPlan
    config: ExperimentConfig
    start_joint_position: np.ndarray
    start_pose: np.ndarray
    reference_joint_position: tuple[float, ...]
    created_at: float

    def payload(self) -> dict[str, Any]:
        path_xyz_m = [self.start_pose[:3]]
        for phase in self.phases:
            path_xyz_m.extend(phase.plan.path_xyz_m)
        path_xyz_m.extend(self.return_plan.path_xyz_m)
        return {
            "kind": "cube_stack_preview",
            "steps": [
                {
                    "label": action.label,
                    "kind": action.kind,
                    **(
                        {
                            "target_xyz_mm": [
                                1000.0 * float(value)
                                for value in action.target_xyz_m
                            ]
                        }
                        if action.target_xyz_m is not None
                        else {}
                    ),
                }
                for action in self.actions
            ],
            "phases": [
                {
                    "labels": list(phase.labels),
                    "duration_s": float(phase.plan.duration_s),
                }
                for phase in self.phases
            ]
            + [
                {
                    "labels": ["return task reference"],
                    "duration_s": float(self.return_plan.duration_s),
                }
            ],
            "path_xyz_mm": [
                [1000.0 * float(value) for value in point]
                for point in path_xyz_m
            ],
            "velocity_percent": 100.0 * self.config.motion.velocity_scale,
            "valid_for_s": CUBE_STACK_PREVIEW_TIMEOUT_S,
            "requires_confirmation": True,
        }


def _planned_motion_endpoint(
    start: ArmSnapshot,
    plan: MotionPlan,
) -> ArmSnapshot:
    backend = plan.backend_plan
    if isinstance(backend, CartesianWaypointTrajectoryPreview):
        solved_pose = backend.plan.solved_pose_6d
    elif (
        isinstance(backend, JointTrajectoryPreview)
        and backend.cartesian_plan is not None
    ):
        solved_pose = backend.cartesian_plan.solved_pose_6d
    else:
        raise ControlError("连续叠放包含无法解析末端位姿的轨迹。")
    return replace(
        start,
        joint_position=np.asarray(plan.target_joint_position, dtype=np.float64).copy(),
        joint_velocity=np.zeros_like(start.joint_velocity),
        joint_acceleration=np.zeros_like(start.joint_position),
        eef_pose_6d=np.asarray(solved_pose, dtype=np.float64).copy(),
    )


def _action_pose(action: Action) -> Pose:
    if action.target_xyz_m is None or action.orientation_xyzw is None:
        raise ControlError(f"连续叠放动作 {action.label!r} 缺少目标位姿。")
    return Pose(
        np.asarray(action.target_xyz_m, dtype=np.float64),
        np.asarray(action.orientation_xyzw, dtype=np.float64),
    )


def speed_partitioned_motion_runs(
    actions: tuple[Action, ...],
    *,
    transit_velocity_scale: float,
    precision_velocity_scale: float,
) -> tuple[tuple[int, tuple[Action, ...], float], ...]:
    """Split continuous motion at transit/precision speed boundaries."""
    runs: list[tuple[int, tuple[Action, ...], float]] = []
    for start_index, run in motion_plan_runs(actions):
        partition_start = start_index
        partition: list[Action] = []
        partition_scale: float | None = None
        for offset, action in enumerate(run):
            velocity_scale = (
                precision_velocity_scale
                if action.label in PRECISION_MOTION_LABELS
                else transit_velocity_scale
            )
            if partition and velocity_scale != partition_scale:
                assert partition_scale is not None
                runs.append(
                    (partition_start, tuple(partition), float(partition_scale))
                )
                partition_start = start_index + offset
                partition = []
            partition.append(action)
            partition_scale = velocity_scale
        if partition:
            assert partition_scale is not None
            runs.append((partition_start, tuple(partition), float(partition_scale)))
    return tuple(runs)


def plan_cube_stack(
    controller: Any,
    *,
    initial: ArmSnapshot,
    pick_actions: tuple[Action, ...],
    target: Any,
    config: ExperimentConfig,
    reference_joint_position: tuple[float, ...],
    transit_velocity_scale: float | None = None,
    precision_velocity_scale: float | None = None,
) -> CubeStackPlan:
    actions = build_stack_actions(pick_actions, target, config)
    transit_scale = (
        config.motion.velocity_scale
        if transit_velocity_scale is None
        else float(transit_velocity_scale)
    )
    precision_scale = (
        config.motion.velocity_scale
        if precision_velocity_scale is None
        else float(precision_velocity_scale)
    )
    virtual = initial
    phases: list[StackPhase] = []
    for start_index, run, velocity_scale in speed_partitioned_motion_runs(
        actions,
        transit_velocity_scale=transit_scale,
        precision_velocity_scale=precision_scale,
    ):
        plan = controller.plan_movej_waypoints(
            tuple(_action_pose(action) for action in run),
            velocity_scale=velocity_scale,
            allow_large_motion=True,
            start_snapshot=virtual,
        )
        phases.append(
            StackPhase(
                start_index=start_index,
                action_count=len(run),
                plan=plan,
                labels=tuple(action.label for action in run),
            )
        )
        virtual = _planned_motion_endpoint(virtual, plan)
    return_plan = controller.plan_movej(
        np.asarray(reference_joint_position, dtype=np.float64),
        velocity_scale=transit_scale,
        allow_large_motion=True,
        start_snapshot=virtual,
    )
    return CubeStackPlan(
        actions=actions,
        phases=tuple(phases),
        return_plan=return_plan,
        config=config,
        start_joint_position=initial.joint_position.copy(),
        start_pose=initial.eef_pose_6d.copy(),
        reference_joint_position=tuple(reference_joint_position),
        created_at=time.time(),
    )


def execute_cube_stack(
    controller: Any,
    preview: CubeStackPlan,
    *,
    before_return: Callable[[], None] | None = None,
) -> dict[str, Any]:
    phases = {phase.start_index: phase for phase in preview.phases}
    phase_elapsed_s: list[float] = []

    def execute_with_terminal_correction(plan: MotionPlan):
        accepted_residual_rad = math.radians(6.0)

        def accepted_result(state: ArmSnapshot) -> MotionResult:
            zeros = np.zeros_like(state.joint_position)
            return MotionResult(
                start_position=plan.start_joint_position,
                target_position=plan.target_joint_position,
                final_position=state.joint_position,
                commanded_duration_s=plan.duration_s,
                max_abs_velocity_rad_s=zeros,
                max_abs_torque_nm=zeros,
            )

        try:
            return controller.execute_motion_plan(
                plan,
                start_tolerance_rad=accepted_residual_rad,
            )
        except MotionSettleError as initial_error:
            state = controller.get_state()
            residual = float(
                np.max(np.abs(state.joint_position - plan.target_joint_position))
            )
            if residual > math.radians(6.0):
                raise
            print(
                "CUBE_TERMINAL_CORRECTION "
                f"residual_deg={math.degrees(residual):.2f}",
                flush=True,
            )
            try:
                return controller.movej(
                    plan.target_joint_position,
                    velocity_scale=0.40,
                    allow_large_motion=True,
                )
            except MotionSettleError:
                corrected = controller.get_state()
                corrected_residual = float(
                    np.max(
                        np.abs(
                            corrected.joint_position - plan.target_joint_position
                        )
                    )
                )
                if corrected_residual <= accepted_residual_rad:
                    print(
                        "CUBE_TERMINAL_ACCEPTED "
                        f"residual_deg={math.degrees(corrected_residual):.2f}",
                        flush=True,
                    )
                    return accepted_result(corrected)
                raise initial_error

    def gripper_duration(delta_m: float) -> float:
        required = controller.minimum_gripper_duration(delta_m)
        velocity_scale = max(0.2, float(preview.config.motion.velocity_scale))
        return max(0.5, required / velocity_scale)

    index = 0
    while index < len(preview.actions):
        action = preview.actions[index]
        if action.kind == "move":
            phase = phases.get(index)
            if phase is None:
                raise ControlError("连续叠放轨迹阶段索引无效。")
            started = time.monotonic()
            execute_with_terminal_correction(phase.plan)
            phase_elapsed_s.append(time.monotonic() - started)
            index += phase.action_count
            continue
        if action.kind == "gripper":
            state = controller.get_state()
            controller.move_gripper(
                action.gripper_width_m,
                gripper_duration(action.gripper_width_m - state.gripper_position),
            )
        elif action.kind == "force_close":
            result = controller.force_close_gripper(
                preview.config.motion.closing_torque_nm,
                minimum_width_m=preview.config.motion.minimum_gripper_width_m,
                timeout_s=preview.config.motion.gripper_timeout_s,
                contact_hold_s=preview.config.motion.contact_hold_s,
                position_hold_after=True,
                torque_hold_after=True,
            )
            maximum_width_m = maximum_plausible_contact_width_m(preview.config)
            plausible_contact = (
                result.stop_reason == "contact"
                and result.final_width_m <= maximum_width_m
            )
            print(
                "CUBE_GRIPPER "
                f"stop={result.stop_reason} "
                f"width_mm={1000.0 * result.final_width_m:.1f} "
                f"torque_nm={result.max_abs_torque_nm:.3f} "
                f"elapsed_s={result.elapsed_s:.2f} "
                f"plausible={int(plausible_contact)}",
                flush=True,
            )
            if not plausible_contact:
                raise RuntimeError(
                    "未确认有效夹取接触，拒绝继续连续叠放"
                    f"（{result.stop_reason}；末端开度 "
                    f"{1000.0 * result.final_width_m:.1f} mm）。"
                )
        elif action.kind == "wait":
            time.sleep(float(action.wait_s or 0.0))
        else:
            raise ControlError(f"连续叠放包含未知动作 {action.kind}。")
        index += 1

    if before_return is not None:
        before_return()

    return_started = time.monotonic()
    return_result = execute_with_terminal_correction(preview.return_plan)
    return_elapsed_s = time.monotonic() - return_started
    current_gripper = controller.get_state().gripper_position
    if abs(HOME_GRIPPER_WIDTH_M - current_gripper) >= 0.0001:
        controller.move_gripper(
            HOME_GRIPPER_WIDTH_M,
            gripper_duration(HOME_GRIPPER_WIDTH_M - current_gripper),
        )
    controller.hold_current()
    return {
        "phase_elapsed_s": phase_elapsed_s,
        "return_elapsed_s": return_elapsed_s,
        "final_joint_error_deg": math.degrees(
            float(np.max(np.abs(return_result.final_error_rad)))
        ),
        "mode": controller.mode.value,
    }
