"""Offline continuity checks over a sequence of Cartesian target poses."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import asdict, dataclass

from embodied.phone.core.models import Pose

from embodied.phone.kinematics.models import IKResult, IKSolverConfig, OfflineKinematics


@dataclass(frozen=True, slots=True)
class KinematicTarget:
    """A named target used only by an offline kinematic preflight."""

    label: str
    pose: Pose

    def __post_init__(self) -> None:
        label = self.label.strip()
        if not label:
            raise ValueError("kinematic target label must not be blank")
        object.__setattr__(self, "label", label)


@dataclass(frozen=True, slots=True)
class KinematicPathStep:
    label: str
    result: IKResult
    joint_step_rad: float
    branch_continuous: bool

    def __post_init__(self) -> None:
        if not math.isfinite(self.joint_step_rad) or self.joint_step_rad < 0.0:
            raise ValueError("joint_step_rad must be non-negative and finite")


@dataclass(frozen=True, slots=True)
class KinematicPathReport:
    backend_name: str
    reference_joint_position: tuple[float, ...]
    steps: tuple[KinematicPathStep, ...]
    maximum_branch_step_rad: float
    all_reachable: bool
    branch_continuous: bool

    def as_dict(self) -> dict[str, object]:
        return asdict(self)


def check_pose_path(
    kinematics: OfflineKinematics,
    targets: Sequence[KinematicTarget],
    *,
    reference_joint_position: tuple[float, ...],
    maximum_branch_step_rad: float,
    solver_config: IKSolverConfig | None = None,
) -> KinematicPathReport:
    """Solve targets from the prior solution and flag discontinuous branches."""

    if not targets:
        raise ValueError("at least one kinematic target is required")
    if not math.isfinite(maximum_branch_step_rad) or maximum_branch_step_rad <= 0.0:
        raise ValueError("maximum_branch_step_rad must be positive and finite")
    effective_solver_config = (
        IKSolverConfig() if solver_config is None else solver_config
    )
    current = kinematics.model.joint_limits.validate_position(reference_joint_position)
    steps: list[KinematicPathStep] = []
    all_reachable = True
    path_continuous = True
    for target in targets:
        result = kinematics.solve_ik(
            target.pose,
            current,
            config=effective_solver_config,
        )
        joint_step = _joint_distance(current, result.joint_position)
        continuous = result.reachable and joint_step <= maximum_branch_step_rad
        step = KinematicPathStep(
            label=target.label,
            result=result,
            joint_step_rad=joint_step,
            branch_continuous=continuous,
        )
        steps.append(step)
        all_reachable = all_reachable and result.reachable
        path_continuous = path_continuous and continuous
        if result.reachable:
            current = result.joint_position
    return KinematicPathReport(
        backend_name=kinematics.backend_name,
        reference_joint_position=reference_joint_position,
        steps=tuple(steps),
        maximum_branch_step_rad=maximum_branch_step_rad,
        all_reachable=all_reachable,
        branch_continuous=path_continuous,
    )


def _joint_distance(first: tuple[float, ...], second: tuple[float, ...]) -> float:
    if len(first) != len(second):
        raise ValueError("joint positions must have the same length")
    return math.sqrt(
        sum(
            (right - left) * (right - left)
            for left, right in zip(first, second, strict=True)
        )
    )
