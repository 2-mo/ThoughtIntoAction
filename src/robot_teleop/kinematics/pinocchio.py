"""Import-isolated Pinocchio adapter for offline FK, IK, and Jacobian checks."""

from __future__ import annotations

import importlib
import math
import threading
from typing import Any

from robot_teleop.core.frames import quaternion_from_matrix, quaternion_to_matrix
from robot_teleop.core.models import Pose, Vector3

from .models import (
    IKResult,
    IKSolverConfig,
    JacobianDiagnostics,
    KinematicsUnavailableError,
    RobotModel,
)


class PinocchioKinematics:
    """Bounded offline kinematics over one fixed-base URDF model.

    The adapter is deliberately outside ``core`` and must only be constructed by
    offline tools or a future backend composition root. It has no SDK, CAN, or
    robot connection behavior.
    """

    def __init__(self, model: RobotModel) -> None:
        pin, np = _load_dependencies()
        try:
            pin_model = pin.buildModelFromUrdf(str(model.urdf_path))
        except Exception as exc:
            raise KinematicsUnavailableError(
                f"Pinocchio could not load {model.urdf_path}: {exc}"
            ) from exc
        if int(pin_model.nq) != model.joint_limits.joint_count:
            raise KinematicsUnavailableError(
                "Pinocchio configuration dimension does not match model metadata"
            )
        if int(pin_model.nv) != model.joint_limits.joint_count:
            raise KinematicsUnavailableError(
                "Pinocchio velocity dimension does not match model metadata"
            )
        joint_names = tuple(str(name) for name in pin_model.names[1:])
        if joint_names != model.joint_names:
            raise KinematicsUnavailableError(
                "URDF joint order does not match configured model metadata"
            )
        frame_id = int(pin_model.getFrameId(model.end_effector_frame))
        if frame_id >= len(pin_model.frames):
            raise KinematicsUnavailableError(
                f"URDF has no end-effector frame {model.end_effector_frame!r}"
            )
        self._model = model
        self._pin = pin
        self._np = np
        self._pin_model = pin_model
        self._data = pin_model.createData()
        self._frame_id = frame_id
        self._lower = np.asarray(model.joint_limits.lower_rad, dtype=np.float64)
        self._upper = np.asarray(model.joint_limits.upper_rad, dtype=np.float64)
        self._lock = threading.RLock()

    @property
    def model(self) -> RobotModel:
        return self._model

    @property
    def backend_name(self) -> str:
        return f"pinocchio-{self._pin.__version__}"

    def forward_pose(self, joint_position: tuple[float, ...]) -> Pose:
        position = self._joint_array(joint_position)
        with self._lock:
            self._update_frames_locked(position)
            placement = self._data.oMf[self._frame_id]
            return _pose_from_placement(placement)

    def jacobian_diagnostics(
        self,
        joint_position: tuple[float, ...],
        *,
        orientation_scale_m: float = 0.10,
    ) -> JacobianDiagnostics:
        if not math.isfinite(orientation_scale_m) or orientation_scale_m <= 0.0:
            raise ValueError("orientation_scale_m must be positive and finite")
        position = self._joint_array(joint_position)
        with self._lock:
            jacobian = self._pin.computeFrameJacobian(
                self._pin_model,
                self._data,
                position,
                self._frame_id,
                self._pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
            ).copy()
        scaled = jacobian.copy()
        scaled[3:, :] *= orientation_scale_m
        singular_values = self._np.linalg.svd(scaled, compute_uv=False)
        singular = tuple(float(value) for value in singular_values)
        minimum = singular[-1]
        maximum = singular[0]
        condition = math.inf if minimum <= 1e-12 else maximum / minimum
        if not math.isfinite(condition):
            condition = float("inf")
        manipulability = float(self._np.prod(singular_values))
        if not math.isfinite(manipulability):
            raise KinematicsUnavailableError(
                "Pinocchio returned invalid Jacobian metrics"
            )
        return JacobianDiagnostics(
            singular_values=singular,
            minimum_singular_value=minimum,
            condition_number=condition,
            manipulability=manipulability,
            orientation_scale_m=orientation_scale_m,
        )

    def solve_ik(
        self,
        target_pose: Pose,
        seed_joint_position: tuple[float, ...],
        *,
        config: IKSolverConfig | None = None,
    ) -> IKResult:
        """Solve a bounded local IK problem from the supplied continuous seed.

        This is intentionally an offline preflight solver. It does not replace
        the robot-specific runtime IK path planned for Phase 5.
        """

        effective_config = IKSolverConfig() if config is None else config
        seed = self._joint_array(seed_joint_position)
        target = self._placement_from_pose(target_pose)
        with self._lock:
            position = seed.copy()
            for iteration in range(effective_config.max_iterations + 1):
                position_error, orientation_error, error = self._ik_error_locked(
                    position,
                    target,
                )
                if (
                    position_error <= effective_config.position_tolerance_m
                    and orientation_error <= effective_config.orientation_tolerance_rad
                ):
                    return IKResult(
                        reachable=True,
                        joint_position=tuple(float(value) for value in position),
                        iterations=iteration,
                        position_error_m=position_error,
                        orientation_error_rad=orientation_error,
                        reason=None,
                    )
                if iteration == effective_config.max_iterations:
                    break
                jacobian = self._pin.computeFrameJacobian(
                    self._pin_model,
                    self._data,
                    position,
                    self._frame_id,
                    self._pin.ReferenceFrame.LOCAL,
                )
                system = jacobian @ jacobian.T
                system += (
                    effective_config.damping * effective_config.damping
                ) * self._np.eye(
                    system.shape[0],
                    dtype=self._np.float64,
                )
                try:
                    step = jacobian.T @ self._np.linalg.solve(system, error)
                except Exception as exc:
                    return IKResult(
                        reachable=False,
                        joint_position=tuple(float(value) for value in position),
                        iterations=iteration,
                        position_error_m=position_error,
                        orientation_error_rad=orientation_error,
                        reason=f"damped Jacobian solve failed: {exc}",
                    )
                step_norm = float(self._np.linalg.norm(step))
                if not math.isfinite(step_norm):
                    return IKResult(
                        reachable=False,
                        joint_position=tuple(float(value) for value in position),
                        iterations=iteration,
                        position_error_m=position_error,
                        orientation_error_rad=orientation_error,
                        reason="damped Jacobian solve produced a non-finite step",
                    )
                if step_norm > effective_config.maximum_joint_step_rad:
                    step *= effective_config.maximum_joint_step_rad / step_norm
                candidate = self._pin.integrate(self._pin_model, position, step)
                candidate = self._np.clip(candidate, self._lower, self._upper)
                if float(self._np.linalg.norm(candidate - position)) <= 1e-12:
                    break
                position = candidate
            position_error, orientation_error, _ = self._ik_error_locked(
                position,
                target,
            )
            return IKResult(
                reachable=False,
                joint_position=tuple(float(value) for value in position),
                iterations=effective_config.max_iterations,
                position_error_m=position_error,
                orientation_error_rad=orientation_error,
                reason=(
                    "bounded IK did not converge within the configured iteration limit"
                ),
            )

    def _joint_array(self, joint_position: tuple[float, ...]) -> Any:
        position = self._model.joint_limits.validate_position(joint_position)
        return self._np.asarray(position, dtype=self._np.float64)

    def _placement_from_pose(self, pose: Pose) -> Any:
        rotation = self._np.asarray(
            quaternion_to_matrix(pose.orientation),
            dtype=self._np.float64,
        )
        translation = self._np.asarray(
            pose.position.as_tuple(),
            dtype=self._np.float64,
        )
        return self._pin.SE3(rotation, translation)

    def _update_frames_locked(self, position: Any) -> None:
        self._pin.forwardKinematics(self._pin_model, self._data, position)
        self._pin.updateFramePlacements(self._pin_model, self._data)

    def _ik_error_locked(self, position: Any, target: Any) -> tuple[float, float, Any]:
        self._update_frames_locked(position)
        current = self._data.oMf[self._frame_id]
        error = self._pin.log6(current.inverse() * target).vector
        position_error = float(self._np.linalg.norm(error[:3]))
        orientation_error = float(self._np.linalg.norm(error[3:]))
        return position_error, orientation_error, error


def _load_dependencies() -> tuple[Any, Any]:
    try:
        pin = importlib.import_module("pinocchio")
        np = importlib.import_module("numpy")
    except ImportError as exc:
        raise KinematicsUnavailableError(
            "Pinocchio is not installed; run uv sync --extra kinematics"
        ) from exc
    return pin, np


def _pose_from_placement(placement: Any) -> Pose:
    rotation = (
        (
            float(placement.rotation[0, 0]),
            float(placement.rotation[0, 1]),
            float(placement.rotation[0, 2]),
        ),
        (
            float(placement.rotation[1, 0]),
            float(placement.rotation[1, 1]),
            float(placement.rotation[1, 2]),
        ),
        (
            float(placement.rotation[2, 0]),
            float(placement.rotation[2, 1]),
            float(placement.rotation[2, 2]),
        ),
    )
    translation = tuple(float(value) for value in placement.translation)
    return Pose(
        position=Vector3(*translation),
        orientation=quaternion_from_matrix(rotation),
    )
