"""Optional Pinocchio kinematics backend.

The ARX SDK remains the source of inverse kinematics. Pinocchio supplies
analytic FK, frame placements, and geometric Jacobians for safety checks.
"""

from __future__ import annotations

import importlib
import threading

import numpy as np

from arx5_control.motion.jacobian import JacobianDiagnostics, analyze_jacobian


class PinocchioUnavailableError(RuntimeError):
    """Raised when the optional Pinocchio backend cannot be constructed."""


class PinocchioKinematics:
    """Thread-safe fixed-base Pinocchio model for one robot URDF."""

    def __init__(self, urdf_path: str, joint_dof: int, eef_link_name: str) -> None:
        try:
            pin = importlib.import_module("pinocchio")
        except ImportError as exc:  # pragma: no cover - dependency-free fallback
            raise PinocchioUnavailableError(
                "Pinocchio is not installed; run uv sync --only-group control"
            ) from exc
        try:
            model = pin.buildModelFromUrdf(str(urdf_path))
        except Exception as exc:
            raise PinocchioUnavailableError(
                f"Pinocchio could not load the robot URDF: {exc}"
            ) from exc
        if int(model.nq) != int(joint_dof) or int(model.nv) != int(joint_dof):
            raise PinocchioUnavailableError(
                "Pinocchio model dimensions do not match the ARX robot config"
            )
        frame_id = int(model.getFrameId(str(eef_link_name)))
        if frame_id >= len(model.frames):
            raise PinocchioUnavailableError(
                f"Pinocchio model has no frame named {eef_link_name}"
            )
        self._pin = pin
        self._model = model
        self._data = model.createData()
        self._eef_frame_id = frame_id
        self._joint_dof = int(joint_dof)
        self._lock = threading.RLock()

    @classmethod
    def try_create(
        cls,
        urdf_path: str,
        joint_dof: int,
        eef_link_name: str,
    ) -> PinocchioKinematics | None:
        """Return a backend when dependencies and URDF are usable."""
        try:
            return cls(urdf_path, joint_dof, eef_link_name)
        except PinocchioUnavailableError:
            return None

    @property
    def backend_name(self) -> str:
        return f"pinocchio-{self._pin.__version__}"

    @property
    def frame_names(self) -> tuple[str, ...]:
        return tuple(str(frame.name) for frame in self._model.frames)

    @property
    def joint_dof(self) -> int:
        return self._joint_dof

    def has_frame(self, name: str) -> bool:
        frame_id = int(self._model.getFrameId(str(name)))
        return frame_id < len(self._model.frames)

    def _joint_vector(self, joint_position: np.ndarray) -> np.ndarray:
        position = np.asarray(joint_position, dtype=np.float64).copy()
        if position.shape != (self._joint_dof,) or not np.all(np.isfinite(position)):
            raise ValueError("joint position must be a finite robot-sized vector")
        return position

    def _update_frames_locked(self, position: np.ndarray) -> None:
        self._pin.forwardKinematics(self._model, self._data, position)
        self._pin.updateFramePlacements(self._model, self._data)

    def forward_kinematics(self, joint_position: np.ndarray) -> np.ndarray:
        position = self._joint_vector(joint_position)
        with self._lock:
            self._update_frames_locked(position)
            placement = self._data.oMf[self._eef_frame_id]
            return np.concatenate(
                (
                    np.asarray(placement.translation, dtype=np.float64),
                    np.asarray(
                        self._pin.rpy.matrixToRpy(placement.rotation),
                        dtype=np.float64,
                    ),
                )
            )

    def forward_kinematics_path(self, joint_positions: np.ndarray) -> np.ndarray:
        positions = np.asarray(joint_positions, dtype=np.float64)
        if (
            positions.ndim != 2
            or positions.shape[0] < 1
            or positions.shape[1] != self._joint_dof
            or not np.all(np.isfinite(positions))
        ):
            raise ValueError("joint path must have shape (N, joint_dof)")
        return np.asarray(
            [self.forward_kinematics(position) for position in positions],
            dtype=np.float64,
        )

    def jacobian_diagnostics(
        self,
        joint_position: np.ndarray,
        *,
        orientation_scale_m: float = 0.10,
    ) -> JacobianDiagnostics:
        position = self._joint_vector(joint_position)
        with self._lock:
            jacobian = self._pin.computeFrameJacobian(
                self._model,
                self._data,
                position,
                self._eef_frame_id,
                self._pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
            )
        return analyze_jacobian(
            np.asarray(jacobian, dtype=np.float64),
            orientation_scale_m=orientation_scale_m,
        )

    def frame_positions(
        self,
        joint_position: np.ndarray,
        frame_names: tuple[str, ...],
    ) -> dict[str, np.ndarray]:
        position = self._joint_vector(joint_position)
        frame_ids: dict[str, int] = {}
        for name in frame_names:
            frame_id = int(self._model.getFrameId(str(name)))
            if frame_id >= len(self._model.frames):
                raise ValueError(f"Pinocchio model has no frame named {name}")
            frame_ids[name] = frame_id
        with self._lock:
            self._update_frames_locked(position)
            return {
                name: np.asarray(
                    self._data.oMf[frame_id].translation,
                    dtype=np.float64,
                ).copy()
                for name, frame_id in frame_ids.items()
            }


__all__ = ["PinocchioKinematics", "PinocchioUnavailableError"]
