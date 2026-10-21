"""Serializable hand-eye sample and camera intrinsic records."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from legacy.hand_eye.core.transforms import validate_transform


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int
    height: int
    fps: int
    camera_matrix: np.ndarray
    distortion_coefficients: np.ndarray
    distortion_model: str
    serial: str | None = None

    def __post_init__(self) -> None:
        matrix = np.asarray(self.camera_matrix, dtype=np.float64)
        distortion = np.asarray(self.distortion_coefficients, dtype=np.float64)
        if self.width <= 0 or self.height <= 0 or self.fps <= 0:
            raise ValueError("camera dimensions and fps must be positive")
        if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
            raise ValueError("camera_matrix must be a finite 3x3 matrix")
        if distortion.ndim != 1 or distortion.size not in (4, 5, 8, 12, 14):
            raise ValueError("distortion_coefficients has an unsupported shape")
        if not np.all(np.isfinite(distortion)):
            raise ValueError("distortion_coefficients contains invalid values")
        object.__setattr__(self, "camera_matrix", matrix.copy())
        object.__setattr__(self, "distortion_coefficients", distortion.copy())

    def to_dict(self) -> dict[str, Any]:
        return {
            "width": self.width,
            "height": self.height,
            "fps": self.fps,
            "camera_matrix": self.camera_matrix.tolist(),
            "distortion_coefficients": self.distortion_coefficients.tolist(),
            "distortion_model": self.distortion_model,
            "serial": self.serial,
        }

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "CameraIntrinsics":
        return cls(
            width=int(value["width"]),
            height=int(value["height"]),
            fps=int(value["fps"]),
            camera_matrix=np.asarray(value["camera_matrix"], dtype=np.float64),
            distortion_coefficients=np.asarray(
                value["distortion_coefficients"], dtype=np.float64
            ),
            distortion_model=str(value["distortion_model"]),
            serial=None if value.get("serial") is None else str(value["serial"]),
        )


@dataclass(frozen=True)
class EyeToHandSample:
    index: int
    timestamp: float
    T_base_eef: np.ndarray
    T_camera_target: np.ndarray
    joint_position_rad: np.ndarray
    reprojection_error_px: float
    translation_std_m: float
    rotation_std_deg: float
    valid_frames: int
    total_frames: int
    image: str | None = None
    annotated_image: str | None = None
    burst_count: int = 1
    burst_translation_std_m: float = 0.0
    burst_rotation_std_deg: float = 0.0

    def __post_init__(self) -> None:
        joints = np.asarray(self.joint_position_rad, dtype=np.float64)
        finite_scalars = (
            self.timestamp,
            self.reprojection_error_px,
            self.translation_std_m,
            self.rotation_std_deg,
            self.burst_translation_std_m,
            self.burst_rotation_std_deg,
        )
        if self.index < 1:
            raise ValueError("sample index must be positive")
        if joints.shape != (6,) or not np.all(np.isfinite(joints)):
            raise ValueError("joint_position_rad must contain six finite values")
        if not all(np.isfinite(value) for value in finite_scalars):
            raise ValueError("sample metrics contain invalid values")
        if self.valid_frames <= 0 or self.total_frames < self.valid_frames:
            raise ValueError("sample frame counts are invalid")
        if self.burst_count < 1:
            raise ValueError("burst_count must be positive")
        object.__setattr__(
            self, "T_base_eef", validate_transform(self.T_base_eef, name="T_base_eef")
        )
        object.__setattr__(
            self,
            "T_camera_target",
            validate_transform(self.T_camera_target, name="T_camera_target"),
        )
        object.__setattr__(self, "joint_position_rad", joints.copy())

    def to_dict(self) -> dict[str, Any]:
        return {
            "index": self.index,
            "timestamp": self.timestamp,
            "T_base_eef": self.T_base_eef.tolist(),
            "T_camera_target": self.T_camera_target.tolist(),
            "joint_position_rad": self.joint_position_rad.tolist(),
            "reprojection_error_px": self.reprojection_error_px,
            "translation_std_m": self.translation_std_m,
            "rotation_std_deg": self.rotation_std_deg,
            "valid_frames": self.valid_frames,
            "total_frames": self.total_frames,
            "image": self.image,
            "annotated_image": self.annotated_image,
            "burst_count": self.burst_count,
            "burst_translation_std_m": self.burst_translation_std_m,
            "burst_rotation_std_deg": self.burst_rotation_std_deg,
        }

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "EyeToHandSample":
        return cls(
            index=int(value["index"]),
            timestamp=float(value["timestamp"]),
            T_base_eef=np.asarray(value["T_base_eef"], dtype=np.float64),
            T_camera_target=np.asarray(value["T_camera_target"], dtype=np.float64),
            joint_position_rad=np.asarray(value["joint_position_rad"], dtype=np.float64),
            reprojection_error_px=float(value["reprojection_error_px"]),
            translation_std_m=float(value["translation_std_m"]),
            rotation_std_deg=float(value["rotation_std_deg"]),
            valid_frames=int(value["valid_frames"]),
            total_frames=int(value["total_frames"]),
            image=value.get("image"),
            annotated_image=value.get("annotated_image"),
            burst_count=int(value.get("burst_count", 1)),
            burst_translation_std_m=float(value.get("burst_translation_std_m", 0.0)),
            burst_rotation_std_deg=float(value.get("burst_rotation_std_deg", 0.0)),
        )


# New shared code should use the installation-neutral name.  Keep the original
# class name as the concrete definition so existing imports and serialized data
# remain fully compatible.
HandEyeSample = EyeToHandSample
