"""Metric camera observations for tabletop grasping.

All points use the D455 RGB optical convention: +X right, +Y down, +Z
forward.  Depth must be an aligned metric depth value, never a colorized
depth-video pixel.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from .projection import normalize_image_points
from .schema import CameraIntrinsics
from .transforms import validate_transform


@dataclass(frozen=True)
class BlockObservation:
    pixel_xy: tuple[float, float]
    depth_m: float
    camera_xyz_m: tuple[float, float, float]
    base_xyz_m: tuple[float, float, float]
    confidence: float
    timestamp: float | None = None
    calibration_profile: str = "eye-to-hand"

    def to_dict(self) -> dict[str, Any]:
        return {
            "pixel_xy": list(self.pixel_xy),
            "depth_m": self.depth_m,
            "camera_xyz_m": list(self.camera_xyz_m),
            "base_xyz_m": list(self.base_xyz_m),
            "confidence": self.confidence,
            "timestamp": self.timestamp,
            "calibration_profile": self.calibration_profile,
        }


def deproject_pixel(
    pixel_xy: tuple[float, float] | list[float] | np.ndarray,
    depth_m: float,
    intrinsics: CameraIntrinsics,
) -> np.ndarray:
    """Deproject one aligned RGB pixel into D455 optical-frame metres."""
    depth = float(depth_m)
    if not np.isfinite(depth) or depth <= 0.0:
        raise ValueError("depth_m must be positive and finite")
    pixels = np.asarray(pixel_xy, dtype=np.float64).reshape(1, 2)
    if not np.all(np.isfinite(pixels)):
        raise ValueError("pixel_xy must be finite")
    normalized = normalize_image_points(pixels, intrinsics)[0]
    return np.asarray([normalized[0] * depth, normalized[1] * depth, depth])


def camera_to_base(point_camera_m: np.ndarray, T_base_camera: np.ndarray) -> np.ndarray:
    """Transform one D455 optical-frame point into the robot base frame."""
    point = np.asarray(point_camera_m, dtype=np.float64).reshape(3)
    if not np.all(np.isfinite(point)):
        raise ValueError("camera point must be finite")
    transform = validate_transform(T_base_camera, name="T_base_camera")
    return transform[:3, :3] @ point + transform[:3, 3]


def observe_pixel(
    pixel_xy: tuple[float, float] | list[float] | np.ndarray,
    depth_m: float,
    intrinsics: CameraIntrinsics,
    T_base_camera: np.ndarray,
    *,
    confidence: float = 1.0,
    timestamp: float | None = None,
    calibration_profile: str = "eye-to-hand",
) -> BlockObservation:
    """Build the structured observation consumed by cube targeting."""
    camera_point = deproject_pixel(pixel_xy, depth_m, intrinsics)
    base_point = camera_to_base(camera_point, T_base_camera)
    confidence_value = float(confidence)
    if not np.isfinite(confidence_value) or not 0.0 <= confidence_value <= 1.0:
        raise ValueError("confidence must be between 0 and 1")
    pixel = np.asarray(pixel_xy, dtype=np.float64).reshape(2)
    return BlockObservation(
        pixel_xy=(float(pixel[0]), float(pixel[1])),
        depth_m=float(depth_m),
        camera_xyz_m=tuple(float(value) for value in camera_point),
        base_xyz_m=tuple(float(value) for value in base_point),
        confidence=confidence_value,
        timestamp=None if timestamp is None else float(timestamp),
        calibration_profile=str(calibration_profile),
    )


def intrinsics_from_realsense(value: dict[str, Any], *, fps: int = 30) -> CameraIntrinsics:
    """Convert the compact WebRTC depth-channel intrinsics payload."""
    required = ("width", "height", "fx", "fy", "ppx", "ppy")
    if not all(name in value for name in required):
        raise ValueError("depth sample is missing RGB intrinsics")
    return CameraIntrinsics(
        width=int(value["width"]),
        height=int(value["height"]),
        fps=int(fps),
        camera_matrix=np.asarray(
            [
                [float(value["fx"]), 0.0, float(value["ppx"])],
                [0.0, float(value["fy"]), float(value["ppy"])],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        ),
        distortion_coefficients=np.asarray(value.get("coeffs", [0.0] * 5), dtype=np.float64),
        distortion_model=str(value.get("model", "none")),
    )


def observe_depth_sample(
    sample: dict[str, Any],
    T_base_camera: np.ndarray,
    *,
    confidence: float = 1.0,
    timestamp: float | None = None,
    calibration_profile: str = "eye-to-hand",
) -> BlockObservation:
    """Turn one WebRTC ``depth_sample`` response into a base-frame observation."""
    if sample.get("ok") is not True:
        raise ValueError(sample.get("error", "depth sample has no valid depth"))
    intrinsics = intrinsics_from_realsense(sample["intrinsics"])
    return observe_pixel(
        (sample["x"], sample["y"]),
        sample["depth_m"],
        intrinsics,
        T_base_camera,
        confidence=confidence,
        timestamp=timestamp,
        calibration_profile=calibration_profile,
    )
