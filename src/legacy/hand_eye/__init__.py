"""Dual-installation hand-eye calibration for D405, D455, and ARX X5."""

from legacy.hand_eye.core.geometry import (
    BlockObservation,
    camera_to_base,
    deproject_pixel,
    intrinsics_from_realsense,
    observe_depth_sample,
    observe_pixel,
)
from legacy.hand_eye.core.validation import (
    ValidationThresholds,
    disable_result,
    enable_validated_result,
    validate_cached_result,
    validate_result,
)

from legacy.hand_eye.core.schema import CameraIntrinsics, EyeToHandSample, HandEyeSample
from legacy.hand_eye.core.solver import CalibrationQualityError
from legacy.hand_eye.eye_in_hand import solve_eye_in_hand
from legacy.hand_eye.eye_to_hand import solve_eye_to_hand

__all__ = [
    "BlockObservation",
    "CalibrationQualityError",
    "CameraIntrinsics",
    "EyeToHandSample",
    "HandEyeSample",
    "ValidationThresholds",
    "camera_to_base",
    "deproject_pixel",
    "disable_result",
    "enable_validated_result",
    "intrinsics_from_realsense",
    "observe_depth_sample",
    "observe_pixel",
    "solve_eye_in_hand",
    "solve_eye_to_hand",
    "validate_cached_result",
    "validate_result",
]
