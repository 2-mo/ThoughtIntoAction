"""Shared acquisition, geometry, storage, and ``A X = Z B`` calibration core."""

from hand_eye_calibration.core.schema import CameraIntrinsics, EyeToHandSample, HandEyeSample
from hand_eye_calibration.core.solver import (
    CalibrationQualityError,
    MotionDiversity,
    analyze_motion_diversity,
    analyze_transform_motion_diversity,
    evaluate_ax_zb_validation,
    solve_ax_zb,
    validate_motion_diversity,
)

__all__ = [
    "CalibrationQualityError",
    "CameraIntrinsics",
    "EyeToHandSample",
    "HandEyeSample",
    "MotionDiversity",
    "analyze_motion_diversity",
    "analyze_transform_motion_diversity",
    "evaluate_ax_zb_validation",
    "solve_ax_zb",
    "validate_motion_diversity",
]
