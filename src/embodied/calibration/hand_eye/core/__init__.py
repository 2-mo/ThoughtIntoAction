"""Shared acquisition, geometry, storage, and ``A X = Z B`` calibration core."""

from embodied.calibration.hand_eye.core.schema import CameraIntrinsics, EyeToHandSample, HandEyeSample
from embodied.calibration.hand_eye.core.solver import (
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
