"""D455 fixed-camera coordinate adaptation for canonical ``A X = Z B``."""

from __future__ import annotations

from typing import Any, Iterable

import numpy as np

from embodied.calibration.hand_eye.core.schema import HandEyeSample
from embodied.calibration.hand_eye.core.solver import evaluate_ax_zb_validation, solve_ax_zb
from embodied.calibration.hand_eye.core.transforms import (
    invert_transform,
    quaternion_xyzw_from_matrix,
    validate_transform,
)

ROLE = "eye_to_hand"
EQUATION = "T_base_eef * T_eef_target = T_base_camera * T_camera_target"


def solve_eye_to_hand(
    samples: Iterable[HandEyeSample],
    *,
    quality: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Solve the fixed D455 transform and target mounting transform."""
    generic = solve_ax_zb(samples, quality=quality)
    T_eef_target = validate_transform(generic["T_x"], name="T_eef_target")
    T_base_camera = validate_transform(generic["T_z"], name="T_base_camera")
    methods = [
        {
            **{
                key: value
                for key, value in method.items()
                if key not in {"T_x", "T_z"}
            },
            "T_base_camera": method["T_z"],
            "T_eef_target": method["T_x"],
        }
        for method in generic["methods"]
    ]
    return {
        "schema_version": generic["schema_version"],
        "created_at": generic["created_at"],
        "calibration_type": ROLE,
        "coordinate_convention": {
            "T_base_eef": "maps ARX X5 EEF-frame points into the robot base frame",
            "T_camera_target": "maps target-frame points into the D455 RGB optical frame",
            "T_base_camera": "maps D455 RGB optical-frame points into the robot base frame",
            "T_eef_target": "maps target-frame points into the ARX X5 EEF frame",
            "camera_optical_axes": "+X right, +Y down, +Z forward",
            "equation": EQUATION,
        },
        "best_method": generic["best_method"],
        "sample_count": generic["sample_count"],
        "inlier_sample_indices": generic["inlier_sample_indices"],
        "excluded_sample_indices": generic["excluded_sample_indices"],
        "T_base_camera": T_base_camera.tolist(),
        "T_camera_base": invert_transform(T_base_camera).tolist(),
        "T_eef_target": T_eef_target.tolist(),
        "translation_xyz_m": T_base_camera[:3, 3].tolist(),
        "quaternion_xyzw": quaternion_xyzw_from_matrix(
            T_base_camera[:3, :3]
        ).tolist(),
        "residual": generic["residual"],
        "motion_diversity": generic["motion_diversity"],
        "leave_one_out_stability": generic["leave_one_out_z_stability"],
        "bootstrap_stability": generic["bootstrap_z_stability"],
        "methods": methods,
        "method_errors": generic["method_errors"],
        "quality_settings": generic["quality_settings"],
    }


def evaluate_eye_to_hand_validation(
    samples: Iterable[HandEyeSample],
    result: dict[str, Any],
) -> dict[str, float | int]:
    """Evaluate D455 held-out poses without refitting the transform."""
    return evaluate_ax_zb_validation(
        samples,
        T_x=np.asarray(result["T_eef_target"], dtype=np.float64),
        T_z=np.asarray(result["T_base_camera"], dtype=np.float64),
    )
