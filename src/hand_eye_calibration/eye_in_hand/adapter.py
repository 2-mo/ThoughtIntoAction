"""D405 EEF-camera coordinate adaptation for canonical ``A X = Z B``."""

from __future__ import annotations

from typing import Any, Iterable

import numpy as np

from hand_eye_calibration.core.schema import HandEyeSample
from hand_eye_calibration.core.solver import (
    bootstrap_transform_stability,
    evaluate_ax_zb_validation,
    leave_one_out_x_stability,
    solve_ax_zb,
)
from hand_eye_calibration.core.transforms import (
    invert_transform,
    quaternion_xyzw_from_matrix,
    validate_transform,
)

ROLE = "eye_in_hand"
EQUATION = (
    "T_base_eef * T_eef_camera = "
    "T_base_target * inv(T_camera_target)"
)


def canonical_samples(
    samples: Iterable[HandEyeSample],
) -> list[HandEyeSample]:
    """Map fixed-table observations into the canonical ``B`` direction."""
    return [
        HandEyeSample(
            **{
                **sample.to_dict(),
                "T_camera_target": invert_transform(sample.T_camera_target),
            }
        )
        for sample in samples
    ]


def solve_eye_in_hand(
    samples: Iterable[HandEyeSample],
    *,
    quality: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Solve the D405 EEF mounting and fixed table-target transforms."""
    transformed = canonical_samples(samples)
    generic = solve_ax_zb(transformed, quality=quality)
    T_eef_camera = validate_transform(generic["T_x"], name="T_eef_camera")
    T_base_target = validate_transform(generic["T_z"], name="T_base_target")
    settings = generic["quality_settings"]
    stability = leave_one_out_x_stability(
        transformed,
        method=str(generic["best_method"]),
        inlier_sample_indices=tuple(generic["inlier_sample_indices"]),
        reference=T_eef_camera,
        quality=settings,
    )
    bootstrap = bootstrap_transform_stability(
        transformed,
        method=str(generic["best_method"]),
        inlier_sample_indices=tuple(generic["inlier_sample_indices"]),
        reference=T_eef_camera,
        transform_component="x",
        quality=settings,
    )
    methods = [
        {
            **{
                key: value
                for key, value in method.items()
                if key not in {"T_x", "T_z"}
            },
            "T_eef_camera": method["T_x"],
            "T_base_target": method["T_z"],
        }
        for method in generic["methods"]
    ]
    return {
        "schema_version": generic["schema_version"],
        "created_at": generic["created_at"],
        "best_method": generic["best_method"],
        "sample_count": generic["sample_count"],
        "inlier_sample_indices": generic["inlier_sample_indices"],
        "excluded_sample_indices": generic["excluded_sample_indices"],
        "residual": generic["residual"],
        "motion_diversity": generic["motion_diversity"],
        "method_errors": generic["method_errors"],
        "quality_settings": settings,
        "calibration_type": ROLE,
        "coordinate_convention": {
            "T_base_eef": "maps ARX X5 EEF-frame points into the robot base frame",
            "T_camera_target": "maps target-frame points into the D405 RGB optical frame",
            "T_eef_camera": "maps D405 RGB optical-frame points into the ARX X5 EEF frame",
            "T_base_target": "maps fixed target-frame points into the robot base frame",
            "camera_optical_axes": "+X right, +Y down, +Z forward",
            "equation": EQUATION,
        },
        "T_eef_camera": T_eef_camera.tolist(),
        "T_camera_eef": invert_transform(T_eef_camera).tolist(),
        "T_base_target": T_base_target.tolist(),
        "T_target_base": invert_transform(T_base_target).tolist(),
        "translation_xyz_m": T_eef_camera[:3, 3].tolist(),
        "quaternion_xyzw": quaternion_xyzw_from_matrix(
            T_eef_camera[:3, :3]
        ).tolist(),
        "leave_one_out_stability": stability,
        "target_leave_one_out_stability": generic["leave_one_out_z_stability"],
        "bootstrap_stability": bootstrap,
        "target_bootstrap_stability": generic["bootstrap_z_stability"],
        "methods": methods,
    }


def evaluate_eye_in_hand_validation(
    samples: Iterable[HandEyeSample],
    result: dict[str, Any],
) -> dict[str, float | int]:
    """Evaluate D405 held-out poses without refitting the transform."""
    return evaluate_ax_zb_validation(
        canonical_samples(samples),
        T_x=np.asarray(result["T_eef_camera"], dtype=np.float64),
        T_z=np.asarray(result["T_base_target"], dtype=np.float64),
    )
