"""Installation-agnostic OpenCV solver for the canonical ``A X = Z B`` model."""

from __future__ import annotations

from dataclasses import dataclass
import datetime as dt
import math
from typing import Any, Iterable

import numpy as np

from legacy.hand_eye.core.schema import HandEyeSample
from legacy.hand_eye.core.transforms import (
    make_transform,
    rotation_distance_rad,
    rotation_vector_from_matrix,
    transform_error,
    validate_transform,
)


class CalibrationQualityError(RuntimeError):
    """Raised when samples cannot constrain a reliable hand-eye result."""


@dataclass(frozen=True)
class MotionDiversity:
    sample_count: int
    translation_span_m: float
    rotation_span_deg: float
    rotation_axis_rank: int
    rotation_singular_values: tuple[float, float, float]

    def to_dict(self) -> dict[str, Any]:
        return {
            "sample_count": self.sample_count,
            "translation_span_m": self.translation_span_m,
            "rotation_span_deg": self.rotation_span_deg,
            "rotation_axis_rank": self.rotation_axis_rank,
            "rotation_singular_values": list(self.rotation_singular_values),
        }


@dataclass(frozen=True)
class _MethodSolution:
    method: str
    T_x: np.ndarray
    T_z: np.ndarray
    inlier_indices: tuple[int, ...]
    excluded_indices: tuple[int, ...]
    metrics: dict[str, float]

    @property
    def score(self) -> float:
        return self.metrics["score"]


DEFAULT_QUALITY = {
    "min_samples": 10,
    "min_translation_span_m": 0.04,
    "min_rotation_span_deg": 20.0,
    "min_rotation_axis_rank": 2,
    "max_outlier_fraction": 0.20,
    # Empirical uncertainty from resampling the accepted inlier set.  This is
    # deliberately a bootstrap estimate rather than an assumed Gaussian
    # distribution: hand-eye residuals often have non-Gaussian tails.
    "bootstrap_iterations": 240,
    "bootstrap_seed": 20260729,
    "bootstrap_min_success_ratio": 0.90,
}


def analyze_transform_motion_diversity(
    transforms: Iterable[np.ndarray],
) -> MotionDiversity:
    """Measure calibration information content from base-to-EEF transforms."""
    values = [validate_transform(transform, name="T_base_eef") for transform in transforms]
    if not values:
        return MotionDiversity(0, 0.0, 0.0, 0, (0.0, 0.0, 0.0))
    translations = np.asarray([transform[:3, 3] for transform in values])
    translation_span = _max_pairwise_distance(translations)
    rotation_span = 0.0
    for index, first in enumerate(values):
        for second in values[index + 1 :]:
            rotation_span = max(
                rotation_span,
                rotation_distance_rad(first[:3, :3], second[:3, :3]),
            )
    reference = values[0][:3, :3]
    rotation_vectors = np.asarray(
        [
            rotation_vector_from_matrix(reference.T @ transform[:3, :3])
            for transform in values[1:]
        ],
        dtype=np.float64,
    )
    if rotation_vectors.size == 0:
        singular_values = np.zeros(3)
    else:
        singular_values = np.linalg.svd(rotation_vectors, compute_uv=False)
        singular_values = np.pad(singular_values, (0, 3 - singular_values.size))[:3]
    axis_rank = int(np.count_nonzero(singular_values > math.radians(4.0)))
    return MotionDiversity(
        sample_count=len(values),
        translation_span_m=float(translation_span),
        rotation_span_deg=math.degrees(rotation_span),
        rotation_axis_rank=axis_rank,
        rotation_singular_values=tuple(float(value) for value in singular_values),
    )


def analyze_motion_diversity(samples: Iterable[HandEyeSample]) -> MotionDiversity:
    """Measure calibration information content from recorded samples."""
    return analyze_transform_motion_diversity(
        sample.T_base_eef for sample in samples
    )


def validate_motion_diversity(
    diversity: MotionDiversity,
    quality: dict[str, Any],
) -> None:
    failures: list[str] = []
    minimum_samples = int(quality["min_samples"])
    minimum_translation = float(quality["min_translation_span_m"])
    minimum_rotation = float(quality["min_rotation_span_deg"])
    minimum_rank = int(quality["min_rotation_axis_rank"])
    if diversity.sample_count < minimum_samples:
        failures.append(
            f"need at least {minimum_samples} samples, got {diversity.sample_count}"
        )
    if diversity.translation_span_m < minimum_translation:
        failures.append(
            f"EEF translation span {diversity.translation_span_m:.3f} m is below "
            f"{minimum_translation:.3f} m"
        )
    if diversity.rotation_span_deg < minimum_rotation:
        failures.append(
            f"EEF rotation span {diversity.rotation_span_deg:.1f} deg is below "
            f"{minimum_rotation:.1f} deg"
        )
    if diversity.rotation_axis_rank < minimum_rank:
        failures.append(
            f"rotation-axis rank {diversity.rotation_axis_rank} is below {minimum_rank}"
        )
    if failures:
        raise CalibrationQualityError("insufficient motion diversity: " + "; ".join(failures))


def solve_ax_zb(
    samples: Iterable[HandEyeSample],
    *,
    quality: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Solve canonical robot-world/hand-eye observations.

    Each sample supplies ``A = T_base_eef`` and ``B = T_camera_target``::

        A @ X = Z @ B

    The installation-specific packages decide what ``X`` and ``Z`` mean and
    adapt these neutral fields to their public coordinate names.
    """
    values = list(samples)
    settings = {**DEFAULT_QUALITY, **(quality or {})}
    diversity = analyze_motion_diversity(values)
    validate_motion_diversity(diversity, settings)

    solutions: list[_MethodSolution] = []
    errors: dict[str, str] = {}
    for method in ("shah", "li"):
        try:
            solutions.append(_solve_robust(values, method, settings))
        except (CalibrationQualityError, RuntimeError, ValueError) as exc:
            errors[method] = str(exc)
    if not solutions:
        raise CalibrationQualityError(f"all OpenCV methods failed: {errors}")
    best = min(solutions, key=lambda solution: solution.score)
    stability = _leave_one_out_z_stability(values, best, settings)
    bootstrap = bootstrap_transform_stability(
        values,
        method=best.method,
        inlier_sample_indices=best.inlier_indices,
        reference=best.T_z,
        transform_component="z",
        quality=settings,
    )
    return {
        "schema_version": 1,
        "created_at": dt.datetime.now().astimezone().isoformat(timespec="seconds"),
        "best_method": best.method,
        "sample_count": len(values),
        "inlier_sample_indices": list(best.inlier_indices),
        "excluded_sample_indices": list(best.excluded_indices),
        "T_x": best.T_x.tolist(),
        "T_z": best.T_z.tolist(),
        "residual": best.metrics,
        "motion_diversity": diversity.to_dict(),
        "leave_one_out_z_stability": stability,
        "bootstrap_z_stability": bootstrap,
        "methods": [
            {
                "method": solution.method,
                "score": solution.score,
                "inlier_sample_indices": list(solution.inlier_indices),
                "excluded_sample_indices": list(solution.excluded_indices),
                "T_x": solution.T_x.tolist(),
                "T_z": solution.T_z.tolist(),
                **solution.metrics,
            }
            for solution in solutions
        ],
        "method_errors": errors,
        "quality_settings": settings,
    }


def evaluate_ax_zb_validation(
    samples: Iterable[HandEyeSample],
    *,
    T_x: np.ndarray,
    T_z: np.ndarray,
) -> dict[str, float | int]:
    """Evaluate held-out canonical samples without fitting or rejection."""
    values = list(samples)
    if not values:
        raise CalibrationQualityError("independent validation has no samples")
    metrics = _evaluate_solution(
        values,
        tuple(range(len(values))),
        validate_transform(T_x, name="T_x"),
        validate_transform(T_z, name="T_z"),
    )
    return {"sample_count": len(values), **metrics}


def leave_one_out_x_stability(
    samples: list[HandEyeSample],
    *,
    method: str,
    inlier_sample_indices: tuple[int, ...],
    reference: np.ndarray,
    quality: dict[str, Any],
) -> dict[str, float | int | None]:
    sample_index_to_offset = {
        sample.index: offset for offset, sample in enumerate(samples)
    }
    offsets = tuple(
        sample_index_to_offset[index]
        for index in inlier_sample_indices
        if index in sample_index_to_offset
    )
    translation_errors: list[float] = []
    rotation_errors: list[float] = []
    for omitted in offsets:
        fold = tuple(index for index in offsets if index != omitted)
        if len(fold) < max(3, int(quality["min_samples"]) - 1):
            continue
        try:
            T_x, _ = _opencv_solve(samples, fold, method)
            translation, rotation = transform_error(reference, T_x)
        except (RuntimeError, ValueError):
            continue
        translation_errors.append(translation)
        rotation_errors.append(rotation)
    if not translation_errors:
        return {
            "successful_folds": 0,
            "translation_rms_m": None,
            "translation_max_m": None,
            "rotation_rms_deg": None,
            "rotation_max_deg": None,
        }
    translations = np.asarray(translation_errors, dtype=np.float64)
    rotations = np.asarray(rotation_errors, dtype=np.float64)
    return {
        "successful_folds": len(translation_errors),
        "translation_rms_m": float(np.sqrt(np.mean(translations**2))),
        "translation_max_m": float(np.max(translations)),
        "rotation_rms_deg": math.degrees(float(np.sqrt(np.mean(rotations**2)))),
        "rotation_max_deg": math.degrees(float(np.max(rotations))),
    }


def _solve_robust(
    samples: list[HandEyeSample],
    method: str,
    quality: dict[str, Any],
) -> _MethodSolution:
    all_indices = tuple(range(len(samples)))
    T_x, T_z = _opencv_solve(samples, all_indices, method)
    per_sample = _per_sample_residuals(
        samples, all_indices, T_x, T_z
    )
    combined = np.asarray(
        [item["translation_m"] + 0.02 * item["rotation_rad"] for item in per_sample]
    )
    median = float(np.median(combined))
    mad = float(np.median(np.abs(combined - median)))
    threshold = median + max(0.003, 4.0 * 1.4826 * mad)
    candidates = [
        index for index, value in enumerate(combined) if float(value) > threshold
    ]
    max_excluded = int(math.floor(len(samples) * float(quality["max_outlier_fraction"])))
    candidates = sorted(candidates, key=lambda index: combined[index], reverse=True)
    excluded = tuple(sorted(candidates[:max_excluded]))
    inliers = tuple(index for index in all_indices if index not in excluded)
    if excluded and len(inliers) >= int(quality["min_samples"]):
        T_x, T_z = _opencv_solve(samples, inliers, method)
    else:
        inliers = all_indices
        excluded = ()
    metrics = _evaluate_solution(samples, inliers, T_x, T_z)
    return _MethodSolution(
        method=method,
        T_x=validate_transform(T_x),
        T_z=validate_transform(T_z),
        inlier_indices=tuple(samples[index].index for index in inliers),
        excluded_indices=tuple(samples[index].index for index in excluded),
        metrics=metrics,
    )


def _opencv_solve(
    samples: list[HandEyeSample],
    indices: tuple[int, ...],
    method: str,
) -> tuple[np.ndarray, np.ndarray]:
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError(
            "OpenCV contrib is required; run uv sync --only-group handeye"
        ) from exc
    method_values = {
        "shah": cv2.CALIB_ROBOT_WORLD_HAND_EYE_SHAH,
        "li": cv2.CALIB_ROBOT_WORLD_HAND_EYE_LI,
    }
    selected = [samples[index] for index in indices]
    A = [sample.T_base_eef for sample in selected]
    B = [sample.T_camera_target for sample in selected]
    try:
        R_x, t_x, R_z, t_z = (
            cv2.calibrateRobotWorldHandEye(
                [value[:3, :3] for value in A],
                [value[:3, 3].reshape(3, 1) for value in A],
                [value[:3, :3] for value in B],
                [value[:3, 3].reshape(3, 1) for value in B],
                method=method_values[method],
            )
        )
    except cv2.error as exc:
        raise RuntimeError(str(exc)) from exc
    T_x = make_transform(
        R_x, np.asarray(t_x).reshape(3)
    )
    T_z = make_transform(
        R_z, np.asarray(t_z).reshape(3)
    )
    return T_x, T_z


def _per_sample_residuals(
    samples: list[HandEyeSample],
    indices: tuple[int, ...],
    T_x: np.ndarray,
    T_z: np.ndarray,
) -> list[dict[str, float]]:
    residuals = []
    for index in indices:
        sample = samples[index]
        left = sample.T_base_eef @ T_x
        right = T_z @ sample.T_camera_target
        translation, rotation = transform_error(left, right)
        residuals.append(
            {
                "translation_m": translation,
                "rotation_rad": rotation,
            }
        )
    return residuals


def _evaluate_solution(
    samples: list[HandEyeSample],
    indices: tuple[int, ...],
    T_x: np.ndarray,
    T_z: np.ndarray,
) -> dict[str, float]:
    residuals = _per_sample_residuals(
        samples, indices, T_x, T_z
    )
    translations = np.asarray([item["translation_m"] for item in residuals])
    rotations = np.asarray([item["rotation_rad"] for item in residuals])
    translation_rms = float(np.sqrt(np.mean(translations**2)))
    rotation_rms_rad = float(np.sqrt(np.mean(rotations**2)))
    return {
        "score": translation_rms + 0.02 * rotation_rms_rad,
        "closure_translation_rms_m": translation_rms,
        "closure_translation_mean_m": float(np.mean(translations)),
        "closure_translation_std_m": float(np.std(translations, ddof=0)),
        "closure_translation_p95_m": float(np.quantile(translations, 0.95)),
        "closure_translation_max_m": float(np.max(translations)),
        "closure_rotation_rms_deg": math.degrees(rotation_rms_rad),
        "closure_rotation_mean_deg": math.degrees(float(np.mean(rotations))),
        "closure_rotation_std_deg": math.degrees(float(np.std(rotations, ddof=0))),
        "closure_rotation_p95_deg": math.degrees(float(np.quantile(rotations, 0.95))),
        "closure_rotation_max_deg": math.degrees(float(np.max(rotations))),
        "mean_reprojection_error_px": float(
            np.mean([samples[index].reprojection_error_px for index in indices])
        ),
    }


def _leave_one_out_z_stability(
    samples: list[HandEyeSample],
    solution: _MethodSolution,
    quality: dict[str, Any],
) -> dict[str, float | int | None]:
    sample_index_to_offset = {
        sample.index: offset for offset, sample in enumerate(samples)
    }
    offsets = tuple(
        sample_index_to_offset[index] for index in solution.inlier_indices
    )
    translation_errors: list[float] = []
    rotation_errors: list[float] = []
    if len(offsets) <= 3:
        return {
            "successful_folds": 0,
            "translation_rms_m": None,
            "translation_max_m": None,
            "rotation_rms_deg": None,
            "rotation_max_deg": None,
        }
    for omitted in offsets:
        fold = tuple(index for index in offsets if index != omitted)
        try:
            _, T_z = _opencv_solve(samples, fold, solution.method)
        except (RuntimeError, ValueError):
            continue
        translation, rotation = transform_error(
            solution.T_z, T_z
        )
        translation_errors.append(translation)
        rotation_errors.append(rotation)
    if not translation_errors:
        raise CalibrationQualityError("leave-one-out stability solve failed")
    translation_values = np.asarray(translation_errors)
    rotation_values = np.asarray(rotation_errors)
    return {
        "successful_folds": len(translation_errors),
        "translation_rms_m": float(np.sqrt(np.mean(translation_values**2))),
        "translation_max_m": float(np.max(translation_values)),
        "rotation_rms_deg": math.degrees(
            float(np.sqrt(np.mean(rotation_values**2)))
        ),
        "rotation_max_deg": math.degrees(float(np.max(rotation_values))),
    }


def bootstrap_transform_stability(
    samples: list[HandEyeSample],
    *,
    method: str,
    inlier_sample_indices: tuple[int, ...],
    reference: np.ndarray,
    transform_component: str,
    quality: dict[str, Any],
) -> dict[str, float | int | None]:
    """Measure transform uncertainty by bootstrap resampling inliers.

    The resampled solves retain duplicate observations as required by the
    bootstrap.  Degenerate resamples are recorded as failed instead of being
    silently treated as zero error; the caller can therefore reject a result
    whose uncertainty estimate is not sufficiently well-conditioned.
    """
    requested = int(quality.get("bootstrap_iterations", 0))
    if requested <= 0:
        return {
            "requested_iterations": requested,
            "successful_iterations": 0,
            "success_ratio": None,
            "translation_rms_m": None,
            "translation_p95_m": None,
            "translation_max_m": None,
            "rotation_rms_deg": None,
            "rotation_p95_deg": None,
            "rotation_max_deg": None,
        }
    if transform_component not in {"x", "z"}:
        raise ValueError(
            f"unknown bootstrap transform component: {transform_component}"
        )

    by_index = {sample.index: sample for sample in samples}
    inliers = [
        by_index[index] for index in inlier_sample_indices if index in by_index
    ]
    if len(inliers) < 3:
        raise CalibrationQualityError("too few inliers for bootstrap stability")
    seed = int(quality.get("bootstrap_seed", 20260729))
    rng = np.random.default_rng(seed)
    translations: list[float] = []
    rotations: list[float] = []
    count = len(inliers)
    offsets = tuple(range(count))
    for _ in range(requested):
        resampled = [
            inliers[int(index)] for index in rng.integers(0, count, size=count)
        ]
        try:
            T_x, T_z = _opencv_solve(resampled, offsets, method)
            candidate = T_x if transform_component == "x" else T_z
            translation, rotation = transform_error(reference, candidate)
        except (RuntimeError, ValueError, np.linalg.LinAlgError):
            continue
        if not (math.isfinite(translation) and math.isfinite(rotation)):
            continue
        translations.append(float(translation))
        rotations.append(float(rotation))

    successful = len(translations)
    if not successful:
        return {
            "requested_iterations": requested,
            "successful_iterations": 0,
            "success_ratio": 0.0,
            "translation_rms_m": None,
            "translation_p95_m": None,
            "translation_max_m": None,
            "rotation_rms_deg": None,
            "rotation_p95_deg": None,
            "rotation_max_deg": None,
        }
    translation_values = np.asarray(translations, dtype=np.float64)
    rotation_values = np.asarray(rotations, dtype=np.float64)
    return {
        "requested_iterations": requested,
        "successful_iterations": successful,
        "success_ratio": successful / requested,
        "translation_rms_m": float(np.sqrt(np.mean(translation_values**2))),
        "translation_p95_m": float(np.quantile(translation_values, 0.95)),
        "translation_max_m": float(np.max(translation_values)),
        "rotation_rms_deg": math.degrees(
            float(np.sqrt(np.mean(rotation_values**2)))
        ),
        "rotation_p95_deg": math.degrees(float(np.quantile(rotation_values, 0.95))),
        "rotation_max_deg": math.degrees(float(np.max(rotation_values))),
    }


def _max_pairwise_distance(values: np.ndarray) -> float:
    maximum = 0.0
    for index, first in enumerate(values):
        for second in values[index + 1 :]:
            maximum = max(maximum, float(np.linalg.norm(first - second)))
    return maximum
