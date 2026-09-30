"""Validation and explicit activation of a solved eye-to-hand profile.

The solve cache is deliberately kept separate from the enabled profile.  A
solve can be inspected or replaced without changing the transform used by
downstream perception code.
"""

from __future__ import annotations

from dataclasses import dataclass
import datetime as dt
import json
import os
from pathlib import Path
from typing import Any

from .transforms import validate_transform


@dataclass(frozen=True)
class ValidationThresholds:
    """Initial acceptance gates for a small tabletop grasping workspace."""

    rms_translation_mm: float = 5.0
    rms_rotation_deg: float = 2.0
    max_translation_mm: float = 10.0
    max_rotation_deg: float = 3.0
    minimum_samples: int = 12


def validate_result(
    result: dict[str, Any],
    *,
    thresholds: ValidationThresholds | None = None,
) -> dict[str, Any]:
    """Evaluate a cached solve and return a JSON-safe validation report."""
    limits = thresholds or ValidationThresholds()
    residual = result.get("residual")
    stability = result.get("leave_one_out_stability")
    checks: dict[str, dict[str, Any]] = {}

    def check(name: str, value: float | int | None, limit: float | int, *, lower: bool = True) -> None:
        numeric = isinstance(value, (int, float)) and value == value
        passed = bool(numeric and (value >= limit if not lower else value <= limit))
        checks[name] = {
            "value": value,
            "limit": limit,
            "passed": passed,
            "operator": ">=" if not lower else "<=",
        }

    sample_count = result.get("sample_count")
    check("sample_count", sample_count, limits.minimum_samples, lower=False)
    closure_translation = (
        1000.0 * residual.get("closure_translation_rms_m")
        if isinstance(residual, dict)
        and isinstance(residual.get("closure_translation_rms_m"), (int, float))
        else None
    )
    closure_rotation = (
        residual.get("closure_rotation_rms_deg")
        if isinstance(residual, dict)
        else None
    )
    closure_translation_max = (
        1000.0 * residual.get("closure_translation_max_m")
        if isinstance(residual, dict)
        and isinstance(residual.get("closure_translation_max_m"), (int, float))
        else None
    )
    closure_rotation_max = (
        residual.get("closure_rotation_max_deg")
        if isinstance(residual, dict)
        else None
    )
    check("closure_translation_rms_mm", closure_translation, limits.rms_translation_mm)
    check("closure_rotation_rms_deg", closure_rotation, limits.rms_rotation_deg)
    check("closure_translation_max_mm", closure_translation_max, limits.max_translation_mm)
    check("closure_rotation_max_deg", closure_rotation_max, limits.max_rotation_deg)

    transform_error: str | None = None
    try:
        validate_transform(result["T_base_camera"], name="T_base_camera")
    except (KeyError, ValueError, TypeError) as exc:
        transform_error = str(exc)
    checks["transform"] = {
        "value": "valid" if transform_error is None else transform_error,
        "limit": "finite proper SE(3)",
        "passed": transform_error is None,
        "operator": "is",
    }
    is_dual = result.get("calibration_type") == "dual_camera_hand_eye"
    if is_dual:
        hand = result.get("eye_in_hand")
        hand_residual = hand.get("residual") if isinstance(hand, dict) else None
        hand_samples = hand.get("sample_count") if isinstance(hand, dict) else None
        check(
            "eye_in_hand_sample_count",
            hand_samples,
            limits.minimum_samples,
            lower=False,
        )
        hand_metrics = {
            "closure_translation_rms_mm": (
                1000.0 * hand_residual.get("closure_translation_rms_m")
                if isinstance(hand_residual, dict)
                and isinstance(
                    hand_residual.get("closure_translation_rms_m"), (int, float)
                )
                else None
            ),
            "closure_rotation_rms_deg": (
                hand_residual.get("closure_rotation_rms_deg")
                if isinstance(hand_residual, dict)
                else None
            ),
            "closure_translation_max_mm": (
                1000.0 * hand_residual.get("closure_translation_max_m")
                if isinstance(hand_residual, dict)
                and isinstance(
                    hand_residual.get("closure_translation_max_m"), (int, float)
                )
                else None
            ),
            "closure_rotation_max_deg": (
                hand_residual.get("closure_rotation_max_deg")
                if isinstance(hand_residual, dict)
                else None
            ),
        }
        for name, limit in (
            ("closure_translation_rms_mm", limits.rms_translation_mm),
            ("closure_rotation_rms_deg", limits.rms_rotation_deg),
            ("closure_translation_max_mm", limits.max_translation_mm),
            ("closure_rotation_max_deg", limits.max_rotation_deg),
        ):
            check(f"eye_in_hand_{name}", hand_metrics[name], limit)
        hand_transform_error: str | None = None
        try:
            validate_transform(result["T_eef_camera"], name="T_eef_camera")
        except (KeyError, ValueError, TypeError) as exc:
            hand_transform_error = str(exc)
        checks["eye_in_hand_transform"] = {
            "value": (
                "valid" if hand_transform_error is None else hand_transform_error
            ),
            "limit": "finite proper SE(3)",
            "passed": hand_transform_error is None,
            "operator": "is",
        }
        for role in ("eye_in_hand", "eye_to_hand"):
            phase = result.get(role)
            held_out = (
                phase.get("independent_validation")
                if isinstance(phase, dict)
                else None
            )
            # Older/manual sessions have no held-out pass and remain
            # compatible.  When an automatic session provides one, make it a
            # first-class acceptance check rather than hiding it in the report.
            if not isinstance(held_out, dict):
                continue
            prefix = f"{role}_independent"
            check(
                f"{prefix}_sample_count",
                held_out.get("sample_count"),
                4,
                lower=False,
            )
            check(
                f"{prefix}_translation_rms_mm",
                (
                    1000.0 * held_out["closure_translation_rms_m"]
                    if isinstance(
                        held_out.get("closure_translation_rms_m"), (int, float)
                    )
                    else None
                ),
                limits.rms_translation_mm,
            )
            check(
                f"{prefix}_rotation_rms_deg",
                held_out.get("closure_rotation_rms_deg"),
                limits.rms_rotation_deg,
            )
            check(
                f"{prefix}_translation_max_mm",
                (
                    1000.0 * held_out["closure_translation_max_m"]
                    if isinstance(
                        held_out.get("closure_translation_max_m"), (int, float)
                    )
                    else None
                ),
                limits.max_translation_mm,
            )
            check(
                f"{prefix}_rotation_max_deg",
                held_out.get("closure_rotation_max_deg"),
                limits.max_rotation_deg,
            )
    passed = all(bool(item["passed"]) for item in checks.values())
    raw_reviews = result.get("operator_reviews", {})
    operator_reviews = {
        role: dict(raw_reviews[role])
        for role in ("eye_in_hand", "eye_to_hand")
        if isinstance(raw_reviews, dict) and isinstance(raw_reviews.get(role), dict)
    }
    operator_reviewed = bool(is_dual and all(
        review.get("accepted") is True
        and isinstance(review.get("fit_sample_count"), int)
        and review["fit_sample_count"] >= 20
        for review in operator_reviews.values()
    ) and len(operator_reviews) == 2)
    report = {
        "schema_version": 3 if is_dual else 1,
        "profile": "dual-camera-hand-eye" if is_dual else "eye-to-hand",
        "passed": passed,
        "checked_at": dt.datetime.now(dt.timezone.utc).isoformat(),
        "thresholds": {
            "rms_translation_mm": limits.rms_translation_mm,
            "rms_rotation_deg": limits.rms_rotation_deg,
            "max_translation_mm": limits.max_translation_mm,
            "max_rotation_deg": limits.max_rotation_deg,
            "minimum_samples": limits.minimum_samples,
        },
        "checks": checks,
        "operator_reviewed": operator_reviewed,
        "operator_reviews": operator_reviews,
        "sample_count": sample_count,
        "source_session": result.get("session"),
        "source_created_at": result.get("created_at"),
        "source_T_base_camera": result.get("T_base_camera"),
        "source_T_eef_camera": result.get("T_eef_camera"),
    }
    return report


def read_json(path: Path) -> dict[str, Any] | None:
    try:
        with path.open("r", encoding="utf-8") as stream:
            value = json.load(stream)
    except (OSError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def write_json(path: Path, value: dict[str, Any]) -> None:
    """Atomically persist a small profile/report, keeping readers consistent."""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def validate_cached_result(
    root: Path,
    *,
    thresholds: ValidationThresholds | None = None,
) -> dict[str, Any]:
    """Validate ``current.json`` and persist ``validation.json``."""
    result_path = root / "current.json"
    result = read_json(result_path)
    if result is None:
        raise FileNotFoundError(f"未找到手眼标定结果：{result_path}")
    report = validate_result(result, thresholds=thresholds)
    write_json(root / "validation.json", report)
    return report


def enable_validated_result(
    root: Path,
    *,
    operator_override: bool = False,
) -> dict[str, Any]:
    """Copy the validated solve into the explicitly enabled profile."""
    result = read_json(root / "current.json")
    report = read_json(root / "validation.json")
    if result is None:
        raise FileNotFoundError("未找到手眼标定结果，请先完成标定。")
    if report is None:
        raise RuntimeError("请先验证手眼标定结果。")
    reviewed_override = False
    if report.get("passed") is not True:
        if not operator_override:
            raise RuntimeError("手眼标定结果未通过验证；请复核后明确选择人工采用。")
        if report.get("operator_reviewed") is not True:
            raise RuntimeError("没有两阶段的 20 组人工评审记录，不能覆盖验证建议。")
        checks = report.get("checks", {})
        required_structural_checks = (
            "sample_count",
            "transform",
            "eye_in_hand_sample_count",
            "eye_in_hand_transform",
        )
        if not all(
            isinstance(checks.get(name), dict)
            and checks[name].get("passed") is True
            for name in required_structural_checks
        ):
            raise RuntimeError("标定结果缺少有效样本或合法 SE(3) 变换，不能人工采用。")
        reviewed_override = True
    current_stamp = result.get("created_at")
    report_stamp = report.get("source_created_at")
    if current_stamp is not None and report_stamp != current_stamp:
        raise RuntimeError("当前求解已更新，请重新验证后再启用。")
    if current_stamp is None and report.get("source_T_base_camera") is not None:
        if report.get("source_T_base_camera") != result.get("T_base_camera"):
            raise RuntimeError("当前求解已更新，请重新验证后再启用。")
    enabled = {
        "schema_version": (
            3 if result.get("calibration_type") == "dual_camera_hand_eye" else 1
        ),
        "profile": (
            "dual-camera-hand-eye"
            if result.get("calibration_type") == "dual_camera_hand_eye"
            else "eye-to-hand"
        ),
        "enabled": True,
        "enabled_at": dt.datetime.now(dt.timezone.utc).isoformat(),
        "source_result": str(root / "current.json"),
        "validation": report,
        "operator_override": reviewed_override,
        "T_base_camera": result["T_base_camera"],
        "T_eef_camera": result.get("T_eef_camera"),
        "camera": result.get("camera"),
        "cameras": result.get("cameras"),
        "target": result.get("target"),
    }
    write_json(root / "enabled.json", enabled)
    return enabled


def disable_result(root: Path) -> dict[str, Any]:
    """Write an explicit disabled marker without deleting the cached solve."""
    disabled = {
        "schema_version": 1,
        "profile": "eye-to-hand",
        "enabled": False,
        "disabled_at": dt.datetime.now(dt.timezone.utc).isoformat(),
    }
    write_json(root / "enabled.json", disabled)
    return disabled
