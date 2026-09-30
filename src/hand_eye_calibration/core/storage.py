"""Atomic session storage and active hand-eye result cache."""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any

from .schema import CameraIntrinsics, EyeToHandSample


class CalibrationSession:
    def __init__(self, path: Path) -> None:
        self.path = path
        self.images = path / "images"
        self.samples_path = path / "samples.json"
        self.result_path = path / "result.json"

    def initialize(
        self,
        *,
        config: dict[str, Any],
        intrinsics: CameraIntrinsics,
    ) -> None:
        self.images.mkdir(parents=True, exist_ok=True)
        if not self.samples_path.exists():
            self.write_samples([], config=config, intrinsics=intrinsics)

    def read_samples(self) -> tuple[list[EyeToHandSample], dict[str, Any]]:
        with self.samples_path.open("r", encoding="utf-8") as stream:
            payload = json.load(stream)
        samples = [EyeToHandSample.from_dict(value) for value in payload["samples"]]
        return samples, payload

    def write_samples(
        self,
        samples: list[EyeToHandSample],
        *,
        config: dict[str, Any],
        intrinsics: CameraIntrinsics,
    ) -> None:
        _atomic_json(
            self.samples_path,
            {
                "schema_version": 1,
                "calibration_type": "eye_to_hand",
                "camera": intrinsics.to_dict(),
                "config": config,
                "samples": [sample.to_dict() for sample in samples],
            },
        )

    def initialize_dual(
        self,
        *,
        config: dict[str, Any],
    ) -> None:
        self.images.mkdir(parents=True, exist_ok=True)
        if not self.samples_path.exists():
            self.write_dual_samples(
                {"eye_in_hand": [], "eye_to_hand": []},
                config=config,
                cameras={},
            )

    def read_dual_samples(
        self,
    ) -> tuple[dict[str, list[EyeToHandSample]], dict[str, Any]]:
        with self.samples_path.open("r", encoding="utf-8") as stream:
            payload = json.load(stream)
        datasets = payload.get("datasets")
        if not isinstance(datasets, dict):
            raise ValueError("session does not contain dual-camera datasets")
        return {
            role: [EyeToHandSample.from_dict(value) for value in datasets.get(role, [])]
            for role in ("eye_in_hand", "eye_to_hand")
        }, payload

    def write_dual_samples(
        self,
        datasets: dict[str, list[EyeToHandSample]],
        *,
        config: dict[str, Any],
        cameras: dict[str, CameraIntrinsics],
        validation_datasets: dict[str, list[EyeToHandSample]] | None = None,
        operator_reviews: dict[str, dict[str, Any]] | None = None,
    ) -> None:
        validation_datasets = validation_datasets or {}
        operator_reviews = operator_reviews or {}
        _atomic_json(
            self.samples_path,
            {
                "schema_version": 3,
                "calibration_type": "dual_camera_hand_eye",
                "cameras": {
                    role: intrinsics.to_dict()
                    for role, intrinsics in cameras.items()
                },
                "config": config,
                "datasets": {
                    role: [sample.to_dict() for sample in datasets.get(role, [])]
                    for role in ("eye_in_hand", "eye_to_hand")
                },
                "validation_datasets": {
                    role: [
                        sample.to_dict()
                        for sample in validation_datasets.get(role, [])
                    ]
                    for role in ("eye_in_hand", "eye_to_hand")
                },
                "operator_reviews": {
                    role: value
                    for role, value in operator_reviews.items()
                    if role in {"eye_in_hand", "eye_to_hand"}
                    and isinstance(value, dict)
                },
            },
        )

    def write_result(self, result: dict[str, Any], *, cache_root: Path) -> None:
        _atomic_json(self.result_path, result)
        write_cached_result(cache_root, "current.json", result)

    def write_phase_result(
        self,
        result: dict[str, Any],
        *,
        cache_root: Path,
        role: str,
    ) -> None:
        """Persist one camera phase without replacing the active profile."""
        _atomic_json(self.result_path, result)
        write_cached_result(cache_root, phase_cache_filename(role), result)


def resolve_session(root: Path, value: str | Path) -> CalibrationSession:
    path = Path(value)
    if not path.is_absolute():
        path = root / path
    return CalibrationSession(path.resolve())


def phase_cache_filename(role: str) -> str:
    """Return the stable per-camera result cache filename for a dual session."""
    if role not in {"eye_in_hand", "eye_to_hand"}:
        raise ValueError(f"unknown hand-eye camera role: {role}")
    return f"{role}.json"


def read_cached_result(cache_root: Path, filename: str) -> dict[str, Any] | None:
    """Read a cache file defensively; a damaged optional phase is ignored."""
    try:
        with (cache_root / filename).open("r", encoding="utf-8") as stream:
            value = json.load(stream)
    except (OSError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def write_cached_result(
    cache_root: Path,
    filename: str,
    result: dict[str, Any],
) -> None:
    """Atomically update a named cache result without copying through a session."""
    if Path(filename).name != filename:
        raise ValueError("cache filename must not contain a directory")
    _atomic_json(cache_root / filename, result)


def _atomic_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)
