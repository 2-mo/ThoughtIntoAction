"""Integrity checks for raw and exported ARX5 episodes."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Iterable

import numpy as np

from .config import CameraConfig, CollectionConfig, load_collection_config
from .raw_episode import atomic_json, read_json


DEFAULT_GRIPPER_POSITION_TOLERANCE_M = 0.005
MINIMUM_USABLE_CAMERA_RATE_FRACTION = 0.5


def read_jsonl(path: Path | str) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    with Path(path).open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            if not line.strip():
                continue
            value = json.loads(line)
            if not isinstance(value, dict):
                raise ValueError(f"{path}:{line_number} must be a JSON object")
            records.append(value)
    return records


def _finite_vector(
    value: Any,
    size: int,
    label: str,
    errors: list[str],
) -> np.ndarray | None:
    try:
        array = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError):
        errors.append(f"{label} is not numeric")
        return None
    if array.shape != (size,):
        errors.append(f"{label} must have shape ({size},), got {array.shape}")
        return None
    if not np.all(np.isfinite(array)):
        errors.append(f"{label} contains non-finite values")
        return None
    return array


def _strictly_increasing(values: np.ndarray) -> bool:
    return len(values) < 2 or bool(np.all(np.diff(values) > 0))


def nearest_alignment_ns(
    reference_ns: np.ndarray,
    query_ns: np.ndarray,
) -> np.ndarray:
    """Return absolute distance from every query timestamp to its nearest reference."""
    if reference_ns.ndim != 1 or query_ns.ndim != 1 or len(reference_ns) == 0:
        raise ValueError("timestamp inputs must be non-empty one-dimensional arrays")
    insertion = np.searchsorted(reference_ns, query_ns, side="left")
    left = np.clip(insertion - 1, 0, len(reference_ns) - 1)
    right = np.clip(insertion, 0, len(reference_ns) - 1)
    return np.minimum(
        np.abs(query_ns - reference_ns[left]),
        np.abs(reference_ns[right] - query_ns),
    )


def nearest_indices(reference_ns: np.ndarray, query_ns: np.ndarray) -> np.ndarray:
    """Return the nearest reference index for each query timestamp."""
    if reference_ns.ndim != 1 or query_ns.ndim != 1 or len(reference_ns) == 0:
        raise ValueError("timestamp inputs must be non-empty one-dimensional arrays")
    insertion = np.searchsorted(reference_ns, query_ns, side="left")
    left = np.clip(insertion - 1, 0, len(reference_ns) - 1)
    right = np.clip(insertion, 0, len(reference_ns) - 1)
    choose_right = np.abs(reference_ns[right] - query_ns) < np.abs(
        query_ns - reference_ns[left]
    )
    return np.where(choose_right, right, left)


def unique_source_frame_indices(frames: list[dict[str, Any]]) -> np.ndarray:
    """Return video indices after removing repeated source-camera frames."""
    kept: list[int] = []
    seen: set[tuple[int | None, float | None]] = set()
    for index, frame in enumerate(frames):
        try:
            frame_number = (
                None
                if frame.get("frame_number") is None
                else int(frame["frame_number"])
            )
            device_timestamp_ms = (
                None
                if frame.get("device_timestamp_ms") is None
                else float(frame["device_timestamp_ms"])
            )
        except (TypeError, ValueError):
            kept.append(index)
            continue
        identity = (frame_number, device_timestamp_ms)
        if identity == (None, None):
            kept.append(index)
        elif identity not in seen:
            seen.add(identity)
            kept.append(index)
    return np.asarray(kept, dtype=np.int64)


def uniform_frame_timestamps(
    overlap_start_ns: int,
    overlap_end_ns: int,
    fps: int,
) -> np.ndarray:
    """Build the constant-rate episode timeline required by LeRobot video."""
    if fps <= 0 or overlap_end_ns < overlap_start_ns:
        return np.asarray([], dtype=np.int64)
    duration_ns = overlap_end_ns - overlap_start_ns
    frame_count = int(math.floor(duration_ns * fps / 1_000_000_000)) + 1
    offsets = np.rint(
        np.arange(frame_count, dtype=np.float64) * (1_000_000_000.0 / fps)
    ).astype(np.int64)
    result = np.int64(overlap_start_ns) + offsets
    return result[result <= overlap_end_ns]


def shared_timeline_frame_selection(
    frames_by_video_key: dict[str, list[dict[str, Any]]],
    source_indices_by_video_key: dict[str, np.ndarray],
    *,
    state_first_ns: int,
    state_last_ns: int,
    fps: int,
) -> tuple[np.ndarray, dict[str, np.ndarray]] | None:
    """Select synchronized video indices from recorder-assigned timeline slots."""
    if fps <= 0 or state_last_ns < state_first_ns:
        raise ValueError("shared timeline bounds or FPS are invalid")
    slots_by_video_key: dict[str, np.ndarray] = {}
    origins: list[int] = []
    has_timeline = False
    missing_timeline = False
    for video_key, frames in frames_by_video_key.items():
        source_indices = source_indices_by_video_key[video_key]
        records = [frames[int(index)] for index in source_indices]
        record_has_timeline = [
            "timeline_slot" in record
            and "timeline_target_monotonic_ns" in record
            for record in records
        ]
        has_timeline = has_timeline or any(record_has_timeline)
        missing_timeline = missing_timeline or not all(record_has_timeline)
        if not records or not all(record_has_timeline):
            continue
        try:
            slots = np.asarray(
                [int(record["timeline_slot"]) for record in records],
                dtype=np.int64,
            )
            targets = np.asarray(
                [int(record["timeline_target_monotonic_ns"]) for record in records],
                dtype=np.int64,
            )
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{video_key} shared timeline is not integral") from exc
        if len(slots) < 2 or not _strictly_increasing(slots):
            raise ValueError(
                f"{video_key} shared timeline slots are not strictly increasing"
            )
        offsets = np.asarray(
            [(int(slot) * 1_000_000_000 + fps // 2) // fps for slot in slots],
            dtype=np.int64,
        )
        stream_origins = targets - offsets
        if int(np.max(stream_origins) - np.min(stream_origins)) > 1:
            raise ValueError(f"{video_key} shared timeline origin is inconsistent")
        origins.append(int(stream_origins[0]))
        slots_by_video_key[video_key] = slots
    if not has_timeline:
        return None
    if missing_timeline or set(slots_by_video_key) != set(frames_by_video_key):
        raise ValueError("camera streams mix shared-timeline and legacy frames")
    if max(origins) - min(origins) > 1:
        raise ValueError("camera streams do not share the same timeline origin")
    origin_ns = origins[0]
    first_slot = max(int(slots[0]) for slots in slots_by_video_key.values())
    last_slot = min(int(slots[-1]) for slots in slots_by_video_key.values())
    if last_slot < first_slot:
        raise ValueError("camera shared timelines do not overlap")
    target_slots = np.arange(first_slot, last_slot + 1, dtype=np.int64)
    frame_ns = origin_ns + np.asarray(
        [
            (int(slot) * 1_000_000_000 + fps // 2) // fps
            for slot in target_slots
        ],
        dtype=np.int64,
    )
    within_state = (frame_ns >= state_first_ns) & (frame_ns <= state_last_ns)
    target_slots = target_slots[within_state]
    frame_ns = frame_ns[within_state]
    if len(frame_ns) < 2:
        raise ValueError("fewer than two shared-timeline frames overlap state")
    selected_indices = {
        video_key: source_indices_by_video_key[video_key][
            nearest_indices(slots, target_slots)
        ]
        for video_key, slots in slots_by_video_key.items()
    }
    return frame_ns, selected_indices


def _read_camera_stream(
    episode_path: Path,
    camera: CameraConfig,
    errors: list[str],
    warnings: list[str],
) -> tuple[list[dict[str, Any]], dict[str, Any], np.ndarray, np.ndarray]:
    label = f"camera[{camera.video_key}]"
    camera_path = episode_path / "cameras" / camera.video_key
    try:
        frames = read_jsonl(camera_path / "frames.jsonl")
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        frames = []
        errors.append(f"{label} stream unreadable: {exc}")
    try:
        summary = read_json(camera_path / "camera.json")
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        summary = {}
        errors.append(f"{label} summary unreadable: {exc}")
    if summary.get("error"):
        errors.append(f"{label} recorder failed: {summary['error']}")
    if summary.get("stage") != "complete":
        errors.append(f"{label} recording is not complete")
    if (
        summary.get("video_key") != camera.video_key
        or summary.get("width") != camera.width
        or summary.get("height") != camera.height
        or summary.get("fps") != camera.fps
    ):
        errors.append(f"{label} metadata does not match collection config")
    storage = summary.get("storage")
    if not isinstance(storage, dict) or (
        storage.get("container") != "matroska"
        or storage.get("codec") != "h264"
        or storage.get("video_file") != "video.mkv"
        or storage.get("frame_metadata_file") != "frames.jsonl"
    ):
        errors.append(f"{label} storage metadata is invalid")
    video_path = camera_path / "video.mkv"
    if not video_path.is_file() or video_path.stat().st_size < 4:
        errors.append(f"{label} video is missing or empty")
    else:
        with video_path.open("rb") as stream:
            if stream.read(4) != b"\x1aE\xdf\xa3":
                errors.append(f"{label} video is not a Matroska container")
    frame_times: list[int] = []
    valid_frames: list[dict[str, Any]] = []
    valid_video_indices: list[int] = []
    for index, record in enumerate(frames):
        try:
            timestamp = int(record["host_monotonic_ns"])
            frame_index = int(record["frame_index"])
        except (KeyError, TypeError, ValueError):
            errors.append(f"{label} frame metadata {index} is incomplete")
            continue
        if frame_index != index:
            errors.append(f"{label} frame index is discontinuous at {index}")
        frame_times.append(timestamp)
        valid_frames.append(record)
        valid_video_indices.append(index)
    raw_frame_ns = np.asarray(frame_times, dtype=np.int64)
    unique_positions = unique_source_frame_indices(valid_frames)
    source_indices = np.asarray(valid_video_indices, dtype=np.int64)[unique_positions]
    repeated_source_frames = len(valid_frames) - len(source_indices)
    if repeated_source_frames:
        warnings.append(
            f"{label}: {repeated_source_frames} repeated source frames were omitted"
        )
    frame_ns = raw_frame_ns[unique_positions]
    if len(frame_ns) < 2:
        errors.append(f"{label} has fewer than two valid frames")
    elif not _strictly_increasing(frame_ns):
        errors.append(f"{label} host timestamps are not strictly increasing")
    if summary.get("frame_count") not in (None, len(frames)):
        errors.append(f"{label} summary frame count does not match metadata")
    source_nonmonotonic = summary.get("source_nonmonotonic_frames")
    if isinstance(source_nonmonotonic, int) and source_nonmonotonic > 0:
        errors.append(
            f"{label} source frame sequence reset {source_nonmonotonic} times"
        )
    source_gaps = summary.get("source_frame_number_gaps")
    if isinstance(source_gaps, int) and source_gaps > 0:
        warnings.append(f"{label}: source camera skipped {source_gaps} frame numbers")
    recorder_duplicates = summary.get("duplicate_source_frames")
    if isinstance(recorder_duplicates, int) and recorder_duplicates > 0:
        warnings.append(
            f"{label}: recorder omitted {recorder_duplicates} repeated source frames"
        )
    return frames, summary, frame_ns, source_indices


def _still_window_seconds(
    timestamps_ns: np.ndarray,
    joint_position: np.ndarray,
    gripper_position: np.ndarray,
    *,
    from_start: bool,
) -> float:
    """Measure consecutive low-speed time at one edge of an episode.

    ARX joint positions are quantized.  Differencing one 100 Hz sample can turn
    a harmless two- or three-count update into a velocity spike, so classify
    each interval from a short edge-facing median window instead of one raw
    difference.
    """
    if len(timestamps_ns) < 2:
        return 0.0
    seconds = timestamps_ns.astype(np.float64) / 1_000_000_000.0
    dt = np.diff(seconds)
    if not np.all(np.isfinite(dt)) or np.any(dt <= 0.0):
        return 0.0
    joint_speed = np.max(np.abs(np.diff(joint_position, axis=0) / dt[:, None]), axis=1)
    gripper_speed = np.abs(np.diff(gripper_position) / dt)
    median_dt = float(np.median(dt))
    filter_samples = max(1, int(round(0.05 / median_dt)))
    filtered_joint_speed = np.empty_like(joint_speed)
    filtered_gripper_speed = np.empty_like(gripper_speed)
    for index in range(len(dt)):
        if from_start:
            window = slice(index, min(len(dt), index + filter_samples))
        else:
            window = slice(max(0, index - filter_samples + 1), index + 1)
        filtered_joint_speed[index] = np.median(joint_speed[window])
        filtered_gripper_speed[index] = np.median(gripper_speed[window])
    still = (filtered_joint_speed <= 0.08) & (filtered_gripper_speed <= 0.01)
    indices: Iterable[int] = range(len(still))
    if not from_start:
        indices = range(len(still) - 1, -1, -1)
    duration = 0.0
    for index in indices:
        if not still[index]:
            break
        duration += float(dt[index])
    return duration


def validate_raw_episode(
    episode_path: Path | str,
    config: CollectionConfig | None = None,
    *,
    write_report: bool = True,
) -> dict[str, Any]:
    """Validate one stopped raw episode and return a JSON-serializable report."""
    cfg = config or load_collection_config()
    path = Path(episode_path)
    errors: list[str] = []
    warnings: list[str] = []
    metrics: dict[str, Any] = {}

    try:
        manifest = read_json(path / "episode.json")
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        manifest = {}
        errors.append(f"episode manifest unreadable: {exc}")

    try:
        states = read_jsonl(path / "state" / "states.jsonl")
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        states = []
        errors.append(f"state stream unreadable: {exc}")
    try:
        state_summary = read_json(path / "state" / "summary.json")
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        state_summary = {}
        errors.append(f"state summary unreadable: {exc}")
    camera_streams = {
        camera.video_key: _read_camera_stream(path, camera, errors, warnings)
        for camera in cfg.cameras
    }
    for camera in cfg.cameras:
        summary = camera_streams[camera.video_key][1]
        controls = summary.get("color_controls")
        if not isinstance(controls, dict):
            warnings.append(
                f"{camera.video_key}: exposure/white-balance lock metadata is missing"
            )
            continue
        applied = controls.get("applied")
        control_warnings = controls.get("warnings")
        if isinstance(control_warnings, list):
            warnings.extend(
                f"{camera.video_key}: {warning}"
                for warning in control_warnings
                if str(warning).strip()
            )
        required_controls = {
            "auto_exposure",
            "exposure_us",
            "auto_white_balance",
            "white_balance_k",
        }
        if (
            not isinstance(applied, dict)
            or not required_controls.issubset(applied)
            or applied.get("auto_exposure") is not False
            or applied.get("auto_white_balance") is not False
            or controls.get("locked") is not True
        ):
            warnings.append(
                f"{camera.video_key}: exposure/white balance was not locked after warmup"
            )

    if manifest.get("dataset_id") != cfg.dataset_id:
        errors.append("dataset id does not match collection config")
    if manifest.get("batch_id") != cfg.batch_id:
        errors.append("batch id does not match collection config")
    if manifest.get("task_instruction") != cfg.task_instruction:
        errors.append("task instruction does not match collection config")
    if state_summary.get("error"):
        errors.append(f"state recorder failed: {state_summary['error']}")

    state_times: list[int] = []
    joint_rows: list[np.ndarray] = []
    gripper_rows: list[float] = []
    for index, record in enumerate(states):
        try:
            timestamp = int(record["host_monotonic_ns"])
        except (KeyError, TypeError, ValueError):
            errors.append(f"state[{index}] has an invalid host timestamp")
            continue
        joint = _finite_vector(
            record.get("joint_position"), 6, f"state[{index}].joint_position", errors
        )
        _finite_vector(
            record.get("joint_velocity"), 6, f"state[{index}].joint_velocity", errors
        )
        _finite_vector(
            record.get("joint_torque"), 6, f"state[{index}].joint_torque", errors
        )
        _finite_vector(
            record.get("eef_pose_6d"), 6, f"state[{index}].eef_pose_6d", errors
        )
        try:
            gripper = float(record["gripper_position"])
            if not math.isfinite(gripper):
                raise ValueError
        except (KeyError, TypeError, ValueError):
            errors.append(f"state[{index}].gripper_position is not finite")
            continue
        if joint is not None:
            state_times.append(timestamp)
            joint_rows.append(joint)
            gripper_rows.append(gripper)

    state_ns = np.asarray(state_times, dtype=np.int64)
    metrics["state_samples"] = len(state_ns)
    metrics["camera_frames"] = {
        video_key: len(stream[2])
        for video_key, stream in camera_streams.items()
    }
    metrics["camera_raw_frames"] = {
        video_key: len(stream[0])
        for video_key, stream in camera_streams.items()
    }
    camera_metrics: dict[str, Any] = {}
    metrics["cameras"] = camera_metrics
    if len(state_ns) < 2:
        errors.append("fewer than two valid state samples")
    elif not _strictly_increasing(state_ns):
        errors.append("state host timestamps are not strictly increasing")

    camera_timestamps = {
        video_key: stream[2] for video_key, stream in camera_streams.items()
    }
    streams_ready = len(state_ns) >= 2 and all(
        len(frame_ns) >= 2 for frame_ns in camera_timestamps.values()
    )
    if streams_ready:
        overlap_start = max(
            int(state_ns[0]),
            *(int(frame_ns[0]) for frame_ns in camera_timestamps.values()),
        )
        overlap_end = min(
            int(state_ns[-1]),
            *(int(frame_ns[-1]) for frame_ns in camera_timestamps.values()),
        )
        overlap_duration_s = max(0.0, (overlap_end - overlap_start) / 1e9)
        metrics["overlap_duration_s"] = overlap_duration_s
        if overlap_duration_s < cfg.sampling.minimum_episode_seconds:
            errors.append(
                f"overlap duration {overlap_duration_s:.3f}s is shorter than "
                f"{cfg.sampling.minimum_episode_seconds:.3f}s"
            )
        if overlap_duration_s > cfg.sampling.maximum_episode_seconds:
            errors.append(
                f"overlap duration {overlap_duration_s:.3f}s exceeds "
                f"{cfg.sampling.maximum_episode_seconds:.3f}s"
            )

        maximum_state_alignment_ms = 0.0
        p99_state_alignment_ms = 0.0
        maximum_video_gap_ms = 0.0
        maximum_drop_fraction = 0.0
        for camera in cfg.cameras:
            frames, summary, frame_ns, _source_indices = camera_streams[
                camera.video_key
            ]
            aligned_frames = frame_ns[
                (frame_ns >= state_ns[0]) & (frame_ns <= state_ns[-1])
            ]
            omitted = len(frame_ns) - len(aligned_frames)
            if omitted:
                warnings.append(
                    f"{camera.video_key}: {omitted} edge frames fall outside the "
                    "state stream and will be omitted"
                )
            if len(aligned_frames) < 2:
                errors.append(
                    f"{camera.video_key}: fewer than two frames overlap the state stream"
                )
                continue
            alignment_ms = nearest_alignment_ns(state_ns, aligned_frames) / 1e6
            camera_maximum_alignment_ms = float(np.max(alignment_ms))
            camera_p99_alignment_ms = float(np.quantile(alignment_ms, 0.99))
            camera_maximum_gap_ms = float(np.max(np.diff(aligned_frames)) / 1e6)
            camera_duration_s = float(
                (aligned_frames[-1] - aligned_frames[0]) / 1e9
            )
            effective_fps = (
                0.0
                if camera_duration_s <= 0.0
                else (len(aligned_frames) - 1) / camera_duration_s
            )
            minimum_usable_fps = (
                camera.fps * MINIMUM_USABLE_CAMERA_RATE_FRACTION
            )
            dropped = int(summary.get("dropped_queue_frames") or 0)
            total_seen = len(frame_ns) + dropped
            drop_fraction = 0.0 if total_seen == 0 else dropped / total_seen
            camera_metrics[camera.video_key] = {
                "frames": len(frame_ns),
                "aligned_frames": len(aligned_frames),
                "maximum_state_alignment_ms": camera_maximum_alignment_ms,
                "p99_state_alignment_ms": camera_p99_alignment_ms,
                "maximum_video_gap_ms": camera_maximum_gap_ms,
                "effective_fps": effective_fps,
                "raw_frames": len(frames),
                "repeated_source_frames": len(frames) - len(frame_ns),
                "queue_drop_fraction": drop_fraction,
            }
            maximum_state_alignment_ms = max(
                maximum_state_alignment_ms, camera_maximum_alignment_ms
            )
            p99_state_alignment_ms = max(
                p99_state_alignment_ms, camera_p99_alignment_ms
            )
            maximum_video_gap_ms = max(
                maximum_video_gap_ms, camera_maximum_gap_ms
            )
            maximum_drop_fraction = max(maximum_drop_fraction, drop_fraction)
            if effective_fps < minimum_usable_fps:
                errors.append(
                    f"{camera.video_key}: effective frame rate "
                    f"{effective_fps:.3f}fps is below minimum usable "
                    f"{minimum_usable_fps:.3f}fps"
                )
            if camera_maximum_alignment_ms > cfg.validation.maximum_state_alignment_ms:
                warnings.append(
                    f"{camera.video_key}: maximum state alignment "
                    f"{camera_maximum_alignment_ms:.3f}ms exceeds "
                    f"{cfg.validation.maximum_state_alignment_ms:.3f}ms; "
                    "state will be interpolated at video timestamps during export"
                )
            if camera_p99_alignment_ms > cfg.validation.maximum_state_alignment_p99_ms:
                warnings.append(
                    f"{camera.video_key}: p99 state alignment "
                    f"{camera_p99_alignment_ms:.3f}ms exceeds "
                    f"{cfg.validation.maximum_state_alignment_p99_ms:.3f}ms; "
                    "state will be interpolated at video timestamps during export"
                )
            if camera_maximum_gap_ms > cfg.validation.maximum_video_gap_ms:
                warnings.append(
                    f"{camera.video_key}: maximum video gap "
                    f"{camera_maximum_gap_ms:.3f}ms exceeds "
                    f"{cfg.validation.maximum_video_gap_ms:.3f}ms; "
                    "available frames will remain timestamp-aligned during export"
                )
            if drop_fraction > cfg.validation.maximum_frame_drop_fraction:
                warnings.append(
                    f"{camera.video_key}: camera queue drop fraction "
                    f"{drop_fraction:.4f} exceeds "
                    f"{cfg.validation.maximum_frame_drop_fraction:.4f}; "
                    "missing frames will be omitted during export"
                )

        frames_by_video_key = {
            video_key: stream[0] for video_key, stream in camera_streams.items()
        }
        source_indices_by_video_key = {
            video_key: stream[3] for video_key, stream in camera_streams.items()
        }
        try:
            shared_selection = shared_timeline_frame_selection(
                frames_by_video_key,
                source_indices_by_video_key,
                state_first_ns=int(state_ns[0]),
                state_last_ns=int(state_ns[-1]),
                fps=cfg.camera.fps,
            )
        except ValueError as exc:
            shared_selection = None
            errors.append(f"shared camera timeline is invalid: {exc}")
        if shared_selection is None:
            export_ns = uniform_frame_timestamps(
                overlap_start,
                overlap_end,
                cfg.camera.fps,
            )
            selected_indices = {
                video_key: source_indices_by_video_key[video_key][
                    nearest_indices(camera_timestamps[video_key], export_ns)
                ]
                for video_key in camera_timestamps
            }
            metrics["camera_timeline_mode"] = "host_timestamp_nearest"
        else:
            export_ns, selected_indices = shared_selection
            metrics["camera_timeline_mode"] = "shared_timeline_slot"
            metrics["overlap_duration_s"] = float(
                (export_ns[-1] - export_ns[0]) / 1e9
            )
        metrics["export_frames"] = len(export_ns)
        if len(export_ns) < 2:
            errors.append("fewer than two dual-camera frames share the common window")
        else:
            exterior_indices = selected_indices["exterior_rgb"]
            wrist_indices = selected_indices["wrist_rgb"]
            exterior_ns = np.asarray(
                [
                    int(
                        frames_by_video_key["exterior_rgb"][int(index)][
                            "host_monotonic_ns"
                        ]
                    )
                    for index in exterior_indices
                ],
                dtype=np.int64,
            )
            wrist_ns = np.asarray(
                [
                    int(
                        frames_by_video_key["wrist_rgb"][int(index)][
                            "host_monotonic_ns"
                        ]
                    )
                    for index in wrist_indices
                ],
                dtype=np.int64,
            )
            camera_alignment_ms = np.abs(
                exterior_ns - wrist_ns
            ) / 1e6
            camera_p99_alignment_ms = float(np.quantile(camera_alignment_ms, 0.99))
            camera_maximum_alignment_ms = float(np.max(camera_alignment_ms))
            duplicate_exterior_frames = len(exterior_indices) - len(
                np.unique(exterior_indices)
            )
            duplicate_wrist_frames = len(wrist_indices) - len(np.unique(wrist_indices))
            metrics["p99_camera_alignment_ms"] = camera_p99_alignment_ms
            metrics["maximum_camera_alignment_ms"] = camera_maximum_alignment_ms
            metrics["duplicate_exterior_frames"] = duplicate_exterior_frames
            metrics["duplicate_wrist_frames"] = duplicate_wrist_frames
            if (
                camera_p99_alignment_ms
                > cfg.validation.maximum_camera_alignment_p99_ms
            ):
                warnings.append(
                    f"D455/D405 p99 alignment {camera_p99_alignment_ms:.3f}ms "
                    f"exceeds {cfg.validation.maximum_camera_alignment_p99_ms:.3f}ms; "
                    "export will preserve the shared camera timeline"
                )
            if duplicate_wrist_frames:
                warnings.append(
                    f"{duplicate_wrist_frames} wrist frames repeat after dual-camera "
                    "timeline alignment"
                )
            if duplicate_exterior_frames:
                warnings.append(
                    f"{duplicate_exterior_frames} exterior frames repeat after "
                    "constant-rate timeline alignment"
                )
        metrics.update(
            {
                "maximum_state_alignment_ms": maximum_state_alignment_ms,
                "p99_state_alignment_ms": p99_state_alignment_ms,
                "maximum_video_gap_ms": maximum_video_gap_ms,
                "camera_queue_drop_fraction": maximum_drop_fraction,
            }
        )

    if state_summary.get("sample_count") not in (None, len(states)):
        errors.append("state summary sample count does not match state stream")

    if len(state_ns) >= 2 and len(joint_rows) == len(state_ns):
        joints = np.stack(joint_rows)
        gripper = np.asarray(gripper_rows)
        limits = manifest.get("limits")
        if isinstance(limits, dict):
            minimum = np.asarray(
                limits.get("joint_position_min_rad", []), dtype=np.float64
            )
            maximum = np.asarray(
                limits.get("joint_position_max_rad", []), dtype=np.float64
            )
            if minimum.shape == (6,) and maximum.shape == (6,):
                if np.any(joints < minimum[None, :] - 1e-4) or np.any(
                    joints > maximum[None, :] + 1e-4
                ):
                    errors.append("joint position exceeds recorded controller limits")
            gripper_max = limits.get("gripper_width_m")
            if isinstance(gripper_max, (int, float)) and math.isfinite(gripper_max):
                gripper_tolerance = limits.get(
                    "gripper_position_tolerance_m",
                    DEFAULT_GRIPPER_POSITION_TOLERANCE_M,
                )
                if not (
                    isinstance(gripper_tolerance, (int, float))
                    and math.isfinite(gripper_tolerance)
                    and gripper_tolerance >= 0.0
                ):
                    gripper_tolerance = DEFAULT_GRIPPER_POSITION_TOLERANCE_M
                if np.any(gripper < -gripper_tolerance) or np.any(
                    gripper > gripper_max + gripper_tolerance
                ):
                    errors.append("gripper position exceeds recorded controller limits")
        initial_still_s = _still_window_seconds(
            state_ns, joints, gripper, from_start=True
        )
        final_still_s = _still_window_seconds(
            state_ns, joints, gripper, from_start=False
        )
        metrics["initial_still_s"] = initial_still_s
        metrics["final_still_s"] = final_still_s
        if initial_still_s < cfg.validation.minimum_still_seconds:
            warnings.append(
                f"initial still window {initial_still_s:.3f}s is shorter than "
                f"{cfg.validation.minimum_still_seconds:.3f}s"
            )
        if final_still_s < cfg.validation.minimum_still_seconds:
            warnings.append(
                f"final still window {final_still_s:.3f}s is shorter than "
                f"{cfg.validation.minimum_still_seconds:.3f}s"
            )

    report = {
        "schema_version": 1,
        "session_id": manifest.get("session_id", path.name.split(".", 1)[0]),
        "ok": not errors,
        "errors": errors,
        "warnings": warnings,
        "metrics": metrics,
    }
    if write_report:
        atomic_json(path / "validation.json", report)
    return report


def validate_exported_dataset(dataset_path: Path | str) -> dict[str, Any]:
    """Perform dependency-light structural checks on a generated dataset."""
    path = Path(dataset_path)
    errors: list[str] = []
    try:
        info = read_json(path / "meta" / "info.json")
        modality = read_json(path / "meta" / "modality.json")
        episodes = read_jsonl(path / "meta" / "episodes.jsonl")
        tasks = read_jsonl(path / "meta" / "tasks.jsonl")
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        return {"ok": False, "errors": [f"metadata unreadable: {exc}"]}
    if info.get("codebase_version") != "v2.1":
        errors.append("dataset is not marked as LeRobot v2.1")
    if info.get("total_episodes") != len(episodes):
        errors.append("total_episodes does not match episodes.jsonl")
    if not tasks:
        errors.append("tasks.jsonl is empty")
    video_keys = tuple(modality.get("video", {}))
    if set(video_keys) != {"exterior_rgb", "wrist_rgb"}:
        errors.append("video modalities must be exterior_rgb and wrist_rgb")
    if info.get("total_videos") != len(episodes) * len(video_keys):
        errors.append("total_videos does not match episodes and camera count")
    total_frames = 0
    for episode in episodes:
        index = int(episode.get("episode_index", -1))
        length = int(episode.get("length", -1))
        total_frames += max(0, length)
        chunk = index // int(info.get("chunks_size", 1000))
        parquet = path / str(info["data_path"]).format(
            episode_chunk=chunk, episode_index=index
        )
        if not parquet.is_file() or parquet.stat().st_size == 0:
            errors.append(f"episode {index} parquet is missing")
        for video_key in video_keys:
            video = path / str(info["video_path"]).format(
                episode_chunk=chunk,
                episode_index=index,
                video_key=f"observation.images.{video_key}",
            )
            if not video.is_file() or video.stat().st_size == 0:
                errors.append(f"episode {index} {video_key} video is missing")
    if total_frames != info.get("total_frames"):
        errors.append("total_frames does not match episodes.jsonl")
    expected_groups = {"eef_9d", "gripper_position", "joint_position"}
    if set(modality.get("state", {})) != expected_groups:
        errors.append("state modality groups are not the ARX5 GR00T schema")
    if set(modality.get("action", {})) != expected_groups:
        errors.append("action modality groups are not the ARX5 GR00T schema")
    if not (path / "meta" / "stats.json").is_file():
        errors.append("meta/stats.json is missing")
    if not (path / "meta" / "relative_stats.json").is_file():
        errors.append("meta/relative_stats.json is missing")
    return {"ok": not errors, "errors": errors}
