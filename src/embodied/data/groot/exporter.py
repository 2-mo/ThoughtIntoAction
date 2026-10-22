"""Deterministic raw-episode to GR00T-flavored LeRobot v2.1 export."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
from typing import Any

import numpy as np

from embodied.data.groot.config import CameraConfig, CollectionConfig, load_collection_config
from embodied.data.groot.geometry import pose_6d_to_eef_9d, unwrap_rpy_series
from embodied.data.groot.raw_episode import RawEpisodeStore, atomic_json, read_json
from embodied.data.groot.recording import _select_h264_encoder
from embodied.data.groot.validator import (
    nearest_indices,
    read_jsonl,
    shared_timeline_frame_selection,
    uniform_frame_timestamps,
    unique_source_frame_indices,
    validate_exported_dataset,
    validate_raw_episode,
)


STATE_NAMES = [
    "eef.x",
    "eef.y",
    "eef.z",
    "eef.rot6d.0",
    "eef.rot6d.1",
    "eef.rot6d.2",
    "eef.rot6d.3",
    "eef.rot6d.4",
    "eef.rot6d.5",
    "gripper.position_m",
    "joint.0.position_rad",
    "joint.1.position_rad",
    "joint.2.position_rad",
    "joint.3.position_rad",
    "joint.4.position_rad",
    "joint.5.position_rad",
]
GRIPPER_POSITION_INDEX = STATE_NAMES.index("gripper.position_m")


def _initial_arm_motion_trim_index(
    state: np.ndarray,
    *,
    fps: int,
    lead_seconds: float,
) -> int:
    values = np.asarray(state, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != len(STATE_NAMES):
        raise ValueError("state must have shape (frames, 16)")
    if fps < 1 or not math.isfinite(lead_seconds) or lead_seconds < 0.0:
        raise ValueError("trim FPS and lead seconds must be valid")
    translation = np.linalg.norm(values[:, :3] - values[0, :3], axis=1)
    joint_delta = np.max(np.abs(values[:, 10:] - values[0, 10:]), axis=1)
    moving = np.flatnonzero(
        (translation > 0.001) | (joint_delta > math.radians(0.5))
    )
    if not len(moving):
        return 0
    lead_frames = int(round(lead_seconds * fps))
    return max(0, int(moving[0]) - lead_frames)


def _collapse_internal_static_frames(
    state: np.ndarray,
    *,
    fps: int,
    maximum_static_seconds: float,
) -> np.ndarray:
    """Keep short holds, but remove long identical pose holds inside an episode."""
    values = np.asarray(state, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != len(STATE_NAMES):
        raise ValueError("state must have shape (frames, 16)")
    if fps < 1 or not math.isfinite(maximum_static_seconds) or maximum_static_seconds < 0.0:
        raise ValueError("static-collapse FPS and duration must be valid")
    if len(values) < 2 or maximum_static_seconds == 0.0:
        return np.arange(len(values), dtype=np.int64)
    maximum_frames = max(1, int(round(maximum_static_seconds * fps)))
    delta_xyz = np.linalg.norm(np.diff(values[:, :3], axis=0), axis=1)
    delta_joints = np.max(np.abs(np.diff(values[:, 10:], axis=0)), axis=1)
    delta_gripper = np.abs(np.diff(values[:, GRIPPER_POSITION_INDEX]))
    static_transition = (
        (delta_xyz < 0.0005)
        & (delta_joints < math.radians(0.3))
        & (delta_gripper < 0.0005)
    )
    keep = np.ones(len(values), dtype=bool)
    index = 0
    while index < len(static_transition):
        if not static_transition[index]:
            index += 1
            continue
        end = index
        while end < len(static_transition) and static_transition[end]:
            end += 1
        run_frames = end - index + 1
        if run_frames > maximum_frames + 1:
            keep[index + maximum_frames : end] = False
        index = end
    return np.flatnonzero(keep)


def _write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        for row in rows:
            stream.write(json.dumps(row, ensure_ascii=False, separators=(",", ":")))
            stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)


def _interpolate_state(
    states: list[dict[str, Any]],
    frame_ns: np.ndarray,
) -> np.ndarray:
    state_ns = np.asarray(
        [int(row["host_monotonic_ns"]) for row in states], dtype=np.int64
    )
    joints = np.asarray([row["joint_position"] for row in states], dtype=np.float64)
    gripper = np.asarray(
        [row["gripper_position"] for row in states], dtype=np.float64
    )[:, None]
    poses = np.asarray([row["eef_pose_6d"] for row in states], dtype=np.float64)
    poses[:, 3:] = unwrap_rpy_series(poses[:, 3:])
    source_s = (state_ns - state_ns[0]).astype(np.float64) / 1e9
    target_s = (frame_ns - state_ns[0]).astype(np.float64) / 1e9

    def interpolate(matrix: np.ndarray) -> np.ndarray:
        return np.column_stack(
            [
                np.interp(target_s, source_s, matrix[:, column])
                for column in range(matrix.shape[1])
            ]
        )

    pose_6d = interpolate(poses)
    return np.concatenate(
        (
            pose_6d_to_eef_9d(pose_6d),
            interpolate(gripper),
            interpolate(joints),
        ),
        axis=1,
    ).astype(np.float32)


def _build_absolute_action(
    state: np.ndarray,
    *,
    gripper_width_m: float,
) -> tuple[np.ndarray, dict[str, Any]]:
    if not np.isfinite(gripper_width_m) or gripper_width_m <= 0.0:
        raise ValueError("gripper_width_m must be positive and finite")
    action = np.concatenate((state[1:], state[-1:]), axis=0).copy()
    raw_gripper = action[:, GRIPPER_POSITION_INDEX].copy()
    action[:, GRIPPER_POSITION_INDEX] = np.clip(
        raw_gripper,
        0.0,
        gripper_width_m,
    )
    clipped_low = int(np.count_nonzero(raw_gripper < 0.0))
    clipped_high = int(np.count_nonzero(raw_gripper > gripper_width_m))
    return action, {
        "control_range_m": [0.0, gripper_width_m],
        "raw_action_min_m": float(np.min(raw_gripper)),
        "raw_action_max_m": float(np.max(raw_gripper)),
        "clipped_low_frames": clipped_low,
        "clipped_high_frames": clipped_high,
        "clipped_frames": clipped_low + clipped_high,
    }


def _encode_video(
    camera_path: Path,
    source_indices: np.ndarray,
    output_path: Path,
    camera: CameraConfig,
) -> str:
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError("OpenCV is required to export camera video") from exc
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        raise RuntimeError("ffmpeg is required to export camera video")
    encoder = _select_h264_encoder(ffmpeg)
    encoder_options = (
        [
            "-c:v",
            "h264_nvenc",
            "-preset",
            "p4",
            "-tune",
            "hq",
            "-rc",
            "constqp",
            "-qp",
            "18",
        ]
        if encoder == "h264_nvenc"
        else [
            "-c:v",
            "libx264",
            "-preset",
            "medium",
            "-crf",
            "18",
        ]
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    command = [
        ffmpeg,
        "-hide_banner",
        "-loglevel",
        "error",
        "-y",
        "-f",
        "rawvideo",
        "-pix_fmt",
        "rgb24",
        "-s:v",
        f"{camera.width}x{camera.height}",
        "-r",
        str(camera.fps),
        "-i",
        "-",
        "-an",
        *encoder_options,
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        str(output_path),
    ]
    process = subprocess.Popen(
        command,
        stdin=subprocess.PIPE,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
    )
    assert process.stdin is not None
    capture = cv2.VideoCapture(str(camera_path / "video.mkv"))
    if not capture.isOpened():
        process.kill()
        process.wait()
        raise RuntimeError(f"failed to open raw camera video: {camera_path}")
    current_index = -1
    bgr: np.ndarray | None = None
    try:
        for requested_index in source_indices:
            target_index = int(requested_index)
            if target_index < current_index:
                raise RuntimeError("camera source indices are not monotonic")
            while current_index < target_index:
                ok, next_bgr = capture.read()
                if not ok:
                    raise RuntimeError(
                        f"raw camera video ended before frame {target_index}: "
                        f"{camera_path}"
                    )
                bgr = next_bgr
                current_index += 1
            if bgr is None:
                raise RuntimeError(f"raw camera video has no frames: {camera_path}")
            if bgr.shape != (
                camera.height,
                camera.width,
                3,
            ):
                raise RuntimeError(
                    f"raw frame has unexpected shape {bgr.shape}: {camera_path}"
                )
            process.stdin.write(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).tobytes())
        process.stdin.close()
        return_code = process.wait()
    except BaseException:
        process.kill()
        process.wait()
        raise
    finally:
        capture.release()
    if return_code:
        error = process.stderr.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"ffmpeg failed ({return_code}): {error.strip()}")
    return encoder


def _write_parquet(
    output_path: Path,
    state: np.ndarray,
    action: np.ndarray,
    *,
    episode_index: int,
    global_start_index: int,
    fps: int,
) -> None:
    try:
        import pyarrow as pa
        import pyarrow.parquet as pq
    except ImportError as exc:
        raise RuntimeError("pyarrow is required to export LeRobot parquet") from exc
    length = len(state)
    list_type = pa.list_(pa.float32(), list_size=state.shape[1])
    arrays = {
        "observation.state": pa.array(state.tolist(), type=list_type),
        "action": pa.array(action.tolist(), type=list_type),
        "timestamp": pa.array(
            np.arange(length, dtype=np.float32) / np.float32(fps),
            type=pa.float32(),
        ),
        "frame_index": pa.array(np.arange(length), type=pa.int64()),
        "episode_index": pa.array(
            np.full(length, episode_index, dtype=np.int64), type=pa.int64()
        ),
        "index": pa.array(
            np.arange(global_start_index, global_start_index + length),
            type=pa.int64(),
        ),
        "task_index": pa.array(np.zeros(length, dtype=np.int64), type=pa.int64()),
        "next.reward": pa.array(np.zeros(length, dtype=np.float32), type=pa.float32()),
        "next.done": pa.array(
            np.arange(length, dtype=np.int64) == length - 1, type=pa.bool_()
        ),
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    pq.write_table(
        pa.table(arrays),
        output_path,
        compression="zstd",
        use_dictionary=False,
        write_statistics=True,
    )


def _dataset_metadata(
    config: CollectionConfig,
    total_episodes: int,
    total_frames: int,
) -> tuple[dict[str, Any], dict[str, Any]]:
    features: dict[str, Any] = {
        "observation.state": {
            "dtype": "float32",
            "shape": [16],
            "names": STATE_NAMES,
        },
        "action": {
            "dtype": "float32",
            "shape": [16],
            "names": STATE_NAMES,
        },
        "timestamp": {"dtype": "float32", "shape": [1], "names": None},
        "frame_index": {"dtype": "int64", "shape": [1], "names": None},
        "episode_index": {"dtype": "int64", "shape": [1], "names": None},
        "index": {"dtype": "int64", "shape": [1], "names": None},
        "task_index": {"dtype": "int64", "shape": [1], "names": None},
        "next.reward": {"dtype": "float32", "shape": [1], "names": None},
        "next.done": {"dtype": "bool", "shape": [1], "names": None},
    }
    for camera in config.cameras:
        features[f"observation.images.{camera.video_key}"] = {
            "dtype": "video",
            "shape": [camera.height, camera.width, 3],
            "names": ["height", "width", "channels"],
            "info": {
                "video.height": camera.height,
                "video.width": camera.width,
                "video.codec": "h264",
                "video.pix_fmt": "yuv420p",
                "video.is_depth_map": False,
                "video.fps": camera.fps,
                "video.channels": 3,
                "has_audio": False,
            },
        }
    info = {
        "codebase_version": "v2.1",
        "robot_type": "arx5",
        "total_episodes": total_episodes,
        "total_frames": total_frames,
        "total_tasks": 1,
        "total_videos": total_episodes * len(config.cameras),
        "total_chunks": (total_episodes + 999) // 1000,
        "chunks_size": 1000,
        "fps": config.camera.fps,
        "splits": {"train": f"0:{total_episodes}"},
        "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
        "video_path": (
            "videos/chunk-{episode_chunk:03d}/{video_key}/"
            "episode_{episode_index:06d}.mp4"
        ),
        "features": features,
    }
    modality = {
        "state": {
            "eef_9d": {"start": 0, "end": 9},
            "gripper_position": {"start": 9, "end": 10},
            "joint_position": {"start": 10, "end": 16},
        },
        "action": {
            "eef_9d": {"start": 0, "end": 9},
            "gripper_position": {"start": 9, "end": 10},
            "joint_position": {"start": 10, "end": 16},
        },
        "video": {
            camera.video_key: {
                "original_key": f"observation.images.{camera.video_key}"
            }
            for camera in config.cameras
        },
        "annotation": {
            "human.task_description": {"original_key": "task_index"},
        },
    }
    return info, modality


def find_groot_root(start: Path | str) -> Path | None:
    """Resolve the explicit installed environment, without scanning sibling projects."""
    from embodied.core.dependencies import dependency_python, dependency_root
    return dependency_root("groot") if dependency_python("groot").is_file() else None


def _groot_python(root: Path) -> Path:
    from embodied.core.dependencies import dependency_python, dependency_root, reject_reference_path
    root = reject_reference_path(root)
    python = (dependency_python("groot") if root == dependency_root("groot")
              else root / ".venv/bin/python")
    if not python.is_file():
        raise RuntimeError(f"GR00T environment missing: {python}; run ./run.sh components setup groot")
    return python


def _groot_environment(root: Path) -> dict[str, str]:
    environment = os.environ.copy()
    environment.pop("PYTHONPATH", None)
    environment.pop("PYTHONHOME", None)
    return environment


def run_groot_stats(
    dataset_path: Path | str,
    *,
    groot_root: Path | str,
) -> None:
    root = Path(groot_root).resolve()
    python = _groot_python(root)
    if not python.is_file():
        raise RuntimeError(f"Isaac-GR00T Python environment not found: {python}")
    config_path = Path(__file__).with_name("modality_config.py").resolve()
    subprocess.run(
        [
            str(python),
            "-m", "gr00t.data.stats",
            "--dataset-path",
            str(Path(dataset_path).resolve()),
            "--embodiment-tag",
            "NEW_EMBODIMENT",
            "--modality-config-path",
            str(config_path),
        ],
        cwd=Path(dataset_path).resolve(),
        env=_groot_environment(root),
        check=True,
    )


def validate_with_groot(
    dataset_path: Path | str,
    *,
    groot_root: Path | str,
) -> None:
    root = Path(groot_root).resolve()
    python = _groot_python(root)
    config_path = Path(__file__).with_name("modality_config.py").resolve()
    script = """
import importlib.util
from pathlib import Path
import sys
dataset, config_path = sys.argv[1:3]
spec = importlib.util.spec_from_file_location("arx5_modality_config", config_path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
from gr00t.configs.data.embodiment_configs import MODALITY_CONFIGS
from gr00t.data.dataset.lerobot_episode_loader import LeRobotEpisodeLoader
loader = LeRobotEpisodeLoader(
    Path(dataset), MODALITY_CONFIGS["new_embodiment"]
)
assert len(loader) > 0
episode = loader[0]
required = {
    "video.exterior_rgb", "video.wrist_rgb",
    "state.eef_9d", "state.gripper_position", "state.joint_position",
    "action.eef_9d", "action.gripper_position", "action.joint_position",
}
missing = required.difference(episode.columns)
assert not missing, missing
print(f"GR00T loader accepted {len(loader)} episodes; first usable episode has {len(episode)} rows")
"""
    subprocess.run(
        [
            str(python),
            "-c",
            script,
            str(Path(dataset_path).resolve()),
            str(config_path),
        ],
        cwd=Path(dataset_path).resolve(),
        env=_groot_environment(root),
        check=True,
    )


def export_dataset(
    raw_root: Path | str,
    output_path: Path | str,
    *,
    config: CollectionConfig | None = None,
    groot_root: Path | str | None = None,
    generate_stats: bool = True,
    allow_incomplete: bool = False,
    initial_arm_motion_lead_s: float | None = None,
    maximum_internal_static_seconds: float | None = None,
) -> dict[str, Any]:
    """Export every accepted episode, publishing only a fully valid dataset."""
    cfg = config or load_collection_config()
    if initial_arm_motion_lead_s is not None and (
        not math.isfinite(initial_arm_motion_lead_s)
        or initial_arm_motion_lead_s < 0.0
    ):
        raise ValueError("initial_arm_motion_lead_s must be finite and non-negative")
    if maximum_internal_static_seconds is not None and (
        not math.isfinite(maximum_internal_static_seconds)
        or maximum_internal_static_seconds < 0.0
    ):
        raise ValueError("maximum_internal_static_seconds must be finite and non-negative")
    raw = Path(raw_root).resolve()
    output = Path(output_path).resolve()
    episodes = RawEpisodeStore(raw, cfg).paths_for_stage("complete")
    if not episodes:
        raise RuntimeError(f"no accepted raw episodes found under {raw}")
    if (
        not allow_incomplete
        and len(episodes) != cfg.target_accepted_episodes
    ):
        raise RuntimeError(
            f"expected exactly {cfg.target_accepted_episodes} accepted episodes, "
            f"found {len(episodes)}; finish collection or pass --allow-incomplete "
            "for a deliberate test export"
        )
    if output.exists():
        raise FileExistsError(
            f"output already exists: {output}; choose a new path to preserve prior exports"
        )
    output.parent.mkdir(parents=True, exist_ok=True)
    build = Path(
        tempfile.mkdtemp(prefix=f".{output.name}.building-", dir=output.parent)
    )
    episode_rows: list[dict[str, Any]] = []
    export_rows: list[dict[str, Any]] = []
    total_frames = 0
    try:
        for episode_index, episode_path in enumerate(episodes):
            report = validate_raw_episode(episode_path, cfg)
            if not report["ok"]:
                joined = "; ".join(report["errors"])
                raise RuntimeError(f"{episode_path.name} failed validation: {joined}")
            states = read_jsonl(episode_path / "state" / "states.jsonl")
            camera_frames = {
                camera.video_key: read_jsonl(
                    episode_path
                    / "cameras"
                    / camera.video_key
                    / "frames.jsonl"
                )
                for camera in cfg.cameras
            }
            camera_source_indices = {
                video_key: unique_source_frame_indices(frames)
                for video_key, frames in camera_frames.items()
            }
            camera_ns = {
                video_key: np.asarray(
                    [
                        int(frames[index]["host_monotonic_ns"])
                        for index in camera_source_indices[video_key]
                    ],
                    dtype=np.int64,
                )
                for video_key, frames in camera_frames.items()
            }
            state_first = int(states[0]["host_monotonic_ns"])
            state_last = int(states[-1]["host_monotonic_ns"])
            shared_selection = shared_timeline_frame_selection(
                camera_frames,
                camera_source_indices,
                state_first_ns=state_first,
                state_last_ns=state_last,
                fps=cfg.camera.fps,
            )
            if shared_selection is None:
                overlap_start = max(
                    state_first,
                    *(int(timestamps[0]) for timestamps in camera_ns.values()),
                )
                overlap_end = min(
                    state_last,
                    *(int(timestamps[-1]) for timestamps in camera_ns.values()),
                )
                frame_ns = uniform_frame_timestamps(
                    overlap_start,
                    overlap_end,
                    cfg.camera.fps,
                )
                source_indices = {
                    video_key: camera_source_indices[video_key][
                        nearest_indices(camera_ns[video_key], frame_ns)
                    ]
                    for video_key in camera_ns
                }
                timeline_mode = "host_timestamp_nearest"
            else:
                frame_ns, source_indices = shared_selection
                timeline_mode = "shared_timeline_slot"
            state = _interpolate_state(states, frame_ns)
            source_frame_count = len(frame_ns)
            trim_start_index = 0
            if initial_arm_motion_lead_s is not None:
                trim_start_index = _initial_arm_motion_trim_index(
                    state,
                    fps=cfg.camera.fps,
                    lead_seconds=initial_arm_motion_lead_s,
                )
                frame_ns = frame_ns[trim_start_index:]
                state = state[trim_start_index:]
                source_indices = {
                    video_key: indices[trim_start_index:]
                    for video_key, indices in source_indices.items()
                }
            collapsed_static_frames = 0
            if maximum_internal_static_seconds is not None:
                keep_indices = _collapse_internal_static_frames(
                    state,
                    fps=cfg.camera.fps,
                    maximum_static_seconds=maximum_internal_static_seconds,
                )
                collapsed_static_frames = len(state) - len(keep_indices)
                frame_ns = frame_ns[keep_indices]
                state = state[keep_indices]
                source_indices = {
                    video_key: indices[keep_indices]
                    for video_key, indices in source_indices.items()
                }
            manifest = read_json(episode_path / "episode.json")
            limits = manifest.get("limits")
            gripper_width_m = (
                limits.get("gripper_width_m") if isinstance(limits, dict) else None
            )
            if not isinstance(gripper_width_m, (int, float)):
                raise RuntimeError(
                    f"{episode_path.name} has no recorded gripper control range"
                )
            action, gripper_action = _build_absolute_action(
                state,
                gripper_width_m=float(gripper_width_m),
            )
            chunk = episode_index // 1000
            parquet_path = (
                build
                / "data"
                / f"chunk-{chunk:03d}"
                / f"episode_{episode_index:06d}.parquet"
            )
            _write_parquet(
                parquet_path,
                state,
                action,
                episode_index=episode_index,
                global_start_index=total_frames,
                fps=cfg.camera.fps,
            )
            video_encoders: dict[str, str] = {}
            for camera in cfg.cameras:
                video_path = (
                    build
                    / "videos"
                    / f"chunk-{chunk:03d}"
                    / f"observation.images.{camera.video_key}"
                    / f"episode_{episode_index:06d}.mp4"
                )
                video_encoders[camera.video_key] = _encode_video(
                    episode_path / "cameras" / camera.video_key,
                    source_indices[camera.video_key],
                    video_path,
                    camera,
                )
            episode_rows.append(
                {
                    "episode_index": episode_index,
                    "tasks": [cfg.task_instruction],
                    "length": len(frame_ns),
                }
            )
            export_rows.append(
                {
                    "episode_index": episode_index,
                    "session_id": manifest.get("session_id", episode_path.stem),
                    "frames": len(frame_ns),
                    "source_frames": source_frame_count,
                    "trimmed_leading_frames": trim_start_index,
                    "collapsed_static_frames": collapsed_static_frames,
                    "maximum_internal_static_seconds": maximum_internal_static_seconds,
                    "initial_arm_motion_lead_s": initial_arm_motion_lead_s,
                    "video_keys": [camera.video_key for camera in cfg.cameras],
                    "video_encoders": video_encoders,
                    "p99_camera_alignment_ms": report["metrics"].get(
                        "p99_camera_alignment_ms"
                    ),
                    "camera_timeline_mode": timeline_mode,
                    "resampled_repeated_frames": {
                        video_key: len(indices) - len(np.unique(indices))
                        for video_key, indices in source_indices.items()
                    },
                    "gripper_action": gripper_action,
                }
            )
            total_frames += len(frame_ns)

        info, modality = _dataset_metadata(cfg, len(episodes), total_frames)
        atomic_json(build / "meta" / "info.json", info)
        atomic_json(build / "meta" / "modality.json", modality)
        _write_jsonl(build / "meta" / "episodes.jsonl", episode_rows)
        _write_jsonl(
            build / "meta" / "tasks.jsonl",
            [{"task_index": 0, "task": cfg.task_instruction}],
        )
        atomic_json(
            build / "meta" / "export.json",
            {
                "schema_version": 1,
                "dataset_id": cfg.dataset_id,
                "batch_id": cfg.batch_id,
                "episodes": export_rows,
                "state_layout": STATE_NAMES,
                "action_definition": (
                    "next synchronized state; final frame repeats; absolute gripper "
                    "position is clipped to the recorded controller range"
                ),
            },
        )
        if generate_stats:
            root = Path(groot_root) if groot_root else find_groot_root(Path(__file__))
            if root is None:
                raise RuntimeError(
                    "Isaac-GR00T checkout not found; pass --groot-root to generate stats"
                )
            run_groot_stats(build, groot_root=root)
            validate_with_groot(build, groot_root=root)
        structural = validate_exported_dataset(build)
        if not structural["ok"]:
            raise RuntimeError(
                "export validation failed: " + "; ".join(structural["errors"])
            )
        build.replace(output)
    except BaseException:
        shutil.rmtree(build, ignore_errors=True)
        raise
    return {
        "ok": True,
        "dataset_path": str(output),
        "episodes": len(episodes),
        "frames": total_frames,
    }
