"""Configuration for ARX5 Isaac-GR00T data collection."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import re
from typing import Any


DEFAULT_CONFIG_PATH = Path(__file__).with_name("config.json")


@dataclass(frozen=True)
class CameraConfig:
    video_key: str
    width: int
    height: int
    fps: int
    video_crf: int
    exposure_us: int
    gain: int
    white_balance_k: int


@dataclass(frozen=True)
class SamplingConfig:
    arm_rate_hz: float
    minimum_episode_seconds: float
    maximum_episode_seconds: float
    action_horizon: int


@dataclass(frozen=True)
class ValidationConfig:
    maximum_state_alignment_ms: float
    maximum_state_alignment_p99_ms: float
    maximum_camera_alignment_p99_ms: float
    maximum_video_gap_ms: float
    maximum_frame_drop_fraction: float
    minimum_still_seconds: float


@dataclass(frozen=True)
class CollectionConfig:
    dataset_id: str
    batch_id: str
    task_instruction: str
    task_display: str
    source_color: str
    target_color: str
    cube_edge_m: float
    target_accepted_episodes: int
    cameras: tuple[CameraConfig, ...]
    sampling: SamplingConfig
    validation: ValidationConfig

    def camera_for(self, video_key: str) -> CameraConfig:
        for camera in self.cameras:
            if camera.video_key == video_key:
                return camera
        raise KeyError(video_key)

    @property
    def camera(self) -> CameraConfig:
        return self.camera_for("exterior_rgb")

    @property
    def wrist_camera(self) -> CameraConfig:
        return self.camera_for("wrist_rgb")


def _positive_number(value: Any, name: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise ValueError(f"{name} must be positive and finite")
    return parsed


def load_collection_config(path: Path | str = DEFAULT_CONFIG_PATH) -> CollectionConfig:
    config_path = Path(path)
    with config_path.open("r", encoding="utf-8") as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        raise ValueError("collection config must be a JSON object")
    cameras = value.get("cameras")
    sampling = value.get("sampling")
    validation = value.get("validation")
    if not all(isinstance(item, dict) for item in (cameras, sampling, validation)):
        raise ValueError("cameras, sampling, and validation config must be objects")
    if set(cameras) != {"exterior_rgb", "wrist_rgb"}:
        raise ValueError("cameras must define exterior_rgb and wrist_rgb")
    if not all(isinstance(cameras[video_key], dict) for video_key in cameras):
        raise ValueError("each camera config must be an object")
    parsed_cameras = tuple(
        CameraConfig(
            video_key=video_key,
            width=int(cameras[video_key]["width"]),
            height=int(cameras[video_key]["height"]),
            fps=int(cameras[video_key]["fps"]),
            video_crf=int(cameras[video_key]["video_crf"]),
            exposure_us=int(cameras[video_key].get("exposure_us", 8000)),
            gain=int(
                cameras[video_key].get(
                    "gain",
                    64 if video_key == "exterior_rgb" else 16,
                )
            ),
            white_balance_k=int(cameras[video_key].get("white_balance_k", 4500)),
        )
        for video_key in ("exterior_rgb", "wrist_rgb")
    )
    result = CollectionConfig(
        dataset_id=str(value["dataset_id"]).strip(),
        batch_id=str(value["batch_id"]).strip(),
        task_instruction=str(value["task_instruction"]).strip(),
        task_display=str(value["task_display"]).strip(),
        source_color=str(value["source_color"]).strip().lower(),
        target_color=str(value["target_color"]).strip().lower(),
        cube_edge_m=_positive_number(value["cube_edge_m"], "cube_edge_m"),
        target_accepted_episodes=int(value["target_accepted_episodes"]),
        cameras=parsed_cameras,
        sampling=SamplingConfig(
            arm_rate_hz=_positive_number(
                sampling["arm_rate_hz"], "sampling.arm_rate_hz"
            ),
            minimum_episode_seconds=_positive_number(
                sampling["minimum_episode_seconds"],
                "sampling.minimum_episode_seconds",
            ),
            maximum_episode_seconds=_positive_number(
                sampling["maximum_episode_seconds"],
                "sampling.maximum_episode_seconds",
            ),
            action_horizon=int(sampling["action_horizon"]),
        ),
        validation=ValidationConfig(
            maximum_state_alignment_ms=_positive_number(
                validation["maximum_state_alignment_ms"],
                "validation.maximum_state_alignment_ms",
            ),
            maximum_state_alignment_p99_ms=_positive_number(
                validation["maximum_state_alignment_p99_ms"],
                "validation.maximum_state_alignment_p99_ms",
            ),
            maximum_camera_alignment_p99_ms=_positive_number(
                validation["maximum_camera_alignment_p99_ms"],
                "validation.maximum_camera_alignment_p99_ms",
            ),
            maximum_video_gap_ms=_positive_number(
                validation["maximum_video_gap_ms"],
                "validation.maximum_video_gap_ms",
            ),
            maximum_frame_drop_fraction=float(
                validation["maximum_frame_drop_fraction"]
            ),
            minimum_still_seconds=_positive_number(
                validation["minimum_still_seconds"],
                "validation.minimum_still_seconds",
            ),
        ),
    )
    if not result.dataset_id or not result.task_instruction or not result.task_display:
        raise ValueError("dataset and task strings must not be empty")
    if not re.fullmatch(r"\d{8}-\d{6}[a-z0-9_-]*", result.batch_id):
        raise ValueError("batch_id must start with YYYYMMDD-HHMMSS")
    if result.source_color == result.target_color:
        raise ValueError("source_color and target_color must differ")
    if result.target_accepted_episodes < 1:
        raise ValueError("target_accepted_episodes must be positive")
    for camera in result.cameras:
        if (
            camera.width < 1
            or camera.height < 1
            or camera.fps < 1
            or not 0 <= camera.video_crf <= 51
            or camera.exposure_us <= 0
            or camera.gain < 0
            or camera.white_balance_k <= 0
        ):
            raise ValueError(
                f"{camera.video_key} dimensions, fps, video CRF, or color controls are invalid"
            )
    if len({camera.fps for camera in result.cameras}) != 1:
        raise ValueError("all cameras must use the same FPS")
    if (
        result.sampling.maximum_episode_seconds
        <= result.sampling.minimum_episode_seconds
    ):
        raise ValueError("maximum episode duration must exceed minimum duration")
    if result.sampling.action_horizon < 1:
        raise ValueError("action_horizon must be positive")
    if not 0.0 <= result.validation.maximum_frame_drop_fraction < 1.0:
        raise ValueError("maximum_frame_drop_fraction must be in [0, 1)")
    return result
