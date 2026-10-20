from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import re
from typing import Any


CHECKPOINT_PATTERN = re.compile(r"checkpoint-(\d+)")
REQUIRED_FILES = (
    "config.json",
    "processor_config.json",
    "statistics.json",
    "model.safetensors.index.json",
    "trainer_state.json",
)
EMBODIMENT = "new_embodiment"
VIDEO_KEYS = ("exterior_rgb", "wrist_rgb")
SUPPORTED_VIDEO_KEYS = (VIDEO_KEYS, ("wrist_rgb",))
STATE_KEYS = ("eef_9d", "gripper_position", "joint_position")
CARTESIAN_KEYS = ("eef_9d", "gripper_position")
SUPPORTED_STATE_ACTION_KEYS = (STATE_KEYS, CARTESIAN_KEYS)
LANGUAGE_KEY = "annotation.human.task_description"
ACTION_HORIZON = 16


class CheckpointError(RuntimeError):
    """Raised when a fine-tuned checkpoint is absent or incompatible."""


@dataclass(frozen=True)
class FineTunedCheckpoint:
    path: Path
    step: int
    embodiment: str = EMBODIMENT
    action_horizon: int = ACTION_HORIZON
    video_keys: tuple[str, ...] = VIDEO_KEYS
    state_keys: tuple[str, ...] = STATE_KEYS
    action_keys: tuple[str, ...] = STATE_KEYS


def _read_object(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise CheckpointError(f"checkpoint file is missing: {path.name}") from exc
    except (OSError, json.JSONDecodeError) as exc:
        raise CheckpointError(f"checkpoint JSON is invalid: {path}") from exc
    if not isinstance(value, dict):
        raise CheckpointError(f"checkpoint JSON must be an object: {path}")
    return value


def _checkpoint_step(path: Path) -> int | None:
    match = CHECKPOINT_PATTERN.fullmatch(path.name)
    return None if match is None else int(match.group(1))


def _required_shards(path: Path) -> tuple[str, ...]:
    index = _read_object(path / "model.safetensors.index.json")
    weight_map = index.get("weight_map")
    if not isinstance(weight_map, dict) or not weight_map:
        raise CheckpointError("model.safetensors.index.json has no weight_map")
    shards = set()
    for value in weight_map.values():
        if not isinstance(value, str) or Path(value).name != value:
            raise CheckpointError("model shard name is invalid")
        shards.add(value)
    return tuple(sorted(shards))


def _is_complete(path: Path) -> bool:
    step = _checkpoint_step(path)
    if step is None:
        return False
    if not all((path / name).is_file() for name in REQUIRED_FILES):
        return False
    try:
        shards = _required_shards(path)
        trainer_state = _read_object(path / "trainer_state.json")
    except CheckpointError:
        return False
    return (
        trainer_state.get("global_step") == step
        and all((path / name).is_file() and (path / name).stat().st_size > 0 for name in shards)
    )


def _expect_modality(
    config: dict[str, Any],
    name: str,
    *,
    delta_indices: list[int],
    modality_keys: tuple[str, ...],
) -> dict[str, Any]:
    value = config.get(name)
    if not isinstance(value, dict):
        raise CheckpointError(f"new_embodiment.{name} is missing")
    if value.get("delta_indices") != delta_indices:
        raise CheckpointError(f"new_embodiment.{name}.delta_indices is incompatible")
    if value.get("modality_keys") != list(modality_keys):
        raise CheckpointError(f"new_embodiment.{name}.modality_keys is incompatible")
    return value


def _validate_processor(
    path: Path,
) -> tuple[tuple[str, ...], tuple[str, ...], tuple[str, ...]]:
    processor = _read_object(path / "processor_config.json")
    kwargs = processor.get("processor_kwargs")
    configs = kwargs.get("modality_configs") if isinstance(kwargs, dict) else None
    embodiment = configs.get(EMBODIMENT) if isinstance(configs, dict) else None
    if not isinstance(embodiment, dict):
        raise CheckpointError("processor does not contain new_embodiment")
    video = embodiment.get("video")
    if not isinstance(video, dict):
        raise CheckpointError("new_embodiment.video is missing")
    if video.get("delta_indices") != [0]:
        raise CheckpointError("new_embodiment.video.delta_indices is incompatible")
    raw_video_keys = video.get("modality_keys")
    if not isinstance(raw_video_keys, list) or not all(
        isinstance(key, str) for key in raw_video_keys
    ):
        raise CheckpointError("new_embodiment.video.modality_keys is incompatible")
    video_keys = tuple(raw_video_keys)
    if video_keys not in SUPPORTED_VIDEO_KEYS:
        raise CheckpointError("new_embodiment.video.modality_keys is incompatible")
    state = embodiment.get("state")
    if not isinstance(state, dict) or state.get("delta_indices") != [0]:
        raise CheckpointError("new_embodiment.state is incompatible")
    state_keys = tuple(state.get("modality_keys", ()))
    if state_keys not in SUPPORTED_STATE_ACTION_KEYS:
        raise CheckpointError("new_embodiment.state.modality_keys is incompatible")
    action = embodiment.get("action")
    if not isinstance(action, dict) or action.get("delta_indices") != list(
        range(ACTION_HORIZON)
    ):
        raise CheckpointError("new_embodiment.action is incompatible")
    action_keys = tuple(action.get("modality_keys", ()))
    if action_keys != state_keys:
        raise CheckpointError("new_embodiment.action.modality_keys is incompatible")
    expected_actions = [
        {
            "rep": "RELATIVE",
            "type": "EEF",
            "format": "XYZ_ROT6D",
            "state_key": "eef_9d",
        },
        {
            "rep": "ABSOLUTE",
            "type": "NON_EEF",
            "format": "DEFAULT",
            "state_key": "gripper_position",
        },
        {
            "rep": "RELATIVE",
            "type": "NON_EEF",
            "format": "DEFAULT",
            "state_key": "joint_position",
        },
    ]
    if action.get("action_configs") != expected_actions[: len(action_keys)]:
        raise CheckpointError("new_embodiment.action.action_configs is incompatible")
    _expect_modality(
        embodiment,
        "language",
        delta_indices=[0],
        modality_keys=(LANGUAGE_KEY,),
    )
    return video_keys, state_keys, action_keys


def validate_checkpoint(path: Path) -> FineTunedCheckpoint:
    resolved = path.expanduser().resolve()
    step = _checkpoint_step(resolved)
    if step is None:
        raise CheckpointError("checkpoint directory must be named checkpoint-N")
    if not _is_complete(resolved):
        raise CheckpointError(f"checkpoint is incomplete: {resolved}")
    video_keys, state_keys, action_keys = _validate_processor(resolved)
    statistics = _read_object(resolved / "statistics.json")
    embodiment_stats = statistics.get(EMBODIMENT)
    if not isinstance(embodiment_stats, dict):
        raise CheckpointError("statistics.json does not contain new_embodiment")
    return FineTunedCheckpoint(
        path=resolved,
        step=step,
        video_keys=video_keys,
        state_keys=state_keys,
        action_keys=action_keys,
    )


def select_checkpoint(
    training_root: Path,
    explicit: Path | None = None,
) -> FineTunedCheckpoint:
    if explicit is not None:
        candidate = explicit if explicit.is_absolute() else training_root.parent.parent / explicit
        return validate_checkpoint(candidate)
    root = training_root.expanduser().resolve()
    if not root.is_dir():
        raise CheckpointError(f"fine-tuned checkpoint root does not exist: {root}")
    candidates = [
        path
        for path in root.rglob("checkpoint-*")
        if path.is_dir() and _is_complete(path)
    ]
    if not candidates:
        raise CheckpointError(f"no complete checkpoint-N found under {root}")
    selected = max(candidates, key=lambda path: (_checkpoint_step(path) or -1, str(path)))
    return validate_checkpoint(selected)
