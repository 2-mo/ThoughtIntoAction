"""Crash-tolerant raw episode directory management."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
from typing import Any
from uuid import uuid4

from arx5_paths import DATASETS_ROOT

from .config import CollectionConfig, load_collection_config


DEFAULT_RAW_ROOT = DATASETS_ROOT / "data-coll"
BATCHED_RAW_ROOT = DATASETS_ROOT / "data-coll-batches"
SESSION_ID_RE = re.compile(r"^[a-z0-9][a-z0-9_-]{7,63}$")
STAGE_DIRECTORIES = {
    "inprogress": "in-progress",
    "complete": "success",
    "rejected": "discarded",
}


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def validate_session_id(value: str) -> str:
    session_id = str(value)
    if not SESSION_ID_RE.fullmatch(session_id):
        raise ValueError("invalid collection session id")
    return session_id


def atomic_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(payload, stream, ensure_ascii=False, indent=2)
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)


def read_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


@dataclass(frozen=True)
class EpisodeLocation:
    session_id: str
    path: Path
    stage: str


class RawEpisodeStore:
    def __init__(
        self,
        root: Path | str | None = None,
        config: CollectionConfig | None = None,
    ) -> None:
        self.config = config or load_collection_config()
        self.root = (
            BATCHED_RAW_ROOT / self.config.batch_id
            if root is None
            else Path(root)
        )

    def new_session_id(self) -> str:
        stamp = datetime.now().astimezone().strftime("%Y%m%d-%H%M%S-%f")
        return f"{stamp}_id-{uuid4().hex[:8]}"

    def stage_root(self, stage: str) -> Path:
        try:
            directory = STAGE_DIRECTORIES[stage]
        except KeyError as exc:
            raise ValueError("unsupported episode stage") from exc
        return self.root / directory

    def paths_for_stage(self, stage: str) -> list[Path]:
        stage_root = self.stage_root(stage)
        if not stage_root.is_dir():
            return []
        return sorted(path for path in stage_root.iterdir() if path.is_dir())

    def _ensure_layout(self) -> None:
        self.root.mkdir(parents=True, exist_ok=True)
        batch_manifest_path = self.root / "batch.json"
        expected_batch = {
            "schema_version": 1,
            "batch_id": self.config.batch_id,
            "dataset_id": self.config.dataset_id,
        }
        if batch_manifest_path.is_file():
            recorded_batch = read_json(batch_manifest_path)
            if any(
                recorded_batch.get(key) != value
                for key, value in expected_batch.items()
            ):
                raise RuntimeError("collection batch directory metadata does not match")
        else:
            atomic_json(
                batch_manifest_path,
                {**expected_batch, "created_at": utc_now()},
            )
        for stage in STAGE_DIRECTORIES:
            self.stage_root(stage).mkdir(parents=True, exist_ok=True)

    def path_for(self, session_id: str, stage: str = "inprogress") -> Path:
        validate_session_id(session_id)
        return self.stage_root(stage) / session_id

    def locate(self, session_id: str) -> EpisodeLocation:
        validate_session_id(session_id)
        matches = [
            EpisodeLocation(session_id, self.path_for(session_id, stage), stage)
            for stage in ("inprogress", "complete", "rejected")
            if self.path_for(session_id, stage).is_dir()
        ]
        if len(matches) != 1:
            raise FileNotFoundError(f"raw episode not uniquely found: {session_id}")
        return matches[0]

    def begin(self) -> EpisodeLocation:
        self._ensure_layout()
        session_id = self.new_session_id()
        path = self.path_for(session_id)
        path.mkdir(mode=0o700)
        (path / "state").mkdir()
        cameras_path = path / "cameras"
        cameras_path.mkdir()
        for camera in self.config.cameras:
            (cameras_path / camera.video_key).mkdir()
        atomic_json(
            path / "episode.json",
            {
                "schema_version": 1,
                "session_id": session_id,
                "stage": "recording",
                "decision": None,
                "created_at": utc_now(),
                "dataset_id": self.config.dataset_id,
                "batch_id": self.config.batch_id,
                "task_instruction": self.config.task_instruction,
                "task_display": self.config.task_display,
                "source_color": self.config.source_color,
                "target_color": self.config.target_color,
                "cube_edge_m": self.config.cube_edge_m,
                "recording_spec": {
                    "arm_rate_hz": self.config.sampling.arm_rate_hz,
                    "camera_timeline": {
                        "mode": "native_rate_shared_guard",
                        "fps": self.config.camera.fps,
                        "master_video_key": "wrist_rgb",
                    },
                    "cameras": {
                        camera.video_key: {
                            "width": camera.width,
                            "height": camera.height,
                            "fps": camera.fps,
                            "source_pixel_format": "rgb8",
                            "source_bit_depth": 8,
                            "color_controls": {
                                "mode": "auto_warmup_then_fixed",
                                "warmup_frames": 30,
                                "lock_before_recording": True,
                                "exposure_us": camera.exposure_us,
                                "gain": camera.gain,
                                "white_balance_k": camera.white_balance_k,
                            },
                            "storage": {
                                "container": "matroska",
                                "codec": "h264",
                                "pixel_format": "yuv444p",
                                "crf": camera.video_crf,
                            },
                        }
                        for camera in self.config.cameras
                    },
                },
            },
        )
        return EpisodeLocation(session_id, path, "inprogress")

    def update(self, session_id: str, **changes: Any) -> dict[str, Any]:
        location = self.locate(session_id)
        manifest_path = location.path / "episode.json"
        payload = read_json(manifest_path)
        payload.update(changes)
        payload["updated_at"] = utc_now()
        atomic_json(manifest_path, payload)
        return payload

    def finish_recording(
        self, session_id: str, state_summary: dict[str, Any]
    ) -> dict[str, Any]:
        return self.update(
            session_id,
            stage="pending_review",
            stopped_at=utc_now(),
            state=state_summary,
        )

    def accept(
        self, session_id: str, validation: dict[str, Any]
    ) -> EpisodeLocation:
        location = self.locate(session_id)
        if location.stage != "inprogress":
            raise ValueError("only pending episodes can be accepted")
        self.update(
            session_id,
            stage="accepted",
            decision="accepted",
            decision_at=utc_now(),
            validation=validation,
        )
        target = self.path_for(session_id, "complete")
        target.parent.mkdir(parents=True, exist_ok=True)
        location.path.replace(target)
        return EpisodeLocation(session_id, target, "complete")

    def reject(self, session_id: str, reason: str) -> EpisodeLocation:
        location = self.locate(session_id)
        if location.stage != "inprogress":
            raise ValueError("only pending episodes can be rejected")
        for camera in self.config.cameras:
            camera_manifest = (
                location.path / "cameras" / camera.video_key / "camera.json"
            )
            if (
                camera_manifest.is_file()
                and read_json(camera_manifest).get("stage") == "recording"
            ):
                raise RuntimeError("相机录制仍在写入，请稍后重试")
        reason = str(reason).strip() or "operator redo"
        self.update(
            session_id,
            stage="rejected",
            decision="rejected",
            decision_reason=reason,
            decision_at=utc_now(),
        )
        target = self.path_for(session_id, "rejected")
        target.parent.mkdir(parents=True, exist_ok=True)
        location.path.replace(target)
        return EpisodeLocation(session_id, target, "rejected")

    def counts(self) -> dict[str, int]:
        self._ensure_layout()
        return {
            "accepted": len(self.paths_for_stage("complete")),
            "rejected": len(self.paths_for_stage("rejected")),
            "inprogress": len(self.paths_for_stage("inprogress")),
            "target": self.config.target_accepted_episodes,
        }
