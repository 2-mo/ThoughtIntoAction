"""Settings for runs started from outside the console, and their progress.

The remote caller supplies only a task prompt and a duration; the model address
and RTC parameters stay on this machine. Those live in a small state file so the
API works without a browser tab open.
"""
from __future__ import annotations

import json
import re
import time
from pathlib import Path

from embodied.core.project import PROJECT_ROOT

CONFIG_PATH = PROJECT_ROOT / "outputs/runtime/policy_remote.json"
PROMPT_LIMIT = 500
SECONDS_RANGE = (1, 300)

DEFAULTS = {
    "model_id": None,
    "rtc": True,
    "chunk_steps": 50,
    "reinfer_remaining": 30,
    "blend": 16,
    "guidance": 10.0,
    "dry_run": False,
}


class RemoteError(RuntimeError):
    """A request the operator should see verbatim in the page or the caller's log."""


def _int(value, name: str, low: int, high: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not low <= value <= high:
        raise RemoteError(f"{name} 必须是 {low}..{high} 的整数")
    return value


def load_config(path: Path | None = None) -> dict:
    path = Path(path or CONFIG_PATH)
    config = dict(DEFAULTS)
    if path.is_file():
        try:
            stored = json.loads(path.read_text())
        except (OSError, ValueError):
            stored = {}
        if isinstance(stored, dict):
            config.update({key: stored[key] for key in DEFAULTS if key in stored})
    return config


def save_config(payload: dict, path: Path | None = None) -> dict:
    """Validate and persist what the local operator chose for remote callers."""
    if not isinstance(payload, dict):
        raise RemoteError("配置必须是 JSON 对象")
    config = load_config(path)
    if "model_id" in payload:
        model_id = payload.get("model_id")
        if model_id is not None and (not isinstance(model_id, str) or not model_id.strip()):
            raise RemoteError("model_id 必须是非空字符串或 null")
        config["model_id"] = model_id
    if "rtc" in payload:
        if type(payload["rtc"]) is not bool:
            raise RemoteError("rtc 必须是布尔值")
        config["rtc"] = payload["rtc"]
    chunk = _int(payload.get("chunk_steps", config["chunk_steps"]), "chunk_steps", 1, 50)
    config["chunk_steps"] = chunk
    config["reinfer_remaining"] = _int(payload.get("reinfer_remaining", config["reinfer_remaining"]),
                                       "reinfer_remaining", 0, chunk - 1)
    config["blend"] = _int(payload.get("blend", config["blend"]), "blend", 0, chunk)
    guidance = payload.get("guidance", config["guidance"])
    if isinstance(guidance, bool) or not isinstance(guidance, (int, float)) or not 0 <= guidance <= 100:
        raise RemoteError("guidance 必须在 0..100")
    config["guidance"] = float(guidance)
    if "dry_run" in payload:
        if type(payload["dry_run"]) is not bool:
            raise RemoteError("dry_run 必须是布尔值")
        config["dry_run"] = payload["dry_run"]
    path = Path(path or CONFIG_PATH)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(config, ensure_ascii=False, indent=2) + "\n")
    temporary.replace(path)
    return config


def request_fields(payload: dict) -> dict:
    """Everything a remote caller is allowed to say: prompt and duration."""
    if not isinstance(payload, dict):
        raise RemoteError("请求必须是 JSON 对象")
    prompt = payload.get("prompt")
    if not isinstance(prompt, str) or not 1 <= len(prompt.strip()) <= PROMPT_LIMIT:
        raise RemoteError(f"prompt 必须是 1..{PROMPT_LIMIT} 字符的字符串")
    low, high = SECONDS_RANGE
    seconds = payload.get("seconds")
    if seconds is None:
        seconds = 60
    if isinstance(seconds, bool) or not isinstance(seconds, (int, float)):
        raise RemoteError("seconds 必须是数字")
    seconds = int(seconds)
    if not low <= seconds <= high:
        raise RemoteError(f"seconds 必须在 {low}..{high}")
    return {"prompt": prompt.strip(), "seconds": seconds}


_EVENT = re.compile(r'^\{"event": "(?P<name>[a-z]+)"')


def progress(lines: list[str]) -> dict:
    """Count what the runner already reported, from its own JSONL events."""
    chunks = rejected = dry_ticks = 0
    latency = None
    last_event = None
    for line in lines:
        text = line.strip()
        if not text.startswith("{"):
            continue
        try:
            event = json.loads(text)
        except ValueError:
            continue
        name = event.get("event")
        last_event = name or last_event
        if name == "chunk":
            chunks += 1
            if isinstance(event.get("latency_ms"), (int, float)):
                latency = event["latency_ms"]
        elif name == "rejected":
            rejected += 1
        elif name == "dry":
            dry_ticks += 1
    return {"chunks": chunks, "rejected": rejected, "last_chunk_latency_ms": latency,
            "last_event": last_event, "parsed_at": time.time()}
