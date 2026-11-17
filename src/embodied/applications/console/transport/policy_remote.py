"""Remote control of a policy run: prompt + duration in, status and feedback out.

The caller (another machine or script) sends only a task prompt and a duration.
The model address and RTC parameters come from `core.policy_remote`, which the
local operator sets on the policy page; the run itself is the same
`./run.sh policy remote` process the page starts.
"""
from __future__ import annotations

import asyncio
import hmac
import json
import time
from pathlib import Path

from fastapi import APIRouter, HTTPException, Query, Request

from embodied.applications.console.core.models import ModelError, load_models
from embodied.applications.console.core.policy_remote import (
    RemoteError, load_config, progress, request_fields, save_config,
)
from embodied.applications.console.core.settings import BackendSettings
from embodied.applications.console.operations import OperationError, OperationManager
from embodied.contracts.runtime.control import ControlClient, live_runs
from embodied.core.project import PROJECT_ROOT

KIND = "policy"
RUNS = PROJECT_ROOT / "outputs/runtime/control"
OUTPUTS = PROJECT_ROOT / "outputs/policy_remote"


def _kernel_snapshot() -> dict:
    """Live robot state while the run is going; empty when no kernel answers."""
    try:
        runs = live_runs(RUNS)
        if not runs:
            return {"connected": False}
        status = ControlClient(runs[-1]).status()
    except (OSError, EOFError, ValueError, KeyError, RuntimeError):
        return {"connected": False}
    return {
        "connected": True,
        "runtime_id": status.get("runtime_id"),
        "phase": status.get("phase"),
        "owner": (status.get("owner") or {}).get("source"),
        "q": status.get("q"),
        "gripper_widths": status.get("gripper_widths"),
    }


def _latest_run(started_at: float) -> dict:
    """The run directory this operation produced, with a short command tail."""
    if not OUTPUTS.is_dir():
        return {}
    newest = max((path for path in OUTPUTS.iterdir() if path.is_dir()),
                 key=lambda path: path.stat().st_mtime, default=None)
    if newest is None or newest.stat().st_mtime < started_at - 5:
        return {}
    tail = []
    commands = newest / "commands.jsonl"
    if commands.is_file():
        try:
            size = commands.stat().st_size
            with commands.open("rb") as handle:
                handle.seek(max(0, size - 8192))
                lines = handle.read().decode("utf-8", "replace").splitlines()[-3:]
            tail = [json.loads(line) for line in lines if line.strip().startswith("{")]
        except (OSError, ValueError):
            tail = []
    summary = {}
    summary_path = newest / "summary.json"
    if summary_path.is_file():
        try:
            summary = json.loads(summary_path.read_text())
        except (OSError, ValueError):
            summary = {}
    return {"directory": str(newest.relative_to(PROJECT_ROOT)), "commands_tail": tail,
            "summary": summary}


def create_policy_remote_router(manager: OperationManager, settings: BackendSettings) -> APIRouter:
    router = APIRouter(prefix="/api/v1/policy/remote", tags=["policy"])

    def authorize(token: str | None) -> None:
        if settings.token is not None and (
            token is None or not hmac.compare_digest(token, settings.token)
        ):
            raise HTTPException(status_code=401, detail="invalid pairing token")

    def operation() -> dict | None:
        for entry in manager.snapshot():
            if entry.get("kind") == KIND:
                return entry
        return None

    def status() -> dict:
        entry = operation()
        config = load_config()
        models = load_models()
        model = next((item for item in models if item["id"] == config.get("model_id")), None)
        running = bool(entry and entry.get("state") in ("starting", "running", "stopping"))
        lines = []
        if entry is not None:
            try:
                lines = manager.log_tail(KIND, 400)
            except OperationError:
                lines = []
        result = {
            "state": entry.get("state") if entry else "idle",
            "running": running,
            "operation": entry,
            "config": config,
            "model": None if model is None else {key: model[key] for key in ("id", "name", "url")},
            "progress": progress(lines) if lines else {"chunks": 0, "rejected": 0,
                                                       "last_chunk_latency_ms": None,
                                                       "last_event": None},
            "events": [line for line in lines[-20:] if line.startswith('{"event"')],
            "kernel": _kernel_snapshot(),
        }
        if entry is not None:
            started = float(entry.get("started_at") or 0)
            result["started_at"] = started
            result["elapsed_s"] = round(time.time() - started, 1) if started else None
            result["run"] = _latest_run(started)
        return result

    @router.get("")
    async def read(token: str | None = Query(default=None)) -> dict:
        authorize(token)
        return await asyncio.to_thread(status)

    @router.put("/config")
    async def configure(request: Request, token: str | None = Query(default=None)) -> dict:
        authorize(token)
        payload = await _payload(request)
        try:
            config = await asyncio.to_thread(save_config, payload)
        except RemoteError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        return {"config": config}

    @router.post("")
    async def start(request: Request, token: str | None = Query(default=None)) -> dict:
        authorize(token)
        payload = await _payload(request)
        try:
            fields = request_fields(payload)
            config = load_config()
            models = load_models()
            model = next((item for item in models if item["id"] == config.get("model_id")), None)
            if model is None:
                raise RemoteError("本机还没有选定远程模型：先在策略页选择并保存远程默认")
            if not model["url"].startswith(("ws://", "wss://")):
                raise RemoteError("登记的模型地址无效")
            operation_payload = {
                "confirm": True, "mode": "remote", "server": model["url"],
                "prompt": fields["prompt"], "seconds": fields["seconds"],
                "rtc": config["rtc"], "chunk_steps": config["chunk_steps"],
                "reinfer_remaining": config["reinfer_remaining"], "blend": config["blend"],
                "guidance": config["guidance"], "dry_run": config["dry_run"],
            }
            entry = await manager.start(KIND, operation_payload)
        except RemoteError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        except ModelError as exc:
            raise HTTPException(status_code=500, detail=str(exc)) from exc
        except OperationError as exc:
            raise HTTPException(status_code=exc.status, detail=str(exc)) from exc
        return {"state": "running", "operation": entry, "prompt": fields["prompt"],
                "seconds": fields["seconds"], "model": {"id": model["id"], "name": model["name"],
                                                        "url": model["url"]}}

    @router.post("/stop")
    async def stop(token: str | None = Query(default=None)) -> dict:
        authorize(token)
        try:
            entry = await manager.stop(KIND)
        except OperationError as exc:
            raise HTTPException(status_code=exc.status, detail=str(exc)) from exc
        return {"operation": entry}

    return router


async def _payload(request: Request) -> dict:
    try:
        payload = await request.json()
    except (json.JSONDecodeError, UnicodeDecodeError):
        raise HTTPException(status_code=400, detail="请求体必须是 JSON")
    if not isinstance(payload, dict):
        raise HTTPException(status_code=400, detail="请求体必须是 JSON 对象")
    return payload
