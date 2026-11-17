"""Device setup API: what is attached, how it pairs, and what must restart.

Read-only discovery plus one validated write. Saving compiles the kernel-facing
profile for the selected vendor; nothing is restarted here so a recording or a
teleoperation session is never interrupted by a configuration change.
"""
from __future__ import annotations

import asyncio
import hmac
import json

from fastapi import APIRouter, HTTPException, Query, Request

from embodied.applications.console.core import setup_library
from embodied.applications.console.core.devices import (
    SetupError, compile_setup, discover, load_setup, pending_restarts, previews, save_setup,
    validate_setup,
)
from embodied.applications.console.core.settings import BackendSettings
from embodied.contracts.runtime.control import live_runs
from embodied.core.project import PROJECT_ROOT, load_config
from embodied.devices.sensors.client import socket_path


def _running() -> dict:
    from embodied.applications.console.kernel.arx import RUNS

    return {
        "kernel": bool(live_runs(RUNS)),
        "camera": socket_path().exists(),
    }


def seed_setup() -> dict:
    """A starting document derived from today's config, so the page opens populated."""
    config = load_config()
    cameras = config.get("cameras") or {}
    dual = config.get("setup") or {}
    robots = dual.get("robots") or []
    if not robots:
        profile = {}
        for name in ("configs/robots/arx5/dual.json", "configs/robots/arx5/robot.json"):
            try:
                profile = json.loads((PROJECT_ROOT / name).read_text())
                break
            except (OSError, ValueError):
                continue
        arms = profile.get("arms") if isinstance(profile, dict) else None
        if arms:
            robots = [{"id": f"arm{index + 1}", "vendor": "arx", "model": "x5",
                       "side": arm.get("id", "left"), "can_interface": arm.get("can_interface", f"can{index}")}
                      for index, arm in enumerate(arms)]
        else:
            robots = [{"id": "arm1", "vendor": "arx", "model": "x5", "side": "single",
                       "can_interface": profile.get("can_interface")
                       or (config.get("robot") or {}).get("can_port", "can0")}]
    seeded = []
    for camera_id, options in cameras.items():
        if not isinstance(options, dict):
            continue
        label = str(options.get("label") or "")
        mount = options.get("mount") or (
            "wrist" if ("wrist" in camera_id or "腕" in label) else "fixed")
        arm = None
        if mount == "wrist":
            if len(robots) == 1:
                arm = robots[0]["id"]
            else:
                arm = robots[1]["id"] if "second" in camera_id else robots[0]["id"]
        serial = options.get("serial") or ""
        if "REPLACE" in serial.upper():
            # The example config ships a placeholder; treat it as unassigned.
            serial = ""
        seeded.append({
            "id": camera_id,
            "label": options.get("label") or camera_id,
            "enabled": options.get("enabled", True) is not False,
            "driver": options.get("driver") or "realsense",
            "model": options.get("model") or "D435i",
            "serial": serial,
            "width": options.get("width") or 640,
            "height": options.get("height") or 480,
            "fps": options.get("fps") or 30,
            "mount": mount,
            "arm": arm,
        })
    try:
        return validate_setup({"version": 1, "robots": robots, "cameras": seeded})
    except SetupError:
        # An unconfigured machine still opens with whatever already exists.
        return {"version": 1, "robots": robots, "cameras": seeded}


def create_setup_router(settings: BackendSettings) -> APIRouter:
    router = APIRouter(prefix="/api/v1/setup", tags=["setup"])

    def authorize(token: str | None) -> None:
        if settings.token is not None and (
            token is None or not hmac.compare_digest(token, settings.token)
        ):
            raise HTTPException(status_code=401, detail="invalid pairing token")

    @router.get("")
    async def read(token: str | None = Query(default=None)) -> dict[str, object]:
        authorize(token)
        saved = await asyncio.to_thread(load_setup)
        running = _running()
        return {
            "setup": saved,
            "seed": await asyncio.to_thread(seed_setup),
            "running": running,
            "restarts": await asyncio.to_thread(
                pending_restarts, saved, kernel_running=running["kernel"], camera_running=running["camera"]
            ) if saved else [],
        }

    @router.get("/devices")
    async def devices(
        refresh: int = Query(default=0, ge=0, le=1),
        token: str | None = Query(default=None),
    ) -> dict[str, object]:
        authorize(token)
        return await asyncio.to_thread(discover, bool(refresh))

    @router.put("")
    async def write(request: Request, token: str | None = Query(default=None)) -> dict[str, object]:
        authorize(token)
        payload = await _payload(request)
        try:
            document = await asyncio.to_thread(validate_setup, payload)
            compiled = await asyncio.to_thread(compile_setup, document)
            saved = await asyncio.to_thread(save_setup, document)
        except SetupError as exc:
            raise HTTPException(status_code=exc.status, detail=str(exc)) from exc
        running = _running()
        return {
            "setup": saved,
            "compiled": compiled,
            "running": running,
            "restarts": pending_restarts(saved, kernel_running=running["kernel"], camera_running=running["camera"]),
        }

    @router.get("/library")
    async def library(token: str | None = Query(default=None)) -> dict[str, object]:
        authorize(token)
        return {
            "entries": await asyncio.to_thread(setup_library.entries),
            "active": await asyncio.to_thread(setup_library.active),
        }

    @router.post("/library")
    async def save_entry(request: Request, token: str | None = Query(default=None)) -> dict[str, object]:
        authorize(token)
        payload = await _payload(request)
        try:
            entry = await asyncio.to_thread(
                setup_library.save, payload.get("name"),
                payload.get("setup") or payload.get("document"), payload.get("slug"),
            )
        except SetupError as exc:
            raise HTTPException(status_code=exc.status, detail=str(exc)) from exc
        return {"entry": entry}

    @router.post("/library/{slug}/apply")
    async def apply_entry(slug: str, token: str | None = Query(default=None)) -> dict[str, object]:
        authorize(token)
        try:
            entry = await asyncio.to_thread(setup_library.load, slug)
            compiled = await asyncio.to_thread(compile_setup, entry["setup"])
            saved = await asyncio.to_thread(save_setup, entry["setup"])
            await asyncio.to_thread(setup_library.set_active, slug)
        except SetupError as exc:
            raise HTTPException(status_code=exc.status, detail=str(exc)) from exc
        running = _running()
        return {
            "setup": saved,
            "compiled": compiled,
            "active": slug,
            "name": entry["name"],
            "running": running,
            "restarts": pending_restarts(saved, kernel_running=running["kernel"], camera_running=running["camera"]),
        }

    @router.delete("/library/{slug}")
    async def delete_entry(slug: str, token: str | None = Query(default=None)) -> dict[str, object]:
        authorize(token)
        try:
            await asyncio.to_thread(setup_library.delete, slug)
        except SetupError as exc:
            raise HTTPException(status_code=exc.status, detail=str(exc)) from exc
        return {"ok": True, "active": await asyncio.to_thread(setup_library.active)}

    @router.get("/previews")
    async def camera_previews(
        serials: str = Query(default=""),
        refresh: int = Query(default=0, ge=0, le=1),
        token: str | None = Query(default=None),
    ) -> dict[str, object]:
        authorize(token)
        wanted = [item.strip() for item in serials.split(",") if item.strip()]
        try:
            return await asyncio.wait_for(asyncio.to_thread(previews, wanted, refresh=bool(refresh)),
                                          timeout=12)
        except asyncio.TimeoutError:
            raise HTTPException(status_code=504, detail="取画面超时：相机可能被其他程序占用") from None

    return router


async def _payload(request: Request) -> dict:
    try:
        payload = await request.json()
    except (json.JSONDecodeError, UnicodeDecodeError):
        raise HTTPException(status_code=400, detail="请求体必须是 JSON") from None
    if not isinstance(payload, dict):
        raise HTTPException(status_code=400, detail="请求体必须是 JSON 对象")
    return payload
