from __future__ import annotations

import asyncio
import base64
import binascii
import hmac
import json
import shutil
import time
from datetime import datetime
from pathlib import Path

from fastapi import APIRouter, HTTPException, Query
from fastapi.responses import Response, StreamingResponse

from embodied.applications.console.core.settings import BackendSettings
from embodied.contracts.paths import PROJECT_ROOT
from embodied.core.project import load_config
from embodied.core.session_protocol import request
from embodied.devices.sensors.client import socket_path

STREAM_POLL_S = 0.02
STREAM_IDLE_S = 0.4


def _camera_preview(camera_id: str) -> tuple[bytes, float | None]:
    """Latest JPEG for one camera; the shared service re-encodes previews every 200 ms."""
    path = socket_path()
    if not path.exists():
        raise RuntimeError("相机服务未启动")
    try:
        # Ask for one feed: the unfiltered snapshot carries every camera plus depth.
        state = request(path, "monitor", camera_id=camera_id, timeout=1.0)
    except OSError as exc:
        raise RuntimeError("相机服务未就绪") from exc
    preview = state.get("preview") or {}
    data = preview.get("jpeg_base64")
    if not data:
        error = (state.get("camera") or {}).get("error")
        raise RuntimeError(error or "相机尚未出帧")
    age = preview.get("age_s")
    return base64.b64decode(data, validate=True), None if age is None else float(age)


def create_collection_router(settings: BackendSettings) -> APIRouter:
    router = APIRouter(prefix="/api/v1/collection", tags=["collection"])

    def authorize(token: str | None) -> None:
        if settings.token is not None and (
            token is None or not hmac.compare_digest(token, settings.token)
        ):
            raise HTTPException(status_code=401, detail="invalid pairing token")

    def known(camera_id: str) -> None:
        if camera_id not in load_config().get("cameras", {}):
            raise HTTPException(status_code=404, detail="unknown camera")

    @router.get("/camera-preview/{camera_id}.jpg", include_in_schema=False)
    async def camera_preview(
        camera_id: str,
        token: str | None = Query(default=None),
    ) -> Response:
        authorize(token)
        known(camera_id)
        try:
            jpeg, age = await asyncio.to_thread(_camera_preview, camera_id)
        except (OSError, RuntimeError, KeyError, ValueError, binascii.Error) as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc
        return Response(
            content=jpeg,
            media_type="image/jpeg",
            headers={
                "Cache-Control": "no-store, max-age=0",
                "X-Frame-Age": "" if age is None else f"{age:.3f}",
            },
        )

    @router.get("/episodes")
    async def episodes(
        directory: str | None = Query(default=None),
        limit: int = Query(default=8, ge=1, le=50),
        token: str | None = Query(default=None),
    ) -> dict[str, object]:
        authorize(token)

        def scan() -> dict[str, object]:
            root = (PROJECT_ROOT / "outputs/datasets").resolve()
            path = Path(directory) if directory else Path("outputs/datasets/runtime")
            if not path.is_absolute():
                path = PROJECT_ROOT / path
            path = path.resolve()
            if root not in path.parents:
                raise HTTPException(status_code=400, detail="data directory must stay under outputs/datasets")
            items: list[dict[str, object]] = []
            total = 0
            complete = failed = 0
            # Per-task totals so the page can show progress for the task being
            # recorded rather than everything that happens to be on disk.
            by_task: dict[str, dict[str, int]] = {}
            if path.is_dir():
                folders = sorted(
                    (entry for entry in path.iterdir() if entry.is_dir()),
                    key=lambda entry: entry.stat().st_mtime,
                    reverse=True,
                )
                for folder in folders:
                    meta = folder / "episode.json"
                    if not meta.is_file():
                        continue
                    total += 1
                    try:
                        data = json.loads(meta.read_text())
                        counts = data.get("counts") or {}
                        lifecycle = data.get("lifecycle")
                        task = str(data.get("task") or "").strip()
                        bucket = by_task.setdefault(task, {"complete": 0, "failed": 0})
                        if lifecycle == "complete":
                            complete += 1
                            bucket["complete"] += 1
                        else:
                            failed += 1
                            bucket["failed"] += 1
                        started, stopped = data.get("started_monotonic_ns"), data.get("stopped_monotonic_ns")
                        seconds = ((stopped - started) / 1e9
                                   if isinstance(started, int) and isinstance(stopped, int) and stopped >= started
                                   else None)
                    except (OSError, ValueError):
                        failed += 1
                        continue
                    if len(items) >= limit:
                        continue
                    items.append(
                        {
                            "name": folder.name,
                            "task": data.get("task"),
                            "lifecycle": lifecycle,
                            "ticks": counts.get("state"),
                            "frames": counts.get("frame"),
                            "seconds": seconds,
                            "cameras": list((data.get("cameras") or {}).keys()),
                            "written_at": datetime.fromtimestamp(meta.stat().st_mtime).isoformat(timespec="seconds"),
                        }
                    )
            return {"directory": str(path.relative_to(PROJECT_ROOT)), "episodes": items, "total": total,
                    "summary": {"complete": complete, "failed": failed, "by_task": by_task}}

        return await asyncio.to_thread(scan)

    @router.delete("/episodes/{name}")
    async def delete_episode(
        name: str,
        directory: str | None = Query(default=None),
        token: str | None = Query(default=None),
    ) -> dict[str, object]:
        """Delete one recorded episode directory; never anything outside outputs/datasets."""
        authorize(token)

        def remove() -> dict[str, object]:
            root = (PROJECT_ROOT / "outputs/datasets").resolve()
            path = Path(directory) if directory else Path("outputs/datasets/runtime")
            if not path.is_absolute():
                path = PROJECT_ROOT / path
            path = path.resolve()
            if root not in path.parents:
                raise HTTPException(status_code=400, detail="data directory must stay under outputs/datasets")
            if not name or name in (".", "..") or "/" in name or "\\" in name:
                raise HTTPException(status_code=400, detail="episode name must be a single folder name")
            target = (path / name).resolve()
            # Only a direct child that still looks like a recording may be removed.
            if target.parent != path or not (target / "episode.json").is_file():
                raise HTTPException(status_code=404, detail="找不到该 episode")
            shutil.rmtree(target)
            return {"deleted": name, "directory": str(path.relative_to(PROJECT_ROOT))}

        return await asyncio.to_thread(remove)

    @router.get("/camera-stream/{camera_id}.mjpeg", include_in_schema=False)
    async def camera_stream(
        camera_id: str,
        token: str | None = Query(default=None),
    ) -> StreamingResponse:
        """Continuously push the latest preview, so the browser draws frames instead of polling."""
        authorize(token)
        known(camera_id)

        async def frames():
            last_stamp = None
            while True:
                try:
                    jpeg, age = await asyncio.to_thread(_camera_preview, camera_id)
                except (OSError, RuntimeError, KeyError, ValueError, binascii.Error):
                    await asyncio.sleep(STREAM_IDLE_S)
                    continue
                # The service re-encodes every 200 ms; its age identifies that frame,
                # so a fast poll does not resend the same JPEG.
                stamp = None if age is None else round(time.monotonic() - age, 2)
                if stamp is None or stamp != last_stamp:
                    last_stamp = stamp
                    yield (
                        b"--frame\r\nContent-Type: image/jpeg\r\n"
                        b"Content-Length: " + str(len(jpeg)).encode() + b"\r\n\r\n"
                        + jpeg + b"\r\n"
                    )
                await asyncio.sleep(STREAM_POLL_S)

        return StreamingResponse(
            frames(),
            media_type="multipart/x-mixed-replace; boundary=frame",
            headers={"Cache-Control": "no-store, max-age=0"},
        )

    return router
