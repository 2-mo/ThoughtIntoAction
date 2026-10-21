from __future__ import annotations

import base64
import binascii
import hmac

from fastapi import APIRouter, HTTPException, Query
from fastapi.responses import Response

from embodied.console.core.settings import BackendSettings
from embodied.console.runtime import ControlRuntime


CAMERA_VIDEO_KEYS = frozenset({"exterior_rgb", "wrist_rgb"})


def create_collection_router(
    runtime: ControlRuntime,
    settings: BackendSettings,
) -> APIRouter:
    router = APIRouter(prefix="/api/v1/collection", tags=["collection"])

    @router.get("/camera-preview/{video_key}.jpg", include_in_schema=False)
    async def camera_preview(
        video_key: str,
        token: str | None = Query(default=None),
    ) -> Response:
        if settings.token is not None and (
            token is None or not hmac.compare_digest(token, settings.token)
        ):
            raise HTTPException(status_code=401, detail="invalid pairing token")
        if video_key not in CAMERA_VIDEO_KEYS:
            raise HTTPException(status_code=404, detail="unknown camera")
        try:
            preview = await runtime.collection_camera_preview(video_key)
            jpeg = base64.b64decode(
                str(preview["jpeg_base64"]),
                validate=True,
            )
        except (KeyError, RuntimeError, binascii.Error, ValueError) as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc
        return Response(
            content=jpeg,
            media_type="image/jpeg",
            headers={
                "Cache-Control": "no-store, max-age=0",
                "X-Camera-Frame-Number": str(preview.get("frame_number") or ""),
                "X-Captured-Monotonic-Ns": str(
                    preview.get("host_monotonic_ns") or ""
                ),
            },
        )

    return router
