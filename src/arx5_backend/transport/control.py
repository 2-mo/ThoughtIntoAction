from __future__ import annotations

from fastapi import APIRouter, WebSocket

from ..core.settings import BackendSettings
from ..runtime import ControlRuntime
from .websocket_adapter import ApiWebSocket


def create_control_router(
    runtime: ControlRuntime,
    settings: BackendSettings,
) -> APIRouter:
    router = APIRouter()

    @router.websocket("/api/v1/control")
    async def control_socket(websocket: WebSocket) -> None:
        origin = websocket.headers.get("origin")
        if origin is not None and origin not in settings.allowed_origins:
            await websocket.close(code=1008, reason="origin is not allowed")
            return
        await websocket.accept()
        await runtime.serve(ApiWebSocket(websocket, runtime.catalog))

    return router
