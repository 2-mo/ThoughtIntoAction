from __future__ import annotations

import hmac
import json

from fastapi import APIRouter, HTTPException, Query, Request

from embodied.applications.console.core.settings import BackendSettings
from embodied.applications.console.operations import OperationError, OperationManager


def create_operations_router(manager: OperationManager, settings: BackendSettings) -> APIRouter:
    router = APIRouter(prefix="/api/v1/operations", tags=["operations"])

    def authorize(token: str | None) -> None:
        if settings.token is not None and (
            token is None or not hmac.compare_digest(token, settings.token)
        ):
            raise HTTPException(status_code=401, detail="invalid pairing token")

    @router.get("")
    async def snapshot(token: str | None = Query(default=None)) -> dict[str, object]:
        authorize(token)
        return {"operations": manager.snapshot()}

    @router.post("/{kind}/start")
    async def start(
        kind: str, request: Request, token: str | None = Query(default=None)
    ) -> dict[str, object]:
        authorize(token)
        try:
            payload = await request.json()
        except (json.JSONDecodeError, UnicodeDecodeError):
            payload = {}
        if not isinstance(payload, dict):
            raise HTTPException(status_code=400, detail="请求体必须是 JSON 对象")
        try:
            return await manager.start(kind, payload)
        except OperationError as exc:
            raise HTTPException(status_code=exc.status, detail=str(exc)) from exc

    @router.post("/{kind}/stop")
    async def stop(kind: str, token: str | None = Query(default=None)) -> dict[str, object]:
        authorize(token)
        try:
            return await manager.stop(kind)
        except OperationError as exc:
            raise HTTPException(status_code=exc.status, detail=str(exc)) from exc

    @router.get("/{kind}/log")
    async def log(
        kind: str,
        token: str | None = Query(default=None),
        lines: int = Query(default=80, ge=1, le=500),
    ) -> dict[str, object]:
        authorize(token)
        try:
            return {"kind": kind, "lines": manager.log_tail(kind, lines)}
        except OperationError as exc:
            raise HTTPException(status_code=exc.status, detail=str(exc)) from exc

    return router
