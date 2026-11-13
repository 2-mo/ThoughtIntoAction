"""Model registry API: list with live state, mount and unmount by id.

The page never sends a command; ids are resolved against configs/models.toml in
`core.models`, which also owns the health probe and the mount bookkeeping.
"""
from __future__ import annotations

import asyncio
import hmac

from fastapi import APIRouter, HTTPException, Query

from embodied.applications.console.core.models import ModelError, describe, mount, unmount
from embodied.applications.console.core.settings import BackendSettings


def create_models_router(settings: BackendSettings) -> APIRouter:
    router = APIRouter(prefix="/api/v1/policy/models", tags=["policy"])

    def authorize(token: str | None) -> None:
        if settings.token is not None and (
            token is None or not hmac.compare_digest(token, settings.token)
        ):
            raise HTTPException(status_code=401, detail="invalid pairing token")

    @router.get("")
    async def read(token: str | None = Query(default=None)) -> dict[str, object]:
        authorize(token)
        return {"models": await asyncio.to_thread(describe)}

    @router.post("/{model_id}/mount")
    async def mount_model(model_id: str, token: str | None = Query(default=None)) -> dict[str, object]:
        authorize(token)
        try:
            return await asyncio.to_thread(mount, model_id)
        except ModelError as exc:
            raise HTTPException(status_code=404 if "未登记" in str(exc) else 409,
                                detail=str(exc)) from exc

    @router.post("/{model_id}/unmount")
    async def unmount_model(model_id: str, token: str | None = Query(default=None)) -> dict[str, object]:
        authorize(token)
        try:
            return await asyncio.to_thread(unmount, model_id)
        except ModelError as exc:
            raise HTTPException(status_code=404 if "未登记" in str(exc) else 503,
                                detail=str(exc)) from exc

    return router
