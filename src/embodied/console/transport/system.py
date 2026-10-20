from __future__ import annotations

from fastapi import APIRouter

from embodied.console.protocol import CommandCatalog


def create_system_router(catalog: CommandCatalog) -> APIRouter:
    router = APIRouter(prefix="/api/v1/system", tags=["system"])

    @router.get("/health")
    async def health() -> dict[str, object]:
        return {"status": "ok", "api_version": 1}

    @router.get("/capabilities")
    async def capabilities() -> dict[str, object]:
        return {"api_version": 1, "features": catalog.describe()}

    return router
