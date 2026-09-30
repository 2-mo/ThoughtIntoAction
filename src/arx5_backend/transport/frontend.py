from __future__ import annotations

from pathlib import Path

from fastapi import APIRouter, HTTPException
from fastapi.responses import FileResponse, Response
from starlette.exceptions import HTTPException as StarletteHTTPException
from starlette.staticfiles import StaticFiles


class SPAStaticFiles(StaticFiles):
    async def get_response(self, path: str, scope: dict) -> Response:
        try:
            response = await super().get_response(path, scope)
        except StarletteHTTPException as exc:
            if exc.status_code != 404 or Path(path).suffix:
                raise
            response = await super().get_response("index.html", scope)
        if path in {"", "index.html"} or not Path(path).suffix:
            response.headers["Cache-Control"] = "no-store"
        elif Path(path).suffix in {".js", ".mjs", ".css", ".html", ".json"}:
            response.headers["Cache-Control"] = "no-cache, must-revalidate"
        return response


def create_ca_router(ca_cert: Path | None) -> APIRouter:
    router = APIRouter()

    @router.get("/arx5-local-ca.crt", include_in_schema=False)
    async def local_ca_certificate() -> FileResponse:
        if ca_cert is None or not ca_cert.is_file():
            raise HTTPException(status_code=404)
        return FileResponse(
            ca_cert,
            media_type="application/x-x509-ca-cert",
            headers={"Cache-Control": "no-store"},
        )

    return router
