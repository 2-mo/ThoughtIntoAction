from __future__ import annotations

from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from embodied.console.core.settings import BackendSettings
from embodied.console.features import feature_catalog
from embodied.console.runtime import ControlRuntime
from embodied.console.transport.collection import create_collection_router
from embodied.console.transport.control import create_control_router
from embodied.console.transport.frontend import SPAStaticFiles, create_ca_router
from embodied.console.transport.system import create_system_router


def create_app(settings: BackendSettings | None = None) -> FastAPI:
    resolved = settings or BackendSettings()
    catalog = feature_catalog()
    runtime = ControlRuntime(resolved, catalog)

    @asynccontextmanager
    async def lifespan(_: FastAPI):
        await runtime.startup()
        yield
        await runtime.shutdown()

    app = FastAPI(
        title="ARX5 Backend",
        version="1.0.0",
        lifespan=lifespan,
        docs_url="/api/docs",
        openapi_url="/api/openapi.json",
    )
    app.state.settings = resolved
    app.state.catalog = catalog
    app.state.runtime = runtime
    app.add_middleware(
        CORSMiddleware,
        allow_origins=list(resolved.allowed_origins),
        allow_credentials=False,
        allow_methods=["GET"],
        allow_headers=["*"],
    )
    app.include_router(create_system_router(catalog))
    app.include_router(create_control_router(runtime, resolved))
    app.include_router(create_collection_router(runtime, resolved))
    app.include_router(create_ca_router(resolved.ca_cert))
    if resolved.frontend_directory is not None:
        app.mount(
            "/",
            SPAStaticFiles(directory=resolved.frontend_directory, html=True),
            name="frontend",
        )
    return app
