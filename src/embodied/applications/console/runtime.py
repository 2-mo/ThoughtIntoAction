from __future__ import annotations

from typing import Any

from embodied.applications.console.core.settings import BackendSettings
from embodied.applications.console.protocol import CommandCatalog


class ControlRuntime:
    """Own the single control session used by all API feature modules: the arx kernel or the legacy arm."""

    def __init__(self, settings: BackendSettings, catalog: CommandCatalog) -> None:
        self.settings = settings
        self.catalog = catalog
        if settings.kernel:
            from embodied.applications.console.kernel.arx import ArxConsole
            self.session = ArxConsole(settings.run, settings.token)
            return
        from embodied.applications.console.kernel.control import (  # Opens the arm directly (legacy).
            ControlKernel,
            TELEOPERATION_CONFIG_PATH,
            load_teleoperation_config,
        )
        self.session = ControlKernel(
            settings.interface,
            settings.token,
            teleoperation_config=load_teleoperation_config(TELEOPERATION_CONFIG_PATH),
            collection_camera_url=settings.camera_url,
            collection_camera_ca=settings.camera_ca,
            collection_camera_origin=settings.camera_origin,
            persistent_controller=settings.auto_connect_arm,
        )

    async def startup(self) -> None:
        await self.session.startup()

    async def serve(self, connection: object) -> None:
        await self.session.handle(connection)  # type: ignore[arg-type]

    async def collection_camera_preview(
        self, video_key: str
    ) -> dict[str, Any]:
        camera = self.session.collection_camera
        if camera is None:
            raise RuntimeError("主机未配置双相机采集服务")
        return await camera.request(
            "collection_camera_preview",
            video_key=video_key,
            timeout_s=5.0,
        )

    async def shutdown(self) -> None:
        await self.session.shutdown()
