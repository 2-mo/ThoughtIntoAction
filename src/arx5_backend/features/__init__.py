from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from arx5_backend.protocol import CommandCatalog


def feature_catalog() -> "CommandCatalog":
    from arx5_backend.protocol import CommandCatalog
    from arx5_backend.features.arm.commands import FEATURE as ARM
    from arx5_backend.features.calibration.commands import FEATURE as CALIBRATION
    from arx5_backend.features.collection.commands import FEATURE as COLLECTION
    from arx5_backend.features.cube.commands import FEATURE as CUBE
    from arx5_backend.features.gripper.commands import FEATURE as GRIPPER
    from arx5_backend.features.teleoperation.commands import FEATURE as TELEOPERATION

    return CommandCatalog(
        (ARM, GRIPPER, TELEOPERATION, CUBE, CALIBRATION, COLLECTION)
    )


__all__ = ["feature_catalog"]
