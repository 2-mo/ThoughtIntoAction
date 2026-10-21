from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from embodied.console.protocol import CommandCatalog


def feature_catalog() -> "CommandCatalog":
    from embodied.console.protocol import CommandCatalog
    from embodied.console.features.arm.commands import FEATURE as ARM
    from embodied.console.features.calibration.commands import FEATURE as CALIBRATION
    from embodied.console.features.collection.commands import FEATURE as COLLECTION
    from embodied.console.features.cube.commands import FEATURE as CUBE
    from embodied.console.features.gripper.commands import FEATURE as GRIPPER
    from embodied.console.features.teleoperation.commands import FEATURE as TELEOPERATION

    return CommandCatalog(
        (ARM, GRIPPER, TELEOPERATION, CUBE, CALIBRATION, COLLECTION)
    )


__all__ = ["feature_catalog"]
