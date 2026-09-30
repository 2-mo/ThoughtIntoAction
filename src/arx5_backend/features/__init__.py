from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..protocol import CommandCatalog


def feature_catalog() -> "CommandCatalog":
    from ..protocol import CommandCatalog
    from .arm.commands import FEATURE as ARM
    from .calibration.commands import FEATURE as CALIBRATION
    from .collection.commands import FEATURE as COLLECTION
    from .cube.commands import FEATURE as CUBE
    from .gripper.commands import FEATURE as GRIPPER
    from .teleoperation.commands import FEATURE as TELEOPERATION

    return CommandCatalog(
        (ARM, GRIPPER, TELEOPERATION, CUBE, CALIBRATION, COLLECTION)
    )


__all__ = ["feature_catalog"]
