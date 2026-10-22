"""Quest input profile; loaded only when this input is selected."""
from dataclasses import dataclass, fields
import json
import math
import os
from pathlib import Path

@dataclass(frozen=True)
class VRConfig:
    sdk_library: str = "/opt/apps/roboticsservice/SDK/x64/libPXREARobotSDK.so"
    base_yaw_deg: float = 0.0
    grip_threshold: float = 0.7
    position_scale: tuple[float, float, float] = (1.0, 1.0, 1.0)


def _read(path, schema):
    values = json.loads(path.read_text())
    if not isinstance(values, dict):
        raise ValueError(f'Configuration must be an object: {path}')
    unknown = set(values) - {field.name for field in fields(schema)}
    if unknown:
        raise ValueError(f'Unknown configuration keys in {path.name}: {sorted(unknown)}')
    return values


def _path(value, base):
    path = Path(value).expanduser()
    return str((path if path.is_absolute() else base / path).resolve())


def load_profile(path):
    vr_config_path = Path(path)
    vr_values = _read(vr_config_path, VRConfig)
    library = os.environ.get('XROBOT_SDK_LIBRARY') or vr_values.get('sdk_library', VRConfig.sdk_library)
    vr_values['sdk_library'] = _path(library, vr_config_path.parent)
    if 'position_scale' in vr_values:
        vr_values['position_scale'] = tuple(vr_values['position_scale'])
    vr = VRConfig(**vr_values)
    if not math.isfinite(vr.base_yaw_deg):
        raise ValueError('base_yaw_deg must be finite')
    if not math.isfinite(vr.grip_threshold) or not 0 < vr.grip_threshold <= 1:
        raise ValueError('grip_threshold must be in (0, 1]')
    if len(vr.position_scale) != 3 or not all(math.isfinite(x) and x > 0 for x in vr.position_scale):
        raise ValueError('position_scale must contain three finite positive values')
    return vr
