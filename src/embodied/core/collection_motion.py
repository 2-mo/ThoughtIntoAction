"""Read literal motion settings from the collection launcher without executing it."""
from pathlib import Path
import re
import math

ROOT = Path(__file__).resolve().parents[3]
MOTION_KEYS = (
    'max_joint_acceleration_rad_s2', 'max_joint_jerk_rad_s3',
    'home_speed_rad_s', 'home_final_speed_rad_s', 'home_acceleration_rad_s2', 'home_jerk_rad_s3',
    'preset_speed_rad_s', 'preset_acceleration_rad_s2', 'preset_jerk_rad_s3',
    'max_command_lead_rad', 'preview_time_s', 'teleop_preview_time_s', 'joint_soft_limit_margin_rad',
)


def collection_motion(path=None):
    """Reuse the launcher's numeric defaults, never source/execute shell code."""
    path = Path(path) if path else ROOT/'启动遥操.sh'
    source = path.read_text()
    values = {}
    for key in ('max_joint_speed_rad_s', *MOTION_KEYS):
        matches = re.findall(r'^' + key.upper() + r'=([0-9.eE+\-]+)\s*(?:#.*)?$', source, re.MULTILINE)
        if len(matches) != 1 or not math.isfinite(float(matches[0])) or float(matches[0]) < 0:
            raise ValueError(f'{path}: {key.upper()} 必须是唯一的有限非负数字赋值')
        values[key] = float(matches[0])
    return values.pop('max_joint_speed_rad_s'), values
