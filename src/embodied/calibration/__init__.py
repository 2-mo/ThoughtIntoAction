"""Hand-eye calibration on the arx kernel, one result per configured camera id.

    ./run.sh calib handeye marker | capture <camera_id> [--auto] | solve <session> | status

Results live in calibrations/hand_eye/<camera_id>.json: a wrist camera stores T_eef_camera
(TCP -> RGB optical), a fixed camera T_base_camera (base -> RGB optical). `load` returns a result
only if it was validated and was measured with the camera now configured under that id.
"""
import json
from pathlib import Path
import time

from embodied.core.project import PROJECT_ROOT

RESULTS = PROJECT_ROOT / 'calibrations/hand_eye'
TRANSFORMS = {'wrist': 'T_eef_camera', 'fixed': 'T_base_camera'}


def mount(camera_id, config=None):
    """`mount` in [cameras.<id>]; by default wrist_rgb is on the wrist and every other camera is fixed."""
    if config is None:
        from embodied.core.project import load_config
        config = load_config()
    options = config['cameras'].get(camera_id)
    if options is None:
        raise ValueError(f'Unknown camera id {camera_id!r}; configured: {", ".join(config["cameras"])}')
    value = options.get('mount', 'wrist' if camera_id == 'wrist_rgb' else 'fixed')
    if value not in TRANSFORMS:
        raise ValueError(f'cameras.{camera_id}.mount must be wrist or fixed')
    return value


def save(camera_id, result, root=None):
    """Make `result` the current calibration of `camera_id`; the previous one is kept beside it."""
    root = Path(root or RESULTS)
    root.mkdir(parents=True, exist_ok=True)
    path = root / f'{camera_id}.json'
    if path.exists():
        path.replace(root / f'{camera_id}.previous.json')
    path.write_text(json.dumps({**result, 'camera_id': camera_id, 'saved_unix_ns': time.time_ns()}, indent=2) + '\n')
    return path


def extrinsics(camera_id, serial=None, root=None):
    """The compact form recorded with episodes: transform, frames, status, quality; None if uncalibrated."""
    result = load(camera_id, serial, root)
    if result is None:
        return None
    name = TRANSFORMS[result['mount']]
    fit, validation = result.get('fit') or {}, result.get('validation') or {}
    return {'mount': result['mount'], name: result[name], 'camera_frame': 'RGB optical', 'eef_frame': 'tcp_link',
            'status': result['status'], 'operator_override': bool(result.get('operator_override')),
            'saved_unix_ns': result.get('saved_unix_ns'), 'method': result.get('method'),
            'fit_translation_rms_m': fit.get('translation_rms_m'),
            'validation_translation_rms_m': validation.get('translation_rms_m')}


def load(camera_id, serial=None, root=None):
    """The current validated result for `camera_id`, or None; a different camera serial invalidates it."""
    path = Path(root or RESULTS) / f'{camera_id}.json'
    try:
        result = json.loads(path.read_text())
    except (OSError, ValueError):
        return None
    if result.get('status') != 'validated' and not result.get('operator_override'):
        return None
    if serial is not None and result.get('camera', {}).get('serial') not in (None, serial):
        return None
    return result
