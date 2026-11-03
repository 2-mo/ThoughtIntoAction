"""Nominal planning model; connected hardware uses controller DH for local IK."""
from pathlib import Path
import hashlib
import json
from types import SimpleNamespace

from .model import Model


def planning_model(config):
    path = Path(config.model_path)
    manifest = json.loads(path.with_suffix('.model.json').read_text())
    root = path.parent.parent
    for name, digest in manifest['sha256'].items():
        if hashlib.sha256((root/name).read_bytes()).hexdigest() != digest:
            raise ValueError(f'RM65 model artifact changed: {name}')
    if manifest['tool_offset_m'] != config.tool_offset_m:
        raise ValueError('RM65 nominal model TCP differs from hardware profile')
    # The authored jaws are display/simulation geometry, not hardware calibration.
    model_config = SimpleNamespace(model_id=config.model_id, model_path=config.model_path,
        joint_min=config.joint_min, joint_max=config.joint_max)
    model = Model(model_config)
    model.nominal_gripper_width = manifest['simplified_gripper_width_m']
    return model
