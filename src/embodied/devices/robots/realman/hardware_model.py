"""Nominal planning geometry and controller-DH FK/Jacobian for hardware."""
from pathlib import Path
import hashlib
import json
from types import SimpleNamespace
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from .model import Model, vector


class JointModel(Model):
    """Use the read-back modified DH chain with the same MuJoCo DLS as simulation."""
    def __init__(self, info):
        dh = np.asarray([info['get_DH_data'][f'joint_{i}'] for i in range(1, 7)], dtype=float)
        if dh.shape != (6, 4) or not np.isfinite(dh).all():
            raise ValueError('Invalid controller DH parameters')
        self.lower = np.deg2rad(vector(info['get_joint_min_pos']['min_pos'], 6, 'joint min') / 1000)
        self.upper = np.deg2rad(vector(info['get_joint_max_pos']['max_pos'], 6, 'joint max') / 1000)
        if np.any(self.lower >= self.upper):
            raise ValueError('Invalid controller joint limits')

        def frame(raw):
            pose = vector(raw['pose'], 6, 'frame') / [1e6, 1e6, 1e6, 1000, 1000, 1000]
            return pose[:3], Rotation.from_euler('xyz', pose[3:])

        def body(parent, name, position, rotation):
            return ET.SubElement(parent, 'body', name=name,
                pos=' '.join(map(str, position)),
                quat=' '.join(map(str, rotation.as_quat()[[3, 0, 1, 2]])))

        root = ET.Element('mujoco', model='rm65_controller_dh')
        ET.SubElement(root, 'compiler', angle='radian')
        world = ET.SubElement(root, 'worldbody')
        work_p, work_r = frame(info['get_current_work_frame'])
        parent = body(world, 'base', -work_r.inv().apply(work_p), work_r.inv())
        for i, (alpha, a, d, offset) in enumerate(dh):
            alpha, offset = np.deg2rad([alpha / 1000, offset / 1000])
            a, d = a / 1e6, d / 1e6
            rotation = Rotation.from_rotvec([alpha, 0, 0]) * Rotation.from_rotvec([0, 0, offset])
            parent = body(parent, f'joint{i+1}_frame', [a, -np.sin(alpha)*d, np.cos(alpha)*d], rotation)
            ET.SubElement(parent, 'joint', name=f'joint{i+1}', type='hinge', axis='0 0 1',
                range=f'{self.lower[i]} {self.upper[i]}')
            # Kinematic-only chain; these inertials are not used for control.
            ET.SubElement(parent, 'inertial', pos='0 0 0', mass='1', diaginertia='1 1 1')
        tool_p, tool_r = frame(info['get_current_tool_frame'])
        tcp = body(parent, 'tcp_link', tool_p, tool_r)
        ET.SubElement(tcp, 'site', name='tcp', size='.001')
        self.model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
        self.data = mujoco.MjData(self.model)
        self.site = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, 'tcp')
        self.tcp_link = 'tcp_link'
        self.jacp, self.jacr = np.zeros((3, 6)), np.zeros((3, 6))


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
