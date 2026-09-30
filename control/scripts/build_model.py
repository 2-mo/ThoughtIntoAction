#!/usr/bin/env python3
"""Generate a matched URDF/MJCF pair from the calibrated X5 source model."""
from pathlib import Path
import argparse
import hashlib
import json
import shutil
import sys
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from arx5_vr.types import RobotConfig


def numbers(values):
    return ' '.join(f'{v:.15g}' for v in values)


def rotation(rpy):
    q = Rotation.from_euler('xyz', np.fromstring(rpy, sep=' ')).as_quat()
    return numbers(q[[3, 0, 1, 2]])


def vector(value, size, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f'{name} must contain {size} finite numbers')
    return result


def tensor(values):
    xx, yy, zz, xy, xz, yz = values
    return np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])


def read_tool(path):
    tool = json.loads(Path(path).read_text())
    keys = {'schema_version', 'name', 'parent_link', 'tcp_link', 'tcp_xyz', 'tcp_rpy', 'calibration_status', 'payload'}
    if not isinstance(tool, dict) or set(tool) != keys:
        raise ValueError('tool.json must contain exactly the documented schema keys')
    if tool['schema_version'] != 1 or tool['parent_link'] != 'link6' or tool['tcp_link'] != 'tcp_link':
        raise ValueError('tool schema 1 requires parent_link=link6 and tcp_link=tcp_link')
    if not isinstance(tool['name'], str) or not tool['name'].strip():
        raise ValueError('tool name must be a nonempty string')
    if tool['calibration_status'] not in ('simulation_geometry_provisional', 'sdk_nominal', 'measured'):
        raise ValueError('calibration_status must be simulation_geometry_provisional, sdk_nominal or measured')
    vector(tool['tcp_xyz'], 3, 'tcp_xyz')
    vector(tool['tcp_rpy'], 3, 'tcp_rpy')
    payload = tool['payload']
    if not isinstance(payload, dict) or set(payload) != {'mass_kg', 'com_xyz', 'inertia'}:
        raise ValueError('payload requires mass_kg, com_xyz and inertia')
    mass = float(payload['mass_kg'])
    if not np.isfinite(mass) or mass < 0:
        raise ValueError('payload mass_kg must be finite and nonnegative')
    vector(payload['com_xyz'], 3, 'payload com_xyz')
    inertia = tensor(vector(payload['inertia'], 6, 'payload inertia'))
    eigenvalues = np.linalg.eigvalsh(inertia)
    if eigenvalues[0] < -1e-12 or eigenvalues[-1] > sum(eigenvalues[:2]) + 1e-12:
        raise ValueError('payload inertia must be positive semidefinite and satisfy the triangle inequality')
    if mass == 0 and np.any(inertia != 0):
        raise ValueError('zero payload mass requires zero inertia')
    return tool


def add_payload(link, tool):
    """Fold only the additional payload into link6, in link6 axes about its COM."""
    payload = tool['payload']
    m1 = float(payload['mass_kg'])
    if m1 == 0:
        return
    inertial = link.find('inertial')
    origin = inertial.find('origin')
    m0 = float(inertial.find('mass').get('value'))
    c0 = np.fromstring(origin.get('xyz'), sep=' ')
    axes0 = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
    source_tensor = inertial.find('inertia')
    i0 = axes0 @ tensor([float(source_tensor.get(key)) for key in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz')]) @ axes0.T
    axes1 = Rotation.from_euler('xyz', tool['tcp_rpy']).as_matrix()
    c1 = np.asarray(tool['tcp_xyz']) + axes1 @ payload['com_xyz']
    i1 = axes1 @ tensor(payload['inertia']) @ axes1.T
    centre = (m0 * c0 + m1 * c1) / (m0 + m1)
    combined = i0 + i1
    for mass, offset in ((m0, c0 - centre), (m1, c1 - centre)):
        combined += mass * (offset @ offset * np.eye(3) - np.outer(offset, offset))
    inertial.find('mass').set('value', numbers([m0 + m1]))
    origin.set('xyz', numbers(centre))
    origin.set('rpy', '0 0 0')
    for key, value in zip(('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'),
                          (combined[0, 0], combined[1, 1], combined[2, 2], combined[0, 1], combined[0, 2], combined[1, 2])):
        source_tensor.set(key, numbers([value]))


def build(source, meshes, *, tool=None, output_dir=None):
    source, meshes = Path(source), Path(meshes)
    tool_path = Path(tool) if tool is not None else ROOT / 'config/tool.json'
    tool_config = read_tool(tool_path)
    assets = Path(output_dir) if output_dir is not None else ROOT / 'models/x5'
    config = RobotConfig()
    source_root = ET.parse(source).getroot()
    if source_root.find("link[@name='tcp_link']") is not None:
        raise ValueError('source already contains tcp_link; rebuild from the original .source.urdf to avoid adding payload twice')
    # The tested Stanford source includes a massless eef_link. Replace that
    # marker with tool.json's fingertip TCP, preserving the six-link dynamics.
    eef = source_root.find("link[@name='eef_link']")
    if eef is not None:
        fixed = source_root.find("joint[@name='gripper_fixed_joint']")
        if (fixed is None or fixed.get('type') != 'fixed'
                or fixed.find('parent').get('link') != 'link6'
                or fixed.find('child').get('link') != 'eef_link'
                or float(eef.find('inertial/mass').get('value')) != 0
                or any(float(v) != 0 for v in eef.find('inertial/inertia').attrib.values())
                or source_root.find("joint/parent[@link='eef_link']") is not None):
            raise ValueError('only the massless Stanford eef_link marker may be replaced')
        source_root.remove(eef)
        source_root.remove(fixed)
    active = source_root.findall('joint')
    links = ['base_link'] + [f'link{i}' for i in range(1, 7)]
    valid_chain = [link.get('name') for link in source_root.findall('link')] == links
    valid_chain = valid_chain and len(active) == 6
    for i, joint in enumerate(active):
        parent, child = joint.find('parent'), joint.find('child')
        valid_chain = valid_chain and (
            i < 6 and joint.get('name') == f'joint{i+1}' and joint.get('type') == 'revolute'
            and parent is not None and parent.get('link') == links[i]
            and child is not None and child.get('link') == links[i+1])
    if not valid_chain:
        raise ValueError('source must be the original six-link arm chain base_link/link1..link6; use tool.json for TCP/payload or explicitly adapt the generator and SDK')
    # Source assets stay immutable. Only this parsed output copy receives payload.
    add_payload(source_root.find("link[@name='link6']"), tool_config)
    tcp_body = ET.SubElement(source_root, 'link', name=tool_config['tcp_link'])
    fixed = ET.SubElement(source_root, 'joint', name='tool_tcp_fixed', type='fixed')
    ET.SubElement(fixed, 'origin', xyz=numbers(tool_config['tcp_xyz']), rpy=numbers(tool_config['tcp_rpy']))
    ET.SubElement(fixed, 'parent', link=tool_config['parent_link'])
    ET.SubElement(fixed, 'child', link=tcp_body.get('name'))
    (assets / 'meshes').mkdir(parents=True, exist_ok=True)
    provenance = assets / 'PROVENANCE.json'
    previous = json.loads(provenance.read_text()) if provenance.exists() else {}
    previous_sources = {(entry['bundled'], entry['sha256']): entry
                        for entry in previous.get('sources', [])}
    manifest = {'variant': 'AC one / ARX X5 2025', 'tcp': 'tcp_link; fixed transform from tool.json', 'sources': []}

    def bundle(path, destination):
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        # Reusing bundled bytes must not replace their original attribution
        # with the local output directory merely because the model was rebuilt.
        entry = previous_sources.get((destination, digest))
        manifest['sources'].append(entry if entry is not None else
                                   {'source': str(path), 'sha256': digest, 'bundled': destination})
        target = assets / destination
        if path.resolve() != target.resolve():
            shutil.copyfile(path, target)

    bundle(source, 'x5_2025.source.urdf')
    for mesh in source_root.findall('.//mesh'):
        name = Path(mesh.get('filename')).name
        mesh.set('filename', f'meshes/{name}')
    for i, joint in enumerate(active):
        joint.find('limit').set('lower', str(config.joint_min[i]))
        joint.find('limit').set('upper', str(config.joint_max[i]))
    ET.indent(source_root)
    ET.ElementTree(source_root).write(assets / 'x5_2025.urdf', encoding='utf-8', xml_declaration=True)
    root = ET.Element('mujoco', model='AC one X5 2025')
    ET.SubElement(root, 'compiler', angle='radian', meshdir='meshes', inertiafromgeom='false')
    ET.SubElement(root, 'option', timestep='.004', gravity='0 0 -9.807', integrator='implicitfast')
    defaults = ET.SubElement(root, 'default')
    ET.SubElement(defaults, 'geom', contype='0', conaffinity='0', rgba='.72 .76 .8 1')
    assets_node = ET.SubElement(root, 'asset')
    for link in source_root.findall('link'):
        name = link.get('name')
        if link.find('visual') is None:
            continue
        file = meshes / f'{name}.STL'
        bundle(file, f'meshes/{file.name}')
        ET.SubElement(assets_node, 'mesh', name=name, file=file.name)
    world = ET.SubElement(root, 'worldbody')
    ET.SubElement(world, 'light', pos='0 -1 2', dir='0 0 -1')
    ET.SubElement(world, 'geom', name='ground', type='plane', size='1.5 1.5 .05', pos='0 0 -.01', rgba='.16 .19 .24 1')
    ET.SubElement(world, 'camera', name='overview', pos='1 -1 .8', xyaxes='.707 .707 0 -.35 .35 .87')
    bodies = {}
    joints = {j.find('child').get('link'): j for j in source_root.findall('joint')}
    for link in source_root.findall('link'):
        name = link.get('name')
        joint = joints.get(name)
        parent = world if joint is None else bodies[joint.find('parent').get('link')]
        body = ET.SubElement(parent, 'body', name=name)
        bodies[name] = body
        if joint is not None:
            origin = joint.find('origin')
            body.set('pos', origin.get('xyz'))
            body.set('quat', rotation(origin.get('rpy')))
            if joint.get('type') != 'fixed':
                limit = joint.find('limit')
                ET.SubElement(body, 'joint', name=joint.get('name'), type='hinge', axis=joint.find('axis').get('xyz'), range=f"{limit.get('lower')} {limit.get('upper')}", limited='true')
        inertial = link.find('inertial')
        if inertial is not None:
            inertia = inertial.find('inertia')
            # The official source and the merged payload use link-aligned axes.
            if inertial.find('origin').get('rpy') != '0 0 0':
                raise ValueError('source inertial axes must be link-aligned; express tool inertia through tool.json')
            ET.SubElement(body, 'inertial', mass=inertial.find('mass').get('value'), pos=inertial.find('origin').get('xyz'), fullinertia=' '.join(inertia.get(k) for k in ['ixx','iyy','izz','ixy','ixz','iyz']))
        if link.find('visual') is not None:
            ET.SubElement(body, 'geom', type='mesh', mesh=name)
    last = bodies['link6']
    ET.SubElement(bodies[tool_config['tcp_link']], 'site', name='tcp', pos='0 0 0', size='.008', rgba='1 .3 .1 1')
    # Two 1 g display jaws are additional approximate bodies; original link inertials stay intact.
    for side, sign in [('left', 1), ('right', -1)]:
        jaw = ET.SubElement(last, 'body', name=f'jaw_{side}', pos=f'.11 {sign * .004} 0')
        ET.SubElement(jaw, 'joint', name=f'gripper_{side}', type='slide', axis=f'0 {sign} 0', range=f'0 {config.gripper_max_width / 2}', limited='true')
        ET.SubElement(jaw, 'inertial', mass='.001', pos='0 0 0', diaginertia='1e-6 1e-6 1e-6')
        ET.SubElement(jaw, 'geom', type='box', size='.035 .004 .012', rgba='.18 .2 .24 1')
    actuators = ET.SubElement(root, 'actuator')
    for i in range(6):
        ET.SubElement(actuators, 'general', name=f'motor{i+1}', joint=f'joint{i+1}', gainprm='1', biastype='affine', biasprm=f'0 {-config.kp[i]} {-config.kd[i]}', forcelimited='true', forcerange=f'{-config.torque_max[i]} {config.torque_max[i]}')
    for side in ['left', 'right']:
        ET.SubElement(actuators, 'general', name=f'jaw_motor_{side}', joint=f'gripper_{side}', gainprm='1', biastype='affine', biasprm='0 -80 -1', forcelimited='true', forcerange='-5 5')
    ET.indent(root)
    ET.ElementTree(root).write(assets / 'x5_2025.xml', encoding='utf-8', xml_declaration=True)
    (assets / 'PROVENANCE.json').write_text(json.dumps(manifest, indent=2) + '\n')
    model_manifest = {key: tool_config[key] for key in ('schema_version', 'parent_link', 'tcp_link', 'calibration_status', 'tcp_xyz', 'tcp_rpy')}
    model_manifest['schema_version'] = 2
    model_manifest['base_link'] = 'base_link'
    model_manifest['tool_sha256'] = hashlib.sha256(tool_path.read_bytes()).hexdigest()
    model_manifest['sha256'] = {name: hashlib.sha256((assets / name).read_bytes()).hexdigest()
                                for name in ('x5_2025.urdf', 'x5_2025.xml')}
    (assets / 'x5_2025.model.json').write_text(json.dumps(model_manifest, indent=2) + '\n')
    return assets / 'x5_2025.xml'


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--urdf', type=Path, default=ROOT / 'models/x5/x5_2025.source.urdf',
                        help='source URDF (default: bundled calibrated Stanford X5 URDF)')
    parser.add_argument('--meshes', type=Path, default=ROOT / 'models/x5/meshes',
                        help='mesh directory (default: bundled meshes)')
    parser.add_argument('--tool', type=Path, default=ROOT / 'config/tool.json',
                        help='TCP transform and additional payload configuration')
    parser.add_argument('--output-dir', type=Path, default=ROOT / 'models/x5',
                        help='output directory containing the matching URDF/MJCF pair')
    args = parser.parse_args()
    print(build(args.urdf, args.meshes, tool=args.tool, output_dir=args.output_dir))
