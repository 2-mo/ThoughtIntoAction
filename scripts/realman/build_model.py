"""Build portable RM65 MuJoCo/display models from the pinned official URDF.

Run with outputs/envs/control/bin/python. No download, ROS or robot connection.
"""
from pathlib import Path
import hashlib
import json
from xml.etree import ElementTree as E

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
ASSETS = ROOT / 'assets/robots/realman'


def numbers(values):
    return ' '.join(format(float(value), '.16g') for value in values)


def origin(element):
    node = element.find('origin')
    if node is None:
        return {}
    rpy = np.fromstring(node.get('rpy', '0 0 0'), sep=' ')
    xyzw = Rotation.from_euler('xyz', rpy).as_quat()
    return {'pos': node.get('xyz', '0 0 0'), 'quat': numbers(xyzw[[3, 0, 1, 2]])}


def build():
    provenance = json.loads((ASSETS / 'UPSTREAM.json').read_text())
    for name, record in provenance['files'].items():
        if hashlib.sha256((ASSETS / name).read_bytes()).hexdigest() != record['sha256']:
            raise ValueError(f'Upstream source changed: {name}')
    source = E.parse(ASSETS / 'upstream/rm_65.urdf').getroot()
    deployment = json.loads((ROOT / 'configs/robots/realman/sim.json').read_text())
    width = deployment['gripper_max_width']
    tool_z = deployment['tool_offset_m']
    scene = E.Element('mujoco', model='rm65_lq_simplified_gripper')
    E.SubElement(scene, 'compiler', angle='radian', autolimits='true')
    E.SubElement(scene, 'option', timestep='0.002', gravity='0 0 -9.81', integrator='implicitfast')
    asset = E.SubElement(scene, 'asset')
    world = E.SubElement(scene, 'worldbody')
    E.SubElement(world, 'light', pos='1 -1 3', diffuse='.8 .8 .8')
    E.SubElement(world, 'geom', name='floor', type='plane', size='1.5 1.5 .05', rgba='.22 .25 .28 1')
    links = {link.get('name'): link for link in source.findall('link')}
    children = {joint.find('child').get('link'): joint for joint in source.findall('joint')}
    bodies = {}
    for name, link in links.items():
        joint = children.get(name)
        parent = world if joint is None else bodies[joint.find('parent').get('link')]
        body = E.SubElement(parent, 'body', name=name, **({} if joint is None else origin(joint)))
        bodies[name] = body
        inertial = link.find('inertial')
        inertia = inertial.find('inertia')
        inertial_origin = origin(inertial)
        if not np.allclose(np.fromstring(inertial_origin['quat'], sep=' '), [1, 0, 0, 0]):
            raise ValueError('Expected official RM65 inertia in link axes')
        E.SubElement(body, 'inertial', pos=inertial_origin['pos'], mass=inertial.find('mass').get('value'),
                     fullinertia=' '.join(inertia.get(k) for k in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz')))
        if joint is not None:
            limit = joint.find('limit')
            E.SubElement(body, 'joint', name=joint.get('name'), type='hinge', axis=joint.find('axis').get('xyz'),
                         range=f"{limit.get('lower')} {limit.get('upper')}", damping='.1', armature='.03')
        mesh_name = Path(link.find('visual/geometry/mesh').get('filename')).name
        E.SubElement(asset, 'mesh', name=name, file='../meshes/' + mesh_name)
        E.SubElement(body, 'geom', name='visual_' + name, type='mesh', mesh=name,
                     rgba='.85 .87 .9 1', contype='0', conaffinity='0')
    flange = bodies['Link6']
    tool = E.SubElement(flange, 'body', name='simplified_gripper', pos='0 0 0')
    E.SubElement(tool, 'geom', name='palm', type='box', pos='0 0 .055', size='.035 .025 .055',
                 mass='.46', rgba='.2 .25 .3 1', contype='0', conaffinity='0')
    for name, direction in (('left', 1), ('right', -1)):
        jaw = E.SubElement(tool, 'body', name='jaw_' + name, pos=f'{direction * .005} 0 {tool_z - .02}')
        E.SubElement(jaw, 'joint', name='jaw_' + name, type='slide', axis=f'{direction} 0 0',
                     range=f'0 {width / 2}', damping='1', armature='.01')
        E.SubElement(jaw, 'geom', type='box', size='.005 .008 .02', mass='.02',
                     rgba='.15 .18 .21 1', contype='0', conaffinity='0')
    tcp = E.SubElement(flange, 'body', name='tcp_link', pos=f'0 0 {tool_z}')
    E.SubElement(tcp, 'site', name='tcp', type='sphere', size='.007', rgba='1 .4 .1 1')
    actuators = E.SubElement(scene, 'actuator')
    for joint in source.findall('joint'):
        name = joint.get('name'); effort = joint.find('limit').get('effort')
        E.SubElement(actuators, 'motor', name='motor_' + name, joint=name, ctrlrange=f'-{effort} {effort}')
    for name in ('left', 'right'):
        E.SubElement(actuators, 'motor', name='motor_jaw_' + name, joint='jaw_' + name, ctrlrange='-20 20')
    # Display URDF has only the six arm axes, as required by RuntimeModel.
    # Jaw width is shown in the MuJoCo viewer and telemetry; no fixed fake jaws.
    for mesh in source.findall('.//mesh'):
        mesh.set('filename', '../meshes/' + Path(mesh.get('filename')).name)
    palm = E.SubElement(source, 'link', name='tool_palm')
    visual = E.SubElement(palm, 'visual'); E.SubElement(visual, 'origin', xyz='0 0 .055')
    E.SubElement(E.SubElement(visual, 'geometry'), 'box', size='.07 .05 .11')
    fixed = E.SubElement(source, 'joint', name='tool_palm_fixed', type='fixed')
    E.SubElement(fixed, 'parent', link='Link6'); E.SubElement(fixed, 'child', link='tool_palm')
    E.SubElement(source, 'link', name='tcp_link')
    fixed = E.SubElement(source, 'joint', name='tool_tcp_fixed', type='fixed')
    E.SubElement(fixed, 'parent', link='Link6'); E.SubElement(fixed, 'child', link='tcp_link')
    E.SubElement(fixed, 'origin', xyz=f'0 0 {tool_z}', rpy='0 0 0')
    runtime = ASSETS / 'runtime'; runtime.mkdir(exist_ok=True)
    for name, element in (('rm65.xml', scene), ('rm65.urdf', source)):
        E.indent(element)
        E.ElementTree(element).write(runtime / name, encoding='unicode', xml_declaration=True)
    files = [*sorted((ASSETS / 'meshes').glob('*.STL')), ASSETS / 'upstream/rm_65.urdf',
             ASSETS / 'UPSTREAM.json', runtime / 'rm65.xml', runtime / 'rm65.urdf']
    manifest = {'schema_version': 1, 'tcp_link': 'tcp_link', 'upstream_commit': provenance['commit'],
                'calibration_status': 'official nominal RM65; not calibrated to the physical arm',
                'tool_offset_m': tool_z, 'simplified_gripper_width_m': width,
                'sha256': {str(p.relative_to(ASSETS)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}}
    (runtime / 'rm65.model.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(runtime / 'rm65.xml')


if __name__ == '__main__':
    build()
