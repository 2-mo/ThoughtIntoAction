"""Display RealMan feedback without opening a robot, XR SDK or command channel."""
from copy import deepcopy
import hashlib
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

from embodied.core.project import PROJECT_ROOT
from embodied.core.telemetry import SCHEMA
from embodied.devices.robots.realman.telemetry import latest_state
from .model import RuntimeModel


def display_model(hands, directory=None):
    """Two nominal arms in an explicit display layout, not a calibrated cell."""
    source = PROJECT_ROOT / 'assets/robots/realman/runtime/rm65.urdf'
    directory = Path(directory or PROJECT_ROOT / 'outputs/runtime/realman/models')
    directory.mkdir(parents=True, exist_ok=True)
    root = ET.Element('robot', name='realman_display_layout')
    ET.SubElement(root, 'link', name='display_world')
    names = []
    for hand in hands:
        tree = ET.parse(source).getroot()
        prefix = hand + '_'
        for mesh in tree.findall('.//mesh'):
            mesh.set('filename', str((source.parent / mesh.get('filename')).resolve()))
        for element in tree:
            if element.tag not in ('link', 'joint'):
                continue
            element.set('name', prefix + element.get('name'))
            if element.tag == 'joint':
                for ref in ('parent', 'child'):
                    element.find(ref).set('link', prefix + element.find(ref).get('link'))
                if element.get('type') != 'fixed':
                    names.append(element.get('name'))
            root.append(element)
        joint = ET.SubElement(root, 'joint', name=prefix + 'display_mount', type='fixed')
        ET.SubElement(joint, 'parent', link='display_world')
        ET.SubElement(joint, 'child', link=prefix + 'base_link')
        ET.SubElement(joint, 'origin', xyz=f'0 {0.4 if hand == "left" else -0.4} 0', rpy='0 0 0')
    content = ET.tostring(root, encoding='unicode')
    digest = hashlib.sha256(content.encode()).hexdigest()[:16]
    path = directory / f'realman-{digest}.urdf'
    if not path.exists():
        path.write_text(content)
    descriptor = {'joint_count': len(names), 'data_contract': {
        'schema': 'embodied.joint_vectors/1', 'representation': 'absolute_joint_position',
        'joint_names': names, 'joint_unit': 'rad', 'gripper': None,
        'robot_type': 'realman_hardware_display_uncalibrated_layout'},
        'parameters': {'model': {'urdf_path': str(path), 'urdf': content,
            'manifest': {'tcp_link': hands[0] + '_tcp_link'}}}}
    return RuntimeModel(descriptor)


class RealManView:
    def __init__(self, *, reader=None, model_factory=display_model):
        self.reader, self.model_factory = reader or latest_state, model_factory
        self.latest = None
        self.stream_id = None

    def step(self, scene):
        state = self.reader()
        if state is None:
            if self.latest is not None and scene:
                scene.set_model(None)
            self.latest = None
            self.stream_id = None
            return False
        self.latest = state
        if scene is None:
            return True
        arms = sorted(state['arms'], key=lambda arm: arm['hand'])
        if self.stream_id != state['stream_id'] or scene.robot is None:
            scene.set_model(self.model_factory([a['hand'] for a in arms]))
            scene._manifest['model_file'] = 'RealMan RM65 双臂 · 底座位置为示意布局'
            scene._manifest['display_layout'] = 'uncalibrated; bases separated for visibility'
            self.stream_id = state['stream_id']
        actual = [q for arm in arms for q in arm['q']]
        target = [q for arm in arms for q in
            ([math.radians(v) for v in arm['target_q_deg']] if arm['target_q_deg'] is not None else arm['q'])]
        reasons = {'xr_stale': 'XR 数据中断，等待恢复后松键并按 B',
            'posture_rejected': '当前姿态不可遥操；用示教器调整后松键按 B 重新检查',
            'waiting_for_B': '等待按 B 开始', 'paused_by_button': '已暂停，按 B 继续',
            'target_rejected': '目标不可达或进入奇异区，双臂已暂停；松键再按 B',
            'release_grip_and_gripper_buttons': '先松开 Grip、X/A 和 Trigger，再按 B', 'active': '手柄控制已开始'}
        reason = reasons.get(state['reason'], state['reason'])
        stamp = min(state['sample_monotonic_ns'], *[int(a['feedback_at'] * 1e9) for a in arms])
        scene.accept({'schema': SCHEMA, 'robot': scene.robot.metadata(), 'stream_id': state['stream_id'],
            'sequence': state['sequence'], 'source': {'kind': state.get('source_kind', 'hardware'), 'input': 'quest',
                'controller': 'RealMan 双臂 · ' + reason},
            'actual': actual, 'target': target, 'gripper': None,
            'sample_monotonic_ns': stamp, 'emitted_monotonic_ns': time.monotonic_ns(), 'paths': {},
            'command_details': {'realman': deepcopy(arms), 'xr_fresh': state['xr_fresh'],
                'paused': state['paused'], 'reason': reason}})
        return True

    def monitor(self):
        state = deepcopy(self.latest)
        if state is None:
            return None
        return {'runtime_id': state['stream_id'], 'state_age_s': max(0.,
            (time.monotonic_ns() - state['sample_monotonic_ns']) / 1e9),
            'input_age_s': None, 'input_error': None if state['xr_fresh'] else 'XR数据未更新',
            'monitor_message': 'RealMan 双臂只读反馈；底座为示意布局',
            'realman': state, 'input': {'xr_fresh': state['xr_fresh'],
                'control_enabled': not state['paused'], 'reason': state['reason']}}
