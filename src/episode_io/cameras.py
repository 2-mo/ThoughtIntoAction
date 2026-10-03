"""Camera identities and independent source clocks shared by recording and export."""
import re


def camera_metadata(manifest):
    cameras = manifest.get('cameras')
    if cameras is None:
        return {'wrist_rgb': manifest.get('camera', {})}
    if not isinstance(cameras, dict) or 'wrist_rgb' not in cameras:
        raise ValueError('Recording must include wrist_rgb metadata')
    for name, meta in cameras.items():
        if not re.fullmatch(r'[a-z][a-z0-9_]*_rgb', name) or not isinstance(meta, dict):
            raise ValueError('Invalid recorded camera identity')
    if manifest.get('camera') != cameras['wrist_rgb']:
        raise ValueError('Legacy wrist metadata differs from cameras.wrist_rgb')
    return cameras


def camera_topic(camera_id):
    if not re.fullmatch(r'[a-z][a-z0-9_]*_rgb', camera_id):
        raise ValueError('Invalid camera ID')
    return '/camera/' + camera_id[:-4] + '/rgb/index'


def camera_id_from_topic(topic):
    match = re.fullmatch(r'/camera/([a-z][a-z0-9_]*)/rgb/index', topic)
    return match[1] + '_rgb' if match else None
