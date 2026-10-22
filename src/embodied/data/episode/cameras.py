"""Camera identities and independent source clocks shared by recording and export."""
import re


def camera_metadata(manifest):
    cameras = manifest.get('cameras')
    if cameras is None:
        cameras = {'wrist_rgb': manifest['camera']} if manifest.get('camera') else {}
    if not isinstance(cameras, dict):
        raise ValueError('Recording cameras must map IDs to metadata')
    for name, meta in cameras.items():
        if not re.fullmatch(r'[a-z][a-z0-9_]*_rgb', name) or not isinstance(meta, dict):
            raise ValueError('Invalid recorded camera identity')
    if manifest.get('camera') is not None and manifest.get('camera') != cameras.get('wrist_rgb'):
        raise ValueError('Legacy wrist metadata differs from cameras.wrist_rgb')
    return cameras


def anchor_camera(manifest):
    """The explicit reference camera, with wrist ordering only for old recordings."""
    cameras = camera_metadata(manifest)
    selected = manifest.get('anchor_camera')
    if selected is None:
        selected = 'wrist_rgb' if 'wrist_rgb' in cameras else next(iter(cameras), None)
    if selected not in cameras:
        raise ValueError('Episode needs a declared reference camera')
    return selected


def camera_topic(camera_id):
    if not re.fullmatch(r'[a-z][a-z0-9_]*_rgb', camera_id):
        raise ValueError('Invalid camera ID')
    return '/camera/' + camera_id[:-4] + '/rgb/index'


def camera_id_from_topic(topic):
    match = re.fullmatch(r'/camera/([a-z][a-z0-9_]*)/rgb/index', topic)
    return match[1] + '_rgb' if match else None
