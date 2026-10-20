"""Explicit camera lifecycle and recording exclusion, without opening camera SDKs."""
from .project import load_config, resolve_path
from .camera_client import socket_path, ensure_service
from .session_protocol import request
from embodied_contracts.device_lease import DeviceLease


def recording_guard():
    return DeviceLease('camera-recording:' + str(socket_path()))


def set_enabled(enabled, camera_id=None):
    if type(enabled) is not bool:
        raise ValueError('Expected camera enabled boolean')
    ensure_service()
    arguments = {} if camera_id is None else {'camera_id': camera_id}
    return request(socket_path(), 'set_enabled', enabled=enabled, timeout=6, **arguments)


def check_legacy_session():
    """Old sessions lack the recording guard and cannot safely cooperate with shutdown."""
    try:
        info = request(resolve_path(load_config()['runtime']['socket']), 'monitor', timeout=.5)
    except (FileNotFoundError, ConnectionRefusedError):
        return
    if info.get('recording'):
        raise RuntimeError('正在录制，不能关闭相机')
    if info.get('camera_switch_protocol') != 1:
        raise RuntimeError('当前遥操版本不支持相机开关；请先退出遥操，再关闭相机')
