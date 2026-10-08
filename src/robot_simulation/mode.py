"""Shared terminal selection, persisted independently of the viewer process."""
from robot_core.project import load_config, resolve_path

MODES = {'real': '真机', 'simulation': '仿真 · MuJoCo / Quest'}


def mode_path():
    socket = resolve_path(load_config().get('viewer_service', {}).get('socket', 'outputs/runtime/viewer.sock'))
    return socket.with_name('workbench-mode.txt')


def selected_mode():
    try:
        mode = mode_path().read_text().strip()
    except FileNotFoundError:
        return 'real'
    if mode not in MODES:
        raise ValueError('工作台模式配置无效')
    return mode


def save_mode(mode):
    if mode not in MODES:
        raise ValueError('模式必须是 real 或 simulation')
    path = mode_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(mode + '\n')
    temporary.replace(path)
