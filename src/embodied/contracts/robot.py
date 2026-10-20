"""X5 robot profile from the arx kernel's own configuration (todo R-01).

control/config/robot.json and workspace.json are the single source. Platform code that
needs limits without a running kernel reads them here; a running kernel reports the same
values through the `describe` action (embodied.contracts.runtime.ControlClient.describe).
"""
import hashlib
import json
from pathlib import Path

from embodied.contracts.registry import PROJECT_ROOT

CONFIG = PROJECT_ROOT / 'control/config'


def load_robot_profile(directory=CONFIG):
    directory = Path(directory)
    source = directory / 'robot.json'
    data = source.read_bytes()
    robot = json.loads(data)
    for key in ('model_path', 'tool_path'):
        robot[key] = str((directory / robot[key]).resolve())
    workspace_path = directory / 'workspace.json'
    workspace = json.loads(workspace_path.read_text()) if workspace_path.is_file() else {'table_enabled': False}
    minimum = workspace['table_z_m'] + workspace['table_clearance_m'] if workspace['table_enabled'] else None
    return {'robot': robot, 'workspace': {'minimum_tcp_z_m': minimum},
            'source': str(source), 'sha256': hashlib.sha256(data).hexdigest()}
