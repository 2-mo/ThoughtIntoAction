"""Episode format identity (todo D-04).

`schema_version` numbers collide across formats (the legacy teleop episode, GR00T raw episodes and
tuning logs all write 1), so formats are told apart by a namespaced id. New writers record it as
`schema`; older episodes are identified by `kind` and layout.
"""
import json
from pathlib import Path

TELEOP = 'x5.teleop/1'     # legacy arm service recording: quest_teleop_episode, MCAP + RGB
RUNTIME = 'x5.runtime/2'   # arx kernel recording: quest_runtime_episode / _trace, MCAP + RGB
GROOT_RAW = 'groot.raw/1'  # GR00T dual-camera raw episode (exported with the GR00T exporter)
KNOWN = (TELEOP, RUNTIME, GROOT_RAW)
LEROBOT_EXPORTERS = {TELEOP: 'schema1', RUNTIME: 'schema2'}  # registry names of the LeRobot embodied.devices.robots.arx5


def identify(manifest):
    if not isinstance(manifest, dict):
        raise ValueError('episode.json is not an object')
    if 'schema' in manifest:
        if manifest['schema'] not in KNOWN:
            raise ValueError(f'Unknown episode format {manifest["schema"]!r}')
        return manifest['schema']
    kind, version = manifest.get('kind'), manifest.get('schema_version')
    if kind == 'quest_teleop_episode' and version == 1:
        return TELEOP
    if kind in ('quest_runtime_episode', 'quest_runtime_trace') and version == 2:
        return RUNTIME
    if kind is None and version == 1 and {'session_id', 'stage'} <= manifest.keys():
        return GROOT_RAW
    raise ValueError(f'Unknown episode format (kind={kind!r}, schema_version={version!r})')


def read(path):
    """(format id, manifest) of an episode directory."""
    manifest = json.loads((Path(path) / 'episode.json').read_text())
    return identify(manifest), manifest
