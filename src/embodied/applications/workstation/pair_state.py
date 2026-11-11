"""Read the pair's deployment identity; no device access or control requests."""
import json
import os
from pathlib import Path

from embodied.contracts.paths import PROJECT_ROOT

PAIR_STATE = PROJECT_ROOT/'outputs/runtime/realman/pair.json'


def active_pair(path=PAIR_STATE):
    try:
        state = json.loads(Path(path).read_text())
        boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        if state.get('schema') != 1 or state.get('boot_id') != boot:
            return None
        os.kill(state['pid'], 0)
        if set(state['runs']) != {'left', 'right'}:
            return None
        for side, value in state['runs'].items():
            run = Path(value)
            identity = json.loads((run/'runtime.json').read_text())
            pair = json.loads((run/'workbench.json').read_text()).get('pair', {})
            if (identity.get('boot_id') != boot or not (run/'runtime.ready').exists()
                    or pair.get('id') != state['runtime_id'] or pair.get('side') != side):
                return None
            os.kill(identity['supervisor_pid'], 0)
        return state
    except (OSError, ValueError, KeyError, TypeError):
        return None


def selected_run(pair=None):
    """Explicit workbench target shared by terminal, phone and console.

    A new pair starts on the labelled left arm. Never infer a side from timestamps.
    """
    pair = active_pair() if pair is None else pair
    if pair is None:
        return None
    side = 'left'
    try:
        selection = json.loads((Path(pair['runs']['left']).parent / (pair['runtime_id'] + '-selection.json')).read_text())
        if selection.get('runtime_id') == pair['runtime_id'] and selection.get('side') in pair['runs']:
            side = selection['side']
    except (OSError, ValueError, TypeError):
        pass
    return side, Path(pair['runs'][side])


def select_side(side):
    pair = active_pair()
    if pair is None or side not in pair['runs']:
        raise ValueError('没有可选择的双臂运行实例')
    path = Path(pair['runs']['left']).parent / (pair['runtime_id'] + '-selection.json')
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps({'runtime_id': pair['runtime_id'], 'side': side}))
    temporary.replace(path)
    return selected_run(pair)
