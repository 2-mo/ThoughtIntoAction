"""Run in OpenPI's environment: run.py OPENPI_CHECKOUT EXPORTED_DATASET norm|train [options]."""
import os
import json
from pathlib import Path
import runpy
import sys


def main():
    checkout, dataset, command, *arguments = sys.argv[1:]
    checkout, dataset = Path(checkout).resolve(), Path(dataset).resolve()
    if command not in ('norm', 'train') or not (dataset/'meta'/'quest_export.json').is_file():
        raise SystemExit('Use: run.py OPENPI_CHECKOUT EXPORTED_DATASET norm|train [options]')
    cache = Path(__file__).resolve().parents[2]/'outputs'/'openpi-datasets'
    link = cache/'local'/'quest_x5'
    link.parent.mkdir(parents=True, exist_ok=True)
    if link.exists() or link.is_symlink():
        if link.resolve() != dataset:
            raise SystemExit(f'{link} already refers to another dataset; choose the intended dataset before training')
    else:
        link.symlink_to(dataset, target_is_directory=True)
    os.environ['HF_LEROBOT_HOME'] = str(cache)
    sys.path.insert(0, str(checkout/'src'))
    os.chdir(checkout)
    from openpi.training import config
    from quest_x5 import train_config
    info = json.loads((dataset/"meta"/"info.json").read_text())
    layout = json.loads((dataset/"meta"/"quest_export.json").read_text())
    if layout.get("layout") not in {"aligned30_raw240_v1", "aligned30_raw250_v2"} or info["fps"] != 30:
        raise SystemExit("Re-export with the versioned 30 FPS converter first")
    if layout.get('layout') == 'aligned30_raw250_v2' and layout.get('action_definition') != 'sent_joint_position + sent_gripper_width':
        raise SystemExit('Unsupported runtime action definition')
    item = train_config(fps=info["fps"])
    config._CONFIGS_DICT[item.name] = item
    script = checkout/'scripts'/('compute_norm_stats.py' if command == 'norm' else 'train.py')
    sys.argv = [str(script), *(['--config-name', item.name] if command == 'norm' else [item.name]), *arguments]
    runpy.run_path(str(script), run_name='__main__')


if __name__ == '__main__':
    main()
