"""Run in OpenPI's environment: run.py OPENPI_CHECKOUT EXPORTED_DATASET norm|train [options]."""
import os
import json
from pathlib import Path
import runpy
import subprocess
import sys

# Checkpoints store normalization under assets/<repo_id>/, so the asset id records which action
# definition a model was trained on: legacy requested targets or submitted runtime commands.
# Historical layout names remain readable; their numeric suffix is not a measured sample rate.
LAYOUTS = {'aligned30_raw240_v1': ('local/quest_x5', 'pi05_quest_x5'),
           'aligned30_raw250_v2': ('local/x5_kernel', 'pi05_x5_kernel'),
           'aligned30_runtime_v2': ('local/x5_kernel', 'pi05_x5_kernel')}


def dataset_config(layout, info):
    if layout.get('layout') not in LAYOUTS or info.get('fps') != 30:
        raise SystemExit('Re-export with the versioned 30 FPS converter first')
    if layout.get('data_contract') is not None and layout['layout'] != 'aligned30_raw240_v1':
        from embodied.policy.openpi.adapter import from_dataset, asset_id
        try:
            spec = from_dataset(layout, info)
        except ValueError as exc:
            raise SystemExit(str(exc)) from exc
        repo = asset_id(spec)
        return repo, 'pi05_' + repo.split('/')[-1]
    if layout['layout'] in ('aligned30_raw250_v2', 'aligned30_runtime_v2') and layout.get('action_definition') != 'sent_joint_position + sent_gripper_width':
        raise SystemExit('Unsupported runtime action definition for the X5 training adapter')
    features = info.get('features', {})
    if (any(tuple(features.get(key, {}).get('shape', [])) != (7,) for key in ('action', 'observation.state'))
            or 'observation.images.wrist' not in features):
        raise SystemExit('The bundled X5 training adapter requires six joints, a gripper and wrist RGB; select an adapter for this robot/camera contract')
    return LAYOUTS[layout['layout']]


def main():
    checkout, dataset, command, *arguments = sys.argv[1:]
    checkout, dataset = Path(checkout).resolve(), Path(dataset).resolve()
    if command not in ('norm', 'train') or not (dataset/'meta'/'quest_export.json').is_file():
        raise SystemExit('Use: run.py OPENPI_CHECKOUT EXPORTED_DATASET norm|train [options]')
    layout = json.loads((dataset/"meta"/"quest_export.json").read_text())
    info = json.loads((dataset/"meta"/"info.json").read_text())
    sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'src'))
    repo_id, name = dataset_config(layout, info)
    cache = Path(__file__).resolve().parents[3]/'outputs'/'openpi-datasets'
    link = cache/repo_id
    link.parent.mkdir(parents=True, exist_ok=True)
    if link.exists() or link.is_symlink():
        if link.resolve() != dataset:
            raise SystemExit(f'{link} already refers to another dataset; choose the intended dataset before training')
    else:
        link.symlink_to(dataset, target_is_directory=True)
    os.environ['HF_LEROBOT_HOME'] = str(cache)
    # The platform adapter is also bundled in exported task archives. OpenPI
    # itself must be installed into the selected environment, not imported from a reference.
    sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'src'))
    from embodied.core.dependencies import reject_reference_path
    reject_reference_path(checkout)
    from embodied.core.dependencies import dependency
    expected = dependency("openpi")["commit"]
    revision = subprocess.check_output(["git", "-C", str(checkout), "rev-parse", "HEAD"], text=True).strip()
    if revision != expected:
        raise SystemExit(f"OpenPI training source revision differs: {revision}; expected {expected}")
    os.chdir(checkout)
    from openpi.training import config
    if layout.get('data_contract') is not None and layout['layout'] != 'aligned30_raw240_v1':
        from joint_contract import train_config
        from embodied.policy.openpi.adapter import from_dataset
        item = train_config(from_dataset(layout, info), repo_id=repo_id, fps=info['fps'], name=name)
    else:
        from quest_x5 import train_config
        item = train_config(repo_id=repo_id, fps=info['fps'], exterior_camera='observation.images.exterior' in info['features'], name=name)
    config._CONFIGS_DICT[item.name] = item
    script = checkout/'scripts'/('compute_norm_stats.py' if command == 'norm' else 'train.py')
    sys.argv = [str(script), *(['--config-name', item.name] if command == 'norm' else [item.name]), *arguments]
    runpy.run_path(str(script), run_name='__main__')


if __name__ == '__main__':
    main()
