"""Read the checkpoint's primitive metadata without importing serialized training code."""
import io
import json
from pathlib import Path
import pickle
import zipfile

DEFAULT_CHECKPOINT = Path('/home/r2/work/checkpoints/2500')
ROOT = Path(__file__).resolve().parents[2]
OPENPI_REVISION = '215abfb217dbac7d5f1273282331b9b1866c0479'
OPENPI = ROOT/'outputs/inference'/('openpi-'+OPENPI_REVISION)


class IgnoredTrainingFilter:
    pass


class MetadataReader(pickle.Unpickler):
    def find_class(self, module, name):
        if (module, name) == ('flax.nnx.filterlib', 'Nothing'):
            return IgnoredTrainingFilter
        raise pickle.UnpicklingError(f'Unsupported metadata object: {module}.{name}')


def inspect_checkpoint(path):
    path = Path(path).resolve()
    with zipfile.ZipFile(path/'metadata.pt') as archive:
        names = [n for n in archive.namelist() if n.endswith('/data.pkl')]
        if len(names) != 1 or archive.getinfo(names[0]).file_size > 1_000_000:
            raise ValueError('Invalid metadata archive')
        meta = MetadataReader(io.BytesIO(archive.read(names[0]))).load()
    config = meta['config']
    model = config['model']
    if not model.get('pi05') or not model.get('discrete_state_input') or model['action_dim'] != 32:
        raise ValueError('Expected PI05 with discrete state and action_dim=32')
    repo = config['data']['repo_id']
    stats = (path/'assets'/repo/'norm_stats.json').resolve()
    if not stats.is_relative_to(path/'assets'):
        raise ValueError('Asset path escapes checkpoint')
    values = json.loads(stats.read_text())['norm_stats']
    for key in ('state', 'actions'):
        for field in ('mean', 'std', 'q01', 'q99'):
            if len(values[key][field]) != 7:
                raise ValueError('Expected seven-dimensional X5 normalization statistics')
    if not (path/'model.safetensors').is_file():
        raise FileNotFoundError(path/'model.safetensors')
    return {'checkpoint': str(path), 'step': meta['global_step'], 'config_name': config['name'],
            'model': model, 'asset_id': repo, 'stats': str(stats),
            'adapter': 'project X5Inputs/X5Outputs; delta6 joints + absolute gripper metres (user confirmed)',
            'openpi_revision': OPENPI_REVISION}


DEFAULT_JAX_CHECKPOINT = Path('/home/r2/work/checkpoints/blue_cube_jax_v1')


def inspect_jax_checkpoint(path, horizon=15):
    path = Path(path).expanduser().resolve()
    if not (path/'params').is_dir():
        steps = [p for p in path.iterdir() if p.name.isdigit() and (p/'params/_METADATA').is_file()]
        if not steps:
            raise ValueError('No JAX step checkpoint with params found')
        path = max(steps, key=lambda p: int(p.name))
    if not (path/'params/_METADATA').is_file():
        raise ValueError('Missing Orbax params metadata')
    files = list((path/'assets').rglob('norm_stats.json'))
    if len(files) != 1:
        raise ValueError('Expected exactly one checkpoint-owned X5 stats asset')
    values = json.loads(files[0].read_text())['norm_stats']
    for key in ('state', 'actions'):
        for field in ('mean', 'std', 'q01', 'q99'):
            if len(values[key][field]) != 7:
                raise ValueError('Expected seven-dimensional X5 statistics')
    if not 1 <= horizon <= 50:
        raise ValueError('JAX horizon must be in 1..50')
    return {'checkpoint':str(path), 'step':int(path.name) if path.name.isdigit() else None,
            'config_name':'pi05_quest_x5', 'model':dict(pi05=True, action_dim=32,
                action_horizon=horizon, max_token_len=200, discrete_state_input=True,
                dtype='bfloat16', pytorch_compile_mode=None),
            'stats':str(files[0]), 'asset_id':str(files[0].parent.relative_to(path/'assets')),
            'openpi_revision':OPENPI_REVISION,
            'config_source':'project X5 adapter defaults; Orbax checkpoint does not store training config'}
