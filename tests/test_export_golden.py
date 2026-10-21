"""Export regression (todo D-05): fixed schema 1 and schema 2 episodes must export to the same data.

Digests cover the aligned frames (parquet columns) and the full-rate robot sidecar, not video bytes
or wall-clock metadata. After an intended change to the export, regenerate with
UPDATE_GOLDEN=1 and review the diff of tests/golden/export_digests.json.
"""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import unittest

import numpy as np

GOLDEN = Path(__file__).resolve().parent / 'golden/export_digests.json'
ORIGIN = 4_552_089_789_885_210


def digest(values):
    array = np.asarray(values)
    if array.dtype.kind == 'f':
        array = np.round(array.astype(np.float64), 9)
    elif array.dtype.kind in 'OUS':
        array = np.asarray([json.dumps(value.tolist() if hasattr(value, 'tolist') else value, sort_keys=True)
                            for value in array.ravel()])
    return hashlib.sha256(repr(array.shape).encode() + array.tobytes()).hexdigest()[:16]


def dataset_digests(root):
    import pandas
    result = {}
    for path in sorted((root / 'data').rglob('*.parquet')):
        table = pandas.read_parquet(path)
        for column in sorted(table.columns):
            result[f'{path.name}:{column}'] = digest(np.stack(table[column].to_numpy()))
    for path in sorted((root / 'robot').glob('*.npz')):
        with np.load(path, allow_pickle=False) as arrays:
            for name in sorted(arrays.files):
                result[f'{path.name}:{name}'] = digest(arrays[name])
    return result


def schema1(root):
    from test_export_package import make_episode
    make_episode(root, 'schema1', count=12, png=True)


def schema2(root):
    from embodied.data.episode.runtime_episode import RuntimeEpisodeWriter
    from test_runtime_collection import tick
    descriptor = {'runtime_id': 'test', 'backend': 'sim', 'parameters': {'model': {'manifest': {'model': 'fixture'}}}}
    camera = {'stream_id': 'rgb', 'fps': 30, 'height': 48, 'width': 64}
    writer = RuntimeEpisodeWriter(root, descriptor, 'golden', camera)
    for i in range(96):
        row = tick(i, ORIGIN + i * 4_000_000)
        row.update(version=2, source='quest', owner_epoch=1, source_sequence=i, operation_id='')
        writer.add_tick(row)
        if i % 8 == 0:
            writer.add_frame({'image': np.full((48, 64, 3), 8 * i % 250, np.uint8), 'stream_id': 'rgb',
                              'host_received_monotonic_ns': ORIGIN + i * 4_000_000 + 3_000_000, 'frame_number': i // 8})
    writer.close('duration')


@unittest.skipUnless(importlib.util.find_spec('lerobot'), 'export environment: ./run.sh check-all --only export')
class ExportGoldenTests(unittest.TestCase):
    def test_exports_match_the_recorded_digests(self):
        from embodied.data.export.lerobot import export
        golden = json.loads(GOLDEN.read_text()) if GOLDEN.is_file() else {}
        current = {}
        for name, build in (('schema1', schema1), ('schema2', schema2)):
            with tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                build(root / 'raw')
                export(root / 'raw', root / 'dataset', 'local/golden', progress=False, decoder='opencv', decode_workers=0)
                current[name] = dataset_digests(root / 'dataset')
        if os.environ.get('UPDATE_GOLDEN') == '1':
            GOLDEN.write_text(json.dumps(current, indent=2, sort_keys=True) + '\n')
            self.skipTest(f'wrote {GOLDEN}')
        for name in current:
            self.assertIn(name, golden, 'no golden digests; run once with UPDATE_GOLDEN=1')
            changed = sorted(key for key in current[name].keys() | golden[name].keys()
                             if current[name].get(key) != golden[name].get(key))
            self.assertEqual(changed, [], f'{name} export changed')
