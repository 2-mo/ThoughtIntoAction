"""Write current-format fixtures while allowing tests to replace a single topic."""
import json
from embodied.data.episode.mcap_io import RecordWriter, read_records


def write_records(root, *, states=None, frames=None):
    path = root / 'episode.mcap'
    previous = read_records(path) if path.exists() else ([], [])
    states = previous[0] if states is None else states
    frames = previous[1] if frames is None else frames
    meta_path = root / 'episode.json'
    meta = json.loads(meta_path.read_text())
    meta['record_storage'] = {'format': 'mcap', 'file': 'episode.mcap'}
    meta_path.write_text(json.dumps(meta))
    with path.open('wb') as stream:
        writer = RecordWriter(stream, {'started_unix_ns': 0, 'started_monotonic_ns': 0, **meta})
        for kind, rows in [('state', states), ('frame', frames)]:
            for sequence, row in enumerate(rows): writer.write(kind, row, sequence)
        writer.finish()
