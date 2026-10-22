"""Validate runtime MCAPs and inspect causal alignment; no motor or camera access."""
import argparse
import json
from pathlib import Path
import struct

from mcap.reader import make_reader
from mcap.exceptions import McapError
from zstandard import ZstdError

from embodied.data.episode.cameras import camera_topic, camera_metadata, anchor_camera
from embodied.data.episode.contract import episode_contract, vector, action_definition
from embodied.data.episode.runtime_episode import TOPICS, frame_kind, split_tick
from embodied.data.episode.video import media_path
from embodied.data.alignment.runtime_alignment import align
from embodied.contracts.runtime.client import validate_tick_joints


def load_episode(path):
    path = Path(path)
    from embodied.data.episode.schema import RUNTIME, read
    schema, meta = read(path)
    if schema != RUNTIME:
        raise ValueError(f'Expected a {RUNTIME} episode, found {schema}')
    contract = episode_contract(meta)
    if meta.get('action_definition') != action_definition(contract):
        raise ValueError('Unsupported action definition')
    if meta.get('lifecycle') != 'complete' or meta.get('error'):
        raise ValueError('Runtime recording is incomplete or failed')
    if meta.get('record_storage') != {'format': 'mcap', 'file': 'episode.mcap'}:
        raise ValueError('Unsupported record storage')
    cameras = camera_metadata(meta)
    topics = {**{key: topic for key, topic in TOPICS.items() if key != 'frame' or key in meta.get('counts', {})},
              **{frame_kind(c): camera_topic(c) for c in cameras}}
    rows = {key: [] for key in topics}
    lookup = {topic: key for key, topic in topics.items()}
    try:
        with (path / 'episode.mcap').open('rb') as stream:
            for schema, channel, message in make_reader(stream, validate_crcs=True).iter_messages(log_time_order=False):
                kind = lookup.get(channel.topic)
                if kind is None or schema is None or schema.name != f"quest_runtime.{kind.split(':')[0]}.v2":
                    raise ValueError('Unknown runtime MCAP channel/schema')
                row = json.loads(message.data)
                if not isinstance(row, dict) or message.sequence != len(rows[kind]):
                    raise ValueError('Invalid MCAP record or sequence')
                rows[kind].append(row)
    except (McapError, EOFError, struct.error, ZstdError) as exc:
        raise ValueError(f'Invalid or incomplete MCAP: {exc}') from exc
    if meta.get('counts') != {key: len(value) for key, value in rows.items()} or not rows['state']:
        raise ValueError('Manifest counts disagree or robot stream is empty')
    ticks = rows['tick']
    if not ticks or ticks[0]['sequence'] != meta['first_sequence'] or ticks[-1]['sequence'] != meta['last_sequence']:
        raise ValueError('Episode sequence boundaries disagree')
    previous, stamp = ticks[0]['sequence'] - 1, 0
    owner_context = None
    extracted = {'state': [], 'command': []}
    joint_count = meta['runtime'].get('joint_count')
    tick_version = meta['runtime'].get('layout', {}).get('version')
    for i, tick in enumerate(ticks):
        if tick_version is not None and tick.get('version') != tick_version:
            raise ValueError('Runtime tick version changed within the recording')
        joint_count = validate_tick_joints(tick, joint_count)
        if not tick['terminal'] and tick.get('owner_epoch', 0):
            context = tick['source'], tick['owner_epoch']
            if owner_context is not None and context != owner_context:
                raise ValueError('Control ownership changed within the episode')
            owner_context = context
        if (tick.get('type') != 'tick' or tick.get('version') not in (1, 2, 3)
                or tick['runtime_id'] != meta['runtime']['runtime_id'] or tick['sequence'] != previous + 1
                or tick['producer_dropped'] != meta['producer_dropped_at_start']
                or tick['observer_gaps'] != meta['observer_gaps_at_start'] or tick['error']):
            raise ValueError('Tick identity, loss or fault invalidates recording')
        if tick['terminal'] and i != len(ticks) - 1:
            raise ValueError('Ticks follow a terminal packet')
        timing = tick['timing']
        if type(timing['completed_ns']) is not int or timing['completed_ns'] <= stamp:
            raise ValueError('Tick timestamps regress')
        for kind, row in split_tick(tick).items():
            vector(row, contract)
            if type(row['host_monotonic_ns']) is not int or not 0 < row['host_monotonic_ns'] <= timing['completed_ns']:
                raise ValueError('Invalid feedback/command timestamp')
            extracted[kind].append(row)
        if tick.get('command') and not 0 < timing['command_started_ns'] <= timing['command_submitted_ns'] <= timing['completed_ns']:
            raise ValueError('Command submission timestamps regress')
        stamp, previous = timing['completed_ns'], tick['sequence']
    if any(extracted[kind] != rows[kind] for kind in extracted):
        raise ValueError('State/command channels disagree with original ticks')
    if bool(ticks[-1]['terminal']) != meta['terminal_received']:
        raise ValueError('Terminal boundary disagrees')
    if meta['stop_reason'] == 'runtime-terminal' and not meta['terminal_received']:
        raise ValueError('Missing runtime terminal')
    if meta['kind'] == 'quest_runtime_episode' and (not cameras or any(not rows[frame_kind(c)] for c in cameras)):
        raise ValueError('Missing camera data')
    if meta['kind'] == 'quest_runtime_trace' and (cameras or rows.get('frame')):
        raise ValueError('Diagnostic trace unexpectedly contains a camera')
    for camera_id, camera in cameras.items():
        previous, stamp = None, 0
        for frame in rows[frame_kind(camera_id)]:
            number, now = frame['frame_number'], frame['host_received_monotonic_ns']
            if (not camera.get('stream_id') or frame.get('stream_id') != camera['stream_id']
                    or (previous is not None and number != previous + 1) or now <= stamp):
                raise ValueError(f'{camera_id}: camera stream changed or contains gaps')
            if not (frame.get('image') or frame.get('video')) or not media_path(path, frame).is_file():
                raise ValueError(f'{camera_id}: missing source image or video')
            previous, stamp = number, now
    return meta, rows


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('validate', 'alignment'))
    parser.add_argument('episode', type=Path)
    args = parser.parse_args(argv)
    try:
        meta, rows = load_episode(args.episode)
        result = {'id': meta['id'], 'kind': meta['kind'], 'counts': meta['counts'],
                  'action_definition': meta['action_definition'], 'valid': True}
        if args.command == 'alignment':
            if meta['kind'] != 'quest_runtime_episode':
                raise ValueError('Robot-only trace has no training observations')
            selected, excluded = align(rows['state'], rows['command'], rows[frame_kind(anchor_camera(meta))],
                                       sources=('quest', 'phone'), contract=episode_contract(meta))
            result.update(eligible_anchors=len(selected), excluded=excluded,
                          note='Export with scripts/datasets/convert.sh; schemas/action definitions remain separate')
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.exit(1, f'Runtime data error: {exc}\n')


if __name__ == '__main__':
    raise SystemExit(main())
