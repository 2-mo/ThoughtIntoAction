"""MCAP robot snapshots and RGB frame indices; media remains relative to episode root."""
import json
import struct
from zstandard import ZstdError
from mcap.reader import make_reader
from mcap.exceptions import McapError
from mcap.writer import Writer, CompressionType

TOPICS = {'state': '/robot/state', 'frame': '/camera/wrist/rgb/index',
          'diagnostics': '/robot/diagnostics', 'event': '/robot/events'}

class RecordWriter:
    def __init__(self, stream, manifest):
        self.writer = Writer(stream, compression=CompressionType.ZSTD, enable_crcs=True)
        self.writer.start(library='QuestArmTeleop', profile='')
        self.offset = manifest['started_unix_ns'] - manifest['started_monotonic_ns']
        self.started = manifest['started_monotonic_ns']
        self.last_diagnostics_ns = None
        self.event_cursor = 0
        self.diagnostic_stream = None
        self.extra_counts = {'diagnostics': 0, 'event': 0}
        self.channels = {}
        for kind, topic in TOPICS.items():
            schema = self.writer.register_schema('quest_teleop.' + kind + '.v1', 'jsonschema',
                json.dumps({'type': 'object', 'description': 'Original episode snapshot; SI units, host monotonic timestamps in nanoseconds'}).encode())
            self.channels[kind] = self.writer.register_channel(topic, 'json', schema)
        self.writer.add_metadata('episode', {'manifest': json.dumps(manifest, ensure_ascii=False),
            'clock': 'log_time = started_unix_ns + (host_monotonic_ns - started_monotonic_ns); camera device time retained in payload',
            'media': 'Relative RGB paths refer to files alongside this MCAP; keep the episode directory together'})

    def write(self, kind, row, sequence):
        stamp = row['host_received_monotonic_ns' if kind == 'frame' else 'host_monotonic_ns'] + self.offset
        self.writer.add_message(self.channels[kind], log_time=stamp, publish_time=stamp,
            sequence=sequence, data=json.dumps(row, ensure_ascii=False, allow_nan=False).encode())

    def write_state(self, row, sequence):
        row = dict(row)
        diagnostics = row.pop('motor_diagnostics', None)
        self.write('state', row, sequence)
        if not diagnostics:
            return
        stamp = diagnostics['host_monotonic_ns']
        if diagnostics.get('stream_id') != self.diagnostic_stream:
            self.event_cursor = 0
            self.last_diagnostics_ns = None
            self.diagnostic_stream = diagnostics.get('stream_id')
        if stamp >= self.started and (self.last_diagnostics_ns is None or stamp - self.last_diagnostics_ns >= 1_000_000_000):
            self.write('diagnostics', {k: v for k, v in diagnostics.items() if k != 'events'}, self.extra_counts['diagnostics'])
            self.extra_counts['diagnostics'] += 1
            self.last_diagnostics_ns = stamp
        events = diagnostics.get('events', [])
        if self.event_cursor and events and events[0]['sequence'] > self.event_cursor + 1:
            raise ValueError('Motor event buffer overflow; diagnostics are incomplete')
        for event in events:
            if event['sequence'] <= self.event_cursor:
                continue
            if event['host_monotonic_ns'] >= self.started:
                self.write('event', {**event, 'stream_id': self.diagnostic_stream}, self.extra_counts['event'])
                self.extra_counts['event'] += 1
            self.event_cursor = event['sequence']

    def finish(self):
        self.writer.finish()


def read_records(path):
    records = {topic: [] for topic in TOPICS.values()}
    try:
        with path.open('rb') as stream:
            # File order retains producer order so validators detect regressing timestamps.
            for _, channel, message in make_reader(stream, validate_crcs=True).iter_messages(topics=list(records), log_time_order=False):
                row = json.loads(message.data)
                if not isinstance(row, dict):
                    raise ValueError('MCAP record must be an object')
                records[channel.topic].append(row)
    except (McapError, EOFError, struct.error, ZstdError) as exc:
        raise ValueError(f"Invalid or incomplete MCAP: {exc}") from exc
    return records[TOPICS['state']], records[TOPICS['frame']]
