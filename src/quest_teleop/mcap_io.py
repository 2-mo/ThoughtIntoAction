"""MCAP robot snapshots and RGB frame indices; media remains relative to episode root."""
import json
import struct
from zstandard import ZstdError
from mcap.reader import make_reader
from mcap.exceptions import McapError
from mcap.writer import Writer, CompressionType

TOPICS = {'state': '/robot/state', 'frame': '/camera/wrist/rgb/index'}

class RecordWriter:
    def __init__(self, stream, manifest):
        self.writer = Writer(stream, compression=CompressionType.ZSTD, enable_crcs=True)
        self.writer.start(library='QuestArmTeleop', profile='')
        self.offset = manifest['started_unix_ns'] - manifest['started_monotonic_ns']
        self.channels = {}
        for kind, topic in TOPICS.items():
            schema = self.writer.register_schema('quest_teleop.' + kind + '.v1', 'jsonschema',
                json.dumps({'type': 'object', 'description': 'Original episode snapshot; SI units, host monotonic timestamps in nanoseconds'}).encode())
            self.channels[kind] = self.writer.register_channel(topic, 'json', schema)
        self.writer.add_metadata('episode', {'manifest': json.dumps(manifest, ensure_ascii=False),
            'clock': 'log_time = started_unix_ns + (host_monotonic_ns - started_monotonic_ns); camera device time retained in payload',
            'media': 'Relative RGB paths refer to files alongside this MCAP; keep the episode directory together'})

    def write(self, kind, row, sequence):
        stamp = row['host_monotonic_ns' if kind == 'state' else 'host_received_monotonic_ns'] + self.offset
        self.writer.add_message(self.channels[kind], log_time=stamp, publish_time=stamp,
            sequence=sequence, data=json.dumps(row, ensure_ascii=False, allow_nan=False).encode())

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
