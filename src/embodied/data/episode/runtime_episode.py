"""Schema 2 storage: original ticks plus independently timestamped feedback/commands."""
from datetime import datetime
import json
from pathlib import Path
import time
import uuid

from mcap.writer import Writer, CompressionType
from embodied.core.storage import save_json
from embodied.data.episode.cameras import camera_topic

ACTION = 'sent_joint_position + sent_gripper_width'
TOPICS = {'tick': '/robot/tick', 'state': '/robot/state', 'command': '/robot/command',
          'frame': '/camera/wrist/rgb/index'}


def frame_kind(camera_id):
    """Count / channel key: the wrist keeps `frame`; other cameras use `frame:<id>`."""
    return 'frame' if camera_id == 'wrist_rgb' else 'frame:' + camera_id


def split_tick(tick):
    """Never backdate an action to the feedback sampled before its submission."""
    shared = {key: tick[key] for key in ('runtime_id', 'sequence', 'phase', 'source', 'error')}
    shared.update({key: tick[key] for key in ('owner_epoch', 'source_sequence', 'operation_id') if key in tick})
    result = {}
    for kind, field, clock in [('state', 'feedback', 'feedback_received_ns'),
                                ('command', 'command', 'command_submitted_ns')]:
        if tick.get(field) is not None:
            result[kind] = {**tick[field], **shared, 'host_monotonic_ns': tick['timing'][clock]}
    return result


class RuntimeEpisodeWriter:
    """Synchronous disk writer for an independent collector, never the robot loop.

    A process crash leaves lifecycle=recording. Only explicit close boundaries
    commit complete; an observed stream or storage failure commits failed.
    """
    def __init__(self, root, descriptor, task, camera=None, *, cameras=None, video=False):
        """`camera` is the wrist metadata; `cameras` maps camera IDs (wrist_rgb required) to metadata.

        video=True stores each camera as H.264 MP4 with per-frame indices; otherwise one PNG per frame.
        """
        if not isinstance(task, str) or not 1 <= len(task.strip()) <= 2000:
            raise ValueError('Provide a task description (1..2000 characters)')
        if not cameras:
            cameras = {'wrist_rgb': camera} if camera else {}
        elif 'wrist_rgb' not in cameras:
            raise ValueError('Recording cameras must include wrist_rgb')
        camera = cameras.get('wrist_rgb')
        self.cameras, self.video = dict(cameras), bool(video)
        self.path = Path(root) / f'arx_{datetime.now():%Y%m%d-%H%M%S}-{uuid.uuid4().hex[:8]}'
        self.path.mkdir(parents=True, exist_ok=False)
        if not self.video:
            for camera_id in self.cameras or {'wrist_rgb': None}:
                (self.path / camera_id).mkdir()
        self.topics = {**TOPICS, **{frame_kind(c): camera_topic(c) for c in self.cameras}}
        self.counts = dict.fromkeys(self.topics, 0)
        self.first_sequence = self.last_sequence = None
        self.last_frames = {}
        self.videos = {}
        self.dropped_baseline = self.gaps_baseline = None
        self.error = None
        self.closed = False
        self.last_tick_time = None
        self.terminal = False
        self.owner_context = None
        self.manifest = {
            'schema_version': 2, 'kind': 'quest_runtime_episode' if camera else 'quest_runtime_trace',
            'id': self.path.name, 'task': task.strip(), 'lifecycle': 'recording', 'outcome': 'unknown',
            'started_monotonic_ns': time.monotonic_ns(), 'started_unix_ns': time.time_ns(),
            'record_storage': {'format': 'mcap', 'file': 'episode.mcap'},
            'rgb_storage': {'format': 'mp4', 'codec': 'h264', 'crf': 20} if self.video else {'format': 'png'},
            'depth_recorded': False, 'action_definition': ACTION, 'runtime': descriptor, 'camera': camera,
            'clock': 'host monotonic_ns; feedback receipt, command submission and image receipt are independent',
            'raw_input_recorded': False, 'counts': dict(self.counts)}
        if len(self.cameras) > 1:
            self.manifest['cameras'] = self.cameras
        self.offset = self.manifest['started_unix_ns'] - self.manifest['started_monotonic_ns']
        save_json(self.path / 'episode.json', self.manifest)
        self.stream = (self.path / 'episode.mcap').open('xb')
        self.writer = Writer(self.stream, compression=CompressionType.ZSTD, enable_crcs=True)
        self.writer.start(library='QuestArmTeleop', profile='quest.runtime.v2')
        self.channels = {}
        for kind, topic in self.topics.items():
            schema = self.writer.register_schema('quest_runtime.' + kind.split(':')[0] + '.v2', 'jsonschema',
                json.dumps({'type': 'object', 'description': self.manifest['clock']}).encode())
            self.channels[kind] = self.writer.register_channel(topic, 'json', schema)
        self.writer.add_metadata('episode', {'manifest': json.dumps(self.manifest, ensure_ascii=False)})

    def _write(self, kind, row, stamp):
        if self.closed:
            raise ValueError('Episode is already closed')
        self.writer.add_message(self.channels[kind], log_time=stamp + self.offset,
            publish_time=stamp + self.offset, sequence=self.counts[kind],
            data=json.dumps(row, ensure_ascii=False, allow_nan=False).encode())
        self.counts[kind] += 1

    def add_tick(self, tick):
        if self.terminal:
            raise ValueError('Tick after runtime termination')
        if tick['runtime_id'] != self.manifest['runtime']['runtime_id']:
            raise ValueError('Runtime changed during recording')
        sequence = tick['sequence']
        if not tick['terminal'] and tick.get('owner_epoch', 0):
            context = tick['source'], tick['owner_epoch']
            if self.owner_context is not None and context != self.owner_context:
                raise ValueError('Control ownership changed; start a separate episode')
            self.owner_context = context
        if type(sequence) is not int or sequence < 0:
            raise ValueError('Invalid tick sequence')
        if self.last_sequence is not None and sequence != self.last_sequence + 1:
            raise ValueError('Robot stream sequence gap')
        if self.first_sequence is None:
            self.first_sequence = sequence
            self.dropped_baseline, self.gaps_baseline = tick['producer_dropped'], tick['observer_gaps']
        if (tick['producer_dropped'], tick['observer_gaps']) != (self.dropped_baseline, self.gaps_baseline):
            raise ValueError('Telemetry loss during recording')
        stamp = tick['timing']['completed_ns']
        if type(stamp) is not int or stamp <= (self.last_tick_time or 0):
            raise ValueError('Invalid tick timestamp')
        self._write('tick', tick, stamp)
        for kind, row in split_tick(tick).items():
            self._write(kind, row, row['host_monotonic_ns'])
        self.last_sequence, self.last_tick_time = sequence, stamp
        self.terminal = tick['terminal']
        if tick['error']:
            self.error = str(tick['error'])

    def add_frame(self, frame, camera_id='wrist_rgb'):
        import cv2
        camera = self.cameras.get(camera_id)
        if not camera or not camera.get('stream_id') or frame.get('stream_id') != camera['stream_id']:
            raise ValueError(f'{camera_id}: camera stream identity changed or missing')
        number, stamp = frame['frame_number'], frame['host_received_monotonic_ns']
        last, last_time = self.last_frames.get(camera_id, (None, 0))
        if (type(number) is not int or type(stamp) is not int or stamp <= last_time
                or (last is not None and number != last + 1)):
            raise ValueError(f'{camera_id}: camera frame gap or timestamp regression')
        image = frame['image']
        if image.shape != (camera['height'], camera['width'], 3) or str(image.dtype) != 'uint8':
            raise ValueError(f'{camera_id}: camera image shape/dtype changed')
        kind = frame_kind(camera_id)
        row = {k: v for k, v in frame.items() if k not in ('image', 'depth_image')}
        if self.video:
            if camera_id not in self.videos:
                from embodied.data.episode.video import RGBVideoWriter
                self.videos[camera_id] = RGBVideoWriter(self.path / f'{camera_id}.mp4', camera['width'], camera['height'],
                                                        camera.get('fps', 30), crf=self.manifest['rgb_storage']['crf'])
            self.videos[camera_id].write(image)
            row.update(video=f'{camera_id}.mp4', video_frame_index=self.counts[kind])
        else:
            row['image'] = f'{camera_id}/{self.counts[kind]:06d}.png'
            if not cv2.imwrite(str(self.path / row['image']), image, [cv2.IMWRITE_PNG_COMPRESSION, 1]):
                raise OSError('Cannot save camera image')
        self._write(kind, row, stamp)
        self.last_frames[camera_id] = number, stamp

    def close(self, reason, error=None):
        if self.closed:
            return self.manifest
        directory = self.manifest['runtime'].get('directory')
        if directory:
            metadata = Path(directory) / 'source-metadata.json'
            try:
                if metadata.is_file():
                    self.manifest['source_metadata_events'] = json.loads(metadata.read_text())
            except (OSError, ValueError) as exc:
                error = error or f'Source metadata could not be preserved: {exc}'
        self.error = error or self.error
        if reason not in ('duration', 'operator-stop', 'runtime-terminal'):
            self.error = self.error or 'Recording ended without an explicit complete boundary'
        if reason == 'runtime-terminal' and not self.terminal:
            self.error = self.error or 'Runtime terminal was not received'
        if not self.counts['state'] or any(not self.counts[frame_kind(c)] for c in self.cameras):
            self.error = self.error or 'Empty robot or camera stream'
        for camera_id, video in self.videos.items():
            try:
                video.close()
            except Exception as exc:
                self.error = self.error or f'{camera_id}: {exc}'
        try:
            self.writer.finish()
            self.stream.flush()
            import os
            os.fsync(self.stream.fileno())
        except Exception as exc:
            self.error = self.error or f'Finalization failed: {exc}'
        finally:
            self.stream.close()
            self.closed = True
        self.manifest.update(lifecycle='failed' if self.error else 'complete', error=self.error,
            stopped_monotonic_ns=time.monotonic_ns(), counts=dict(self.counts), stop_reason=reason,
            first_sequence=self.first_sequence, last_sequence=self.last_sequence,
            terminal_received=self.terminal, producer_dropped_at_start=self.dropped_baseline,
            observer_gaps_at_start=self.gaps_baseline)
        save_json(self.path / 'episode.json', self.manifest)
        return self.manifest
