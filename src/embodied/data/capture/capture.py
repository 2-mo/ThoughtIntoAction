"""Capture an explicit bounded interval of the read-only runtime stream."""
import argparse
from contextlib import ExitStack, closing
import json
import math
from pathlib import Path
import queue
import threading
import time

from embodied.data.episode.runtime_episode import RuntimeEpisodeWriter
from embodied.contracts.runtime import RuntimeReader


def hand_eye(camera_id, metadata):
    """Extrinsics of a real camera at recording time (embodied.applications.calibration); synthetic cameras have none."""
    if not metadata.get('serial'):
        return None
    from embodied.applications import calibration
    return calibration.extrinsics(camera_id, metadata['serial'])


def capture(reader, root, task, duration, camera=None, *, cameras=None, video=False, anchor=None,
            demonstration=None, camera_roles=None, stop_event=None, ready=None, on_created=None):
    """`cameras` maps configured IDs to sources; `anchor` selects the training reference camera."""
    if not math.isfinite(duration) or duration < 0:
        raise ValueError('Duration must be finite and nonnegative')
    if reader.mode != 'full':
        raise ValueError('Collection requires the full runtime stream')
    cameras = cameras or ({'wrist_rgb': camera} if camera else {})
    writer = RuntimeEpisodeWriter(root, reader.descriptor, task,
                                  cameras={key: source.metadata for key, source in cameras.items()}, video=video, anchor=anchor,
                                  hand_eye={key: hand_eye(key, source.metadata) for key, source in cameras.items()},
                                  demonstration=demonstration, camera_roles=camera_roles)
    print(f'Recording: {writer.path}', flush=True)
    stop = threading.Event()
    frames, failures = queue.Queue(maxsize=64 * max(1, len(cameras))), queue.Queue(maxsize=len(cameras) or 1)
    dropped = dict.fromkeys(cameras, 0)
    from embodied.data.capture.status import CaptureStatus
    from embodied.data.episode.runtime_episode import frame_kind
    heartbeat = CaptureStatus(writer, frames, dropped)

    def camera_loop(camera_id, source):
        try:
            while not stop.is_set():
                frame = source.read()
                if not stop.is_set():
                    try:
                        frames.put_nowait((camera_id, frame))
                    except queue.Full:
                        dropped[camera_id] += 1
                        raise RuntimeError('Camera queue overflow; recording is incomplete')
        except Exception as exc:
            failures.put_nowait(f'{camera_id} failed: {type(exc).__name__}: {exc}')

    threads = [threading.Thread(target=camera_loop, args=item, name=f'collection-{item[0]}', daemon=True)
               for item in cameras.items()]
    deadline = time.monotonic() + duration if duration else float('inf')
    reason, error = 'duration', None
    end_ns = None
    try:
        heartbeat.start()
        if on_created is not None:
            on_created(writer)
        for thread in threads:
            thread.start()
        while True:
            if stop_event is not None and stop_event.is_set():
                reason = 'context-exit'
                break
            tick = reader.read()
            writer.add_tick(tick)
            while not frames.empty():
                camera_id, frame = frames.get_nowait()
                writer.add_frame(frame, camera_id)
            heartbeat.progress_ns = time.monotonic_ns()
            if ready is not None and writer.counts['state'] >= 2 and all(writer.counts[frame_kind(key)] >= 2 for key in cameras):
                ready.set()
            if heartbeat.error:
                raise RuntimeError(heartbeat.error)
            if not failures.empty():
                raise RuntimeError(failures.get_nowait())
            if tick['error']:
                raise RuntimeError(tick['error'])
            if reader.terminal:
                reason = 'runtime-terminal'
                break
            if time.monotonic() >= deadline:
                break
    except KeyboardInterrupt:
        reason = 'operator-stop'
    except Exception as exc:
        reason, error = 'stream-error', f'{type(exc).__name__}: {exc}'
    finally:
        end_ns = time.monotonic_ns()
        heartbeat.state = 'finalizing'
        heartbeat.error, heartbeat.reason = error, reason
        stop.set()
        for thread in threads:
            if thread.ident is not None:
                thread.join(timeout=4)
            if thread.is_alive():
                error = error or f'{thread.name} did not stop'
        if threads:
            if not failures.empty():
                error = error or failures.get_nowait()
            try:
                while not frames.empty():
                    camera_id, frame = frames.get_nowait()
                    if frame['host_received_monotonic_ns'] <= end_ns:
                        writer.add_frame(frame, camera_id)
            except Exception as exc:
                error = error or str(exc)
        try:
            manifest = writer.close(reason, error)
        except BaseException as exc:
            heartbeat.fail(exc)
            raise
        if heartbeat.thread.ident is not None:
            heartbeat.close(manifest)
    return writer.path, manifest


def open_cameras(stack, ids, *, config=None, registry=None):
    """Open explicitly configured camera IDs; each driver owns its own options."""
    from embodied.contracts.registry import Registry
    from embodied.core.project import load_config
    from embodied.devices.sensors.control import recording_guard
    config, registry = config if config is not None else load_config(), registry or Registry()
    if not ids or len(set(ids)) != len(ids) or any(key not in config['cameras'] for key in ids):
        raise ValueError('Select unique configured camera IDs')
    cameras, guarded = {}, False
    for key in ids:
        options = config['cameras'][key]
        driver = options.get('driver')
        if not driver:
            raise ValueError(f'Camera {key} must declare its driver')
        selected = {name: options[name] for name in ('serial', 'width', 'height', 'fps') if name in options}
        if 'shared-service' in registry.spec('camera', driver).capabilities:
            if not guarded:
                stack.enter_context(closing(recording_guard()))
                guarded = True
            selected['camera_id'] = key
        cameras[key] = stack.enter_context(registry.create('camera', driver, **selected))
    return cameras


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True, help='Exact outputs/runtime/control/... directory')
    parser.add_argument('--output', type=Path, default=Path('outputs/datasets/runtime'))
    parser.add_argument('--task', required=True)
    parser.add_argument('--duration', type=float, default=0, help='0 records until Ctrl+C or runtime termination')
    parser.add_argument('--camera', default='none',
                        help='none creates a diagnostic trace, wrist subscribes to the shared camera service')
    parser.add_argument('--camera-options', default='{}', help='JSON constructor options for the camera plugin')
    parser.add_argument('--camera-id', default='camera_rgb', help='Recorded ID for a directly selected --camera plugin')
    parser.add_argument('--camera-roles', help='JSON role-to-camera mapping, e.g. {"base_0_rgb":"front_rgb"}')
    parser.add_argument('--cameras', help='Comma-separated configured camera IDs from the shared camera service, '
                        'e.g. wrist_rgb,exterior_rgb (replaces --camera)')
    parser.add_argument('--rgb', choices=('mp4', 'png'), default='mp4', help='Image storage with --cameras')
    parser.add_argument('--anchor-camera', help='Reference camera ID for training alignment (defaults to wrist or first selected camera)')
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error('duration must be finite and nonnegative')
    try:
        with ExitStack() as stack:
            camera, cameras = None, {}
            if args.cameras:
                ids = [key.strip() for key in args.cameras.split(',') if key.strip()]
                cameras = open_cameras(stack, ids)
            elif args.camera != 'none':
                from embodied.contracts.registry import Registry
                if args.camera == 'wrist':
                    cameras = open_cameras(stack, ['wrist_rgb'])
                else:
                    from embodied.data.episode.cameras import camera_topic
                    camera_topic(args.camera_id)
                    registry = Registry()
                    options = json.loads(args.camera_options)
                    if 'shared-service' in registry.spec('camera', args.camera).capabilities:
                        from embodied.devices.sensors.control import recording_guard
                        stack.enter_context(closing(recording_guard()))
                        if options.get('camera_id', args.camera_id) != args.camera_id:
                            raise ValueError('Camera service ID must match --camera-id')
                        options['camera_id'] = args.camera_id
                    cameras = {args.camera_id: stack.enter_context(registry.create('camera', args.camera, **options))}
            reader = stack.enter_context(RuntimeReader(args.run, 'full'))
            path, meta = capture(reader, args.output, args.task, args.duration, camera,
                                 cameras=cameras or None, video=bool(cameras) and args.rgb == 'mp4', anchor=args.anchor_camera,
                                 camera_roles=json.loads(args.camera_roles) if args.camera_roles else None)
        print(json.dumps({'path': str(path), 'lifecycle': meta['lifecycle'],
                          'counts': meta['counts'], 'error': meta['error']}, ensure_ascii=False))
        return int(meta['lifecycle'] != 'complete')
    except (OSError, ValueError, RuntimeError, EOFError) as exc:
        parser.exit(1, f'Collection failed: {exc}\n')


if __name__ == '__main__':
    raise SystemExit(main())
