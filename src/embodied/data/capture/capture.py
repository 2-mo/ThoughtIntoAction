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


def capture(reader, root, task, duration, camera=None, *, cameras=None, video=False):
    """`cameras` maps camera IDs (wrist_rgb first) to camera objects; `camera` alone is the wrist camera."""
    if not math.isfinite(duration) or duration < 0:
        raise ValueError('Duration must be finite and nonnegative')
    if reader.mode != 'full':
        raise ValueError('Collection requires the full runtime stream')
    cameras = cameras or ({'wrist_rgb': camera} if camera else {})
    writer = RuntimeEpisodeWriter(root, reader.descriptor, task,
                                  cameras={key: source.metadata for key, source in cameras.items()}, video=video,
                                  hand_eye={key: hand_eye(key, source.metadata) for key, source in cameras.items()})
    print(f'Recording: {writer.path}', flush=True)
    stop = threading.Event()
    frames, failures = queue.Queue(maxsize=64 * max(1, len(cameras))), queue.Queue(maxsize=len(cameras) or 1)

    def camera_loop(camera_id, source):
        try:
            while not stop.is_set():
                frame = source.read()
                if not stop.is_set():
                    frames.put_nowait((camera_id, frame))
        except Exception as exc:
            failures.put_nowait(f'{camera_id} failed: {type(exc).__name__}: {exc}')

    threads = [threading.Thread(target=camera_loop, args=item, name=f'collection-{item[0]}', daemon=True)
               for item in cameras.items()]
    for thread in threads:
        thread.start()
    deadline = time.monotonic() + duration if duration else float('inf')
    reason, error = 'duration', None
    end_ns = None
    try:
        while True:
            tick = reader.read()
            writer.add_tick(tick)
            while not frames.empty():
                camera_id, frame = frames.get_nowait()
                writer.add_frame(frame, camera_id)
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
        stop.set()
        for thread in threads:
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
        manifest = writer.close(reason, error)
    return writer.path, manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True, help='Exact outputs/runtime/control/... directory')
    parser.add_argument('--output', type=Path, default=Path('outputs/datasets/runtime'))
    parser.add_argument('--task', required=True)
    parser.add_argument('--duration', type=float, default=0, help='0 records until Ctrl+C or runtime termination')
    parser.add_argument('--camera', default='none',
                        help='none creates a diagnostic trace, wrist subscribes to the shared camera service')
    parser.add_argument('--camera-options', default='{}', help='JSON constructor options for the camera plugin')
    parser.add_argument('--cameras', help='Comma-separated configured camera IDs from the shared camera service, '
                        'e.g. wrist_rgb,exterior_rgb (replaces --camera)')
    parser.add_argument('--rgb', choices=('mp4', 'png'), default='mp4', help='Image storage with --cameras')
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error('duration must be finite and nonnegative')
    try:
        with ExitStack() as stack:
            camera, cameras = None, {}
            if args.cameras:
                from embodied.contracts.registry import Registry
                from embodied.devices.sensors.control import recording_guard
                from embodied.core.project import load_config
                config, registry = load_config(), Registry()
                ids = [key.strip() for key in args.cameras.split(',') if key.strip()]
                if not ids or ids[0] != 'wrist_rgb' or len(set(ids)) != len(ids) or any(k not in config['cameras'] for k in ids):
                    parser.error('--cameras lists unique configured IDs starting with wrist_rgb')
                stack.enter_context(closing(recording_guard()))
                for key in ids:
                    options = config['cameras'][key]
                    driver = options.get('driver', config['cameras']['wrist_rgb']['driver'])
                    selected = {k: options[k] for k in ('serial', 'width', 'height', 'fps')}
                    if 'shared-service' in registry.spec('camera', driver).capabilities:
                        selected['camera_id'] = key  # Otherwise the service answers for its default camera.
                    cameras[key] = stack.enter_context(registry.create('camera', driver, **selected))
            elif args.camera != 'none':
                from embodied.contracts.registry import Registry
                from embodied.devices.sensors.control import recording_guard
                from embodied.core.project import load_config
                options = load_config()['cameras']['wrist_rgb']
                driver = options['driver'] if args.camera == 'wrist' else args.camera
                registry = Registry()
                shared = 'shared-service' in registry.spec('camera', driver).capabilities
                if shared:
                    stack.enter_context(closing(recording_guard()))
                selected = {k: options[k] for k in ('serial', 'width', 'height', 'fps')} if shared else {}
                selected.update(json.loads(args.camera_options))
                camera = stack.enter_context(registry.create('camera', driver, **selected))
            reader = stack.enter_context(RuntimeReader(args.run, 'full'))
            path, meta = capture(reader, args.output, args.task, args.duration, camera,
                                 cameras=cameras or None, video=bool(cameras) and args.rgb == 'mp4')
        print(json.dumps({'path': str(path), 'lifecycle': meta['lifecycle'],
                          'counts': meta['counts'], 'error': meta['error']}, ensure_ascii=False))
        return int(meta['lifecycle'] != 'complete')
    except (OSError, ValueError, RuntimeError, EOFError) as exc:
        parser.exit(1, f'Collection failed: {exc}\n')


if __name__ == '__main__':
    raise SystemExit(main())
