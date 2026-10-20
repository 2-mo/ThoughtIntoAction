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


def capture(reader, root, task, duration, camera=None):
    if not math.isfinite(duration) or duration < 0:
        raise ValueError('Duration must be finite and nonnegative')
    if reader.mode != 'full':
        raise ValueError('Collection requires the full runtime stream')
    writer = RuntimeEpisodeWriter(root, reader.descriptor, task, camera.metadata if camera else None)
    print(f'Recording: {writer.path}', flush=True)
    stop = threading.Event()
    frames, failures = queue.Queue(maxsize=64), queue.Queue(maxsize=1)

    def camera_loop():
        try:
            while not stop.is_set():
                frame = camera.read()
                if not stop.is_set():
                    frames.put_nowait(frame)
        except Exception as exc:
            failures.put_nowait(f'Camera failed: {type(exc).__name__}: {exc}')

    thread = threading.Thread(target=camera_loop, name='collection-camera', daemon=True) if camera else None
    if thread:
        thread.start()
    deadline = time.monotonic() + duration if duration else float('inf')
    reason, error = 'duration', None
    end_ns = None
    try:
        while True:
            tick = reader.read()
            writer.add_tick(tick)
            while not frames.empty():
                writer.add_frame(frames.get_nowait())
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
        if thread:
            thread.join(timeout=4)
            if thread.is_alive():
                error = error or 'Camera reader did not stop'
            if not failures.empty():
                error = error or failures.get_nowait()
            try:
                while not frames.empty():
                    frame = frames.get_nowait()
                    if frame['host_received_monotonic_ns'] <= end_ns:
                        writer.add_frame(frame)
            except Exception as exc:
                error = error or str(exc)
        manifest = writer.close(reason, error)
    return writer.path, manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True, help='Exact control/run/... directory')
    parser.add_argument('--output', type=Path, default=Path('datasets/runtime'))
    parser.add_argument('--task', required=True)
    parser.add_argument('--duration', type=float, default=0, help='0 records until Ctrl+C or runtime termination')
    parser.add_argument('--camera', default='none',
                        help='none creates a diagnostic trace, wrist subscribes to the shared camera service')
    parser.add_argument('--camera-options', default='{}', help='JSON constructor options for the camera plugin')
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error('duration must be finite and nonnegative')
    try:
        with ExitStack() as stack:
            camera = None
            if args.camera != 'none':
                from embodied.contracts.registry import Registry
                from embodied.sensors.control import recording_guard
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
            path, meta = capture(reader, args.output, args.task, args.duration, camera)
        print(json.dumps({'path': str(path), 'lifecycle': meta['lifecycle'],
                          'counts': meta['counts'], 'error': meta['error']}, ensure_ascii=False))
        return int(meta['lifecycle'] != 'complete')
    except (OSError, ValueError, RuntimeError, EOFError) as exc:
        parser.exit(1, f'Collection failed: {exc}\n')


if __name__ == '__main__':
    raise SystemExit(main())
