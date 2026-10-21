"""Read-only RGB integrity checks using receipt clocks, never exposure sync claims."""
import numpy as np
from embodied.data.episode.video import media_path, SequentialFrameReader


def nearest_skew(reference, query):
    reference, query = np.asarray(reference, dtype=np.int64), np.asarray(query, dtype=np.int64)
    if not len(reference) or np.any(np.diff(reference) <= 0):
        raise ValueError('Reference timestamps must be increasing')
    right = np.clip(np.searchsorted(reference, query), 0, len(reference) - 1)
    left = np.clip(right - 1, 0, len(reference) - 1)
    return np.minimum(abs(query - reference[left]), abs(query - reference[right])) / 1e9


def inspect_camera(root, camera, frames, state_times, *, max_gap_s=.15, max_skew_s=.06,
                   started=None, stopped=None, check_images=True, strict_capture=True):
    errors = []
    report = {'frame_count': len(frames), 'errors': errors, 'missing_source_frames': 0}
    try:
        if len(frames) < 2:
            raise ValueError('Need at least two RGB frames')
        if any(type(f['host_received_monotonic_ns']) is not int for f in frames):
            raise ValueError('RGB receipt timestamps must be integer nanoseconds')
        stamps = np.array([f['host_received_monotonic_ns'] for f in frames], dtype=np.int64)
        gaps = np.diff(stamps) / 1e9
        report['frame_hz'] = float((len(frames) - 1) / max((stamps[-1] - stamps[0]) / 1e9, 1e-9))
        report['max_gap_s'] = float(max(gaps))
        if np.any(gaps <= 0):
            errors.append('Non-increasing RGB timestamps')
        if np.any(gaps > max_gap_s):
            errors.append(f'RGB stream has gaps exceeding {max_gap_s}s')
        if report['frame_hz'] < .8 * camera['fps']:
            errors.append('RGB average rate is below 80% of configured rate')
        if started is not None and stopped is not None:
            if stamps[0] < started or stamps[-1] > stopped:
                errors.append('RGB timestamps outside episode interval')
            if stamps[0] - started > max_gap_s * 1e9 or stopped - stamps[-1] > max_gap_s * 1e9:
                errors.append('RGB stream does not cover episode boundaries')
        if strict_capture or any('frame_number' in f for f in frames):
            numbers = [f['frame_number'] for f in frames]
            if any(type(n) is not int or n < 0 for n in numbers):
                raise ValueError('Invalid source frame number')
            increments = [b - a for a, b in zip(numbers, numbers[1:])]
            if any(n <= 0 for n in increments):
                errors.append('Repeated/regressed source camera frames')
            report['missing_source_frames'] = sum(max(0, n - 1) for n in increments)
            if report['missing_source_frames']:
                errors.append('Missing source camera frames')
        if strict_capture or any('device_timestamp_ms' in f for f in frames):
            device = np.array([f['device_timestamp_ms'] for f in frames], dtype=float)
            if not np.isfinite(device).all() or np.any(np.diff(device) <= 0):
                errors.append('Invalid camera device timestamps')
            if len({f['device_timestamp_domain'] for f in frames}) != 1:
                errors.append('Camera timestamp domain changed inside episode')
        streams = {f.get('stream_id') for f in frames}
        if len(streams) != 1 or (camera.get('stream_id') and streams != {camera['stream_id']}):
            errors.append('Camera restarted inside episode')
        skew = nearest_skew(state_times, stamps)
        report['receive_time_alignment_p99_s'] = float(np.quantile(skew, .99))
        report['receive_time_alignment_max_s'] = float(max(skew))
        if max(skew) > max_skew_s:
            errors.append('RGB receipt/robot sampling alignment exceeds limit')
        seen, video_indices = set(), {}
        with SequentialFrameReader() as reader:
            for frame in frames:
                path = media_path(root, frame)
                key = (path, frame.get('video_frame_index'))
                if key in seen:
                    raise ValueError('Multiple frames reference one image')
                seen.add(key)
                if not path.is_file():
                    raise ValueError(f'Missing image: {path.name}')
                if 'video' in frame:
                    index = frame.get('video_frame_index')
                    if type(index) is not int or index != video_indices.get(path, -1) + 1:
                        raise ValueError('Missing or unordered video frame index')
                    video_indices[path] = index
                if check_images:
                    image = reader.read(root, frame)
                    if image is None or image.shape != (camera['height'], camera['width'], 3):
                        raise ValueError(f'Corrupt image or incorrect resolution: {path.name}')
    except (OSError, ValueError, KeyError, TypeError, OverflowError) as exc:
        errors.append(str(exc))
    report['valid'] = not errors
    return report
