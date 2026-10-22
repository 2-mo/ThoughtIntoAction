"""Verify capture heartbeats without opening camera/control connections."""
import json
from pathlib import Path
import time

HEARTBEAT_TTL_S = 2.


def process_start_ticks(pid):
    if type(pid) is not int or pid <= 0:
        return None
    try:
        fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        return None if fields[0] == 'Z' else fields[19]
    except (OSError, ValueError, IndexError):
        return None


def capture_snapshot(path, identity, manifest, *, now_ns=None, process_reader=process_start_ticks):
    """Final manifests are authoritative; a heartbeat only proves current activity."""
    if manifest.get('lifecycle') != 'recording':
        return None
    try:
        heartbeat = json.loads((Path(path) / 'capture-status.json').read_text())
        if (heartbeat.get('schema') != 'embodied.capture_status/1' or heartbeat.get('schema_version') != 1
                or any(heartbeat.get(key) != identity.get(key) for key in ('runtime_id', 'boot_id'))
                or heartbeat.get('episode', {}).get('id') != manifest.get('id', Path(path).name)):
            raise ValueError('录制心跳身份与当前记录不一致')
        stamp = heartbeat.get('host_monotonic_ns')
        now_ns = time.monotonic_ns() if now_ns is None else now_ns
        if type(stamp) is not int or not 0 < stamp <= now_ns:
            raise ValueError('录制心跳时间无效')
        age = (now_ns - stamp) / 1e9
        process = heartbeat.get('process_start_ticks')
        alive = isinstance(process, str) and process_reader(heartbeat.get('pid')) == process
        status = heartbeat.get('state')
        if status not in ('recording', 'finalizing', 'complete', 'failed'):
            raise ValueError('未知录制心跳状态')
        state = ('ERROR' if status == 'failed' else 'INTERRUPTED' if not alive else 'STALE'
                 if age >= HEARTBEAT_TTL_S else 'RECORDING' if status == 'recording' else 'FINALIZING')
        live = state in ('RECORDING', 'FINALIZING')
        record = {**heartbeat.get('episode', {}), 'counts': heartbeat.get('counts'),
                  'camera_counts': heartbeat.get('camera_counts'), 'queue_size': heartbeat.get('queue_size'),
                  'queue_capacity': heartbeat.get('queue_capacity'), 'dropped': heartbeat.get('dropped'),
                  'error': heartbeat.get('error'), 'stop_reason': heartbeat.get('stop_reason'),
                  'live': live, 'heartbeat_age_s': age, 'heartbeat_ttl_s': HEARTBEAT_TTL_S,
                  'duration_s': max(0, stamp - heartbeat['started_monotonic_ns']) / 1e9
                    if type(heartbeat.get('started_monotonic_ns')) is int else None}
        hint = {'RECORDING': '采集进程在线；计数为已交给写盘器的样本，完成前不代表持久化成功。',
                'FINALIZING': '正在结束写盘；等待 episode.json 完成边界。',
                'STALE': '录制进程仍在，但心跳已过期；以下为最后计数。',
                'INTERRUPTED': '录制进程已退出或 PID 已复用；记录尚未完整结束。',
                'ERROR': heartbeat.get('error') or '采集进程报告错误；记录尚未完整结束。'}[state]
        return {'collection': {'state': state, 'task': record.get('task') or manifest.get('task')},
                'recording': record, 'collection_hint': hint}
    except FileNotFoundError:
        return None
    except (OSError, ValueError, TypeError, AttributeError) as exc:
        return {'collection': {'state': 'UNVERIFIED', 'task': manifest.get('task')},
                'recording': None, 'collection_hint': f'无法确认录制心跳：{exc}'}
