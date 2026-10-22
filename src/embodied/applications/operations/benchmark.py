"""Measured motion reports from the same full runtime stream used by collection.

No driver is opened here. Command/feedback timestamps are independent; lag is a
position-fit estimate, not a claim about CAN latency or mechanical vibration.
"""
from bisect import bisect_right
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import threading
import time

import numpy as np

from embodied.contracts.runtime import RuntimeReader
from embodied.core.storage import save_json


def _percentiles(values):
    if not values:
        return None
    return dict(zip(('p50', 'p95', 'p99', 'max'), map(float, np.percentile(values, [50, 95, 99, 100]))))


def analyze_motion(ticks, *, pause_speed_rad_s=.01, pause_duration_s=.06):
    """Describe a motion's full ticks; missing data stays null, gaps stay visible."""
    if not all(math.isfinite(v) and v > 0 for v in (pause_speed_rad_s, pause_duration_s)):
        raise ValueError('Pause thresholds must be finite and positive')
    feedback, commands, periods, gaps, previous = [], [], [], 0, None
    count = None
    runtime_id = None
    for tick in ticks:
        if runtime_id is not None and tick.get('runtime_id') != runtime_id:
            raise ValueError('Motion report cannot mix runtime identities')
        runtime_id = tick.get('runtime_id')
        stamp = tick.get('timing') or {}
        sequence = tick['sequence']
        if previous:
            delta = sequence - previous['sequence']
            if delta <= 0:
                raise ValueError('Repeated or regressed benchmark sequence')
            gaps += delta - 1
            if delta == 1 and stamp.get('tick_ns', 0) > previous['timing'].get('tick_ns', 0):
                periods.append((stamp['tick_ns'] - previous['timing']['tick_ns']) / 1e6)
        previous = tick
        if tick.get('error'):
            raise ValueError(f"Runtime error in benchmark: {tick['error']}")
        for key, output, clock in (('feedback', feedback, 'feedback_received_ns'),
                                    ('command', commands, 'command_submitted_ns')):
            row = tick.get(key)
            if not row or (key == 'command' and not row.get('submitted', True)):
                continue
            q, dq = np.asarray(row.get('q'), dtype=float), np.asarray(row.get('dq'), dtype=float)
            if q.ndim != 1 or dq.shape != q.shape or q.size == 0 or not np.isfinite([q, dq]).all():
                raise ValueError('Benchmark requires finite q/dq feedback and commands')
            count = q.size if count is None else count
            if q.size != count:
                raise ValueError('Benchmark joint count changed')
            ns = stamp.get(clock)
            if type(ns) is not int or ns <= 0 or (output and ns <= output[-1][0]):
                raise ValueError('Benchmark source timestamps must increase')
            output.append((ns, q, dq, sequence))
    result = {'feedback_samples': len(feedback), 'command_samples': len(commands),
              'sequence_gaps': gaps, 'cycle_ms': _percentiles(periods),
              'feedback_peak_velocity_rad_s': None, 'command_peak_velocity_rad_s': None,
              'reference_error_rms_rad': None, 'reference_error_max_rad': None,
              'estimated_position_lag_ms': None, 'candidate_pause_count': 0,
              'measured_joint_delta_rad': None,
              'pause_thresholds': {'speed_rad_s': pause_speed_rad_s, 'duration_s': pause_duration_s},
              'alignment': 'latest command at or before feedback timestamp (zero-order hold)',
              'lag_definition': 'best position RMSE over 0..200 ms; not transport-only latency'}
    if feedback:
        result['feedback_peak_velocity_rad_s'] = np.max(np.abs([r[2] for r in feedback]), axis=0).tolist()
        result['measured_joint_delta_rad'] = (feedback[-1][1] - feedback[0][1]).tolist()
    if commands:
        result['command_peak_velocity_rad_s'] = np.max(np.abs([r[2] for r in commands]), axis=0).tolist()
    if not feedback or not commands:
        return result
    command_ns = [r[0] for r in commands]
    errors, pause_start, pause_counted, previous_feedback = [], None, False, None
    for ns, q, dq, seq in feedback:
        index = bisect_right(command_ns, ns) - 1
        if index < 0:
            continue
        command = commands[index]
        # A stale reference or a missing tick must not fabricate a long stall.
        contiguous = previous_feedback is None or seq == previous_feedback[3] + 1
        fresh = ns - command[0] <= 100_000_000
        if fresh:
            errors.append(q - command[1])
        intended = np.abs(command[2]) > pause_speed_rad_s * 2
        paused = fresh and np.any(intended & (np.abs(dq) < pause_speed_rad_s))
        if not paused or not contiguous:
            pause_start, pause_counted = None, False
        if paused:
            pause_start = ns if pause_start is None else pause_start
            if not pause_counted and (ns - pause_start) / 1e9 >= pause_duration_s:
                result['candidate_pause_count'] += 1
                pause_counted = True
        previous_feedback = (ns, q, dq, seq)
    if errors:
        values = np.asarray(errors)
        result['reference_error_rms_rad'] = np.sqrt(np.mean(values**2, axis=0)).tolist()
        result['reference_error_max_rad'] = np.max(np.abs(values), axis=0).tolist()
    # Compare every lag on the same interior samples; endpoint clipping cannot
    # make a large lag look artificially good. Missing sequences invalidate fit.
    if not gaps and len(feedback) >= 20 and len(commands) >= 20:
        origin = commands[0][0]
        ct = (np.asarray(command_ns) - origin) / 1e9
        fq = np.asarray([r[1] for r in feedback])
        ft = (np.asarray([r[0] for r in feedback]) - origin) / 1e9
        cq = np.asarray([r[1] for r in commands])
        moving = np.ptp(cq, axis=0) > .005
        valid = (ft >= .2) & (ft <= ct[-1])
        if moving.any() and valid.sum() >= 10:
            grid = np.linspace(0., .2, 101)
            scores = [float(np.mean((fq[valid][:, moving] - np.column_stack([
                np.interp(ft[valid] - lag, ct, cq[:, j]) for j in np.flatnonzero(moving)]))**2)) for lag in grid]
            best = int(np.argmin(scores))
            result['estimated_position_lag_ms'] = float(grid[best] * 1000)
            result['lag_fit_boundary'] = best == len(grid) - 1
    return result


def compare_reports(current, baseline):
    """Compare identical test definitions; retain configuration changes explicitly."""
    fields = ('schema', 'joint', 'delta_rad', 'repeats', 'requested_speed_rad_s')
    if (any(current.get(key) != baseline.get(key) for key in fields)
            or current.get('status') != 'completed' or baseline.get('status') != 'completed'
            or current['runtime'].get('backend') != baseline['runtime'].get('backend')
            or current['description'].get('model_id') != baseline['description'].get('model_id')):
        raise ValueError('Compare completed benchmarks with the same model, backend and test definition')
    initial = [np.asarray(value.get('start_q', []), dtype=float) for value in (current, baseline)]
    if (initial[0].ndim != 1 or not initial[0].size or initial[0].shape != initial[1].shape
            or not all(np.isfinite(q).all() for q in initial)
            or not np.allclose(*initial, atol=.001, rtol=0)):
        raise ValueError('Benchmark starting joint positions must match within 0.001 rad')
    metrics = ('execute_to_settled_s', 'planning_ms', 'peak_feedback_rad_s', 'estimated_position_lag_ms')
    def summary(report, key):
        values = []
        for item in report['operations']:
            measured = item.get('measurement', {})
            peak = measured.get('feedback_peak_velocity_rad_s')
            value = (max(peak) if peak else None) if key == 'peak_feedback_rad_s' else (
                measured.get(key) if key == 'estimated_position_lag_ms' else item.get(key))
            if value is not None:
                values.append(value)
        return float(np.median(values)) if values else None
    result = {}
    for key in metrics:
        now, before = summary(current, key), summary(baseline, key)
        result[key] = {'baseline': before, 'current': now,
                       'change_percent': 100 * (now / before - 1) if now is not None and before else None}
    parameters = [value['descriptor'].get('parameters', {}) for value in (current, baseline)]
    config_keys = ('effective_robot_config', 'planning_limits_config', 'gain_policy', 'controller_policy')
    changed = any(parameters[0].get(key) != parameters[1].get(key) for key in config_keys)
    return {'schema': 'embodied.motion_benchmark_comparison/1', 'aggregate': 'median over out-and-return operations',
            'start_position_tolerance_rad': .001,
            'metrics': result, 'configuration_changed': changed,
            'source_changed': parameters[0].get('source_sha256') != parameters[1].get('source_sha256'),
            'baseline_runtime': baseline['runtime'], 'current_runtime': current['runtime']}


def benchmark(robot, *, joint=0, delta_rad=math.radians(10), repeats=1, speed=0., output='outputs/benchmarks'):
    """Execute explicit out-and-back MoveJ pairs and retain the exact raw ticks.

    The caller owns Robot's lifecycle. Existing limits and completion rules apply;
    a failed run produces a report and propagates the error, never a success label.
    """
    if type(joint) is not int or joint < 0 or type(repeats) is not int or repeats < 1:
        raise ValueError('joint must be zero-based and repeats a positive integer')
    if not math.isfinite(delta_rad) or delta_rad == 0:
        raise ValueError('delta_rad must be finite and nonzero')
    start_q = list(robot.q)
    if joint >= len(start_q):
        raise ValueError('joint is outside the connected robot')
    target = list(start_q)
    target[joint] += delta_rad
    # Preview before opening collection or moving; invalid goals fail early.
    robot.plan_movej(target, speed=speed)
    folder = Path(output) / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S-') + str(time.monotonic_ns()))
    folder.mkdir(parents=True)
    ticks, operations, errors = [], [], []
    done = threading.Event()
    progress = threading.Condition()
    identity = dict(robot._connected().identity)
    report = {'schema': 'embodied.motion_benchmark/1', 'runtime': identity,
              'description': robot.describe(), 'joint': joint, 'delta_rad': delta_rad,
              'start_q': start_q,
              'repeats': repeats, 'requested_speed_rad_s': speed, 'operations': operations,
              'status': 'running', 'error': None}
    reader = RuntimeReader(robot.run, 'full', timeout=.5)
    report['descriptor'] = reader.descriptor
    def collect():
        try:
            with (folder / 'ticks.jsonl').open('x') as stream:
                while not done.is_set():
                    tick = reader.read()
                    if tick.get('terminal') or tick.get('error'):
                        raise RuntimeError(tick.get('error') or 'Runtime ended during benchmark')
                    stream.write(json.dumps(tick, allow_nan=False) + '\n')
                    with progress:
                        ticks.append(tick)
                        progress.notify_all()
        except Exception as exc:
            if not done.is_set():
                # Serialize failure with submission: an in-flight execute must
                # finish acquiring its operation before cancellation is sent.
                with progress:
                    errors.append(f'{type(exc).__name__}: {exc}')
                    progress.notify_all()
                robot._cancel_on_error()
    thread = threading.Thread(target=collect, name='motion-benchmark-observer', daemon=True)
    thread.start()
    caught = None
    try:
        for repeat in range(repeats):
            for direction, goal in (('out', target), ('return', start_q)):
                if errors:
                    raise RuntimeError(errors[0])
                began = time.monotonic_ns()
                plan = robot.plan_movej(goal, speed=speed)
                planned = time.monotonic_ns()
                item = {'repeat': repeat, 'direction': direction, 'planning_ms': (planned - began) / 1e6,
                        'plan_metrics': plan['metrics'], 'limits': plan.get('limits'), 'status': 'running'}
                operations.append(item)
                with progress:
                    if errors:
                        raise RuntimeError(errors[0])
                    motion = robot.execute(plan, wait=False)
                item['operation_id'] = motion.operation_id
                motion.wait()
                finished = time.monotonic_ns()
                with progress:
                    caught_up = progress.wait_for(
                        lambda: errors or (ticks and ticks[-1]['timing']['feedback_received_ns'] >= finished),
                        timeout=2.,
                    )
                    if errors:
                        raise RuntimeError(errors[0])
                    if not caught_up:
                        raise RuntimeError('Observer did not catch up with motion completion; report would be truncated')
                    samples = [tick for tick in ticks if planned <= tick['timing']['feedback_received_ns'] <= finished]
                measured = analyze_motion(samples)
                if min(measured['feedback_samples'], measured['command_samples']) < 2:
                    raise RuntimeError('Insufficient motion telemetry; performance cannot be measured')
                item.update(status='completed', execute_to_settled_s=(finished - planned) / 1e9,
                            measurement=measured)
        report['status'] = 'completed'
    except BaseException as exc:
        caught = exc
        robot._cancel_on_error()
        report.update(status='failed', error=f'{type(exc).__name__}: {exc}')
        if operations and operations[-1]['status'] == 'running':
            operations[-1]['status'] = 'failed'
    finally:
        done.set()
        thread.join(timeout=1)
        reader.close()
        if thread.is_alive():
            errors.append('Observer did not stop')
        if errors:
            report.update(status='failed', error=report['error'] or errors[0])
        report['raw_ticks'] = 'ticks.jsonl'
        save_json(folder / 'report.json', report)
        lines = ['# Motion benchmark', '', f"Status: {report['status']}", '',
                 '| Direction | Plan (s) | Execute → settled (s) | Peak feedback (rad/s) | Candidate pauses |',
                 '| --- | ---: | ---: | ---: | ---: |']
        for item in operations:
            stats = item.get('measurement', {})
            peak = stats.get('feedback_peak_velocity_rad_s')
            lines.append(f"| {item['direction']} | {item['plan_metrics']['duration_s']:.3f} | "
                         f"{item.get('execute_to_settled_s', '—')} | {max(peak) if peak else '—'} | "
                         f"{stats.get('candidate_pause_count', '—')} |")
        lines.extend(['', 'Candidate pauses use declared thresholds, not a sensor-independent diagnosis.',
                      'Independent timestamps, per-joint errors, cycle percentiles and limiting factors are in report.json.'])
        (folder / 'report.md').write_text('\n'.join(lines) + '\n')
    if caught is not None:
        raise caught
    if errors:
        raise RuntimeError(errors[0])
    return folder
