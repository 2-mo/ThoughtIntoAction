"""Deterministic collector races using fake readers/robots; no sockets or devices."""
import copy
import importlib
import json
from queue import Empty, Queue
import threading
import time
from types import SimpleNamespace

import pytest

subject = importlib.import_module('embodied.applications.operations.benchmark')


class Reader:
    descriptor = {'parameters': {}}

    def __init__(self):
        self.items = Queue()
        self.closed = False
        self.sequence = 0

    def read(self):
        try:
            value = self.items.get(timeout=.25)
        except Empty as exc:
            raise TimeoutError('fake reader idle') from exc
        if isinstance(value, BaseException):
            raise value
        return value() if callable(value) else value

    def tick(self, position, speed):
        stamp = time.monotonic_ns()
        row = {'runtime_id': 'fake', 'sequence': self.sequence,
               'timing': {'tick_ns': stamp, 'feedback_received_ns': stamp,
                          'command_submitted_ns': stamp - 1},
               'feedback': {'q': [position] + [0.] * 6, 'dq': [speed] + [0.] * 6},
               'command': {'q': [position] + [0.] * 6, 'dq': [speed] + [0.] * 6}}
        self.sequence += 1
        return row

    def close(self):
        self.closed = True


class Robot:
    q = [0.] * 7
    run = '/unused/fake-runtime'

    def __init__(self):
        self.plans = 0
        self.executed = []
        self.cancelled = threading.Event()
        self.plan_hook = lambda: None
        self.execute_hook = lambda plan: SimpleNamespace(operation_id='fake', wait=lambda: None)

    def _connected(self):
        return SimpleNamespace(identity={'runtime_id': 'fake', 'backend': 'sim'})

    def describe(self):
        return {'model_id': 'seven'}

    def plan_movej(self, goal, *, speed):
        self.plans += 1
        self.plan_hook()
        return {'goal': goal, 'metrics': {'duration_s': .1}}

    def execute(self, plan, *, wait):
        assert wait is False
        self.executed.append(plan)
        return self.execute_hook(plan)

    def _cancel_on_error(self):
        self.cancelled.set()


def test_collector_failure_while_planning_prevents_execute_and_closes_reader(tmp_path, monkeypatch):
    reader, robot = Reader(), Robot()
    monkeypatch.setattr(subject, 'RuntimeReader', lambda *args, **kwargs: reader)

    def fail_during_second_plan():
        if robot.plans == 2:  # The first preview deliberately precedes the collector.
            reader.items.put(RuntimeError('collector failed during planning'))
            assert robot.cancelled.wait(2), 'collector did not report its failure'

    robot.plan_hook = fail_during_second_plan
    with pytest.raises(RuntimeError, match='collector failed during planning'):
        subject.benchmark(robot, output=tmp_path)
    assert robot.executed == [], 'failure was already known before execute'
    assert reader.closed
    report = json.loads(next(tmp_path.glob('*/report.json')).read_text())
    assert report['status'] == 'failed'
    assert report['operations'][0]['status'] == 'failed'


def test_report_waits_for_collector_tail_and_retains_peak_and_start_pose(tmp_path, monkeypatch):
    reader, robot = Reader(), Robot()
    monkeypatch.setattr(subject, 'RuntimeReader', lambda *args, **kwargs: reader)
    returned = [threading.Event(), threading.Event()]
    release = [threading.Event(), threading.Event()]
    analyzed = [threading.Event(), threading.Event()]
    measurements = []
    original_analyze = subject.analyze_motion

    def analyze(ticks, **kwargs):
        result = original_analyze(ticks, **kwargs)
        measurements.append(result)
        analyzed[len(measurements) - 1].set()
        return result

    monkeypatch.setattr(subject, 'analyze_motion', analyze)

    def execute(plan):
        index = len(robot.executed) - 1
        entered = threading.Event()
        held = []
        goal = plan['goal'][0]
        start = 0. if index == 0 else .1

        def wait():
            reader.items.put(reader.tick(start, .1))
            reader.items.put(reader.tick((start + goal) / 2, .1))

            def delayed_tail():
                # Reaching this read proves both preceding rows were collected.
                entered.set()
                if not release[index].wait(2):
                    raise TimeoutError('test did not release collector tail')
                return held[0]

            reader.items.put(delayed_tail)
            assert entered.wait(2), 'collector did not reach the tail barrier'
            # This peak happened before Motion.wait returned, but arrives late.
            held.append(reader.tick(goal, 1.2))
            # The following row is generated after release, proving catch-up.
            reader.items.put(lambda: reader.tick(goal, 0.))
            returned[index].set()

        return SimpleNamespace(operation_id=f'op-{index}', wait=wait)

    robot.execute_hook = execute
    outcome = {}

    def run():
        try:
            outcome['folder'] = subject.benchmark(robot, delta_rad=.1, output=tmp_path)
        except BaseException as exc:
            outcome['error'] = exc

    worker = threading.Thread(target=run)
    worker.start()
    try:
        for index in range(2):
            assert returned[index].wait(2), outcome.get('error')
            assert not analyzed[index].wait(.05), 'measurement ran before the collector caught up'
            release[index].set()
        worker.join(2)
        assert not worker.is_alive()
        assert 'error' not in outcome, outcome.get('error')
    finally:
        for event in release:
            event.set()
        worker.join(3)
    assert reader.closed
    report = json.loads((outcome['folder'] / 'report.json').read_text())
    assert report['status'] == 'completed'
    assert report['start_q'] == [0.] * 7
    assert not robot.cancelled.is_set()
    for operation in report['operations']:
        measured = operation['measurement']
        assert measured['feedback_samples'] == 3
        assert measured['feedback_peak_velocity_rad_s'][0] == 1.2
        assert abs(measured['measured_joint_delta_rad'][0]) == pytest.approx(.1)


def test_baseline_requires_matching_start_pose_with_declared_tolerance():
    baseline = {'schema': 'embodied.motion_benchmark/1', 'joint': 0, 'delta_rad': .1,
                'repeats': 1, 'requested_speed_rad_s': 0., 'status': 'completed',
                'runtime': {'backend': 'sim'}, 'description': {'model_id': 'seven'},
                'descriptor': {'parameters': {}}, 'operations': [], 'start_q': [0.] * 7}
    current = copy.deepcopy(baseline)
    current['start_q'][0] = .0005
    assert subject.compare_reports(current, baseline)['start_position_tolerance_rad'] == .001
    current['start_q'][0] = .01
    with pytest.raises(ValueError):
        subject.compare_reports(current, baseline)
