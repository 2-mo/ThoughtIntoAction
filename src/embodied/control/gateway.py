"""Bounded command ingress. Socket handling and planning never run on the robot thread."""
import hashlib
import inspect
import json
import math
import os
from pathlib import Path
import queue
import socket
import threading
import time
from collections import OrderedDict

import numpy as np

from embodied.contracts.rpc import receive, send
from embodied.control.commands import CommandEngine, MotionPlanner

# Robot facts clients read instead of keeping their own copies (todo C-01, R-01).
DESCRIBED_ROBOT_FIELDS = ('variant', 'control_hz', 'joint_min', 'joint_max', 'park_q', 'gripper_max_width',
                          'torque_max', 'joint_step_limit', 'can_interface')
ACTIONS = ('describe', 'status', 'acquire', 'source-metadata', 'heartbeat', 'release', 'hold', 'cancel', 'stop',
           'preview', 'execute', 'free-drive', 'joint', 'cartesian', 'gripper')


class Gateway:
    def __init__(self, directory, runtime_id, config, planner_factory=MotionPlanner, ingresses=None, *, motion_limits=None, capabilities=None):
        from embodied.control.motion_limits import MotionLimits
        self.directory = Path(directory)
        self.model_id = config.model_id
        self.ingresses = ingresses or {}
        self.config = config
        self.capabilities = capabilities or {}
        self.motion_limits = motion_limits or MotionLimits.load(config)
        self.engine = CommandEngine(runtime_id, config, self.model_id, self.ingresses, motion_limits=self.motion_limits)
        parameters = inspect.signature(planner_factory).parameters
        accepts_limits = ('motion_limits' in parameters or any(
            item.kind == inspect.Parameter.VAR_KEYWORD for item in parameters.values()))
        self.planner = (planner_factory(config, self.model_id, motion_limits=self.motion_limits)
                        if accepts_limits else planner_factory(config, self.model_id))
        self.pending = queue.Queue(maxsize=64)
        self.snapshot = None
        self.closed = threading.Event()
        self.plan_lock = threading.Lock()
        self.metadata_lock = threading.Lock()
        self.metadata_events = []
        self.plans = OrderedDict()
        self.slots = threading.BoundedSemaphore(8)
        self.path = self.directory / 'commands.sock'
        self.server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        descriptor = os.open(self.directory, os.O_RDONLY | os.O_DIRECTORY)
        try:
            self.server.bind(f'/proc/self/fd/{descriptor}/{self.path.name}')
        finally:
            os.close(descriptor)
        self.path.chmod(0o600)
        self.server.listen(8)
        self.server.settimeout(.1)
        self.thread = threading.Thread(target=self._serve, daemon=True)
        self.thread.start()

    def _serve(self):
        while not self.closed.is_set():
            try:
                sock, _ = self.server.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            if not self.slots.acquire(blocking=False):
                sock.close()
                continue
            threading.Thread(target=self._peer, args=(sock,), daemon=True).start()

    def _peer(self, sock):
        try:
            with sock:
                sock.settimeout(2)
                try:
                    request = receive(sock)
                    if (request.get('version') != 1 or request.get('runtime_id') != self.engine.runtime_id
                            or any(key.startswith('_') for key in request)):
                        raise ValueError('Invalid command envelope')
                    action = request.get('action')
                    if action in ('acquire', 'source-metadata'):
                        metadata = request.get('metadata', {})
                        if not isinstance(metadata, dict) or len(json.dumps(metadata, allow_nan=False)) > 8192:
                            raise ValueError('Source metadata must be a bounded JSON object')
                        if len(self.metadata_events) >= 1024:
                            raise ValueError('Source metadata history full; start a new runtime')
                    if action == 'describe':
                        send(sock, {'ok': True, 'result': self.describe()})
                        return
                    snapshot = self.snapshot
                    if snapshot is None:
                        raise ValueError('Worker feedback not ready')
                    if action == 'status':
                        result = snapshot
                    elif action == 'preview':
                        with self.plan_lock:
                            plan = self.planner.plan(request, snapshot, time.monotonic())
                            self.plans[plan['plan_id']] = plan
                            while len(self.plans) > 64:
                                self.plans.popitem(last=False)
                            tcp = self._tcp_path(plan['q'])
                        workspace = getattr(self.planner, 'workspace', None)
                        result = {**{key: value for key, value in plan.items() if not key.startswith('_')},
                                  'q': plan['q'].tolist(), 'collision_check': 'none', 'tcp': tcp,
                                  'workspace': workspace.describe() if workspace else None}
                    else:
                        if action in ('cartesian', 'joint'):
                            self._check_workspace(action, request)
                        if action == 'execute':
                            with self.plan_lock:
                                plan = self.plans.get(request.get('plan_id'))
                            if plan is None:
                                raise ValueError('Unknown preview plan')
                            request['_plan'] = plan
                        # Client deadlines use this host monotonic clock and cannot be renewed by queue delay.
                        stamp = request.get('sent_monotonic_ns')
                        if type(stamp) is not int or not 0 <= time.monotonic_ns()-stamp <= 250_000_000:
                            raise ValueError('Stale command envelope')
                        future = {'request': request, 'done': threading.Event(), 'deadline': stamp + 250_000_000}
                        self.pending.put_nowait(future)
                        if not future['done'].wait(.5):
                            raise TimeoutError('Worker ACK timed out; query operation status before retry')
                        if 'error' in future:
                            raise ValueError(future['error'])
                        result = future['result']
                        if action in ('acquire', 'source-metadata'):
                            with self.metadata_lock:
                                self.metadata_events.append({'source': result['source'], 'epoch': result['epoch'],
                                    'host_monotonic_ns': time.monotonic_ns(), 'metadata': metadata})
                                temporary = self.directory / 'source-metadata.partial'
                                temporary.write_text(json.dumps(self.metadata_events, allow_nan=False, indent=2))
                                temporary.replace(self.directory / 'source-metadata.json')
                    send(sock, {'ok': True, 'result': result})
                except Exception as exc:
                    try:
                        send(sock, {'ok': False, 'error': f'{type(exc).__name__}: {exc}'})
                    except OSError:
                        pass
        finally:
            self.slots.release()

    def describe(self):
        """Read-only; needs no lease or feedback."""
        robot = {name: getattr(self.config, name) for name in DESCRIBED_ROBOT_FIELDS if hasattr(self.config, name)}
        robot['joint_count'] = len(self.config.park_q)
        robot['gripper_velocity_max_m_s'] = self.capabilities.get('gripper_velocity_max_m_s', self.config.gripper_velocity_max_m_s)
        workspace = getattr(self.planner, 'workspace', None)
        motion_limits = getattr(self, 'motion_limits', getattr(self.planner, 'motion_limits', None))
        return {'protocol': 1, 'runtime_id': self.engine.runtime_id, 'model_id': self.model_id,
                'robot': {k: list(v) if isinstance(v, tuple) else v for k, v in robot.items()},
                'workspace': workspace.describe() if workspace else None,
                'motion_limits': motion_limits.describe() if motion_limits else None, 'actions': list(ACTIONS)}

    def _tcp_path(self, qs, points=160):
        """Base TCP poses [x, y, z, qx, qy, qz, qw] along a plan, at most `points` samples, ending at the goal."""
        fk = getattr(getattr(self.planner, 'model', None), 'fk', None)
        if fk is None:
            return None
        index = sorted({*np.linspace(0, len(qs) - 1, min(points, len(qs))).round().astype(int).tolist()})
        return [np.asarray(fk(qs[i]), dtype=float).tolist() for i in index]

    def _check_workspace(self, action, request):
        # Streamed targets from any gateway source; the robot thread only arbitrates.
        workspace = getattr(self.planner, 'workspace', None)
        try:
            target = [float(v) for v in request.get('pose' if action == 'cartesian' else 'q')]
        except (TypeError, ValueError):
            return  # Malformed targets are rejected by CommandEngine with its own message.
        if workspace is None or len(target) != (7 if action == 'cartesian' else len(self.config.park_q)) or not all(map(math.isfinite, target)):
            return
        if action == 'cartesian':
            workspace.check_pose(target)
        else:
            with self.plan_lock:
                workspace.check_path(self.planner.model.fk, [target])

    def tick(self, state, now):
        # Limit per-tick ingress work; no network wait, file IO, or planning here.
        for _ in range(8):
            try:
                future = self.pending.get_nowait()
            except queue.Empty:
                break
            try:
                if time.monotonic_ns() >= future['deadline']:
                    raise ValueError('Command expired in ingress queue')
                future['result'] = self.engine.request(future['request'], state, time.monotonic())
            except Exception as exc:
                future['error'] = str(exc)
            finally:
                future['done'].set()
        # Resolve ownership after this tick's queued requests, so a candidate from
        # the previous owner can never cross an acquire/release boundary.
        owner = self.engine.owner
        candidates = {name: ingress.poll(state, now, self.engine.epoch,
                      bool(owner and owner['ingress'] == name)) for name, ingress in self.ingresses.items()}
        command = self.engine.next(state, now, candidates)
        self.snapshot = {**self.engine.status(state, now),
                         'ingresses': {name: dict(ingress.status) for name, ingress in self.ingresses.items()}}
        return command

    def close(self):
        self.closed.set()
        self.server.close()
        self.thread.join(timeout=1)
        self.path.unlink(missing_ok=True)
        for ingress in self.ingresses.values():
            ingress.close()
