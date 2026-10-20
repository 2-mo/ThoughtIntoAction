"""Bounded command ingress. Socket handling and planning never run on the robot thread."""
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import socket
import threading
import time
from collections import OrderedDict

from embodied_contracts.rpc import receive, send
from .commands import CommandEngine, MotionPlanner


class Gateway:
    def __init__(self, directory, runtime_id, config, planner_factory=MotionPlanner, ingresses=None):
        self.directory = Path(directory)
        manifest = Path(config.model_path).with_suffix('.model.json')
        self.model_id = hashlib.sha256(manifest.read_bytes()).hexdigest()
        self.ingresses = ingresses or {}
        self.engine = CommandEngine(runtime_id, config, self.model_id, self.ingresses)
        self.planner = planner_factory(config, self.model_id)
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
                        workspace = getattr(self.planner, 'workspace', None)
                        result = {**plan, 'q': plan['q'].tolist(), 'collision_check': 'none',
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

    def _check_workspace(self, action, request):
        # Streamed targets from any gateway source; the robot thread only arbitrates.
        workspace = getattr(self.planner, 'workspace', None)
        try:
            target = [float(v) for v in request.get('pose' if action == 'cartesian' else 'q')]
        except (TypeError, ValueError):
            return  # Malformed targets are rejected by CommandEngine with its own message.
        if workspace is None or len(target) != (7 if action == 'cartesian' else 6) or not all(map(math.isfinite, target)):
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
