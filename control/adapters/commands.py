"""Worker-owned authority and bounded command execution; no sockets or SDK ownership."""
import hashlib
import json
import time
import uuid
from collections import OrderedDict

import numpy as np
from arx5_vr.types import ControlCommand, Mode
from arx5_vr.model import finite_vector


class CommandEngine:
    def __init__(self, runtime_id, config, model_id, ingress_channels=()):
        self.runtime_id, self.config, self.model_id = runtime_id, config, model_id
        self.owner = None
        self.ingress_channels = frozenset(ingress_channels)
        self.ingress_paused = False
        self.last_revocation = None
        self.epoch = 0
        self.expires = 0
        self.last_sequence = -1
        self.command = None
        self.command_expires = 0
        self.operation = None
        self.operations = OrderedDict()
        self.reset_reference = True
        self.stopped = False
        self.phase = 'idle'
        self.source_sequence = -1

    def clear(self, reason='released'):
        if self.operation:
            self.operation['status'] = 'cancelled'
            self.operation['reason'] = reason
        self.operation = self.command = None
        self.last_revocation = {'reason': reason, 'epoch': self.epoch,
                                'source': self.owner['source'] if self.owner else None}
        self.owner = None
        self.epoch += 1
        self.reset_reference = True
        self.phase = reason
        self.source_sequence = -1

    def authorize(self, request, now):
        if not self.owner or now >= self.expires:
            raise ValueError('No active control lease')
        if request.get('token') != self.owner['token'] or request.get('epoch') != self.epoch:
            raise ValueError('Control owner/epoch mismatch')
        seq = request.get('sequence')
        if type(seq) is not int or seq <= self.last_sequence:
            raise ValueError('Repeated or regressed command sequence')
        self.last_sequence = seq
        self.expires = now + .5  # Lease heartbeats never refresh a target deadline.

    def request(self, request, state, now):
        if request.get('version') != 1 or request.get('runtime_id') != self.runtime_id:
            raise ValueError('Command protocol/runtime mismatch')
        action = request.get('action')
        if action == 'stop':
            self.clear('stopped')
            self.stopped = True
            return {'status': 'applied', 'stopped': True}
        if self.stopped:
            raise ValueError('Runtime stop is latched; restart explicitly')
        if self.owner and now >= self.expires:
            self.clear('lease-expired')
        if action == 'acquire':
            if self.owner:
                raise ValueError('Control is already owned; release it first')
            source = request.get('source')
            if not isinstance(source, str) or not source.isascii() or not source.replace('-', '').replace('_', '').isalnum() or len(source) > 31:
                raise ValueError('Source must be an ASCII identifier of at most 31 characters')
            if np.max(np.abs(state.dq)) > .05:
                raise ValueError('Wait for the robot to settle before transferring control')
            ingress = request.get('ingress', 'rpc')
            if ingress != 'rpc' and ingress not in self.ingress_channels:
                raise ValueError('Requested command ingress is not installed')
            purpose = request.get('purpose', 'operation')
            if purpose not in ('teleop', 'operation'):
                raise ValueError('Control purpose must be teleop or operation')
            self.epoch += 1
            self.owner = {'source': source, 'token': uuid.uuid4().hex, 'epoch': self.epoch,
                          'ingress': ingress, 'purpose': purpose}
            self.ingress_paused = False
            self.last_sequence = -1
            self.expires = now + .5
            self.reset_reference = True
            self.source_sequence = -1
            self.phase = 'acquired'
            return {**self.owner, 'q': state.q.tolist(), 'pose': state.pose.tolist(),
                    'gripper_width': state.gripper_width, 'status': 'applied'}
        self.authorize(request, now)
        if action == 'heartbeat':
            return {'status': 'applied'}
        if action == 'source-metadata':
            return {'status': 'applied', 'source': self.owner['source'], 'epoch': self.epoch}
        if action == 'release':
            self.clear()
            return {'status': 'applied'}
        if action in ('hold', 'cancel'):
            if self.operation:
                self.operation['status'] = 'cancelled'
            self.operation = self.command = None
            self.reset_reference = True
            self.phase = 'hold'
            self.ingress_paused = True
            return {'status': 'applied', 'physical_stopped': False}
        if self.owner['ingress'] != 'rpc':
            raise ValueError('This lease consumes an installed ingress; release it before RPC motion')
        if action == 'execute':
            plan = request['_plan']  # Added by the bounded gateway, never trusted from a client.
            expected_epoch = plan['epoch'] if plan['owner_present'] else plan['epoch'] + 1
            if expected_epoch != self.epoch or plan['model_id'] != self.model_id or now > plan['expires']:
                raise ValueError('Preview expired or its owner/model changed')
            key = plan['plan_id']
            if key in self.operations:
                return {k: v for k, v in self.operations[key].items() if k != 'plan'}
            if np.max(np.abs(state.q - plan['q'][0])) > .03 or np.max(np.abs(state.dq)) > .05:
                raise ValueError('Robot moved since preview')
            if self.operation:
                raise ValueError('Cancel the active operation first')
            self.operation = {'operation_id': key, 'status': 'running', 'started': now,
                              'plan': plan, 'kind': plan['kind']}
            self.operations[key] = self.operation
            while len(self.operations) > 64:
                self.operations.popitem(last=False)
            self.command = None
            self.phase = plan['kind']
            return {'operation_id': key, 'status': 'running'}
        if self.operation:
            raise ValueError('Cancel the active operation before sending targets')
        ttl = request.get('ttl_ms', 100)
        if type(ttl) is not int or not 20 <= ttl <= 250:
            raise ValueError('Target TTL must be 20..250 ms')
        stamp = request.get('sent_monotonic_ns')
        if type(stamp) is not int or not 0 <= now - stamp / 1e9 < ttl / 1000:
            raise ValueError('Target expired before application')
        source_sequence = request.get('source_sequence', -1)
        if type(source_sequence) is not int or not -1 <= source_sequence < 2**63:
            raise ValueError('Invalid source sequence')
        if action == 'free-drive':
            # Optional gripper_width keeps the gripper position-controlled while the arm is free (teaching).
            width = request.get('gripper_width')
            if width is not None and (isinstance(width, bool) or not isinstance(width, (int, float))
                                      or not np.isfinite(width) or not 0 <= width <= self.config.gripper_max_width):
                raise ValueError('Invalid gripper width in metres')
            self.command = (ControlCommand(Mode.FREE_DRIVE, gripper_mode=Mode.FREE_DRIVE) if width is None else
                            ControlCommand(Mode.FREE_DRIVE, gripper_mode=Mode.COMMAND_FOLLOWING, gripper_target=float(width)))
        elif action in ('joint', 'cartesian', 'gripper'):
            q = None
            pose = None
            if action == 'joint':
                q = finite_vector(request.get('q'), 6, 'joint target')
                if np.any(q < np.asarray(self.config.joint_min)) or np.any(q > np.asarray(self.config.joint_max)):
                    raise ValueError('Joint target exceeds model limits')
            if action == 'cartesian':
                pose = finite_vector(request.get('pose'), 7, 'TCP pose')
                if abs(np.linalg.norm(pose[3:]) - 1) > .01:
                    raise ValueError('TCP quaternion must be normalized')
                if np.linalg.norm(pose[:3] - state.pose[:3]) > .15:
                    raise ValueError('Cartesian stream target exceeds 0.15 m lead; use a previewed move')
            width = request.get('gripper_width', state.gripper_width)
            if not isinstance(width, (int, float)) or not np.isfinite(width) or not 0 <= width <= self.config.gripper_max_width:
                raise ValueError('Invalid gripper width in metres')
            self.command = ControlCommand(Mode.COMMAND_FOLLOWING, pose_target=pose,
                joint_target=q, gripper_target=width, hold=action == 'gripper', joint_step_limit=.5 / 250)
        else:
            raise ValueError('Unsupported control action')
        self.command_expires = stamp / 1e9 + ttl / 1000
        self.source_sequence = source_sequence
        self.phase = 'teleop' if self.owner['purpose'] == 'teleop' else action
        return {'status': 'applied'}

    def next(self, state, now, candidates=None):
        if self.owner and now >= self.expires:
            self.clear('lease-expired')
        candidate = (candidates or {}).get(self.owner['ingress']) if self.owner else None
        if candidate is not None and candidate.epoch == self.epoch and candidate.fault:
            self.clear(candidate.fault)
            candidate = None
        command = None
        if self.owner and self.operation:
            op, plan = self.operation, self.operation['plan']
            elapsed = now - op['started']
            q = np.array([np.interp(elapsed, plan['times'], plan['q'][:, i]) for i in range(6)])
            command = ControlCommand(Mode.COMMAND_FOLLOWING, joint_target=q,
                gripper_target=plan['gripper_width'], joint_step_limit=.5 / 250)
            if elapsed >= plan['times'][-1]:
                settled = np.max(np.abs(state.q - q)) < .015 and np.max(np.abs(state.dq)) < .02
                if settled:
                    op.setdefault('settled_since', now)
                else:
                    op.pop('settled_since', None)
                if settled and now - op['settled_since'] >= .25:
                    op['status'] = 'completed'
                    self.operation = None
                    self.reset_reference = True
                elif elapsed > plan['times'][-1] + 6:
                    op['status'] = 'failed'
                    op['reason'] = 'Robot did not settle'
                    self.operation = None
                    self.reset_reference = True
                    command = None
        elif self.owner and self.command is not None and now < self.command_expires:
            command = self.command
        elif (self.owner and candidate is not None and not self.ingress_paused
              and candidate.epoch == self.epoch and now < candidate.expires_at):
            command = candidate.command
            self.source_sequence = candidate.sequence
            self.phase = candidate.phase
        else:
            self.phase = 'hold'
        if command is None or self.reset_reference:
            self.phase = 'hold'
            command = ControlCommand(Mode.COMMAND_FOLLOWING, hold=True, gripper_target=state.gripper_width)
        if self.reset_reference:
            command.reset_reference = True
            self.reset_reference = False
        return command

    def status(self, state, now):
        return {'runtime_id': self.runtime_id, 'model_id': self.model_id, 'epoch': self.epoch,
                'owner': None if not self.owner else {k: v for k, v in self.owner.items() if k != 'token'},
                'last_revocation': self.last_revocation,
                'lease_remaining_s': max(0, self.expires-now) if self.owner else 0., 'stopped': self.stopped, 'phase': self.phase,
                'q': state.q.tolist(), 'dq': state.dq.tolist(), 'pose': state.pose.tolist(),
                'gripper_width': state.gripper_width, 'feedback_monotonic_ns': int(state.received_at * 1e9),
                'operations': [{k: v for k, v in op.items() if k != 'plan'} for op in self.operations.values()]}


class MotionPlanner:
    """Independent kinematics instance; never shares mutable IK data with the worker."""
    def __init__(self, config, model_id):
        from arx5_vr.model import X5Model
        from .workspace import Workspace
        self.model, self.config, self.model_id = X5Model(config), config, model_id
        self.workspace = Workspace.load()

    def plan(self, request, snapshot, now):
        start = np.asarray(snapshot['q'])
        kind = request.get('kind', 'movej')
        speed = float(request.get('speed', .15))
        if not np.isfinite(speed) or not .01 <= speed <= .5:
            raise ValueError('Joint speed must be .01..0.5 rad/s')
        if kind == 'home':
            goal = np.asarray(self.config.park_q)
        elif kind in ('movej', 'jog'):
            goal = finite_vector(request.get('q'), 6, 'joint target')
        elif kind == 'movel':
            target = finite_vector(request.get('pose'), 7, 'TCP pose')
            if abs(np.linalg.norm(target[3:]) - 1) > .01:
                raise ValueError('TCP quaternion must be normalized')
            from scipy.spatial.transform import Rotation, Slerp
            origin = self.model.fk(start)
            rotations = Slerp([0, 1], Rotation.from_quat([origin[3:], target[3:]]))
            path, seed = [start], start
            for fraction in np.linspace(0, 1, 101)[1:]:
                pose = np.r_[origin[:3] + fraction * (target[:3]-origin[:3]), rotations(fraction).as_quat()]
                seed, converged, residual = self.model.solve_ik(pose, seed)
                if not converged or residual > .001 or np.max(np.abs(seed-path[-1])) > .1:
                    raise ValueError('MoveL has an unreachable/discontinuous IK segment')
                path.append(seed.copy())
            qs = np.asarray(path)
        else:
            raise ValueError('Unsupported motion primitive')
        if kind != 'movel':
            if np.any(goal < self.model.lower) or np.any(goal > self.model.upper):
                raise ValueError('Target exceeds model limits')
            from embodied.contracts.vendor.arx_motion.profiles import minimum_smoothstep9_duration, smoothstep9_fraction
            duration = max(.4, minimum_smoothstep9_duration(goal-start, velocity_limit=speed, acceleration_limit=.5, jerk_limit=2.))
            times = np.linspace(0, duration, min(4000, int(duration*250)+2))
            u = times / duration
            qs = start + np.asarray([smoothstep9_fraction(float(a)) for a in u])[:, None] * (goal-start)
        else:
            # Smooth the IK path, then stretch time until sampled joint derivatives
            # satisfy the same velocity/acceleration/jerk bounds as MoveJ.
            from scipy.interpolate import CubicSpline
            from embodied.contracts.vendor.arx_motion.profiles import smoothstep9_fraction
            curve = CubicSpline(np.linspace(0, 1, len(qs)), qs, axis=0)
            u = np.linspace(0, 1, 2001)
            qs = curve([smoothstep9_fraction(float(a)) for a in u])
            v = np.gradient(qs, u, axis=0)
            a = np.gradient(v, u, axis=0)
            j = np.gradient(a, u, axis=0)
            duration = 1.05 * max(1., float(np.max(np.abs(v))) / speed,
                (float(np.max(np.abs(a))) / .5)**.5, (float(np.max(np.abs(j))) / 2.)**(1/3))
            times = u * duration
        if times[-1] > 60:
            raise ValueError('Plan exceeds 60 s; split the move or increase speed')
        if np.any(qs < self.model.lower) or np.any(qs > self.model.upper):
            raise ValueError('Planned path exceeds joint limits')
        velocity = np.gradient(qs, times, axis=0)
        acceleration = np.gradient(velocity, times, axis=0)
        jerk = np.gradient(acceleration, times, axis=0)
        if (np.max(np.abs(velocity)) > speed * 1.01 or np.max(np.abs(acceleration)) > .505
                or np.max(np.abs(jerk)) > 2.02):
            raise ValueError('Planned path exceeds sampled derivative bounds')
        width = float(request.get('gripper_width', snapshot['gripper_width']))
        if not np.isfinite(width) or not 0 <= width <= self.config.gripper_max_width:
            raise ValueError('Invalid gripper width')
        self.workspace.check_path(self.model.fk, qs)
        identity = {'kind': kind, 'epoch': snapshot['epoch'], 'owner_present': snapshot['owner'] is not None, 'model_id': self.model_id,
                    'q': qs.tolist(), 'times': times.tolist(), 'gripper_width': width}
        digest = hashlib.sha256(json.dumps(identity, sort_keys=True).encode()).hexdigest()
        return {**identity, 'q': qs, 'plan_id': digest, 'expires': now + 30}
