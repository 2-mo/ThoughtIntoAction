"""Worker-owned authority and bounded command execution; no sockets or SDK ownership."""
import hashlib
import json
import math
import time
import uuid
from collections import OrderedDict, deque

import numpy as np
from embodied.contracts.control import ControlCommand, Mode
from embodied.motion.validation import finite_vector
from embodied.motion.profiles import smoothstep9_derivatives


class PlannedTrajectory:
    """Evaluate one time-scaled path without constructing splines on the worker.

    The public description is JSON-compatible and part of the preview digest.
    This private cached evaluator keeps its arrays ready for the 250 Hz loop.
    """
    def __init__(self, duration, start, goal, *, knots=None, coefficients=None):
        self.duration = float(duration)
        self.start, self.goal = np.asarray(start).copy(), np.asarray(goal).copy()
        self.delta = self.goal - self.start
        self.knots = None if knots is None else np.asarray(knots).copy()
        self.coefficients = None if coefficients is None else np.asarray(coefficients).copy()

    def describe(self):
        result = {'time_law': 'smoothstep9', 'duration_s': self.duration,
                  'start': self.start.tolist(), 'goal': self.goal.tolist()}
        if self.knots is not None:
            result.update(knots=self.knots.tolist(), coefficients=self.coefficients.tolist())
        return result

    def sample(self, elapsed):
        if elapsed <= 0:
            return self.start.copy(), np.zeros_like(self.start)
        if elapsed >= self.duration:
            return self.goal.copy(), np.zeros_like(self.start)
        from embodied.contracts.vendor.arx_motion.profiles import smoothstep9_fraction
        alpha = elapsed / self.duration
        fraction = smoothstep9_fraction(alpha)
        speed = 630. * alpha**4 * (1. - alpha)**4 / self.duration
        if self.knots is None:
            return self.start + fraction * self.delta, speed * self.delta
        index = int(np.clip(np.searchsorted(self.knots, fraction, side='right') - 1,
                            0, len(self.knots) - 2))
        offset = fraction - self.knots[index]
        a, b, c, d = self.coefficients[:, index]
        return ((a * offset + b) * offset + c) * offset + d, ((3. * a * offset + 2. * b) * offset + c) * speed


class RuckigTrajectory:
    """Keep the shared planner's exact evaluator for the command loop."""
    def __init__(self, trajectory, start, goal, initial_velocity):
        self.trajectory = trajectory
        self.duration = float(trajectory.duration_s)
        self.start, self.goal = np.asarray(start).copy(), np.asarray(goal).copy()
        self.initial_velocity = np.asarray(initial_velocity).copy()

    def describe(self):
        return {'time_law': 'ruckig', 'duration_s': self.duration,
                'start': self.start.tolist(), 'goal': self.goal.tolist(),
                'initial_velocity': self.initial_velocity.tolist()}

    def sample(self, elapsed):
        if elapsed >= self.duration:
            return self.goal.copy(), np.zeros_like(self.goal)
        q, dq, _ = self.trajectory.at_time(max(0., float(elapsed)))
        return np.asarray(q), np.asarray(dq)


class StationaryTrajectory:
    """A zero-distance request only needs a reference and settling feedback."""
    def __init__(self, q, duration):
        self.goal, self.duration = np.asarray(q).copy(), float(duration)

    def describe(self):
        return {'time_law': 'stationary', 'duration_s': self.duration, 'goal': self.goal.tolist()}

    def sample(self, elapsed):
        return self.goal.copy(), np.zeros_like(self.goal)


class SettlingWindow:
    """Require near-goal, low-motion feedback over a complete 250 ms window.

    Motor velocity readouts are quantized even at rest. Average their absolute
    values over time (never signed velocities), and independently bound the
    measured position span by the same velocity budget. No gain or motion limit
    is changed. Missing/invalid samples cannot establish a settled interval.
    """
    def __init__(self, settings):
        self.duration = settings['duration_s']
        self.position_limit = settings['position_tolerance_rad']
        self.velocity_limit = settings['velocity_tolerance_rad_s']
        self.max_gap = settings['max_sample_gap_s']
        self.samples = deque()

    def clear(self):
        self.samples.clear()

    def update(self, now, q, dq, goal):
        if (not np.isfinite(now) or not np.isfinite(q).all() or not np.isfinite(dq).all()
                or np.max(np.abs(q - goal)) >= self.position_limit):
            self.clear()
            return False
        if self.samples and not 0 < now - self.samples[-1][0] <= self.max_gap:
            self.clear()
        self.samples.append((now, q.copy(), np.abs(dq)))
        cutoff = now - self.duration
        while len(self.samples) > 1 and self.samples[1][0] <= cutoff:
            self.samples.popleft()
        elapsed = now - self.samples[0][0]
        if elapsed < self.duration:
            return False
        times, positions, speeds = zip(*self.samples)
        times, speeds = np.asarray(times), np.asarray(speeds)
        # The first retained sample precedes the window boundary. Clip its
        # interval so earlier low speeds cannot dilute the last 250 ms.
        fraction = (cutoff - times[0]) / (times[1] - times[0])
        speeds[0] += fraction * (speeds[1] - speeds[0])
        times[0] = cutoff
        mean_speed = np.trapz(speeds, x=times, axis=0) / self.duration
        return bool(np.all(mean_speed < self.velocity_limit)
                    and np.all(np.ptp(positions, axis=0) < self.velocity_limit * self.duration))


class CommandEngine:
    def __init__(self, runtime_id, config, model_id, ingress_channels=(), motion_limits=None, capabilities=None):
        from embodied.control.motion_limits import MotionLimits
        self.runtime_id, self.config, self.model_id = runtime_id, config, model_id
        self.motion_limits = motion_limits if motion_limits is not None else MotionLimits.load(config)
        self.gripper_capabilities = (capabilities or {}).get('gripper', {'present': config.gripper_max_width > 0})
        self.gripper_hold_effort = None
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
        self.completed_target = None
        self.operations = OrderedDict()
        self.reset_reference = True
        self.stopped = False
        self.phase = 'idle'
        self.source_sequence = -1
        self.settling = SettlingWindow(self.motion_limits.settling)

    def clear(self, reason='released', *, preserve_completed=False):
        self.gripper_hold_effort = None
        self.settling.clear()
        if not preserve_completed or self.operation is not None:
            self.completed_target = None
        if self.operation:
            self.operation['status'] = 'cancelled'
            self.operation['reason'] = reason
        self.operation = self.command = None
        self.last_revocation = {'reason': reason, 'epoch': self.epoch,
                                'source': self.owner['source'] if self.owner else None}
        self.owner = None
        self.epoch += 1
        self.reset_reference = self.completed_target is None
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
            self.completed_target = None
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
            # Authority is released, while a successfully settled terminal
            # reference belongs to the kernel until the next explicit action.
            self.clear(preserve_completed=True)
            return {'status': 'applied'}
        if action in ('hold', 'cancel'):
            self.gripper_hold_effort = None
            self.settling.clear()
            if self.operation:
                self.operation['status'] = 'cancelled'
            self.operation = self.command = None
            self.completed_target = None
            self.reset_reference = True
            self.phase = 'hold'
            self.ingress_paused = True
            return {'status': 'applied', 'physical_stopped': False}
        if self.owner['ingress'] != 'rpc':
            raise ValueError('This lease consumes an installed ingress; release it before RPC motion')
        if action == 'grasp':
            if self.operation:
                raise ValueError('Finish or cancel the active operation before grasping')
            from embodied.control.grasp import Grasp
            grasp = Grasp(request, state, self.gripper_capabilities, now)
            key = uuid.uuid4().hex
            self.operation = {'operation_id': key, 'status': 'running', 'started': now,
                              'kind': 'grasp', 'plan': grasp}
            self.operations[key] = self.operation
            while len(self.operations) > 64:
                self.operations.popitem(last=False)
            self.command = self.completed_target = None
            self.gripper_hold_effort = None
            self.phase = 'grasp'
            return {'operation_id': key, 'status': 'running'}
        if action == 'execute':
            plan = request['_plan']  # Added by the bounded gateway, never trusted from a client.
            expected_epoch = plan['epoch'] if plan['owner_present'] else plan['epoch'] + 1
            if expected_epoch != self.epoch or plan['model_id'] != self.model_id or now > plan['expires']:
                raise ValueError('Preview expired or its owner/model changed')
            key = plan['plan_id']
            if key in self.operations:
                return {k: v for k, v in self.operations[key].items() if k != 'plan'}
            start_tolerance = self.settling.position_limit if plan.get('_trajectory') is not None else .03
            if np.max(np.abs(state.q - plan['q'][0])) > start_tolerance or np.max(np.abs(state.dq)) > .05:
                raise ValueError('Robot moved since preview')
            if self.operation:
                raise ValueError('Cancel the active operation first')
            self.settling.clear()
            self.completed_target = None
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
            self.gripper_hold_effort = None
            # Optional gripper_width keeps the gripper position-controlled while the arm is free (teaching).
            width = request.get('gripper_width')
            if width is not None and (isinstance(width, bool) or not isinstance(width, (int, float))
                                      or not np.isfinite(width) or not 0 <= width <= self.config.gripper_max_width):
                raise ValueError('Invalid gripper width in metres')
            self.command = (ControlCommand(Mode.FREE_DRIVE, gripper_mode=Mode.FREE_DRIVE) if width is None else
                            ControlCommand(Mode.FREE_DRIVE, gripper_mode=Mode.COMMAND_FOLLOWING, gripper_target=float(width)))
        elif action in ('joint', 'cartesian', 'gripper'):
            if action == 'gripper':
                if not self.gripper_capabilities.get('present'):
                    raise ValueError('The connected robot has no gripper')
                self.gripper_hold_effort = None
            q = None
            pose = None
            if action == 'joint':
                q = finite_vector(request.get('q'), len(self.config.park_q), 'joint target')
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
                joint_target=q, gripper_target=width, hold=action == 'gripper',
                joint_step_limit=self.motion_limits.step_limits())
        else:
            raise ValueError('Unsupported control action')
        self.completed_target = None
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
        if self.owner and self.operation and self.operation['kind'] == 'grasp':
            op, grasp = self.operation, self.operation['plan']
            try:
                effort = grasp.update(state, now)
                command = ControlCommand(Mode.COMMAND_FOLLOWING, joint_target=grasp.arm_q.copy(),
                    gripper_target=state.gripper_width, gripper_effort=effort, hold=True)
                if grasp.result is not None:
                    op.update(status='completed', result=grasp.result)
                    self.operation = None
                    self.gripper_hold_effort = grasp.effort if grasp.result['confirmed'] else None
                    command.gripper_effort = self.gripper_hold_effort
                    self.completed_target = (grasp.arm_q.copy(), state.gripper_width, None, False)
            except (ValueError, RuntimeError) as exc:
                op.update(status='failed', reason=str(exc))
                self.operation = self.completed_target = None
                self.gripper_hold_effort = None
                self.reset_reference = True
        elif self.owner and self.operation:
            op, plan = self.operation, self.operation['plan']
            elapsed = now - op['started']
            trajectory = plan.get('_trajectory')
            if trajectory is not None:
                q, dq = trajectory.sample(elapsed)
            else:
                # Older/custom q-only planner plugins retain their original
                # position-only semantics instead of inventing a velocity.
                q = np.array([np.interp(elapsed, plan['times'], plan['q'][:, i]) for i in range(len(self.config.park_q))])
                dq = None
            command = ControlCommand(Mode.COMMAND_FOLLOWING, joint_target=q,
                joint_velocity_target=dq, gripper_target=plan['gripper_width'],
                joint_step_limit=self.motion_limits.step_limits(plan.get('limits')),
                planned=trajectory is not None)
            if elapsed >= plan['times'][-1]:
                if self.settling.update(now, state.q, state.dq, q):
                    op['status'] = 'completed'
                    self.operation = None
                    self.completed_target = (q.copy(), plan['gripper_width'], plan.get('limits'), trajectory is not None)
                    self.settling.clear()
                elif elapsed > plan['times'][-1] + self.motion_limits.settling['timeout_s']:
                    op['status'] = 'failed'
                    op['reason'] = 'Robot did not settle'
                    errors = np.abs(state.q - q)
                    op['settling'] = {
                        'joint_error_rad': errors.tolist(), 'worst_joint': int(np.argmax(errors)) + 1,
                        'max_joint_error_rad': float(np.max(errors)),
                        'position_tolerance_rad': self.settling.position_limit,
                        'trajectory_duration_s': float(plan['times'][-1]),
                        'settling_elapsed_s': elapsed - float(plan['times'][-1]),
                    }
                    self.operation = None
                    self.completed_target = None
                    self.reset_reference = True
                    self.settling.clear()
                    command = None
        elif self.owner and self.command is not None and now < self.command_expires:
            command = self.command
        elif (self.owner and candidate is not None and not self.ingress_paused
              and candidate.epoch == self.epoch and now < candidate.expires_at):
            command = candidate.command
            command.joint_step_limit = self.motion_limits.step_limits()
            self.source_sequence = candidate.sequence
            self.phase = candidate.phase
        elif self.completed_target is not None:
            q, width, limits, planned = self.completed_target
            command = ControlCommand(Mode.COMMAND_FOLLOWING, joint_target=q.copy(),
                joint_velocity_target=np.zeros_like(q), gripper_target=width,
                joint_step_limit=self.motion_limits.step_limits(limits), planned=planned)
            self.phase = 'hold'
        else:
            self.phase = 'hold'
        if command is None or self.reset_reference:
            self.phase = 'hold'
            command = ControlCommand(Mode.COMMAND_FOLLOWING, hold=True, gripper_target=state.gripper_width)
        if self.gripper_hold_effort is not None:
            command.gripper_effort = self.gripper_hold_effort
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
                'gripper_width': state.gripper_width, 'gripper_velocity': state.gripper_velocity,
                'gripper_effort': state.gripper_effort, 'gripper_contact': state.gripper_contact,
                'feedback_monotonic_ns': int(state.received_at * 1e9),
                'operations': [{k: v for k, v in op.items() if k != 'plan'} for op in self.operations.values()]}


class MotionPlanner:
    """Independent kinematics instance; never shares mutable IK data with the worker."""
    def __init__(self, config, model_id, motion_limits=None):
        from embodied.contracts.registry import Registry
        from embodied.control.motion_limits import MotionLimits
        from embodied.control.workspace import Workspace
        self.model = Registry().create("kinematics", config.model_plugin, runtime="control", config=config)
        self.config, self.model_id = config, model_id
        self.workspace = Workspace.load()
        self.motion_limits = motion_limits if motion_limits is not None else MotionLimits.load(config)

    def plan(self, request, snapshot, now):
        start = finite_vector(snapshot['q'], len(self.config.park_q), 'start joint position')
        kind = request.get('kind', 'movej')
        from embodied.motion.cartesian_execution import plan_path, apply_tcp_limits
        from embodied.motion.validation import tcp_limits
        requested_tcp = tcp_limits(request)
        if requested_tcp and kind not in ('movel', 'move_path'):
            raise ValueError('TCP limits apply only to movel and move_path')
        limits = self.motion_limits.resolve(request)
        if kind in ('movel', 'move_path'):
            requested_tcp = self.motion_limits.resolve_cartesian(request)
            limits['tcp'] = requested_tcp
        velocity_limit = np.asarray(limits['joint_velocity_rad_s'])
        acceleration_limit, jerk_limit = limits['acceleration'], limits['jerk']
        minimum_duration = 1. / self.config.control_hz
        if kind == 'home':
            goal = np.asarray(self.config.park_q)
        elif kind in ('movej', 'jog'):
            goal = finite_vector(request.get('q'), len(self.config.park_q), 'joint target')
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
        elif kind == 'move_path':
            trajectory, times, qs, velocity, acceleration, jerk = plan_path(
                self.model, start, request, limits, self.config.control_hz)
            duration = trajectory.duration
        else:
            raise ValueError('Unsupported motion primitive')
        if kind == 'move_path':
            limiting_factors = trajectory.limiting_factors
            derivative_evaluation = 'sampled_analytic'
        elif kind != 'movel':
            if np.any(goal < self.model.lower) or np.any(goal > self.model.upper):
                raise ValueError('Target exceeds model limits')
            if np.max(np.abs(goal - start)) < 1e-9:
                duration = minimum_duration
                times = np.array([0., duration])
                qs = np.stack([start, goal])
                velocity = acceleration = jerk = np.zeros_like(qs)
                trajectory = StationaryTrajectory(goal, duration)
                limiting_factors = ['minimum_duration']
            else:
                from embodied.motion.offline import plan_ruckig_joint_trajectory
                # Managed operations require a settled start at execute. Do
                # not inject quantized standstill motor velocity into idle axes.
                initial_velocity = np.zeros_like(start)
                curve = plan_ruckig_joint_trajectory(start, goal, initial_velocity,
                    min_position=self.model.lower, max_position=self.model.upper,
                    max_velocity=velocity_limit, max_acceleration=acceleration_limit,
                    max_jerk=jerk_limit, sample_rate_hz=self.config.control_hz,
                    minimum_duration_s=minimum_duration)
                duration = curve.duration_s
                times = np.r_[0., curve.sample_times_s]
                qs = np.vstack([start, curve.positions])
                velocity = np.vstack([initial_velocity, curve.velocities])
                acceleration = np.vstack([np.zeros_like(start), curve.accelerations])
                jerk = np.diff(acceleration, axis=0) / np.diff(times)[:, None]
                trajectory = RuckigTrajectory(curve, start, goal, initial_velocity)
                ratios = {'speed': float(np.max(np.abs(velocity) / velocity_limit)),
                          'acceleration': float(np.max(np.abs(acceleration))) / acceleration_limit,
                          'jerk': float(np.max(np.abs(jerk))) / jerk_limit}
                limiting_factors = [name for name, ratio in ratios.items() if ratio >= .995]
                if math.isclose(duration, minimum_duration, rel_tol=1e-6):
                    limiting_factors.append('minimum_duration')
            derivative_evaluation = 'native_ruckig_samples'
        else:
            # Smooth the IK path, then stretch time until sampled joint derivatives
            # satisfy the same velocity/acceleration/jerk bounds as MoveJ.
            from scipy.interpolate import CubicSpline
            from embodied.contracts.vendor.arx_motion.profiles import smoothstep9_fraction
            curve = CubicSpline(np.linspace(0, 1, len(qs)), qs, axis=0)
            u = np.linspace(0, 1, 2001)
            fractions = np.asarray([smoothstep9_fraction(float(a)) for a in u])
            qs = curve(fractions)
            ds, dds, ddds = (value[:, None] for value in smoothstep9_derivatives(u))
            path_v, path_a, path_j = (curve(fractions, order) for order in (1, 2, 3))
            v = path_v * ds
            a = path_a * ds**2 + path_v * dds
            j = path_j * ds**3 + 3. * path_a * ds * dds + path_v * ddds
            path_peaks = np.array([np.max(np.abs(values), axis=0) for values in (v, a, j)])
            timing = {'minimum_duration': minimum_duration,
                      'speed': float(np.max(path_peaks[0] / velocity_limit)),
                      'acceleration': float(np.max(path_peaks[1] / acceleration_limit))**.5,
                      'jerk': float(np.max(path_peaks[2] / jerk_limit))**(1/3)}
            duration = 1.05 * max(timing.values())
            times = u * duration
            trajectory = PlannedTrajectory(duration, qs[0], qs[-1], knots=curve.x, coefficients=curve.c)
            velocity, acceleration, jerk = v / duration, a / duration**2, j / duration**3
            limiting_factors = [name for name, value in timing.items()
                               if math.isclose(float(value), float(max(timing.values())), rel_tol=1e-9)]
            derivative_evaluation = 'sampled_analytic'
        tcp = {}
        if kind in ('movel', 'move_path'):
            trajectory, times, qs, velocity, acceleration, jerk, tcp, tcp_limited = apply_tcp_limits(
                self.model, trajectory, times, qs, velocity, acceleration, jerk,
                requested_tcp, limits, self.config.control_hz)
            duration = float(times[-1])
            if tcp_limited:
                limiting_factors = tcp_limited
        if times[-1] > 60:
            raise ValueError('Plan exceeds 60 s; split the move or increase speed')
        if np.any(qs < self.model.lower) or np.any(qs > self.model.upper):
            raise ValueError('Planned path exceeds joint limits')
        if (np.any(np.abs(velocity) > velocity_limit * 1.01)
                or np.max(np.abs(acceleration)) > acceleration_limit * 1.01
                or np.max(np.abs(jerk)) > jerk_limit * 1.01):
            raise ValueError('Planned path exceeds sampled derivative bounds')
        width = float(request.get('gripper_width', snapshot['gripper_width']))
        if not np.isfinite(width) or not 0 <= width <= self.config.gripper_max_width:
            raise ValueError('Invalid gripper width')
        self.workspace.check_path(self.model.fk, qs)
        peaks = [float(np.max(np.abs(values))) for values in (velocity, acceleration, jerk)]
        metrics = {'duration_s': float(duration), 'peak_velocity_rad_s': float(peaks[0]),
                   'peak_acceleration_rad_s2': float(peaks[1]), 'peak_jerk_rad_s3': float(peaks[2]),
                   'limiting_factors': limiting_factors,
                   'derivative_evaluation': derivative_evaluation, **tcp}
        description = trajectory.describe()
        description['limits'] = limits
        identity = {'kind': kind, 'epoch': snapshot['epoch'], 'owner_present': snapshot['owner'] is not None, 'model_id': self.model_id,
                    'q': qs.tolist(), 'times': times.tolist(), 'gripper_width': width,
                    'trajectory': description, 'limits': limits, 'metrics': metrics}
        digest = hashlib.sha256(json.dumps(identity, sort_keys=True).encode()).hexdigest()
        return {**identity, 'q': qs, '_trajectory': trajectory, 'plan_id': digest, 'expires': now + 30}
