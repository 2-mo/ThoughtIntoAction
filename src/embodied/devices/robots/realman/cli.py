"""RealMan status and Quest relative-pose preview/control; no implicit activation."""
import argparse
from contextlib import ExitStack
import json
import time
import sys

import numpy as np

from embodied.contracts.control import ControlCommand, Mode
from .backend import Backend, DEFAULT_CONFIG, load_config, vector
from .geometry import matrix, quaternion, limited_rotation, rotation_vector


class RelativeTarget:
    def __init__(self, config):
        self.config = config
        self.basis = np.asarray(config['xr_to_robot_basis'])
        self.reference = None
        self.robot_reference = None
        self.origin = None
        self.limits = []

    def reset(self):
        self.reference = None

    def update(self, controller, robot_pose):
        controller = vector(controller, 7)
        if not np.isclose(np.linalg.norm(controller[3:]), 1, atol=.01):
            raise ValueError('Invalid tracked controller quaternion')
        if self.reference is None:
            self.reference = controller.copy()
            self.robot_reference = robot_pose.copy()
        if self.origin is None:
            self.origin = robot_pose.copy()
        self.limits = []
        target = self.robot_reference.copy()
        delta = self.basis @ (controller[:3] - self.reference[:3]) * self.config['position_scale']
        if np.linalg.norm(delta) > self.config['translation_radius_m']:
            self.limits.append('translation_boundary')
        delta *= min(1., self.config['translation_radius_m'] / max(np.linalg.norm(delta), 1e-12))
        target[:3] += delta
        rotation = self.basis @ matrix(controller[3:]) @ matrix(self.reference[3:]).T @ self.basis.T
        desired = quaternion(rotation @ matrix(self.robot_reference[3:]))
        if np.linalg.norm(rotation_vector(desired, self.robot_reference[3:])) > self.config['rotation_radius_rad']:
            self.limits.append('rotation_boundary')
        target[3:] = limited_rotation(desired, self.robot_reference[3:], self.config['rotation_radius_rad'])
        offset = target[:3] - self.origin[:3]
        radius = self.config['translation_radius_m']
        if np.linalg.norm(offset) > radius:
            self.limits.append('translation_boundary')
            target[:3] = self.origin[:3] + offset * radius / np.linalg.norm(offset)
        if np.linalg.norm(rotation_vector(target[3:], self.origin[3:])) > self.config['rotation_radius_rad']:
            self.limits.append('rotation_boundary')
        target[3:] = limited_rotation(target[3:], self.origin[3:], self.config['rotation_radius_rad'])
        return target


class GripGate:
    """Release to arm, squeeze to engage, release below 0.2 to disengage."""
    def __init__(self):
        self.armed = False
        self.following = False

    def update(self, grip, fresh):
        if not fresh:
            self.armed = self.following = False
        elif grip < .2:
            self.armed = True
            self.following = False
        elif grip > .8 and self.armed:
            self.following = True
        return self.following


class XRClock:
    def __init__(self, stale_s):
        self.stale_s = stale_s
        self.previous = 0
        self.updated_at = float('-inf')

    def fresh(self, timestamp, now):
        rollback = timestamp < self.previous
        if timestamp > self.previous:
            self.updated_at = now
        self.previous = timestamp
        return timestamp > 0 and not rollback and now - self.updated_at < self.stale_s


def run():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['status', 'quest', 'check-kinematics'])
    parser.add_argument('--config', default=str(DEFAULT_CONFIG))
    parser.add_argument('--execute', action='store_true', help='Permit motion; default is read-only target preview')
    parser.add_argument('--duration', type=float, default=30., help='Seconds; 0 runs until B or Ctrl+C')
    args = parser.parse_args()
    if args.execute and args.action != 'quest':
        parser.error('--execute is only valid with quest')
    if not np.isfinite(args.duration) or args.duration < 0:
        parser.error('--duration must be finite and nonnegative')
    config = load_config(args.config)
    with ExitStack() as stack:
        backend = Backend(config, allow_motion=args.execute)
        stack.callback(backend.close)
        if args.action == 'status':
            state = backend.read_state(time.monotonic())
            print(json.dumps({'identity': backend.info, 'joint_rad': state.q.tolist(),
                              'tcp_m_xyzw': state.pose.tolist(), 'fault': state.fault,
                              'gripper': 'unknown; not configured', 'motion_sent': False}, ensure_ascii=False, indent=2))
            return
        if args.action == 'check-kinematics':
            checker = backend.prepare_motion_check()
            print(json.dumps({'model_fit': checker.fit,
                              'current_pose_ik': checker.check(backend.state.pose, backend.state),
                              'motion_sent': False}, indent=2))
            return
        import xrobotoolkit_sdk as xr
        xr.init()
        stack.callback(xr.close)
        stack.callback(backend.stop)  # Stop before closing the XR SDK, which can block.
        tracker = RelativeTarget(config)
        tracker.origin = backend.origin.copy()
        print('EXECUTE: right Grip is the deadman; B stops and exits.' if args.execute else
              'READ-ONLY PREVIEW: hold right Grip to preview relative targets; B exits.', flush=True)
        print('Release Grip first. No gripper commands; no automatic enable/home.', flush=True)
        gate = GripGate()
        clock = XRClock(config["stale_s"])
        last_update = time.monotonic()
        last_print = 0.
        startup_deadline = last_update + 5.
        seen_xr = False
        next_identity_check = last_update + 1.
        next_status = last_update
        samples = 0
        fresh_samples = 0
        previews = 0
        deadline = last_update + args.duration if args.duration else float('inf')
        while time.monotonic() < deadline:
            started = time.monotonic()
            state = backend.read_state(started)
            if started >= next_identity_check:
                for key in ('get_current_tool_frame', 'get_current_work_frame'):
                    reply = backend.client.query(key)
                    if reply != backend.info[key]:
                        raise RuntimeError('Robot coordinate/tool configuration changed during session')
                next_identity_check = started + 1.
            timestamp = int(xr.get_time_stamp_ns())
            fresh = clock.fresh(timestamp, started)
            samples += 1
            fresh_samples += int(fresh)
            grip = float(xr.get_right_grip())
            if not np.isfinite(grip) or not 0 <= grip <= 1:
                raise ValueError('Invalid Grip reading')
            following = gate.update(grip, fresh)
            if started >= next_status:
                print(json.dumps({'xr_fresh': fresh, 'right_grip': grip, 'robot_fault': state.fault,
                                  'armed': gate.armed, 'following': gate.following, 'preview': not args.execute}), flush=True)
                next_status = started + 1.
            if xr.get_B_button():
                break
            if state.fault:
                raise RuntimeError(state.fault)
            if fresh:
                seen_xr = True
            if not fresh:
                backend.stop()
                tracker.reset()
                if args.execute and (seen_xr or started > startup_deadline):
                    raise RuntimeError('XR timestamp stalled, missing or rolled back; restart session')
            elif not following:
                backend.stop()
                tracker.reset()
            else:
                target = tracker.update(xr.get_right_controller_pose(), state.pose)
                previews += 1
                if args.execute:
                    try:
                        backend.step(ControlCommand(Mode.COMMAND_FOLLOWING, pose_target=target, engaged=True), started)
                    except Exception:
                        print(json.dumps({'command_diagnostics': backend.telemetry()['command']}), flush=True)
                        raise
                if started - last_print > .5:
                    print(json.dumps({'preview': not args.execute, 'target_m_xyzw': target.tolist(),
                                      'limits': sorted(set(tracker.limits)),
                                      'command_diagnostics': backend.telemetry()['command'] if args.execute else None}), flush=True)
                    last_print = started
            time.sleep(max(0, 1/config['control_hz'] - (time.monotonic() - started)))
        print(json.dumps({'samples': samples, 'fresh_xr_samples': fresh_samples, 'target_samples': previews,
                          'execute_requested': args.execute}), flush=True)


def main():
    try:
        run()
    except KeyboardInterrupt:
        print('已退出；已尝试停止机械臂，请确认现场状态。', flush=True)
    except (RuntimeError, ValueError, OSError) as exc:
        print(f'会话已停止：{exc}。请确认机械臂已停，再处理原因并重新启动。', file=sys.stderr, flush=True)
        raise SystemExit(1) from None


if __name__ == '__main__':
    main()
