"""RealMan status and Quest relative-pose preview/control; no implicit activation."""
import argparse
from contextlib import ExitStack
import json
import time

import numpy as np

from embodied.contracts.control import ControlCommand, Mode
from .backend import Backend, DEFAULT_CONFIG, load_config, vector
from .geometry import matrix, quaternion, limited_rotation


class RelativeTarget:
    def __init__(self, config):
        self.config = config
        self.basis = np.asarray(config['xr_to_robot_basis'])
        self.reference = None
        self.robot_reference = None

    def reset(self):
        self.reference = None

    def update(self, controller, robot_pose):
        controller = vector(controller, 7)
        if not np.isclose(np.linalg.norm(controller[3:]), 1, atol=.01):
            raise ValueError('Invalid tracked controller quaternion')
        if self.reference is None:
            self.reference = controller.copy()
            self.robot_reference = robot_pose.copy()
        target = self.robot_reference.copy()
        delta = self.basis @ (controller[:3] - self.reference[:3]) * self.config['position_scale']
        delta *= min(1., self.config['translation_radius_m'] / max(np.linalg.norm(delta), 1e-12))
        target[:3] += delta
        rotation = self.basis @ matrix(controller[3:]) @ matrix(self.reference[3:]).T @ self.basis.T
        desired = quaternion(rotation @ matrix(self.robot_reference[3:]))
        target[3:] = limited_rotation(desired, self.robot_reference[3:], self.config['rotation_radius_rad'])
        return target


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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['status', 'quest'])
    parser.add_argument('--config', default=str(DEFAULT_CONFIG))
    parser.add_argument('--execute', action='store_true', help='Permit motion; default is read-only target preview')
    parser.add_argument('--duration', type=float, default=30.)
    args = parser.parse_args()
    if args.execute and args.action != 'quest':
        parser.error('--execute is only valid with quest')
    if not np.isfinite(args.duration) or args.duration <= 0:
        parser.error('--duration must be finite and positive')
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
        import xrobotoolkit_sdk as xr
        xr.init()
        stack.callback(xr.close)
        tracker = RelativeTarget(config)
        print('EXECUTE: right Grip is the deadman; B stops and exits.' if args.execute else
              'READ-ONLY PREVIEW: hold right Grip to preview relative targets; B exits.', flush=True)
        print('Release Grip first. No gripper commands; no automatic enable/home.', flush=True)
        released = False
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
        deadline = last_update + args.duration
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
            if started >= next_status:
                print(json.dumps({'xr_fresh': fresh, 'right_grip': grip, 'robot_fault': state.fault,
                                  'armed': released, 'preview': not args.execute}), flush=True)
                next_status = started + 1.
            if xr.get_B_button():
                break
            if state.fault:
                raise RuntimeError(state.fault)
            if not fresh:
                backend.stop()
                tracker.reset()
                released = False
                if args.execute and (seen_xr or started > startup_deadline):
                    raise RuntimeError('XR timestamp stalled, missing or rolled back; restart session')
            elif grip < .2:
                seen_xr = True
                backend.stop()
                tracker.reset()
                released = True
            elif grip > .8 and released:
                target = tracker.update(xr.get_right_controller_pose(), state.pose)
                previews += 1
                if args.execute:
                    backend.step(ControlCommand(Mode.COMMAND_FOLLOWING, pose_target=target, engaged=True), started)
                if started - last_print > .5:
                    print(json.dumps({'preview': not args.execute, 'target_m_xyzw': target.tolist()}), flush=True)
                    last_print = started
            else:
                backend.stop()
                tracker.reset()
            time.sleep(max(0, 1/config['control_hz'] - (time.monotonic() - started)))
        print(json.dumps({'samples': samples, 'fresh_xr_samples': fresh_samples, 'target_samples': previews,
                          'execute_requested': args.execute}), flush=True)


if __name__ == '__main__':
    main()
