"""RealMan status and Quest relative-pose preview/control; no implicit activation."""
import argparse
from contextlib import ExitStack
import json
import time
import sys

import numpy as np

from embodied.contracts.control import ControlCommand, Mode
from .backend import Backend, DEFAULT_CONFIG, load_config
from .gripper import status as gripper_status, move as gripper_move, Buttons, Jog
from .teleop import RelativeTarget, GripGate, XRClock


def run():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['status', 'quest', 'check-kinematics', 'gripper-status', 'gripper-open', 'gripper-close'])
    parser.add_argument('--config', default=str(DEFAULT_CONFIG))
    parser.add_argument('--execute', action='store_true', help='Permit motion; default is read-only target preview')
    parser.add_argument('--duration', type=float, default=30., help='Seconds; 0 runs until B or Ctrl+C')
    parser.add_argument('--gripper', action='store_true', help='Hold A to open or Trigger to close in small position segments; release Grip first')
    parser.add_argument('--xr-startup-timeout', type=float, default=15.,
                        help='Seconds to wait for the first XR frame; live stale limit is unchanged')
    args = parser.parse_args()
    if args.execute and args.action not in ('quest', 'gripper-open', 'gripper-close'):
        parser.error('--execute requires quest or gripper-open/close')
    if args.action in ('gripper-open', 'gripper-close') and not args.execute:
        parser.error('Gripper movement requires --execute')
    if args.gripper and (args.action != 'quest' or not args.execute):
        parser.error('--gripper requires quest --execute')
    if not np.isfinite(args.duration) or args.duration < 0:
        parser.error('--duration must be finite and nonnegative')
    if not np.isfinite(args.xr_startup_timeout) or args.xr_startup_timeout <= 0:
        parser.error('--xr-startup-timeout must be finite and positive')
    config = load_config(args.config)
    with ExitStack() as stack:
        backend = Backend(config, allow_motion=args.execute and args.action == 'quest')
        stack.callback(backend.close)
        if args.action in ('gripper-open', 'gripper-close') or args.gripper:
            backend.client.allow_gripper = True
        if args.action in ('gripper-open', 'gripper-close'):
            if backend.state.fault:
                raise RuntimeError(backend.state.fault)
            print(json.dumps(gripper_move(backend.client, args.action.removeprefix('gripper-'))), flush=True)
            return
        if args.action == 'gripper-status':
            raw = gripper_status(backend.client)
            print(json.dumps({'model': 'EG2-4C2', 'feedback': raw,
                              'online': raw['status'] == 1, 'enabled': raw['enable'] == 1,
                              'opening_unit': 'controller raw value; not calibrated to mm',
                              'motion_sent': False, 'configuration_changed': False}, ensure_ascii=False, indent=2))
            return
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
        print('Release Grip first. No automatic enable/home.', flush=True)
        if args.gripper:
            print('Gripper: release Grip; HOLD A=open, HOLD Trigger=close in 10/1000 position segments. Release ends scheduling; current segment may finish. Speed/force inherited from controller.', flush=True)
        else:
            print('Gripper control disabled.', flush=True)
        buttons = Buttons()
        jog = Jog()
        last_gripper_report = None
        last_gripper_input = None
        gripper_actions = 0
        gate = GripGate()
        clock = XRClock(config["stale_s"])
        last_update = time.monotonic()
        last_print = 0.
        startup_deadline = last_update + args.xr_startup_timeout
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
                                  'armed': gate.armed, 'following': gate.following, 'preview': not args.execute,
                                  'xr_timestamp_ns': timestamp,
                                  'waiting_for_first_xr': not seen_xr and not fresh}), flush=True)
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
                if args.execute and seen_xr:
                    raise RuntimeError('XR data stalled or timestamp rolled back after connection; restart session')
                if args.execute and started > startup_deadline:
                    raise RuntimeError(f'No first XR frame within {args.xr_startup_timeout:g} s; check Toolkit connection')
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
            if args.gripper:
                a = bool(xr.get_A_button())
                trigger = float(xr.get_right_trigger())
                action = buttons.update(a, trigger, fresh=fresh, grip=grip, now=started)
                input_state = (a, round(trigger, 1), fresh, grip < .2, buttons.ready)
                if input_state != last_gripper_input:
                    print(json.dumps({'gripper_input': {
                        'right_A': a, 'right_trigger': trigger, 'xr_fresh': fresh,
                        'grip_released': grip < .2, 'buttons_armed': buttons.ready,
                        'requested_action': action}}), flush=True)
                    last_gripper_input = input_state
                # No gripper TCP transactions while the arm is following.
                # Release/Grip/B never queues another position segment.
                if not following:
                    report = jog.update(backend.client, action, time.monotonic())
                    if report is not None:
                        gripper_actions += int(report.get('accepted', False))
                        report_key = (report.get('reason'), report.get('position_target_raw'))
                        if report_key != last_gripper_report:
                            print(json.dumps({'gripper': report}), flush=True)
                            last_gripper_report = report_key
            time.sleep(max(0, 1/config['control_hz'] - (time.monotonic() - started)))
        print(json.dumps({'samples': samples, 'fresh_xr_samples': fresh_samples, 'target_samples': previews,
                          'execute_requested': args.execute, 'gripper_actions': gripper_actions}), flush=True)


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
