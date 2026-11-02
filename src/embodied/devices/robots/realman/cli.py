"""RealMan status and Quest relative-pose preview/control; no implicit activation."""
import argparse
from contextlib import ExitStack
import json
import time
import sys

import numpy as np

from .backend import Backend, DEFAULT_CONFIG, load_config
from .gripper import status as gripper_status, move as gripper_move, DEFAULT_STEP
from .quest_control import run_quest


def run():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['status', 'quest', 'check-kinematics', 'gripper-status', 'gripper-open', 'gripper-close', 'zero', 'pregrasp', 'quest-dual'])
    parser.add_argument('--config', help='Robot config; in quest-dual this selects the right arm (default .18)')
    parser.add_argument('--left-config', help='Left arm config for quest-dual (default .19)')
    parser.add_argument('--execute', action='store_true', help='Permit motion; default is read-only target preview')
    parser.add_argument('--duration', type=float, default=30., help='Seconds; 0 runs until B or Ctrl+C')
    parser.add_argument('--gripper', action='store_true', help='Hold A (right) / X (left) to open, corresponding Trigger to close; works while following with Grip')
    parser.add_argument('--gripper-speed', type=int, metavar='1..1000',
                        help='Speed for explicit full open/close or startup open only (default 100); position segments inherit controller speed')
    parser.add_argument('--gripper-step', type=int, metavar='5..100',
                        help=f'Raw opening units per segment (default {DEFAULT_STEP}); larger segments continue farther after release')
    parser.add_argument('--gripper-initialize-open', action='store_true',
                        help='Explicit full open at startup (BOTH grippers in quest-dual, speed 100/1000), including all-zero feedback; requires clear empty jaws')
    parser.add_argument('--position-only', action='store_true', help='Translate while preserving the TCP orientation at each Grip engagement')
    parser.add_argument('--xr-startup-timeout', type=float, default=15.,
                        help='Seconds to wait for the first XR frame; live stale limit is unchanged')
    args = parser.parse_args()
    quest_action = args.action in ('quest', 'quest-dual')
    if args.left_config is not None and args.action != 'quest-dual':
        parser.error('--left-config requires quest-dual')
    if args.execute and args.action not in ('quest', 'quest-dual', 'gripper-open', 'gripper-close', 'zero', 'pregrasp'):
        parser.error('--execute requires quest/quest-dual, zero, pregrasp or gripper-open/close')
    if args.action in ('gripper-open', 'gripper-close') and not args.execute:
        parser.error('Gripper movement requires --execute')
    if args.gripper and (not quest_action or not args.execute):
        parser.error('--gripper requires quest/quest-dual --execute')
    if args.gripper_step is not None:
        if not args.gripper:
            parser.error('--gripper-step requires quest/quest-dual --execute --gripper')
        if not 5 <= args.gripper_step <= 100:
            parser.error('--gripper-step must be in 5..100 raw units')
    gripper_step = args.gripper_step if args.gripper_step is not None else DEFAULT_STEP
    if args.position_only and not quest_action:
        parser.error('--position-only requires quest/quest-dual')
    if args.gripper_initialize_open and not (args.execute and (
            args.action == 'gripper-open' or (quest_action and args.gripper))):
        parser.error('--gripper-initialize-open requires quest/quest-dual --execute --gripper or gripper-open --execute')
    if args.gripper_speed is not None:
        if not (args.action in ('gripper-open', 'gripper-close') or args.gripper_initialize_open):
            parser.error('--gripper-speed requires an explicit full open/close or --gripper-initialize-open')
        if not 1 <= args.gripper_speed <= 1000:
            parser.error('--gripper-speed must be in 1..1000')
    gripper_speed = args.gripper_speed if args.gripper_speed is not None else 100
    if not np.isfinite(args.duration) or args.duration < 0:
        parser.error('--duration must be finite and nonnegative')
    if not np.isfinite(args.xr_startup_timeout) or args.xr_startup_timeout <= 0:
        parser.error('--xr-startup-timeout must be finite and positive')
    config_path = args.config or (str(DEFAULT_CONFIG.with_name('rm65-18.json'))
        if args.action == 'quest-dual' else str(DEFAULT_CONFIG))
    config = load_config(config_path)
    left_config = None
    if args.action == 'quest-dual':
        left_config = load_config(args.left_config or str(DEFAULT_CONFIG))
        if left_config['host'] == config['host']:
            parser.error('Dual arms must use different robot hosts')
        if left_config['control_hz'] != config['control_hz']:
            parser.error('Dual arms must use the same control_hz')
    with ExitStack() as stack:
        backend = Backend(config, allow_motion=args.execute and quest_action)
        stack.callback(backend.close)
        if args.action in ('zero', 'pregrasp'):
            from .positioning import JOINT_POSES, joint_preview, execute_joint_target
            target = JOINT_POSES[args.action]
            print(json.dumps({'joint_preview': joint_preview(backend, target), 'execute_requested': args.execute}), flush=True)
            if args.execute:
                print(json.dumps(execute_joint_target(backend, target)), flush=True)
            return
        if args.action in ('gripper-open', 'gripper-close') or args.gripper:
            backend.client.allow_gripper = True
        if args.action in ('gripper-open', 'gripper-close'):
            if backend.state.fault:
                raise RuntimeError(backend.state.fault)
            print(json.dumps(gripper_move(backend.client, args.action.removeprefix('gripper-'),
                speed=gripper_speed, initialize_open=args.gripper_initialize_open)), flush=True)
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
        bindings = [('right', backend)]
        if left_config is not None:
            left_backend = Backend(left_config, allow_motion=args.execute)
            stack.callback(left_backend.close)
            bindings.insert(0, ('left', left_backend))
        import xrobotoolkit_sdk as xr
        run_quest(bindings, xr, execute=args.execute, duration=args.duration,
            startup_timeout=args.xr_startup_timeout, position_only=args.position_only,
            gripper=args.gripper, gripper_step=gripper_step,
            initialize_open=args.gripper_initialize_open, gripper_speed=gripper_speed)


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
