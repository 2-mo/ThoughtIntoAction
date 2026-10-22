"""Command-line access to Robot operations and kernel administration."""
import argparse
import json
import math
from pathlib import Path
import time

from embodied.contracts.runtime.control import ControlClient, ControlSession
from .robot import Robot


def add_motion_options(parser):
    parser.add_argument('--speed', type=float, help='joint speed cap, rad/s; 0 uses backend per-joint limits (omitted: runtime motion.json)')
    parser.add_argument('--acceleration', type=float, help='planned joint acceleration, rad/s² (omitted: runtime motion.json)')
    parser.add_argument('--jerk', type=float, help='planned joint jerk, rad/s³ (omitted: runtime motion.json)')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    commands = parser.add_subparsers(dest='command', required=True)
    commands.add_parser('status')
    commands.add_parser('stop')
    add_motion_options(commands.add_parser('home'))
    free_drive = commands.add_parser('free-drive')
    free_drive.add_argument('--duration', type=float, default=0, help='0 until Ctrl+C; release returns to hold')
    for name in ('preview', 'movej', 'movel'):
        child = commands.add_parser(name)
        target = child.add_mutually_exclusive_group(required=True)
        if name != 'movel':
            target.add_argument('--q', type=float, nargs='+', help='joint positions in radians; one value per joint')
        if name != 'movej':
            target.add_argument('--pose', type=float, nargs='+', help='base TCP: XYZ + rotation vector (6) or XYZ + XYZW quaternion (7)')
            target.add_argument('--xyz', type=float, nargs=3, help='base TCP position in metres; preserve orientation')
            target.add_argument('--offset', type=float, nargs=3, help='relative TCP displacement in metres')
            child.add_argument('--frame', choices=('base', 'tool'), default='base', help='offset frame; absolute targets use base')
        add_motion_options(child)
        if name == 'preview':
            child.add_argument('--output', type=Path)
    execute = commands.add_parser('execute')
    execute.add_argument('--plan', type=Path, required=True)
    jog = commands.add_parser('jog')
    jog.add_argument('--joint', type=int, required=True, help='one-based joint number')
    jog.add_argument('--step', type=float, required=True, help='radians, absolute value <= .1')
    add_motion_options(jog)
    gripper = commands.add_parser('gripper')
    gripper.add_argument('--width', type=float, required=True)
    teach = commands.add_parser('teach')
    teach.add_argument('--output', type=Path, required=True)
    restore = commands.add_parser('return')
    restore.add_argument('--point', type=Path, required=True)
    add_motion_options(restore)
    args = parser.parse_args(argv)
    if getattr(args, 'pose', None) is not None and len(args.pose) not in (6, 7):
        parser.error('--pose requires 6 values (XYZ + rotation vector) or 7 (XYZ + XYZW)')
    if getattr(args, 'frame', 'base') == 'tool' and getattr(args, 'offset', None) is None:
        parser.error('--frame tool requires --offset; absolute targets use the base frame')
    if args.command == 'jog' and (args.joint < 1 or not 0 < abs(args.step) <= .1):
        parser.error('Jog requires a positive joint number and nonzero step <= .1 rad')
    if args.command == 'stop':
        print(json.dumps(ControlClient(args.run).request('stop')))
        return 0
    if args.command == 'free-drive':
        if not 0 <= args.duration < float('inf'):
            parser.error('duration must be finite and nonnegative')
        with ControlSession(args.run) as session:
            try:
                deadline = time.monotonic()+args.duration if args.duration else float('inf')
                while time.monotonic() < deadline:
                    session.command('free-drive', ttl_ms=250)
                    time.sleep(.05)
                session.command('hold')
            except KeyboardInterrupt:
                session.command('cancel')
                return 130
        return 0
    motion_options = {key: getattr(args, key) for key in ('speed', 'acceleration', 'jerk')
                      if getattr(args, key, None) is not None}
    needs_velocity_capability = ('speed' in motion_options
                                and (motion_options['speed'] == 0 or motion_options['speed'] > .5))
    with Robot.connect(args.run) as robot:
        if any(key in motion_options for key in ('acceleration', 'jerk')) or needs_velocity_capability:
            limits = robot.describe().get('motion_limits')
            if not isinstance(limits, dict):
                parser.error('当前内核不支持可调 acceleration/jerk；需加载新版内核后使用。')
            if needs_velocity_capability:
                velocity = limits.get('joint_velocity_limits_rad_s')
                if (not isinstance(velocity, list) or len(velocity) != len(robot.q)
                        or not all(type(v) in (int, float) and math.isfinite(v) and v > 0 for v in velocity)):
                    parser.error('当前内核未提供逐轴速度能力；需加载新版内核后使用此速度。')
        try:
            if args.command == 'status':
                print(json.dumps(robot.state, indent=2))
                return 0
            if args.command == 'teach':
                with args.output.open('x') as output:
                    json.dump(robot.teach(), output, indent=2)
                return 0
            if args.command == 'gripper':
                robot.gripper.move(args.width)
                return 0
            if args.command == 'execute':
                motion = robot.execute(json.loads(args.plan.read_text()), wait=False)
            elif args.command == 'home':
                motion = robot.home(wait=False, **motion_options)
            elif args.command == 'jog':
                motion = robot.jog(args.joint-1, args.step, wait=False, **motion_options)
            elif args.command == 'return':
                motion = robot.return_to(json.loads(args.point.read_text()), wait=False, **motion_options)
            else:
                if getattr(args, 'q', None) is not None:
                    plan = robot.plan_movej(args.q, **motion_options)
                else:
                    plan = robot.plan_movel(pose=args.pose, xyz=args.xyz, offset=args.offset,
                                            frame=args.frame, **motion_options)
                if args.command == 'preview':
                    if args.output:
                        with args.output.open('x') as output:
                            json.dump(plan, output, indent=2)
                    print(json.dumps({'plan_id': plan['plan_id'], 'duration_s': plan['times'][-1],
                                      'collision_check': plan.get('collision_check', 'none'),
                                      **{key: plan[key] for key in ('limits', 'metrics') if key in plan}}))
                    return 0
                motion = robot.execute(plan, wait=False)
            print(json.dumps({'operation_id': motion.operation_id, 'status': 'running'}), flush=True)
            print(json.dumps(motion.wait()))
        except KeyboardInterrupt:
            return 130
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
