"""Preview, move, jog and teach through a single explicitly acquired control session."""
import argparse
import json
from pathlib import Path
import time

from embodied.contracts.runtime.control import ControlClient, ControlSession


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    commands = parser.add_subparsers(dest='command', required=True)
    commands.add_parser('status')
    commands.add_parser('stop')
    commands.add_parser('home')
    free_drive = commands.add_parser('free-drive')
    free_drive.add_argument('--duration', type=float, default=0, help='0 until Ctrl+C; release returns to hold')
    for name in ('preview', 'movej', 'movel'):
        child = commands.add_parser(name)
        child.add_argument('--q', type=float, nargs=6)
        child.add_argument('--pose', type=float, nargs=7, help='base TCP: xyz metres and xyzw quaternion')
        child.add_argument('--speed', type=float, default=.15)
        if name == 'preview':
            child.add_argument('--output', type=Path)
    execute = commands.add_parser('execute')
    execute.add_argument('--plan', type=Path, required=True)
    jog = commands.add_parser('jog')
    jog.add_argument('--joint', type=int, choices=range(1, 7), required=True)
    jog.add_argument('--step', type=float, required=True, help='radians, absolute value <= .1')
    gripper = commands.add_parser('gripper')
    gripper.add_argument('--width', type=float, required=True)
    teach = commands.add_parser('teach')
    teach.add_argument('--output', type=Path, required=True)
    restore = commands.add_parser('return')
    restore.add_argument('--point', type=Path, required=True)
    args = parser.parse_args(argv)
    client = ControlClient(args.run)
    if args.command == 'status':
        print(json.dumps(client.status(), indent=2)); return 0
    if args.command == 'stop':
        print(json.dumps(client.request('stop'))); return 0
    if args.command == 'teach':
        state = client.status()
        with args.output.open('x') as output:
            json.dump({k: state[k] for k in ('model_id', 'q', 'pose', 'gripper_width', 'feedback_monotonic_ns')}, output, indent=2)
        return 0
    if args.command == 'preview':
        if (args.q is None) == (args.pose is None):
            parser.error('Provide exactly one of --q and --pose')
        plan = client.preview('movej' if args.q else 'movel', q=args.q, pose=args.pose, speed=args.speed)
        if args.output:
            with args.output.open('x') as output:
                json.dump(plan, output, indent=2)
        print(json.dumps({'plan_id': plan['plan_id'], 'duration_s': plan['times'][-1], 'collision_check': 'none'}))
        return 0
    with ControlSession(args.run) as session:
        try:
            if args.command == 'free-drive':
                if not 0 <= args.duration < float('inf'):
                    parser.error('duration must be finite and nonnegative')
                deadline = time.monotonic()+args.duration if args.duration else float('inf')
                while time.monotonic() < deadline:
                    session.command('free-drive', ttl_ms=250)
                    time.sleep(.05)
                session.command('hold')
                return 0
            if args.command == 'gripper':
                deadline = time.monotonic()+2
                while time.monotonic() < deadline:
                    session.command('gripper', gripper_width=args.width)
                    time.sleep(.04)
                if abs(session.status()['gripper_width'] - args.width) > .003:
                    raise RuntimeError('Gripper did not reach the requested width')
                return 0
            if args.command == 'execute':
                plan = json.loads(args.plan.read_text())
            elif args.command == 'home':
                plan = session.preview('home')
            elif args.command == 'jog':
                if not 0 < abs(args.step) <= .1:
                    parser.error('Jog step must be nonzero and <= .1 rad')
                q = session.status()['q']; q[args.joint-1] += args.step
                plan = session.preview('jog', q=q)
            elif args.command == 'return':
                point = json.loads(args.point.read_text())
                if point['model_id'] != session.status()['model_id']:
                    raise ValueError('Teach point model changed')
                plan = session.preview('movej', q=point['q'], gripper_width=point['gripper_width'])
            else:
                if args.command == 'movej' and args.q is None or args.command == 'movel' and args.pose is None:
                    parser.error('movej requires --q; movel requires --pose')
                plan = session.preview(args.command, q=args.q, pose=args.pose, speed=args.speed)
            result = session.execute(plan)
            print(json.dumps(result), flush=True)
            print(json.dumps(session.wait(result['operation_id'], timeout=plan['times'][-1]+8)))
        except KeyboardInterrupt:
            session.command('cancel')
            return 130
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
