"""Hand-eye calibration of a configured camera through the arx kernel.

  marker                 printable ArUco target (measure the printed black square)
  capture <camera_id>    teach by hand and sample with s / v, or --auto: teach a centre pose, press c,
                         and the kernel visits a sampling plan around it; solves and saves when validated
  solve <session>        re-solve a session offline; --save makes it the current result
  status                 current result of every configured camera

The target is fixed on the table for a wrist camera, and held by the gripper for a fixed camera.
"""
import argparse
from contextlib import ExitStack
from datetime import datetime
import json
from pathlib import Path
import signal

import numpy as np

from embodied import calibration
from embodied.calibration.session import load as load_session
from embodied.calibration.solver import solve
from embodied.calibration.target import MarkerDetector, marker_svg, target_spec
from embodied.core.project import PROJECT_ROOT
from embodied.core.storage import save_json

SESSIONS = PROJECT_ROOT / 'outputs/calibration/hand_eye'
WINDOW = 'hand-eye calibration'


def config():
    from embodied.core.project import load_config
    return load_config()


def marker_options(args, settings):
    size = args.size_m if args.size_m is not None else settings.get('marker_size_m')
    if size is None:
        raise ValueError('Measure the printed black square and pass --size-m, or set [hand_eye] marker_size_m in configs/local.toml')
    return target_spec(float(size), args.marker_id if args.marker_id is not None else int(settings.get('marker_id', 0)))


def summary(result):
    fit, validation = result['fit'], result.get('validation') or {}
    text = (f"{result['status']} ({result['method']}): fit {fit['count']} poses, "
            f"{1000 * fit['translation_rms_m']:.2f} mm / {fit['rotation_rms_deg']:.2f}° RMS")
    if validation.get('count'):
        text += (f"; held-out {validation['count']} poses, {1000 * validation['translation_rms_m']:.2f} mm / "
                 f"{validation['rotation_rms_deg']:.2f}° RMS")
    return text


def solve_session(directory):
    data = load_session(directory)
    result = solve(data['samples'], data['mount'])
    result.update({key: data[key] for key in ('camera_id', 'robot', 'camera', 'target')},
                  source_session=str(Path(directory).resolve()))
    save_json(Path(directory) / 'result.json', result)
    return result


def finish(directory, result, force=False):
    print(summary(result))
    if result['status'] == 'validated' or force:
        path = calibration.save(result['camera_id'], {**result, 'operator_override': result['status'] != 'validated'})
        print(f'Saved as the current {result["camera_id"]} calibration: {path}')
    else:
        print(f'Not saved: held-out validation did not pass. Inspect {directory}/result.json; '
              f'`./run.sh calib handeye solve {directory} --save --force` adopts it anyway.')


def capture(args):
    import cv2
    from embodied.calibration.arm import CalibrationArm
    from embodied.calibration.session import Session
    from embodied.contracts.runtime.control import live_runs
    from embodied.sensors.client import SharedCamera
    settings = config()
    mount = calibration.mount(args.camera_id, settings)
    spec = marker_options(args, settings.get('hand_eye', {}))
    options = settings['cameras'][args.camera_id]
    runs = [args.run] if args.run else live_runs(PROJECT_ROOT / 'control/run')
    if not runs:
        raise ValueError('No running kernel: start ./run.sh kernel sim or ./run.sh kernel real first')
    output = Path(args.output or SESSIONS / args.camera_id / datetime.now().strftime('%Y%m%d-%H%M%S'))
    print(f'{args.camera_id} ({mount}): target {"fixed on the table" if mount == "wrist" else "held by the gripper"}; '
          f'kernel {Path(runs[-1]).name}.', flush=True)
    session = None
    def interrupted(signum, frame):
        raise KeyboardInterrupt(f'Signal {signum}')
    previous = signal.signal(signal.SIGTERM, interrupted)
    try:
        with ExitStack() as stack:
            camera = stack.enter_context(SharedCamera(options['serial'], options['width'], options['height'],
                                                      options['fps'], camera_id=args.camera_id))
            detector = MarkerDetector(spec, camera.metadata)
            arm = stack.enter_context(CalibrationArm(runs[-1]))
            session = Session(output, args.camera_id, mount, camera.metadata, spec, arm.robot)
            stack.callback(cv2.destroyAllWindows)

            def sample(split, index=None):
                session.capture(camera, arm, detector, split, index)

            arm.teach()
            keys = 'c = centre and start' if args.auto else 's = sample, v = held-out sample'
            print(f'Arm is free: move it by hand so the camera sees the target. {keys}, q = finish.', flush=True)
            while True:
                image = camera.read()['image']
                try:
                    detection = detector.detect(image)
                    text = f"target {detection['reprojection_error_undistorted_px']:.2f}px | {keys} | q"
                except ValueError as exc:
                    detection, text = None, str(exc)
                count = len(session.data['samples'])
                cv2.putText(image, f'{count} samples | {text}', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 0), 2)
                cv2.imshow(WINDOW, image)
                key = cv2.waitKey(1) & 0xff
                if key in (ord('q'), 27) or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                    break
                if args.auto and key == ord('c'):
                    if detection is None:
                        print('The target must be visible at the centre pose.', flush=True)
                        continue
                    run_auto(args, arm, session, sample)
                    break
                if not args.auto and key in (ord('s'), ord('v')):
                    arm.hold()
                    try:
                        sample('fit' if key == ord('s') else 'validation')
                    except ValueError as exc:
                        print(f'Sample rejected: {exc}', flush=True)
                    arm.teach()
            arm.hold()
        session.finish('complete')
    except BaseException as exc:
        if session is not None:
            session.finish('interrupted' if isinstance(exc, KeyboardInterrupt) else 'failed', exc)
        raise
    finally:
        signal.signal(signal.SIGTERM, previous)
    print(f'Session: {output}', flush=True)
    finish(output, solve_session(output))


def run_auto(args, arm, session, sample):
    from embodied.calibration.plan import kernel_model, make_plan, run_plan, validate_plan
    arm.hold()
    model = kernel_model(Path(arm.session.parameters['model']['xml_path']).with_suffix('.urdf'))
    center = arm.snapshot()['q']
    if args.plan:
        plan = json.loads(Path(args.plan).read_text())
        validate_plan(plan, model)
        if np.max(np.abs(np.asarray(center) - plan['center'])) > .03:
            raise ValueError('Teach the plan centre pose first (within 0.03 rad)')
    else:
        plan = make_plan(model, center, args.step_rad)
    session.data['plan'] = plan
    session.save()
    print(f'Sampling {len(plan["waypoints"])} poses within ±{plan["step_rad"]} rad at {args.speed} rad/s; '
          'every move is previewed by the kernel. Ctrl+C or the e-stop stops; the arm then holds.', flush=True)
    taken = run_plan(arm, plan, sample, speed=args.speed,
                     log=lambda text: (session.skip(None, None, text), print(text, flush=True)))
    print(f'{taken} samples captured; back at the centre pose.', flush=True)


def status():
    settings = config()
    for camera_id, options in settings['cameras'].items():
        mount = calibration.mount(camera_id, settings)
        path = calibration.RESULTS / f'{camera_id}.json'
        result = calibration.load(camera_id, options.get('serial') or None)
        if result is not None:
            state = summary(result) + (' (adopted without validation)' if result.get('operator_override') else '')
        elif path.exists():
            state = 'not usable: not validated or measured with another camera serial'
        else:
            state = 'not calibrated'
        print(f'{camera_id:14} {mount:5}  {state}')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    commands = parser.add_subparsers(dest='command', required=True)
    marker = commands.add_parser('marker', help='Printable ArUco target (SVG)')
    shoot = commands.add_parser('capture', help='Capture a session through the running kernel, then solve')
    shoot.add_argument('camera_id', help='Configured camera id, e.g. wrist_rgb or exterior_rgb')
    shoot.add_argument('--auto', action='store_true', help='Kernel-driven sampling plan around a taught centre pose')
    shoot.add_argument('--run', type=Path, help='control/run/<RUN> (default: newest running kernel)')
    shoot.add_argument('--step-rad', type=float, default=.25, help='--auto: joint offsets around the centre')
    shoot.add_argument('--speed', type=float, default=.3, help='--auto: joint speed, rad/s (kernel bound 0.5)')
    shoot.add_argument('--plan', help='--auto: reuse a saved plan (its centre must be taught first)')
    shoot.add_argument('--output', help='New session directory')
    for sub in (marker, shoot):
        sub.add_argument('--size-m', type=float, help='Measured black square, metres (default [hand_eye] marker_size_m)')
        sub.add_argument('--marker-id', type=int, help='DICT_4X4_50 id (default [hand_eye] marker_id or 0)')
    marker.add_argument('--output', default='outputs/calibration/marker.svg')
    solver = commands.add_parser('solve', help='Solve a captured session offline')
    solver.add_argument('session')
    solver.add_argument('--save', action='store_true', help='Make it the current result when validated')
    solver.add_argument('--force', action='store_true', help='With --save: adopt a result that failed validation')
    commands.add_parser('status', help='Current result of every configured camera')
    args = parser.parse_args(argv)
    if args.command == 'marker':
        path = Path(args.output)
        if path.exists():
            raise ValueError('Marker output already exists')
        path.parent.mkdir(parents=True, exist_ok=True)
        spec = marker_options(args, config().get('hand_eye', {}))
        path.write_text(marker_svg(spec))
        save_json(path.with_suffix('.json'), spec)
        print(f'{path}: print at 100%, then measure the BLACK square (nominal {1000 * spec["black_square_size_m"]:g} mm).')
    elif args.command == 'capture':
        if not .05 <= args.speed <= .5:
            raise ValueError('--speed must be within 0.05..0.5 rad/s')
        capture(args)
    elif args.command == 'solve':
        result = solve_session(args.session)
        if args.save:
            finish(args.session, result, args.force)
        else:
            print(summary(result))
    else:
        status()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        raise SystemExit(130)
    except (ValueError, OSError, RuntimeError) as exc:
        raise SystemExit(str(exc))
