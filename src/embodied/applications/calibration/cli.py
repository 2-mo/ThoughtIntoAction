"""Hand-eye calibration of a configured camera through the control kernel.

  marker                 printable ArUco target (measure the printed black square)
  capture <camera_id>    teach by hand and sample with s / v, or --auto: teach a centre pose, press c,
                         and the kernel visits a sampling plan around it; solves and saves when validated
  solve <session>        re-solve a session offline; --save makes it the current result
  plan <session>         preset RM65 points using an earlier wrist session as approximate view geometry
  preview <plan>         replay preset points in an existing simulation, without a camera
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

from embodied.applications import calibration
from embodied.applications.calibration.session import load as load_session, pose_matrix
from embodied.applications.calibration.solver import diversity, solve
from embodied.applications.calibration.target import MarkerDetector, marker_svg, target_spec
from embodied.core.project import PROJECT_ROOT
from embodied.core.storage import save_json
from embodied.core.transforms import mean_pose, transform

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
    if result['status'] != 'validated':
        reasons = []
        if not result['leave_one_out']['passed']:
            reasons.append('leave-one-out mounting stability failed')
        if not validation.get('count'):
            reasons.append('no held-out poses (need at least 4)')
        else:
            if validation.get('failure'):
                reasons.append(validation['failure'])
            limits = result['quality_limits']
            if (validation['translation_max_m'] > limits['translation_m']
                    or validation['rotation_max_deg'] > limits['rotation_deg']):
                reasons.append(f"held-out maximum {1000 * validation['translation_max_m']:.2f} mm / "
                               f"{validation['rotation_max_deg']:.2f}° exceeds per-pose limits "
                               f"{1000 * limits['translation_m']:g} mm / {limits['rotation_deg']:g}°")
        text += '\nNot validated: ' + '; '.join(reasons)
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
        print(f'Not adopted as the current calibration. Raw samples and candidate result are saved in {directory}. '
              'Improve the target view and collect a new session; inspect result.json for details.')


def capture(args):
    import cv2
    from embodied.applications.calibration.arm import CalibrationArm
    from embodied.applications.calibration.session import Session
    from embodied.contracts.runtime.control import live_runs
    from embodied.devices.sensors.client import SharedCamera
    settings = config()
    mount = calibration.mount(args.camera_id, settings)
    spec = marker_options(args, settings.get('hand_eye', {}))
    options = settings['cameras'][args.camera_id]
    runs = [args.run] if args.run else live_runs(PROJECT_ROOT / 'outputs/runtime/control')
    if not runs:
        raise ValueError('No running kernel: start ./run.sh kernel sim or ./run.sh kernel real first')
    output = Path(args.output or SESSIONS / args.camera_id / datetime.now().strftime('%Y%m%d-%H%M%S'))
    print(f'{args.camera_id} ({mount}): target {"fixed on the table" if mount == "wrist" else "held by the gripper"}; '
          f'black square {spec["black_square_size_m"]:g} m; '
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
            stack.callback(cv2.destroyAllWindows)
            keys = 'c = centre and start' if args.auto else 's = sample, v = held-out sample'
            auto_running = False

            def display(image=None):
                image = camera.read()['image'] if image is None else image.copy()
                try:
                    detection = detector.detect(image)
                    text = f"target {detection['reprojection_error_undistorted_px']:.2f}px"
                except ValueError as exc:
                    detection, text = None, str(exc)
                controls = 'AUTO | q/Esc = stop' if auto_running else f'{keys} | q'
                count = len(session.data['samples'])
                cv2.putText(image, f'{count} samples | {text}', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 0), 2)
                cv2.putText(image, controls, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 0), 2)
                cv2.imshow(WINDOW, image)
                key = cv2.waitKey(1) & 0xff
                closed = cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1
                return detection, key, closed

            def refresh(image=None):
                _, key, closed = display(image)
                if key in (ord('q'), 27) or closed:
                    raise KeyboardInterrupt('Capture stopped from the preview window')

            arm = stack.enter_context(CalibrationArm(runs[-1], update=refresh))
            session = Session(output, args.camera_id, mount, camera.metadata, spec, arm.robot)

            def sample(split, index=None):
                session.capture(camera, arm, detector, split, index, update=refresh)
                refresh()

            arm.teach()
            print(f'Prepare a pose that sees the target (hand teaching depends on the backend). '
                  f'{keys}, q = finish.', flush=True)
            while True:
                detection, key, closed = display()
                if key in (ord('q'), 27) or closed:
                    break
                if args.auto and key == ord('c'):
                    if detection is None:
                        print('The target must be visible at the centre pose.', flush=True)
                        continue
                    auto_running = True
                    session.data['center_observation'] = {
                        'T_camera_target': detection['T_camera_target'].tolist()}
                    if not run_auto(args, arm, session, sample):
                        auto_running = False
                        arm.teach()
                        continue
                    break
                if not args.auto and key in (ord('s'), ord('v')):
                    arm.hold()
                    try:
                        sample('fit' if key == ord('s') else 'validation')
                    except ValueError as exc:
                        print(f'Sample rejected: {exc}', flush=True)
                    arm.teach()
            # Context exit holds without refreshing a window the operator may have closed.
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
    from embodied.applications.calibration.plan import kernel_model, make_plan, make_realman_plan, run_plan, validate_plan
    arm.hold()
    model = kernel_model(Path(arm.session.parameters['model']['xml_path']).with_suffix('.urdf'))
    center = arm.snapshot()['q']
    realman = arm.session.parameters['effective_robot_config']['model_plugin'].startswith('realman')
    try:
        if args.plan:
            plan = json.loads(Path(args.plan).read_text())
            validate_plan(plan, model)
            if not realman and np.max(np.abs(np.asarray(center) - plan['center'])) > .03:
                raise ValueError('Teach the plan centre pose first (within 0.03 rad)')
            view = plan.get('view')
            if view:
                if (view['camera_id'] != session.data['camera_id']
                        or view['camera']['serial'] != session.data['camera']['serial']
                        or view['target'] != session.data['target'] or session.data['mount'] != 'wrist'):
                    raise ValueError('Plan view belongs to another camera, target or mounting')
                # Re-anchor to the live board; a saved candidate is only a planning estimate.
                view['T_base_target'] = (model.fk(center) @ transform(view['T_eef_camera'])
                                         @ transform(session.data['center_observation']['T_camera_target'])).tolist()
                view['camera'] = {key: session.data['camera'][key] for key in view['camera']}
            if realman:
                # The saved view guides new points around the operator's current pose.
                plan = make_realman_plan(model, center, args.step_rad, view=view)
            elif view:
                plan['center'] = list(center)
                validate_plan(plan, model)
        else:
            plan = (make_realman_plan if realman else make_plan)(model, center, args.step_rad)
    except ValueError as exc:
        print(f'Cannot start sampling: {exc}. Adjust the centre view/range and press c again, or q to finish.', flush=True)
        return False
    session.data['plan'] = plan
    session.save()
    print(f'Sampling {len(plan["waypoints"])} poses within ±{plan["step_rad"]} rad at {args.speed} rad/s; '
          'every move is previewed by the kernel. Ctrl+C or the e-stop stops; the arm then holds.', flush=True)
    taken = run_plan(arm, plan, sample, speed=args.speed,
                     log=lambda text: (session.skip(None, None, text), print(text, flush=True)))
    print(f'{taken} samples captured; back at the centre pose.', flush=True)
    return True


def prepare_plan(args):
    from embodied.applications.calibration.plan import kernel_model, make_realman_plan, validate_plan
    from embodied.control.settings import load_settings
    robot = load_settings(args.robot_config).robot
    if not robot.model_plugin.startswith('realman'):
        raise ValueError('This preset uses an RM65 robot configuration')
    model = kernel_model(Path(robot.model_path).with_suffix('.urdf'))
    source = load_session(args.session)
    result = json.loads((Path(args.session) / 'result.json').read_text())
    if source['mount'] != 'wrist' or source.get('plan', {}).get('robot') != model.metadata():
        raise ValueError('Reference must be a wrist session captured with this robot URDF/TCP')
    x = transform(result['T_eef_camera'])
    # Use only fitting inliers to estimate board location in the nominal simulation model.
    fit = [row for row in source['samples'] if row['split'] == 'fit' and row['id'] in result['inlier_ids']]
    board = mean_pose([model.fk(row['q']) @ x @ transform(row['T_camera_target']) for row in fit])
    view = {'camera_id': source['camera_id'], 'target': source['target'],
            'camera': {key: source['camera'][key] for key in
                       ('serial', 'width', 'height', 'matrix', 'distortion_model', 'coefficients')},
            'T_eef_camera': x.tolist(), 'T_base_target': board.tolist(),
            'source_session': str(Path(args.session).resolve()), 'source_status': result['status'],
            'purpose': 'approximate view planning only; not an adopted calibration'}
    plan = make_realman_plan(model, source['plan']['center'], args.step_rad, view=view)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as output:
        json.dump(plan, output, indent=2)
        output.write('\n')
    print(json.dumps({'plan': str(args.output), **validate_plan(plan, model)}, indent=2))


def preview_plan(args):
    from embodied.applications.calibration.arm import CalibrationArm
    from embodied.applications.calibration.plan import kernel_model, run_plan, target_visible, validate_plan
    from embodied.contracts.runtime.control import ControlClient
    client = ControlClient(args.run)
    if client.identity['backend'] != 'sim':
        raise ValueError('Plan preview requires a simulation runtime; hardware is never executed here')
    plan = json.loads(args.plan.read_text())
    model = kernel_model(Path(client.parameters['model']['xml_path']).with_suffix('.urdf'))
    summary = validate_plan(plan, model)
    report = {'simulation_only': True, 'runtime': str(args.run), 'plan': str(args.plan),
              'summary': summary, 'samples': [], 'skipped': [], 'status': 'running'}
    try:
        with CalibrationArm(args.run) as arm:
            arm.move(plan['center'], args.speed)
            arm.hold()
            def sample(split, index):
                measured = arm.snapshot()
                report['samples'].append({'index': index, 'split': split, **measured})
                if np.max(np.abs(np.asarray(measured['q']) - plan['waypoints'][index]['q'])) > .01:
                    raise RuntimeError(f'Simulation did not settle at waypoint {index}')
                if plan.get('view') and not target_visible(pose_matrix(measured['pose']), plan['view']):
                    raise RuntimeError(f'Simulated waypoint {index} fails the predicted target view')
                print(f'Simulation {index + 1}/{summary["waypoints"]}: {split}', flush=True)
            taken = run_plan(arm, plan, sample, speed=args.speed, log=report['skipped'].append)
            report['final_q'] = arm.snapshot()['q']
            if taken != summary['waypoints']:
                raise ValueError(f'Only {taken}/{summary["waypoints"]} simulation poses reached; inspect preview report')
            report['measured_diversity'] = {
                split: diversity([pose_matrix(row['pose']) for row in report['samples'] if row['split'] == split],
                                 minimum=10 if split == 'fit' else 4) for split in ('fit', 'validation')}
        report['status'] = 'complete'
    except BaseException as exc:
        report.update(status='failed', error=str(exc))
        raise
    finally:
        save_json(args.plan.with_suffix('.preview.json'), report)
    print(f'Simulation replay complete; report: {args.plan.with_suffix(".preview.json")}')


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
    shoot.add_argument('--run', type=Path, help='outputs/runtime/control/<RUN> (default: newest running kernel)')
    shoot.add_argument('--step-rad', type=float, default=.25, help='--auto: joint offsets around the centre')
    shoot.add_argument('--speed', type=float, default=.3, help='--auto: joint speed, rad/s (kernel bound 0.5)')
    shoot.add_argument('--plan', help='--auto: RM65 regenerates points at the current centre using the saved view; ARX replays the saved plan')
    shoot.add_argument('--output', help='New session directory')
    for sub in (marker, shoot):
        sub.add_argument('--size-m', type=float, help='Measured black square, metres (default [hand_eye] marker_size_m)')
        sub.add_argument('--marker-id', type=int, help='DICT_4X4_50 id (default [hand_eye] marker_id or 0)')
    marker.add_argument('--output', default='outputs/calibration/marker.svg')
    solver = commands.add_parser('solve', help='Solve a captured session offline')
    solver.add_argument('session')
    solver.add_argument('--save', action='store_true', help='Make it the current result when validated')
    solver.add_argument('--force', action='store_true', help='With --save: adopt a result that failed validation')
    from embodied.contracts.paths import ROBOT_CONFIG
    preset = commands.add_parser('plan', help='Prepare RM65 points offline from an earlier wrist session')
    preset.add_argument('session', type=Path)
    preset.add_argument('--robot-config', type=Path, default=ROBOT_CONFIG)
    preset.add_argument('--step-rad', type=float, default=.25)
    preset.add_argument('--output', type=Path, required=True)
    preview = commands.add_parser('preview', help='Replay a saved plan in simulation only, without camera sampling')
    preview.add_argument('plan', type=Path)
    preview.add_argument('--run', type=Path, required=True)
    preview.add_argument('--speed', type=float, default=.17)
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
    elif args.command in {'capture', 'preview'}:
        if not .05 <= args.speed <= .5:
            raise ValueError('--speed must be within 0.05..0.5 rad/s')
        (capture if args.command == 'capture' else preview_plan)(args)
    elif args.command == 'plan':
        prepare_plan(args)
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
