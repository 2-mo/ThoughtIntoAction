"""Wrist D435i calibration: marker, manual capture, previewed automatic capture, offline solve."""

import argparse
from contextlib import ExitStack
from datetime import datetime
import json
from pathlib import Path
import signal

import numpy as np

from embodied.core.model import DEFAULT_URDF, RobotModel
from embodied.core.storage import save_json
from embodied.calibration.wrist.motion import execute_plan, make_plan, validate_plan
from embodied.calibration.wrist.solver import solve
from embodied.calibration.wrist.target import MarkerDetector, marker_svg, target_spec


def read_json(path):
    with Path(path).open() as stream:
        return json.load(stream)


def capture(args, robot, plan=None):
    import cv2
    from embodied.core.project import load_config
    from embodied.core.plugins import Registry
    from embodied.calibration.wrist.session import Session
    config = load_config()
    spec = target_spec(args.marker_size_m, args.marker_id)
    if not config["cameras"]["wrist_rgb"]["enabled"]:
        raise ValueError("Wrist camera disabled in project config")
    camera_options = config["cameras"]["wrist_rgb"]
    output = args.output or f"outputs/calibration/{datetime.now():%Y%m%d-%H%M%S-%f}"
    if Path(output).exists():
        raise ValueError("Output session directory already exists; choose a new directory")
    session = None
    def interrupted(signum, frame):
        raise KeyboardInterrupt(f"Signal {signum}")
    previous_handler = signal.signal(signal.SIGTERM, interrupted)
    try:
        with ExitStack() as stack:
            camera = stack.enter_context(Registry().create(
                "camera", camera_options["driver"], serial=args.serial or camera_options["serial"],
                width=camera_options.get("width", 640), height=camera_options.get("height", 480),
                fps=camera_options.get("fps", 30)))
            detector = MarkerDetector(spec, camera.metadata)
            if plan is not None:
                # Validate visibility before even creating the arm controller.
                detector.detect(camera.read()["image"])
            plugin = 'arx' if args.runtime else 'legacy'
            options = {'directory': args.runtime, 'manual': plan is None} if args.runtime else {}
            print(f"Calibration arm plugin: {plugin}; initial mode: {'hold' if args.runtime and plan else 'free-drive'}.", flush=True)
            arm = stack.enter_context(Registry().create('calibration_arm', plugin, robot=robot, **options))
            session = Session(output, robot, camera.metadata, spec)
            if plan is not None:
                session.data["plan"] = plan
                session.save()
                execute_plan(arm, robot, plan, lambda split, i: session.capture(camera, arm, detector, split, i))
            else:
                print("Fixed board; move the damped arm by hand. s=fit, v=held-out validation, q=finish.")
                while True:
                    arm.snapshot()  # Renew the lease and check feedback while waiting for capture.
                    image = camera.read()["image"]
                    try:
                        detection = detector.detect(image)
                        text = f"marker OK: {detection['reprojection_error_undistorted_px']:.2f}px; s / v / q"
                    except ValueError as exc:
                        text = str(exc)
                    cv2.putText(image, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
                    cv2.imshow("D435i hand-eye", image)
                    key = cv2.waitKey(1) & 0xff
                    if key in (ord("q"), 27):
                        break
                    if cv2.getWindowProperty("D435i hand-eye", cv2.WND_PROP_VISIBLE) < 1:
                        break
                    if key in (ord("s"), ord("v")):
                        try:
                            session.capture(camera, arm, detector, "fit" if key == ord("s") else "validation")
                        except ValueError as exc:
                            print(f"Sample rejected: {exc}", flush=True)
            session.finish("complete")
    except BaseException as exc:
        if session is not None:
            session.finish("interrupted" if isinstance(exc, KeyboardInterrupt) else "failed", exc)
        raise
    finally:
        signal.signal(signal.SIGTERM, previous_handler)
        cv2.destroyAllWindows()
    print(f"Session: {output}/session.json; solve offline after collection.")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", default=str(DEFAULT_URDF))
    parser.add_argument("--eef", default="link6")
    parser.add_argument('--runtime', type=Path, help='Use the model and command gateway of this arx run')
    commands = parser.add_subparsers(dest="command", required=True)
    marker = commands.add_parser("marker", help="Generate a printable SVG; measure the printed black square")
    marker.add_argument("--marker-size-m", type=float, required=True)
    marker.add_argument("--marker-id", type=int, default=0)
    marker.add_argument("--output", default="outputs/calibration/marker.svg")
    planner = commands.add_parser("plan", help="Generate an offline sampling plan; never opens hardware")
    centers = planner.add_mutually_exclusive_group(required=True)
    centers.add_argument("--center", nargs=6, type=float)
    centers.add_argument("--from-session", help="Use the LAST manually captured robot pose")
    planner.add_argument("--eef-bounds", nargs=6, type=float, metavar=("XMIN", "XMAX", "YMIN", "YMAX", "ZMIN", "ZMAX"))
    planner.add_argument("--step-rad", type=float, default=0.3)
    planner.add_argument("--output", default="outputs/calibration/plan.json")
    manual = commands.add_parser("manual", help="Damped hand-guided capture (opens CAN + RGB)")
    auto = commands.add_parser("auto", help="Validate plan; only --actuate opens hardware and moves")
    auto.add_argument("--plan", required=True)
    auto.add_argument("--actuate", action="store_true")
    for sub in (manual, auto):
        sub.add_argument("--marker-size-m", type=float, required=sub is manual)
        sub.add_argument("--marker-id", type=int, default=0)
        sub.add_argument("--serial", help="Override configured D435i serial")
        sub.add_argument("--output", help="New session directory; existing directories are refused")
    solver = commands.add_parser("solve", help="Fit and independently validate a captured session")
    solver.add_argument("session")
    solver.add_argument("--output", help="Default: result.json beside session.json")
    args = parser.parse_args(argv)
    if args.runtime:
        parameters = read_json(args.runtime / 'parameters.json')
        args.urdf = str(Path(parameters['model']['xml_path']).with_suffix('.urdf'))
    robot = RobotModel(args.urdf, args.eef)
    if args.command == "marker":
        path = Path(args.output)
        if path.exists():
            raise ValueError("Marker output already exists")
        path.parent.mkdir(parents=True, exist_ok=True)
        spec = target_spec(args.marker_size_m, args.marker_id)
        path.write_text(marker_svg(spec))
        save_json(path.with_suffix(".json"), spec)
        print(f"{path}: print at 100%; measure the BLACK square ({args.marker_size_m * 1000:g} mm).")
    elif args.command == "plan":
        center = args.center
        if args.from_session:
            data = read_json(args.from_session)
            if data["robot"] != robot.metadata() or not data["samples"]:
                raise ValueError("Session model mismatch or no samples")
            center = data["samples"][-1]["q"]
        bounds = np.asarray(args.eef_bounds).reshape(3, 2).tolist() if args.eef_bounds else None
        plan = make_plan(robot, center, bounds, args.step_rad)
        save_json(args.output, plan)
        print(json.dumps({"plan": args.output, **validate_plan(plan, robot)}, indent=2))
    elif args.command == "solve":
        data = read_json(args.session)
        if data.get("schema_version") != 1 or data.get("kind") != "eye_in_hand_samples":
            raise ValueError("Unknown calibration session schema")
        if data["robot"] != robot.metadata():
            raise ValueError("Session robot model/frame mismatch")
        # Ensure recorded FK agrees with q/current URDF before estimating extrinsics.
        for row in data["samples"]:
            if not np.allclose(robot.fk(row["q"]), row["T_base_eef"], atol=1e-7, rtol=0):
                raise ValueError("Stored robot pose disagrees with joint angles/URDF")
        result = solve(data["samples"])
        result.update({key: data[key] for key in ("robot", "camera", "target")})
        result["source_session"] = str(Path(args.session).resolve())
        output = args.output or str(Path(args.session).with_name("result.json"))
        save_json(output, result)
        print(f"{output}: {result['status']}. Project calibration flag was not changed.")
    elif args.command == "auto":
        plan = read_json(args.plan)
        summary = validate_plan(plan, robot)
        print(json.dumps(summary, indent=2))
        if args.actuate:
            if args.marker_size_m is None or plan.get("eef_bounds_m") is None:
                raise ValueError("Live auto needs --marker-size-m and a plan with measured --eef-bounds")
            capture(args, robot, plan)
        else:
            print("Offline plan check complete; no hardware opened. Start view-arm, then use control-arm --replay to inspect motion.")
    else:
        capture(args, robot)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        raise SystemExit(130)
    except (ValueError, OSError, RuntimeError) as exc:
        raise SystemExit(str(exc))
