"""Control a virtual arm service via terminal or pose input; monitoring is independent."""

import argparse
from contextlib import ExitStack
import time
import json
import shlex
import threading

import numpy as np

from robot_core.model import DEFAULT_URDF, RobotModel
from robot_control.trajectory import trajectory
from robot_core.plugins import Registry
from robot_control.client import ArmClient


def replay_points(path, robot):
    with open(path) as stream:
        value = json.load(stream)
    if value.get("robot") != robot.metadata():
        raise ValueError("Replay model/frame metadata differs from the current robot")
    rows = value.get("waypoints", value.get("samples", []))
    if not rows:
        raise ValueError("No waypoints/samples in replay file")
    qs = [robot.joints(row["q"]) for row in rows]
    return [robot.joints(value["center"]), *qs] if "center" in value else qs


def main(argv=None):
    parser = argparse.ArgumentParser(prog="control-arm", description=__doc__)
    parser.add_argument("--input", default="terminal", help="terminal / quest / idle / registered pose-input plugin")
    parser.add_argument("--service-socket", help="Local arm-service socket; expects a virtual service")
    parser.add_argument("--urdf", default=str(DEFAULT_URDF))
    parser.add_argument("--eef", default="link6")
    parser.add_argument("--replay", help="Calibration plan or session JSON")
    parser.add_argument("--headless", action="store_true", help="Validate/FK only; no telemetry")
    parser.add_argument("--control-rate-hz", type=int, default=50)
    parser.add_argument("--max-joint-speed-rad-s", type=float, default=.2)
    parser.add_argument("--joint-soft-limit-margin-rad", type=float, default=.05)
    parser.add_argument("--acceleration", type=float, default=.3, help="Terminal trajectory acceleration limit (rad/s²)")
    pose = parser.add_argument_group("Pose inputs (quest / idle / registered plugin)")
    pose.add_argument("--control-mode", choices=("pose", "position"), default="pose")
    pose.add_argument("--scale-factor", type=float, default=.5)
    pose.add_argument("--home-joints", nargs=6, type=float)
    pose.add_argument("--grasp-ready-joints", nargs=6, type=float)
    pose.add_argument("--home-hold-s", type=float, default=1.)
    pose.add_argument("--preset-tolerance-rad", type=float, default=.03)
    args = parser.parse_args(argv)
    registry = Registry()
    if args.input != "terminal":
        registry.spec("input", args.input)
        if args.replay:
            parser.error("--replay requires --input terminal; replay and live pose input cannot run together")
        if args.eef != "link6":
            parser.error("Pose-input control currently requires --eef link6")
    for name in ("control_rate_hz", "max_joint_speed_rad_s", "acceleration",
                 "scale_factor", "home_hold_s", "preset_tolerance_rad"):
        if not np.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(f"--{name.replace('_', '-')} must be finite and positive")
    for name in ("joint_soft_limit_margin_rad",):
        if not np.isfinite(getattr(args, name)) or getattr(args, name) < 0:
            parser.error(f"--{name.replace('_', '-')} must be finite and nonnegative")
    robot = RobotModel(args.urdf, args.eef)
    motion = {"speed": args.max_joint_speed_rad_s, "acceleration": args.acceleration,
              "hz": args.control_rate_hz, "margin": args.joint_soft_limit_margin_rad}
    rows = replay_points(args.replay, robot) if args.replay else []
    if args.replay:
        with open(args.replay) as stream:
            motion.update(json.load(stream).get("motion", {}))
    rows = [robot.joints(q, motion["margin"]) for q in rows]
    if args.headless:
        print(json.dumps({"robot": robot.metadata(), "poses": [robot.fk(x).tolist() for x in rows]}, indent=2))
        return
    print(f"Virtual arm control: input={args.input}; connecting to arm-service", flush=True)
    if args.input == "terminal":
        run_terminal(args, robot, rows, motion)
    else:
        run_pose_input(args, robot)


def run_pose_input(args, robot):
    # Import only for a live pose input. No second CLI parser or old-command forwarding.
    from copy import deepcopy
    from quest_teleop.base_controller import DEFAULT_X5_MANIPULATOR_CONFIG
    from quest_teleop.presets import (
        DEFAULT_HOME_JOINTS, DEFAULT_GRASP_READY_JOINTS, PresetSingleX5Controller,
    )

    config = deepcopy(DEFAULT_X5_MANIPULATOR_CONFIG)
    config["right_arm"]["control_mode"] = args.control_mode
    controller = PresetSingleX5Controller(
        robot_urdf_path=str(robot.urdf), dynamics_urdf_path=str(robot.urdf),
        manipulator_config=config, can_ports={"right_arm": "simulation"},
        backend="service", service_socket=args.service_socket, input_plugin=args.input,
        simulation_mode=True, actuate=False,
        control_rate_hz=args.control_rate_hz, scale_factor=args.scale_factor,
        max_joint_speed_rad_s=args.max_joint_speed_rad_s,
        joint_soft_limit_margin_rad=args.joint_soft_limit_margin_rad,
        home_joints=DEFAULT_HOME_JOINTS if args.home_joints is None else args.home_joints,
        grasp_ready_joints=DEFAULT_GRASP_READY_JOINTS if args.grasp_ready_joints is None else args.grasp_ready_joints,
        home_hold_s=args.home_hold_s, preset_tolerance_rad=args.preset_tolerance_rad,
        enable_camera=False, enable_log_data=False, enable_session=False,
    )
    try:
        controller.run()
    finally:
        controller.close()


def run_terminal(args, robot, rows, motion):
    with ExitStack() as resources:
        arm = ArmClient(robot, path=args.service_socket, hardware=False, actuate=True,
                        input_name="terminal", speed=motion["speed"])
        resources.callback(arm.close)
        arm.start_control()
        # The terminal keeps its control lease while waiting for the next command.
        # Feedback publication belongs exclusively to arm-service.
        stop = threading.Event()
        errors = []
        def publish():
            while not stop.is_set():
                try:
                    arm.get_joint_positions()
                except Exception as exc:
                    errors.append(exc)
                    stop.set()
                    return
                stop.wait(1. / motion["hz"])
        worker = threading.Thread(target=publish, name="terminal-arm-lease", daemon=True)
        worker.start()
        def stop_sampling():
            stop.set()
            worker.join(timeout=2)
        resources.callback(stop_sampling)

        def move(target):
            start = arm.get_joint_positions()[:6]
            times, qs = trajectory(robot, start, target, **motion)
            beginning = time.monotonic()
            for sample, t in zip(qs, times):
                time.sleep(max(0., beginning + t - time.monotonic()))
                arm.set_joint_positions(sample)
                arm.set_catch_pos(arm.get_joint_positions()[6])
                if errors:
                    raise errors[0]
            # Hold the final command until feedback settles. The service can
            # coalesce requests and rate-limit more tightly than the trajectory.
            deadline = time.monotonic() + 5.
            while np.max(np.abs(arm.get_joint_positions()[:6] - target)) > .001:
                if time.monotonic() > deadline:
                    raise RuntimeError("Arm did not settle at the trajectory endpoint")
                arm.set_joint_positions(target)
                arm.set_catch_pos(arm.get_joint_positions()[6])
                time.sleep(1. / motion["hz"])

        for target in rows:
            move(target)
        print("Commands (rad/m): q J1 ... J6 | ik x y z rx ry rz (rotation vector) | fk | exit")
        while True:
            try:
                if errors:
                    raise errors[0]
                tokens = shlex.split(input("sim> "))
                if not tokens:
                    continue
                if tokens[0] in {"exit", "quit"}:
                    break
                q = arm.get_joint_positions()[:6]
                if tokens == ["fk"]:
                    print(robot.fk(q))
                    continue
                values = np.array([float(x) for x in tokens[1:]])
                if tokens[0] == "q":
                    target = robot.joints(values)
                elif tokens[0] == "ik" and len(values) == 6:
                    import cv2
                    from robot_core.transforms import from_rt
                    target = robot.ik(from_rt(cv2.Rodrigues(values[3:])[0], values[:3]), q)
                else:
                    raise ValueError("Use q/ik with six numbers, fk, or exit")
                move(target)
            except (EOFError, KeyboardInterrupt):
                break
            except ValueError as exc:
                print(exc)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        raise SystemExit(130)
