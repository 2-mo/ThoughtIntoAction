"""Operate the running teleop session; never construct a robot/camera controller."""

import argparse
import json
import time

from embodied.core.session_protocol import request


def main(argv=None):
    from embodied.core.project import load_config, resolve_path
    parser = argparse.ArgumentParser(description=__doc__)
    groups = parser.add_subparsers(dest="group", required=True)
    session = groups.add_parser("session")
    session_commands = session.add_subparsers(dest="command", required=True)
    status = session_commands.add_parser("status")
    status.add_argument("--watch", action="store_true")
    pause = session_commands.add_parser("pause")
    resume = session_commands.add_parser("resume")
    demo = session_commands.add_parser("demo", help="Synthetic robot/RGB; no devices")
    demo.add_argument("--seconds", type=float, default=0, help="0 runs until Ctrl+C")
    recording = groups.add_parser("record")
    commands = recording.add_subparsers(dest="command", required=True)
    start = commands.add_parser("start")
    start.add_argument("--task", required=True)
    stop = commands.add_parser("stop")
    stop.add_argument("--outcome", choices=["success", "failure", "unknown", "discarded"], default="unknown")
    for sub in (status, start, stop, pause, resume):
        sub.add_argument("--socket", help="Override local session socket")
    args = parser.parse_args(argv)
    if args.group == "session" and args.command == "demo":
        from legacy.quest_teleop.runtime.demo import run_demo
        run_demo(args.seconds)
        return
    path = args.socket or str(resolve_path(load_config()["runtime"]["socket"]))
    if args.group == "session" and args.command in ("pause", "resume"):
        print(json.dumps(request(path, "input." + args.command), ensure_ascii=False))
        return
    if args.group == "record":
        values = {"task": args.task} if args.command == "start" else {"outcome": args.outcome}
        result = request(path, "record." + args.command, **values)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return
    while True:
        print(json.dumps(request(path, "status"), ensure_ascii=False, indent=2), flush=True)
        if not args.watch:
            break
        time.sleep(1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        raise SystemExit(130)
    except (OSError, ValueError, RuntimeError) as exc:
        raise SystemExit(f"Session command failed: {exc}. Start teleop-presets-x5 with --enable-session.")
