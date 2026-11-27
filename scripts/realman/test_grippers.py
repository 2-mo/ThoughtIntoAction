"""Inspect both EG2 grippers or test one empty gripper with explicit execution."""

import argparse
from contextlib import closing
import json
from pathlib import Path
import sys
import time

from embodied.devices.robots.realman.gripper import move, require_ready, status
from embodied.devices.robots.realman.protocol import Client


ROOT = Path(__file__).resolve().parents[2]
CONFIGS = {"left": "rm65-19.json", "right": "rm65-18.json"}


def run(side, action, *, speed, client_factory=Client):
    config = json.loads((ROOT / "configs/robots/realman" / CONFIGS[side]).read_text())
    with closing(client_factory(config["host"], config.get("port", 8080),
                                timeout_s=config.get("timeout_s", .3),
                                allow_gripper=action != "status")) as client:
        before = status(client)
        yield {"side": side, "host": config["host"], "phase": "before", "feedback": before}
        require_ready(before)
        if action == "status":
            return
        arm = client.query("get_current_arm_state")["arm_state"]
        if not isinstance(arm.get("err"), list):
            raise ValueError("Missing RM65 arm error list")
        if any(arm["err"]):
            raise RuntimeError(f"RM65 reports arm errors: {arm['err']}")
        for direction in (("open", "close", "open") if action == "cycle" else (action,)):
            started = time.monotonic()
            result = move(client, direction, speed=speed, force=50)
            yield {"side": side, "phase": "ack", "elapsed_s": time.monotonic() - started, **result}
            if not result["accepted"]:
                raise RuntimeError(result["reason"])
            deadline = started + 6
            while time.monotonic() < deadline:
                feedback = status(client)
                yield {"side": side, "phase": "sample", "action": direction,
                       "elapsed_s": time.monotonic() - started, "feedback": feedback}
                require_ready(feedback)
                position = feedback["actpos"]
                if direction == "close" and feedback["mode"] == 6:
                    raise RuntimeError("Force stop before confirming empty close")
                reached = position >= 995 if direction == "open" else position <= 5
                if reached and feedback["mode"] in (1, 2, 3):
                    yield {"side": side, "phase": "complete", "action": direction,
                           "elapsed_s": time.monotonic() - started}
                    break
                time.sleep(.08)
            else:
                raise TimeoutError(f"{direction}: endpoint not confirmed within 6 s")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("status", "open", "close", "cycle"), nargs="?",
                        default="status")
    parser.add_argument("--arm", choices=("left", "right", "both"), default="both")
    parser.add_argument("--speed", type=int, default=200, help="EG2 raw speed, 1–1000")
    parser.add_argument("--execute", action="store_true",
                        help="Use after stopping the target arm's control session and clearing the jaws")
    args = parser.parse_args()
    if args.action != "status" and (not args.execute or args.arm == "both"):
        parser.error("Motion requires --execute and one explicit --arm")
    if not 1 <= args.speed <= 1000:
        parser.error("--speed must be in 1..1000")
    failed = False
    for side in CONFIGS if args.arm == "both" else (args.arm,):
        try:
            for row in run(side, args.action, speed=args.speed):
                print(json.dumps(row), flush=True)
        except (OSError, ValueError, RuntimeError, KeyError) as exc:
            print(json.dumps({"side": side, "phase": "error", "error": str(exc)}), flush=True)
            failed = True
    return int(failed)


if __name__ == "__main__":
    sys.exit(main())
