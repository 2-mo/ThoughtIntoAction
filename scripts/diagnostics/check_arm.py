#!/usr/bin/env python3
"""Print feedback from arm-service without acquiring control or opening CAN."""

import argparse
import math
import time

import numpy as np

from legacy.robot_control.client import ArmClient
from embodied.core.model import RobotModel
from embodied.core.project import load_config, resolve_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--service-socket")
    parser.add_argument("--virtual", action="store_true", help="Expect a virtual service")
    parser.add_argument("--samples", type=int, default=10)
    parser.add_argument("--interval", type=float, default=.2)
    args = parser.parse_args()
    if args.samples <= 0 or not math.isfinite(args.interval) or args.interval < 0:
        parser.error("Samples must be positive and interval finite/nonnegative")
    robot = RobotModel(resolve_path(load_config()["robot"]["urdf"]))
    client = ArmClient(robot, path=args.service_socket, hardware=not args.virtual)
    try:
        for index in range(args.samples):
            state = client.snapshot()
            print(f"sample={index + 1} q={np.round(state['actual_pos'], 4)} "
                  f"dq={np.round(state['actual_vel'], 4)} gripper={state['gripper_actual_pos']:.4f} "
                  f"mode={client.info['mode']} owner={client.info['owner']}", flush=True)
            time.sleep(args.interval)
    finally:
        client.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
