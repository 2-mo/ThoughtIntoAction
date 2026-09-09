#!/usr/bin/env python3
"""Print the Quest right-controller data received by XRoboToolkit PC Service."""

import argparse
import time

import xrobotoolkit_sdk as xrt


def main() -> int:
    parser = argparse.ArgumentParser(description="Check XRoboToolkit right-controller data.")
    parser.add_argument("--duration", type=float, default=5.0, help="Sampling duration in seconds.")
    parser.add_argument("--interval", type=float, default=0.2, help="Sampling interval in seconds.")
    args = parser.parse_args()

    received_data = False
    xrt.init()
    try:
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            timestamp_ns = int(xrt.get_time_stamp_ns())
            pose = list(xrt.get_right_controller_pose())
            grip = float(xrt.get_right_grip())
            trigger = float(xrt.get_right_trigger())
            print(
                f"timestamp={timestamp_ns} pose={pose} "
                f"right_grip={grip:.3f} right_trigger={trigger:.3f}"
            )
            received_data = received_data or timestamp_ns > 0
            time.sleep(args.interval)
    finally:
        xrt.close()

    if not received_data:
        print("No XR data received. Start XRoboToolkit PC Service and the Quest app.")
        return 1

    print("XR data is updating.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
