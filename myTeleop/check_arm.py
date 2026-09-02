#!/usr/bin/env python3
"""Initialize the ARX5 controller in damping mode and print feedback."""

import argparse
import time
from pathlib import Path

import arx5_interface as arx5
import numpy as np


def read_can_counter(name: str) -> int:
    counter_path = Path("/sys/class/net/can0/statistics") / name
    return int(counter_path.read_text(encoding="utf-8").strip())


def format_values(values: object) -> str:
    array = np.asarray(values, dtype=float)
    if not np.all(np.isfinite(array)):
        raise RuntimeError(f"Non-finite ARX feedback: {array}")
    return np.array2string(array, precision=4, suppress_small=True)


def main() -> int:
    parser = argparse.ArgumentParser(description="Read ARX X5 joint feedback.")
    parser.add_argument("--can-port", default="can0", help="SocketCAN interface name.")
    parser.add_argument("--samples", type=int, default=10, help="Number of feedback samples.")
    parser.add_argument("--interval", type=float, default=0.2, help="Sampling interval in seconds.")
    parser.add_argument("--model", default="X5", help="ARX5 SDK model name.")
    parser.add_argument("--gripper-open-readout", type=float, default=-3.4)
    parser.add_argument("--gripper-width", type=float, default=0.082)
    args = parser.parse_args()

    rx_before = read_can_counter("rx_packets")
    tx_before = read_can_counter("tx_packets")

    robot_config = arx5.RobotConfigFactory.get_instance().get_config(args.model)
    robot_config.gripper_open_readout = args.gripper_open_readout
    robot_config.gripper_width = args.gripper_width
    controller_config = arx5.ControllerConfigFactory.get_instance().get_config(
        "joint_controller", robot_config.joint_dof
    )
    controller_config.background_send_recv = True

    arm = arx5.Arx5JointController(robot_config, controller_config, args.can_port)
    arm.set_log_level(arx5.LogLevel.INFO)
    try:
        for index in range(args.samples):
            state = arm.get_joint_state()
            positions = format_values(state.pos())
            velocities = format_values(state.vel())
            torques = format_values(state.torque())
            print(
                f"sample={index + 1} q={positions} dq={velocities} "
                f"torque={torques} gripper={state.gripper_pos:.4f}"
            )
            time.sleep(args.interval)
    finally:
        arm.set_to_damping()

    rx_after = read_can_counter("rx_packets")
    tx_after = read_can_counter("tx_packets")
    print(f"CAN packets: RX {rx_before} -> {rx_after}, TX {tx_before} -> {tx_after}")
    print("ARX feedback check completed; the arm is in damping mode.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
