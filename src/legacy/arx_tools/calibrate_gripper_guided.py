#!/usr/bin/env python3
"""Exclusive, browser-driven wrapper around the vendor gripper calibration."""

from __future__ import annotations

import argparse

import arx5_interface as arx5

from embodied.contracts.device_lease import DeviceLease


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ARX X5 夹爪引导零点标定")
    parser.add_argument("--model", default="X5")
    parser.add_argument("--interface", default="can0")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    lock = None
    controller = None
    try:
        # Same lease as the arm service, the arx kernel and ArmController.
        lock = DeviceLease(args.interface)
        robot_config = arx5.RobotConfigFactory.get_instance().get_config(args.model)
        # ``Arx5JointController`` validates the interpreted gripper width
        # while it is being constructed.  A stale zero/readout is exactly the
        # failure this recovery tool fixes, so use a deliberately large
        # temporary scale only for that constructor.  It maps any normal motor
        # angle into the SDK's ±5 mm tolerance without commanding motion; the
        # vendor calibration routine below reads the raw motor angle directly.
        robot_config.gripper_open_readout = 1000.0
        controller_config = arx5.ControllerConfigFactory.get_instance().get_config(
            "joint_controller", robot_config.joint_dof
        )
        # The vendor routine sends only gripper zero/reset frames.  Keep its
        # background loop disabled so no arm trajectory is created here.
        controller_config.background_send_recv = False
        controller_config.gravity_compensation = False
        controller = arx5.Arx5JointController(
            robot_config, controller_config, args.interface
        )
        print("GRIPPER_CALIBRATION_READY", flush=True)
        controller.calibrate_gripper()
        print("GRIPPER_CALIBRATION_FINISHED", flush=True)
        return 0
    except KeyboardInterrupt:
        print("GRIPPER_CALIBRATION_CANCELLED", flush=True)
        return 130
    except Exception as exc:
        print(f"GRIPPER_CALIBRATION_ERROR {exc}", flush=True)
        return 1
    finally:
        if controller is not None:
            del controller
        if lock is not None:
            lock.close()


if __name__ == "__main__":
    raise SystemExit(main())
