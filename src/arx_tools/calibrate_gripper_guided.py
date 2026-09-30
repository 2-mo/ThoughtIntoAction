#!/usr/bin/env python3
"""Exclusive, browser-driven wrapper around the vendor gripper calibration."""

from __future__ import annotations

import argparse
import fcntl
import os
from pathlib import Path
import sys

import arx5_interface as arx5


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ARX X5 夹爪引导零点标定")
    parser.add_argument("--model", default="X5")
    parser.add_argument("--interface", default="can0")
    return parser.parse_args()


class InterfaceLock:
    """Use the same advisory lock as the controller before touching CAN."""

    def __init__(self, interface: str) -> None:
        safe_name = "".join(
            char if char.isalnum() or char in "-_" else "_"
            for char in interface
        )
        self.path = Path("/tmp") / f"arx5-control-{safe_name}.lock"
        self.fd: int | None = None

    def acquire(self) -> None:
        fd = os.open(self.path, os.O_RDWR | os.O_CREAT, 0o600)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            os.close(fd)
            raise RuntimeError(
                "CAN 正被其他控制器占用；请先断开机械臂后重试"
            ) from exc
        os.ftruncate(fd, 0)
        os.write(fd, f"pid={os.getpid()}\n".encode("ascii"))
        self.fd = fd

    def release(self) -> None:
        if self.fd is None:
            return
        try:
            fcntl.flock(self.fd, fcntl.LOCK_UN)
        finally:
            os.close(self.fd)
            self.fd = None


def main() -> int:
    args = parse_args()
    lock = InterfaceLock(args.interface)
    controller = None
    try:
        lock.acquire()
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
        lock.release()


if __name__ == "__main__":
    raise SystemExit(main())
