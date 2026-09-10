#!/usr/bin/env python3
"""Listen to X5 DM gripper startup feedback; never send CAN commands."""
import argparse
import json
import math
from pathlib import Path
import socket
import struct
import time
import tomllib


def describe_angle(angle, readout, width):
    candidates = []
    for turns in range(-2, 3):
        value = (angle + turns * 2 * math.pi) / readout * width
        if 0 <= value <= width:
            candidates.append({"turns": turns, "width_m": value})
    return {"raw_rad": angle, "sdk_width_m": angle / readout * width,
            "possible_turn_branches_not_calibration": candidates}


def main():
    root = Path(__file__).resolve().parents[2]
    config = tomllib.loads((root / "configs/project.toml").read_text())["arm_service"]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--can", default="can0")
    parser.add_argument("--seconds", type=float, default=30)
    parser.add_argument("--motor-id", type=int, default=8)
    parser.add_argument("--output", type=Path, default=root / "outputs/logs/gripper-startup.jsonl")
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 0 < args.seconds <= 300:
        parser.error("seconds must be in (0, 300]")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    count = 0
    with socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as bus, args.output.open("a") as log:
        bus.bind((args.can,))
        bus.settimeout(.2)
        end = time.monotonic() + args.seconds
        print("只监听 CAN，不使能、不运动、不校零。", flush=True)
        while time.monotonic() < end:
            try:
                packet = bus.recv(16)
            except socket.timeout:
                continue
            can_id, length, data = struct.unpack("=IB3x8s", packet)
            # X5 DM feedback uses master CAN ID 0, motor ID in the low nibble.
            if can_id != 0 or length != 8 or data[0] & 15 != args.motor_id:
                continue
            angle = int.from_bytes(data[1:3], "big") * 25 / 65535 - 12.5
            record = {"time_ns": time.time_ns(), "data": data.hex(),
                      "motor_state": data[0] >> 4,
                      **describe_angle(angle, config["gripper_open_readout"], config["gripper_width"])}
            log.write(json.dumps(record) + "\n")
            log.flush()
            if count == 0:
                print(json.dumps(record, ensure_ascii=False), flush=True)
            count += 1
    print(f"夹爪反馈 {count} 帧；记录：{args.output}")
    if count == 0:
        print("未收到夹爪反馈，无法确认启动角度；不能据此认定夹爪零点异常。")


if __name__ == "__main__":
    main()
