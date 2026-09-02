#!/usr/bin/env python3
"""Small, reversible X5 joint tracking test without XR or Placo."""

import time

import numpy as np
import tyro

from teleop_official_x5 import OfficialX5HardwareAdapter


def main(
    backend: str = "stanford",
    can_port: str = "can0",
    joint: int = 1,
    delta_rad: float = 0.03,
    max_joint_speed_rad_s: float = 0.05,
    control_rate_hz: int = 50,
    preview_time_s: float = 0.05,
    joint_soft_limit_margin_rad: float = 0.05,
    settle_tolerance_rad: float = 0.005,
    timeout_s: float = 3.0,
    max_uncommanded_drift_rad: float = 0.05,
    max_test_joint_overshoot_rad: float = 0.02,
    max_abs_joint_speed_rad_s: float = 1.0,
    gripper_open_readout: float = -3.4,
    gripper_width: float = 0.082,
    actuate: bool = False,
) -> None:
    """Read one joint, or move it slightly and return when --actuate is set."""
    if backend not in {"stanford", "arx2025"}:
        raise ValueError("backend must be 'stanford' or 'arx2025'")
    if joint < 1 or joint > 6:
        raise ValueError("joint must be between 1 and 6")
    if not 0.0 < abs(delta_rad) <= 0.1:
        raise ValueError("abs(delta_rad) must be in (0, 0.1]")
    if not 0.0 < max_joint_speed_rad_s <= 0.2:
        raise ValueError("max_joint_speed_rad_s must be in (0, 0.2]")
    if control_rate_hz <= 0 or preview_time_s <= 0.0 or timeout_s <= 0.0:
        raise ValueError("rates and times must be positive")
    if (
        max_uncommanded_drift_rad <= 0.0
        or max_test_joint_overshoot_rad <= 0.0
        or max_abs_joint_speed_rad_s <= 0.0
    ):
        raise ValueError("watchdog limits must be positive")

    adapter_class = OfficialX5HardwareAdapter
    if backend == "arx2025":
        from teleop_arx_x5_2025 import ARXX52025HardwareAdapter

        adapter_class = ARXX52025HardwareAdapter
    adapter = adapter_class(
            can_port=can_port,
            actuate=actuate,
            preview_time_s=preview_time_s,
            gripper_open_readout=gripper_open_readout,
            gripper_width=gripper_width,
            control_rate_hz=control_rate_hz,
            max_joint_speed_rad_s=max_joint_speed_rad_s,
            joint_soft_limit_margin_rad=joint_soft_limit_margin_rad,
        )
    initial_state = adapter.get_joint_positions()
    initial_q = initial_state[:6].copy()
    gripper_position = float(initial_state[6])
    print(f"Initial q={np.round(initial_q, 5)}, gripper={gripper_position:.5f} m")

    diagnostics = adapter.get_runtime_diagnostics()
    if "kp" in diagnostics:
        print(
            "Controller gains: "
            f"kp={np.round(diagnostics['kp'], 3)} "
            f"kd={np.round(diagnostics['kd'], 3)}"
        )
    else:
        print("Controller gains are managed internally by ARX X5-2025 type=2.")

    if not actuate:
        print("READ-ONLY complete. Add --actuate only when the arm is supported.")
        adapter.set_to_damping()
        return

    index = joint - 1
    q_min = np.asarray(adapter.robot_config.joint_pos_min, dtype=float)
    q_max = np.asarray(adapter.robot_config.joint_pos_max, dtype=float)
    soft_min = q_min[index] + joint_soft_limit_margin_rad
    soft_max = q_max[index] - joint_soft_limit_margin_rad
    target_q = initial_q.copy()
    target_q[index] = float(np.clip(initial_q[index] + delta_rad, soft_min, soft_max))
    if abs(target_q[index] - initial_q[index]) < settle_tolerance_rad:
        opposite = initial_q[index] - delta_rad
        target_q[index] = float(np.clip(opposite, soft_min, soft_max))
    if abs(target_q[index] - initial_q[index]) < settle_tolerance_rad:
        adapter.set_to_damping()
        raise RuntimeError(f"J{joint} has no room for the requested test movement")

    period_s = 1.0 / control_rate_hz

    def check_watchdog(
        actual: np.ndarray,
        velocity: np.ndarray,
        phase_start: np.ndarray,
        target: np.ndarray,
    ) -> None:
        other_joint_mask = np.ones(6, dtype=bool)
        other_joint_mask[index] = False
        other_drift = np.abs(actual - initial_q)
        if np.any(other_drift[other_joint_mask] > max_uncommanded_drift_rad):
            offending = int(
                np.flatnonzero(
                    other_joint_mask
                    & (other_drift > max_uncommanded_drift_rad)
                )[0]
            )
            raise RuntimeError(
                f"WATCHDOG: uncommanded J{offending + 1} drift "
                f"{other_drift[offending]:.4f} rad"
            )

        corridor_min = min(phase_start[index], target[index])
        corridor_max = max(phase_start[index], target[index])
        if (
            actual[index] < corridor_min - max_test_joint_overshoot_rad
            or actual[index] > corridor_max + max_test_joint_overshoot_rad
        ):
            raise RuntimeError(
                f"WATCHDOG: J{joint} left its safe test corridor; "
                f"actual={actual[index]:.4f} rad"
            )

        max_velocity = float(np.max(np.abs(velocity)))
        if max_velocity > max_abs_joint_speed_rad_s:
            offending = int(np.argmax(np.abs(velocity)))
            raise RuntimeError(
                f"WATCHDOG: J{offending + 1} speed "
                f"{velocity[offending]:.3f} rad/s"
            )

    def follow(target: np.ndarray, label: str) -> bool:
        deadline = time.monotonic() + timeout_s
        next_report = 0.0
        phase_start = adapter.get_joint_positions()[:6]
        while time.monotonic() < deadline:
            started = time.monotonic()
            before = adapter.get_joint_positions()[:6]
            before_velocity = adapter.get_joint_velocities()[:6]
            check_watchdog(before, before_velocity, phase_start, target)
            adapter.set_joint_positions(target)
            adapter.set_catch_pos(gripper_position)
            actual = adapter.get_joint_positions()[:6]
            velocity = adapter.get_joint_velocities()[:6]
            check_watchdog(actual, velocity, phase_start, target)
            error = float(target[index] - actual[index])
            if started >= next_report:
                diagnostics = adapter.get_runtime_diagnostics()
                command_label = (
                    "adapter_pos" if "actual_current" in diagnostics else "sdk_pos"
                )
                report = (
                    f"{label}: J{joint} actual={actual[index]:+.5f} "
                    f"target={target[index]:+.5f} error={error:+.5f} "
                    f"{command_label}={diagnostics['sdk_cmd_pos'][index]:+.5f} "
                    f"actual_vel={diagnostics['actual_vel'][index]:+.5f} "
                    f"sdk_vel={diagnostics['sdk_cmd_vel'][index]:+.5f}"
                )
                if "actual_torque" in diagnostics:
                    report += (
                        f" actual_tau={diagnostics['actual_torque'][index]:+.4f}"
                        f" sdk_tau={diagnostics['sdk_cmd_torque'][index]:+.4f}"
                        f" ts=({diagnostics['feedback_timestamp']:.4f},"
                        f"{diagnostics['command_timestamp']:.4f},"
                        f"{diagnostics['controller_timestamp']:.4f})"
                    )
                else:
                    report += (
                        f" actual_current="
                        f"{diagnostics['actual_current'][index]:+.4f}"
                    )
                print(report)
                next_report = started + 0.2
            if abs(error) <= settle_tolerance_rad:
                return True
            time.sleep(max(0.0, period_s - (time.monotonic() - started)))
        return False

    returned = False
    try:
        print(
            f"Testing J{joint}: {initial_q[index]:+.5f} -> "
            f"{target_q[index]:+.5f} rad"
        )
        reached = follow(target_q, "OUT")
        print(f"Outward target {'reached' if reached else 'timed out'}; returning.")
        returned = follow(initial_q, "BACK")
        print(f"Initial target {'reached' if returned else 'timed out'}.")
    finally:
        adapter.set_to_damping()
        print("X5 is in damping mode.")

    if not returned:
        raise RuntimeError("Joint did not return within tolerance before timeout")


if __name__ == "__main__":
    tyro.cli(main)
