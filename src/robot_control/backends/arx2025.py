"""Optional ARX X5-2025 type=2 backend; loaded only by explicit experimental commands."""

import sys
import time
from types import SimpleNamespace

import numpy as np

from robot_core.project import X5_PYTHON_ROOT
from robot_control.trajectory import bounded_joint_target


ARX_X5_PYTHON_ROOT = X5_PYTHON_ROOT
ARX_X5_2025_URDF = (
    ARX_X5_PYTHON_ROOT / "bimanual" / "script" / "x5_2025.urdf"
)

X5_JOINT_MIN = np.array([-3.14, -0.05, -0.1, -1.6, -1.57, -2.0])
X5_JOINT_MAX = np.array([2.618, 3.5, 3.2, 1.55, 1.57, 2.0])


class ARXX52025HardwareAdapter:
    """Expose XRoboToolkit's arm API through official ``SingleArm(type=2)``."""

    def __init__(
        self,
        can_port: str,
        actuate: bool,
        control_rate_hz: int,
        max_joint_speed_rad_s: float,
        joint_soft_limit_margin_rad: float,
        gripper_open_readout: float,
        gripper_width: float,
        **_: object,
    ) -> None:
        if not ARX_X5_2025_URDF.is_file():
            raise FileNotFoundError(ARX_X5_2025_URDF)
        self.actuate = actuate
        self._max_step_rad = max_joint_speed_rad_s / control_rate_hz
        self._max_command_lead_rad = 0.05
        self._soft_limit_margin_rad = joint_soft_limit_margin_rad
        self._gripper_open_readout = gripper_open_readout
        self._gripper_width = gripper_width
        self.robot_config = SimpleNamespace(
            joint_dof=6,
            joint_pos_min=X5_JOINT_MIN.copy(),
            joint_pos_max=X5_JOINT_MAX.copy(),
            gripper_width=gripper_width,
        )

        # ARX documents type=2 as the X5-2025 / AC-one hardware path. Its
        # constructor selects x5_2025.urdf and the matching low-level driver.
        # Load the optional native SDK only when requested, so --help and the
        # default Stanford backend do not require its system KDL/ROS libraries.
        if str(ARX_X5_PYTHON_ROOT) not in sys.path:
            sys.path.insert(0, str(ARX_X5_PYTHON_ROOT))
        try:
            from bimanual import SingleArm
        except (ImportError, OSError, TypeError) as exc:
            raise RuntimeError(
                "Official ARX SDK unavailable. Build it with ./run.sh build-x5-2025-sdk "
                "and install its system KDL/ROS libraries; see docs/setup.md."
            ) from exc
        from embodied_contracts.device_lease import DeviceLease
        self._device_lease = DeviceLease(can_port)
        try:
            self.arm = SingleArm(
                {
                    "can_port": can_port,
                    "type": 2,
                    "num_joints": 6,
                    "dt": 1.0 / control_rate_hz,
                }
            )
            time.sleep(0.1)
            raw = self._read_raw_positions()
            self._q_target = raw[:6].copy()
            self._gripper_target = self._raw_gripper_to_width(raw[6])
            self._last_diagnostic_time = 0.0
            self._previous_q_desired = self._q_target.copy()
            self._peak_ik_step = 0.0
            self._position_mode_active = False
            if not actuate:
                self.set_to_damping()

        except BaseException:
            self.close()
            raise

    def _read_raw_positions(self) -> np.ndarray:
        raw = np.asarray(self.arm.get_joint_positions(), dtype=float)
        if raw.shape[0] < 7 or not np.all(np.isfinite(raw[:7])):
            self.set_to_damping()
            raise RuntimeError(f"Invalid ARX X5-2025 feedback: {raw}")
        q = raw[:6]
        if np.any(q < X5_JOINT_MIN - 0.2) or np.any(q > X5_JOINT_MAX + 0.2):
            self.set_to_damping()
            raise RuntimeError(f"Out-of-range ARX X5-2025 feedback: {q}")
        return raw

    def _raw_gripper_to_width(self, raw_position: float) -> float:
        if abs(self._gripper_open_readout) < 1e-9:
            return 0.0
        return float(
            np.clip(
                raw_position / self._gripper_open_readout * self._gripper_width,
                0.0,
                self._gripper_width,
            )
        )

    def _width_to_raw_gripper(self, width: float) -> float:
        return float(
            np.clip(width, 0.0, self._gripper_width)
            / self._gripper_width
            * self._gripper_open_readout
        )

    def get_joint_positions(self) -> np.ndarray:
        raw = self._read_raw_positions()
        return np.concatenate((raw[:6], [self._raw_gripper_to_width(raw[6])]))

    def get_joint_velocities(self) -> np.ndarray:
        raw = np.asarray(self.arm.get_joint_velocities(), dtype=float)
        if raw.shape[0] < 6 or not np.all(np.isfinite(raw[:6])):
            self.set_to_damping()
            raise RuntimeError(f"Invalid ARX X5-2025 velocity feedback: {raw}")
        gripper_velocity = 0.0
        if raw.shape[0] >= 7 and abs(self._gripper_open_readout) > 1e-9:
            gripper_velocity = (
                raw[6] / self._gripper_open_readout * self._gripper_width
            )
        return np.concatenate((raw[:6], [gripper_velocity]))

    def get_runtime_diagnostics(self) -> dict[str, np.ndarray]:
        raw = self._read_raw_positions()
        velocity = np.asarray(self.arm.get_joint_velocities(), dtype=float)
        current = np.asarray(self.arm.get_joint_currents(), dtype=float)
        return {
            "actual_pos": raw[:6].copy(),
            "actual_vel": velocity[:6].copy(),
            "actual_current": current[:6].copy(),
            "sdk_cmd_pos": self._q_target.copy(),
            "sdk_cmd_vel": np.zeros(6),
        }

    def set_joint_positions(self, positions: np.ndarray) -> bool:
        desired = np.asarray(positions, dtype=float)
        if desired.shape != (6,) or not np.all(np.isfinite(desired)):
            raise ValueError(f"Expected six finite X5 joints, got {desired}")
        actual = self._read_raw_positions()[:6]
        desired_step = float(np.max(np.abs(desired - self._previous_q_desired)))
        self._peak_ik_step = max(self._peak_ik_step, desired_step)
        self._previous_q_desired = desired.copy()
        self._q_target, limited = bounded_joint_target(
            q_actual=self._q_target,
            q_desired=desired,
            q_min=X5_JOINT_MIN,
            q_max=X5_JOINT_MAX,
            max_step_rad=self._max_step_rad,
            soft_limit_margin_rad=self._soft_limit_margin_rad,
        )
        self._q_target = np.clip(
            self._q_target,
            actual - self._max_command_lead_rad,
            actual + self._max_command_lead_rad,
        )
        now = time.monotonic()
        if now - self._last_diagnostic_time >= 0.2:
            print(
                "ARX X5-2025 "
                f"q_actual={np.round(actual, 4)} "
                f"q_des={np.round(desired, 4)} "
                f"q_safe={np.round(self._q_target, 4)} "
                f"peak_ik_step={self._peak_ik_step:.4f} "
                f"tracking_error={np.max(np.abs(limited - actual)):.4f}"
            )
            self._last_diagnostic_time = now
            self._peak_ik_step = 0.0
        return True

    def set_catch_pos(self, position: float) -> bool:
        self._gripper_target = float(
            np.clip(position, 0.0, self._gripper_width)
        )
        if self.actuate:
            # Submit one coherent arm target per XR control cycle. The official
            # type=2 driver supplies its own position loop and gravity model.
            raw_gripper = self._width_to_raw_gripper(self._gripper_target)
            # ARX's current leader/follower example forwards the complete
            # seven-value get_joint_positions() result here, then updates the
            # gripper separately. Sending only six values can leave the
            # closed-source controller's position target uninitialized.
            complete_target = np.concatenate((self._q_target, [raw_gripper]))
            low_level_arm = self.arm.arm
            low_level_arm.set_joint_positions(complete_target.tolist())
            low_level_arm.set_catch(raw_gripper)
            if not self._position_mode_active:
                # Target exchange and status changes are asynchronous inside
                # the official driver. Prime several complete current-pose
                # targets while motors remain SOFT, allowing its controller
                # thread to replace the default zero trajectory before status
                # 5 can enable position control.
                for _ in range(5):
                    time.sleep(0.02)
                    low_level_arm.set_joint_positions(complete_target.tolist())
                    low_level_arm.set_catch(raw_gripper)
                self._read_raw_positions()
                low_level_arm.set_arm_status(5)
                self.arm.arm_status = 5
                self._position_mode_active = True
                print("ARX X5-2025 position mode primed at the current pose.")
        return True

    def set_to_damping(self) -> None:
        if getattr(self, "arm", None) is not None:
            self.arm.arm.set_arm_status(0)
            self.arm.arm_status = 0
            self._position_mode_active = False

    def close(self) -> None:
        try:
            self.set_to_damping()
        finally:
            self.arm = None
            lease = getattr(self, "_device_lease", None)
            if lease is not None:
                lease.close()

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass
