"""Synthetic feedback and RGB to exercise CLI/recording/monitoring without any device."""

import copy
import signal
import time

import numpy as np

from legacy.quest_teleop.runtime.session import TeleopSession


class SyntheticCamera:
    def __init__(self, serial, width, height, fps):
        self.metadata = {"serial": "SYNTHETIC", "model": "synthetic RGB", "width": width,
                         "height": height, "fps": fps, "format": "bgr8"}
        self.sequence = 0

    def __enter__(self):
        return self

    def read(self):
        import cv2
        time.sleep(1 / self.metadata["fps"])
        self.sequence += 1
        image = np.zeros((self.metadata["height"], self.metadata["width"], 3), np.uint8)
        image[:] = [40, 30, 20]
        cv2.putText(image, f"SYNTHETIC - frame {self.sequence}", (10, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, .6, (120, 220, 160), 1)
        stamp = time.monotonic_ns()
        return {"image": image, "frame_number": self.sequence, "device_timestamp_ms": stamp / 1e6,
                "device_timestamp_domain": "synthetic", "host_received_monotonic_ns": stamp}

    def __exit__(self, *args):
        pass


def run_demo(seconds=0):
    from embodied.core.project import load_config, resolve_path
    from embodied.core.model import RobotModel
    if not np.isfinite(seconds) or seconds < 0:
        raise ValueError("Demo seconds must be finite and non-negative")
    config = copy.deepcopy(load_config())
    config["paths"]["dataset_dir"] = "outputs/demo-episodes"
    config["cameras"]["wrist_rgb"].update(enabled=True, serial="SYNTHETIC", hand_eye_calibrated=False)
    model = RobotModel(resolve_path(config["robot"]["urdf"]))
    control = {"mode": "simulation", "backend": "synthetic", "actuate": False,
               "gripper_width_m": .082, "control_rate_hz": 50}
    session = TeleopSession(config, model.metadata(), control, camera_factory=SyntheticCamera)
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    old = signal.signal(signal.SIGTERM, interrupted)
    try:
        session.start()
        print("Synthetic session running. No CAN/Quest/RealSense opened; data goes to outputs/demo-episodes.", flush=True)
        start = time.monotonic()
        while seconds == 0 or time.monotonic() - start < seconds:
            now = time.monotonic()
            q = np.array([0., 1.5, 1.45, -1.2, 0., 0.])
            q[0] = .1 * np.sin(now - start)
            session.publish({"actual_pos": q.tolist(), "actual_vel": [.1 * np.cos(now - start), 0., 0., 0., 0., 0.],
                "actual_torque": [0.] * 6, "sdk_cmd_pos": q.tolist(), "sdk_cmd_vel": [0.] * 6,
                "sdk_cmd_torque": [0.] * 6, "kp": [0.] * 6, "kd": [0.] * 6,
                "gripper_actual_pos": .04, "gripper_actual_vel": 0., "gripper_actual_torque": 0.,
                "gripper_mode": "position", "feedback_timestamp": now, "command_timestamp": now,
                "controller_timestamp": now, "requested_pos": None, "requested_gripper_pos": None,
                "requested_monotonic_ns": None, "submission": None, "actuate": False,
                "source": "hold", "preset_name": None})
            time.sleep(max(0., now + .02 - time.monotonic()))
    finally:
        session.close()
        signal.signal(signal.SIGTERM, old)
