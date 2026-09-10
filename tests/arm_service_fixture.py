"""In-process service with real local RPC and a virtual device, never a native SDK."""

import tempfile
from pathlib import Path
import threading
import time
from unittest.mock import Mock

from robot_core.model import RobotModel
from robot_core.project import load_config
from robot_control.backends.factories import simulated
from robot_control.service import ArmService, serve


class ServiceFixture:
    def __init__(self, hardware=False, table_safety=False):
        self.folder = tempfile.TemporaryDirectory(prefix="x5-service-test-")
        self.path = Path(self.folder.name) / "arm.sock"
        self.config = Path(self.folder.name) / "config.toml"
        self.config.write_text(f'[arm_service]\nsocket="{self.path}"\n')
        self.robot = RobotModel()
        settings = {key: value for key, value in load_config()["arm_service"].items()
                    if key not in {"backend", "socket"}}
        self.arm = simulated(settings={**settings, "dynamics_urdf_path": str(self.robot.urdf)},
                             initial_q=[0, 1.5, 1.45, -1.52, 0, 0])
        def recording():
            q, dq = self.arm.get_joint_positions(), self.arm.get_joint_velocities()
            return {"actual_pos": q[:6].tolist(), "actual_vel": dq[:6].tolist(),
                    "gripper_actual_pos": float(q[6]), "gripper_actual_vel": 0.,
                    "feedback_timestamp": time.monotonic()}
        self.arm.get_recording_snapshot = recording
        self.publisher = Mock()
        self.service = ArmService(self.robot, self.arm, self.publisher, backend="simulated",
                                  hardware=hardware, settings=settings,
                                  safety={"table_enabled": table_safety, "table_z_m": 0., "table_clearance_m": 0.})
        self.ready, self.stop = threading.Event(), threading.Event()
        self.errors = []
        def run():
            try:
                serve(self.service, self.path, ready=self.ready, stop=self.stop)
            except BaseException as exc:
                self.errors.append(exc)
            finally:
                self.service.release()
        self.worker = threading.Thread(target=run, daemon=True)
        self.worker.start()
        if not self.ready.wait(3):
            self.close()
            raise RuntimeError(f"Fixture startup failed: {self.errors}")

    def close(self):
        self.stop.set()
        self.worker.join(3)
        self.arm.close()
        self.folder.cleanup()
        assert not self.worker.is_alive(), "Service failed to stop"
        assert not self.errors, self.errors
