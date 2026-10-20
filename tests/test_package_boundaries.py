"""Shared robotics must work without the Quest application or legacy R5 imports."""

import os
from pathlib import Path
import subprocess
import sys
import textwrap
import unittest


ROOT = Path(__file__).resolve().parents[1]
GUARD = """
import importlib.abc
import sys

class ForbiddenImport(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if any(fullname == name or fullname.startswith(name + '.') for name in forbidden):
            raise AssertionError('Unexpected dependency: ' + fullname)

sys.meta_path.insert(0, ForbiddenImport())
"""


class PackageBoundaryTests(unittest.TestCase):
    def test_collection_and_storage_without_control_or_quest(self):
        self.probe(
            {'legacy.quest_teleop', 'legacy.robot_control', 'arx5_vr', 'iceoryx2', 'arx5_interface',
             'xrobotoolkit_python', 'pyrealsense2'},
            '''
            from embodied.data.episode.recording import EpisodeWriter
            from embodied.data.episode.mcap_io import RecordWriter
            from embodied.data.capture.capture import capture
            from embodied.data.export.runtime_v2 import load_episode
            from embodied.contracts.runtime import RuntimeReader
            from embodied.data.alignment.runtime_alignment import align
            ''',
        )

    def probe(self, forbidden, source):
        env = {**os.environ, "PYTHONPATH": os.pathsep.join(map(str, (
            ROOT / "src", ROOT / "third_party/XRoboToolkit-Teleop-Sample-Python")))}
        result = subprocess.run(
            [sys.executable, "-B", "-c",
             f"forbidden = {forbidden!r}\n" + GUARD + textwrap.dedent(source)],
            env=env, cwd="/tmp", capture_output=True, text=True, timeout=30,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_offline_calibration_and_simulation_without_quest_or_device_sdks(self):
        self.probe(
            {"legacy.quest_teleop", "myTeleop", "project_paths", "xrobotoolkit_teleop",
             "xrobotoolkit_python", "arx_r5_python", "arx5_interface", "pyrealsense2"},
            '''
            import json
            from pathlib import Path
            import tempfile

            from embodied.core.model import RobotModel
            from embodied.sensors.realsense import RealSenseCamera
            from legacy.robot_control.motion import move_and_settle
            from embodied.calibration.wrist.arm import CalibrationArm
            from embodied.calibration.wrist.session import Session
            from embodied.calibration.wrist.cli import main as handeye
            from legacy.robot_control.cli import main as simulation
            from unittest.mock import patch

            # Terminal input must remain usable with all Quest/device imports blocked.
            with patch("builtins.input", return_value="exit"):
                simulation(["--headless"])
            # Even a Quest configuration can be validated without starting its SDK.
            simulation(["--input", "quest", "--headless"])

            with tempfile.TemporaryDirectory() as directory:
                plan = Path(directory) / "plan.json"
                handeye(["plan", "--center", "0", "1.0", "1.0", "-0.9", "0", "0",
                         "--output", str(plan)])
                handeye(["auto", "--plan", str(plan)])
                simulation(["--headless", "--replay", str(plan)])
                assert json.loads(plan.read_text())["robot"] == RobotModel().metadata()
            ''',
        )

    def test_monitor_never_imports_a_controller_or_device_sdk(self):
        self.probe(
            {"legacy.quest_teleop", "legacy.robot_control", "arx5_interface", "xrobotoolkit_python",
             "xrobotoolkit_teleop", "pyrealsense2"},
            '''
            from unittest.mock import patch
            from embodied.monitor.cli import main
            with patch("embodied.monitor.cli.StateSubscriber.receive", side_effect=KeyboardInterrupt):
                main(["--mode", "real", "--port", "0"])
            # The MuJoCo workbench modules load without starting a runtime.
            from embodied.monitor.arx_preview import Simulation
            from embodied.monitor.session import SimulationSession
            ''',
        )

    def test_service_client_and_quest_entry_do_not_import_hardware_sdk(self):
        self.probe(
            {"arx5_interface", "arx_r5_python", "bimanual", "pyrealsense2"},
            '''
            from legacy.robot_control.client import ArmClient
            from legacy.quest_teleop.cli import presets_main
            import inspect
            options = inspect.signature(presets_main).parameters
            assert {"service_socket", "preview_time_s", "max_command_lead_rad", "joint_soft_limit_margin_rad"} <= options.keys()
            assert not {"backend", "can_port", "telemetry"} & options.keys()
            ''',
        )

    def test_shared_x5_backend_imports_without_quest_or_r5(self):
        self.probe(
            {"legacy.quest_teleop", "myTeleop", "project_paths", "xrobotoolkit_teleop",
             "xrobotoolkit_python", "arx_r5_python"},
            '''
            from unittest.mock import patch
            import arx5_interface as arx5

            with patch.object(arx5, "Arx5JointController",
                              side_effect=AssertionError("Import opened CAN")) as constructor:
                from legacy.robot_control.hardware import OfficialX5HardwareAdapter
                from legacy.robot_control.trajectory import bounded_joint_target, trajectory
                constructor.assert_not_called()
            ''',
        )


if __name__ == "__main__":
    unittest.main()
