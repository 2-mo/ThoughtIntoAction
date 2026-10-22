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
    def test_robot_api_and_pose_math_without_hardware_or_input_sdks(self):
        self.probe(
            {'legacy', 'embodied.control', 'arx5_interface', 'pyrealsense2', 'xrobotoolkit_python',
             'embodied.devices', 'pinocchio', 'mujoco', 'ruckig'},
            '''
            from embodied import Robot
            from embodied.motion.contracts import Pose
            assert Pose.from_rotvec([0, 0, 0], [0, 0, 0]).orientation_xyzw.tolist() == [0, 0, 0, 1]
            robot = Robot.connect('/not-started')
            assert robot.run.name == 'not-started'
            ''',
        )

    def test_project_paths_are_device_free_and_do_not_import_toml_parsers(self):
        self.probe(
            {"tomllib", "tomli", "embodied.core.dependencies", "embodied.devices",
             "legacy", "arx5_interface", "pinocchio", "mujoco"},
            '''
            from embodied.core.project import PROJECT_ROOT, LOG_DIR, resolve_path
            assert resolve_path("outputs") == PROJECT_ROOT / "outputs"
            assert LOG_DIR.is_absolute()
            ''',
        )

    def test_device_paths_do_not_load_a_sdk_or_dependency_manifest(self):
        self.probe(
            {"tomllib", "tomli", "embodied.core.dependencies", "arx5_interface", "pinocchio", "mujoco"},
            '''
            from embodied.devices.robots.arx5.paths import X5_2025_URDF
            assert X5_2025_URDF.is_file()
            ''',
        )

    def test_control_configuration_import_has_no_device_or_toml_dependency(self):
        self.probe(
            {"tomllib", "tomli", "embodied.devices", "embodied.applications", "legacy", "arx5_interface"},
            '''
            from embodied.control.settings import Settings, load_settings
            from embodied.contracts.paths import CONTROL_CONFIG
            assert CONTROL_CONFIG.is_dir()
            ''',
        )

    def test_collection_and_storage_without_control_or_quest(self):
        self.probe(
            {'legacy.quest_teleop', 'legacy.robot_control', 'embodied.control', 'iceoryx2', 'arx5_interface',
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
        env = {**os.environ, "PYTHONPATH": str(ROOT / "src")}
        result = subprocess.run(
            [sys.executable, "-B", "-c",
             f"forbidden = {forbidden!r}\n" + GUARD + textwrap.dedent(source)],
            env=env, cwd="/tmp", capture_output=True, text=True, timeout=30,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_offline_calibration_and_simulation_without_quest_or_device_sdks(self):
        self.probe(
            {"legacy.quest_teleop", "myTeleop", "project_paths", "xrobotoolkit_teleop", "embodied.devices.inputs.quest", "xrobotoolkit_sdk",
             "xrobotoolkit_python", "arx_r5_python", "arx5_interface", "pyrealsense2"},
            '''
            import json
            from pathlib import Path
            import tempfile

            from embodied.devices.robots.arx5.pinocchio_model import X5PinocchioModel
            from embodied.devices.sensors.realsense import RealSenseCamera
            from legacy.robot_control.motion import move_and_settle
            from embodied.applications.calibration.arm import CalibrationArm
            from embodied.applications.calibration.session import Session
            from embodied.applications.calibration.plan import kernel_model, make_plan
            from embodied.applications.calibration.cli import main as handeye
            from legacy.robot_control.cli import main as simulation
            from unittest.mock import patch

            # Terminal input must remain usable with all Quest/device imports blocked.
            with patch("builtins.input", return_value="exit"):
                simulation(["--headless"])
            # Even a Quest configuration can be validated without starting its SDK.
            simulation(["--input", "quest", "--headless"])

            with tempfile.TemporaryDirectory() as directory:
                handeye(["marker", "--size-m", "0.1", "--output", str(Path(directory) / "marker.svg")])
                plan = make_plan(kernel_model(), [0, 1.0, 1.0, -0.9, 0, 0])
                assert plan["robot"] == kernel_model().metadata() and len(plan["waypoints"]) == 26
            ''',
        )

    def test_monitor_never_imports_a_controller_or_device_sdk(self):
        self.probe(
            {"legacy.quest_teleop", "legacy.robot_control", "arx5_interface", "xrobotoolkit_python",
             "xrobotoolkit_teleop", "embodied.devices.inputs.quest", "xrobotoolkit_sdk", "pyrealsense2"},
            '''
            from unittest.mock import patch
            from embodied.applications.monitor.cli import main
            with patch("embodied.applications.monitor.cli.StateSubscriber.receive", side_effect=KeyboardInterrupt):
                main(["--mode", "real", "--port", "0"])
            # The MuJoCo workbench modules load without starting a runtime.
            from embodied.applications.monitor.arx_preview import Simulation
            from embodied.applications.monitor.session import SimulationSession
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

    def test_x5_device_backend_imports_without_quest_or_r5(self):
        self.probe(
            {"legacy.quest_teleop", "myTeleop", "project_paths", "xrobotoolkit_teleop", "embodied.devices.inputs.quest", "xrobotoolkit_sdk",
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
