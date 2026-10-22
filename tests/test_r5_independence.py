"""X5 startup, source checks and builds work without the optional R5 SDK."""

from contextlib import redirect_stdout
import importlib.util
import io
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from scripts.launch import build_command
from embodied.core.project import load_config

ROOT = Path(__file__).resolve().parents[1]


def script(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / "scripts/setup" / (name + ".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class R5IndependenceTests(unittest.TestCase):
    def test_x5_entries_and_cli_help_without_r5(self):
        command, env = build_command("teleop-presets-x5", [], load_config())
        env["QUEST_TEST_ROOT"] = str(ROOT)
        self.assertFalse(any("/R5/" in path for path in env["LD_LIBRARY_PATH"].split(os.pathsep)))
        source = r'''
import importlib
import importlib.abc
import os
from pathlib import Path
import runpy
import sys
from unittest.mock import patch

forbidden = ("myTeleop", "project_config", "project_paths", "scripts",
             "arx_r5_python", "embodied.devices.inputs.quest._vendor.hardware.arx_r5_teleop_controller",
             "embodied.devices.inputs.quest._vendor.hardware.interface.arx_r5")
class NoR5(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if any(fullname == prefix or fullname.startswith(prefix + ".") for prefix in forbidden):
            raise AssertionError("Unexpected R5 import: " + fullname)
sys.meta_path.insert(0, NoR5())

import arx5_interface as arx5
import tyro
with patch.object(arx5, "Arx5JointController", side_effect=AssertionError("Opened CAN")) as hardware, \
     patch("embodied.devices.inputs.quest._vendor.common.base_teleop_controller.XrClient", side_effect=AssertionError("Opened XR")) as xr:
    entries = [(name, getattr(importlib.import_module(name), function)) for name, function in (
        ("legacy.quest_teleop.cli", "baseline_main"), ("legacy.quest_teleop.cli", "presets_main"),
        ("legacy.quest_teleop.legacy.controller", "main"))]
    diagnostic = Path(os.environ["QUEST_TEST_ROOT"]) / "scripts/diagnostics/joint_tracking.py"
    entries.append(("joint-tracking", runpy.run_path(str(diagnostic))["main"]))
    for name, main in entries:
        try:
            tyro.cli(main, args=["--help"])
        except SystemExit as exc:
            assert exc.code == 0, (name, exc.code)
        else:
            raise AssertionError(name + " did not exit after --help")
    hardware.assert_not_called()
    xr.assert_not_called()
assert not any(name.startswith(forbidden) for name in sys.modules)
maps = Path("/proc/self/maps").read_text()
assert "/dependencies/R5/" not in maps and "libarx_r5_src" not in maps
'''
        result = subprocess.run([command[0], "-B", "-c", source], env=env, cwd="/tmp",
                                capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_default_source_check_skips_absent_optional_repositories(self):
        dependencies = script("dependencies")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "dependencies.lock.toml").write_bytes((ROOT / "dependencies.lock.toml").read_bytes())
            import tomllib
            entries = tomllib.loads((root / "dependencies.lock.toml").read_text())["managed"]
            required = [entry for entry in entries if entry["required"]]
            self.assertTrue(required)
            self.assertFalse(any(entry["name"] == "R5" for entry in required))
            revisions = {}
            for entry in required:
                repo = root / entry["path"]
                (repo / ".git").mkdir(parents=True)
                revisions[str(repo)] = entry["commit"]
            def git(arguments, **kwargs):
                self.assertIn(arguments[2], revisions)
                return revisions[arguments[2]] if "rev-parse" in arguments else ""
            with patch.object(dependencies, "ROOT", root), patch("sys.argv", ["dependencies.py"]), \
                 patch.object(dependencies.subprocess, "check_output", side_effect=git), redirect_stdout(io.StringIO()):
                self.assertEqual(dependencies.main(), 0)

    def test_native_rebuild_installs_pinned_companion_library_without_reference_paths(self):
        installer = script("rebuild_xr_sdk")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "dependencies.lock.toml").write_bytes((ROOT / "dependencies.lock.toml").read_bytes())
            from embodied.core.dependencies import dependency_root
            native = dependency_root("xr-sdk", root=root) / "lib/libPXREARobotSDK.so"
            native.parent.mkdir(parents=True)
            native.write_bytes(b"pinned-native-library")
            with patch.object(installer, "ROOT", root), patch("sys.prefix", str(root / ".venv")), \
                 patch.dict(os.environ, {"UV_PROJECT_ENVIRONMENT": "/another/project/.venv"}), \
                 patch("sys.argv", ["rebuild_xr_sdk.py"]), \
                 patch.object(installer.platform, "machine", return_value="x86_64"), \
                 patch.object(installer.shutil, "which", return_value="/usr/bin/uv"), \
                 patch.object(installer.subprocess, "run") as run:
                installer.main()
            calls = [call.args[0] for call in run.call_args_list]
            self.assertEqual(calls[0][-3:], ["--fetch", "--name", "xr-sdk"])
            self.assertEqual(calls[1], ["/usr/bin/uv", "sync", "--locked", "--managed-python", "--no-cache",
                                        "--reinstall-package", "xrobotoolkit-sdk"])
            self.assertNotIn("UV_PROJECT_ENVIRONMENT", run.call_args.kwargs["env"])
            self.assertFalse(any("third_party" in token for args in calls for token in args))
            self.assertEqual((root / ".venv/lib/libPXREARobotSDK.so").read_bytes(), native.read_bytes())
            self.assertIn("import xrobotoolkit_sdk", calls[-1])


if __name__ == "__main__":
    unittest.main()
