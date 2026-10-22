"""Ratchet for the platform refactor (docs/plans/00-guardrails.md, G-04).

Each check lists the files that still contain a pattern being removed. New files
must not join a list, and a file that no longer matches must be dropped from
convergence_baseline.json, so the lists only shrink.
"""

import ast
import json
from pathlib import Path
import re
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
BASELINE = Path(__file__).with_name("convergence_baseline.json")
SOURCE_ROOTS = ("src", "scripts", "apps/console/src", "docs/examples")
SKIPPED_DIRS = {".venv", "__pycache__", "node_modules", "vendor", "native", "run", "build", "dist", "models"}
CODE_SUFFIXES = {".py", ".sh", ".ts", ".js", ".mjs", ".html"}
# A shared numerical planner, not a dependency on platform applications. Keep
# this exact: other motion modules may introduce optional model dependencies.
CONTROL_SHARED_MODULES = {"embodied.motion.offline", "embodied.motion.validation"}
# These standalone installers must bootstrap before a project environment exists.
# An independent test below restricts each to the single repository src directory.
INSTALLER_BOOTSTRAPS = {
    "scripts/setup/dependencies.py",
    "scripts/setup/groot_dependency.py",
    "scripts/setup/groot_environment.py",
    "scripts/setup/rebuild_xr_sdk.py",
}

TEXT_CHECKS = {  # name: (suffixes, pattern, files that are the intended single source)
    # Target: the ARX device adapter owns the SDK; legacy paths are ratcheted away.
    "sdk_controller": ({".py"}, re.compile(r"Arx5JointController\("), set()),
    # Target: none after packaging (todo B-01).
    "sys_path": ({".py"}, re.compile(r"sys\.path\.(insert|append)\("), INSTALLER_BOOTSTRAPS),
    # Target: none besides the upstream arx-televr default; others read the robot profile (todo R-02).
    "can0_literal": (CODE_SUFFIXES, re.compile(r"""(["'=])can0\b"""), {"src/embodied/devices/robots/arx5/profile.py"}),
    # Target: none; machine paths come from configs/local.toml (todo R-05).
    "home_path": (CODE_SUFFIXES, re.compile(r"/home/[A-Za-z0-9_]+/"), set()),
}


def source_files():
    for name in SOURCE_ROOTS:
        for path in sorted((ROOT / name).rglob("*")):
            relative = path.relative_to(ROOT)
            if path.is_file() and path.suffix in CODE_SUFFIXES and not SKIPPED_DIRS & set(relative.parts[:-1]):
                yield relative.as_posix(), path


def control_imports_of_src(files):
    """Control depends on contracts and shared math; concrete devices are resolved by Registry."""
    packages = {path.name for path in (ROOT / "src").iterdir() if (path / "__init__.py").is_file()}
    found = set()
    for relative, path in files:
        if not relative.startswith("src/embodied/control/") or path.suffix != ".py":
            continue
        for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"), relative)):
            if isinstance(node, ast.Import):
                names = [alias.name for alias in node.names]
            elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
                # Count `from embodied import applications` as the dependency
                # it requests, and support the explicit shared-planner spelling.
                names = ([f"{node.module}.{alias.name}" for alias in node.names]
                         if node.module in packages or node.module == "embodied.motion"
                         else [node.module])
            else:
                continue
            for name in names:
                allowed = (name == "embodied.contracts" or name.startswith("embodied.contracts.")
                           or name == "embodied.control" or name.startswith("embodied.control.")
                           or name in CONTROL_SHARED_MODULES)
                if name.split(".")[0] in packages and not allowed:
                    found.add(f"{relative}:{'.'.join(name.split('.')[:2])}")
    return found


def current():
    files = list(source_files())
    result = {name: set() for name in TEXT_CHECKS}
    for relative, path in files:
        text = path.read_text(encoding="utf-8", errors="replace")
        for name, (suffixes, pattern, exempt) in TEXT_CHECKS.items():
            if path.suffix in suffixes and relative not in exempt and pattern.search(text):
                result[name].add(relative)
    result["control_imports_src"] = control_imports_of_src(files)
    return result


class ConvergenceTests(unittest.TestCase):
    def test_control_shared_import_boundary(self):
        cases = {
            "from embodied.contracts.runtime import ControlClient": False,
            "from embodied.control.ipc import RobotBridge": False,
            "from embodied.motion.validation import finite_vector": False,
            "from embodied import contracts": False,
            "from embodied.motion.offline import plan_ruckig_joint_trajectory": False,
            "import embodied.motion.offline as planner": False,
            "from embodied.motion import offline": False,
            "from embodied.motion import PinocchioKinematics": True,
            "from embodied.motion import *": True,
            "import embodied.motion.pinocchio_backend": True,
            "from embodied.applications.operations.robot import Robot": True,
            "from embodied import applications": True,
            "import embodied.devices.sensors": True,
            "from embodied.devices.robots.arx5.kinematics import X5Model": True,
            "from embodied.core.project import load_config": True,
            "import embodied.data.capture": True,
            "import embodied.policy": True,
            "from legacy.robot_control import hardware": True,
        }
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "adapter.py"
            for statement, forbidden in cases.items():
                with self.subTest(statement=statement):
                    source.write_text(statement + "\n", encoding="utf-8")
                    self.assertEqual(bool(control_imports_of_src([
                        ("src/embodied/control/worker.py", source)])), forbidden)

    def test_launcher_cannot_import_concrete_devices_or_workflows(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "app.py"
            for statement, forbidden in (
                ("from embodied.devices.inputs.quest.composition import native_components", True),
                ("from embodied.devices.robots.arx5.real import RealRobot", True),
                ("import embodied.devices.sensors", True),
                ("from embodied.applications.operations.robot import Robot", True),
            ):
                with self.subTest(statement=statement):
                    source.write_text(statement + "\n")
                    self.assertEqual(bool(control_imports_of_src([
                        ("src/embodied/control/app.py", source)])), forbidden)

    def test_standalone_installers_only_bootstrap_the_repository_source(self):
        expected = ast.dump(ast.parse('sys.path.insert(0, str(ROOT / "src"))').body[0].value)
        for relative in sorted(INSTALLER_BOOTSTRAPS):
            with self.subTest(installer=relative):
                tree = ast.parse((ROOT / relative).read_text())
                calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)
                         and isinstance(node.func, ast.Attribute)
                         and isinstance(node.func.value, ast.Attribute)
                         and isinstance(node.func.value.value, ast.Name)
                         and node.func.value.value.id == "sys" and node.func.value.attr == "path"]
                self.assertEqual([ast.dump(call) for call in calls], [expected])

    def test_lists_only_shrink(self):
        baseline = json.loads(BASELINE.read_text(encoding="utf-8"))
        found = current()
        self.assertEqual(set(baseline), set(found), "checks in convergence_baseline.json")
        for name in sorted(found):
            with self.subTest(check=name):
                expected = set(baseline[name])
                added, removed = sorted(found[name] - expected), sorted(expected - found[name])
                self.assertFalse(added, f"{name}: new occurrences; remove them instead of adding to the baseline")
                self.assertFalse(removed, f"{name}: cleaned up; delete these entries from convergence_baseline.json")


if __name__ == "__main__":
    unittest.main()
