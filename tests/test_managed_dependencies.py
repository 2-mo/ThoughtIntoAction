"""Dependency isolation without installing native SDKs, model packages or opening devices."""
from pathlib import Path
import hashlib
import importlib.util
import json
import tempfile
import tomllib
import unittest
from unittest.mock import patch

from embodied.core.dependencies import dependency, dependency_root, dependency_python, reject_reference_path

ROOT = Path(__file__).resolve().parents[1]


class ManagedDependencyTests(unittest.TestCase):
    def test_reference_checkout_cannot_be_a_runtime_or_install_source(self):
        with self.assertRaisesRegex(ValueError, "Reference checkout"):
            reject_reference_path(ROOT / "third_party/Isaac-GR00T")
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / "dependencies.lock.toml").write_text('''[[managed]]
name = "bad"
commit = "0123456789012345678901234567890123456789"
path = "third_party/model"
''')
            with self.assertRaisesRegex(ValueError, "Reference checkout"):
                dependency_root("bad", root=root)

    def test_installations_are_pinned_and_interpreters_separate_from_sources(self):
        for name in ("xr-sdk", "x5-official", "groot", "openpi"):
            with self.subTest(name=name):
                record = dependency(name)
                self.assertEqual(len(record["commit"]), 40)
                self.assertTrue(dependency_root(name).is_relative_to(ROOT / "outputs/dependencies"))
        self.assertEqual(dependency_python("groot"), ROOT / "outputs/envs/groot/bin/python")
        with self.assertRaisesRegex(ValueError, "Unknown managed dependency"):
            dependency("reference-only")

    def test_xr_git_metadata_matches_the_native_library_revision(self):
        project = tomllib.loads((ROOT / "pyproject.toml").read_text())
        lock = tomllib.loads((ROOT / "uv.lock").read_text())
        source = project["tool"]["uv"]["sources"]["xrobotoolkit-sdk"]
        self.assertEqual(source["rev"], dependency("xr-sdk")["commit"])
        self.assertEqual(source["git"], dependency("xr-sdk")["url"])
        package = next(p for p in lock["package"] if p["name"] == "xrobotoolkit-sdk")
        self.assertEqual(package["source"]["git"], source["git"] + "?rev=" + source["rev"] + "#" + source["rev"])

    def test_quest_subset_is_owned_and_matches_its_provenance(self):
        folder = ROOT / "src/embodied/devices/inputs/quest/_vendor"
        manifest = json.loads((folder / "PROVENANCE.json").read_text())
        self.assertEqual(manifest["license"], "MIT")
        self.assertEqual(len(manifest["files"]), 9)
        self.assertIn("XR Robotics", (folder / "LICENSE").read_text())
        for record in manifest["files"]:
            with self.subTest(module=record["path"]):
                path = folder / record["path"]
                self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), record["sha256"])
                self.assertNotIn("from xrobotoolkit_teleop", path.read_text())

    def test_groot_setup_installs_non_editable_package_in_declared_environment(self):
        path = ROOT / "scripts/setup/groot_environment.py"
        spec = importlib.util.spec_from_file_location("groot_environment", path)
        installer = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(installer)
        with patch("sys.argv", [str(path), "--tensorrt"]), \
             patch.object(Path, "is_file", return_value=True), \
             patch.object(installer.subprocess, "run") as run:
            installer.main()
        args = run.call_args.args[0]
        self.assertIn("--no-editable", args)
        self.assertEqual(args[-2:], ["--extra", "tensorrt"])
        self.assertEqual(run.call_args.kwargs["env"]["UV_PROJECT_ENVIRONMENT"], str(ROOT / "outputs/envs/groot"))
        self.assertNotIn("third_party", str(run.call_args))


if __name__ == "__main__":
    unittest.main()
