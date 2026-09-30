"""Isolated control launch and import provenance; never start a real backend."""

import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from workstation.control import ROOT, control_environment, verify_sources


class ControlEntryTests(unittest.TestCase):
    def test_legacy_and_arx_lease_exclude_each_other_across_processes(self):
        from robot_core.device_lease import DeviceLease
        import uuid
        device = 'offline-lease-' + uuid.uuid4().hex
        lease = DeviceLease(device)
        self.addCleanup(lease.close)
        source = ('from adapters.device_lease import DeviceLease; '
                  'DeviceLease(' + repr(device) + ')')
        env = {**os.environ, 'PYTHONPATH': str(ROOT / 'control')}
        python = str(ROOT / 'control/.venv/bin/python')
        if not Path(python).is_file():
            self.skipTest('Control environment not installed')
        blocked = subprocess.run([python, '-B', '-c', source], env=env, capture_output=True, text=True)
        self.assertNotEqual(blocked.returncode, 0)
        self.assertIn('owned by another', blocked.stderr)
        lease.close()
        released = subprocess.run([python, '-B', '-c', source], env=env, capture_output=True, text=True)
        self.assertEqual(released.returncode, 0, released.stderr)

    def test_source_import_is_accounted_for(self):
        self.assertEqual(verify_sources()["status"], "verified")

    def test_provenance_detects_edits_and_external_links(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "model.py"
            source.write_text("original")
            digest = hashlib.sha256(source.read_bytes()).hexdigest()
            (root / "UPSTREAM_FILES.json").write_text(json.dumps({"model.py": {"sha256": digest, "executable": False}}))
            (root / "LOCAL_PATCHES.json").write_text("{}")
            self.assertEqual(verify_sources(root)["files"], 1)
            source.write_text("changed")
            with self.assertRaisesRegex(ValueError, "Unrecorded change"):
                verify_sources(root)
            source.unlink()
            source.symlink_to("/etc/hosts")
            with self.assertRaisesRegex(ValueError, "linked source"):
                verify_sources(root)

    def test_control_removes_foreign_environment_paths(self):
        names = ("PYTHONPATH", "PYTHONHOME", "VIRTUAL_ENV", "LD_LIBRARY_PATH", "UV_PROJECT_ENVIRONMENT")
        with patch.dict(os.environ, dict.fromkeys(names, "/foreign/python312")):
            env = control_environment()
        self.assertFalse(set(names) & env.keys())
        self.assertEqual(env["OPENBLAS_NUM_THREADS"], "1")

    def test_verify_works_without_legacy_environment_or_sdk(self):
        env = {**os.environ, "QUEST_TELEOP_PYTHON": "/missing/python", "PYTHONDONTWRITEBYTECODE": "1"}
        result = subprocess.run([str(ROOT / "run.sh"), "control", "verify"], cwd="/tmp",
                                env=env, capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout)["status"], "verified")

    def test_demo_never_accepts_a_real_backend(self):
        from workstation.control import command_for
        from argparse import Namespace
        with patch("workstation.control.Path.is_file", return_value=True):
            with self.assertRaisesRegex(ValueError, "simulation only"):
                command_for(Namespace(command="start", backend="real", input="demo"))


if __name__ == "__main__":
    unittest.main()
