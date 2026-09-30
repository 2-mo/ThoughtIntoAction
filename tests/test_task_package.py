import hashlib
import json
import os
from pathlib import Path
import tempfile
import subprocess
import sys
import unittest
import zipfile

from mcap_fixture import write_records
from quest_teleop.mcap_io import read_records
from quest_teleop.task_package import finalize_task


class TaskPackageTests(unittest.TestCase):
    def test_lossless_archive_contains_real_files_and_exact_nanoseconds(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / ".raw" / "episode"
            source.mkdir(parents=True)
            (source / "episode.json").write_text(json.dumps({
                "lifecycle": "complete", "outcome": "discarded", "task": "pick place",
                "task_run_id": "run", "counts": {"state": 1, "frame": 1}}))
            write_records(source, states=[{"host_monotonic_ns":4552089789885210}],
                          frames=[{"host_received_monotonic_ns":4552089789885211}])
            content = (source / "episode.mcap").read_bytes()
            (source / "wrist_rgb.mp4").write_bytes(b"unchanged encoded RGB")
            target = finalize_task(root / ".raw", ["episode"], root / "pick_place", 240)
            self.assertEqual(read_records(source / "episode.mcap")[0][0]["host_monotonic_ns"], 4552089789885210)
            with zipfile.ZipFile(target) as package:
                manifest = json.loads(package.read("pick_place/manifest.json"))
                self.assertEqual(manifest["episodes"][0]["outcome"], "discarded")
                self.assertIn("tools/convert.sh", "\n".join(package.namelist()))
                for relative, info in manifest["files"].items():
                    actual = package.read("pick_place/" + relative)
                    self.assertEqual(hashlib.sha256(actual).hexdigest(), info["sha256"])
                self.assertEqual(package.read("pick_place/episodes/episode/episode.mcap"), content)
                self.assertEqual(package.read("pick_place/episodes/episode/wrist_rgb.mp4"),
                                 b"unchanged encoded RGB")
                package.extractall(root / 'unpacked')
            bundle = root / 'unpacked/pick_place'
            probe = subprocess.run([sys.executable, '-B', '-c',
                'from pathlib import Path; from episode_io.mcap_io import read_records; '
                'from quest_teleop.export_lerobot import load_episode; '
                'from quest_teleop.export_decode import BatchFrameReader; '
                'assert len(read_records(Path("episodes/episode/episode.mcap"))[0]) == 1'],
                cwd=bundle, env={**os.environ, 'PYTHONPATH': str(bundle / 'tools/src')},
                capture_output=True, text=True, timeout=20)
            self.assertEqual(probe.returncode, 0, probe.stderr)
            meta_path = source / 'episode.json'
            meta = json.loads(meta_path.read_text())
            meta['schema_version'] = 99
            meta_path.write_text(json.dumps(meta))
            with self.assertRaisesRegex(ValueError, 'Unsupported/mixed'):
                finalize_task(root / '.raw', ['episode'], root / 'wrong_converter', 250)
            with self.assertRaises(FileExistsError):
                finalize_task(root / ".raw", ["episode"], root / "pick_place", 240)
