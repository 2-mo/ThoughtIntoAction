import hashlib
import json
from pathlib import Path
import tempfile
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
            with self.assertRaises(FileExistsError):
                finalize_task(root / ".raw", ["episode"], root / "pick_place", 240)
