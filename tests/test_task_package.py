import gzip
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
import zipfile

from quest_teleop.compressed_io import compress_jsonl, open_jsonl
from quest_teleop.task_package import finalize_task


class TaskPackageTests(unittest.TestCase):
    def test_lossless_archive_contains_real_files_and_exact_nanoseconds(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / ".raw" / "episode"
            source.mkdir(parents=True)
            content = b'{"host_monotonic_ns":4552089789885210}\n'
            for name in ("states.jsonl", "frames.jsonl"):
                (source / name).write_bytes(content)
            (source / "episode.json").write_text(json.dumps({
                "lifecycle": "complete", "outcome": "discarded", "task": "pick place",
                "task_run_id": "run", "counts": {"state": 1, "frame": 1}}))
            (source / "wrist_rgb.mp4").write_bytes(b"unchanged encoded RGB")
            target = finalize_task(root / ".raw", ["episode"], root / "pick_place", 240)
            self.assertFalse((source / "states.jsonl").exists())
            with open_jsonl(source / "states.jsonl") as stream:
                self.assertEqual(stream.read().encode(), content)
            with zipfile.ZipFile(target) as package:
                manifest = json.loads(package.read("pick_place/manifest.json"))
                self.assertEqual(manifest["episodes"][0]["outcome"], "discarded")
                self.assertIn("tools/convert.sh", "\n".join(package.namelist()))
                for relative, info in manifest["files"].items():
                    actual = package.read("pick_place/" + relative)
                    self.assertEqual(hashlib.sha256(actual).hexdigest(), info["sha256"])
                self.assertEqual(gzip.decompress(package.read(
                    "pick_place/episodes/episode/states.jsonl.gz")), content)
                self.assertEqual(package.read("pick_place/episodes/episode/wrist_rgb.mp4"),
                                 b"unchanged encoded RGB")
            with self.assertRaises(FileExistsError):
                finalize_task(root / ".raw", ["episode"], root / "pick_place", 240)

    def test_compression_is_repeatable_and_legacy_reader_still_works(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "states.jsonl"
            path.write_text('{"timestamp":123}\n')
            with open_jsonl(path) as stream:
                self.assertEqual(json.loads(stream.readline())["timestamp"], 123)
            first = compress_jsonl(path)
            self.assertEqual(compress_jsonl(path), first)
