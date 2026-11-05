"""Collection data listing: directory guard and episode summarization."""
import asyncio
import json
import tempfile
import unittest
from pathlib import Path

from fastapi import HTTPException

from embodied.applications.console.core.settings import BackendSettings
from embodied.applications.console.transport.collection import create_collection_router
from embodied.contracts.paths import PROJECT_ROOT


class EpisodeRouteTests(unittest.TestCase):
    def setUp(self):
        router = create_collection_router(BackendSettings())
        self.route = next(
            route for route in router.routes if route.path == "/api/v1/collection/episodes"
        )
        self.root = PROJECT_ROOT / "outputs/datasets"
        self.root.mkdir(parents=True, exist_ok=True)

    def test_rejects_directories_outside_datasets(self):
        for value in ("/tmp", "src/embodied"):
            with self.assertRaises(HTTPException) as raised:
                asyncio.run(self.route.endpoint(directory=value, limit=3, token=None))
            self.assertEqual(raised.exception.status_code, 400)

    def test_lists_episodes_with_summary_fields(self):
        with tempfile.TemporaryDirectory(dir=self.root) as folder:
            episode = Path(folder) / "episode_20260101-120000-abcdef12"
            episode.mkdir()
            (episode / "episode.json").write_text(json.dumps({
                "task": "把方块放到棋盘上",
                "lifecycle": "complete",
                "counts": {"state": 900, "frame": 120},
                "cameras": {"wrist_rgb": {}},
            }))
            result = asyncio.run(
                self.route.endpoint(directory=str(Path(folder)), limit=3, token=None)
            )
        self.assertEqual(result["total"], 1)
        summary = result["episodes"][0]
        self.assertEqual(summary["task"], "把方块放到棋盘上")
        self.assertEqual(summary["lifecycle"], "complete")
        self.assertEqual(summary["ticks"], 900)
        self.assertEqual(summary["frames"], 120)
        self.assertEqual(summary["cameras"], ["wrist_rgb"])

    def test_missing_directory_returns_an_empty_listing(self):
        result = asyncio.run(
            self.route.endpoint(directory="outputs/datasets/does-not-exist", limit=3, token=None)
        )
        self.assertEqual(result["episodes"], [])
        self.assertEqual(result["total"], 0)


class DeleteEpisodeTests(unittest.TestCase):
    def setUp(self):
        router = create_collection_router(BackendSettings())
        self.route = next(
            route for route in router.routes if route.path == "/api/v1/collection/episodes/{name}"
        )
        self.root = PROJECT_ROOT / "outputs/datasets"
        self.root.mkdir(parents=True, exist_ok=True)

    def make_episode(self, folder):
        episode = Path(folder) / "episode_20260101-120000-abcdef12"
        episode.mkdir()
        (episode / "episode.json").write_text(json.dumps({"task": "x", "lifecycle": "complete"}))
        return episode

    def test_deletes_only_an_episode_folder(self):
        with tempfile.TemporaryDirectory(dir=self.root) as folder:
            episode = self.make_episode(folder)
            other = Path(folder) / "keep-me"
            other.mkdir()
            result = asyncio.run(
                self.route.endpoint(name=episode.name, directory=str(Path(folder)), token=None)
            )
            self.assertEqual(result["deleted"], episode.name)
            self.assertFalse(episode.exists())
            self.assertTrue(other.exists())

    def test_rejects_names_that_escape_the_directory(self):
        with tempfile.TemporaryDirectory(dir=self.root) as folder:
            self.make_episode(folder)
            for name in ("..", ".", "../runtime", "a/b"):
                with self.assertRaises(HTTPException) as raised:
                    asyncio.run(
                        self.route.endpoint(name=name, directory=str(Path(folder)), token=None)
                    )
                self.assertEqual(raised.exception.status_code, 400)

    def test_rejects_folders_outside_datasets_or_without_episode_json(self):
        with tempfile.TemporaryDirectory(dir=self.root) as folder:
            plain = Path(folder) / "plain-folder"
            plain.mkdir()
            with self.assertRaises(HTTPException) as missing:
                asyncio.run(
                    self.route.endpoint(name=plain.name, directory=str(Path(folder)), token=None)
                )
            self.assertEqual(missing.exception.status_code, 404)
            with self.assertRaises(HTTPException) as outside:
                asyncio.run(self.route.endpoint(name="runtime", directory="/tmp", token=None))
            self.assertEqual(outside.exception.status_code, 400)


if __name__ == "__main__":
    unittest.main()
