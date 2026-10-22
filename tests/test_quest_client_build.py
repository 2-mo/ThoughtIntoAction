"""Small local Git fixtures and mocked Unity; no downloads, ADB, or hardware."""
import copy
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location(
    "quest_client_build", Path(__file__).resolve().parents[1] / "scripts/setup/build_quest_client.py")
client = importlib.util.module_from_spec(spec)
spec.loader.exec_module(client)


class QuestClientBuildTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.project = self.root / "outputs/build/quest/source"
        self.repo = self.root / "upstream"
        self.repo.mkdir()
        self.patches = self.root / "patches"
        shutil.copytree(client.PATCHES, self.patches)
        # Only tiny synthetic upstream files enter the fixture repository.
        original = {
            client.UI_PATH: "class UIOperate {\n    void Awake() {\n        OVRPlugin.systemDisplayFrequency = 90.0f;\n    }\n}\n",
            client.TCP_PATH: "// upstream TCP fixture\n",
            "ProjectSettings/ProjectVersion.txt": "m_EditorVersion: 2021.3.45f2\n",
            "Packages/manifest.json": '{"dependencies":{"com.unity.shadergraph":"14.0.9"}}\n',
            "LICENSE": "MIT fixture\n",
        }
        for name, text in original.items():
            target = self.repo / name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(text)
        (self.patches / "TcpHandler.patch").write_text(
            f"diff --git a/{client.TCP_PATH} b/{client.TCP_PATH}\n"
            f"--- a/{client.TCP_PATH}\n+++ b/{client.TCP_PATH}\n"
            "@@ -1 +1,2 @@\n // upstream TCP fixture\n+// retry fixture\n")
        subprocess.run(["git", "init", "-q", str(self.repo)], check=True)
        self.git("add", ".")
        self.git("-c", "user.name=Test", "-c", "user.email=test@example.invalid", "commit", "-qm", "fixture")
        revision = self.git("rev-parse", "HEAD").strip()
        upstream = copy.deepcopy(client.UPSTREAM)
        upstream["revision"] = revision
        upstream["audited_blobs"] = {name: client.git_blob(text.encode()) for name, text in original.items()}
        for name, value in {
            "ROOT": self.root, "PATCHES": self.patches, "UPSTREAM": upstream,
            "REVISION": revision, "CACHE": self.root / "outputs/dependencies/cache.git",
        }.items():
            handle = patch.object(client, name, value)
            handle.start()
            self.addCleanup(handle.stop)

    def git(self, *args):
        return subprocess.check_output(["git", "-C", str(self.repo), *args], text=True)

    def prepared(self):
        return client.prepare(self.project, self.repo)

    def unity(self):
        executable = self.root / "Unity"
        executable.write_text("not executed")
        executable.chmod(0o755)
        return executable

    def test_prepare_exports_pinned_commit_and_preserves_checkout(self):
        (self.repo / client.UI_PATH).write_text("local developer edit\n")
        self.prepared()
        self.assertIn("AddComponent<QuestUsbConnection>()", (self.project / client.UI_PATH).read_text())
        self.assertIn("retry fixture", (self.project / client.TCP_PATH).read_text())
        self.assertEqual((self.repo / client.UI_PATH).read_text(), "local developer edit\n")
        manifest = client.validate_project(self.project)
        self.assertEqual(manifest["status"], "prepared; not built or installed")
        self.assertEqual(manifest["package"], "com.thoughtintoaction.quest")
        self.assertFalse(list(self.root.rglob("*.apk")))

    def test_missing_cache_requires_explicit_fetch(self):
        with patch.object(client, "run") as run:
            with self.assertRaisesRegex(RuntimeError, "prepare --fetch"):
                client.prepare(self.project)
        run.assert_not_called()

    def test_existing_project_is_never_overwritten(self):
        self.project.mkdir(parents=True)
        marker = self.project / "user-work"
        marker.write_text("keep")
        with self.assertRaisesRegex(RuntimeError, "不覆盖"):
            self.prepared()
        self.assertEqual(marker.read_text(), "keep")

    def test_bad_upstream_identity_leaves_no_partial_project(self):
        client.UPSTREAM["audited_blobs"][client.TCP_PATH] = "0" * 40
        with self.assertRaisesRegex(RuntimeError, "源码.*不一致"):
            self.prepared()
        self.assertFalse(self.project.exists())

    def test_source_export_cannot_escape_outputs(self):
        with self.assertRaisesRegex(RuntimeError, "outputs/build"):
            client.prepare(self.root / "src/upstream", self.repo)

    def test_changed_source_stops_before_launching_unity(self):
        self.prepared()
        (self.project / client.TCP_PATH).write_text("changed")
        with patch.object(client, "run") as run:
            with self.assertRaisesRegex(RuntimeError, "受管文件已改变"):
                client.build(self.project, self.unity(), self.root / "client.apk")
        run.assert_not_called()

    def test_changed_patch_requires_new_preparation(self):
        self.prepared()
        (self.patches / "TcpHandler.patch").write_text("new patch")
        with self.assertRaisesRegex(RuntimeError, "补丁已更新"):
            client.validate_project(self.project)

    def test_unity_success_without_an_apk_is_not_reported_as_built(self):
        self.prepared()
        output = self.root / "client.apk"
        with patch.object(client, "run"):
            with self.assertRaisesRegex(RuntimeError, "未生成 APK"):
                client.build(self.project, self.unity(), output)
        self.assertFalse(output.with_suffix(".build.json").exists())

    def test_build_runs_exact_entry_and_records_uninstalled_artifact(self):
        self.prepared()
        output = self.root / "client.apk"
        unity = self.unity()

        def mock_unity(command, **kwargs):
            self.assertEqual(command[0], unity)
            self.assertIn("QuestUsbBuild.Build", command)
            self.assertIn("-nographics", command)
            self.assertEqual(kwargs["env"]["TIA_QUEST_APK"], str(output))
            output.write_bytes(b"mock APK")

        with patch.object(client, "run", side_effect=mock_unity) as run:
            self.assertEqual(client.build(self.project, unity, output), output)
        self.assertEqual(run.call_count, 1)
        record = json.loads(output.with_suffix(".build.json").read_text())
        self.assertEqual(record["status"], "built; not installed")
        self.assertEqual(record["apk_sha256"], client.sha256(output))
        with self.assertRaisesRegex(RuntimeError, "APK 已存在"):
            client.build(self.project, unity, output)


if __name__ == "__main__":
    unittest.main()
