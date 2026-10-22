"""Synthetic APKs and mocked network/ADB only; no devices or downloads."""
import importlib.util
import json
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import Mock, patch
import zipfile

spec = importlib.util.spec_from_file_location("quest_client", Path(__file__).resolve().parents[1] / "scripts/quest/client.py")
client = importlib.util.module_from_spec(spec)
spec.loader.exec_module(client)


def manifest(package, *, utf8=True, attr_padding=0):
    values = ["manifest", "package", package]
    offsets, strings = [], b""
    for value in values:
        offsets.append(len(strings))
        if utf8:
            payload = value.encode()
            strings += bytes([len(value), len(payload)]) + payload + b"\0"
        else:
            strings += struct.pack("<H", len(value)) + value.encode("utf-16-le") + b"\0\0"
    strings += b"\0" * (-len(strings) % 4)
    start = 28 + len(offsets) * 4
    pool = struct.pack("<HHI5I", 1, 28, start + len(strings), len(values), 0, 0x100 if utf8 else 0, start, 0)
    pool += struct.pack("<3I", *offsets) + strings
    ext = struct.pack("<II6H", 0xffffffff, 0, 20 + attr_padding, 20, 1, 0, 0, 0)
    attr = struct.pack("<IIIHBBI", 0xffffffff, 1, 2, 8, 0, 3, 2)
    root = struct.pack("<HHIII", 0x102, 16, 56 + attr_padding, 1, 0xffffffff) + ext + b"\0" * attr_padding + attr
    close = struct.pack("<HHIIIII", 0x103, 16, 24, 1, 0xffffffff, 0xffffffff, 0)
    payload = pool + root + close
    return struct.pack("<HHI", 3, 8, 8 + len(payload)) + payload


class DistributionTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        replacement = patch.object(client, "ROOT", self.root)
        replacement.start()
        self.addCleanup(replacement.stop)
        self.apk = self.root / "built.apk"
        self.make_apk(client.PACKAGE)

    def make_apk(self, package):
        with zipfile.ZipFile(self.apk, "w") as output:
            output.writestr("AndroidManifest.xml", manifest(package))
        self.record = {"status": "built; not installed", "package": client.PACKAGE,
                       "revision": "a" * 40, "source_manifest_sha256": "b" * 64,
                       "apk_sha256": client.digest(self.apk)}
        self.apk.with_suffix(".build.json").write_text(json.dumps(self.record))

    def release(self):
        metadata = client.make_release_metadata(version="quest-v1", source_revision="a" * 40,
                                                project_commit="c" * 40, sha256=client.digest(self.apk),
                                                apk_size=self.apk.stat().st_size)
        metadata_bytes = (json.dumps(metadata, indent=2) + "\n").encode()
        import hashlib
        sums = f"{metadata['apk']['sha256']}  {client.APK_NAME}\n{hashlib.sha256(metadata_bytes).hexdigest()}  {client.METADATA_NAME}\n"
        release = {"tag_name": "quest-v1", "draft": False,
                   "assets": [{"name": name} for name in (client.APK_NAME, client.METADATA_NAME, client.CHECKSUM_NAME)]}
        return {"latest": json.dumps(release).encode(), "quest-v1": json.dumps(release).encode(),
                client.METADATA_NAME: metadata_bytes, client.CHECKSUM_NAME: sums.encode(),
                client.APK_NAME: self.apk.read_bytes()}

    def downloader(self, files):
        def download(url, destination, limit):
            self.assertTrue(client.allowed_url(url))
            contents = files[url.rsplit("/", 1)[-1]]
            self.assertLessEqual(len(contents), limit)
            Path(destination).write_bytes(contents)
        return download

    def test_reads_real_binary_manifest_encodings_and_attribute_offset(self):
        for utf8 in (True, False):
            for padding in (0, 8):
                self.assertEqual(client.binary_manifest_package(manifest(client.PACKAGE, utf8=utf8, attr_padding=padding)), client.PACKAGE)

    def test_rejects_truncated_manifest_and_fake_official_identity(self):
        with self.assertRaises(ValueError):
            client.binary_manifest_package(manifest(client.PACKAGE)[:-1])
        self.make_apk("com.xrobotoolkit.client.quest")
        with self.assertRaisesRegex(ValueError, "内部包名"):
            client.verify_local(self.apk)

    def test_metadata_requires_complete_revision_and_exact_package(self):
        with self.assertRaises(ValueError):
            client.make_release_metadata(version="quest-v1", source_revision="abc", project_commit="c" * 40,
                                         sha256="d" * 64, apk_size=100)
        with self.assertRaises(ValueError):
            client.make_release_metadata(version="../../escape", source_revision="a" * 40, project_commit="c" * 40,
                                         sha256="d" * 64, apk_size=100)

    def test_package_has_verified_three_assets_and_never_overwrites(self):
        output = self.root / "outputs/releases/quest-v1"
        client.package_release(self.apk, version="quest-v1", project_commit="c" * 40, output_dir=output)
        self.assertEqual({item.name for item in output.iterdir()}, {client.APK_NAME, client.METADATA_NAME, client.CHECKSUM_NAME})
        metadata = json.loads((output / client.METADATA_NAME).read_text())
        self.assertEqual(metadata["project_commit"], "c" * 40)
        self.assertEqual(metadata["apk"]["sha256"], client.digest(output / client.APK_NAME))
        with self.assertRaisesRegex(ValueError, "已存在"):
            client.package_release(self.apk, version="quest-v1", project_commit="c" * 40, output_dir=output)

    def test_local_install_refuses_missing_or_mismatching_build_record_before_adb(self):
        self.apk.with_suffix(".build.json").unlink()
        with patch.object(client.subprocess, "run") as run:
            with self.assertRaises(SystemExit):
                client.main(["install", "--apk", str(self.apk)])
        run.assert_not_called()

    def test_remote_release_checks_metadata_checksum_and_internal_package(self):
        files = self.release()
        with patch.object(client, "download", side_effect=self.downloader(files)):
            cached = client.download_release()
        self.assertTrue(cached.is_relative_to(self.root / "outputs"))
        self.assertEqual(cached.read_bytes(), self.apk.read_bytes())
        files[client.APK_NAME] = b"corrupted"
        with patch.object(client, "download", side_effect=self.downloader(files)):
            with self.assertRaisesRegex(ValueError, "大小不匹配"):
                client.download_release()
        self.assertEqual(cached.read_bytes(), self.apk.read_bytes())
        self.assertFalse(list(cached.parent.parent.glob(".download-*")))

    def test_wrong_metadata_checksum_leaves_no_apk_cache(self):
        files = self.release()
        files[client.METADATA_NAME] += b" "
        with patch.object(client, "download", side_effect=self.downloader(files)):
            with self.assertRaisesRegex(ValueError, "来源记录 SHA256"):
                client.download_release()
        self.assertFalse(list((self.root / "outputs").rglob("*.apk")))

    def test_missing_release_does_not_fallback_or_call_adb(self):
        with patch.object(client, "download", side_effect=RuntimeError("尚未发布可访问的 Release")), \
             patch.object(client.subprocess, "run") as run:
            with self.assertRaisesRegex(RuntimeError, "尚未发布"):
                client.download_release()
        run.assert_not_called()

    def test_download_sources_stay_on_https_github(self):
        for url in ("http://github.com/2-mo/ThoughtIntoAction/releases/download/v/a.apk",
                    "https://github.com/other/repo/releases/download/v/a.apk", "https://evil.test/a.apk",
                    "https://github.com@evil.test/a.apk"):
            self.assertFalse(client.allowed_url(url))
        with self.assertRaises(ValueError):
            client.download("https://evil.test/a.apk", self.root / "bad.apk", 100)

    def test_private_release_download_uses_authenticated_gh_on_fixed_host(self):
        destination = self.root / "release.json"

        def gh(command, **kwargs):
            self.assertEqual(command, ["gh", "api", "--hostname", "github.com", "repos/2-mo/ThoughtIntoAction/releases/latest"])
            self.assertEqual(kwargs["env"]["GH_HOST"], "github.com")
            kwargs["stdout"].write(b"{}")
            return Mock(returncode=0, stderr=b"")

        with patch.object(client.subprocess, "run", side_effect=gh):
            client.download("https://api.github.com/repos/2-mo/ThoughtIntoAction/releases/latest", destination, 100)
        self.assertEqual(destination.read_bytes(), b"{}")
        with patch.object(client.subprocess, "run", return_value=Mock(returncode=1, stderr=b"HTTP 404")):
            with self.assertRaisesRegex(RuntimeError, "仓库权限"):
                client.download("https://api.github.com/repos/2-mo/ThoughtIntoAction/releases/latest", destination, 100)

    def test_only_one_authorized_usb_is_selected(self):
        self.assertEqual(client.authorized_usb("Q device usb:3-1\n192.168.1.2:5555 device product:Quest"), "Q")
        for text in ("Q unauthorized transport_id:1", "Q device product:Quest", "A device usb:1\nB device usb:2"):
            with self.assertRaises(RuntimeError):
                client.authorized_usb(text)

    def test_install_uses_only_scoped_adb_install_and_requires_success(self):
        results = [Mock(stdout="Q device usb:3-1\n", stderr=""), Mock(stdout="Success\n", stderr="")]
        with patch.object(client.subprocess, "run", side_effect=results) as run:
            client.main(["install", "--apk", str(self.apk)])
        self.assertEqual([call.args[0] for call in run.call_args_list],
                         [["adb", "devices", "-l"], ["adb", "-s", "Q", "install", "-r", str(self.apk)]])
        results[1] = Mock(stdout="Failure [INSTALL_FAILED_UPDATE_INCOMPATIBLE]\n", stderr="")
        with patch.object(client.subprocess, "run", side_effect=results):
            with self.assertRaisesRegex(RuntimeError, "未确认安装成功"):
                client.install(self.apk)


if __name__ == "__main__":
    unittest.main()
