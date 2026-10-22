"""Verify and install the Thought into Action Quest APK; never launch robot control."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess
import tempfile
from urllib.parse import quote, urlsplit
import zipfile

ROOT = Path(__file__).resolve().parents[2]
REPOSITORY = "2-mo/ThoughtIntoAction"
PACKAGE = "com.thoughtintoaction.quest"
APK_NAME = "ThoughtIntoAction-Quest-USB.apk"
METADATA_NAME = "quest-client-release.json"
CHECKSUM_NAME = "SHA256SUMS"
MAX_APK_SIZE = 512 * 1024 * 1024


def digest(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def valid_hex(value, length):
    return isinstance(value, str) and re.fullmatch(r"[0-9a-f]{%d}" % length, value) is not None


def valid_version(version):
    if not isinstance(version, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]{0,95}", version):
        raise ValueError("版本标签须为字母、数字、点、下划线或连字符")
    return version


def make_release_metadata(*, version, source_revision, project_commit, sha256, apk_size, package=PACKAGE):
    """Pure metadata constructor; package_release additionally verifies actual APK contents."""
    valid_version(version)
    if package != PACKAGE or not valid_hex(source_revision, 40) or not valid_hex(project_commit, 40):
        raise ValueError("发布记录必须使用新版包名与两个完整 40 位源码提交")
    if not valid_hex(sha256, 64) or type(apk_size) is not int or not 0 < apk_size <= MAX_APK_SIZE:
        raise ValueError("无效的 APK SHA256 或文件大小")
    return {"schema": 1, "version": version, "source_revision": source_revision,
            "project_commit": project_commit, "package": package,
            "apk": {"filename": APK_NAME, "size": apk_size, "sha256": sha256}}


def binary_manifest_package(data):
    """Read the root package attribute from Android's binary XML, without an SDK."""
    def u16(offset):
        return struct.unpack_from("<H", data, offset)[0]

    def u32(offset):
        return struct.unpack_from("<I", data, offset)[0]

    if len(data) < 8 or u16(0) != 3 or u16(2) != 8 or u32(4) != len(data):
        raise ValueError("APK 的 AndroidManifest.xml 不是完整的 Android binary XML")
    pool = None

    def string(index):
        if pool is None:
            raise ValueError("Android manifest 缺少字符串表")
        start, header, end, count, flags, strings_start, strings_end = pool
        if index >= count:
            raise ValueError("Android manifest 字符串索引越界")
        offset = strings_start + u32(start + header + 4 * index)

        def length(position, width):
            if position + width > strings_end:
                raise ValueError("Android manifest 字符串长度越界")
            value = data[position] if width == 1 else u16(position)
            marker = 0x80 if width == 1 else 0x8000
            if value & marker:
                if position + 2 * width > strings_end:
                    raise ValueError("Android manifest 字符串长度截断")
                following = data[position + width] if width == 1 else u16(position + width)
                return ((value & (marker - 1)) << (8 * width)) | following, position + 2 * width
            return value, position + width

        chars, offset = length(offset, 1 if flags & 0x100 else 2)
        if flags & 0x100:
            size, offset = length(offset, 1)
            end = offset + size
            if end >= strings_end or data[end] != 0:
                raise ValueError("Android manifest UTF-8 字符串截断")
            value = data[offset:end].decode("utf-8")
        else:
            end = offset + chars * 2
            if end + 2 > strings_end or data[end:end + 2] != b"\0\0":
                raise ValueError("Android manifest UTF-16 字符串截断")
            value = data[offset:end].decode("utf-16-le")
        if len(value.encode("utf-16-le")) // 2 != chars:
            raise ValueError("Android manifest 字符串长度不一致")
        return value

    offset = 8
    while offset < len(data):
        if offset + 8 > len(data):
            raise ValueError("Android manifest chunk 截断")
        kind, header, size = struct.unpack_from("<HHI", data, offset)
        end = offset + size
        if header < 8 or size < header or end > len(data):
            raise ValueError("Android manifest chunk 大小无效")
        if kind == 1:
            if pool is not None or header < 28:
                raise ValueError("Android manifest 字符串表无效")
            count, styles, flags, strings_start, styles_start = struct.unpack_from("<IIIII", data, offset + 8)
            if header + 4 * (count + styles) > strings_start or strings_start > size:
                raise ValueError("Android manifest 字符串表越界")
            if styles_start and not strings_start <= styles_start <= size:
                raise ValueError("Android manifest 样式表越界")
            pool = (offset, header, end, count, flags, offset + strings_start,
                    offset + styles_start if styles_start else end)
        elif kind == 0x102:
            ext = offset + header
            if (header < 16 or ext + 20 > end or u32(ext) != 0xffffffff
                    or string(u32(ext + 4)) != "manifest"):
                raise ValueError("Android manifest 根元素无效")
            attr_start, attr_size, count = struct.unpack_from("<HHH", data, ext + 8)
            if attr_start < 20 or attr_size < 20 or ext + attr_start + attr_size * count > end:
                raise ValueError("Android manifest 属性表越界")
            packages = []
            for index in range(count):
                attr = ext + attr_start + index * attr_size
                if u32(attr) == 0xffffffff and string(u32(attr + 4)) == "package":
                    if u16(attr + 12) != 8 or data[attr + 14] != 0 or data[attr + 15] != 3:
                        raise ValueError("Android manifest 包名不是字符串")
                    value = string(u32(attr + 16))
                    if u32(attr + 8) != 0xffffffff and string(u32(attr + 8)) != value:
                        raise ValueError("Android manifest 包名属性不一致")
                    packages.append(value)
            if len(packages) != 1:
                raise ValueError("Android manifest 必须包含唯一包名")
            return packages[0]
        offset = end
    raise ValueError("Android manifest 缺少根元素")


def apk_package(apk):
    with zipfile.ZipFile(apk) as archive:
        manifests = [entry for entry in archive.infolist() if entry.filename == "AndroidManifest.xml"]
        if len(manifests) != 1 or not 0 < manifests[0].file_size <= 4 * 1024 * 1024:
            raise ValueError("APK 必须包含唯一且有效的 AndroidManifest.xml")
        return binary_manifest_package(archive.read(manifests[0]))


def verify_apk(apk, sha256, size=None):
    apk = Path(apk)
    actual_size = apk.stat().st_size
    if not 0 < actual_size <= MAX_APK_SIZE or (size is not None and actual_size != size):
        raise ValueError("APK 文件大小不匹配")
    if not valid_hex(sha256, 64) or digest(apk) != sha256:
        raise ValueError("APK SHA256 校验失败")
    if apk_package(apk) != PACKAGE:
        raise ValueError("APK 内部包名不是 com.thoughtintoaction.quest；拒绝安装或发布官方旧版")


def verify_local(apk, build_record=None):
    apk = Path(apk).resolve()
    record_path = Path(build_record) if build_record else apk.with_suffix(".build.json")
    if not record_path.is_file():
        raise ValueError(f"缺少本地构建记录：{record_path}；不能将下载的官方 APK 当作新版")
    record = json.loads(record_path.read_text())
    if (not isinstance(record, dict) or record.get("status") != "built; not installed" or record.get("package") != PACKAGE
            or not valid_hex(record.get("revision"), 40)
            or not valid_hex(record.get("source_manifest_sha256"), 64)):
        raise ValueError("本地构建记录不完整或不是新版客户端")
    verify_apk(apk, record.get("apk_sha256"))
    return record


def package_release(apk, *, version, project_commit, output_dir, build_record=None):
    apk = Path(apk).resolve()
    record = verify_local(apk, build_record)
    metadata = make_release_metadata(version=version, source_revision=record["revision"],
                                     project_commit=project_commit, sha256=record["apk_sha256"],
                                     apk_size=apk.stat().st_size)
    output_dir = Path(output_dir).resolve()
    if not output_dir.is_relative_to((ROOT / "outputs").resolve()):
        raise ValueError("发布文件只能生成到本仓库 outputs 下")
    if output_dir.exists():
        raise ValueError(f"发布目录已存在，不覆盖：{output_dir}")
    output_dir.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".quest-release-", dir=output_dir.parent) as staging:
        source = Path(staging) / "release"
        source.mkdir()
        shutil.copyfile(apk, source / APK_NAME)
        (source / METADATA_NAME).write_text(json.dumps(metadata, indent=2) + "\n")
        (source / CHECKSUM_NAME).write_text("".join(
            f"{digest(source / name)}  {name}\n" for name in (APK_NAME, METADATA_NAME)))
        verify_apk(source / APK_NAME, metadata["apk"]["sha256"], metadata["apk"]["size"])
        source.rename(output_dir)
    return output_dir


def allowed_url(url):
    parsed = urlsplit(url)
    if parsed.scheme != "https" or parsed.username or parsed.password or parsed.port not in (None, 443):
        return False
    if parsed.hostname == "api.github.com":
        return parsed.path.startswith(f"/repos/{REPOSITORY}/releases/")
    if parsed.hostname == "github.com":
        return parsed.path.startswith(f"/{REPOSITORY}/releases/download/")
    return False


def download(url, destination, limit):
    """gh handles private repository authentication and GitHub asset redirects."""
    if not allowed_url(url):
        raise ValueError("只允许受信 GitHub Release HTTPS 来源")
    parsed = urlsplit(url)
    env = {**os.environ, "GH_HOST": "github.com", "GH_PROMPT_DISABLED": "1"}
    try:
        if parsed.hostname == "api.github.com":
            with Path(destination).open("wb") as stream:
                result = subprocess.run(["gh", "api", "--hostname", "github.com", parsed.path.lstrip("/")],
                                        stdout=stream, stderr=subprocess.PIPE, env=env, timeout=30)
        else:
            tag, name = parsed.path.rsplit("/", 2)[-2:]
            valid_version(tag)
            if name not in {APK_NAME, METADATA_NAME, CHECKSUM_NAME}:
                raise ValueError("非预期的 Release 附件名")
            result = subprocess.run(["gh", "release", "download", tag, "--repo", "github.com/" + REPOSITORY,
                                     "--pattern", name, "--output", str(destination)],
                                    stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, env=env, timeout=180)
    except FileNotFoundError as error:
        raise RuntimeError("私有仓库下载需要 GitHub CLI；安装 gh 后执行 gh auth login --hostname github.com") from error
    if result.returncode:
        detail = result.stderr.decode(errors="replace").strip()
        if "404" in detail or "release not found" in detail.lower():
            raise RuntimeError(f"{REPOSITORY} 尚未发布可访问的新客户端 Release，或当前 gh 登录账号没有仓库权限")
        raise RuntimeError("GitHub 下载失败；检查 gh auth status 与私有仓库权限：" + detail)
    if Path(destination).stat().st_size > limit:
        raise ValueError("下载文件超过预期大小")


def download_release(version=None):
    if version:
        valid_version(version)
    endpoint = "tags/" + quote(version, safe="") if version else "latest"
    cache = ROOT / "outputs/downloads/quest-client"
    cache.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".download-", dir=cache) as staging:
        staging = Path(staging)
        download(f"https://api.github.com/repos/{REPOSITORY}/releases/{endpoint}", staging / "release.json", 2 * 1024 * 1024)
        release = json.loads((staging / "release.json").read_text())
        if not isinstance(release, dict):
            raise ValueError("GitHub Release 响应格式无效")
        tag = valid_version(release.get("tag_name"))
        if release.get("draft") or (version and tag != version):
            raise ValueError("Release 版本不匹配或尚未发布")
        names = {asset.get("name") for asset in release.get("assets", [])}
        if not {APK_NAME, METADATA_NAME, CHECKSUM_NAME} <= names:
            raise RuntimeError(f"Release {tag} 尚无完整新版 APK、来源记录和 SHA256；不会安装官方旧版")
        base = f"https://github.com/{REPOSITORY}/releases/download/{quote(tag, safe='')}/"
        for name in (METADATA_NAME, CHECKSUM_NAME):
            download(base + name, staging / name, 64 * 1024)
        sums = {}
        for line in (staging / CHECKSUM_NAME).read_text().splitlines():
            match = re.fullmatch(r"([0-9a-f]{64})  ([A-Za-z0-9._-]+)", line)
            if not match or match[2] in sums:
                raise ValueError("SHA256SUMS 格式无效")
            sums[match[2]] = match[1]
        if sums.get(METADATA_NAME) != digest(staging / METADATA_NAME):
            raise ValueError("Release 来源记录 SHA256 校验失败")
        metadata = json.loads((staging / METADATA_NAME).read_text())
        if not isinstance(metadata, dict) or not isinstance(metadata.get("apk"), dict):
            raise ValueError("Release 元数据格式无效")
        details = metadata.get("apk", {})
        expected = make_release_metadata(version=metadata.get("version"), source_revision=metadata.get("source_revision"),
                                         project_commit=metadata.get("project_commit"), sha256=details.get("sha256"),
                                         apk_size=details.get("size"), package=metadata.get("package"))
        if metadata != expected or metadata["version"] != tag or sums.get(APK_NAME) != details["sha256"]:
            raise ValueError("Release 元数据与版本或 SHA256 不一致")
        download(base + APK_NAME, staging / APK_NAME, details["size"])
        verify_apk(staging / APK_NAME, details["sha256"], details["size"])
        target = cache / (tag + "-" + details["sha256"][:16])
        target.mkdir(exist_ok=True)
        # Only fully verified artifacts enter the cache; each replacement is atomic.
        for name in (METADATA_NAME, CHECKSUM_NAME, APK_NAME):
            os.replace(staging / name, target / name)
        return target / APK_NAME


def authorized_usb(output):
    devices = [line.split() for line in output.splitlines()]
    ready = [fields[0] for fields in devices if len(fields) >= 3 and fields[1] == "device"
             and any(field.startswith("usb:") for field in fields[2:])]
    if len(ready) != 1:
        raise RuntimeError("需要唯一一台已授权的 USB Quest；请连接数据线并在头显内允许 USB 调试")
    return ready[0]


def install(apk):
    result = subprocess.run(["adb", "devices", "-l"], capture_output=True, text=True, check=True, timeout=10)
    serial = authorized_usb(result.stdout)
    result = subprocess.run(["adb", "-s", serial, "install", "-r", str(apk)],
                            capture_output=True, text=True, check=True, timeout=180)
    if "Success" not in result.stdout.splitlines():
        raise RuntimeError("ADB 未确认安装成功：" + (result.stderr or result.stdout).strip())
    print(f"新版客户端已安装：{PACKAGE}；未启动应用或机器人。")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="action", required=True)
    installer = commands.add_parser("install", help="安装已校验的本地或 GitHub Release 新版 APK")
    source = installer.add_mutually_exclusive_group()
    source.add_argument("--version", help="GitHub Release 标签；默认 latest")
    source.add_argument("--apk", type=Path, help="本地已构建 APK，需配套 .build.json")
    installer.add_argument("--build-record", type=Path)
    packager = commands.add_parser("package", help="生成可发布的 APK、来源元数据与 SHA256SUMS，不上传")
    packager.add_argument("--apk", required=True, type=Path)
    packager.add_argument("--build-record", type=Path)
    packager.add_argument("--version", required=True)
    packager.add_argument("--project-commit", required=True, help="本项目源码的完整 40 位 Git commit")
    packager.add_argument("--output-dir", type=Path)
    args = parser.parse_args(argv)
    try:
        if args.action == "package":
            version = valid_version(args.version)
            output = args.output_dir or ROOT / "outputs/releases/quest-client" / version
            print(package_release(args.apk, version=version, project_commit=args.project_commit,
                                  output_dir=output, build_record=args.build_record))
        else:
            if args.build_record and not args.apk:
                raise ValueError("--build-record 仅用于 --apk 本地安装")
            if args.apk:
                verify_local(args.apk, args.build_record)
                apk = args.apk.resolve()
            else:
                apk = download_release(args.version)
            install(apk)
    except (OSError, ValueError, RuntimeError, KeyError, TypeError, struct.error,
            zipfile.BadZipFile, subprocess.SubprocessError) as error:
        detail = error.stderr.strip() if isinstance(error, subprocess.CalledProcessError) and error.stderr else str(error)
        parser.exit(1, detail + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
