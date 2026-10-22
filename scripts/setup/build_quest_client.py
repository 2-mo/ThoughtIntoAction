"""Prepare the pinned Quest USB client, then explicitly build it with Unity.

Only preparation with --fetch downloads sources. No command installs an APK,
starts a headset app, changes ADB mappings, or touches robot control.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile

ROOT = Path(__file__).resolve().parents[2]
PATCHES = Path(__file__).with_name("quest-client")
UPSTREAM = json.loads((PATCHES / "upstream.json").read_text())
REVISION = UPSTREAM["revision"]
BUILD = ROOT / "outputs/build/quest-client"
CACHE = ROOT / "outputs/dependencies/quest-client.git"
MARKER = "thought-into-action-source.json"
UI_PATH = "Assets/Scripts/UI/UIOperate.cs"
TCP_PATH = "Assets/Scripts/Network/TcpHandler.cs"
ADDITIONS = {
    "QuestUsbConnection.cs": "Assets/Scripts/QuestUsbConnection.cs",
    "QuestUsbBuild.cs": "Assets/Editor/QuestUsbBuild.cs",
    "LICENSE.upstream": "LICENSE.ThoughtIntoAction-upstream",
    "upstream.json": "ThoughtIntoAction-upstream.json",
}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def git_blob(data: bytes) -> str:
    return hashlib.sha1(b"blob " + str(len(data)).encode() + b"\0" + data).hexdigest()


def run(args, **kwargs):
    return subprocess.run(list(map(str, args)), check=True, **kwargs)


def patch_source(project: Path) -> dict[str, str]:
    """Fail closed on unreviewed upstream; never patch a developer checkout."""
    for name, expected in UPSTREAM["audited_blobs"].items():
        if git_blob((project / name).read_bytes()) != expected:
            raise RuntimeError(f"官方源码与已核对版本不一致：{name}")
    ui = project / UI_PATH
    text = ui.read_text()
    anchor = "        OVRPlugin.systemDisplayFrequency = 90.0f;"
    if text.count(anchor) != 1:
        raise RuntimeError("UIOperate 启动位置发生变化，停止应用补丁")
    # git apply also works outside a repository. Prevent parent repo discovery.
    env = {key: value for key, value in os.environ.items()
           if key not in {"GIT_DIR", "GIT_WORK_TREE", "GIT_INDEX_FILE"}}
    env["GIT_CEILING_DIRECTORIES"] = str(project.parent)
    patch = PATCHES / "TcpHandler.patch"
    run(["git", "apply", "--check", patch], cwd=project, env=env)
    run(["git", "apply", patch], cwd=project, env=env)
    ui.write_text(text.replace(anchor, anchor + "\n        gameObject.AddComponent<QuestUsbConnection>();"))
    for source, destination in ADDITIONS.items():
        target = project / destination
        if target.exists():
            raise RuntimeError(f"补丁目标已存在：{destination}")
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(PATCHES / source, target)
    files = {*UPSTREAM["audited_blobs"], *ADDITIONS.values()}
    return {name: sha256(project / name) for name in sorted(files)}


def managed_location(project: Path) -> Path:
    project = project.resolve()
    if not project.is_relative_to((ROOT / "outputs/build").resolve()):
        raise RuntimeError("Unity 项目必须位于本仓库 outputs/build 下")
    return project


def prepare(project: Path, source_checkout: Path | None = None, fetch: bool = False) -> Path:
    project = managed_location(project)
    if project.exists():
        raise RuntimeError(f"目录已存在，不覆盖现有工程：{project}；可指定新的 --project-dir")
    repo = source_checkout.resolve() if source_checkout else CACHE
    if not repo.exists():
        if source_checkout or not fetch:
            raise RuntimeError("没有官方源码缓存；使用 prepare --fetch，或提供 --source-checkout")
        repo.parent.mkdir(parents=True, exist_ok=True)
        run(["git", "init", "--bare", repo], stdout=subprocess.DEVNULL)
    result = subprocess.run(["git", "-C", str(repo), "cat-file", "-e", REVISION + "^{commit}"],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if result.returncode:
        if source_checkout or not fetch:
            raise RuntimeError(f"源码缓存中没有固定版本 {REVISION}；使用 prepare --fetch")
        run(["git", "-C", repo, "fetch", "--depth", "1", UPSTREAM["repository"], REVISION])
    project.parent.mkdir(parents=True, exist_ok=True)
    # Export to staging; failed validation leaves no half-patched project behind.
    with tempfile.TemporaryDirectory(prefix=".quest-prepare-", dir=project.parent) as staging:
        staging = Path(staging)
        source = staging / "source"
        source.mkdir()
        archive = staging / "source.tar"
        with archive.open("wb") as stream:
            run(["git", "-C", repo, "archive", REVISION], stdout=stream)
        with tarfile.open(archive) as contents:
            for member in contents.getmembers():
                target = (source / member.name).resolve()
                if not target.is_relative_to(source) or not (member.isdir() or member.isfile()):
                    raise RuntimeError(f"不支持的源码归档路径：{member.name}")
            contents.extractall(source, filter="data")
        hashes = patch_source(source)
        provenance = {
            "repository": UPSTREAM["repository"], "revision": REVISION,
            "unity_version": UPSTREAM["unity_version"], "package": UPSTREAM["package"],
            "build_notes": UPSTREAM["build_notes"],
            "files_sha256": hashes,
            "patches_sha256": {name: sha256(PATCHES / name)
                               for name in ["TcpHandler.patch", *ADDITIONS]},
            "status": "prepared; not built or installed",
        }
        (source / MARKER).write_text(json.dumps(provenance, indent=2) + "\n")
        source.rename(project)
    return project


def validate_project(project: Path) -> dict:
    manifest = project / MARKER
    if not manifest.is_file():
        raise RuntimeError("缺少受管源码记录；先执行 prepare")
    data = json.loads(manifest.read_text())
    if any(data.get(name) != UPSTREAM[name] for name in ("revision", "unity_version", "package")):
        raise RuntimeError("工程来源或应用标识与当前补丁不一致；请重新 prepare 到新目录")
    expected_files = {*UPSTREAM["audited_blobs"], *ADDITIONS.values()}
    if set(data.get("files_sha256", {})) != expected_files:
        raise RuntimeError("工程校验文件清单不完整；请重新 prepare 到新目录")
    for name, expected in data["files_sha256"].items():
        if not (project / name).is_file() or sha256(project / name) != expected:
            raise RuntimeError(f"受管文件已改变，请重新 prepare 到新目录：{name}")
    for name in ["TcpHandler.patch", *ADDITIONS]:
        if data.get("patches_sha256", {}).get(name) != sha256(PATCHES / name):
            raise RuntimeError(f"补丁已更新，请重新 prepare 到新目录：{name}")
    return data


def build(project: Path, unity: Path, output: Path) -> Path:
    project = managed_location(project)
    validate_project(project)
    unity = unity.expanduser().resolve()
    if not unity.is_file() or not os.access(unity, os.X_OK):
        raise RuntimeError(f"找不到可执行的 Unity Editor：{unity}")
    output = output.resolve()
    if output.exists():
        raise RuntimeError(f"APK 已存在，请指定新的 --output：{output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    log = output.with_suffix(".build.log")
    print("构建采用官方指定 Unity 2021.3.45f2；上游 Shader Graph 14.0.9 声明 Unity 2022.3，"
          "且上游报告 2022.3 偶发崩溃。兼容性尚待此构建及头显验证，详见 docs/guides/quest-client.md。",
          flush=True)
    # Unity reads the requested output from the environment, never shell expansion.
    command = [unity, "-batchmode", "-nographics", "-quit", "-buildTarget", "Android",
               "-projectPath", project, "-executeMethod", "QuestUsbBuild.Build", "-logFile", log]
    try:
        run(command, env={**os.environ, "TIA_QUEST_APK": str(output)})
    except subprocess.CalledProcessError as error:
        raise RuntimeError(f"Unity 构建失败，日志：{log}") from error
    if not output.is_file() or output.stat().st_size == 0:
        raise RuntimeError(f"Unity 未生成 APK，日志：{log}")
    record = {
        "revision": REVISION, "package": UPSTREAM["package"],
        "unity_version": UPSTREAM["unity_version"], "apk_sha256": sha256(output),
        "source_manifest_sha256": sha256(project / MARKER), "status": "built; not installed",
        "build_notes": UPSTREAM["build_notes"],
    }
    output.with_suffix(".build.json").write_text(json.dumps(record, indent=2) + "\n")
    return output


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    prep = commands.add_parser("prepare", help="导出固定官方源码并应用补丁，不运行 Unity")
    prep.add_argument("--project-dir", type=Path, default=BUILD / "source")
    prep.add_argument("--source-checkout", type=Path, help="包含固定 commit 的只读本地源码")
    prep.add_argument("--fetch", action="store_true", help="允许下载固定版本到 outputs/dependencies")
    compile_parser = commands.add_parser("build", help="用指定 Unity Editor 构建 APK，不安装")
    compile_parser.add_argument("--project-dir", type=Path, default=BUILD / "source")
    compile_parser.add_argument("--unity", type=Path, required=True, help="Unity 2021.3.45f2 Editor 可执行文件")
    compile_parser.add_argument("--output", type=Path, default=BUILD / "ThoughtIntoAction-Quest-USB.apk")
    args = parser.parse_args(argv)
    try:
        if args.command == "prepare":
            result = prepare(args.project_dir, args.source_checkout, args.fetch)
            print(f"源码已准备（尚未构建或安装）：{result}")
        else:
            result = build(args.project_dir, args.unity, args.output)
            print(f"APK 已构建（尚未安装）：{result}")
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as error:
        parser.exit(1, f"{error}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
