"""Run the vendored arx control application in its own Python environment."""

import argparse
from contextlib import contextmanager
import hashlib
import json
import os
from pathlib import Path
import signal
import sys
import subprocess
import time

from embodied.core.project import PROJECT_ROOT as ROOT

from embodied.contracts.paths import CONTROL_RUNS, CONTROL_ENVIRONMENT, CONTROL_PYTHON, CONTROL_BUILD

CONTROL = ROOT / "src/embodied/control"
SCRIPTS = ROOT / "scripts/control"
SDK_VENDOR = ROOT / "src/embodied/devices/robots/arx5/vendor/arx5-sdk"


@contextmanager
def sim_kernel(log_path, timeout=20, *, viewer=False, config=None):
    """Own an idle MuJoCo kernel, optionally with its read-only desktop viewer."""
    if viewer and sys.platform.startswith("linux") and not os.environ.get("DISPLAY"):
        raise ValueError("MuJoCo viewer requires DISPLAY; use viewer=False or --headless over SSH")
    command = [str(ROOT / "run.sh"), "control", "start", "--backend", "sim", "--input", "idle", "--managed"]
    if config is not None:
        command.extend(['--config', str(Path(config).expanduser().resolve())])
    if not viewer:
        command.append("--headless")
    with Path(log_path).open("w") as log:
        runtime = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline, directory = time.monotonic() + timeout, None
            while directory is None and time.monotonic() < deadline:
                if runtime.poll() is not None:
                    raise RuntimeError(Path(log_path).read_text())
                for path in (CONTROL_RUNS).glob("*/runtime.json"):
                    if json.loads(path.read_text())["supervisor_pid"] == runtime.pid and (path.parent / "runtime.ready").exists():
                        directory = path.parent
                time.sleep(.05)
            if directory is None:
                raise RuntimeError("kernel did not start")
            yield directory
        finally:
            runtime.send_signal(signal.SIGINT)
            runtime.wait(timeout=30)


def verify_sources():
    """Verify immutable SDK/native assets; platform code is maintained as normal source."""
    vendor = SDK_VENDOR
    provenance = json.loads((vendor / 'PROVENANCE.json').read_text())
    checked = 0
    for name, key in [('source.tar', 'archive_sha256'), ('gripper_turns.h', 'gripper_turns_sha256')]:
        if hashlib.sha256((vendor / name).read_bytes()).hexdigest() != provenance[key]:
            raise ValueError(f'SDK asset checksum mismatch: {name}')
        checked += 1
    native = ROOT / 'src/embodied/devices/inputs/quest/native'
    for line in (native / 'SHA256SUMS').read_text().splitlines():
        if not line.strip():
            continue
        expected, name = line.split(maxsplit=1)
        asset = native / name.lstrip('*')
        if hashlib.sha256(asset.read_bytes()).hexdigest() != expected:
            raise ValueError(f'Native asset checksum mismatch: {name}')
        checked += 1
    return {'files': checked, 'status': 'verified', 'scope': 'SDK and native distribution assets'}


def control_environment():
    """Do not inherit Python 3.12 extensions or Conda library overrides."""
    env = os.environ.copy()
    for name in ("PYTHONPATH", "PYTHONHOME", "VIRTUAL_ENV", "LD_LIBRARY_PATH",
                 "UV_PROJECT_ENVIRONMENT", "UV_PROJECT", "CONDA_PREFIX", "CONDA_DEFAULT_ENV", "QUEST_ARX_GRIPPER_TURNS"):
        env.pop(name, None)
    env.update(PYTHONDONTWRITEBYTECODE="1", PYTHONUNBUFFERED="1",
               OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1", MKL_NUM_THREADS="1",
               NUMEXPR_NUM_THREADS="1", VECLIB_MAXIMUM_THREADS="1")
    env["PYTHONPATH"] = str(ROOT / "src")
    env["UV_PROJECT_ENVIRONMENT"] = str(ROOT / "outputs/envs/control")
    return env


def hardware_environment(env, *, robot_plugin=None, config=None):
    from embodied.control.settings import load_settings
    from embodied.contracts.registry import Registry
    profile = load_settings(config).robot
    driver = Registry().create('robot', robot_plugin or profile.hardware_plugin, runtime='control')
    prepare = getattr(driver, 'environment', None)
    return prepare(env) if prepare is not None else env


def build_sdk(headers):
    vendor = SDK_VENDOR
    provenance = json.loads((vendor / 'PROVENANCE.json').read_text())
    for name, key in [('source.tar', 'archive_sha256'), ('gripper_turns.h', 'gripper_turns_sha256')]:
        if hashlib.sha256((vendor / name).read_bytes()).hexdigest() != provenance[key]:
            raise ValueError(f'SDK source identity mismatch: {name}')
    subprocess.run(['uv', 'sync', '--locked', '--project', str(CONTROL_ENVIRONMENT), '--extra', 'arx', '--group', 'sdk-build'],
                   env=control_environment(), check=True)
    command = [str(CONTROL_PYTHON), str(ROOT / 'scripts/setup/build_gripper_sdk.py'),
               '--source-archive', str(vendor / 'source.tar'), '--patch-header', str(vendor / 'gripper_turns.h'),
               '--build-dir', str(CONTROL_BUILD / 'arx-gripper'), '--headers', str(headers.resolve())]
    subprocess.run(command, cwd=ROOT, env=control_environment(), check=True)
    hardware_environment(control_environment())
    return 0


def command_for(args):
    if args.command == "setup":
        return ["bash", str(SCRIPTS / "setup.sh"), "arx" if args.sdk else "sim"]
    python = CONTROL_PYTHON
    if not python.is_file():
        raise ValueError("Control environment missing; run ./run.sh control setup")
    if args.command == "check":
        return ["bash", str(SCRIPTS / "check.sh"), *(["--simulation"] if args.simulation else [])]
    if args.command == 'viewer':
        directory = runtime_directory(args.run)
        identity = json.loads((directory / 'runtime.json').read_text())
        params = json.loads((directory / 'parameters.json').read_text())
        return [str(python), '-B', '-m', 'embodied.control', '_viewer', '--prefix', identity['runtime_id'],
                '--config', params['config_path']]
    cmd = [str(python), "-B", "-m", "embodied.control", "run" if args.command == "start" else args.command]
    if args.command == "start":
        if args.backend == "real" and args.input == "demo":
            raise ValueError("Synthetic demo input is simulation only")
        if not 0 <= args.duration < float("inf"):
            raise ValueError("Duration must be finite and nonnegative")
        cmd.extend(["--backend", args.backend, "--input", "vr" if args.input == "quest" else args.input,
                    "--duration", str(args.duration)])
        if getattr(args, "managed", False):
            cmd.append("--managed")
        if getattr(args, "robot_plugin", None):
            cmd.extend(["--robot-plugin", args.robot_plugin])
        for key in ('input_plugin', 'routing_plugin', 'ingress_plugin', 'ingress_channel', 'controller_plugin'):
            if getattr(args, key, None):
                cmd.extend(['--' + key.replace('_', '-'), getattr(args, key)])
        if getattr(args, "planner_plugin", None):
            cmd.extend(["--planner-plugin", args.planner_plugin])
        if args.headless:
            cmd.append("--headless")
        if args.record_tuning:
            cmd.append("--record-tuning")
        if not args.no_telemetry:
            cmd.append("--telemetry")
        if args.report:
            cmd.extend(["--report", str(Path(args.report).resolve())])
    elif args.command == "doctor" and args.vr:
        cmd.append("--vr")
    for key in ("config", "vr_config"):
        if value := getattr(args, key, None):
            cmd.extend(["--" + key.replace("_", "-"), str(Path(value).resolve())])
    return cmd


def stale_runs(keep, root=None):
    """Run directories beyond the newest `keep`, never one whose supervisor is still alive (todo K-12)."""
    from embodied.contracts.runtime.control import live_runs
    root = Path(root or CONTROL_RUNS)
    runs = sorted(path.parent for path in root.glob("*/runtime.json"))
    live = {path.resolve() for path in live_runs(root)}
    return [path for path in runs[:max(0, len(runs) - keep)] if path.resolve() not in live]


def clean(keep, yes):
    import shutil
    stale = stale_runs(keep)
    size = sum(f.stat().st_size for path in stale for f in path.rglob("*") if f.is_file())
    print(f"{len(stale)} 个旧运行目录（{size / 1e6:.0f} MB），保留最新 {keep} 个与运行中的内核。")
    if not stale:
        return 0
    if not yes:
        if not sys.stdin.isatty() or input("删除这些目录？输入 yes 确认：").strip() != "yes":
            print("未删除。非交互使用时加 --yes。")
            return 1
    for path in stale:
        shutil.rmtree(path)
    print(f"已删除 {len(stale)} 个。")
    return 0


def runtime_directory(selected=None):
    if selected is not None:
        return Path(selected).resolve()
    active = sorted((CONTROL_RUNS).glob('*/observer.sock'))
    if not active:
        raise ValueError('No active observer; start control with telemetry')
    return active[-1].parent


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--print-command", action="store_true", help="Resolve only; do not launch")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("verify", help="Check upstream import and declared patches without SDKs")
    setup = sub.add_parser("setup", help="Build isolated locked Python 3.10 environment")
    setup.add_argument("--sdk", action="store_true", help="Also install the upstream SDK; does not enable hardware")
    check = sub.add_parser("check", help="Run the upstream offline checks (fake hardware only)")
    check.add_argument("--simulation", action="store_true")
    sdk = sub.add_parser('build-sdk', help='Rebuild the local gripper correction for control Python; never opens CAN')
    sdk.add_argument('--headers', type=Path, default=ROOT / 'outputs/build/arx-gripper/deps/usr/include')
    doctor = sub.add_parser("doctor", help="Inspect dependencies and model without connecting devices")
    doctor.add_argument("--vr", action="store_true")
    viewer = sub.add_parser('viewer', help='Attach the upstream read-only MuJoCo viewer; closing it leaves control running')
    viewer.add_argument('--run', type=Path)
    start = sub.add_parser("start", help="Launch arx; real startup enables and moves Home")
    start.add_argument("--backend", choices=("sim", "real"), default="sim")
    start.add_argument("--input", choices=("quest", "demo", "idle"), default="quest")
    start.add_argument("--managed", action="store_true", help="Enable plugin command gateway")
    start.add_argument("--robot-plugin")
    start.add_argument("--input-plugin", "--vr-plugin", dest="input_plugin")
    start.add_argument("--routing-plugin", default="arx.native")
    start.add_argument("--ingress-plugin", default="arx.native")
    start.add_argument("--ingress-channel", default="native")
    start.add_argument("--controller-plugin")
    start.add_argument("--planner-plugin", default="motion.default")
    start.add_argument("--headless", action="store_true")
    start.add_argument("--duration", type=float, default=0)
    start.add_argument("--record-tuning", action="store_true")
    start.add_argument("--no-telemetry", action="store_true", help="Run without the workbench observer")
    start.add_argument("--report")
    for child in (doctor, start):
        child.add_argument("--config", help="arx robot JSON; root launcher resolves paths from the repository")
        child.add_argument("--vr-config", help="arx VR JSON")
    cleaner = sub.add_parser("clean", help="Delete old outputs/runtime/control directories; keeps the newest and running ones")
    cleaner.add_argument("--keep", type=int, default=20, help="Newest run directories to keep (default 20)")
    cleaner.add_argument("--yes", action="store_true", help="Do not ask for confirmation")
    observe = sub.add_parser("observe", help="Read telemetry without loading control or device libraries")
    observe.add_argument("--run", type=Path, help="Run directory; defaults to the newest active observer")
    observe.add_argument("--mode", choices=("latest", "full"), default="latest")
    observe.add_argument("--count", type=int, default=1, help="Number of ticks; 0 reads through termination")
    observe.add_argument("--output", type=Path, help="New JSONL file; existing files are never replaced")
    args = parser.parse_args(argv)
    try:
        if args.command == "clean":
            if args.keep < 0:
                raise ValueError("--keep must be nonnegative")
            return clean(args.keep, args.yes)
        if args.command == "observe":
            from embodied.contracts.runtime.client import RuntimeReader
            if args.count < 0:
                raise ValueError("Count must be nonnegative")
            directory = runtime_directory(args.run)
            output = args.output.open('x') if args.output else sys.stdout
            try:
                with RuntimeReader(directory, args.mode) as reader:
                    output.write(json.dumps(reader.descriptor, ensure_ascii=False) + '\n')
                    count = 0
                    while not reader.terminal and (not args.count or count < args.count):
                        output.write(json.dumps(reader.read(), ensure_ascii=False) + '\n')
                        count += 1
            finally:
                if args.output:
                    output.close()
            return 0
        result = verify_sources()
        if args.command == "verify":
            print(json.dumps(result))
            return 0
        if args.command == 'build-sdk':
            return build_sdk(args.headers)
        command = command_for(args)
        if args.print_command:
            print(json.dumps({"argv": command, "cwd": str(ROOT)}, ensure_ascii=False))
            return 0
        os.chdir(ROOT)
        env = control_environment()
        if args.command == 'start' and args.backend == 'real':
            env = hardware_environment(env, robot_plugin=args.robot_plugin, config=args.config)
        os.execvpe(command[0], command, env)
    except (OSError, ValueError, EOFError, subprocess.SubprocessError) as exc:
        print(f"Control launch error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
