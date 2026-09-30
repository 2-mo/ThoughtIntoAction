"""Run the vendored arx control application in its own Python environment."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import subprocess

ROOT = Path(__file__).resolve().parents[2]
CONTROL = ROOT / "control"


def verify_sources(directory=CONTROL):
    """Accept only original upstream bytes or explicitly inventoried local patches."""
    directory = Path(directory)
    files = json.loads((directory / "UPSTREAM_FILES.json").read_text())
    patches = json.loads((directory / "LOCAL_PATCHES.json").read_text())
    failures = []
    for name, original in files.items():
        path = directory / name
        patch = patches.get(name)
        if patch and (patch.get("upstream_sha256") != original["sha256"] or not patch.get("reason")):
            failures.append(f"Invalid patch provenance: {name}")
            continue
        expected = patch["sha256"] if patch else original["sha256"]
        if path.is_symlink() or not path.is_file():
            failures.append(f"Missing or linked source: {name}")
        elif hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            failures.append(f"Unrecorded change: {name}")
        elif bool(path.stat().st_mode & 0o111) != original["executable"]:
            failures.append(f"Executable mode mismatch: {name}")
    failures.extend(f"Patch has no upstream source: {name}" for name in patches.keys() - files.keys())
    if failures:
        raise ValueError("\n".join(failures))
    return {"files": len(files), "local_patches": len(patches), "status": "verified"}


def control_environment():
    """Do not inherit Python 3.12 extensions or Conda library overrides."""
    env = os.environ.copy()
    for name in ("PYTHONPATH", "PYTHONHOME", "VIRTUAL_ENV", "LD_LIBRARY_PATH",
                 "UV_PROJECT_ENVIRONMENT", "UV_PROJECT", "CONDA_PREFIX", "CONDA_DEFAULT_ENV", "QUEST_ARX_GRIPPER_TURNS"):
        env.pop(name, None)
    env.update(PYTHONDONTWRITEBYTECODE="1", PYTHONUNBUFFERED="1",
               OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1", MKL_NUM_THREADS="1",
               NUMEXPR_NUM_THREADS="1", VECLIB_MAXIMUM_THREADS="1")
    return env


def hardware_environment(env):
    path = CONTROL / 'build/arx-gripper/source/python'
    env = {**env, 'PYTHONPATH': str(path), 'QUEST_ARX_GRIPPER_TURNS': '1'}
    result = subprocess.run([str(CONTROL / '.venv/bin/python'), '-c',
                             'import arx5_interface as sdk; assert getattr(sdk, "quest_gripper_turns_version", None) == 1'],
                            cwd=CONTROL, env=env, capture_output=True, text=True, timeout=10)
    if result.returncode:
        raise ValueError('Local Python 3.10 gripper patch missing; run ./run.sh control build-sdk')
    return env


def build_sdk(headers):
    vendor = CONTROL / 'vendor/arx5-sdk'
    provenance = json.loads((vendor / 'PROVENANCE.json').read_text())
    for name, key in [('source.tar', 'archive_sha256'), ('gripper_turns.h', 'gripper_turns_sha256')]:
        if hashlib.sha256((vendor / name).read_bytes()).hexdigest() != provenance[key]:
            raise ValueError(f'SDK source identity mismatch: {name}')
    subprocess.run(['uv', 'sync', '--locked', '--project', str(CONTROL), '--extra', 'arx', '--group', 'sdk-build'],
                   env=control_environment(), check=True)
    command = [str(CONTROL / '.venv/bin/python'), str(ROOT / 'scripts/setup/build_gripper_sdk.py'),
               '--source-archive', str(vendor / 'source.tar'), '--patch-header', str(vendor / 'gripper_turns.h'),
               '--build-dir', str(CONTROL / 'build/arx-gripper'), '--headers', str(headers.resolve())]
    subprocess.run(command, cwd=ROOT, env=control_environment(), check=True)
    hardware_environment(control_environment())
    return 0


def command_for(args):
    if args.command == "setup":
        return ["bash", str(CONTROL / "setup.sh"), "arx" if args.sdk else "sim"]
    python = CONTROL / ".venv/bin/python"
    if not python.is_file():
        raise ValueError("Control environment missing; run ./run.sh control setup")
    if args.command == "check":
        return ["bash", str(CONTROL / "check.sh"), *(["--simulation"] if args.simulation else [])]
    if args.command == 'viewer':
        directory = runtime_directory(args.run)
        identity = json.loads((directory / 'runtime.json').read_text())
        params = json.loads((directory / 'parameters.json').read_text())
        return [str(python), '-B', '-m', 'arx5_vr', '_viewer', '--prefix', identity['runtime_id'],
                '--config', params['config_path']]
    cmd = [str(python), "-B", "-m", "arx5_vr", "run" if args.command == "start" else args.command]
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


def runtime_directory(selected=None):
    if selected is not None:
        return Path(selected).resolve()
    active = sorted((CONTROL / 'run').glob('*/observer.sock'))
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
    start.add_argument("--planner-plugin", default="arx.basic")
    start.add_argument("--headless", action="store_true")
    start.add_argument("--duration", type=float, default=0)
    start.add_argument("--record-tuning", action="store_true")
    start.add_argument("--no-telemetry", action="store_true", help="Run without the workbench observer")
    start.add_argument("--report")
    for child in (doctor, start):
        child.add_argument("--config", help="arx robot JSON; root launcher resolves paths from the repository")
        child.add_argument("--vr-config", help="arx VR JSON")
    observe = sub.add_parser("observe", help="Read telemetry without loading control or device libraries")
    observe.add_argument("--run", type=Path, help="Run directory; defaults to the newest active observer")
    observe.add_argument("--mode", choices=("latest", "full"), default="latest")
    observe.add_argument("--count", type=int, default=1, help="Number of ticks; 0 reads through termination")
    observe.add_argument("--output", type=Path, help="New JSONL file; existing files are never replaced")
    args = parser.parse_args(argv)
    try:
        if args.command == "observe":
            from runtime_client.client import RuntimeReader
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
            print(json.dumps({"argv": command, "cwd": str(CONTROL)}, ensure_ascii=False))
            return 0
        os.chdir(CONTROL)
        env = control_environment()
        if args.command == 'start' and args.backend == 'real':
            env = hardware_environment(env)
        os.execvpe(command[0], command, env)
    except (OSError, ValueError, EOFError, subprocess.SubprocessError) as exc:
        print(f"Control launch error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
