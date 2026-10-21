#!/usr/bin/env python3
"""Run every offline check group, each in its own environment, through run.sh."""

import argparse
from pathlib import Path
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
RUN = str(ROOT / "run.sh")
# End-to-end checks write evidence to a new directory per run.
EVIDENCE = ROOT / "outputs/checks" / time.strftime("%Y%m%d-%H%M%S")
PLATFORM = ["env", "PYTHONPATH=src", "OPENBLAS_NUM_THREADS=1", "OMP_NUM_THREADS=1", str(ROOT / ".venv/bin/python"), "-B"]
# name: (file that must exist, commands)
GROUPS = {
    "control": ("control/.venv/bin/python", [[RUN, "control", "verify"], [RUN, "control", "check"]]),
    "platform": (".venv/bin/python", [[RUN, "test", "-q"]]),
    "motion": ("environments/arx_motion/.venv/bin/python", [[RUN, "components", "run", "test.motion"]]),
    "phone": ("environments/arx_motion/.venv/bin/python", [[RUN, "components", "run", "test.phone"]]),
    "rebot": ("environments/rebot/.venv/bin/python", [[RUN, "components", "run", "test.rebot"]]),
    # Pages of src/embodied/monitor/web; Playwright from: npm install --prefix .cache/viewer-validation playwright
    "browser": (".cache/viewer-validation/node_modules/playwright/index.mjs",
                [["node", str(path)] for path in sorted((ROOT / "tests/browser").glob("*.mjs"))]),
    # Heavy MuJoCo groups last: the legacy services in the browser group are timing-sensitive under load.
    # MuJoCo end to end: native routers + workbench buttons, component faults, capture, plugins + calibration.
    "sim": ("control/.venv/bin/python", [
        [str(ROOT / "control/.venv/bin/python"), "-B", str(ROOT / "control/scripts/check_workbench_teleop.py")],
        [str(ROOT / "control/.venv/bin/python"), "-B", str(ROOT / "control/scripts/check_native_components.py"),
         "--quick", "--output", str(EVIDENCE / "native-components")],
        [*PLATFORM, "scripts/check_control_runtime.py", "--output", str(EVIDENCE / "control-runtime")],
        [*PLATFORM, "scripts/check_workbench.py", "--calibration", "--output", str(EVIDENCE / "workbench")],
        [*PLATFORM, "scripts/check_policy_kernel.py", "--output", str(EVIDENCE / "policy-kernel")],
        ["env", "PYTHONPATH=src", "OPENBLAS_NUM_THREADS=1", str(ROOT / "environments/arx_motion/.venv/bin/python"), "-B",
         "scripts/check_groot_kernel.py", "--output", str(EVIDENCE / "groot-kernel")],
    ]),
    # Dual-video GR00T export through the official loader, synthetic measured-state episode.
    "groot": ("environments/arx_motion/.venv-web/bin/python", [
        [str(ROOT / "environments/arx_motion/.venv-web/bin/python"), "-B", "scripts/check_groot_export.py",
         "--output", str(EVIDENCE / "groot-export")]]),
}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--only", nargs="+", choices=GROUPS, help="run only these groups")
    parser.add_argument("--simulation", action="store_true", help="also run the 12 s MuJoCo route in control check")
    parser.add_argument("--print", action="store_true", help="print the commands without running them")
    args = parser.parse_args(argv)
    results = []
    for name in args.only or GROUPS:
        python, commands = GROUPS[name]
        if name == "control" and args.simulation:
            commands = [*commands[:-1], [*commands[-1], "--simulation"]]
        if args.print:
            for command in commands:
                print(name, " ".join(command[1:]))
            continue
        if not (ROOT / python).is_file():
            results.append((name, "SKIP", f"missing {python}"))
            continue
        if missing := next((c[0] for c in commands if c[0] != RUN and not shutil.which(c[0])), None):
            results.append((name, "SKIP", f"missing {missing}"))
            continue
        print(f"== {name}", flush=True)
        started = time.monotonic()
        code = next((code for code in (subprocess.call(command, cwd=ROOT) for command in commands) if code), 0)
        results.append((name, "FAIL" if code else "PASS", f"{time.monotonic() - started:.0f} s"))
    for name, status, detail in results:
        print(f"{status:4}  {name:8}  {detail}")
    return 1 if any(status == "FAIL" for _, status, _ in results) else 0


if __name__ == "__main__":
    raise SystemExit(main())
