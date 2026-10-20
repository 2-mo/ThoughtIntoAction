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
# name: (file that must exist, commands)
GROUPS = {
    "control": ("control/.venv/bin/python", [[RUN, "control", "verify"], [RUN, "control", "check"]]),
    "platform": (".venv/bin/python", [[RUN, "test", "-q"]]),
    "motion": ("environments/arx_motion/.venv/bin/python", [[RUN, "components", "run", "test.motion"]]),
    "phone": ("environments/arx_motion/.venv/bin/python", [[RUN, "components", "run", "test.phone"]]),
    "rebot": ("environments/rebot/.venv/bin/python", [[RUN, "components", "run", "test.rebot"]]),
    # Pages of src/robot_simulation/web; Playwright from: npm install --prefix .cache/viewer-validation playwright
    "browser": (".cache/viewer-validation/node_modules/playwright/index.mjs",
                [["node", str(path)] for path in sorted((ROOT / "tests").glob("browser_*.mjs"))]),
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
