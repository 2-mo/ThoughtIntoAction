#!/usr/bin/env python3
"""Run offline project checks with the configured teleoperation environment."""

from pathlib import Path
import subprocess
import sys

root = Path(__file__).resolve().parents[1]
raise SystemExit(subprocess.call([sys.executable, "-B", "-m", "unittest", "discover", "-s", str(root / "tests"), "-v", *sys.argv[1:]], cwd=root))
