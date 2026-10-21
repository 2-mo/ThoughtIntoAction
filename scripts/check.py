#!/usr/bin/env python3
"""Run offline project checks with the configured teleoperation environment."""

from pathlib import Path
import subprocess
import sys

root = Path(__file__).resolve().parents[1]
# Paths and markers come from [tool.pytest.ini_options] in pyproject.toml.
raise SystemExit(subprocess.call([sys.executable, "-B", "-m", "pytest", *sys.argv[1:]], cwd=root))
