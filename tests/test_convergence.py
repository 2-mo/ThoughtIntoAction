"""Ratchet for the platform refactor (todo/00-guardrails.md, G-04).

Each check lists the files that still contain a pattern being removed. New files
must not join a list, and a file that no longer matches must be dropped from
convergence_baseline.json, so the lists only shrink.
"""

import ast
import json
from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
BASELINE = Path(__file__).with_name("convergence_baseline.json")
SOURCE_ROOTS = ("src", "control", "scripts", "apps/console/src", "integrations", "examples")
SKIPPED_DIRS = {".venv", "__pycache__", "node_modules", "vendor", "native", "run", "build", "dist", "models"}
CODE_SUFFIXES = {".py", ".sh", ".ts", ".js", ".mjs", ".html"}

TEXT_CHECKS = {  # name: (suffixes, pattern, files that are the intended single source)
    # Target: only control/arx5_vr/real.py talks to the arm SDK (todo C-02).
    "sdk_controller": ({".py"}, re.compile(r"Arx5JointController\("), set()),
    # Target: none after packaging (todo B-01).
    "sys_path": ({".py"}, re.compile(r"sys\.path\.(insert|append)\("), set()),
    # Target: none besides the upstream arx-televr default; others read the robot profile (todo R-02).
    "can0_literal": (CODE_SUFFIXES, re.compile(r"""(["'=])can0\b"""), {"control/arx5_vr/types.py"}),
    # Target: none; machine paths come from configs/local.toml (todo R-05).
    "home_path": (CODE_SUFFIXES, re.compile(r"/home/[A-Za-z0-9_]+/"), set()),
}


def source_files():
    for name in SOURCE_ROOTS:
        for path in sorted((ROOT / name).rglob("*")):
            relative = path.relative_to(ROOT)
            if path.is_file() and path.suffix in CODE_SUFFIXES and not SKIPPED_DIRS & set(relative.parts[:-1]):
                yield relative.as_posix(), path


def control_imports_of_src(files):
    """Control kernel imports of platform code in src/; only L0 embodied.contracts is allowed (todo K-03)."""
    packages = {path.name for path in (ROOT / "src").iterdir() if (path / "__init__.py").is_file()}
    found = set()
    for relative, path in files:
        if not relative.startswith("control/") or path.suffix != ".py":
            continue
        for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"), relative)):
            if isinstance(node, ast.Import):
                names = [alias.name for alias in node.names]
            elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
                names = [node.module]
            else:
                continue
            for name in names:
                if name.split(".")[0] in packages and not (name == "embodied.contracts" or name.startswith("embodied.contracts.")):
                    found.add(f"{relative}:{'.'.join(name.split('.')[:2])}")
    return found


def current():
    files = list(source_files())
    result = {name: set() for name in TEXT_CHECKS}
    for relative, path in files:
        text = path.read_text(encoding="utf-8", errors="replace")
        for name, (suffixes, pattern, exempt) in TEXT_CHECKS.items():
            if path.suffix in suffixes and relative not in exempt and pattern.search(text):
                result[name].add(relative)
    result["control_imports_src"] = control_imports_of_src(files)
    return result


class ConvergenceTests(unittest.TestCase):
    def test_lists_only_shrink(self):
        baseline = json.loads(BASELINE.read_text(encoding="utf-8"))
        found = current()
        self.assertEqual(set(baseline), set(found), "checks in convergence_baseline.json")
        for name in sorted(found):
            with self.subTest(check=name):
                expected = set(baseline[name])
                added, removed = sorted(found[name] - expected), sorted(expected - found[name])
                self.assertFalse(added, f"{name}: new occurrences; remove them instead of adding to the baseline")
                self.assertFalse(removed, f"{name}: cleaned up; delete these entries from convergence_baseline.json")


if __name__ == "__main__":
    unittest.main()
