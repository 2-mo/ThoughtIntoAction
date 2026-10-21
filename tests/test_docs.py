"""Relative links in maintained Markdown and the site resolve (history and third-party copies excluded)."""
import re
import subprocess
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
SKIP = ("third_party/", "docs/history/imported/", "archives/")


class DocumentationLinkTests(unittest.TestCase):
    def test_relative_links_resolve(self):
        files = subprocess.run(["git", "-c", "core.quotepath=false", "ls-files", "*.md"], cwd=ROOT,
                               capture_output=True, text=True, check=True).stdout.split("\n")
        broken = []
        for name in filter(None, files):
            if name.startswith(SKIP):
                continue
            for target in re.findall(r"\]\(([^)\s#]+)(?:#[^)]*)?\)", (ROOT / name).read_text(encoding="utf-8")):
                if not re.match(r"^[a-z]+:", target) and not target.startswith(("/", "<")) \
                        and not (ROOT / name).parent.joinpath(target).exists():
                    broken.append(f"{name}: {target}")
        self.assertEqual(broken, [])

    def test_site_links_and_assets_resolve(self):
        page = ROOT / "site/index.html"
        targets = re.findall(r'(?:href|src)="([^"#]+)"', page.read_text(encoding="utf-8"))
        broken = [t for t in targets if not re.match(r"^[a-z]+:", t) and not (page.parent / t).exists()]
        self.assertEqual(broken, [])


if __name__ == "__main__":
    unittest.main()
