"""Check relative links to files in the project's Markdown documents (not web URLs).

usage: python tools\\check_links.py        (run in the project root)
Checks STATUS.md, CLAUDE.md and docs/**/*.md except docs/core/ (core uses code-style paths, not links)
and docs/records/checks/ (frozen copies of documents as they were when reviewed; their links point to the old places).
"""

import re
import sys
from pathlib import Path
from urllib.parse import unquote

root = Path(__file__).resolve().parents[1]
files = [root / "STATUS.md", root / "CLAUDE.md"] + [
    p
    for p in (root / "docs").rglob("*.md")
    if p.relative_to(root / "docs").parts[:1] != ("core",)
    and p.relative_to(root / "docs").parts[:2] != ("records", "checks")
]
link = re.compile(r"\]\(([^)\s]+)\)")
bad = 0
for f in files:
    if not f.exists():
        continue
    for n, line in enumerate(f.read_text(encoding="utf-8").splitlines(), 1):
        for target in link.findall(line):
            if re.match(r"^[a-z]+:", target) or target.startswith("#"):
                continue
            path = unquote(target.split("#")[0])
            if path and not (f.parent / path).exists():
                bad += 1
                print(f"{f.relative_to(root)}:{n}: missing {target}")
print("broken links:", bad)
sys.exit(1 if bad else 0)
