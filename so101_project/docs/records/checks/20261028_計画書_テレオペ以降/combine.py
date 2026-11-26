"""Join the two target documents into one file for r001_check.py."""
from pathlib import Path

here = Path(__file__).parent
docs = here.parents[2]
parts = [docs / "計画書_テレオペ以降.md", docs / "データ収集と学習の手順書.md"]
text = "\n\n---\n\n".join(f"<!-- {p.name} -->\n" + p.read_text(encoding="utf-8") for p in parts)
out = here / "target.md"
out.write_text(text, encoding="utf-8")
print(out, out.stat().st_size)
