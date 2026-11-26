"""Freeze the plan as target.md and fill docs/core/tools/r001_prompt.md into prompt.md (for the subagent reviewer)."""
from pathlib import Path

here = Path(__file__).parent
docs = here.parents[2]
plan = docs / "plan" / "計画書_テレオペ以降.md"
(here / "target.md").write_text(plan.read_text(encoding="utf-8"), encoding="utf-8")
prompt = (docs / "core" / "tools" / "r001_prompt.md").read_text(encoding="utf-8")
for key, name in [("EXTRA", "extra.md"), ("REQUEST", "request.md"), ("PLAN", "plan.md"), ("DOC", "target.md")]:
    prompt = prompt.replace("{{" + key + "}}", (here / name).read_text(encoding="utf-8").strip())
(here / "prompt.md").write_text(prompt, encoding="utf-8")
print(here / "prompt.md", len(prompt))
