"""既存のプロジェクトの AI 向けの設定と文書から、昔の定石の型と長すぎる CLAUDE.md を拾う（機械の一次検査）。

使い方: python audit.py [プロジェクトのフォルダ]   （省略すると今のフォルダ）

この道具は種（ai-knowhow スキル）に入って配られる。マスターは Claude_knowhow/harness/knowhow_audit_template.py。
型は同じフォルダの patterns.json（ノウハウ集の「古い」項目の検出の欄から作った）。標準ライブラリだけで動く。
拾ったものは「疑い」。引用や説明の中の語も拾うので、1件ずつ人か AI が見て決める。ファイルは何も書き換えない。
"""
import json
import os
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
SKIP_DIRS = {".git", "node_modules", ".venv", "venv", "__pycache__", "knowhow_previous"}
SKIPPED_REPOS: list[str] = []


def walk(root: Path):
    """root の下のファイルとフォルダ。入れ子の git リポジトリ（ほかの人のコードの写しなど）には入らない。"""
    for dirpath, dirnames, filenames in os.walk(root):
        d = Path(dirpath)
        keep = []
        for n in dirnames:
            if n in SKIP_DIRS or (d / n / "SEED.json").exists():  # 植えた種の写しは見ない
                continue
            if (d / n / ".git").exists():
                SKIPPED_REPOS.append((d / n).relative_to(root).as_posix())
                continue
            keep.append(n)
        dirnames[:] = keep
        for n in dirnames:
            yield d / n
        for n in filenames:
            yield d / n


def ai_docs(root: Path):
    """AI に読ませる文書: CLAUDE.md・CLAUDE.local.md・AGENTS.md（どの階層でも）と .claude/ の中の .md。"""
    for p in walk(root):
        if not p.is_file():
            continue
        rel = p.relative_to(root).as_posix()
        if p.name in ("CLAUDE.md", "CLAUDE.local.md", "AGENTS.md") or (rel.startswith(".claude/") and p.suffix == ".md"):
            yield p, rel


def main() -> int:
    try:
        sys.stdout.reconfigure(encoding="utf-8")  # Windows でパイプに出すと文字化けするため
    except Exception:
        pass
    root =Path(sys.argv[1] if len(sys.argv) > 1 else ".").expanduser().resolve()
    pat = json.loads((HERE / "patterns.json").read_text(encoding="utf-8"))
    found = []
    docs = list(ai_docs(root))
    for c in pat["checks"]:
        rx = re.compile(c["regex"])
        if c["where"] == "全体":
            for p in walk(root):
                rel = p.relative_to(root).as_posix() + ("/" if p.is_dir() else "")
                if rx.search(rel):
                    found.append((rel, 0, c, rel))
                    break
            continue
        for p, rel in docs:
            if c["where"] == "CLAUDE.md" and p.name not in ("CLAUDE.md", "CLAUDE.local.md", "AGENTS.md"):
                continue
            text = p.read_text(encoding="utf-8", errors="replace")
            if c["where"] == "CLAUDE.md":
                for m in rx.finditer(text):
                    n = text.count("\n", 0, m.start()) + 1
                    found.append((rel, n, c, text.splitlines()[n - 1]))
                continue
            for n, line in enumerate(text.splitlines(), 1):
                if rx.search(line):
                    found.append((rel, n, c, line))
    for p, rel in docs:
        if p.name == "CLAUDE.md":
            n = len(p.read_text(encoding="utf-8", errors="replace").splitlines())
            if n > pat["claudemd_max_lines"]:
                found.append((rel, 0, {"id": "長さ", "主張": f"CLAUDE.md が {n} 行（目安は {pat['claudemd_max_lines']} 行未満）", "instead": pat["claudemd_item"]}, ""))
    repos = sorted(set(SKIPPED_REPOS))
    if repos:
        print("入れ子の git リポジトリは見ていない（ほかの人のコードなら直さない。自分のものなら、そのフォルダを指定して別に動かす）: " + "、".join(repos))
    if not any(p.name == "CLAUDE.md" and p.parent == root for p, _ in docs):
        print("直下に CLAUDE.md が無い。入口の文書（STATUS.md など）があるなら、AI は自動では読まない（K-0050 と ドキュメント.md を見て、短い CLAUDE.md を作るか判断する）。")
    if not found:
        print(f"{root}: 昔の定石の型は見つからなかった（道具が拾えるものだけを見た）。")
        return 0
    by = {}
    for rel, n, c, line in found:
        by.setdefault(c["id"], (c, []))[1].append((rel, n, line))
    for cid, (c, rows) in by.items():
        print(f"\n■ {cid} {c['主張']}")
        if c.get("instead"):
            print(f"  代わりに: {c['instead']}")
        for rel, n, line in rows[:15]:
            print(f"  {rel}{':' + str(n) if n else ''}  {line.strip()[:100]}")
        if len(rows) > 15:
            print(f"  …ほか {len(rows) - 15} 件")
    print(f"\n疑い {len(found)} 件（{len(by)} 種類）。1件ずつ見て、直す案は利用者の承認を取ってから当てる。")
    return 1


if __name__ == "__main__":
    sys.exit(main())
