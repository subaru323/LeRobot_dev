"""core をプロジェクトに埋め込み、CLAUDE.md や docs のフォルダを生やす。

使い方（マスターのフォルダで）:
    python core_master/tools/seed.py <プロジェクトのフォルダ> [--update] [--no-settings] [--core-only]

すること:
1. core_master を <プロジェクト>/docs/core に写し、版とファイルごとのハッシュを docs/core/MANIFEST.json に残す。
   写したファイルには読み取り専用の印を付ける。
2. <プロジェクト>/.claude/settings.json が無ければ、docs/core の編集と新規作成を禁じる決まり（permissions.deny）で作る。
   既にあれば書き換えず、足りない決まりを表示する（手で足す）。--no-settings で何もしない。
3. 無ければ作る: CLAUDE.md、STATUS.md、docs/README.md、docs/records/R001_LOG.md、docs/rules/検査の一覧.md、
   docs/rules、docs/guides、docs/plan、docs/knowledge、docs/core_proposals。既にあるものは書き換えない。
--update: 既に docs/core があるときに、新しい版で入れ替える（プロジェクトの docs/core 以外は触らない）。
    前の版の MANIFEST.json と今のファイルを比べ、変わっていたら一覧を出して止まる。--force で入れ替える。
    古い版は消さずに docs/core_previous/v<版>/ に移す。そこに同じ版が既にあれば止まる。
--core-only: 3 の代わりに docs/core_proposals だけを作る（自分の置き場所を持つ、中盤以降のプロジェクト向け）。
既にあるファイルは、どれも上書きしない。
MASTER.md、CHANGELOG.md、seed.py はマスターだけのもので、埋め込まない。
"""
import argparse
import datetime
import hashlib
import json
import os
import shutil
import stat
import sys

MASTER = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DENY = ["Edit(/docs/core/**)", "Write(/docs/core/**)"]
SKIP = {"__pycache__", "CHANGELOG.md", "MASTER.md", "seed.py"}  # マスターだけで使うもの


def _read(path):
    with open(path, encoding="utf-8") as f:
        return f.read()


def _write_new(path, text, made):
    """無いときだけ作る。既にあれば何もしない（"x" で開くので、ほかから作られていても上書きしない）。"""
    if os.path.exists(path):
        return
    os.makedirs(os.path.dirname(path), exist_ok=True)
    try:
        with open(path, "x", encoding="utf-8", newline="\n") as f:
            f.write(text)
    except FileExistsError:
        return
    made.append(path)


def _writable(root):
    for d, _, files in os.walk(root):
        for n in files:
            os.chmod(os.path.join(d, n), stat.S_IREAD | stat.S_IWRITE)


def _changed_since_seed(core):
    """前に埋め込んだときの MANIFEST.json と比べて、変わった・増えた・消えたファイルを返す。"""
    mp = os.path.join(core, "MANIFEST.json")
    if not os.path.exists(mp):
        return ["(MANIFEST.json が無いので比べられない)"]
    old = json.loads(_read(mp)).get("files", {})
    now = {}
    for d, _, files in os.walk(core):
        for n in files:
            p = os.path.join(d, n)
            rel = os.path.relpath(p, core).replace("\\", "/")
            if rel != "MANIFEST.json":
                now[rel] = hashlib.sha256(open(p, "rb").read()).hexdigest()
    out = [f"変わった: {k}" for k in now if k in old and old[k] != now[k]]
    out += [f"増えた: {k}" for k in now if k not in old]
    out += [f"消えた: {k}" for k in old if k not in now]
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("project")
    ap.add_argument("--update", action="store_true")
    ap.add_argument("--no-settings", action="store_true")
    ap.add_argument("--force", action="store_true")
    ap.add_argument("--core-only", action="store_true",
                    help="docs/core と docs/core_proposals だけを作る（自分の置き場所を持つ、中盤以降のプロジェクト向け）")
    a = ap.parse_args(argv)

    proj = os.path.abspath(a.project)
    if not os.path.isdir(proj):
        raise SystemExit(f"フォルダが無い: {proj}")
    version = _read(os.path.join(MASTER, "VERSION")).strip()
    core = os.path.join(proj, "docs", "core")
    if os.path.exists(core):
        if not a.update:
            raise SystemExit(f"既に {core} がある。新しい版で入れ替えるときは --update を付ける。")
        changed = _changed_since_seed(core)
        if changed and not a.force:
            print("前に埋め込んだときから、docs/core の中が変わっている:")
            for c in changed:
                print("  ", c)
            raise SystemExit("変えた中身を修正案（docs/core_proposals/）に拾ってから、--force を付けて入れ替える。")
        # 古い版は消さずに docs/core_previous/v<版>/ へ移す。同じ版が既にあれば止める（上書きしない）
        old_version = (_read(os.path.join(core, "VERSION")).strip()
                       if os.path.exists(os.path.join(core, "VERSION")) else "unknown")
        backup = os.path.join(proj, "docs", "core_previous", f"v{old_version}")
        if os.path.exists(backup):
            raise SystemExit(f"既に {backup} がある。上書きしないので、中身を確かめて移すか消してからやり直す。")
        os.makedirs(os.path.dirname(backup), exist_ok=True)
        shutil.move(core, backup)
        print(f"古い docs/core（v{old_version}）を {os.path.relpath(backup, proj)} に移した")

    shutil.copytree(MASTER, core, ignore=lambda d, names: [n for n in names if n in SKIP])
    manifest = {"version": version, "seeded": datetime.date.today().isoformat(), "files": {}}
    for d, _, files in os.walk(core):
        for n in files:
            p = os.path.join(d, n)
            manifest["files"][os.path.relpath(p, core).replace("\\", "/")] = hashlib.sha256(open(p, "rb").read()).hexdigest()
    with open(os.path.join(core, "MANIFEST.json"), "w", encoding="utf-8") as f:
        json.dump(manifest, f, ensure_ascii=False, indent=1)
    for d, _, files in os.walk(core):
        for n in files:
            os.chmod(os.path.join(d, n), stat.S_IREAD)
    print(f"docs/core に core v{version} を写した（{len(manifest['files'])} ファイル、読み取り専用）")

    if not a.no_settings:
        sp = os.path.join(proj, ".claude", "settings.json")
        if os.path.exists(sp):
            # 既にある設定ファイルは書き換えない。足りない決まりがあれば、足す行を出すだけ
            try:
                deny = json.loads(_read(sp)).get("permissions", {}).get("deny", [])
            except json.JSONDecodeError:
                deny = []
            missing = [r for r in DENY if r not in deny]
            if missing:
                print(f"注意: .claude/settings.json が既にあるので書き換えていない。docs/core を読むだけにするには、"
                      f"permissions.deny に次を手で足す: {missing}")
        else:
            _write_new(sp, json.dumps({"permissions": {"deny": DENY}}, ensure_ascii=False, indent=2) + "\n", [])
            print(f".claude/settings.json を作った（permissions.deny: {DENY}）")

    if a.core_only:
        p = os.path.join(proj, "docs", "core_proposals")
        if not os.path.exists(p):
            os.makedirs(p)
            open(os.path.join(p, ".gitkeep"), "w").close()
            print("作った: docs/core_proposals")
        print("--core-only: CLAUDE.md や docs のほかのフォルダは作っていない。CLAUDE.md から docs/core/方針.md を指す行は手で足す。")
        return
    t = os.path.join(MASTER, "templates")
    fill = lambda s: s.replace("{{VERSION}}", version).replace("{{DATE}}", datetime.date.today().isoformat())
    made = []
    _write_new(os.path.join(proj, "CLAUDE.md"), fill(_read(os.path.join(t, "CLAUDE.md"))), made)
    _write_new(os.path.join(proj, "STATUS.md"), fill(_read(os.path.join(t, "STATUS.md"))), made)
    _write_new(os.path.join(proj, "docs", "README.md"), fill(_read(os.path.join(t, "docs_README.md"))), made)
    _write_new(os.path.join(proj, "docs", "records", "R001_LOG.md"), _read(os.path.join(t, "R001_LOG.md")), made)
    _write_new(os.path.join(proj, "docs", "rules", "検査の一覧.md"), _read(os.path.join(t, "検査の一覧.md")), made)
    for sub in ("rules", "guides", "plan", "knowledge", "core_proposals"):
        p = os.path.join(proj, "docs", sub)
        if not os.path.exists(p):
            os.makedirs(p)
            open(os.path.join(p, ".gitkeep"), "w").close()
            made.append(p + os.sep)
    for p in made:
        print("作った:", os.path.relpath(p, proj))
    for name in ("CLAUDE.md", "STATUS.md"):
        if os.path.join(proj, name) not in made:
            print(f"既にあるので書き換えていない: {name}（docs/core/templates/{name} を見て、必要なら足す）")
    if a.update:
        print(f"CLAUDE.md と docs/README.md の版の表記を v{version} に直す（手で）。")


if __name__ == "__main__":
    sys.exit(main())
