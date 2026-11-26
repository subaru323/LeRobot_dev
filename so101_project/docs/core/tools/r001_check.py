"""R-001 の点検を、自動メモリなしの claude -p で回す（core の道具。単独で動く）。

使い方（プロジェクトのルートで）:
    python docs/core/tools/r001_check.py <結論の文書> <依頼の要旨> <計画> <出力> [--extra 追加の頼み] [--allow-read] [--allow-web]

依頼文は同じフォルダの r001_prompt.md。点検の出力と費用を <出力> に書く。
点検役は新しい会話の AI で、利用者の自動メモリと CLAUDE.md は入れない。書き込み・シェル・サブエージェントの道具は禁じる。
--allow-read はプロジェクトのファイルを読ませる（プロジェクトのルートで起動する）。--allow-web は Web で出典を確かめさせる。
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
# --allowedTools は「確認なしで使ってよい道具」を決めるだけで、ほかの道具を禁じないので、禁じる道具を明示する
READ_ONLY = ["--disallowedTools", "Bash,PowerShell,Edit,Write,NotebookEdit,Task,Agent"]


def _read(path):
    with open(path, encoding="utf-8") as f:
        return f.read()


def _claude():
    path = os.environ.get("CLAUDE_BIN") or shutil.which("claude")
    if not path:
        for name in ("claude.exe", "claude"):
            cand = os.path.join(os.path.expanduser("~"), ".local", "bin", name)
            if os.path.isfile(cand):
                path = cand
                break
    if not path:
        raise SystemExit("claude が見つかりません。Claude Code の CLI を入れるか、環境変数 CLAUDE_BIN にパスを設定してください。")
    return [path]


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("doc")
    ap.add_argument("request")
    ap.add_argument("plan")
    ap.add_argument("out")
    ap.add_argument("--extra")
    ap.add_argument("--model", default="claude-opus-5-5")
    ap.add_argument("--allow-web", action="store_true")
    ap.add_argument("--allow-read", action="store_true")
    a = ap.parse_args(argv)

    prompt = (_read(os.path.join(HERE, "r001_prompt.md"))
              .replace("{{EXTRA}}", _read(a.extra) if a.extra else "")
              .replace("{{REQUEST}}", _read(a.request))
              .replace("{{PLAN}}", _read(a.plan))
              .replace("{{DOC}}", _read(a.doc)))
    # --safe-mode: プロジェクトや利用者の CLAUDE.md を読み込ませない（2026-10-14 に確かめた）
    args = ["-p", "--output-format", "json", "--model", a.model, "--safe-mode", "--strict-mcp-config"] + READ_ONLY
    tools = (["WebSearch", "WebFetch"] if a.allow_web else []) + (["Read", "Glob", "Grep"] if a.allow_read else [])
    if tools:
        args += ["--allowedTools", ",".join(tools)]
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    # 読ませないときは、プロジェクトの外の空のフォルダで起動する
    tmp = None if a.allow_read else tempfile.mkdtemp(prefix="r001_")
    cwd = os.getcwd() if a.allow_read else tmp
    env = dict(os.environ, CLAUDE_CODE_DISABLE_AUTO_MEMORY="1")
    try:
        proc = subprocess.run(_claude() + args, input=prompt.encode("utf-8"), capture_output=True, cwd=cwd, env=env,
                              timeout=3600)
    except subprocess.TimeoutExpired:
        with open(a.out, "w", encoding="utf-8") as f:
            f.write(f"# R-001 点検の出力\n\n対象: {a.doc}\n\n時間切れ（60分）で、点検の結果は取れなかった。\n")
        raise SystemExit("時間切れ。点検をやり直す。")
    finally:
        if tmp:
            shutil.rmtree(tmp, ignore_errors=True)
    stdout = proc.stdout.decode("utf-8", errors="replace")
    try:
        data = json.loads(stdout.strip().splitlines()[-1]) if stdout.strip() else {}
    except (json.JSONDecodeError, IndexError):
        data = {}
    meta = {"cost_usd": data.get("total_cost_usd"), "num_turns": data.get("num_turns"),
            "returncode": proc.returncode, "usage": data.get("usage")}
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    with open(a.out, "w", encoding="utf-8") as f:
        f.write(f"# R-001 点検の出力\n\n対象: {a.doc}\n\n```json\n{json.dumps(meta, ensure_ascii=False, indent=1)}\n```\n\n")
        f.write(data.get("result") or stdout)
    print(f"wrote {a.out} returncode={proc.returncode} cost={meta['cost_usd']}")
    if proc.stderr.strip():
        print(proc.stderr.decode("utf-8", errors="replace")[-2000:], file=sys.stderr)


if __name__ == "__main__":
    main()
