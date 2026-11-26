"""結果の文書を機械で検査する（R-001 の前の、安い一次検査。core の道具）。

使い方: python docs/core/tools/doc_lint.py <ファイル> [<ファイル> ...]

検査すること:
1. 使ってはいけない言葉（docs/core/方針.md 4）。「効かない」「ほぼ効かない」「差は無い」など。
   「」で囲んだ言及（「効かない」と書かない、など）と、直後に打ち消しが続くもの（効かないとは言えない、など）は数えない。
2. 手で書いた割合・倍率の疑い（「約2割」「3〜4割」「1.5倍」「約20%」）。集計の出力から写したかを、人か AI が確かめる印。
   同じ行に出典の印（「集計」「出力」「表の」）があれば数えない。
3. 制御文字（書き換えのときに記号が化けた疑い。docs/core/方針.md 10）。これは引用・表・コードの行も検査する。
引用（> で始まる行）、表、コードの行は、1 と 2 を検査しない。見つけたものを「ファイル:行: 種類: 行の中身」で出す。見つけたら終了コード 1。
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

FORBIDDEN = ["ほぼ効かない", "効かない", "効かなかった", "差は無い", "差はない", "差が無かった", "差がなかった"]
AFTER_NEGATION = re.compile(r"^.{0,3}(とは言え|とは書か|とは言い切れ|と書かない|と言わない|と言い切らない|とは限らない)")
RATIO = re.compile(r"(約\s*)?\d+(\.\d+)?\s*(〜\s*\d+(\.\d+)?\s*)?(割|倍|%|％)")
SOURCE_MARKS = ["集計", "出力", "表の"]
CTRL = re.compile(r"[\x00-\x08\x0b\x0c\x0e-\x1f]")


def _forbidden_hit(line: str, word: str) -> bool:
    for m in re.finditer(re.escape(word), line):
        before, after = line[:m.start()], line[m.end():]
        if before.count("「") > before.count("」"):
            continue  # 「」の中の言及
        if AFTER_NEGATION.search(after):
            continue
        return True
    return False


def lint(path: Path) -> list[str]:
    out = []
    in_code = False
    for n, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if line.strip().startswith("```"):
            in_code = not in_code
            continue
        if CTRL.search(line):
            out.append(f"{path}:{n}: 制御文字（書き換えで化けた疑い。方針10）: {line.strip()[:120]!r}")
        if in_code or line.lstrip().startswith(">") or line.lstrip().startswith("|"):
            continue
        for w in FORBIDDEN:
            if _forbidden_hit(line, w):
                out.append(f"{path}:{n}: 禁止語「{w}」: {line.strip()[:120]}")
                break
        if RATIO.search(line) and not any(s in line for s in SOURCE_MARKS):
            out.append(f"{path}:{n}: 手書きの割合の疑い: {line.strip()[:120]}")
    return out


def main() -> None:
    found = []
    for a in sys.argv[1:]:
        found += lint(Path(a))
    print("\n".join(found) if found else "見つからなかった")
    sys.exit(1 if found else 0)


if __name__ == "__main__":
    main()
