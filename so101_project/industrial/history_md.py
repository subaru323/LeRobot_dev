r"""Decision history in a form a person can read (industrial plan: deliverable 2 of 4).

For every case (development + unseen) and method, one table: time, what the controller received, what it did, and
its decision with the reason. The header records the case number, the input version (case version + sha256 of the
code files), how it was run, and where the outputs are. The scorer's result is written last and labelled as such.

usage (the .venv python):
  python industrial\history_md.py [--methods baseline,candidate] [--seed 0]
writes outputs\industrial\<stamp>_history\判断の履歴.md
"""

import argparse
import datetime
import hashlib
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cell import CASE_VERSION, CASES, UNSEEN2_CASES, UNSEEN3_CASES, UNSEEN4_CASES, UNSEEN5_CASES, UNSEEN6_CASES, UNSEEN7_CASES, UNSEEN8_CASES, UNSEEN9_CASES, UNSEEN10_CASES, UNSEEN11_CASES, UNSEEN_CASES    # noqa: E402
from run import ALL_CASES, ROOT, run_case             # noqa: E402
from video import describe_action, describe_decision, timeline   # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))


def received(ob):
    k = ob["type"]
    if k == "inspection":
        return f"検査 {ob['event_id']} {ob['track_id']} {ob['verdict']}（t={ob['t']}, enc={ob['encoder_at_inspection']}）"
    if k == "conveyor":
        return f"コンベア {ob['state']}（enc={ob['encoder']}）"
    if k == "image":
        return f"絵 撮影 t={ob['t_capture']}" if ob.get("image") is not None else "絵が届かない"
    if k == "robot":
        return f"腕の完了 {ob['cmd_id']}"
    return k


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--methods", default="baseline,candidate")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = os.path.join(ROOT, "outputs", "industrial", stamp + "_history")
    os.makedirs(out_dir, exist_ok=True)
    sha = {f: hashlib.sha256(open(os.path.join(HERE, f), "rb").read()).hexdigest()[:12]
           for f in ("cell.py", "controllers.py", "score.py", "run.py")}
    L = ["# 判断の履歴（架空の排除セル）", "",
         "架空案件の見本。制御が受け取った物・した事・理由を時刻の順に並べる。最後の「採点」だけは、制御に渡さない本当の記録から出す。", "",
         f"- 入力の版：ケースの版 `{CASE_VERSION}`、乱数種 {a.seed}。コードの sha256 の頭 12 文字：" + "、".join(f"`{f}` {h}" for f, h in sha.items()),
         f"- 流し方：`python industrial\\history_md.py --methods {a.methods} --seed {a.seed}`（`MAIN_PG\\lerobot\\.venv` の python）",
         f"- 出力の場所：`outputs\\industrial\\{stamp}_history\\`。同じ入力の絵と細かい記録は `python industrial\\run.py` の `history_<ケース>_<やり方>.json`",
         "- ケース番号：D = 開発用（制御と一緒に作った）、U = 初見（期待を先に書き、制御を直さずに流した）、V = 初見 2（2 回目。視野の外の追跡番号を数に入れない直しの前に期待を書いた）、W = 旧 初見 3（2 回目。くっついた部品の直し v3 の前に期待を書いたが、初めて流した後に世界を直したので、3 回目に開発用へ移した。初見ではない）、X = 初見 4（2 回目。v2 の境目の余裕。制御は直していない）、Y = 初見 5（3 回目。重なる部品。直し v4 の前に期待を書いた）、Z = 初見 6（3 回目。印の読み落とし。直し v5 の前に期待を書いた）、R = 初見 7・8・9（4 回目。視野の端。直し v6 の前に期待を書いた）、Q = 初見 10（5 回目。2 個とも NG。直し v7 の前に期待を書いた）、P = 初見 11（5 回目。点検 C42n の形の値違い。直し v8 の前に期待を書いた）。いまの candidate は v8。U の表も v8 で流した物で、`u_third_part` は見た後に変わった", ""]
    num = {c: f"D{i + 1:02d}" for i, c in enumerate(CASES)}
    num.update({c: f"U{i + 1:02d}" for i, c in enumerate(UNSEEN_CASES)})
    num.update({c: f"V{i + 1:02d}" for i, c in enumerate(UNSEEN2_CASES)})   # run I-2: written before the fix
    num.update({c: f"W{i + 1:02d}" for i, c in enumerate(UNSEEN3_CASES)})   # run I-2: written before the v3 fix
    num.update({c: f"X{i + 1:02d}" for i, c in enumerate(UNSEEN4_CASES)})   # run I-2: margin of the view rule
    num.update({c: f"Y{i + 1:02d}" for i, c in enumerate(UNSEEN5_CASES)})   # run I-3: overlapping parts, before v4
    num.update({c: f"Z{i + 1:02d}" for i, c in enumerate(UNSEEN6_CASES)})   # run I-3: the mark, before v5
    num.update({c: f"R{i + 1:02d}" for i, c in enumerate({**UNSEEN7_CASES, **UNSEEN8_CASES, **UNSEEN9_CASES})})   # I-4, before v6
    num.update({c: f"Q{i + 1:02d}" for i, c in enumerate(UNSEEN10_CASES)})   # run I-5: two NG tracks, before v7
    num.update({c: f"P{i + 1:02d}" for i, c in enumerate(UNSEEN11_CASES)})   # run I-5: review C42n forms, before v8
    L += ["## 一覧", "", "| 番号 | ケース | 設定 | " + " | ".join(a.methods.split(",")) + " |", "|---|---|---|" + "---|" * len(a.methods.split(","))]
    runs = {}
    for c in ALL_CASES:
        for m in a.methods.split(","):
            runs[(c, m)] = run_case(c, m, a.seed)
        L.append(f"| {num[c]} | {c} | `{ALL_CASES[c]}` | " + " | ".join(runs[(c, m)]["score"]["outcome"] for m in a.methods.split(",")) + " |")
    L.append("")
    for c in ALL_CASES:
        L += [f"## {num[c]} {c}", "", f"設定 `{ALL_CASES[c]}`", ""]
        for m in a.methods.split(","):
            r = runs[(c, m)]
            L += [f"### {m}", "", "| 時刻 s | 受け取った物 | した事 | 判断と理由 |", "|---|---|---|---|"]
            for st in timeline(r):
                rec = "<br>".join(received(ob) for ob in st["fed"])
                dec = "<br>".join(describe_decision(d) for d in st["decisions"]) or "-"
                L.append(f"| {st['t']} | {rec} | {describe_action(st['action'])} | {dec} |")
            s = r["score"]
            L += ["", f"採点（制御には渡さない）：**{s['outcome']}**。箱に入った物 {', '.join(s['bin']) or 'なし'}、"
                      f"誤排除 {s['wrong_reject']}、流出 {'あり' if s['escape'] else 'なし'}、"
                      f"完了と宣言したのに部品が箱に無い番号 {s['false_done']}、指令 {s['commands']}（余分 {s['extra_commands']}）、"
                      f"工程の時間 {s['cycle_s']} s", ""]
    path = os.path.join(out_dir, "判断の履歴.md")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(L) + "\n")
    print(f"wrote {path}  ({len(ALL_CASES)} cases x {len(a.methods.split(','))} methods, {len(L)} lines)")


if __name__ == "__main__":
    main()
