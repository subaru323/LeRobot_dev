r"""Same process, three robots: the common process form, per-model profiles, and the desk checks (industrial plan
step 3, handbook 4). Deliverable 3 of 4 (the model difference table) is written from here.

No maker environment is on this PC (no ROBOGUIDE, no WINCAPS III), so this stops at: input checks, unit and frame
round trips, failure checks (unreachable point, missing done signal, unsupported / unconfirmed command), and the
verification request. MODEL-SPECIFIC BLANKS ARE NOT GUESSED: a value is either quoted from a public maker page (with
the URL and the date it was read) or from this project's own files, or it is UNCONFIRMED and every check that needs it
answers "cannot judge" (never "ok").

usage (any python 3 with numpy):
  python industrial\transfer.py            writes outputs\industrial\<stamp>_transfer\ (difference table, drafts, request)
  python industrial\transfer.py selftest   the same + the failure checks; exit 1 if a check does not behave
"""

import datetime
import json
import os
import sys

import numpy as np

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
U = "未確認"     # unconfirmed

# ---------------------------------------------------------------------------------------------------------------
# 1. The common process (model independent). Cell frame: x = conveyor direction, y = up, z = across; origin = the
#    inspection point on the belt centre line, belt top at y = 0. mm, s. Same frame as industrial/cell.py (+ y).
PROCESS = [
    {"step": "wait_inspection", "state": "観測待ち", "needs": ["io.event_in"],
     "in": "inspection {event_id, track_id, verdict, t}", "done_when": "NG event for a track not handled yet"},
    {"step": "shoot", "state": "候補照合", "needs": ["camera.trigger"],
     "in": "conveyor stopped", "done_when": "picture with t_capture after the stop"},
    {"step": "approach", "state": "排除指令", "needs": ["motion.linear", "frame.user"],
     "target": "pick point (x, z from the picture) at y = +50", "point": "pick_above", "done_when": "in position"},
    {"step": "grip", "state": "排除指令", "needs": ["motion.linear", "io.gripper"],
     "target": "pick point at y = +5", "point": "pick", "done_when": "gripper closed signal"},
    {"step": "place", "state": "排除指令", "needs": ["motion.linear", "io.gripper"],
     "target": "bin point (480, +60, 120)", "point": "bin", "done_when": "gripper open signal, then arm done signal"},
    {"step": "reshoot", "state": "再撮影", "needs": ["camera.trigger", "timer.timeout"],
     "in": "arm done", "done_when": "picture with t_capture after the done signal, within the timeout"},
    {"step": "decide", "state": "完了／理由付き保留", "needs": ["io.event_out"],
     "done_when": "complete only if: pick point empty AND bin +1 AND picture fresh; else hold with the reason"},
]
POINTS = {"pick_above": (500.0, 50.0, 0.0), "pick": (500.0, 5.0, 0.0), "bin": (480.0, 60.0, 120.0)}

# ---------------------------------------------------------------------------------------------------------------
# 2. Model profiles. Every field: (value, source). Source "" never appears: it is a URL + date, a project file, or U.
SRC_CRX = "https://www.fanuc.co.jp/ja/product/robot/model/crx/lineup/crx-20ia_l.html（2026-11-19 に読んだ）"
SRC_RG = "https://www.fanuc.co.jp/en/product/robot/function/roboguide.htm（2026-11-19 に読んだ）"
SRC_WC = "https://www.denso-wave.com/ja/Robot/product/software/wincaps3.html（2026-11-19 に読んだ）"
SRC_SO = "sim/scene-data.js の R_IN、R_OUT（tools/reach-ring.js の実測 80〜240mm に 5mm の余裕。運ぶ高さ 50mm、真上から）"

MODELS = {
    "SO-101": {
        "model": ("SO-101 フォロワー（このプロジェクトの腕）", "STATUS.md"),
        "controller": ("LeRobot（PC）＋ STS3215 サーボ。制御は tools/puzzle_run.py の形", "STATUS.md・tools/puzzle_run.py"),
        "reach": ({"kind": "ring", "r_in": 85.0, "r_out": 235.0}, SRC_SO),
        "payload": (U, U), "repeatability": (U, U),
        "units": ("URDF は m・rad。sim/plan-cli.js の frames[].q は度", "docs/records/夜の作業_2026-11-19_指示.md の列 P"),
        "base_in_cell": ({"t": (490.0, 0.0, -100.0), "yaw_deg": 0.0}, "【仮】この架空セルの置き方（決めたのはこの回）"),
        # status words: あり / 一部 / 無い / 未確認. convert() accepts only あり (review C42f-2: partial was passing)
        "caps": {"motion.linear": ("一部：sim は真上からつかむ姿勢を解いて関節の列を作る。工具先端の直線の動きは保証していない", "sim/README.md"),
                 "frame.user": ("一部：紙の座標 → 腕の換算はある。この架空セルの座標から腕への換算は、ここ（transfer.py）の【仮】の置き方だけ", "sim/README.md"),
                 "io.gripper": ("一部：爪の角度の指令だけ。閉じた・開いたの信号は無い", "sim/README.md"),
                 "camera.trigger": ("あり：上と手首のカメラ（sim では描画）", "sim/README.md"),
                 "timer.timeout": ("無い：industrial/controllers.py は回数の上限だけで、時間の期限は無い", "industrial/controllers.py"),
                 "io.event_in": ("一部：Python の dict で受け取る。設備の信号の口は無い", "industrial/controllers.py"),
                 "io.event_out": ("一部：Python の dict で返す。設備の信号の口は無い", "industrial/controllers.py")},
        "program_format": ("Python（tools/puzzle_run.py と同じ形）", "tools/puzzle_run.py"),
        "offline_env": ("sim/（three.js の運動学）・sim/physics/（MuJoCo）", "sim/README.md"),
        "stop_recovery": (U, U),
    },
    "FANUC CRX-20iA/L": {
        "model": ("CRX-20iA/L（6 軸。可搬 20 kg・リーチ 1418 mm・繰り返し ±0.04 mm・本体 41 kg・IP67）", SRC_CRX),
        "controller": ("CRX シリーズの制御装置として R-30iB Mini Plus・R-50iA Mate・R-50iA Compact DC が並ぶ。この機種でどれを使うかは" + U, SRC_CRX),
        "reach": ({"kind": "sphere", "r": 1418.0, "note": "リーチの測り方（どこからどこまで）は" + U + "。粗い判定にだけ使う"}, SRC_CRX),
        "payload": ("20 kg（手首）", SRC_CRX), "repeatability": ("±0.04 mm", SRC_CRX),
        "units": (U, U),
        "base_in_cell": ({"t": (460.0, -50.0, -700.0), "yaw_deg": 0.0}, "【仮】この架空セルの置き方（決めたのはこの回）"),
        "caps": {k: (U, U) for k in ("motion.linear", "frame.user", "io.gripper", "camera.trigger", "timer.timeout", "io.event_in", "io.event_out")},
        "program_format": (U, U),
        "offline_env": ("ROBOGUIDE（PC でロボットの系の見える化・模擬。CRX に対応するかはこのページに書いていない＝" + U + "）", SRC_RG),
        "stop_recovery": (U, U),
    },
    "デンソーウェーブ（機種は未選定）": {
        "model": (U + "（手順書 4：工具込みの負荷・到達・設置条件と使える制御装置で選ぶ。まだ選んでいない）", "docs/産業用フィジカルAI開発手順書.md"),
        "controller": ("RC9 シリーズは WINCAPS III の動く OS の条件に名前がある。どの制御装置を使うかは" + U, SRC_WC),
        "reach": (U, U), "payload": (U, U), "repeatability": (U, U), "units": (U, U),
        "base_in_cell": ({"t": (460.0, -50.0, -600.0), "yaw_deg": 0.0}, "【仮】この架空セルの置き方（決めたのはこの回）"),
        "caps": {k: (U, U) for k in ("motion.linear", "frame.user", "io.gripper", "camera.trigger", "timer.timeout", "io.event_in", "io.event_out")},
        "program_format": ("PAC 言語・PacScript（WINCAPS III でプログラムを作る言語として書いてある）。構文は" + U + "なので下書きは書かない", SRC_WC),
        "offline_env": ("WINCAPS III。模擬（サイクルタイム・動作・姿勢・干渉）は製品版だけ（Light 版・試供版には無い）。RC9 のプロジェクトは Windows 10/11 64bit", SRC_WC),
        "stop_recovery": (U, U),
    },
}


# ---------------------------------------------------------------------------------------------------------------
# 3. Frames and units
def T_base_in_cell(m):
    """4x4: base frame -> cell frame. Base: Z up (= cell y). yaw 0: base X = cell +z (toward the belt for a base at
    negative z, beside the belt), base Y = cell +x... checked in selftest (the pick point must be on base +X)."""
    b = MODELS[m]["base_in_cell"][0]
    a = np.radians(b["yaw_deg"])
    # base X, Y, Z expressed in the cell frame (x, y, z): X = (sin a, 0, cos a), Z = (0, 1, 0), Y = Z x X
    X = np.array([np.sin(a), 0.0, np.cos(a)])
    Z = np.array([0.0, 1.0, 0.0])
    Y = np.cross(Z, X)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = X, Y, Z, b["t"]
    return T


def cell_to_base(m, p_mm):
    return (np.linalg.inv(T_base_in_cell(m)) @ np.r_[p_mm, 1.0])[:3]


def base_to_cell(m, p_mm):
    return (T_base_in_cell(m) @ np.r_[p_mm, 1.0])[:3]


def reach_check(m, p_base_mm):
    """'ok' / 'out' / 'cannot judge (reason)'. Only a coarse necessary condition (no orientation, no joints)."""
    r = MODELS[m]["reach"][0]
    if r == U or not isinstance(r, dict):
        return "判定できない（リーチが" + U + "）"
    if r["kind"] == "sphere":
        d = float(np.linalg.norm(p_base_mm))
        return "届く（粗い）" if d <= r["r"] else f"届かない（{d:.0f} > {r['r']:.0f} mm）"
    if r["kind"] == "ring":       # horizontal distance from the pan axis; carry height fixed at 50 mm (SO-101 sim)
        d = float(np.hypot(p_base_mm[0], p_base_mm[1]))
        return "届く（粗い）" if r["r_in"] <= d <= r["r_out"] else f"届かない（水平 {d:.0f} mm、輪 {r['r_in']:.0f}〜{r['r_out']:.0f}）"
    return "判定できない（形が不明）"


def convert(m, process=PROCESS):
    """the process -> a per-model draft. Refuses (lists every step) when a needed capability is not plainly あり
    (partial, absent or unconfirmed)."""
    caps = MODELS[m]["caps"]
    missing = [(s["step"], c, caps[c][0]) for s in process for c in s["needs"] if c in caps and not caps[c][0].startswith("あり")]
    unknown = [(s["step"], c) for s in process for c in s["needs"] if c not in caps]
    draft = []
    for s in process:
        d = {"step": s["step"], "state": s["state"]}
        if "point" in s:
            d["target_base_mm"] = [round(float(x), 3) for x in cell_to_base(m, np.array(POINTS[s["point"]]))]
        draft.append(d)
    return {"ok": not missing and not unknown, "missing": missing, "unknown": unknown, "draft": draft}


def run_signals(trace, timeout_s=3.0):
    """the decide step on a recorded signal trace: [(t, name)]. Returns (state, why). 'done' must follow 'cmd'."""
    t_cmd = next((t for t, n in trace if n == "cmd"), None)
    if t_cmd is None:
        return "hold", "指令が記録に無い"
    t_done = next((t for t, n in trace if n == "done" and t >= t_cmd), None)
    if t_done is None or t_done - t_cmd > timeout_s:
        return "hold", f"完了の信号が {timeout_s} 秒以内に来ない"
    t_img = next((t for t, n in trace if n == "image" and t > t_done), None)
    if t_img is None or t_img - t_done > timeout_s:
        return "hold", "完了の信号の後の絵が無い"
    return "next", "完了の信号と、その後の絵がある（絵の中身の確かめは controllers.py）"


# ---------------------------------------------------------------------------------------------------------------
# 4. Outputs
def table_md():
    cols = ["機種・制御装置", "工具と基準座標", "単位", "教示点", "信号", "停止復帰", "出力形式", "試した環境", "証拠", "未確認"]
    L = ["# 機種の差分表（架空案件の見本）", "",
         "同じ工程「検査結果を待つ → 指定の位置へ運ぶ → 再確認」を 3 つの機種に割り当てた表。列は手順書 4 のとおり。",
         "**機種固有の値は、メーカーの公開ページかこのプロジェクトのファイルで確かめた物だけを書き、出どころを付けた。分からない所は「未確認」。推測で埋めていない。**",
         "メーカーの環境（ROBOGUIDE・WINCAPS III）はこの PC に無いので、ここまでの検算は机上（入力の検査・座標の往復・失敗の検算）だけ。", "",
         "| " + " | ".join(["機種"] + cols) + " |", "|" + "---|" * (len(cols) + 1)]
    for m, p in MODELS.items():
        conv = convert(m)
        pts = {k: reach_check(m, cell_to_base(m, np.array(v))) for k, v in POINTS.items()}
        tool = f"架空セルの座標（x 流れ・y 上・z 横、mm）→ 腕の土台の座標。土台の置き方 {p['base_in_cell'][0]}（{p['base_in_cell'][1]}）"
        unconf = [k for k, v in p.items() if k != "caps" and (v[0] == U or (isinstance(v[0], str) and U in v[0]))]
        unconf += [f"caps.{k}" for k, v in p["caps"].items() if v[0] == U]
        row = [p["model"][0] + " ／ " + p["controller"][0], tool, p["units"][0],
               "取る点・箱の点は絵から決まる（教示しない）。届くか（粗い）：" + "、".join(f"{k} {v}" for k, v in pts.items()),
               "、".join(f"{k}={v[0]}" for k, v in p["caps"].items()), p["stop_recovery"][0], p["program_format"][0],
               p["offline_env"][0], "下の「出どころ」", "、".join(unconf) or "-"]
        L.append("| " + " | ".join([m] + [str(x).replace("|", "／") for x in row]) + " |")
    L += ["", "## 変換の結果（工程に要る機能が「あり」でなければ（一部・無い・未確認）、変換は黙って進まずに断る）", ""]
    for m in MODELS:
        c = convert(m)
        L.append(f"- {m}：{'変換できた（下書きは json）' if c['ok'] else '断った。「あり」でない（工程, 機能）の組 ' + str(len(c['missing'])) + ' 件、機能は：' + '、'.join(sorted({x[1] for x in c['missing']}))}")
    L += ["", "## 出どころ", ""]
    for m, p in MODELS.items():
        srcs = sorted({v[1] for k, v in p.items() if k != "caps" and v[1] != U} | {v[1] for v in p["caps"].values() if v[1] != U})
        L.append(f"- {m}：" + "；".join(srcs))
    return "\n".join(L) + "\n"


def request_md():
    L = ["# 検証の依頼票（架空案件の見本）", "",
         "メーカーの環境がある人に頼む項目。1 行が 1 つの確かめ。結果の欄に、環境の名前・版・結果・日付を書いてもらう。", "",
         "| 番号 | 機種 | 確かめること | なぜ要るか | 結果（環境・版・日付） |", "|---|---|---|---|---|"]
    n = 0
    for m, p in MODELS.items():
        if m == "SO-101":
            continue
        for k, v in list(p.items()) + [(f"caps.{c}", x) for c, x in p["caps"].items()]:
            if k == "caps":
                continue
            if v[0] == U or (isinstance(v[0], str) and U in v[0]):
                n += 1
                L.append(f"| R{n:02d} | {m} | {k} | 変換・届くかの判定・信号の順に要る（いまは判定できないと出る） | |")
        n += 1
        L.append(f"| R{n:02d} | {m} | 下書きの取る点・箱の点（土台の座標、json）を読み込み、届くか・姿勢が続くか・干渉 | 粗い判定（球）しかしていない | |")
        n += 1
        L.append(f"| R{n:02d} | {m} | 完了の信号が来ないとき・絵が来ないときに止まり、保留を出すか（信号の順） | 机上の検算（run_signals）だけ | |")
    return "\n".join(L) + "\n"


READ_PAGES = (SRC_CRX, SRC_RG, SRC_WC)


def source_ok(value, src):
    """a value's source must be: 未確認 (with an 未確認 value), 【仮】 (this run's own layout), a maker page in
    READ_PAGES, or project files that exist (several joined by ・; text after the first space is a note)."""
    if src == U:
        return value == U
    if src.startswith("【仮】") or src in READ_PAGES:
        return True
    files = [part.split(" ")[0] for part in src.split("・")]
    return all(os.path.exists(os.path.join(ROOT, f)) for f in files)


def selftest():
    fails = 0

    def check(ok, msg):
        nonlocal fails
        fails += not ok
        print(f"  {'ok  ' if ok else 'FAIL'} {msg}")

    print("== frames and units ==")
    for m in MODELS:
        e = max(np.abs(base_to_cell(m, cell_to_base(m, np.array(v))) - np.array(v)).max() for v in POINTS.values())
        check(e < 1e-9, f"{m}: cell -> base -> cell round trip, max error {e:.1e} mm")
    p = np.array(POINTS["pick"])
    wrong = reach_check("FANUC CRX-20iA/L", cell_to_base("FANUC CRX-20iA/L", p) * 1000.0)   # metres read as mm
    check(wrong.startswith("届かない"), f"COARSE: only a x1000 unit mix-up is caught by the reach check: {wrong}")
    for m in MODELS:
        xb = cell_to_base(m, np.array(POINTS["pick"]))[0]
        check(xb > 0, f"{m}: the pick point is on base +X (toward the belt), x = {xb:.0f} mm")
    print("== unreachable points ==")
    far = np.array([500.0, 0.0, 900.0])        # 1600 mm from the CRX base (across the belt)
    r = reach_check("FANUC CRX-20iA/L", cell_to_base("FANUC CRX-20iA/L", far))
    check(r.startswith("届かない"), f"CRX: a point 1600 mm away -> {r}")
    r = reach_check("FANUC CRX-20iA/L", cell_to_base("FANUC CRX-20iA/L", p))
    check(r.startswith("届く"), f"CRX: the pick point -> {r}")
    r = reach_check("SO-101", cell_to_base("SO-101", np.array([490.0, 50.0, -60.0])))
    check(r.startswith("届かない"), f"SO-101: a point 40 mm from the pan axis (inside the ring) -> {r}")
    r = reach_check("デンソーウェーブ（機種は未選定）", cell_to_base("デンソーウェーブ（機種は未選定）", p))
    check(r.startswith("判定できない"), f"Denso (reach unconfirmed) never says ok -> {r}")
    print("== missing done signal ==")
    check(run_signals([(0, "cmd"), (1.0, "done"), (1.2, "image")])[0] == "next", "cmd -> done -> image: next")
    s = run_signals([(0, "cmd"), (1.2, "image")])
    check(s[0] == "hold", f"no done signal -> {s}")
    s = run_signals([(0, "cmd"), (5.0, "done"), (5.2, "image")])
    check(s[0] == "hold", f"done after the timeout -> {s}")
    s = run_signals([(0, "cmd"), (1.0, "done"), (0.9, "image")])
    check(s[0] == "hold", f"only a picture from before the done signal -> {s}")
    print("== unsupported / unconfirmed command ==")
    c = convert("SO-101")
    check(not c["ok"] and {x[1] for x in c["missing"]} == {"motion.linear", "frame.user", "io.gripper", "timer.timeout", "io.event_in", "io.event_out"},
          f"SO-101: refuses partial / absent capabilities: {sorted({x[1] for x in c['missing']})}")
    MODELS["_all_yes"] = dict(MODELS["SO-101"], caps={k: ("あり：試し", "【仮】") for k in MODELS["SO-101"]["caps"]})
    c = convert("_all_yes")
    del MODELS["_all_yes"]
    check(c["ok"] and len(c["draft"]) == len(PROCESS), "a profile with every capability あり converts (the ok path works)")
    for m in ("FANUC CRX-20iA/L", "デンソーウェーブ（機種は未選定）"):
        c = convert(m)
        check(not c["ok"] and len(c["missing"]) > 0, f"{m}: refuses, {len(c['missing'])} unconfirmed (step, capability)")
    bad = PROCESS + [{"step": "laser_mark", "state": "x", "needs": ["io.laser"]}]
    c = convert("SO-101", bad)
    check(not c["ok"] and c["unknown"] == [("laser_mark", "io.laser")], f"a command no profile knows is refused: {c['unknown']}")
    print("== no blank filled by guess ==")
    bad = [(m, k, v[1]) for m, p in MODELS.items() for k, v in list(p.items()) + list(p["caps"].items())
           if k != "caps" and not source_ok(v[0], v[1])]
    check(not bad, "every value is 未確認, 【仮】, from a maker page read this night, or from a project file that exists" + (f"; bad {bad}" if bad else ""))
    check(not source_ok("あり", "sim/no_such_file.js") and not source_ok("あり", "https://example.com/x"),
          "the source check fails on a missing file and on an unlisted URL")
    print("SELFTEST", "OK" if not fails else f"FAILED ({fails})")
    return fails


def main():
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out = os.path.join(ROOT, "outputs", "industrial", stamp + "_transfer")
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, "機種の差分表.md"), "w", encoding="utf-8") as f:
        f.write(table_md())
    with open(os.path.join(out, "検証の依頼票.md"), "w", encoding="utf-8") as f:
        f.write(request_md())
    with open(os.path.join(out, "drafts.json"), "w", encoding="utf-8") as f:
        json.dump({"process": PROCESS, "points_cell_mm": POINTS, "models": {m: convert(m) for m in MODELS}}, f, ensure_ascii=False, indent=1)
    print(f"wrote {out}")
    if len(sys.argv) > 1 and sys.argv[1] == "selftest":
        sys.exit(1 if selftest() else 0)


if __name__ == "__main__":
    main()
