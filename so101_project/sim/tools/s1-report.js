// S1 のまとめ(2026-11-22。点検 C59 で直した): tools/s1-sweep.js の outputs/s1/*.jsonl を読み、表にする。
//   node tools/s1-report.js > ../outputs/s1/report.md      (sim/ で)
//   1) 格子(盤 × 辺 × すき間)の結果の表。ok に、いちばん近い所が 1mm【仮】未満なら「ok小」
//   2) すき間と挟める幅の線だけの目安(下)との違い。目安は届く範囲・空いたマスへ指が下りる順番・3 マス以上を挟むことを見ないので、
//      結果の根拠ではなく、どこで線から外れるかを見るためのもの
//      目安: 「すき間 < 指の厚さ + 余裕 × 2」か「その盤で 2 マスでしか挟めないピースがあり、2 × 辺 + すき間 > 挟める幅」なら断る見込み
//   3) 段ごとのいちばん近い所(ok の回)と、全体のいちばん近い所(どの組・どの段か)
//   4) 高さ・余裕・つかみ方・盤の形の回
"use strict";
const fs = require("fs"), path = require("path");
const D = path.join(__dirname, "..", "..", "outputs", "s1");
const read = f => fs.existsSync(path.join(D, f)) ? fs.readFileSync(path.join(D, f), "utf8").trim().split("\n").filter(Boolean).map(l => JSON.parse(l)) : [];
const PH = ["move", "open_descend", "approach_close", "lift", "carry", "place", "release", "withdraw"];
const PHJ = { move: "移る", open_descend: "開いて下ろす", approach_close: "寄せて閉じる", lift: "上げる", carry: "運ぶ", place: "下ろして置く", release: "開く", withdraw: "抜く" };
const SETNAME = { 2: "盤 2(3×4・L J O)", 6: "盤 6(ロの字・I L J)", 3: "盤 3(3×3・I3 V3 V3)" };
const NEED2 = { 2: true, 6: false, 3: false };   // 2 マスでしか挟めないピースがあるか(O)
const SMALL = 1.0;   // 【仮】ok でも、いちばん近い所がこれ未満なら「ok小」
const sh = v => (v === null || v === undefined ? "—" : v.toFixed(1));
const tag = r => r.verdict === "ok" ? (r.min_clear_mm !== null && r.min_clear_mm < SMALL ? "ok小" : "ok") : r.verdict === "拒否" ? "拒" : r.verdict === "拒否(目安)" ? "目" : r.verdict === "未解決" ? "未" : "落";
function phaseOf(r, t) {   // いちばん近い所の時刻がどの手のどの段か
  for (const p of r.phase || []) for (const k of PH) { const e = p[k]; if (e && e[5] && e[4] === t) return `${p.name} の${PHJ[k]}`; }
  return "";
}
function worstAt(r) {   // 段の表から、ほかのピース・その手のピースのいちばん近い所
  let w = null;
  (r.phase || []).forEach(p => PH.forEach(k => {
    const e = p[k]; if (!e) return;
    if (e[0] !== null && (!w || e[0] < w.v)) w = { v: e[0], who: `${p.name} の${PHJ[k]}・${e[1]}` };
    if (e[3] !== null && (!w || e[3] < w.v)) w = { v: e[3], who: `${p.name} の${PHJ[k]}・自分のピース` };
  }));
  return w;
}
const L = [];
L.push("# S1 寸法と把持の可否（sim の計画だけ）", "");
L.push("`tools/s1-sweep.js` の結果（`outputs/s1/*.jsonl`）。表の印：ok = 経路が出て当たり無し、ok小 = ok だが全体のいちばん近い所が " + SMALL + "mm【仮】未満、拒 = 寸法と入力で決まる断り、目 = 決めた定数の線（目安）で断る（理由の欄に線の名前）、未 = 探しきれない・当たりが残る。格子では高さ = 辺（立方体）。置き場は sim の既定の置き方。", "");

const grid = read("grid_set2.jsonl").concat(read("grid_set63.jsonl"));
const cellsOf = [...new Set(grid.map(r => r.ui.CELL))].sort((a, b) => a - b), gapsOf = [...new Set(grid.map(r => r.ui.GAP))].sort((a, b) => a - b);
const off = [];
for (const set of [2, 6, 3]) {
  const rows = grid.filter(r => r.ui.SET === set);
  if (!rows.length) continue;
  L.push(`## ${SETNAME[set]}`, "", "辺＼すき間 | " + gapsOf.join(" | "), "|---" + "|---".repeat(gapsOf.length) + "|");
  for (const c of cellsOf) {
    L.push(`${c} | ` + gapsOf.map(g => {
      const r = rows.find(x => x.ui.CELL === c && x.ui.GAP === g);
      if (!r) return " ";
      const d = r.dims_mm;
      if (d) {
        const est = g < d.fitGap - 1e-6 || (NEED2[set] && 2 * c + g > d.maxSpan + 1e-6) ? "断る" : "通る";
        const got = r.verdict === "ok" || r.verdict === "未解決" ? "通る" : "断る";
        if (est !== got || r.verdict === "未解決") off.push({ set, c, g, est, r });
      }
      return tag(r);
    }).join(" | "));
  }
  const cnt = k => rows.filter(r => r.verdict === k).length;
  L.push("", `ok ${cnt("ok")}（うち ok小 ${rows.filter(r => tag(r) === "ok小").length}）・拒否 ${cnt("拒否")}・拒否(目安) ${cnt("拒否(目安)")}・未解決 ${cnt("未解決")}（${rows.length} 回）。`, "");
}
if (off.length) {
  L.push("### 線だけの目安と違った回・未解決の回", "", "| 盤 | 辺 | すき間 | 目安 | 結果 | 指の厚さ+余裕 | 挟める幅 | 理由（線） |", "|---|---|---|---|---|---|---|---|");
  off.forEach(o => L.push(`| ${o.set} | ${o.c} | ${o.g} | ${o.est} | ${o.r.verdict} | ${sh(o.r.dims_mm.fitGap)} | ${sh(o.r.dims_mm.maxSpan)} | ${((o.r.why || "") + (o.r.rule ? "（" + o.r.rule + "）" : "")).slice(0, 150).replace(/\|/g, "/")} |`));
  L.push("");
}
const dimRows = cellsOf.map(c => (grid.find(r => r.ui.CELL === c && r.dims_mm) || {}).dims_mm).filter(Boolean);
if (dimRows.length) {
  L.push("### 辺ごとの爪の寸法（高さ = 辺。指の両側の余裕 2mm）", "", "| 辺 | 指の厚さ | 指が入るすき間 | 挟める幅（目安の線） |", "|---|---|---|---|");
  dimRows.forEach(d => L.push(`| ${d.cell} | ${sh(d.fingerT)} | ${sh(d.fitGap)} | ${sh(d.maxSpan)} |`));
  L.push("");
}
const okRows = grid.filter(r => r.verdict === "ok").concat(read("shapes.jsonl").filter(r => r.verdict === "ok"));
if (okRows.length) {
  L.push("## 段ごとのいちばん近い所（ok の回。ほかのピースまで mm。「—」は 20mm 以上）", "",
         "全体 = 当たり判定のいちばん近い所（接する組を除く。机・台・自分のピースを含む）。自分 = その手のピースに、触れていない間に近づいた所。", "",
         "| 盤 | 辺 | すき間 | 全体（組・段） | " + PH.map(k => PHJ[k]).join(" | ") + " | 自分 |", "|---|---|---|---|" + "---|".repeat(PH.length + 1));
  okRows.forEach(r => {
    const mins = PH.map(k => { const v = r.phase.map(p => p[k][0]).filter(x => x !== null); return v.length ? Math.min(...v) : null; });
    const own = r.phase.map(p => Math.min(...PH.map(k => p[k][3] === null ? Infinity : p[k][3]))).filter(isFinite);
    L.push(`| ${r.ui.SET} | ${r.ui.CELL ?? 14} | ${r.ui.GAP ?? 15} | ${sh(r.min_clear_mm)}（${r.min_clear_pair}・${phaseOf(r, r.min_clear_t) || r.min_clear_t.toFixed(2) + " 秒"}） | ${mins.map(sh).join(" | ")} | ${own.length ? sh(Math.min(...own)) : "—"} |`);
  });
  L.push("");
}
for (const [f, title] of [["height.jsonl", "高さ（盤 2・辺 14・すき間 15）"], ["margin.jsonl", "指の両側の余裕（盤 2・辺 14・すき間 15）"], ["grasp.jsonl", "つかみ方の番号（盤 2・既定の寸法）"], ["shapes.jsonl", "盤の形（既定の寸法）"]]) {
  const rows = read(f);
  if (!rows.length) continue;
  L.push(`## ${title}`, "", "| 入力 | 結果 | 理由（線） | 指の厚さ | 挟める幅 | 全体のいちばん近い所 | 段の表のいちばん近い所 |", "|---|---|---|---|---|---|---|");
  rows.forEach(r => {
    const inp = r.ui._note || Object.entries(r.ui).filter(([k]) => f === "shapes.jsonl" ? k === "SET" : !["SET", "CELL", "GAP"].includes(k)).map(([k, v]) => `${k} ${JSON.stringify(v)}`).join("・");
    const w = worstAt(r);
    L.push(`| ${inp} | ${tag(r)} | ${((r.why || "") + (r.rule ? "（" + r.rule + "）" : "")).slice(0, 130).replace(/\|/g, "/")} | ${r.dims_mm ? sh(r.dims_mm.fingerT) : ""} | ${r.dims_mm ? sh(r.dims_mm.maxSpan) : ""} | ${r.verdict === "ok" || r.verdict === "未解決" ? sh(r.min_clear_mm) + "（" + r.min_clear_pair + "）" : ""} | ${w ? sh(w.v) + "（" + w.who + "）" : ""} |`);
  });
  L.push("");
}
process.stdout.write(L.join("\n") + "\n");
