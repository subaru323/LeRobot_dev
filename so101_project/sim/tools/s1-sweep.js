// S1(総合計画書 改訂版の「寸法と把持の可否」。2026-11-22): 辺・高さ・すき間・盤の形・爪の余裕・つかみ方を振って、計画が
//   ok(経路が出る)/ 拒否(寸法で決まる。どの寸法かを理由に書く)/ 未解決(探しきれない・当たりが残る)のどれになるかと、
//   1 手ごと・段ごと(開いて下ろす・寄せて閉じる・上げる・運ぶ・下ろして置く・開く・抜く)の、ほかのピースと机までのいちばん近い所を残す。
//   node tools/s1-sweep.js <part> [--out FILE]      (sim/ で。part: grid | height | margin | grasp | shapes | default)
//     grid   … 盤 2(3×4 L J O)・6(ロの字 I L J)・3(3×3 I3 V3 V3)× 辺 10〜30mm × すき間 5〜25mm(境界の前後を含む)。--sets 2,6 で盤を選ぶ
//     height … 盤 2・辺 14・すき間 15 で、高さ 2〜30mm
//     margin … 盤 2・辺 14・すき間 15 で、指の両側の余裕 0〜5mm
//     grasp  … 盤 2・既定の寸法で、ピースごとに つかみ方の番号を 1 つずつ変える
//     shapes … 既定の寸法(辺 14・すき間 15)で盤 0〜7
//     default … 盤 2・既定の寸法を 1 回(早い確かめ)
//   phase の各段は [ほかのピース mm, 組, 机 mm, その手のピース mm(触れていない間), ほかのピースの時刻, 判定の数]。null は 20mm 以上(包む球のすき間)。
//   段の窓は [始め, 終わり)、「移る」は前の手の終わりからこの手の真上まで(scene-data.js の phaseClearOf)
//   1 行 1 つの JSON(1 回の計画)を --out(既定 ../outputs/s1/<part>.jsonl)へ書き、終わりに数を出す。計画は当たり判定の細かい測り直しあり(plan-cli の plan と同じ)。
//   置き場はこちらで決める既定の置き場(SET ごとの placeSlots)。上のカメラの台あり・手首のカメラあり・手首で確かめる、は既定のまま。
"use strict";
const fs = require("fs"), path = require("path");
const { load } = require("./_load");
load("kinematics.js", "so101.js", "motion.js", "puzzle.js", "models/so101_visuals.js", "models/so101_points.js", "models/so101_tris.js",
     "collide.js", "chikuwa-collision.js", "self-collision.js");
global.window = { SCENE_SKIP_DEFAULT: 1 }; load("scene-data.js"); delete global.window;
const M = SCENE_MODEL;
const part = process.argv[2] || "default";
const oi = process.argv.indexOf("--out");
const out = oi > 0 ? process.argv[oi + 1] : path.join(__dirname, "..", "..", "outputs", "s1", part + ".jsonl");
const si = process.argv.indexOf("--sets"), SETS = si > 0 ? process.argv[si + 1].split(",").map(Number) : [2, 6, 3];   // grid の盤(既定 2,6,3)
const PH = ["move", "open_descend", "approach_close", "lift", "carry", "place", "release", "withdraw"];
const mm = v => (isFinite(v) ? +(v * 1000).toFixed(2) : null);

function cases() {
  const C = [];
  if (part === "default") C.push({ SET: 2 });
  if (part === "grid")
    for (const SET of SETS) for (const CELL of [10, 12, 14, 15, 16, 18, 20, 25, 30]) for (const GAP of [5, 10, 13, 14, 15, 17, 20, 25]) C.push({ SET, CELL, GAP });
  if (part === "height") for (const HEIGHT of [2, 3, 4, 5, 6, 8, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 28, 30]) C.push({ SET: 2, CELL: 14, GAP: 15, HEIGHT });
  if (part === "margin") for (const _FIT_MARGIN of [0, 0.5, 1, 2, 3, 4, 5]) C.push({ SET: 2, CELL: 14, GAP: 15, _FIT_MARGIN });
  if (part === "shapes") for (let SET = 0; SET < 8; SET++) C.push({ SET });
  if (part === "grasp") {
    const d = M.buildFromUi({ SET: 2 })._check, maxRun = d.dims.maxRun;
    d.pieces.forEach(p => { const n = PUZZLE.graspOptions(p.shape, maxRun).length; for (let g = 0; g <= n; g++) C.push({ SET: 2, GRASP: { [p.name]: g }, _note: `${p.name} の つかみ方 ${g}/${n}` }); });
  }
  return C;
}

fs.mkdirSync(path.dirname(out), { recursive: true });
const fo = fs.openSync(out, "w");
const C = cases(), n = {};
C.forEach((ui, i) => {
  const t0 = Date.now();
  let c, err = null;
  try { c = M.buildFromUi(Object.assign({}, ui))._check; } catch (e) { err = String(e && e.stack || e); }
  const row = { i, ui, sec: (Date.now() - t0) / 1000 };
  if (err) { row.verdict = "落ちた"; row.why = err; }
  else {
    row.verdict = c.verdict; row.rule = c.verdictRule || ""; row.why = c.why || (c.ok ? "" : "当たりが残る"); row.ok = c.ok;
    row.dims_mm = c.dims ? Object.fromEntries(Object.entries(c.dims).map(([k, v]) => [k, k === "maxRun" ? v : mm(v)])) : null;
    row.self_min_mm = c.self ? mm(c.self.min) : null; row.hit_frames = c.hitFrames; row.min_clear_mm = mm(c.minClear); row.min_clear_pair = c.minClearPair; row.min_clear_t = c.minClearAt;
    row.T = c.T; row.order = c.order; row.grasps = (c.grasps || []).map(g => ({ name: g.name, run: g.run, gl: g.gl }));
    row.phase = (c.phaseClear || []).map(p => Object.assign({ name: p.name }, Object.fromEntries(PH.map(k => [k, [mm(p[k].pc), p[k].pcPair, mm(p[k].tb), mm(p[k].own), p[k].pcT, p[k].n]]))));
  }
  n[row.verdict] = (n[row.verdict] || 0) + 1;
  fs.writeSync(fo, JSON.stringify(row) + "\n");
  console.log(`${i + 1}/${C.length} ${JSON.stringify(ui)} → ${row.verdict}${row.why ? ": " + row.why.slice(0, 140) : ""} (${row.sec.toFixed(1)} s)`);
});
fs.closeSync(fo);
console.log("counts", JSON.stringify(n), "->", path.relative(process.cwd(), out));
