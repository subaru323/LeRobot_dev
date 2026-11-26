// 台の足跡から何 mm 空ければ、ピースを掴むときに腕(手首のカメラを含む)が台に当たらないか、を掃いて測る。
//   (点検 C11 の 1 の続き。置き場は台の足跡から SLOT_CLEAR = 20mm 空けているが、20mm で足りない置き場がファイルの試しで出た)
//   見ている世界: 3×4(L J O)の、こちらで決めた置き場から、O だけを台の柱のまわりの格子へ動かす(向き 0°・45°)。
//     置き場の足跡の決まり(20mm)は外し(NO_AVOID)、ほかのピース・盤との重なりは理由を出して飛ばす。
//   出すもの: 格子の点ごとに、O の外側の長方形と台の足跡の距離・解けたか・腕(と持ったピース)と台のいちばん近い所。
//     最後に「台に当たった置き場の、足跡からのいちばん遠い距離」= それより空ければ、この試しでは当たらない。
//   ★この試しは O だけ・向き 2 つ・格子 10mm。L・J のような長い形や、ほかの向きはまだ。
//   実行: node tools/stand-gap-sweep.js [--step 10]
const { load } = require("./_load");
load("kinematics.js", "so101.js", "motion.js", "puzzle.js",
     "models/so101_visuals.js", "models/so101_points.js", "models/so101_tris.js", "collide.js", "scene-data.js");
const M = SCENE_MODEL, def = M.defaultUiValues();
// 2026-11-17 に上のカメラの台を置き方 C へ替えた(K-12)。この道具は柱の足跡をいまの場面から取るので、A の記録(out/stand-sweep.json)と食い違う。
//   C の掃き出しは tools/pole-sweep.js。A のときの結果を作り直すには so101.js・scene-data.js を A に戻してから --force を付けて流す
if (process.argv.indexOf("--force") < 0) { console.log("★この道具は置き方 A の柱を調べたもの。いまの台は C(tools/pole-sweep.js を使う)。流すなら --force"); process.exit(2); }
const arg = k => { const i = process.argv.indexOf("--" + k); return i > 0 ? process.argv[i + 1] : undefined; };
const STEP = +(arg("step") ?? 10) / 1000;
const ui = { SET: 2, GAP: def.GAP, SOL: 0 };
const base = M.buildFromUi(ui)._check, FOOT = base.foot, CELL = base.cell, P = CELL + def.GAP / 1000;
const O = 2, cellsO = PUZZLE.SHAPES.O;
function rectOf(xz, yaw) {
  const c = Math.cos(yaw * Math.PI / 180), s = Math.sin(yaw * Math.PI / 180), h = CELL / 2 * (Math.abs(c) + Math.abs(s));
  const r = [Infinity, Infinity, -Infinity, -Infinity];
  cellsO.forEach(cl => { const x = xz[0] + cl[0] * P * c - cl[1] * P * s, z = xz[1] + cl[0] * P * s + cl[1] * P * c;
    r[0] = Math.min(r[0], x - h); r[1] = Math.min(r[1], z - h); r[2] = Math.max(r[2], x + h); r[3] = Math.max(r[3], z + h); });
  return r;
}
function gap(a, b) {
  const dx = Math.max(a[0] - b[2], b[0] - a[2], 0), dz = Math.max(a[1] - b[3], b[1] - a[3], 0);
  return a[0] < b[2] && b[0] < a[2] && a[1] < b[3] && b[1] < a[3] ? -1 : Math.hypot(dx, dz);
}
// 柱(足跡の 2 つ目以降 = 柱の部品)のまわり 80mm を格子で
const post = FOOT.slice(1).reduce((r, f) => [Math.min(r[0], f[0]), Math.min(r[1], f[1]), Math.max(r[2], f[2]), Math.max(r[3], f[3])], [Infinity, Infinity, -Infinity, -Infinity]);
console.log(`[台の柱の足跡] x ${(post[0] * 1000).toFixed(0)}〜${(post[2] * 1000).toFixed(0)}mm・z ${(post[1] * 1000).toFixed(0)}〜${(post[3] * 1000).toFixed(0)}mm(机 = 0、x は前、z は右)。O を格子 ${STEP * 1000}mm で動かす`);
const rows = [];
let tried = 0;
for (const yaw of [0, 45]) {
  for (let x = post[0] - 0.08; x <= post[2] + 0.08 + 1e-9; x += STEP) for (let z = post[1] - 0.08; z <= post[3] + 0.08 + 1e-9; z += STEP) {
    // O の真ん中を (x, z) に置く(局所の原点は角のマスなので、真ん中から戻す)
    const c = Math.cos(yaw * Math.PI / 180), s = Math.sin(yaw * Math.PI / 180), m = P / 2;
    const o = [x - (m * c - m * s), z - (m * s + m * c)];
    const g = Math.min(...FOOT.map(f => gap(rectOf(o, yaw), f)));
    if (g < 0 || g > 0.045) continue;                       // 足跡に重なる所と、45mm より遠い所は見ない
    const L = { pieces: base.slots.map((sl, i) => i === O ? { x: o[0] * 1000, z: o[1] * 1000, yaw } : { x: sl[0] * 1000, z: sl[1] * 1000, yaw: 0 }) };
    const q = M.buildFromUi(Object.assign({ LAYOUT: L, LAYOUT_ONLY: true, NO_AVOID: true }, ui))._check;
    if (q.layoutWhy) continue;                               // ほかのピース・盤に重なる
    tried++;
    const r = M.buildFromUi(Object.assign({ LAYOUT: L, NO_AVOID: true }, ui))._check;
    rows.push({ yaw, x: o[0], z: o[1], g, solved: r.solved, why: r.why, minStand: r.minStand, pair: r.minStandPair, hit: r.hitFrames, clear: r.minClear, clearPair: r.minClearPair });
  }
}
rows.sort((a, b) => a.g - b.g);
const standHit = rows.filter(r => r.solved && r.minStand < 0);
console.log(`試した置き場 ${tried}(足跡から 0〜45mm・O の向き 0° と 45°)。解けた ${rows.filter(r => r.solved).length}・台に当たった ${standHit.length}`);
for (let lo = 0; lo < 0.045; lo += 0.005) {
  const b = rows.filter(r => r.solved && r.g >= lo && r.g < lo + 0.005);
  if (!b.length) continue;
  const h = b.filter(r => r.minStand < 0), mn = Math.min(...b.map(r => r.minStand));
  console.log(`  足跡から ${(lo * 1000).toFixed(0)}〜${(lo * 1000 + 5).toFixed(0)}mm: 解けた ${b.length}・台に当たった ${h.length}・台にいちばん近い ${isFinite(mn) ? (mn * 1000).toFixed(1) + "mm" : "15mm より遠い"}` +
    (h.length ? `(${[...new Set(h.map(r => r.pair))].join("・")})` : ""));
}
const far = standHit.reduce((m, r) => Math.max(m, r.g), -1);
console.log(far < 0 ? "台に当たった置き場は無い" :
  `台に当たった置き場の、足跡からのいちばん遠い距離: ${(far * 1000).toFixed(1)}mm(向き ${standHit.find(r => r.g === far).yaw}°・O の局所の原点 x ${(standHit.find(r => r.g === far).x * 1000).toFixed(0)}・z ${(standHit.find(r => r.g === far).z * 1000).toFixed(0)}mm)`);
