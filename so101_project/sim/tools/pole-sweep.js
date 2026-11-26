// 上のカメラの置き方 C(柱・足の金具・吊る腕・吊る板・カメラ。so101.js の OVERHEAD_C)に、ピースを掴みに行く腕が当たる置き場を掃き出す。
//   (2026-11-17 に置き方を A から C へ替えたとき(K-12)。A の柱で同じことをした tools/stand-path.js → out/stand-sweep.json と同じやり方)
//   見ている世界: 3×4(L J O)の、こちらで決めた置き場から、O だけを C の柱の足跡のまわりの格子へ動かす(向き 0°・45°)。
//     置き場の決まり(足跡から 20mm)は外し(NO_AVOID)、置き場の選び直しも止める(NO_RESLOT。選び直すと O が格子の点から動く)。
//     爪のまわりの細かい判定(S-8)は台との距離に関係しないので切る(NO_REFINE。速くするため)。
//     足跡から 0〜45mm の点だけを見る。ほかのピース・盤に重なる点は数えて飛ばす。届く範囲の外のマスがある点は、試して「解けない」に入る。
//   出すもの: out/pole-sweep.json(stand-sweep.json と同じ形の行 + why)と、試した数・届かない数・解けた数・台に当たった数(当たった部品ごと)、
//     決まり(足跡から 20mm 以上は置いてよい)なのに当たった数。
//   ★負の検算: 台を作業域へずらした(STAND_SHIFT [-180, 30]mm)同じ条件の場面では台に当たること(この道具の条件で台の判定が生きていること)。
//   実行: node tools/pole-sweep.js [--step 10] [--in out/pole-sweep.json] [--out FILE]   (掃くと 25 分ほど。--in があれば掃き直さずにファイルから読む。
//         --out は書く先。既定は out/pole-sweep.json。sim/ からの相対でなく、流した場所からの相対)
const fs = require("fs"), path = require("path");
const { load } = require("./_load");
load("kinematics.js", "so101.js", "motion.js", "puzzle.js",
     "models/so101_visuals.js", "models/so101_points.js", "models/so101_tris.js", "collide.js",
     "chikuwa-collision.js", "self-collision.js", "scene-data.js");
const M = SCENE_MODEL, def = M.defaultUiValues();
const arg = k => { const i = process.argv.indexOf("--" + k); return i > 0 ? process.argv[i + 1] : undefined; };
const STEP = +(arg("step") ?? 10) / 1000;
const ui = { SET: 2, GAP: def.GAP, SOL: 0, NO_REFINE: true, NO_RESLOT: true };
const base = M.buildFromUi(ui)._check, FOOT = base.foot, CELL = base.cell, P = CELL + def.GAP / 1000;
// C の部品の足跡(運ぶ高さ + 30mm より下にある所。いまは柱と柱の足の金具)。2026-11-19 まで足跡の 1 つ目は腕を載せる板(肩はその上)で、外していた。
//   C の台の設計(本家の板なし)にしてからは、足跡は全部 C の部品
const RIG = M.STAND_PARTS.some(s => s.id === "st_plate") ? FOOT.slice(1) : FOOT;
const O = 2, cellsO = PUZZLE.SHAPES.O;
const mm = x => (x * 1000).toFixed(1);
let ng = 0;
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
// ---- 負の検算: 台を作業域へずらすと、この条件(NO_REFINE・NO_RESLOT・NO_AVOID)でも台に当たるはず ----
{
  const r = M.buildFromUi(Object.assign({ NO_AVOID: true, STAND_SHIFT: [-180, 30] }, ui))._check;
  if (!(r.solved && r.minStand < 0)) { ng++; console.log(`★NG(負の検算): 台を作業域へずらしても台に当たらない(台との距離 ${r.minStandPair ? mm(r.minStand) + "mm " + r.minStandPair : "15mm より遠い"})`); }
  else console.log(`負の検算 OK: 台を [-180, 30]mm ずらすと ${r.minStandPair} ${mm(r.minStand)}mm(この条件でも台の判定は生きている)`);
}
// ---- 掃く ----
if (!RIG.length) { console.log("★NG: C の部品の足跡が無い(台が出ていないか、足跡の高さの決まりが変わった)"); process.exit(1); }
const box = RIG.reduce((r, f) => [Math.min(r[0], f[0]), Math.min(r[1], f[1]), Math.max(r[2], f[2]), Math.max(r[3], f[3])], [Infinity, Infinity, -Infinity, -Infinity]);
console.log(`[C の足跡] x ${mm(box[0])}〜${mm(box[2])}mm・z ${mm(box[1])}〜${mm(box[3])}mm(机 = 0、x は前、z は右)。O を格子 ${STEP * 1000}mm で動かす`);
let rows = [], skip = { overlap: 0, other: 0 };
const t0 = Date.now(), inFile = arg("in");
const out = arg("out") ? path.resolve(arg("out")) : path.join(__dirname, "..", "out", "pole-sweep.json");   // --out: 書く先(2026-11-20。隣の会話の頼み。夜の作業で既定のファイルを上書きしないため)
if (arg("out") !== undefined && inFile) { console.log("★--in と --out は一緒に使えない(--in のときは書かない)"); process.exit(1); }
if (inFile) { const j = JSON.parse(fs.readFileSync(inFile, "utf8")); rows = j.rows; skip = j.skip; console.log(`${inFile}(${j.made})から読んだ。掃き直さない`); }
else for (const yaw of [0, 45]) {
  for (let x = box[0] - 0.08; x <= box[2] + 0.08 + 1e-9; x += STEP) for (let z = box[1] - 0.08; z <= box[3] + 0.08 + 1e-9; z += STEP) {
    const c = Math.cos(yaw * Math.PI / 180), s = Math.sin(yaw * Math.PI / 180), m = P / 2;
    const o = [x - (m * c - m * s), z - (m * s + m * c)];
    const g = Math.min(...RIG.map(f => gap(rectOf(o, yaw), f)));   // C の部品の足跡までの距離(腕を載せる板は含めない。点検 C27 の 4)
    if (g < 0 || g > 0.045 || FOOT.some(f => gap(rectOf(o, yaw), f) < 0)) continue;
    const L = { pieces: base.slots.map((sl, i) => i === O ? { x: o[0] * 1000, z: o[1] * 1000, yaw } : { x: sl[0] * 1000, z: sl[1] * 1000, yaw: 0 }) };
    const lq = M.buildFromUi(Object.assign({ LAYOUT: L, LAYOUT_ONLY: true, NO_AVOID: true }, ui))._check, lw = lq.layoutWhy;
    if (lw) { if (/重な/.test(lw)) skip.overlap++; else skip.other++; continue; }
    const far = (lq.layoutNotes || []).some(n => /届く範囲/.test(n));   // 届く範囲の外のマスがある(解けるかは逆運動学しだい)
    const r = M.buildFromUi(Object.assign({ LAYOUT: L, NO_AVOID: true }, ui))._check;
    rows.push({ yaw, cx: x, cz: z, g, far, solved: r.solved, why: r.solved ? "" : r.why, minStand: r.minStandPair ? r.minStand : null, pair: r.minStandPair || null });
  }
}
if (!inFile) fs.writeFileSync(out, JSON.stringify({ step: STEP, made: new Date().toISOString(), rig: "C", skip, rows }, null, 0));
const S = rows.filter(r => r.solved), hit = S.filter(r => r.minStand !== null && r.minStand < 0);
console.log(`試した ${rows.length}(うち届く範囲の外のマスがある ${rows.filter(r => r.far).length}。ほかに ピース・盤に重なる ${skip.overlap}・その他 ${skip.other} を飛ばした)・解けた ${S.length}・台に当たった ${hit.length}` +
  (inFile ? "" : `  ${((Date.now() - t0) / 1000).toFixed(0)} 秒。${path.relative(process.cwd(), out)} に書いた`));
const unsolved = rows.filter(r => !r.solved);   // 解けない理由は、位置の差の数字を落とし、最初の理由でまとめる
if (unsolved.length) { const w = {}; unsolved.forEach(r => { const k = (r.why || "").replace(/[(（]位置の差[^)）]*[)）]/g, "").replace(/\s*\/.*$/, ""); w[k] = (w[k] || 0) + 1; });
  console.log("  解けない理由: " + Object.entries(w).map(([k, v]) => `${k} ${v}`).join(" / ")); }
const by = {}; hit.forEach(r => { const p = r.pair.split("|")[1]; by[p] = (by[p] || 0) + 1; });
console.log(`  当たった台の部品: ${Object.entries(by).map(([k, v]) => k + " " + v).join("・") || "なし"}`);
for (let lo = 0; lo < 0.045; lo += 0.005) {
  const b = S.filter(r => r.g >= lo && r.g < lo + 0.005);
  if (!b.length) continue;
  const h = b.filter(r => r.minStand !== null && r.minStand < 0), ds = b.filter(r => r.minStand !== null).map(r => r.minStand);
  console.log(`  足跡から ${(lo * 1000).toFixed(0)}〜${(lo * 1000 + 5).toFixed(0)}mm: 解けた ${b.length}・台に当たった ${h.length}・台にいちばん近い ${ds.length ? mm(Math.min(...ds)) + "mm" : "15mm より遠い"}` +
    (h.length ? `(${[...new Set(h.map(r => r.pair))].join("・")})` : ""));
}
const ruleMiss = hit.filter(r => r.g >= 0.02);
console.log(`決まり(足跡から 20mm 以上は置いてよい)なのに当たった: ${ruleMiss.length}/${S.filter(r => r.g >= 0.02).length}` +
  (ruleMiss.length ? `(いちばん遠い ${mm(Math.max(...ruleMiss.map(r => r.g)))}mm)` : ""));
process.exit(ng ? 1 : 0);
