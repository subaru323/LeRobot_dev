// 台の柱に腕が当たるかを、「足跡からの距離」と「肩の軸から見た向き(腕の通り道)」のどちらがよく分けるか、を比べる。
//   (2026-11-16 の夜の記録の続き。stand-gap-sweep.js で、足跡から 40〜45mm でも当たる置き場が出た → 距離では決まらないらしい)
//   仮説: 腕(肩・上腕・肘・前腕)は、上から見ると肩の回転軸から置き場へ向かう細い帯の中にある。
//         だから「その帯(幅 2w)が、肩から置き場の少し先までの間で、柱の足跡と重なるか」で当たりが決まる。
//   外れなら: 帯の判定で分けても、距離で分けるのと同じくらいしか当たらない(どの幅 w でも)。
//   世界: stand-gap-sweep.js と同じ(3×4・こちらで決めた置き場から O だけを柱のまわりの格子へ。向き 0°・45°。置き場の決まり 20mm は外す)。
//         爪のまわりの細かい判定(S-8)は台との距離に関係しないので切る(NO_REFINE。速くするため)。
//   出すもの: 解けた置き場ごとに 台に当たったか(台との距離 < 0)、足跡からの距離、肩から見た帯が柱と重なるか。
//            2 つの分け方それぞれで、いちばんよく分けるしきい値と、そのときの取り違えの数。
//   ★負の検算: (1) 当たりの印をでたらめに入れ替えると、帯の分け方の取り違えが増えること(分け方が印を見ていること)。
//              (2)(3) 肩の位置を左・右へ 150mm ずらした(うその)帯で分けると、取り違えが増えること(肩の位置を使っていること。増え方は小さい)。
//              (4) 帯を逆向き(置き場から肩と反対の向き)にすると、取り違えが大きく増えること(通り道を見ていること)。
//   実行: node tools/stand-path.js [--step 10] [--in out/stand-sweep.json]   (--in があれば掃き直さずにファイルから読む)
const fs = require("fs"), path = require("path");
const { load } = require("./_load");
load("kinematics.js", "so101.js", "motion.js", "puzzle.js",
     "models/so101_visuals.js", "models/so101_points.js", "models/so101_tris.js", "collide.js", "scene-data.js");
const M = SCENE_MODEL, def = M.defaultUiValues();
// 2026-11-17 に上のカメラの台を置き方 C へ替えた(K-12)。この道具は柱の足跡をいまの場面から取るので、A の記録(out/stand-sweep.json)と食い違う。
//   C の掃き出しは tools/pole-sweep.js。A のときの結果を作り直すには so101.js・scene-data.js を A に戻してから --force を付けて流す
if (process.argv.indexOf("--force") < 0) { console.log("★この道具は置き方 A の柱を調べたもの。いまの台は C(tools/pole-sweep.js を使う)。流すなら --force"); process.exit(2); }
const arg = k => { const i = process.argv.indexOf("--" + k); return i > 0 ? process.argv[i + 1] : undefined; };
const STEP = +(arg("step") ?? 10) / 1000;
const ui = { SET: 2, GAP: def.GAP, SOL: 0, NO_REFINE: true };
const base = M.buildFromUi(ui)._check, FOOT = base.foot, CELL = base.cell, P = CELL + def.GAP / 1000;
const POSTS = FOOT.slice(1);                       // 足跡の 1 つ目は腕を載せる板(肩はその上)。2 つ目から柱の部品
const SH = [SO101.joints[0].origin[0], SO101.joints[0].origin[2]];   // 肩の回転軸(上から見た x, z。机 = 0 の座標でも同じ)
const O = 2, cellsO = PUZZLE.SHAPES.O;
const mm = x => (x * 1000).toFixed(1);
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
// ---- 掃く(または読む) ----
let rows;
const inFile = arg("in");
if (inFile) rows = JSON.parse(fs.readFileSync(inFile, "utf8")).rows;
else {
  const post = POSTS.reduce((r, f) => [Math.min(r[0], f[0]), Math.min(r[1], f[1]), Math.max(r[2], f[2]), Math.max(r[3], f[3])], [Infinity, Infinity, -Infinity, -Infinity]);
  rows = [];
  const t0 = Date.now();
  for (const yaw of [0, 45]) {
    for (let x = post[0] - 0.08; x <= post[2] + 0.08 + 1e-9; x += STEP) for (let z = post[1] - 0.08; z <= post[3] + 0.08 + 1e-9; z += STEP) {
      const c = Math.cos(yaw * Math.PI / 180), s = Math.sin(yaw * Math.PI / 180), m = P / 2;
      const o = [x - (m * c - m * s), z - (m * s + m * c)];
      const g = Math.min(...FOOT.map(f => gap(rectOf(o, yaw), f)));
      if (g < 0 || g > 0.045) continue;
      const L = { pieces: base.slots.map((sl, i) => i === O ? { x: o[0] * 1000, z: o[1] * 1000, yaw } : { x: sl[0] * 1000, z: sl[1] * 1000, yaw: 0 }) };
      if (M.buildFromUi(Object.assign({ LAYOUT: L, LAYOUT_ONLY: true, NO_AVOID: true }, ui))._check.layoutWhy) continue;
      const r = M.buildFromUi(Object.assign({ LAYOUT: L, NO_AVOID: true }, ui))._check;
      rows.push({ yaw, cx: x, cz: z, g, solved: r.solved, minStand: r.minStandPair ? r.minStand : null, pair: r.minStandPair || null });
    }
  }
  const out = path.join(__dirname, "..", "out", "stand-sweep.json");
  fs.writeFileSync(out, JSON.stringify({ step: STEP, made: new Date().toISOString(), rows }, null, 0));
  console.log(`掃いた: ${rows.length} 置き場・${((Date.now() - t0) / 1000).toFixed(0)} 秒。${path.relative(process.cwd(), out)} に書いた`);
}
const S = rows.filter(r => r.solved);
const hit = r => r.minStand !== null && r.minStand < 0;
console.log(`[台の柱] 足跡 ${POSTS.length} 個・肩の軸 x ${mm(SH[0])}・z ${mm(SH[1])}mm。試した ${rows.length}・解けた ${S.length}・台に当たった ${S.filter(hit).length}`);
// ---- 帯の判定: 肩 sh から置き場の真ん中 (cx, cz) の向きに、長さ (距離 + ext)、幅 2w の帯が、柱の足跡と重なるか ----
function bandHits(r, w, ext, sh, rev) {
  const dx = r.cx - sh[0], dz = r.cz - sh[1], L = Math.hypot(dx, dz), ux = dx / L, uz = dz / L;
  if (rev) sh = [r.cx, r.cz];                    // 逆向き: 置き場から、肩と反対の向きへ同じ長さ(負の検算 4)
    return POSTS.some(f => {
    // 長方形の 4 隅を帯の座標(前 t・横 n)へ。長方形と帯の重なりは、4 隅の t の範囲・n の範囲だけでは決まらないので、
    //   長方形の上の点を 1mm おきに打って、どれかが帯の中にあるかで見る(柱の足跡は 100mm 角ほどなので数万点で済む)
    for (let x = f[0]; x <= f[2] + 1e-9; x += 0.001) for (let z = f[1]; z <= f[3] + 1e-9; z += 0.001) {
      const px = x - sh[0], pz = z - sh[1], t = px * ux + pz * uz, n = -px * uz + pz * ux;
      if (t >= 0 && t <= L + ext && Math.abs(n) <= w) return true;
    }
    return false;
  });
}
// しきい値 1 つで 2 つに分けるときの、いちばん少ない取り違え(値が小さい方を「当たる」とみなす)
function bestSplit(vals, labels) {
  const idx = vals.map((v, i) => i).sort((a, b) => vals[a] - vals[b]);
  const P1 = labels.filter(Boolean).length;
  let best = { err: P1, th: -Infinity }, tp = 0, fp = 0;           // しきい値より下を全部「当たらない」とした場合から
  idx.forEach((i, k) => {
    if (labels[i]) tp++; else fp++;
    const err = (P1 - tp) + fp;
    if (err < best.err && (k === idx.length - 1 || vals[idx[k + 1]] > vals[i])) best = { err, th: vals[i] };
  });
  return best;
}
const lab = S.map(hit);
const dist = bestSplit(S.map(r => r.g), lab);
console.log(`距離で分ける: 足跡から ${mm(dist.th)}mm 以下を「当たる」とすると、取り違え ${dist.err}/${S.length}`);
const half = [5, 10, 15, 20, 25, 30, 40].map(x => x / 1000), exts = [0, 20, 40, 60].map(x => x / 1000);
function bandBest(labels, sh, rev) {
  let best = null;
  for (const w of half) for (const ext of exts) {
    const pred = S.map(r => bandHits(r, w, ext, sh, rev));
    const err = pred.reduce((e, p, i) => e + (p !== labels[i] ? 1 : 0), 0);
    if (!best || err < best.err) best = { err, w, ext, pred };
  }
  return best;
}
const band = bandBest(lab, SH);
const tp = band.pred.filter((p, i) => p && lab[i]).length, fp = band.pred.filter((p, i) => p && !lab[i]).length, fn = band.pred.filter((p, i) => !p && lab[i]).length;
console.log(`帯で分ける: 幅 ±${mm(band.w)}mm・置き場の先 ${mm(band.ext)}mm まで、で取り違え ${band.err}/${S.length}(帯が重なり当たった ${tp}・重なるのに当たらない ${fp}・重ならないのに当たった ${fn})`);
// ---- 点検 C12 で足した比べ(2026-11-17) ----
// (C12-2) 距離は柱の足跡だけで測り直す(上の g は腕を載せる板との距離も含む)
const gPost = S.map(r => { const c = Math.cos(r.yaw * Math.PI / 180), s = Math.sin(r.yaw * Math.PI / 180), m = P / 2;
  const o = [r.cx - (m * c - m * s), r.cz - (m * s + m * c)]; return Math.min(...POSTS.map(f => gap(rectOf(o, r.yaw), f))); });
const distPost = bestSplit(gPost, lab);
console.log(`柱だけとの距離で分ける: ${mm(distPost.th)}mm 以下を「当たる」とすると、取り違え ${distPost.err}/${S.length}`);
{ const hp = gPost.filter((g, i) => lab[i]), th0 = Math.max(...hp);
  console.log(`  いまの決まり(足跡から 20mm 以上は置いてよい)で、20mm 以上なのに当たった: ${S.filter((r, i) => r.g >= 0.02 && lab[i]).length}/${S.filter(r => r.g >= 0.02).length}。` +
    `柱だけとの距離で見落とし 0 にするには ${mm(th0)}mm より遠くに置く必要があり、置けるのは ${gPost.filter(g => g > th0).length}/${S.length}`); }
// (C12-2) 肩から見た向き 1 つで分ける: 置き場の真ん中への向きと、柱の足跡の真ん中への向きの角度差
const pc = POSTS[0], pAng = Math.atan2((pc[1] + pc[3]) / 2 - SH[1], (pc[0] + pc[2]) / 2 - SH[0]);
const angDiff = S.map(r => { let d = Math.atan2(r.cz - SH[1], r.cx - SH[0]) - pAng; d = Math.atan2(Math.sin(d), Math.cos(d)); return Math.abs(d) * 180 / Math.PI; });
const ang = bestSplit(angDiff, lab);
console.log(`肩から見た向きで分ける: 柱の真ん中の向き ${(pAng * 180 / Math.PI).toFixed(1)}° との差 ${ang.th.toFixed(1)}° 以下を「当たる」とすると、取り違え ${ang.err}/${S.length}`);
// (C12-1) 置き場の決まりに使うなら「当たるのに通す」(見落とし)を 0 にしたい。幅ごとに、見落とし・言い過ぎ・置ける数を出す
console.log(`  幅ごと(先の長さ 0mm。先の長さ 0〜60mm はどの幅でも同じ数だった): 幅 / 取り違え / 見落とし(当たるのに重ならない) / 置ける(重ならない)`);
for (const w of half.concat([0.05])) {
  const pr = S.map(r => bandHits(r, w, 0, SH)), miss = pr.filter((p, i) => !p && lab[i]).length, err = pr.filter((p, i) => p !== lab[i]).length;
  console.log(`    ±${mm(w)}mm: ${err} / ${miss} / ${pr.filter(p => !p).length}`);
}
{ const idx = S.map((r, i) => i).filter(i => lab[i]); const th0 = Math.max(...idx.map(i => angDiff[i]));
  console.log(`  向きで見落とし 0 にするには差 ${th0.toFixed(1)}° 以下を全部「当たる」とする必要があり、置けるのは ${angDiff.filter(a => a > th0).length}/${S.length}`); }
// (C12-3) 向き 0° で選んだ幅を 45° で試す(と逆)
for (const [a, b] of [[0, 45], [45, 0]]) {
  const A = S.map((r, i) => i).filter(i => S[i].yaw === a), B = S.map((r, i) => i).filter(i => S[i].yaw === b);
  let bw = null, be = Infinity;
  for (const w of half) { const e = A.filter(i => bandHits(S[i], w, 0, SH) !== lab[i]).length; if (e < be) { be = e; bw = w; } }
  const eb = B.filter(i => bandHits(S[i], bw, 0, SH) !== lab[i]).length;
  console.log(`  ピースの向き ${a}° で幅を選ぶと ±${mm(bw)}mm(取り違え ${be}/${A.length})。それを向き ${b}° で試すと ${eb}/${B.length}`);
}
// 足跡からの距離ごとに、帯と当たりを並べる
for (let lo = 0; lo < 0.045; lo += 0.01) {
  const b = S.map((r, i) => ({ r, i })).filter(o => o.r.g >= lo && o.r.g < lo + 0.01);
  if (!b.length) continue;
  const h = b.filter(o => lab[o.i]).length, ph = b.filter(o => band.pred[o.i]).length, agree = b.filter(o => band.pred[o.i] === lab[o.i]).length;
  console.log(`  足跡から ${(lo * 1000).toFixed(0)}〜${(lo * 1000 + 10).toFixed(0)}mm: 解けた ${b.length}・当たった ${h}・帯が重なる ${ph}・帯と当たりが合う ${agree}`);
}
let ng = 0;
// ★負の検算(1): 当たりの印をでたらめに入れ替える(種 1)→ 帯の取り違えは増えるはず
let seed = 1; const rnd = () => (seed = (seed * 16807) % 2147483647) / 2147483647;
const shuf = lab.slice(); for (let i = shuf.length - 1; i > 0; i--) { const j = Math.floor(rnd() * (i + 1)); [shuf[i], shuf[j]] = [shuf[j], shuf[i]]; }
const bandShuf = bandBest(shuf, SH);
// 種を変えて 50 回入れ替える(点検 C12 の 3。1 回だけでは運かもしれない)
const shufErr = [];
for (let k = 0; k < 50; k++) { const sh2 = lab.slice(); for (let i = sh2.length - 1; i > 0; i--) { const j = Math.floor(rnd() * (i + 1)); [sh2[i], sh2[j]] = [sh2[j], sh2[i]]; } shufErr.push(bandBest(sh2, SH).err); }
shufErr.sort((a, b) => a - b);
const ok1 = bandShuf.err > band.err && shufErr[0] > band.err;
console.log(`負の検算(1) 当たりの印を入れ替える: 帯の取り違え ${bandShuf.err}/${S.length}(元 ${band.err})。さらに 50 回: いちばん少ない ${shufErr[0]}・真ん中 ${shufErr[25]} → ${ok1 ? "OK(増えた)" : "★NG(増えない = 帯は印を見ていない)"}`);
// ★負の検算(2): 肩の位置を左(z の負)へ 150mm ずらした帯 → 取り違えは増えるはず
const bandMir = bandBest(lab, [SH[0], SH[1] - 0.15]);
const ok2 = bandMir.err > band.err;
console.log(`負の検算(2) 肩を左へ 150mm ずらした帯: 取り違え ${bandMir.err}/${S.length}(元 ${band.err})→ ${ok2 ? "OK(増えた)" : "★NG(増えない = 向きを使っていない)"}`);
// ★負の検算(3): 肩を右(柱と反対)へ 150mm ずらした帯 → 取り違えは増えるはず((2) は柱の側へずらすので、向きが少ししか変わらない)
const bandR = bandBest(lab, [SH[0], SH[1] + 0.15]);
const ok3 = bandR.err > band.err;
console.log(`負の検算(3) 肩を右へ 150mm ずらした帯: 取り違え ${bandR.err}/${S.length}(元 ${band.err})→ ${ok3 ? "OK(増えた)" : "★NG(増えない = 向きを使っていない)"}`);
// ★負の検算(4): 帯を逆向き(置き場から、肩と反対の向きへ同じ長さ)にする → 「通り道の上か」を見ているなら、取り違えは大きく増えるはず
const bandRev = bandBest(lab, SH, true);
const ok4 = bandRev.err > band.err;
console.log(`負の検算(4) 帯を逆向き(置き場の向こう側): 取り違え ${bandRev.err}/${S.length}(元 ${band.err})→ ${ok4 ? "OK(増えた)" : "★NG(増えない = 通り道を見ていない)"}`);
// 参考: 何も見ずに全部「当たらない」/全部「当たる」とした取り違え
const nHit = lab.filter(Boolean).length;
console.log(`参考: 全部「当たらない」とすると取り違え ${nHit}/${S.length}、全部「当たる」とすると ${S.length - nHit}/${S.length}`);
if (!ok1) ng++; if (!ok2) ng++; if (!ok3) ng++; if (!ok4) ng++;
const verdict = band.err < distPost.err ? "帯(= 肩から見た向き)の方がよく分ける(仮説に合う)" : "帯は距離よりよく分けない(仮説は外れ)";
console.log(`結論: 柱だけとの距離 ${distPost.err}・帯 ${band.err}・向き 1 つ ${ang.err} → ${verdict}。どれも見落とし 0 にすると置ける所は大きく減る(上の表)`);
console.log(ng ? `NG ${ng} 件` : "OK(負の検算 4 つとも取り違えが増えた)");
process.exit(ng ? 1 : 0);
