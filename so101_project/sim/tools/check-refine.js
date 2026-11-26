// S-8 の検算: 爪のまわりの測り直し(scene-data.js の fineDist)が、本当のいちばん近い距離を外していないか、
//   別の作り方(力ずく)で測って比べる。
//   fineDist … 粗い判定で近かった所のまわりだけを、1.5mm → 0.5mm の 2 段で箱の表面を探し、格子と箱で三角形を絞る。
//   力ずく   … 箱の面を全部 0.25mm 間隔で打ち(三角形の箱から 6mm 以内の所だけ)、三角形を 1 つ残らず見る(格子は使わない。
//              三角形ごとの箱までの距離が、それまでのいちばん近い値以上の三角形だけ飛ばす。これは答えを変えない)。
//   本当のいちばん近い距離を D とすると、どちらも箱の上の本物の点からの距離なので D 以上。
//   打つ間隔のぶん、力ずくは D + 0.25/√2 mm 以内、fineDist は D + 0.5/√2 mm 以内に入るはず。
//   → fine ≥ 力ずく − 0.18mm かつ fine ≤ 力ずく + 0.36mm なら合格。
//   比べる組: いちばん近い組、粗い → 細かいで縮んだ量の大きい組 5 つ、くじで選んだ組 5 つ(種 1)。
//   箱の上のいちばん近い所が「角・辺・面」のどれかも出す(角と辺では 0.5mm と 0.25mm の格子が同じ点を打つので、差が 0 になりやすい)。
//   ★外れの検算: 力ずくの側だけ箱を 1mm ずらすと、いちばん近い組で合格の幅を出ること(比べ方が鈍くないこと)。
//   この検算で言えること・言えないこと(点検 C12 の 4〜7。2026-11-17):
//   - 言えるのは「fineDist の探し方(種・2 段の格子・打ち切り)が、箱の上のいちばん近い所を外さない」まで。
//     三角形・closestOnTri・全関節 0 の姿勢へ移す式(toZero)は両方で同じ物を使うので、そこの誤りはこの検算では消える。
//   - fineDist の 0.5mm の点は全部、力ずくの 0.25mm の点に含まれる(箱の半分の辺 7mm が両方で割り切れる)ので、
//     下側「fine ≥ 力ずく − 0.18mm」は作りから必ず成り立つ。効いているのは上側だけ。
//   - 2026-11-17 の 22 組は、いちばん近い所が 19 組で箱の辺、2 組で面。爪の平らな面とピースの辺が平行に向き合う組が多く、
//     格子の粗さの誤差が出にくい形だった。外れの検算の 1mm ずらしは、幅 0.36mm に比べて大きい。
//   - 測り直した組は jaw と j5 だけで、手首のカメラ(cam)の組は 3×4・4×5(すき間 15mm・0 番)には 1 つも無かった。
//   実行: node tools/check-refine.js
const { load } = require("./_load");
load("kinematics.js", "so101.js", "motion.js", "puzzle.js",
     "models/so101_visuals.js", "models/so101_points.js", "models/so101_tris.js", "collide.js", "scene-data.js");
const M = SCENE_MODEL;
// 力ずくの側の三角形(scene-data.js の gripFields とは別に、STL の三角形から作り直す)
const TRIS = {};
Object.keys(SO101_VISUALS).forEach(link => {
  const g = SO101.LINK_GROUP[link] || (SO101.CAMERA_GROUP[link] ? "cam" : null);
  if (g !== "j5" && g !== "jaw" && g !== "cam") return;
  SO101_VISUALS[link].forEach(v => {
    const T = SO101_TRIS[v.mesh];
    if (!T) return;
    const Pv = SO101.visualPose(link, v.xyz, v.rpy);
    for (let i = 0; i < T.length; i += 9) {
      const t = [0, 3, 6].map(o => { const w = SO101.mv(Pv.R, [T[i + o] * 1e-4, T[i + o + 1] * 1e-4, T[i + o + 2] * 1e-4]); return [w[0] + Pv.p[0], w[1] + Pv.p[1], w[2] + Pv.p[2]]; });
      t.bb = [0, 1, 2].map(k => Math.min(t[0][k], t[1][k], t[2][k])).concat([0, 1, 2].map(k => Math.max(t[0][k], t[1][k], t[2][k])));
      (TRIS[g] || (TRIS[g] = [])).push(t);
    }
  });
});
const STEP = 0.00025, NEAR = 0.006;
function brute(e, shift) {
  const g = e.pair.split("|")[0], T = TRIS[g];
  const lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
  T.forEach(t => t.forEach(v => { for (let k = 0; k < 3; k++) { lo[k] = Math.min(lo[k], v[k]); hi[k] = Math.max(hi[k], v[k]); } }));
  let best = Infinity, n = 0, where = "";
  e.cubes.forEach(b => {
    for (let k = 0; k < 3; k++) for (const sg of [-1, 1]) {
      const a1 = (k + 1) % 3, a2 = (k + 2) % 3, N1 = Math.round(2 * b.h[a1] / STEP), N2 = Math.round(2 * b.h[a2] / STEP);
      for (let i = 0; i <= N1; i++) for (let j = 0; j <= N2; j++) {
        const l = [0, 0, 0];
        l[k] = sg * b.h[k]; l[a1] = -b.h[a1] + 2 * b.h[a1] * i / N1; l[a2] = -b.h[a2] + 2 * b.h[a2] * j / N2;
        const w = KIN.matVec(b.M, l); w[0] += b.c[0] + shift; w[1] += b.c[1]; w[2] += b.c[2];
        const p = e.toZero(w);
        if (p[0] < lo[0] - NEAR || p[0] > hi[0] + NEAR || p[1] < lo[1] - NEAR || p[1] > hi[1] + NEAR || p[2] < lo[2] - NEAR || p[2] > hi[2] + NEAR) continue;
        n++;
        for (const t of T) {
          // 三角形の箱までの距離が、いまのいちばん近い値以上なら、その三角形は見なくてよい(答えは変わらない。速くするだけ)
          const bb = t.bb; let a2 = 0;
          for (let k2 = 0; k2 < 3; k2++) { const o = p[k2] < bb[k2] ? bb[k2] - p[k2] : (p[k2] > bb[k2 + 3] ? p[k2] - bb[k2 + 3] : 0); a2 += o * o; }
          if (a2 >= best * best) continue;
          const q = COLLIDE.closestOnTri(p, t[0], t[1], t[2]);
          const d = Math.hypot(p[0] - q[0], p[1] - q[1], p[2] - q[2]);
          if (d < best) { best = d; const on = [0, 1, 2].filter(k3 => Math.abs(Math.abs(l[k3]) - b.h[k3]) < 1e-9).length; where = on === 3 ? "角" : on === 2 ? "辺" : "面"; }
        }
      }
    }
  });
  return { d: best, n, where };
}
let bad = 0, rows = 0;
const mm = x => (x * 1000).toFixed(3);
for (const ui of [{ SET: 2, GAP: 15, SOL: 0 }, { SET: 0, GAP: 15, SOL: 0 }]) {
  const log = [];
  M.buildFromUi(Object.assign({ REFINE_LOG: log }, ui));
  const ok = log.filter(e => e.fine !== null);
  const pick = new Map();
  const minE = ok.reduce((a, e) => (e.fine < a.fine ? e : a));
  pick.set(minE, "いちばん近い");
  ok.slice().sort((a, b) => (b.coarse - b.fine) - (a.coarse - a.fine)).slice(0, 5).forEach(e => pick.has(e) || pick.set(e, "縮み大"));
  let seed = 1; const rnd = () => (seed = (seed * 16807) % 2147483647) / 2147483647;
  for (let i = 0; i < 5; i++) { const e = ok[Math.floor(rnd() * ok.length)]; if (!pick.has(e)) pick.set(e, "くじ"); }
  console.log(`[${JSON.stringify(ui)}] 測り直した組 ${log.length}(うち測れた ${ok.length})。比べる ${pick.size}`);
  for (const [e, why] of pick) {
    const b = brute(e, 0), lowOk = e.fine >= b.d - 0.00018, highOk = e.fine <= b.d + 0.00036;
    rows++; if (!(lowOk && highOk)) bad++;
    console.log(`  ${why.padEnd(6)} t=${e.t.toFixed(2)}s ${e.pair.padEnd(8)} 粗い ${mm(e.coarse)} → fine ${mm(e.fine)}・力ずく ${mm(b.d)}mm(点 ${b.n}・箱の${b.where})差 ${mm(e.fine - b.d)} ${lowOk && highOk ? "OK" : "★NG"}`);
  }
  if (ui.SET === 2) {
    const b = brute(minE, 0.001), off = !(minE.fine >= b.d - 0.00018 && minE.fine <= b.d + 0.00036);
    rows++; if (!off) bad++;
    console.log(`  外れの検算(力ずくだけ箱を 1mm ずらす): fine ${mm(minE.fine)}・力ずく ${mm(b.d)}mm → ${off ? "OK(幅を出た)" : "★NG(ずらしても合格した = 比べ方が鈍い)"}`);
  }
}
console.log(bad ? `★NG ${bad}/${rows}` : `全部 OK(${rows} 行)`);
process.exit(bad ? 1 : 0);
