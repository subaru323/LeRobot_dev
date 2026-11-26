// 上のカメラから、写す姿勢(と、比べに待ちの姿勢)の腕が、置き場のピースと盤のマスを隠さないか(計画書_完成形まで の S-3 の残り・V-1)。
//   見ている世界: scene-data.js の 4 種(既定のすき間・台あり・手首のカメラあり)の、最初と最後のコマ(写す姿勢 PHOTO_Q)と待ちの姿勢。
//     上のカメラは so101.js の OVERHEAD の置き方【仮】、絵は 640×480・縦の画角 60°・絵の長い辺が左右(overhead-view.js の既定)。
//   測り方: マスの上の面の 5 点(真ん中と、角から 2mm 内側の 4 点)へ、レンズの先から線を引き、
//     腕(手首のカメラを含む)と台の形の点のどれかが線から 2.5mm 以内にあれば「隠れる」とする。
//     ★形の点は約 3mm 間隔なので、隠れる所を少し大きめにも小さめにも出しうる(点の間をすり抜ける線は、隠れないと出る)。
//   見ていないもの: レンズのゆがみ・光と影・ピースどうしで隠れること・置いている途中のコマ(写すのは写す姿勢のときとする)。
//   実行: node tools/overhead-occlusion.js
const { load } = require("./_load");
load("kinematics.js", "so101.js", "motion.js", "puzzle.js",
     "models/so101_visuals.js", "models/so101_points.js", "models/so101_tris.js", "collide.js", "scene-data.js",
     "chikuwa-collision.js", "self-collision.js");
const M = SCENE_MODEL, RY = M.robotY(true), W = 640, H = 480, FOV = SO101.CAM_FOV.top, TOL = 0.0025;
const oc = SO101.overheadCamera(), C = [oc.tip[0], oc.tip[1] + RY, oc.tip[2]];
const norm = v => { const n = Math.hypot(...v); return v.map(x => x / n); };
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
// 式は so101.js の overheadImage(画面の three.js のカメラと同じ向き。2026-11-17 の夜に左右の裏返りを直した)
const IMG = SO101.overheadImage({ tip: C, dir: oc.dir, up: oc.up }, FOV, W, H), z = IMG.fwd, x = IMG.right, y = IMG.up, f = IMG.f;
const project = IMG.project;
const inImage = q => q && q[0] >= 0 && q[0] < W && q[1] >= 0 && q[1] < H;
// 台の点(机 = 0 の座標)
const standPts = [];
M.STAND_PARTS.forEach(s => {
  if (s.mesh) { const p0 = M.standPos(s.mesh), raw = SO101_POINTS[s.mesh];
    for (let i = 0; i < raw.length; i += 3) standPts.push([raw[i] * 1e-4 + p0[0], raw[i + 1] * 1e-4 + p0[1], raw[i + 2] * 1e-4 + p0[2]]); return; }
  // 置き方 C の箱(2026-11-19 から。so101.js の OVERHEAD_C.boxesMm): 6 つの面に 4mm おきの点を打つ(机 = 0 の座標)
  if (s.box) {
    const lo = [s.box.lo[0], s.box.lo[1] + RY, s.box.lo[2]], hi = [s.box.hi[0], s.box.hi[1] + RY, s.box.hi[2]], sz = lo.map((v, i) => hi[i] - v);
    const n = sz.map(d => Math.max(1, Math.ceil(d / 0.004)));
    for (let ax = 0; ax < 3; ax++) for (const side of [0, 1]) {
      const u = (ax + 1) % 3, v = (ax + 2) % 3;
      for (let iu = 0; iu <= n[u]; iu++) for (let iv = 0; iv <= n[v]; iv++) {
        const w = [0, 0, 0]; w[ax] = side ? hi[ax] : lo[ax]; w[u] = lo[u] + sz[u] * iu / n[u]; w[v] = lo[v] + sz[v] * iv / n[v]; standPts.push(w);
      }
    }
    return;
  }
  // 置き方 C の円柱(線分と半径。2026-11-19 まで): 表面に 4mm おきの点を打つ(机 = 0 の座標)
  const a = [s.seg.a[0], s.seg.a[1] + RY, s.seg.a[2]], b = [s.seg.b[0], s.seg.b[1] + RY, s.seg.b[2]], w = b.map((v, i) => v - a[i]), L = Math.hypot(...w), wn = w.map(v => v / L);
  const t = Math.abs(wn[1]) < 0.9 ? [0, 1, 0] : [1, 0, 0], u0 = [wn[1] * t[2] - wn[2] * t[1], wn[2] * t[0] - wn[0] * t[2], wn[0] * t[1] - wn[1] * t[0]], un = Math.hypot(...u0), u = u0.map(v => v / un);
  const v = [wn[1] * u[2] - wn[2] * u[1], wn[2] * u[0] - wn[0] * u[2], wn[0] * u[1] - wn[1] * u[0]], nA = Math.max(8, Math.ceil(2 * Math.PI * s.seg.r / 0.004));
  for (let l = 0; l <= L + 1e-9; l += 0.004) for (let k = 0; k < nA; k++) { const th = 2 * Math.PI * k / nA;
    standPts.push([0, 1, 2].map(i => a[i] + wn[i] * l + s.seg.r * (u[i] * Math.cos(th) + v[i] * Math.sin(th)))); }
});
// 待ちの姿勢の腕の点(scene-data.js の当たり判定と同じ置き方)
function armPts(q, jaw) {
  const st = KIN.fk(SO101.axes, q, SO101.tool), out = [];
  Object.keys(M.ARM.points).forEach(g => {
    if (g === "base") { M.ARM.points.base.forEach(p => out.push([p[0], p[1] + RY, p[2]])); return; }
    const fr = st.frames[(g === "jaw" || g === "cam") ? 4 : (+g.slice(1) - 1)];
    M.ARM.points[g].forEach(p0 => {
      const p = g === "jaw" ? M.jawRotate(p0, jaw) : p0, w = KIN.add(fr.p, KIN.matVec(fr.R, KIN.sub(p, fr.o)));
      out.push([w[0], w[1] + RY, w[2]]);
    });
  });
  return out;
}
function hidden(T, pts) {             // レンズの先 → T の線から TOL 以内に点があるか(T の手前 3mm まで)
  const d = KIN.sub(T, C), L = Math.hypot(...d), u = d.map(v => v / L);
  for (const p of pts) {
    const w = KIN.sub(p, C), s = dot(w, u);
    if (s <= TOL || s >= L - 0.003) continue;   // レンズの先から TOL 以内はカメラ自身(2026-11-19。C のカメラの箱の下の面がレンズの先を通り、その点が丸めの誤差で s > 0 になって全部の線を隠していた。点検 C40 の後)
    const e = [w[0] - u[0] * s, w[1] - u[1] * s, w[2] - u[2] * s];
    if (e[0] * e[0] + e[1] * e[1] + e[2] * e[2] < TOL * TOL) return true;
  }
  return false;
}
function cellSamples(xz, h, cell) {   // マスの上の面の 5 点
  const k = cell / 2 - 0.002;
  return [[0, 0], [k, k], [k, -k], [-k, k], [-k, -k]].map(o => [xz[0] + o[0], h, xz[1] + o[1]]);
}
console.log(`[上のカメラ] レンズの先 (${C.map(v => (v * 1000).toFixed(0)).join(", ")}) mm(机 = 0)・縦の画角 ${FOV}°・640×480・線から ${TOL * 1000}mm 以内の点で隠れると見る`);
// 台だけで隠れる点があるかを先に見る(台の形がレンズの前に来ていないことの検算)
const sets = M.controls.find(c => c.key === "SET").options, def = M.defaultUiValues();
const scenes = sets.map(o => ({ label: o.label, c: M.buildFromUi({ SET: o.value, GAP: def.GAP, SOL: 0 })._check }));
function count(c, pts) {               // [隠れる, 絵の外, 全部] をピースごと・盤
  const res = [];
  c.pieceCells.forEach(cells => { let n = 0, h = 0, o = 0; cells.forEach(xz => cellSamples(xz, c.cell, c.cell).forEach(T => { n++; if (!inImage(project(T))) o++; else if (hidden(T, pts)) h++; })); res.push([h, o, n]); });
  let n = 0, h = 0, o = 0; c.boardCells.forEach(xz => cellSamples(xz, 0.0005, c.cell).forEach(T => { n++; if (!inImage(project(T))) o++; else if (hidden(T, pts)) h++; }));
  res.push([h, o, n]);
  return res;
}
const fmt = r => r.map((v, i) => (i < r.length - 1 ? "ピース" + i : "盤") + " " + v[0] + "/" + v[2] + (v[1] ? "(絵の外 " + v[1] + ")" : "")).join("・");
const total = r => r.reduce((a, v) => a + v[0] + v[1], 0);
// 解き終えた後(最後のコマ): 写す姿勢・最後の爪の角度で、盤に置いたピースの上の面(高さ = 辺)が隠れないか(点検 C11 の 7)
function countDone(c, pts) {
  let n = 0, h = 0, o = 0;
  c.boardCells.forEach(xz => cellSamples(xz, c.cell, c.cell).forEach(T => { n++; if (!inImage(project(T))) o++; else if (hidden(T, pts)) h++; }));
  return [[h, o, n]];
}
let bad = 0, badHome = 0;
scenes.forEach(S => {
  const st = count(S.c, standPts), photo = count(S.c, armPts(S.c.photoQ, S.c.homeJaw)), home = count(S.c, armPts(S.c.homeQ, S.c.homeJaw));
  const done = countDone(S.c, armPts(S.c.photoQ, S.c.lastJaw));
  bad += total(done);
  console.log(`[${S.label}] 台だけで隠れる: ${fmt(st)}`);
  console.log(`  写す姿勢 (${S.c.photoQ.join(", ")})° の腕で隠れる: ${fmt(photo)}`);
  console.log(`  解き終えた後(写す姿勢・最後の爪 ${S.c.lastJaw.toFixed(1)}°)、盤に置いたピースの上の面: ${done[0][0]}/${done[0][2]}${done[0][1] ? "(絵の外 " + done[0][1] + ")" : ""}`);
  console.log(`  (比べ)待ちの姿勢 (${S.c.homeQ.map(v => v.toFixed(0)).join(", ")})° の腕で隠れる: ${fmt(home)}`);
  bad += total(st) + total(photo); badHome += total(home);
});
// 負の検算: 待ちの姿勢では隠れる点が出るはず(出なければ、隠れる数え方が死んでいる)
if (!badHome) { console.log("★NG: 待ちの姿勢でも隠れる点が 0(数え方が見ていない)"); process.exitCode = 1; }
// 写すときの姿勢を探す(関節の可動域の中を格子で)。速く選ぶために、まず腕の点を絵に打って(線から TOL の太さ)
// 隠れる点を数え、少ない順に 8 つだけ線の測り方で数え直し、台・机から 5mm より離れるものを出す
const LIM = SO101.limitsDeg, samples = [];
scenes.forEach(S => {
  S.c.pieceCells.forEach(cells => cells.forEach(xz => cellSamples(xz, S.c.cell, S.c.cell).forEach(T => samples.push(T))));
  S.c.boardCells.forEach(xz => cellSamples(xz, 0.0005, S.c.cell).forEach(T => samples.push(T)));
});
const sq = samples.map(project).map(q => q && inImage(q) ? [Math.floor(q[0]), Math.floor(q[1])] : null);
function quickHidden(pts) {
  const mask = new Uint8Array(W * H);
  pts.forEach(p => {
    const d = [p[0] - C[0], p[1] - C[1], p[2] - C[2]], zc = dot(d, z);
    if (zc <= 0) return;
    const u = W / 2 + f * dot(d, x) / zc, v = H / 2 - f * dot(d, y) / zc, r = Math.ceil(f * TOL / zc + 1);
    for (let j = Math.max(0, Math.floor(v - r)); j <= Math.min(H - 1, v + r); j++)
      for (let i = Math.max(0, Math.floor(u - r)); i <= Math.min(W - 1, u + r); i++) mask[j * W + i] = 1;
  });
  return sq.reduce((a, q) => a + (q === null || mask[q[1] * W + q[0]] ? 1 : 0), 0);
}
const grid = [];
for (let pan = -110; pan <= 110; pan += 20) for (let lift = -100; lift <= 60; lift += 20)
  for (let elbow = -90; elbow <= 90; elbow += 30) for (let flex = -90; flex <= 90; flex += 45) grid.push([pan, lift, elbow, flex, 0]);
const nonBase = M.ARM.points.base.length;
const scored = grid.map(q => {
  const pts = armPts(q, 0), low = Math.min(...pts.slice(nonBase).map(p => p[1]));
  return low < 0.005 ? null : { q, low, quick: quickHidden(pts) };
}).filter(Boolean).sort((a, b) => a.quick - b.quick);
const best = [];
for (const o of scored) {
  if (best.length >= 5) break;
  const sc = M.standClearance(o.q, 0), dmin = Math.min(Infinity, ...Object.values(sc.pairs));
  if (dmin < 0.005) continue;
  const self = SELF_COLLIDE.pose(o.q, 0);                       // 腕どうし(S-10。self-collision.js)。WARN より近い姿勢は捨てる
  if (self.clearance < SELF_COLLIDE.WARN) continue;
  const pts = armPts(o.q, 0), hid = scenes.reduce((a, S) => a + total(count(S.c, pts)), 0);
  best.push({ q: o.q, hid, quick: o.quick, dmin, low: o.low, self: self.clearance, selfPair: self.pair });
}
console.log(`[写すときの姿勢を探す] 格子 ${grid.length} 通り(机から 5mm より上は ${scored.length} 通り)。絵に打って数えた隠れる点の少ない順に、台から 5mm・腕どうし ${SELF_COLLIDE.WARN * 1000}mm より離れるもの 5 つ(4 種の点 全部で ${samples.length} 点):`);
best.forEach(o => console.log(`  (${o.q.join(", ")})°: 隠れる・絵の外 ${o.hid} 点(絵に打って ${o.quick} 点)/ 台まで ${isFinite(o.dmin) ? (o.dmin * 1000).toFixed(0) + "mm" : "15mm より遠い"}・机まで ${(o.low * 1000).toFixed(0)}mm・腕どうし ${(o.self * 1000).toFixed(1)}mm(${o.selfPair})`));
console.log(bad ? `★写す姿勢で、隠れる・絵の外の点が 4 種で ${bad} 点ある` : `写す姿勢で、4 種の全部の点が絵に入り、隠れない(こちらで決めた置き場・写す前と解き終えた後。待ちの姿勢では ${badHome} 点が隠れる・絵の外)`);
if (bad) process.exitCode = 1;
