// K-02: 上のカメラの置き方の候補を、同じ量で比べる(docs/Simulation_Logic_Documents/08 §3・現在の課題 K-02)。
//   このファイルは docs を書く会話が新しく作った(2026-11-17)。scene-data.js・so101.js・self-collision.js ほかは読むだけで、書き換えない。
//   候補(08 §3):
//     A  いまの本家の台(so101.js の OVERHEAD)。写す姿勢は scene-data.js の PHOTO_Q。
//        B(腕を柱の反対へよける)は、A の台のまま写す姿勢だけが違うので、A の写す姿勢の探し(overhead-occlusion.js)の中に入る。ここでは別に出さない。
//     C  上のカメラを、腕から離した別の柱に付け、置き場と盤の真上から真下を向けて見る。高さは、(i) 角が全部、縁の余裕 0.075 を残して絵に入り、
//        (ii) 動きの全コマで腕の点が柱・吊る腕・カメラから CLEAR_C【仮】より離れる、いちばん低い所。
//        柱は、肩の軸から置き場の真ん中へ向かう向きの先で、動きの全コマで腕の点から CLEAR_C あく所に立て、柱の上から水平の腕でカメラを吊る【仮】。台の柱(カメラの柱)は無くす。
//     D  上のカメラをやめ、手首のカメラで置き場を何か所か写して見つける。写す回数・時間の増え方・mm/画素を目安で出す。
//   出すもの(候補ごと):
//     (1) 写す姿勢で隠れる点・絵の外の点(overhead-occlusion.js と同じ数え方: マスの上の面の 5 点へ線を引き、腕の点が 2.5mm 以内なら隠れる)
//     (2) 腕どうしの隙間(scene-data.js の _check.self)・台や机との一番近い所(_check.minClear)
//     (3) 台の柱に当たる置き場の数(out/stand-sweep.json の当たりを、当たった台の部品で分けて数える。C・D はカメラの柱が無い)
//         C は、動きの全コマで腕の点と、柱・吊る腕・カメラ(半径【仮】)との一番近い所も出す
//     (4) 角でいちばん粗い mm/画素と、縁の余裕(縦の画角 60°【仮】・640×480 / 1280×720 / 1920×1080)。上の面の高さで写したときの、光線の傾きによるずれの最大(14mm × tan)
//     (5) 時間(scene-data.js の動きの全体 T。D は写す回数 × 【仮】の時間を足す)
//   ★負の検算:
//     (a) C のカメラを、角が入る一番低い所より 80mm 下げると、角が絵から出る(高さの決め方が角を見ていること)
//     (b) C で、待ちの姿勢では隠れる点が出る(隠れる数え方が C のカメラで生きていること。overhead-occlusion.js と同じ考え)
//     (c) C の柱を置き場の真ん中に立てると、腕と柱の一番近い所が 0 より小さくなる(柱の当たりの測り方が生きていること)
//   ★C・C2 の (4) は、縦の画角 60° のままの値と、角が縁の余裕を残して入るいちばん狭い画角に合わせた値の 2 つを出す(点検 C26 の 1。高さより画角で細かさが決まる)。
//   ★C の柱に当たるかは、8 種の動きでしか見ていない(A のような置き場の掃き出しはしていない。点検 C26 の 4)。
//   ★D の写す回数は、撮る高さ = つかむ場所の上で止まる高さとした目安。範囲は置き場だけ(盤は入れていない。点検 C26 の 7)。
//   見ていないもの: レンズのゆがみ・光と影・ピースを見つける道具の実力(K6)・カメラの台の形(C の柱と吊る腕は線分と半径【仮】で見た)・ケーブル・
//     D の写す姿勢の腕どうしの当たりと、写す位置の組み方(目安の数だけ)・腕の位置の誤差が D の見つけた位置に乗る量。
//   実行: node tools/k02-compare.js [--sets 0,1,2,3]
const { load } = require("./_load");
load("kinematics.js", "so101.js", "motion.js", "puzzle.js",
     "models/so101_visuals.js", "models/so101_points.js", "models/so101_tris.js", "collide.js", "scene-data.js",
     "chikuwa-collision.js", "self-collision.js");
const fs = require("fs"), path = require("path");
// RY: 2026-11-17 の比べは、本家の板(7.2mm)を敷いた腕の高さ 9.6mm で出した。2026-11-19 から M.robotY(true) は C の台(板なし)の 2.4mm を返すので、
//   比べを同じ条件で出し直せるよう、ここで 9.6mm を自分で持つ(隣の会話の点検 C40 の 6)。
const RY_PLATE = (SO101.OVERHEAD.plateMm + SO101.OVERHEAD.baseBelowOriginMm) / 1000;
if (Math.abs(SCENE_MODEL.robotY(true) - RY_PLATE) > 1e-6) console.log(`  注: scene-data.js の robotY(true) は ${(SCENE_MODEL.robotY(true) * 1000).toFixed(1)}mm。この比べは本家の板ありの ${(RY_PLATE * 1000).toFixed(1)}mm で出す`);
const M = SCENE_MODEL, RY = RY_PLATE, FOV = SO101.CAM_FOV.top, TOL = 0.0025, MARGIN = 0.075, D2R = Math.PI / 180;
const RES = [[640, 480], [1280, 720], [1920, 1080]];
const POLE_START = 0.020;      // C の柱を、届く輪の外のこの距離から外へ 10mm ずつ出し、動きの全コマで腕から CLEAR_C あく所に立てる【仮】
const CAM_R = 0.025;           // カメラの体を、レンズの先と基板の前の面を結ぶ線から この半径の筒とみる【仮】(32×32mm の基板)
const POLE_R = 0.012;          // 柱と吊る腕の太さ(半径)【仮】
const CLEAR_C = 0.030;         // C の柱・吊る腕・カメラと、腕の点の間に残す隙間【仮】
const T_SCAN_MOVE = 0.8, T_SCAN_SETTLE = 0.3, T_SHOT = 0.5;   // D の 1 か所あたり: 動く・揺れが収まる・撮って見つける【仮】
const arg = k => { const i = process.argv.indexOf("--" + k); return i > 0 ? process.argv[i + 1] : undefined; };
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const PAN_X = SO101.joints[0].origin[0], R_OUT = 0.235;

// ---- 世界(4 種〜7 種。全部の判定をした動き) ----
const opts = M.controls.find(c => c.key === "SET").options, def = M.defaultUiValues();
const setIdx = (arg("sets") || opts.map(o => o.value).join(",")).split(",").map(Number);
const t0 = Date.now();
const scenes = setIdx.map(v => ({ set: v, label: opts.find(o => o.value === v).label, c: M.buildFromUi({ SET: v, GAP: def.GAP, SOL: 0 })._check }));
console.log(`[世界] ${scenes.length} 種を組んだ(${((Date.now() - t0) / 1000).toFixed(0)} 秒)。全部の判定あり・台あり・縦の画角 ${FOV}°【仮】`);

// ---- 腕の点(overhead-occlusion.js と同じ置き方) ----
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
const nonBase = M.ARM.points.base.length;
function cellSamples(xz, h, cell) { const k = cell / 2 - 0.002; return [[0, 0], [k, k], [k, -k], [-k, k], [-k, -k]].map(o => [xz[0] + o[0], h, xz[1] + o[1]]); }
function cellCorners(xz, h, cell) { const k = cell / 2; return [[k, k], [k, -k], [-k, k], [-k, -k]].map(o => [xz[0] + o[0], h, xz[1] + o[1]]); }

// ---- カメラ(tip, dir, up は机 = 0 の座標) ----
function makeCam(cam, W, H, fov) {
  const img = SO101.overheadImage(cam, fov || FOV, W, H);
  const inImg = p => { const q = img.project(p); return q && q[0] >= 0 && q[0] < W && q[1] >= 0 && q[1] < H ? q : null; };
  function hidden(T, pts) {
    const C = cam.tip, d = sub(T, C), L = Math.hypot(...d), u = d.map(v => v / L);
    for (const p of pts) {
      const w = sub(p, C), s = dot(w, u);
      if (s <= 0 || s >= L - 0.003) continue;
      const e = [w[0] - u[0] * s, w[1] - u[1] * s, w[2] - u[2] * s];
      if (e[0] * e[0] + e[1] * e[1] + e[2] * e[2] < TOL * TOL) return true;
    }
    return false;
  }
  return { cam, img, W, H, fov: fov || FOV, inImg, hidden };
}
// 隠れる・絵の外: 写す前(置き場のピースと盤)と、解き終えた後(盤に置いたピース)
function occl(K, q, jawPre, jawDone) {
  const pre = armPts(q, jawPre), done = armPts(q, jawDone);
  let hid = 0, out = 0, n = 0;
  scenes.forEach(S => {
    const c = S.c, check = (T, pts) => { n++; if (!K.inImg(T)) out++; else if (K.hidden(T, pts)) hid++; };
    c.pieceCells.forEach(cells => cells.forEach(xz => cellSamples(xz, c.cell, c.cell).forEach(T => check(T, pre))));
    c.boardCells.forEach(xz => cellSamples(xz, 0.0005, c.cell).forEach(T => check(T, pre)));
    c.boardCells.forEach(xz => cellSamples(xz, c.cell, c.cell).forEach(T => check(T, done)));
  });
  return { hid, out, n };
}
// 角(上の面の 4 隅)で: 絵に入るか・縁の余裕・いちばん粗い mm/画素・光線の傾きによる上の面のずれ(14mm × tan)
function corners(K) {
  let worst = 0, minMargin = Infinity, outN = 0, maxTilt = 0, n = 0;
  scenes.forEach(S => {
    const c = S.c, pts = [];
    c.pieceCells.forEach(cells => cells.forEach(xz => cellCorners(xz, c.cell, c.cell).forEach(p => pts.push(p))));
    c.boardCells.forEach(xz => cellCorners(xz, 0.0005, c.cell).forEach(p => pts.push(p)));
    pts.forEach(p => {
      n++;
      const q = K.img.project(p);
      if (!q || q[0] < 0 || q[0] >= K.W || q[1] < 0 || q[1] >= K.H) { outN++; return; }
      minMargin = Math.min(minMargin, Math.min(q[0], K.W - q[0], q[1], K.H - q[1]) / K.H);
      const a = K.img.unproject(q[0] + 1, q[1], p[1]), b = K.img.unproject(q[0], q[1] + 1, p[1]);
      worst = Math.max(worst, Math.hypot(a[0] - p[0], a[2] - p[2]), Math.hypot(b[0] - p[0], b[2] - p[2]));
      const d = sub(p, K.cam.tip), tilt = Math.acos(Math.min(1, -d[1] / Math.hypot(...d)));
      maxTilt = Math.max(maxTilt, tilt);
    });
  });
  return { worst, minMargin, outN, n, shiftMm: 14 * Math.tan(maxTilt), tiltDeg: maxTilt / D2R };
}
// 画角を合わせたとき: 縁の余裕 MARGIN を残して角が全部入る、いちばん狭い縦の画角(0.5° 刻み・20〜FOV°)。真下を向く C・C2 だけに使う
//   (A は縁の余裕が 60° でも 0.099 しかなく、狭められない。画角の狭いレンズの品があるかは見ていない)
function fittedRow(cam) {
  return RES.map(([W, H]) => {
    let f = 20; for (; f <= FOV; f += 0.5) { const r = corners(makeCam(cam, W, H, f)); if (!r.outN && r.minMargin >= MARGIN) break; }
    if (f > FOV) return `${W}×${H}: 合わせられない`;
    const r = corners(makeCam(cam, W, H, f)); return `${W}×${H}: 縦 ${f.toFixed(1)}° で ${(r.worst * 1000).toFixed(2)}mm/画素・縁 ${r.minMargin.toFixed(3)}`;
  }).join(" | ");
}
function cornerRow(cam) {
  return RES.map(([W, H]) => { const r = corners(makeCam(cam, W, H)); return `${W}×${H}: ` + (r.outN ? `絵の外 ${r.outN}/${r.n}` : `${(r.worst * 1000).toFixed(2)}mm/画素・縁 ${r.minMargin.toFixed(3)}${r.minMargin < MARGIN ? "✗" : ""}`); }).join(" | ");
}
// 点と線分の距離
function segDist(p, a, b) { const ab = sub(b, a), t = Math.max(0, Math.min(1, dot(sub(p, a), ab) / dot(ab, ab))); return Math.hypot(...sub(p, [a[0] + ab[0] * t, a[1] + ab[1] * t, a[2] + ab[2] * t])); }
// 動きの全コマ(1 つおき)で、腕の点と線分(半径 r)の一番近い所
const FRAME_CACHE = new Map();   // 動きの全コマ(1 つおき)の腕の点(土台を除き、点も 1 つおき)。世界の組ごとに一度だけ作る
function frames(list) {
  list = list || scenes;
  if (FRAME_CACHE.has(list)) return FRAME_CACHE.get(list);
  const out = [];
  list.forEach(S => S.c.motion.forEach((f, i) => {
    if (i % 2) return;
    const pts = armPts(f.q, f.jaw), n = Math.ceil((pts.length - nonBase) / 2), a = new Float32Array(n * 3);
    let top = -Infinity, j = 0;
    for (let k = nonBase; k < pts.length; k += 2) { a[j++] = pts[k][0]; a[j++] = pts[k][1]; a[j++] = pts[k][2]; if (pts[k][1] > top) top = pts[k][1]; }
    out.push({ set: S.set, t: f.t, pts: a, top });
  }));
  FRAME_CACHE.set(list, out);
  return out;
}
function motionClear(segs, list) {
  let best = { d: Infinity };
  frames(list).forEach(F => {
    const A = F.pts;
    for (let k = 0; k < A.length; k += 3) { const p = [A[k], A[k + 1], A[k + 2]]; for (const s of segs) {
      const d = segDist(p, s.a, s.b) - s.r;
      if (d < best.d) best = { d, what: s.name, set: F.set, t: F.t };
    } }
  });
  return best;
}
// 柱を立てる所: 肩の軸から dir の向きに、届く輪の外 POLE_START から 10mm ずつ出し、縦の柱(高さ 1.2m)が動きの全コマで腕から CLEAR_C あく所
function placePole(uXZ, list) {
  for (let r = R_OUT + POLE_START; r < 0.8; r += 0.01) {
    const xz = [PAN_X + uXZ[0] * r, uXZ[1] * r], cl = motionClear([{ a: [xz[0], 0, xz[1]], b: [xz[0], 1.2, xz[1]], r: POLE_R, name: "柱" }], list);
    if (cl.d >= CLEAR_C) return { xz, r, cl };
  }
  return null;
}
const mm = v => (v * 1000).toFixed(1);
// 台の柱に当たる置き場(out/stand-sweep.json。3×4 の O を柱のまわりの格子に置いた掃き出し)
const sweepPath = path.join(__dirname, "..", "out", "stand-sweep.json");
let sweep = null;
if (fs.existsSync(sweepPath)) {
  const j = JSON.parse(fs.readFileSync(sweepPath, "utf8")), hit = j.rows.filter(r => r.minStand < 0), by = {};
  hit.forEach(r => { const k = r.pair.split("|")[1]; by[k] = (by[k] || 0) + 1; });
  // 分母は「解けて、置き場の決まり(足跡から 20mm)を満たす」行(点検 C26 の 2。388 は格子の全部で、解けない行も入る)
  const okRows = j.rows.filter(r => r.solved && r.g >= 0.02);
  sweep = { made: j.made, n: j.rows.length, hit: hit.length, by, post: hit.filter(r => r.pair.split("|")[1] !== "st_plate").length,
            okN: okRows.length, okHit: okRows.filter(r => r.minStand < 0).length, solvedN: j.rows.filter(r => r.solved).length };
}
const selfOf = () => scenes.map(S => S.c.self ? `${mm(S.c.self.min)}(${S.c.self.pair})` : "—").join("・");
const clearOf = () => scenes.map(S => `${mm(S.c.minClear)}(${S.c.minClearPair})`).join("・");
const timeOf = () => scenes.map(S => S.c.T.toFixed(1)).join("・");

// =====================================================================
// A: いまの本家の台
// =====================================================================
// ★2026-11-17 の夜から、SO101.overheadCamera() は置き方 C を返す(利用者「Go with C」・K-12)。A は本家の台の値(SO101.OVERHEAD。比べる用に残っている)から組む
//   (C にする前の overheadCamera と同じ式: 上の部品の位置 + カメラの板の面 → レンズの向きにレンズの先まで。机の座標で (34, 529, -116)mm・25°)。
//   ただし、いまの scene-data.js の場面には A の台の柱が無く(C の柱がある)、種 0・1 の置き場も A のときと違う。
//   だから A の列の (1) 隠れる・(2) 腕どうし・(5) 時間は、C の場面の上で A のカメラから見た値になる。A のときの値は 08 §3-1 の表(2026-11-17 に流した出力)。
const OA = SO101.OVERHEAD, topA = SO101.overheadPartPos("overhead_cam_mount_top"), faceA = topA.map((v, i) => v + OA.camFaceMm[i] / 1000);
const camA = { tip: faceA.map((v, i) => v + OA.camDir[i] * OA.lensTipMm / 1000 + (i === 1 ? RY : 0)), dir: OA.camDir.slice(), up: OA.camUp.slice() };
const c0 = scenes[0].c;
console.log(`\n== A いまの本家の台。レンズの先 (${camA.tip.map(v => (v * 1000).toFixed(0)).join(", ")}) mm・光軸は真下から ${(Math.acos(-camA.dir[1]) / D2R).toFixed(1)}° 傾く`);
const occAll = (K) => { let h = 0, o = 0, n = 0; scenes.forEach(S => { const r = occlOne(K, S); h += r.hid; o += r.out; n += r.n; }); return { hid: h, out: o, n }; };
function occlOne(K, S) {     // 種ごとの写す姿勢・爪で
  const c = S.c, pre = armPts(c.photoQ, c.homeJaw), done = armPts(c.photoQ, c.lastJaw);
  let hid = 0, out = 0, n = 0; const check = (T, pts) => { n++; if (!K.inImg(T)) out++; else if (K.hidden(T, pts)) hid++; };
  c.pieceCells.forEach(cells => cells.forEach(xz => cellSamples(xz, c.cell, c.cell).forEach(T => check(T, pre))));
  c.boardCells.forEach(xz => cellSamples(xz, 0.0005, c.cell).forEach(T => check(T, pre)));
  c.boardCells.forEach(xz => cellSamples(xz, c.cell, c.cell).forEach(T => check(T, done)));
  return { hid, out, n };
}
const rA = occAll(makeCam(camA, 640, 480));
console.log(`  (1) 写す姿勢 (${c0.photoQ.join(", ")})° で 隠れる ${rA.hid}・絵の外 ${rA.out} / ${rA.n} 点`);
console.log(`  (2) 腕どうし ${selfOf()} mm・台や机やピースとの一番近い所 ${clearOf()} mm`);
console.log(`  (3) 台の柱に当たる置き場 ` + (sweep ? `${sweep.okHit}/${sweep.okN}(解けて置き場の決まりを満たすもの。格子 ${sweep.n}・解けた ${sweep.solvedN}・当たり全部 ${sweep.hit}: `
  + `${Object.entries(sweep.by).map(([k, v]) => k + " " + v).join("・")}。stand-sweep.json ${sweep.made.slice(0, 10)}・写す姿勢を替える前)` : "stand-sweep.json が無い"));
const cA = corners(makeCam(camA, 640, 480));
console.log(`  (4) ${cornerRow(camA)}`);
console.log(`      上の面のずれ(光線の傾き ${cA.tiltDeg.toFixed(1)}° まで)最大 ${cA.shiftMm.toFixed(1)}mm(点ごとに直せる)`);
console.log(`  (5) 動きの全体 ${timeOf()} 秒`);

// =====================================================================
// C: 別の柱で、置き場と盤の真上から真下を向く
// =====================================================================
let bb = [Infinity, -Infinity, Infinity, -Infinity];   // xmin, xmax, zmin, zmax(角)
scenes.forEach(S => { const c = S.c; [...c.pieceCells.flat(), ...c.boardCells].forEach(xz => { const k = c.cell / 2; bb = [Math.min(bb[0], xz[0] - k), Math.max(bb[1], xz[0] + k), Math.min(bb[2], xz[1] - k), Math.max(bb[3], xz[1] + k)]; }); });
const ctr = [(bb[0] + bb[1]) / 2, (bb[2] + bb[3]) / 2], longX = (bb[1] - bb[0]) >= (bb[3] - bb[2]);
// 真下を向き、絵の長い辺を置き場の長い向きに合わせる(up は絵の上 = 短い辺の向き)
const camAt = (h, cx, cz) => ({ tip: [cx, h, cz], dir: [0, -1, 0], up: longX ? [0, 0, -1] : [1, 0, 0] });
function fitHeight(W, H) {
  for (let h = 0.15; h <= 1.2; h += 0.005) { const r = corners(makeCam(camAt(h, ctr[0], ctr[1]), W, H)); if (!r.outN && r.minMargin >= MARGIN) return h; }
  return NaN;
}
// 柱: 肩の軸から置き場の真ん中へ向かう向きの先、届く輪の外
const dirXZ = [ctr[0] - PAN_X, ctr[1]], dn = Math.hypot(...dirXZ), uXZ = dirXZ.map(v => v / dn);
const poleC = placePole(uXZ), poleXZ = poleC.xz;
const camBody = cam => { const back = [cam.tip[0] - cam.dir[0] * 0.026, cam.tip[1] - cam.dir[1] * 0.026, cam.tip[2] - cam.dir[2] * 0.026]; return { a: cam.tip, b: back, r: CAM_R, name: "カメラ" }; };
const segsC = (pole, cam) => [
  { a: [pole[0], 0, pole[1]], b: [pole[0], cam.tip[1] + 0.03, pole[1]], r: POLE_R, name: "柱" },
  { a: [pole[0], cam.tip[1] + 0.03, pole[1]], b: [cam.tip[0], cam.tip[1] + 0.03, cam.tip[2]], r: POLE_R, name: "吊る腕" },
  camBody(cam)];
// 高さ: 角が入る一番低い所と、腕のいちばん高い所の 60mm 下の高い方から、腕との隙間が CLEAR_C になるまで 10mm ずつ上げる
//   (10mm 刻みの始まりが変わるので、下から全部試したときと 10mm 以内で違いうる。--sets 2・2026-11-17: 全部試すと C 525・C2 435mm、この探し方で C 520・C2 430mm)
const hFit = fitHeight(640, 480), armTop = Math.max(...frames().map(F => F.top));
let hC = Math.max(hFit, Math.round((armTop - 0.06) * 100) / 100), clrC = motionClear(segsC(poleXZ, camAt(hC, ctr[0], ctr[1])));
while (clrC.d < CLEAR_C && hC < 1.2) { hC += 0.01; clrC = motionClear(segsC(poleXZ, camAt(hC, ctr[0], ctr[1]))); }
const camC = camAt(hC, ctr[0], ctr[1]);
console.log(`\n== C 別の柱で真上から。置き場と盤の角の範囲 x ${mm(bb[0])}〜${mm(bb[1])}・z ${mm(bb[2])}〜${mm(bb[3])} mm → 真ん中 (${mm(ctr[0])}, ${mm(ctr[1])}) の上`);
console.log(`  レンズの先の高さ ${mm(hC)}mm(角が入る一番低い所は ${mm(hFit)}mm。動きの全コマの腕の点のいちばん高い所 ${mm(armTop)}mm。腕から ${mm(CLEAR_C)}mm【仮】あく所まで上げた)`);
// 写す姿勢: いまの PHOTO_Q と待ちの姿勢
const KC = makeCam(camC, 640, 480);
const cand = [{ name: "いまの写す姿勢", q: c0.photoQ }, { name: "待ちの姿勢", q: c0.homeQ }];
const tryPose = q => { const r = occl(KC, q, c0.homeJaw, c0.lastJaw); return r; };
cand.forEach(o => { o.r = tryPose(o.q); });
cand.forEach(o => console.log(`  (1) ${o.name} (${o.q.map(v => v.toFixed(0)).join(", ")})° で 隠れる ${o.r.hid}・絵の外 ${o.r.out} / ${o.r.n} 点`));
console.log(`  柱の位置 (${mm(poleXZ[0])}, ${mm(poleXZ[1])}) mm(肩の軸から ${mm(poleC.r)}mm。縦の柱だけなら腕から ${mm(poleC.cl.d)}mm)・吊る腕の長さ ${mm(Math.hypot(camC.tip[0] - poleXZ[0], camC.tip[2] - poleXZ[1]))}mm【仮】`);
console.log(`  (2) 腕どうし・台や机との一番近い所は、写す姿勢が同じなら A と同じ(動きは同じ)`);
console.log(`  (3) 台の柱(カメラの柱)は無い → stand-sweep の当たり ${sweep ? sweep.post : "?"} 件は消える(板 st_plate の当たり ${sweep ? (sweep.by.st_plate || 0) : "?"} 件)。` +
            `動きの全コマで、腕と C の柱・吊る腕・カメラの一番近い所 ${mm(clrC.d)}mm(${clrC.what}・種 ${clrC.set}・${clrC.t.toFixed(1)} 秒)`);
const cC = corners(KC);
console.log(`  (4) ${cornerRow(camC)}`);
console.log(`      画角を合わせたとき ${fittedRow(camC)}`);
console.log(`      上の面のずれ(光線の傾き ${cC.tiltDeg.toFixed(1)}° まで)最大 ${cC.shiftMm.toFixed(1)}mm`);
console.log(`  (5) 動きは A と同じ(写す姿勢を替えない限り)`);
// 負の検算
const negA = corners(makeCam(camAt(hFit - 0.08, ctr[0], ctr[1]), 640, 480));
const negB = cand[1].r.hid;   // 待ちの姿勢で隠れる点
const negC = motionClear(segsC(ctr, camC));
console.log(`  負の検算: (a) 角が入る一番低い所から 80mm 下げると絵の外 ${negA.outN}/${negA.n} ${negA.outN ? "OK" : "★NG"}・(b) 待ちの姿勢で隠れる ${negB} ${negB ? "OK" : "★NG"}・(c) 柱を置き場の真ん中に立てると一番近い所 ${mm(negC.d)}mm ${negC.d < 0 ? "OK" : "★NG"}`);
if (!negA.outN || !negB || negC.d >= 0) process.exitCode = 1;

// =====================================================================
// C2: C の柱で、写す姿勢も低くする(C の柱なら、腕を高く上げて上のカメラの視野から逃がす要が無い)
//   1) 格子(overhead-occlusion.js と同じ)から、腕の点のいちばん高い所が C2_TOP【仮】未満・机から 5mm 上・板(st_plate)から 5mm・腕どうし WARN より離れ、
//      C のカメラ(仮の高さ)で隠れる点が 0 の姿勢を選ぶ(絵に打って速く数え、少ない順に線の測り方で数え直す)。
//   2) その姿勢を写す姿勢(_PHOTO_Q)にして、全部の判定で動きを組み直す(台の柱は scene-data.js のまま残る = 厳しめ)。
//   3) 組み直した動きの全コマで、腕から CLEAR_C あく高さまでカメラを上げる。
// =====================================================================
const C2_TOP = 0.25;
console.log(`\n== C2 C の柱で、写す姿勢も低くする`);
const hProbe = Math.max(hFit, 0.30), KP = makeCam(camAt(hProbe, ctr[0], ctr[1]), 640, 480);
const sampAll = [];
scenes.forEach(S => { const c = S.c; c.pieceCells.forEach(cells => cells.forEach(xz => cellSamples(xz, c.cell, c.cell).forEach(T => sampAll.push(T)))); c.boardCells.forEach(xz => cellSamples(xz, 0.0005, c.cell).forEach(T => sampAll.push(T))); });
const sqP = sampAll.map(T => KP.inImg(T)).map(q => q ? [Math.floor(q[0]), Math.floor(q[1])] : null);
function quickHiddenP(pts) {        // 腕の点を絵に打って(線から TOL の太さ)、隠れる・絵の外の見本の数
  const W = 640, H = 480, mask = new Uint8Array(W * H), C = KP.cam.tip, X = KP.img.right, Y = KP.img.up, Z = KP.img.fwd, f = KP.img.f;
  pts.forEach(p => {
    const d = sub(p, C), zc = dot(d, Z); if (zc <= 0) return;
    const u = W / 2 + f * dot(d, X) / zc, v = H / 2 - f * dot(d, Y) / zc, r = Math.ceil(f * TOL / zc + 1);
    for (let j = Math.max(0, Math.floor(v - r)); j <= Math.min(H - 1, v + r); j++) for (let i = Math.max(0, Math.floor(u - r)); i <= Math.min(W - 1, u + r); i++) mask[j * W + i] = 1;
  });
  return sqP.reduce((a, q) => a + (q === null || mask[q[1] * W + q[0]] ? 1 : 0), 0);
}
const grid2 = [];
const LIM_PAD = 10;   // 関節の可動域の端から残す角度【仮】(実機の校正と角度の換算のずれを見込む。点検 C26 の 2)
const inLim = q => q.every((v, i) => v >= SO101.joints[i].limitsDeg[0] + LIM_PAD && v <= SO101.joints[i].limitsDeg[1] - LIM_PAD);
for (let pan = -110; pan <= 110; pan += 20) for (let lift = -100; lift <= 60; lift += 20) for (let elbow = -90; elbow <= 90; elbow += 30) for (let flex = -90; flex <= 90; flex += 45) { const q = [pan, lift, elbow, flex, 0]; if (inLim(q)) grid2.push(q); }
const scored2 = grid2.map(q => { const pts = armPts(q, 0), arm = pts.slice(nonBase), top = Math.max(...arm.map(p => p[1])), low = Math.min(...arm.map(p => p[1])); return (top >= C2_TOP || low < 0.005) ? null : { q, top, quick: quickHiddenP(pts) }; })
  .filter(Boolean).sort((a, b) => a.quick - b.quick || a.top - b.top);
let pick = null, noPlate = 0;
for (const o of scored2) {
  if (o.quick > 0) break;
  const sc = M.standClearance(o.q, 0), platePairs = Object.entries(sc.pairs).filter(([k]) => /st_plate/.test(k)).map(([, v]) => v);
  if (!platePairs.length) { noPlate++; } else if (Math.min(...platePairs) < 0.005) continue;   // 板の当たりが無い = 台が替わった。黙らず数えて下で書く
  const self = SELF_COLLIDE.pose(o.q, 0); if (self.clearance < SELF_COLLIDE.WARN) continue;
  const r = occl(KP, o.q, 0, 0); if (r.hid || r.out) continue;
  pick = { ...o, self: self.clearance, selfPair: self.pair }; break;
}
if (noPlate) console.log(`  ★注: 板(st_plate)との離れを測れなかった姿勢が ${noPlate} 通り(scene-data.js の台に板が無い、または当たりの組が出ない)。C2 の「板から 5mm」は確かめていない`);
let scenes2 = null, camC2 = null;
if (!pick) console.log(`  腕のいちばん高い所 ${mm(C2_TOP)}mm 未満で、隠れない写す姿勢が格子に無い(${scored2.length} 通りを見た)`);
else {
  console.log(`  写す姿勢の候補 (${pick.q.join(", ")})°: 腕のいちばん高い所 ${mm(pick.top)}mm・腕どうし ${mm(pick.self)}mm(${pick.selfPair})。格子 ${grid2.length} 通りのうち高さの条件に入る ${scored2.length} 通りから選んだ`);
  FRAME_CACHE.delete(scenes);   // メモリを空ける(C の分はもう使わない)
  const t1 = Date.now();
  scenes2 = scenes.map(S => ({ set: S.set, label: S.label, c: M.buildFromUi({ SET: S.set, GAP: def.GAP, SOL: 0, _PHOTO_Q: pick.q })._check }));
  console.log(`  組み直した(${((Date.now() - t1) / 1000).toFixed(0)} 秒): 解けた ${scenes2.filter(S => S.c.solved).length}/${scenes2.length}・当たったコマ ${scenes2.map(S => S.c.hitFrames).join("・")}`);
  const hitBad = scenes2.filter(S => !S.c.solved || S.c.hitFrames);
  scenes2.forEach(S => { if (!S.c.solved) console.log(`    [${S.label}] 解けない: ${S.c.why || S.c.layoutWhy}`); });
  const top2a = Math.max(...frames(scenes2).map(F => F.top));
  const pole2 = placePole(uXZ, scenes2), pXZ2 = pole2.xz;
  console.log(`  柱の位置 (${mm(pXZ2[0])}, ${mm(pXZ2[1])}) mm(肩の軸から ${mm(pole2.r)}mm)`);
  let h2 = Math.max(hFit, Math.round((top2a - 0.06) * 100) / 100), cl2 = motionClear(segsC(pXZ2, camAt(h2, ctr[0], ctr[1])), scenes2);
  while (cl2.d < CLEAR_C && h2 < 1.2) { h2 += 0.01; cl2 = motionClear(segsC(pXZ2, camAt(h2, ctr[0], ctr[1])), scenes2); }
  camC2 = camAt(h2, ctr[0], ctr[1]);
  const K2 = makeCam(camC2, 640, 480);
  let hid = 0, out = 0, n = 0; scenes2.forEach(S => { if (!S.c.solved) return; const r = occlOne(K2, S); hid += r.hid; out += r.out; n += r.n; });
  const top2 = Math.max(...frames(scenes2).map(F => F.top)), c2 = corners(makeCam(camC2, 640, 480));
  console.log(`  レンズの先の高さ ${mm(h2)}mm(組み直した動きの腕のいちばん高い所 ${mm(top2)}mm・腕と柱・吊る腕・カメラの一番近い所 ${mm(cl2.d)}mm ${cl2.what}・種 ${cl2.set}・${cl2.t.toFixed(1)} 秒)`);
  console.log(`  (1) 写す姿勢で 隠れる ${hid}・絵の外 ${out} / ${n} 点`);
  console.log(`  (2) 腕どうし ${scenes2.map(S => S.c.self ? mm(S.c.self.min) + "(" + S.c.self.pair + ")" : "—").join("・")} mm・台や机やピースとの一番近い所 ${scenes2.map(S => mm(S.c.minClear) + "(" + S.c.minClearPair + ")").join("・")} mm`);
  console.log(`  (3) 台の柱は無い(C と同じ)`);
  console.log(`  (4) ${cornerRow(camC2)}`);
  console.log(`      画角を合わせたとき ${fittedRow(camC2)}`);
  console.log(`      上の面のずれ(光線の傾き ${c2.tiltDeg.toFixed(1)}° まで)最大 ${c2.shiftMm.toFixed(1)}mm`);
  console.log(`  (5) 動きの全体 ${scenes2.map(S => S.c.T.toFixed(1)).join("・")} 秒(A は ${timeOf()})`);
  if (hitBad.length) process.exitCode = 1;
}

// =====================================================================
// D: 手首のカメラだけで見つける(目安)
// =====================================================================
console.log(`\n== D 手首のカメラだけ(目安)。つかむ場所の上で爪を開き終えたコマの手首の画角で、置き場を何か所か写すとした数`);
scenes.forEach(S => {
  const c = S.c, foots = [];
  c.stepTimes.forEach(st => {
    const t = st.tHover + 0.4, f = c.motion.reduce((a, b) => Math.abs(b.t - t) < Math.abs(a.t - t) ? b : a);
    const cf = SO101.urdfFK(f.q.map(x => x * D2R), f.jaw * D2R).wrist_camera_link;
    const cam = { tip: [cf.p[0], cf.p[1] + RY, cf.p[2]], dir: [cf.R[2], cf.R[5], cf.R[8]], up: [-cf.R[1], -cf.R[4], -cf.R[7]] };
    const img = SO101.overheadImage(cam, FOV, 640, 480);
    const q = [[0, 0], [640, 0], [640, 480], [0, 480]].map(p => img.unproject(p[0], p[1], c.cell));
    if (q.some(p => !p)) return;
    const w = Math.min(Math.hypot(q[0][0] - q[1][0], q[0][2] - q[1][2]), Math.hypot(q[3][0] - q[2][0], q[3][2] - q[2][2]));
    const h = Math.min(Math.hypot(q[0][0] - q[3][0], q[0][2] - q[3][2]), Math.hypot(q[1][0] - q[2][0], q[1][2] - q[2][2]));
    foots.push([w, h, cam.tip[1] - c.cell]);
  });
  if (!foots.length) { console.log(`  [${S.label}] 手首の足跡が机に届かない`); return; }
  const fw = Math.min(...foots.map(f => f[0])), fh = Math.min(...foots.map(f => f[1]));
  let pb = [Infinity, -Infinity, Infinity, -Infinity];
  c.pieceCells.flat().forEach(xz => { const k = c.cell / 2; pb = [Math.min(pb[0], xz[0] - k), Math.max(pb[1], xz[0] + k), Math.min(pb[2], xz[1] - k), Math.max(pb[3], xz[1] + k)]; });
  const ov = 0.8, nx = Math.ceil((pb[1] - pb[0]) / (Math.min(fw, fh) * ov)), nz = Math.ceil((pb[3] - pb[2]) / (Math.min(fw, fh) * ov)), shots = nx * nz;
  const add = shots * (T_SCAN_MOVE + T_SCAN_SETTLE + T_SHOT);
  console.log(`  [${S.label}] 足跡(上の面の高さ)いちばん狭い所 ${mm(Math.min(fw, fh))}mm 四方・置き場の範囲 ${mm(pb[1] - pb[0])}×${mm(pb[3] - pb[2])}mm → ${shots} か所(重ね 20%)・` +
              `足す時間 ${add.toFixed(1)} 秒(1 か所 ${(T_SCAN_MOVE + T_SCAN_SETTLE + T_SHOT).toFixed(1)} 秒【仮】)→ 全体 ${(c.T + add).toFixed(1)} 秒(いまの ${c.T.toFixed(1)})`);
});
console.log(`  (1)(2) D の写す姿勢は組んでいない(腕どうし・隠れるは見ていない)。(3) 台の柱は無い。(4) mm/画素は camera-budget.js の手首の欄(60° で 0.52〜0.59)。腕の位置の誤差が見つけた位置にそのまま乗る`);
console.log(`\n【仮】縦の画角 60°・柱は腕から 30mm あく所(届く輪の外 20mm から探す)・カメラの半径 25mm・柱の半径 12mm・D の 1 か所 ${(T_SCAN_MOVE + T_SCAN_SETTLE + T_SHOT).toFixed(1)} 秒・縁の余裕 ${MARGIN}`);
