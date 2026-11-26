// 列 P: 置けたかの画像の確認の片寄りの原因を 1 つずつ外す(夜の作業 2026-11-19 の 2 回目。点検 C42e)。
//   前の回の tools/puzzle_mj_imgbias.py は「3 個を盤の目標に並べて描く → detect → 目標と match」で L 1.05mm・J 1.12mm・O 0.4〜0.5mm の片寄りを出した。
//   ここでは物理を通さず、絵の条件を 1 つずつ変えて、どれで片寄りが消えるか・変わるかを見る。
//   sim/ のファイルは読むだけ(plan-cli.js と同じ順で読み込み、同じ render・detect・match を呼ぶ)。
//
//   流し方(ルートから): node tools/puzzle_imgbias_iso.js [--set 2] [--out outputs/night/<名前>]
//   出力: <out>/result.json(条件ごとの行)と、画面に表
//
//   条件(どれも上のカメラ・較正は sim のまま・noise 0 と 3):
//     all3        3 個とも盤の目標。腕は始まりの姿勢(前の回の imgbias と同じ)
//     all3_mjpose 3 個とも盤の目標。腕と爪は MuJoCo の回の終わりの写す姿勢(爪が開いている)
//     all3_mjstate 3 個とも MuJoCo の回の終わりの真値(--mjrun の run.json の final_states)。腕もその回の終わり = MuJoCo の回の絵を noise だけ替えて描き直す
//     noarm       3 個とも盤の目標。腕の三角形を描かない
//     alone_<名>  そのピースだけを盤の目標に描き、detect にもそのピースだけを教える
//     yaw_<名>_<d> そのピースだけ、盤の目標から向きを d 度回す(片寄りが向きに付いて動くか)
//     pos_<名>_<dx>_<dz> そのピースだけ、盤の目標から dx・dz mm ずらす(片寄りが絵の中の場所で変わるか)
//   1 行: {cond, noise, seed, name, off_mm(いちばん悪いマスのずれ), phi(向きのずれ 度。測った − 描いた), dx_mm, dz_mm(重心のずれ), mode, sil, nPix}
"use strict";
const fs = require("fs"), path = require("path");
const ROOTP = path.resolve(__dirname, "..");
const { load } = require(path.join(ROOTP, "sim", "tools", "_load"));
load("kinematics.js", "so101.js", "motion.js", "puzzle.js", "models/so101_visuals.js", "models/so101_points.js", "models/so101_tris.js",
     "collide.js", "chikuwa-collision.js", "self-collision.js");
global.window = { SCENE_SKIP_DEFAULT: 1 }; load("scene-data.js"); delete global.window;
load("vision.js", "vision-wrist.js", "robot-map.js");
const M = SCENE_MODEL, V = VISION;

function arg(name, def) { const i = process.argv.indexOf(name); return i > 0 ? process.argv[i + 1] : def; }
const SET = +arg("--set", 2);
const stamp = new Date().toISOString().replace(/[-:T]/g, "").slice(0, 15);
const OUT = path.resolve(ROOTP, arg("--out", path.join("outputs", "night", stamp + "_imgbias_iso")));
const MJRUN = arg("--mjrun", path.join(ROOTP, "outputs", "night", "20261119_233844_mj_set2_seed1_24348", "run.json"));

function fakeK1() {   // sim/plan-cli.js の fakeK1 の写し(同じ値。変わったら合わせる)
  const cal = {}, sign = {}, offset = {};
  ROBOT_MAP.JOINTS.concat(["gripper"]).forEach((m, i) => { cal[m] = { id: i + 1, drive_mode: 0, homing_offset: 0, range_min: 2048 - 1700, range_max: 2048 + 1700 }; });
  ROBOT_MAP.JOINTS.forEach((m, i) => { sign[m] = i % 2 ? -1 : 1; offset[m] = (i - 2) * 7.5; });
  return { robot_id: "fake", date: "-", verified: true, calibration: cal, sign, offset, gripper: [{ lerobot: 2, gap_mm: 0 }, { lerobot: 60, gap_mm: 30 }] };
}

const def = M.defaultUiValues();
const base = { SET, GAP: def.GAP, STAND: 1, SOL: 0, NO_REFINE: true };
const d0 = M.buildFromUi(base), c0 = d0._check;
const map = ROBOT_MAP.make(fakeK1(), M.gripFor(c0.cell));
const cam = d0.overheadCamera, fov = SO101.CAM_FOV.top;
const scene = JSON.parse(fs.readFileSync(path.join(ROOTP, "sim", "out", "physics", `set${SET}_gap15_stand1`, "scene.json"), "utf8"));
const TG = scene.pieces.map(p => ({ name: p.name, x: p.end.pos[0] * 1000, z: p.end.pos[2] * 1000, yaw: p.end.rot }));
const start = c0.motion[0];
const mj = fs.existsSync(MJRUN) ? JSON.parse(fs.readFileSync(MJRUN, "utf8")) : null;

function draw(pieces, q, jaw, opts) {   // pieces: [{x,z,yaw} | null](null は描かない)
  const lay = { slots: pieces.map(p => p ? [p.x / 1000, p.z / 1000] : [5, 5]), rots: pieces.map(p => p ? p.yaw : 0) };
  let tris = V.worldTris(d0, lay, q, jaw, {});
  if (opts.noArm) tris = tris.filter(t => t.tag !== "arm");
  return V.render(tris, cam, { fov, noise: opts.noise, seed: opts.seed });
}
function centroid(shape, x, z, yaw) { const C = V.cellsOf(shape, c0.pitch, x / 1000, z / 1000, yaw); return [C.reduce((s, c) => s + c[0], 0) / C.length * 1000, C.reduce((s, c) => s + c[1], 0) / C.length * 1000]; }

const rows = [];
function run(cond, pieces, q, jaw, opts, onlyIdx) {
  // onlyIdx: detect に教えるピースの番号(null で全部)。pieces の null でない物と同じにする
  const im = draw(pieces, q, jaw, opts);
  const known = { pieces: onlyIdx == null ? c0.pieces : onlyIdx.map(i => c0.pieces[i]), pitch: c0.pitch, cell: c0.cell };
  const det = V.detect(im, { cam, fov }, known);
  if (det.why) { rows.push({ cond, noise: opts.noise, seed: opts.seed, why: det.why }); return; }
  known.pieces.forEach((p, k) => {                 // detect は out[p.idx] に書く(教えた物だけ。ほかは空き)
    const idx = p.idx, m = det.layout.pieces[idx], t = pieces[idx];
    const mt = V.match(p.shape, c0.pitch, [t.x / 1000, t.z / 1000, t.yaw], [m.x / 1000, m.z / 1000, m.yaw]);
    const ct = centroid(p.shape, t.x, t.z, t.yaw), cm = centroid(p.shape, m.x, m.z, m.yaw);
    const per = det.per[k] || {};
    rows.push({ cond, noise: opts.noise, seed: opts.seed, name: p.name, off_mm: +(mt.worstCell * 1000).toFixed(3), phi: +mt.phi.toFixed(3),
                dx_mm: +(cm[0] - ct[0]).toFixed(3), dz_mm: +(cm[1] - ct[1]).toFixed(3), mode: per.mode, sil: per.sil, nPix: per.nPix });
  });
}

const t0 = Date.now();
const NOISES = [0, 3], SEEDS = [1, 2, 3];
let mjQ = null, mjJaw = null, mjStates = null;
if (mj) {
  const Lend = mj.observations[mj.observations.length - 1].lerobot, u = map.toUrdf(Lend);
  mjQ = u.q; mjJaw = u.jaw;
  mjStates = mj.final_states.map(s => ({ x: s.x, z: s.z, yaw: s.yaw }));
}
for (const noise of NOISES) for (const seed of SEEDS) {
  if (noise === 0 && seed > 1) continue;            // noise 0 は種で変わらない
  const o = { noise, seed };
  run("all3", TG, start.q, start.jaw, o, null);
  run("noarm", TG, start.q, start.jaw, Object.assign({ noArm: true }, o), null);
  if (mj) { run("all3_mjpose", TG, mjQ, mjJaw, o, null); run("all3_mjstate", mjStates, mjQ, mjJaw, o, null); }
  TG.forEach((t, i) => {
    const only = TG.map((u, j) => j === i ? u : null);
    run("alone_" + t.name, only, start.q, start.jaw, o, [i]);
    if (mj) { const s = mjStates.map((u, j) => j === i ? u : null); run("alone_mjstate_" + t.name, s, mjQ, mjJaw, o, [i]); }
    for (const d of [-2, -1, 1, 2]) run(`yaw_${t.name}_${d}`, TG.map((u, j) => j === i ? Object.assign({}, u, { yaw: u.yaw + d }) : null), start.q, start.jaw, o, [i]);
    for (const [dx, dz] of [[-20, 0], [20, 0], [0, -20], [0, 20]]) run(`pos_${t.name}_${dx}_${dz}`, TG.map((u, j) => j === i ? Object.assign({}, u, { x: u.x + dx, z: u.z + dz }) : null), start.q, start.jaw, o, [i]);
  });
}
// 絵の細かさ: 1 個だけ・盤の目標から x を 0〜1.5mm(0.25mm 刻み)ずらし、640×480 と 1280×960 で向きのずれの散らばりを比べる
//   (片寄りが画素の格子のせいなら、細かくすると小さくなる)
const resRows = [];
for (const [W, H] of [[640, 480], [1280, 960]]) TG.forEach((t, i) => {
  for (let s = 0; s <= 6; s++) {
    const pcs = TG.map((u, j) => j === i ? Object.assign({}, u, { x: u.x + s * 0.25 }) : null);
    const lay = { slots: pcs.map(p => p ? [p.x / 1000, p.z / 1000] : [5, 5]), rots: pcs.map(p => p ? p.yaw : 0) };
    const im = V.render(V.worldTris(d0, lay, start.q, start.jaw, {}), cam, { fov, noise: 0, seed: 1, W, H });
    const det = V.detect(im, { cam, fov }, { pieces: [c0.pieces[i]], pitch: c0.pitch, cell: c0.cell });
    if (det.why) { resRows.push({ W, name: t.name, dx: s * 0.25, why: det.why }); continue; }
    const m = det.layout.pieces[c0.pieces[i].idx], p = pcs[i];
    const mt = V.match(c0.pieces[i].shape, c0.pitch, [p.x / 1000, p.z / 1000, p.yaw], [m.x / 1000, m.z / 1000, m.yaw]);
    resRows.push({ W, name: t.name, dx: s * 0.25, off_mm: +(mt.worstCell * 1000).toFixed(3), phi: +mt.phi.toFixed(3), mode: det.per[0].mode });
  }
});
// 画素 1 つが机の上で何 mm か(絵の真ん中の画素と、その右隣を机に戻した点の差)
const mmPerPx = [640, 1280].map(W => { const H = W * 3 / 4, oi = SO101.overheadImage(cam, fov, W, H);   // 絵の真ん中の画素で測る
  const a = oi.unproject(W / 2, H / 2, 0), b = oi.unproject(W / 2 + 1, H / 2, 0);
  return { W, mm: +(Math.hypot(a[0] - b[0], a[2] - b[2]) * 1000).toFixed(3) }; });
// MuJoCo の回の最後の上の絵(実際の .rgb)をそのまま detect し、ここで描き直した絵と画素を比べる
let mjShot = null;
if (mj) {
  const ob = mj.observations[mj.observations.length - 1];
  if (fs.existsSync(ob.img)) {
    const buf = fs.readFileSync(ob.img), im0 = { W: ob.W, H: ob.H, rgb: new Uint8ClampedArray(buf.buffer, buf.byteOffset, buf.length) };
    const det = V.detect(im0, { cam, fov }, { pieces: c0.pieces, pitch: c0.pitch, cell: c0.cell });
    const re = draw(mjStates, mjQ, mjJaw, { noise: 0, seed: 1 });
    let nd = 0, nBig = 0; for (let q = 0; q < buf.length; q++) { const d = Math.abs(buf[q] - re.rgb[q]); if (d) nd++; if (d > 12) nBig++; }
    mjShot = { img: ob.img, why: det.why || null, rows: det.why ? [] : c0.pieces.map((p, k) => { const m = det.layout.pieces[p.idx], t = mjStates[k];
      const mt = V.match(p.shape, c0.pitch, [TG[k].x / 1000, TG[k].z / 1000, TG[k].yaw], [m.x / 1000, m.z / 1000, m.yaw]);
      return { name: p.name, off_vs_target_mm: +(mt.worstCell * 1000).toFixed(3), phi_vs_target: +mt.phi.toFixed(3) }; }),
      bytes_differ_from_redraw_noise0: nd, bytes_differ_more_than_12: nBig, bytes: buf.length };
  }
}
// まとめ: 条件 × ピースごとに、noise ごとの平均
const sum = {};
rows.forEach(r => { if (r.why) return; const k = `${r.cond}|${r.name}|n${r.noise}`; (sum[k] = sum[k] || []).push(r); });
const table = Object.keys(sum).map(k => { const a = sum[k], av = f => +(a.reduce((s, r) => s + r[f], 0) / a.length).toFixed(3); const [cond, name, n] = k.split("|");
  return { cond, name, noise: n, n: a.length, off_mm: av("off_mm"), phi: av("phi"), dx_mm: av("dx_mm"), dz_mm: av("dz_mm"), modes: [...new Set(a.map(r => r.mode))].join("") }; });
fs.mkdirSync(OUT, { recursive: true });
fs.writeFileSync(path.join(OUT, "result.json"), JSON.stringify({ set: SET, targets: TG, mjrun: mj ? MJRUN : null, mjStates, start_q: start.q, start_jaw: start.jaw, mjQ, mjJaw, table, rows, resRows, mmPerPx, mjShot, sec: (Date.now() - t0) / 1000 }, null, 1));
console.log("cond                 name noise  n  off_mm    phi   dx_mm   dz_mm mode");
table.forEach(t => console.log(`${t.cond.padEnd(20)} ${t.name.padEnd(4)} ${t.noise.padEnd(4)} ${String(t.n).padStart(2)} ${t.off_mm.toFixed(3).padStart(7)} ${t.phi.toFixed(3).padStart(6)} ${t.dx_mm.toFixed(3).padStart(7)} ${t.dz_mm.toFixed(3).padStart(7)} ${t.modes}`));
console.log("res: W name  dx    off_mm   phi  mode");
resRows.forEach(r => console.log(`res: ${r.W} ${r.name} ${r.dx.toFixed(2)} ${r.why ? r.why : r.off_mm.toFixed(3) + " " + r.phi.toFixed(3) + " " + r.mode}`));
console.log("mm per px:", JSON.stringify(mmPerPx));
console.log("MuJoCo shot:", JSON.stringify(mjShot));
rows.filter(r => r.why).forEach(r => console.log("FAILED", r.cond, r.noise, r.seed, r.why));
console.log("wrote", OUT, "in", ((Date.now() - t0) / 1000).toFixed(1), "s");
