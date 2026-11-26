// Where does the top camera go for the Soma build? (2026-11-26, the user's "カメラの位置とかはどうする？")
// Takes the collision-checked path (tools/soma_collide.js --path-json) and a top camera (by default the C rig of sim/so101.js
// OVERHEAD_C: a pole in front of the arm (x 353 mm, beyond the cube), the lens 540 mm above the table looking straight down, 640x480, vertical view 60 deg), and
// measures, in the model (the arm goes exactly where solved, the pieces sit exactly at their places):
//   rig  : the arm's parts (sim/self-collision.js rounded boxes, the same measure as tools/soma_collide.js) against the rig's
//          boxes, over every frame of the path; the arm's highest point.
//   see  : the top faces that the camera should see: before each pick, the pieces still on the table; after each place, the top
//          cell of every column of the cube built so far. 5 points per face (the centre and 4 points 0.6 x half the cell's side
//          off it: inside the face at any turn). Each point: out of the picture, hidden by the other pieces' cells (the cube's and
//          the table pieces'; ray against their boxes: the table turns are multiples of 90 deg), by the rig's pole (ray against its boxes), by the stand under the base, or by the arm at the photo pose
//          (the arm's shape points within 2.5 mm of the ray: as sim/tools/overhead-occlusion.js).
//   photo: the arm's photo pose: the path's first pose, and a search (a grid in the joint ranges) for a pose that hides the
//          fewest points while staying 5 mm off the table, the stand, the rig, every piece (on the table and the whole cube)
//          and 3 mm off itself (SELF_COLLIDE.WARN). For the best pose: the arm's shape points' nearest distance to any ray.
//   scale: pixels per mm on the cube's top (120 mm) and on the table; how far a cell's top moves in the picture when it sits 1 mm
//          higher (does the picture tell the seat?).
// Not checked: the held piece against the rig, the arm off its path (wobble, sag), the cables, lens distortion, light and shadow,
// the wrist camera, a camera whose view is not straight down, the moves between the path and the photo pose.
// usage: node tools/soma_camera.js outputs/soma/path_160_160.json [--lens-mm X,Y,Z] [--fov 60] [--search 0] [--json out.json]
//        (--search 0: no photo pose search, a minute instead of ten; --near: the photo poses nearest the leave poses first)
//        (--lens-mm: the lens tip, table = 0, arm's pan axis frame as sim/so101.js OVERHEAD_C.lensTipMm; the rig's boxes move
//         with it in x and z, and the pole and the hanger keep their lengths from the lens;
//         --pole-dx-mm D: only the pole, its foot and its bracket move D in x, the boom grows by D)
const T = "C:/Users/genes/Documents/PROJECTS/lerobot/sim/tools/";
const fs = require("fs");
const { load } = require(T + "_load");
load("kinematics.js", "so101.js", "motion.js", "puzzle.js", "models/so101_visuals.js", "models/so101_points.js", "models/so101_tris.js",
     "collide.js", "chikuwa-collision.js", "self-collision.js");
global.window = { SCENE_SKIP_DEFAULT: 1 }; load("scene-data.js"); delete global.window;
const M = SCENE_MODEL, SC = SELF_COLLIDE, CK = CK_COLLIDE;
const path = JSON.parse(fs.readFileSync(process.argv[2], "utf-8"));
const arg = (k, d) => { const i = process.argv.indexOf(k); return i > 0 ? process.argv[i + 1] : d; };
const W = 640, H = 480, FOV = +arg("--fov", String(SO101.CAM_FOV.top)), TOL = 0.0025, KEEP = 0.005;
const RY = path.robot_y_m, C = path.cell_m, A = path.across_m;
const OC = SO101.OVERHEAD_C;
const lens0 = OC.lensTipMm, lensMm = arg("--lens-mm", null) ? arg("--lens-mm").split(",").map(Number) : lens0.slice();
const dLens = [lensMm[0] - lens0[0], lensMm[1] - lens0[1], lensMm[2] - lens0[2]];
// the rig's boxes (table = 0, m). Moved with the lens: x, z shift; the pole and the foot keep their base on the table and stretch
const rig = OC.boxesMm.map(b => {
  const lo = b.lo.slice(), hi = b.hi.slice();
  lo[0] += dLens[0]; hi[0] += dLens[0]; lo[2] += dLens[2]; hi[2] += dLens[2];
  if (b.id === "c_pole") hi[1] += dLens[1];
  else if (b.id !== "c_foot") { lo[1] += dLens[1]; hi[1] += dLens[1]; }
  const dx = +arg("--pole-dx-mm", "0");
  if (["c_pole", "c_foot", "c_bracket"].includes(b.id)) { lo[0] += dx; hi[0] += dx; }
  if (b.id === "c_arm") hi[0] += dx;
  return { id: b.id, lo: lo.map(v => v / 1000), hi: hi.map(v => v / 1000) };
});
const DESIGN_KEEP_MM = 25;      // hardware/c_mount.py: KEEP_MARGIN 20 + CABLE_R 5 around the pole (both GUESS)
const LENS = lensMm.map(v => v / 1000);
const IMG = SO101.overheadImage({ tip: LENS, dir: OC.camDir, up: OC.camUp }, FOV, W, H);
const inImage = q => q && q[0] >= 0 && q[0] < W && q[1] >= 0 && q[1] < H;
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

// ---- the arm (world: table at Y = 0, base_link at RY)
const toArm = p => [p[0], p[1] - RY, p[2]];
const coarse = SC.coarseSets(), fine = SC.fineSets(), ALL = ["j1", "j2", "j3", "j4", "j5", "jaw", "cam"];   // as soma_collide.js (the base sits on the stand)
function armVs(q, jaw, obs) {
  const stC = SC.placed(q, jaw, coarse); let stF = null, best = { d: Infinity };
  for (const g of ALL) for (const o of obs) {
    let r = CK.partsObstacleClearance(stC[g], o.ob);
    if (r.clearance < 0.012) { stF = stF || SC.placed(q, jaw, fine); r = CK.partsObstacleClearance(stF[g], o.ob); }
    if (r.clearance < best.d) best = { d: r.clearance, g, what: o.what };
  }
  return best;
}
const boxOb = (lo, hi, what) => ({ ob: { type: "box", c: toArm(lo.map((v, i) => (v + hi[i]) / 2)), half: lo.map((v, i) => (hi[i] - v) / 2), yaw: 0 }, what });
const rigObs = rig.map(b => boxOb(b.lo, b.hi, "rig " + b.id));
const st = path.stand, standLo = st.c.map((v, i) => v - st.half[i]), standHi = st.c.map((v, i) => v + st.half[i]);
const standOb = boxOb(standLo, standHi, "stand");
function armPts(q, jaw) {                                      // the arm's shape points (world), as overhead-occlusion.js
  const fk = KIN.fk(SO101.axes, q, SO101.tool), out = [];
  Object.keys(M.ARM.points).forEach(g => {
    if (g === "base") { M.ARM.points.base.forEach(p => out.push([p[0], p[1] + RY, p[2]])); return; }
    const fr = fk.frames[(g === "jaw" || g === "cam") ? 4 : (+g.slice(1) - 1)];
    M.ARM.points[g].forEach(p0 => {
      const p = g === "jaw" ? M.jawRotate(p0, jaw) : p0, w = KIN.add(fr.p, KIN.matVec(fr.R, KIN.sub(p, fr.o)));
      out.push([w[0], w[1] + RY, w[2]]);
    });
  });
  return out;
}

// ---- the pieces: cube cells (axis-aligned boxes) and table cells (turned; only their tops are sampled)
const P = path.pieces;
const cubeBox = c => ({ lo: [c[0] - A / 2, c[1] - C / 2, c[2] - A / 2], hi: [c[0] + A / 2, c[1] + C / 2, c[2] + A / 2] });
function tops(cells) {                                         // the top cell of each column (x, z rounded to 1 mm)
  const col = {};
  cells.forEach(c => { const k = Math.round(c[0] * 1000) + "," + Math.round(c[2] * 1000); if (!col[k] || c[1] > col[k][1]) col[k] = c; });
  return Object.values(col);
}
function facePts(c) {                                          // 5 points on the top face, inside it at any turn
  const y = c[1] + C / 2, r = 0.6 * A / 2;
  return [[0, 0], [r, 0], [-r, 0], [0, r], [0, -r]].map(o => [c[0] + o[0], y, c[2] + o[1]]);
}
function rayHitsBox(p, b) {                                   // the segment lens -> p (stopping 1 mm short of p) through box b?
  const d = [p[0] - LENS[0], p[1] - LENS[1], p[2] - LENS[2]], L = Math.hypot(...d);
  let t0 = 0, t1 = 1 - 0.001 / L;
  for (let a = 0; a < 3; a++) {
    if (Math.abs(d[a]) < 1e-12) { if (LENS[a] < b.lo[a] || LENS[a] > b.hi[a]) return false; continue; }
    let u = (b.lo[a] - LENS[a]) / d[a], v = (b.hi[a] - LENS[a]) / d[a];
    if (u > v) [u, v] = [v, u];
    t0 = Math.max(t0, u); t1 = Math.min(t1, v);
    if (t0 > t1) return false;
  }
  return true;
}
function hiddenByPts(p, pts) {
  const d = KIN.sub(p, LENS), L = Math.hypot(...d), u = d.map(v => v / L);
  for (const s of pts) {
    const w = KIN.sub(s, LENS), t = dot(w, u);
    if (t <= TOL || t >= L - 0.003) continue;
    const e = [w[0] - u[0] * t, w[1] - u[1] * t, w[2] - u[2] * t];
    if (e[0] * e[0] + e[1] * e[1] + e[2] * e[2] < TOL * TOL) return true;
  }
  return false;
}
const lowRig = rig.filter(b => b.lo[1] < LENS[1] - 1e-6 && b.id !== "c_cam");   // what is below the lens can hide

// the views: [label, [{piece, cell, others: boxes that can hide it}]]
if (P.some(p => Math.abs(p.table_turn_deg / 90 - Math.round(p.table_turn_deg / 90)) > 1e-6)) throw new Error("table turns not multiples of 90 deg: the table cells are not axis-aligned boxes here");
const views = [];
for (let n = 0; n < P.length; n++) {
  const onTable = [];
  for (let m = n; m < P.length; m++) tops(P[m].table_cells).forEach(c => onTable.push({ piece: m, c }));
  views.push({ label: `before pick ${n + 1}`, faces: onTable, boxes: [].concat(...P.slice(0, n).map(p => p.cube_cells.map(cubeBox)), ...P.slice(n).map(p => p.table_cells.map(cubeBox))) });
  const built = [].concat(...P.slice(0, n + 1).map((p, m) => p.cube_cells.map(c => ({ piece: m, c }))));
  const later = [];
  for (let m = n + 1; m < P.length; m++) tops(P[m].table_cells).forEach(c => later.push({ piece: m, c }));
  views.push({ label: `after place ${n + 1}`, faces: tops(built.map(b => b.c)).map(c => ({ piece: built.find(b => b.c === c).piece, c })).concat(later),
               boxes: built.map(b => cubeBox(b.c)).concat(...P.slice(n + 1).map(p => p.table_cells.map(cubeBox))) });
}
function countView(v, pts) {
  const r = { n: 0, out: 0, cube: 0, rig: 0, stand: 0, arm: 0 };
  v.faces.forEach(f => facePts(f.c).forEach(p => {
    r.n++;
    if (!inImage(IMG.project(p))) { r.out++; return; }
    const self = cubeBox(f.c);
    if (v.boxes.some(b => b.lo[1] !== self.lo[1] || b.lo[0] !== self.lo[0] || b.lo[2] !== self.lo[2] ? rayHitsBox(p, b) : false)) { r.cube++; return; }
    if (lowRig.some(b => rayHitsBox(p, b))) { r.rig++; return; }
    if (rayHitsBox(p, { lo: standLo, hi: standHi })) { r.stand++; return; }
    if (pts && hiddenByPts(p, pts)) { r.arm++; return; }
  }));
  r.bad = r.out + r.cube + r.rig + r.stand + r.arm;
  return r;
}

// ---- 1. the rig against the arm over the whole path
let worst = { d: Infinity }, topY = -Infinity, topAt = -1;
path.frames.forEach((f, i) => {
  const r = armVs(f.q, f.jaw, rigObs);
  if (r.d < worst.d) worst = Object.assign(r, { frame: i, n: f.n, ph: f.ph });
  { const y = Math.max(...armPts(f.q, f.jaw).map(p => p[1])); if (y > topY) { topY = y; topAt = i; } }
});
console.log(`[top camera] lens (${lensMm.join(", ")}) mm (table = 0), straight down, ${W}x${H}, vertical view ${FOV} deg; arm base raised ${path.raise_mm} mm, cube ${path.cube_centre_from_pan_mm} mm in front of the pan axis`);
console.log(`[rig vs arm, ${path.frames.length} frames] smallest gap ${(worst.d * 1000).toFixed(1)} mm (${worst.g} - ${worst.what}, frame ${worst.frame}: piece ${worst.n + 1} ${worst.ph}); the arm's highest point ${(topY * 1000).toFixed(0)} mm above the table (frame ${topAt}), the lens ${lensMm[1]} mm; the rig design keeps ${DESIGN_KEEP_MM} mm around the pole (hardware/c_mount.py, GUESS): ${worst.d * 1000 >= DESIGN_KEEP_MM ? "kept" : "NOT kept"}`);

// ---- 2. what the camera sees with no arm, and at the path's first pose
const sum = rs => rs.reduce((a, r) => { Object.keys(r).forEach(k => a[k] = (a[k] || 0) + r[k]); return a; }, {});
const fmt = r => `${r.bad}/${r.n} lost (out of the picture ${r.out}, behind other pieces ${r.cube}, the rig ${r.rig}, the stand ${r.stand}, the arm ${r.arm})`;
const noArm = views.map(v => countView(v, null));
console.log(`[no arm] ${fmt(sum(noArm))}`);
views.forEach((v, i) => { if (noArm[i].bad) console.log(`    ${v.label}: ${fmt(noArm[i])}`); });
const f0 = path.frames[0], startPts = armPts(f0.q, f0.jaw), atStart = views.map(v => countView(v, startPts));
console.log(`[the path's first pose (${f0.q.map(v => v.toFixed(0)).join(", ")}) deg] ${fmt(sum(atStart))}`);

// ---- 3. a photo pose
const allCube = [].concat(...P.map(p => p.cube_cells.map(c => { const b = cubeBox(c); return boxOb(b.lo, b.hi, "cube"); })));
const allTable = [].concat(...P.map(p => p.table_cells.map(c => boxOb([c[0] - 0.029, c[1] - C / 2, c[2] - 0.029], [c[0] + 0.029, c[1] + C / 2, c[2] + 0.029], "table piece"))));
// (a table cell turned by any angle fits in a 58 mm square around its centre: 39 mm x sqrt 2 = 55.2 mm)
const allPts = [].concat(...views.map(v => [].concat(...v.faces.map(f => facePts(f.c)))));
const sq = allPts.map(p => IMG.project(p)).map(q => inImage(q) ? [Math.floor(q[0]), Math.floor(q[1])] : null);
function quick(pts) {
  const mask = new Uint8Array(W * H);
  pts.forEach(p => {
    const d = [p[0] - LENS[0], p[1] - LENS[1], p[2] - LENS[2]], zc = dot(d, IMG.fwd);
    if (zc <= 0) return;
    const u = W / 2 + IMG.f * dot(d, IMG.right) / zc, v = H / 2 - IMG.f * dot(d, IMG.up) / zc, r = Math.ceil(IMG.f * TOL / zc + 1);
    for (let j = Math.max(0, Math.floor(v - r)); j <= Math.min(H - 1, v + r); j++)
      for (let i = Math.max(0, Math.floor(u - r)); i <= Math.min(W - 1, u + r); i++) mask[j * W + i] = 1;
  });
  return sq.reduce((a, q) => a + (q && mask[q[1] * W + q[0]] ? 1 : 0), 0);
}
const LIM = SO101.limitsDeg, grid = [];
if (arg("--search", "1") !== "0") for (let pan = -110; pan <= 110; pan += 10) for (let lift = -100; lift <= 100; lift += 20)
  for (let elbow = -90; elbow <= 90; elbow += 15) for (let flex = -90; flex <= 90; flex += 30) grid.push([pan, lift, elbow, flex, 0]);
const inLim = q => !LIM || q.every((v, i) => !LIM[i] || (v >= LIM[i][0] && v <= LIM[i][1]));
const nBase = M.ARM.points.base.length, jaw0 = f0.jaw;
const cand = grid.filter(inLim).map(q => {
  const pts = armPts(q, jaw0), low = Math.min(...pts.slice(nBase).map(p => p[1]));
  return low < KEEP ? null : { q, low, quick: quick(pts.slice(nBase)) };
}).filter(Boolean).sort((a, b) => a.quick - b.quick);
// --near (2026-11-26): among the poses that hide nothing when drawn into the picture, the nearest first: the largest joint change
// (pan, lift, elbow, flex) from the poses the arm leaves the cube at (the end of each piece's "leave"). A far photo pose made the
// MuJoCo arm's shoulder load pass the stop line on the way (puzzle_run.py --soma-correct, 2026-11-26)
const leaveEnds = path.frames.filter((f, i) => f.ph === "leave" && (!path.frames[i + 1] || path.frames[i + 1].ph !== "leave")).map(f => f.q);
const farFrom = q => Math.max(...leaveEnds.map(e => Math.max(...[0, 1, 2, 3].map(j => Math.abs(q[j] - e[j])))));
let order = cand.slice(0, 600);
if (process.argv.includes("--near")) order = cand.filter(o => o.quick === 0).sort((a, b) => farFrom(a.q) - farFrom(b.q));
const best = [];
for (const o of order) {               // the 600 that hide the fewest when drawn into the picture
  if (best.length >= 5) break;
  const self = SC.pose(o.q, jaw0);
  if (self.clearance < SC.WARN) continue;
  const g = armVs(o.q, jaw0, [standOb].concat(rigObs, allCube, allTable));
  if (g.d < KEEP) continue;
  const pts = armPts(o.q, jaw0), rs = sum(views.map(v => countView(v, pts)));
  best.push({ q: o.q, far_deg: +farFrom(o.q).toFixed(0), lost: rs.bad, arm: rs.arm, quick: o.quick, gap_mm: +(g.d * 1000).toFixed(1), gap_to: g.what, self_mm: +(self.clearance * 1000).toFixed(1), low_mm: +(o.low * 1000).toFixed(0) });
}
console.log(`[photo pose search] grid ${grid.length} (5 mm above the table: ${cand.length}); the fewest hidden, 5 mm off the stand, the rig, every piece, 3 mm off itself:`);
function rayMargin(pts) {                                     // the arm points' nearest distance to any ray of a visible point
  let m = Infinity;
  views.forEach(v => v.faces.forEach(fc => facePts(fc.c).forEach(p => {
    if (!inImage(IMG.project(p))) return;
    const d = KIN.sub(p, LENS), L = Math.hypot(...d), u = d.map(x => x / L);
    for (const s of pts) {
      const w = KIN.sub(s, LENS), t = dot(w, u);
      if (t <= TOL || t >= L - 0.003) continue;
      const e = Math.hypot(w[0] - u[0] * t, w[1] - u[1] * t, w[2] - u[2] * t);
      if (e < m) m = e;
    }
  })));
  return m;
}
if (best.length) best[0].ray_margin_mm = +(rayMargin(armPts(best[0].q, jaw0)) * 1000).toFixed(1);
best.forEach(o => console.log(`    (${o.q.join(", ")}) deg (at most ${o.far_deg} deg from a leave pose): lost ${o.lost} (by the arm ${o.arm}); nearest ${o.gap_mm} mm (${o.gap_to}), self ${o.self_mm} mm (line 3 mm), lowest ${o.low_mm} mm${o.ray_margin_mm !== undefined ? `; the arm stays ${o.ray_margin_mm} mm off every ray (hidden = within ${TOL * 1000} mm)` : ""}`));

// ---- 4. scale
const f = IMG.f, cubeTop = 3 * C, tab = 0;
const cubeCx = P.reduce((a, p) => a + p.cube_cells.reduce((b, c) => b + c[0], 0), 0) / P.reduce((a, p) => a + p.cube_cells.length, 0);
const cubeCz = P.reduce((a, p) => a + p.cube_cells.reduce((b, c) => b + c[2], 0), 0) / P.reduce((a, p) => a + p.cube_cells.length, 0);
const rFar = Math.max(...P.flatMap(p => p.cube_cells.map(c => Math.hypot(c[0] - LENS[0], c[2] - LENS[2]) + A / 2)));
const pxmm = h => f / (LENS[1] - h) / 1000;
const seatPx = (r, h) => f * r * (1 / (LENS[1] - h - 0.001) - 1 / (LENS[1] - h));
console.log(`[scale] ${pxmm(cubeTop).toFixed(2)} px/mm on the cube's top (120 mm), ${pxmm(tab).toFixed(2)} px/mm on the table; the cube's centre is ${(Math.hypot(cubeCx - LENS[0], cubeCz - LENS[2]) * 1000).toFixed(0)} mm off the lens axis; a top cell 1 mm higher moves at most ${seatPx(rFar, cubeTop).toFixed(2)} px (its farthest corner, ${(rFar * 1000).toFixed(0)} mm off the axis)`);

const out = { lens_mm: lensMm, fov_deg: FOV, raise_mm: path.raise_mm, rig_vs_arm: { gap_mm: +(worst.d * 1000).toFixed(1), part: worst.g, what: worst.what, frame: worst.frame },
  arm_top_mm: +(topY * 1000).toFixed(0), no_arm: sum(noArm), at_first_pose: sum(atStart), photo_candidates: best,
  views: views.map((v, i) => ({ label: v.label, no_arm: noArm[i], at_first_pose: atStart[i] })),
  scale: { px_per_mm_cube_top: +pxmm(cubeTop).toFixed(3), px_per_mm_table: +pxmm(tab).toFixed(3), seat_1mm_px_max: +seatPx(rFar, cubeTop).toFixed(3) } };
if (arg("--json", null)) fs.writeFileSync(arg("--json"), JSON.stringify(out, null, 1));
