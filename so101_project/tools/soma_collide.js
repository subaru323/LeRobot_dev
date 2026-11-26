// Does anything hit while the SO-101 (sim/ model) builds the Soma cube by the plan (tools/soma_plan.py) at the spots found by
// tools/soma_reach.js? (2026-11-24, after 点検 C65.) One setting (--raise-mm, --cx-mm) of outputs/soma/reach.json.
//
// The path per piece (the tool straight down, sim/motion.js topDown, exact jaw direction; the tool point on the fixed finger's
// face, C/2 from the gripped cell's centre; all moves across go around the pan axis at the highest carry height):
//   to the pick  : from above the last place to above the pick, the jaw open (46.1 mm)
//   pick         : down with the open fixed finger SHIFT (3 mm) off the cell's face, across to it (touching),
//                  close, up (holding the piece)
//   carry        : across to above the place, the jaw turning by the table turn (the way round the arm can follow)
//   place        : down to the place (or the release height for a drop); leave: open, SHIFT off, up
// Every 5 mm of tool travel (and every 5 deg of turn, every 1 mm of SHIFT) the pose is solved and measured.
// Measured (the smallest gap, m; negative = into it):
//   arm   : the arm's parts (sim/self-collision.js rounded boxes, the 1.5 mm level: they are up to 1.8 mm FATTER than the arm,
//           so a gap >= 0 here is a gap in the model) against: the pieces already in the cube, the pieces still on the table,
//           the stand under the base (a box under the base's footprint), the table. The base itself is not checked (it sits there)
//   held  : the held piece's cells (boxes, sampled on their faces every 4 mm) against the pieces in the cube, the pieces on the
//           table, the stand, and the arm's parts other than the two fingers (j5 = the body with the fixed finger, jaw = the
//           moving finger; they hold the gripped cell). The other cells of the held piece ARE checked against j5 and jaw.
//   "(grip)": the fingers on the cell they hold (closing, opening): reported, not judged. "(shifted off)": the same 3 mm off, judged.
//   "(seat)": the held piece against the cube while it is lowered into its place, and against the table at the pick and at a
//   first-layer place. It meets them by design (the clearance c sideways, 0 under it at the end), so only < -0.1 mm counts.
//   The piece about to be picked counts as a table piece until it is held; the piece let go counts as the cube when leaving.
// self  : the arm against itself along the whole path (sim/self-collision.js motionCheck, and the closest pair per piece/phase).
// Checks of this script: where it puts the held piece at the pick / the place against the piece on the table / in the cube. It uses
// the same REACH both ways, so it checks the turns and the jaw directions, not REACH itself. The verdict includes self (< 0 = NG).
// The plan (x, y, z up) goes to the screen (X, Y up, Z) as (x, z, -y): right-handed, the same hand as the STL (点検 C67 の 1).
// Not checked: from the rest pose to the first pick and back; joint-space moves between the frames (the path is Cartesian).
// usage note: the cube and the table pieces do not move (MuJoCo moved the whole cube by 1.2-2.5 mm after the first piece).
// Pieces: cells 40 mm tall and 40 - c across (--clear-mm, default 1.0), no chamfer. The cube on the table, centre --cx-mm in front
// of the pan axis. Model only: the arm goes exactly where solved (no sag, no servo lag), the pieces do not move unless held.
// usage: node tools/soma_collide.js outputs/soma/plan.json outputs/soma/reach.json --raise-mm 160 --cx-mm 160 [--clear-mm 1.0]
//        [--json out.json] [--path-json path.json]   (the measured path, for tools/puzzle_run.py --soma)
//        [--photo-q A,B,C,D,E]   (2026-11-26) after each piece's leave: to this pose (URDF deg) in joint space, a photo there
//                                 (the frame marked shoot), and back, every 2 deg checked as the rest (phase "photo")
//        --serve                 (2026-11-26) no batch: one JSON request per line on stdin, one answer per line on stdout, for
//                                 tools/puzzle_run.py --soma-correct: {n, place_mm: [dx, dz], cube_off_mm: [[dx, dz] per piece
//                                 placed], start_q: [5], start_jaw, last: {cell_mm: [x, z], yaw} | null} -> piece n's frames
//                                 (the place moved by place_mm, the pieces in the cube taken at their offsets), checked as here
const T = "C:/Users/genes/Documents/PROJECTS/lerobot/sim/tools/";
const fs = require("fs");
const { load } = require(T + "_load");
load("kinematics.js", "so101.js", "motion.js", "puzzle.js", "models/so101_visuals.js", "models/so101_points.js", "models/so101_tris.js",
     "collide.js", "chikuwa-collision.js", "self-collision.js");
global.window = { SCENE_SKIP_DEFAULT: 1 }; load("scene-data.js"); delete global.window;
const M = SCENE_MODEL, SC = SELF_COLLIDE, CK = CK_COLLIDE;
const plan = JSON.parse(fs.readFileSync(process.argv[2], "utf-8"));
const reach = JSON.parse(fs.readFileSync(process.argv[3], "utf-8"));
const arg = (k, d) => { const i = process.argv.indexOf(k); return i > 0 ? process.argv[i + 1] : d; };
const RAISE = +arg("--raise-mm", "160"), CXMM = +arg("--cx-mm", "160"), CLR = +arg("--clear-mm", "1.0") / 1000;
const row = reach.rows.find(r => r.raise_mm === RAISE && r.cube_centre_from_pan_mm === CXMM);
if (!row || !row.all_ok) throw new Error(`no reachable row for raise ${RAISE} mm, cube ${CXMM} mm in ${process.argv[3]}`);
const C = plan.cell_mm / 1000, BAND = plan.grip_band_mm / 1000, GY = M.GRASP_Y, DEGR = Math.PI / 180;
const ROBOT_Y = M.robotY(true) + RAISE / 1000, PAN_X = SO101.joints[0].origin[0], CX = PAN_X + CXMM / 1000, CZ = row.cz_mm / 1000;
const G = M.gripFor(C, BAND);
const JAW_OPEN = G.angleFor(C + 0.0061), JAW_HOLD = G.angleFor(C - CLR);   // 46.1 mm open (the band's largest), shut on 40 - c
const HX = (C - CLR) / 2, HY = C / 2;
const NEAR = 0.003;
const SHIFT = 0.003;            // the open fixed finger goes down / up this far from the cell's face
let curOff = 0;                 // the fixed finger's distance from the cell's face at the frame being measured
const FINGERS = { j5: 1, jaw: 1 };
const PHOTO_Q = arg("--photo-q", null) ? arg("--photo-q").split(",").map(Number) : null, SERVE = process.argv.includes("--serve");

// ---- the world (screen coordinates: X, Y up, Z; the table at Y = 0). Arm parts are in the arm frame = world with Y - ROBOT_Y
const toArm = p => [p[0], p[1] - ROBOT_Y, p[2]];
function cellsAt(st, centre, turnDeg, ySeatBottom) {          // a piece's cell boxes: gripped cell centre (X, Z), turned, bottom height
  const [i, j, k] = st.grip.cell, minz = Math.min(...st.cells.map(c => c[2]));
  const co = Math.cos(turnDeg * DEGR), si = Math.sin(turnDeg * DEGR);
  return st.cells.map(c => {
    const dx = (c[0] - i) * C, dz = -(c[1] - j) * C;            // the plan's y -> -Z (right-handed; 点検 C67 の 1)
    return { c: [centre[0] + dx * co - dz * si, ySeatBottom + (c[2] - minz + 0.5) * C, centre[1] + dx * si + dz * co],
             yaw: -turnDeg * DEGR, gripped: c[0] === i && c[1] === j && c[2] === k };
  });
}
const cubeCells = plan.steps.map(st => st.cells.map(c => ({ c: [CX + (c[0] - 1) * C, (c[2] + 0.5) * C, CZ - (c[1] - 1) * C], yaw: 0,
  gripped: c[0] === st.grip.cell[0] && c[1] === st.grip.cell[1] && c[2] === st.grip.cell[2] })));
const tableCells = plan.steps.map((st, n) => cellsAt(st, [row.steps[n].pick.cell_mm[0] / 1000, row.steps[n].pick.cell_mm[1] / 1000], row.steps[n].pick.table_turn_deg, 0));
const asBox = (b, shrinkY) => ({ type: "box", c: toArm(b.c), half: [HX, HY - (shrinkY || 0), HX], yaw: b.yaw });
// the stand: a box under the base, the base's footprint + 5 mm, from the table up to the base's bottom
const bb = (() => { const b = [1e9, 1e9, 1e9, -1e9, -1e9, -1e9]; SC.groupTris().base.forEach(t => t.forEach(p => { for (let a = 0; a < 3; a++) { b[a] = Math.min(b[a], p[a]); b[a + 3] = Math.max(b[a + 3], p[a]); } })); return b; })();
const STAND = RAISE > 0 ? { type: "box", c: [(bb[0] + bb[3]) / 2, (bb[1] - ROBOT_Y) / 2, (bb[2] + bb[5]) / 2], half: [(bb[3] - bb[0]) / 2 + 0.005, (ROBOT_Y + bb[1]) / 2, (bb[5] - bb[2]) / 2 + 0.005], yaw: 0 } : null;
const TABLE = { type: "planeY", y: -ROBOT_Y };

// ---- measuring
const coarse = SC.coarseSets(), fine = SC.fineSets();
function armVs(q, jaw, obs, groups) {                         // smallest gap of the arm's parts to the obstacles: {d, g, what}
  const stC = SC.placed(q, jaw, coarse), stF = {};
  let best = { d: Infinity };
  for (const g of groups) for (const o of obs) {
    let r = CK.partsObstacleClearance(stC[g], o.ob);
    if (r.clearance < 0.012) { stF.p = stF.p || SC.placed(q, jaw, fine); r = CK.partsObstacleClearance(stF.p[g], o.ob); }
    if (r.clearance < best.d) best = { d: r.clearance, g, what: o.what };
  }
  return best;
}
function samples(b) {                                         // points on a cell box's faces, every 4 mm (world)
  const out = [], n = Math.ceil(2 * HX / 0.004), co = Math.cos(-b.yaw), si = Math.sin(-b.yaw);
  for (let a = 0; a <= n; a++) for (let e = 0; e <= n; e++) {
    const u = -HX + 2 * HX * a / n, v = -HX + 2 * HX * e / n, w = -HY + 2 * HY * e / n;
    for (const [lx, ly, lz] of [[u, -HY, v], [u, HY, v], [-HX, w, u], [HX, w, u], [u, w, -HX], [u, w, HX]])
      out.push([b.c[0] + lx * co - lz * si, b.c[1] + ly, b.c[2] + lx * si + lz * co]);
  }
  return out;
}
function boxVsBoxes(held, others, what) {                     // held cells against other cell boxes (both ways sampled)
  let best = { d: Infinity };
  for (const h of held) for (const o of others) {
    if (Math.hypot(h.c[0] - o.c[0], h.c[1] - o.c[1], h.c[2] - o.c[2]) > 0.09) continue;
    const ob = { type: "box", c: o.c, half: [HX, HY, HX], yaw: o.yaw }, hb = { type: "box", c: h.c, half: [HX, HY, HX], yaw: h.yaw };
    for (const p of samples(h)) { const d = CK.pointToObstacle(p, ob); if (d < best.d) best = { d, what }; }
    for (const p of samples(o)) { const d = CK.pointToObstacle(p, hb); if (d < best.d) best = { d, what }; }
  }
  return best;
}
function heldVsBox(held, ob, what) {
  let best = { d: Infinity };
  for (const h of held) for (const p of samples(h)) { const d = CK.pointToObstacle(p, ob); if (d < best.d) best = { d, what }; }
  return best;
}

// ---- the path
let q = null;
function solve(tool, yaw) {
  const r = MOTION.topDown(toArm(tool), yaw, q, { exact: true });
  if (!r.ok) return null;
  q = r.q; return q;
}
const ALL = ["j1", "j2", "j3", "j4", "j5", "jaw", "cam"];
const FRAMES = [];
const out = { about: "tools/soma_collide.js", raise_mm: RAISE, cube_centre_from_pan_mm: CXMM, clear_mm: CLR * 1000, jaw_open_deg: +JAW_OPEN.toFixed(1),
              jaw_hold_deg: +JAW_HOLD.toFixed(1), stand: STAND ? { half_mm: STAND.half.map(v => +(v * 1000).toFixed(1)) } : null, steps: [] };
const REACH = HX;               // the tool point (the fixed finger's face) is HX = (C - c)/2 from the gripped cell's centre: the piece
                                // lies against the fixed finger when shut (点検 C67 の 4: was C/2, so it was put 0.5 mm off and the check was circular)
const carryAll = Math.max(...row.steps.map(s => s.carry_y_mm)) / 1000;
let last = null;                                              // {cellXZ, yaw} above the last place
function buildPiece(n, opt) {                                  // piece n's frames (pushed to FRAMES) and its record
  opt = opt || {};
  const place = opt.place || null, coff = opt.cubeOff || [];
  const shiftCells = (cells, d) => d ? cells.map(b => Object.assign({}, b, { c: [b.c[0] + d[0], b.c[1], b.c[2] + d[1]] })) : cells;
  const cubeNow = cubeCells.map((cs, k) => shiftCells(cs, k < n ? coff[k] : k === n ? place : null));
  const st = plan.steps[n], rs = row.steps[n];
  const pickXZ = [rs.pick.cell_mm[0] / 1000, rs.pick.cell_mm[1] / 1000], placeXZ = [rs.cell_mm[0] / 1000 + (place ? place[0] : 0), rs.cell_mm[1] / 1000 + (place ? place[1] : 0)];
  const yPick = rs.pick.tool_mm[1] / 1000, yPlace = rs.tool_mm[1] / 1000, yCarry = rs.carry_y_mm / 1000;
  const jyPick = rs.pick.jaw_yaw_deg, jyPlace = rs.place_yaw_deg;
  const inCube = [].concat(...cubeNow.slice(0, n)).map(b => ({ ob: asBox(b), what: "cube", b }));
  const inCubeAfter = inCube.concat(cubeNow[n].map(b => ({ ob: asBox(b), what: "cube (this piece, let go)", b })));
  const onTable = [].concat(...tableCells.slice(n + 1)).map(b => ({ ob: asBox(b), what: "table piece", b }));
  const onTableBefore = onTable.concat(tableCells[n].map(b => ({ ob: asBox(b), what: "table piece (this one, not yet held)", b })));
  const onTableHeld = [].concat(...tableCells.slice(n + 1));
  const statics = (STAND ? [{ ob: STAND, what: "stand" }] : []).concat([{ ob: TABLE, what: "table" }]);
  const rec = { piece: st.piece, phases: {} };
  const worst = (ph, key, r, extra) => {
    const P = rec.phases[ph] = rec.phases[ph] || {};
    if (!P[key] || r.d < P[key].d) P[key] = Object.assign({ d: r.d, g: r.g, what: r.what }, extra || {});
  };
  let fail = null, heldFirst = null, heldLast = null;
  function frame(ph, tool, yaw, jaw, held) {
    if (!solve(tool, yaw)) { fail = fail || `${ph}: no pose at tool ${tool.map(v => (v * 1000).toFixed(0))} yaw ${yaw.toFixed(0)}`; return; }
    check(ph, tool, jaw, held);
  }
  function jointLine(ph, qa, qb, jaw) {                       // a move in joint space (the photo pose): every 2 deg, checked as the rest
    const nS = Math.max(1, Math.ceil(Math.max(...qa.map((v, i) => Math.abs(qb[i] - v))) / 2));
    for (let s = 1; s <= nS; s++) { q = qa.map((v, i) => v + (qb[i] - v) * s / nS); curOff = SHIFT; check(ph, null, jaw, false); }
  }
  function check(ph, tool, jaw, held) {
    FRAMES.push({ q: q.slice(), jaw, t: FRAMES.length, n, ph, held: !!held });
    // this piece's gripped cell, when it is not held (before the pick, after letting go): the fingers come to it by design
    // (the fixed finger's face 0.5 mm + SHIFT from it). Kept apart as "(grip)"; the other parts are checked as usual
    const after = ph === "leave" || ph === "photo";
    const cubeL = after ? inCubeAfter : inCube, tb = held || after ? onTable : onTableBefore;
    const gr = cubeL.concat(tb).filter(o => o.b.gripped && o.what.includes("this"));
    worst(ph, "arm-cube", armVs(q, jaw, cubeL.filter(o => !gr.includes(o)), ALL));
    if (tb.length) worst(ph, "arm-table piece", armVs(q, jaw, tb.filter(o => !gr.includes(o)), ALL));
    if (gr.length) {
      worst(ph, "arm-this piece's gripped cell", armVs(q, jaw, gr, ["j1", "j2", "j3", "j4", "cam"]));
      worst(ph, curOff >= SHIFT - 1e-9 ? "fingers-gripped cell (shifted off)" : "fingers-gripped cell (grip)", armVs(q, jaw, gr, ["j5", "jaw"]));
    }
    worst(ph, "arm-stand/table", armVs(q, jaw, statics, ALL));
    if (held) {
      const jy = MOTION.jawYawDeg(q), turn = jy - jyPlace;    // the piece turned with the jaw (0 at the place; mod 360 is fine)
      const u = [Math.cos(jy * DEGR), Math.sin(jy * DEGR)];
      const gc = [tool[0] + REACH * u[0], tool[2] + REACH * u[1]];
      const k = st.grip.cell[2], minz = Math.min(...st.cells.map(c => c[2]));
      const cellBottom = tool[1] - GY + BAND - C;           // the gripped cell's bottom
      const hc = cellsAt(st, gc, turn, cellBottom - (k - minz) * C);
      if (!heldFirst) heldFirst = hc; heldLast = hc;
      const seat = ph === "place" || ph === "pick";            // it sits on the table (pick) / meets its seat in the cube (place)
      const rc = boxVsBoxes(hc, [].concat(...cubeNow.slice(0, n)), "cube");
      worst(ph, ph === "place" ? "held-cube (seat)" : "held-cube", rc);
      if (onTableHeld.length) worst(ph, "held-table piece", boxVsBoxes(hc, onTableHeld, "table piece"));
      if (STAND) worst(ph, "held-stand", heldVsBox(hc, Object.assign({}, STAND, { c: [STAND.c[0], STAND.c[1] + ROBOT_Y, STAND.c[2]] }), "stand"));
      worst(ph, seat ? "held-table (seat)" : "held-table", heldVsBox(hc, { type: "planeY", y: 0 }, "table"));
      const hb = hc.map(b => ({ ob: asBox(b), what: "held piece", b }));
      worst(ph, "held-arm", armVs(q, jaw, hb, ["j1", "j2", "j3", "j4", "cam"]));
      worst(ph, "held(other cells)-fingers", armVs(q, jaw, hb.filter(h => !h.b.gripped), ["j5", "jaw"]));
    }
  }
  // a move of the tool: a, b = [X, height, Z] of the gripped cell's centre; the tool sits C/2 + off from it against the way the
  // jaw opens (off = SHIFT: the open fixed finger 3.5 mm from the cell's face instead of 0.5 mm). polar: across around the pan
  // axis (the radius and the angle go evenly; keeps the piece off the stand) instead of straight. The jaw turns ya -> yb the way
  // that the arm can follow (yb, yb + 360, yb - 360 tried, the shorter turn first; only the frames of the way taken are measured)
  function poses(a, b, ya, yb, offA, offB, polar) {
    const ra = Math.hypot(a[0] - PAN_X, a[2]), rb = Math.hypot(b[0] - PAN_X, b[2]);
    const ta = Math.atan2(a[2], a[0] - PAN_X); let tb = Math.atan2(b[2], b[0] - PAN_X);
    if (tb - ta > Math.PI) tb -= 2 * Math.PI; if (ta - tb > Math.PI) tb += 2 * Math.PI;
    const L = polar ? Math.hypot(rb - ra, (ra + rb) / 2 * (tb - ta), b[1] - a[1]) : Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]);
    const nT = Math.max(1, Math.ceil(L / 0.005), Math.ceil(Math.abs(yb - ya) / 5), Math.ceil(Math.abs(offB - offA) / 0.001));
    const P = [];
    for (let s = 0; s <= nT; s++) {
      const t = s / nT, yw = ya + (yb - ya) * t, off = offA + (offB - offA) * t, y = a[1] + (b[1] - a[1]) * t;
      let x, z;
      if (polar) { const r = ra + (rb - ra) * t, th = ta + (tb - ta) * t; x = PAN_X + r * Math.cos(th); z = r * Math.sin(th); }
      else { x = a[0] + (b[0] - a[0]) * t; z = a[2] + (b[2] - a[2]) * t; }
      P.push([[x - (REACH + off) * Math.cos(yw * DEGR), y, z - (REACH + off) * Math.sin(yw * DEGR)], yw, off]);
    }
    return P;
  }
  function line(ph, a, b, ya, yb, jaw, held, o) {
    o = o || {};
    if (fail) return yb;
    const offA = o.offA || 0, offB = o.offB === undefined ? offA : o.offB;
    const cands = ya === yb ? [yb] : [yb, yb + 360, yb - 360].sort((u, v) => Math.abs(u - ya) - Math.abs(v - ya));
    for (const y2 of cands) {                                 // dry run: can the arm follow this way round?
      const q0 = q; let ok = true;
      for (const [tool, yw] of poses(a, b, ya, y2, offA, offB, o.polar)) if (!solve(tool, yw)) { ok = false; break; }
      q = q0;
      if (!ok) continue;
      for (const [tool, yw, off] of poses(a, b, ya, y2, offA, offB, o.polar)) { curOff = off; frame(ph, tool, yw, jaw, held); if (fail) break; }
      return y2;
    }
    fail = `${ph}: no way for the arm (jaw ${ya.toFixed(0)} -> ${yb.toFixed(0)} deg, either way round)`;
    return yb;
  }
  // to the pick (jaw open): from above the last place, across (around the pan axis) at the highest carry height, then down beside
  // the cell with the fixed finger SHIFT further off, across to it, close; up; across to above the place; down; open; off; up
  const from = last || { cellXZ: pickXZ, yaw: jyPick };
  const H = (xz, y) => [xz[0], y, xz[1]];
  let y1 = line("to pick", H(from.cellXZ, carryAll), H(pickXZ, carryAll), from.yaw, jyPick, JAW_OPEN, false, { polar: true, offA: SHIFT });
  line("pick", H(pickXZ, carryAll), H(pickXZ, yPick), y1, y1, JAW_OPEN, false, { offA: SHIFT });
  line("pick", H(pickXZ, yPick), H(pickXZ, yPick), y1, y1, JAW_OPEN, false, { offA: SHIFT, offB: 0 });
  line("pick", H(pickXZ, yPick), H(pickXZ, yPick), y1, y1, JAW_HOLD, true);
  line("pick", H(pickXZ, yPick), H(pickXZ, carryAll), y1, y1, JAW_HOLD, true);
  const y2 = line("carry", H(pickXZ, carryAll), H(placeXZ, carryAll), y1, y1 + (jyPlace - jyPick), JAW_HOLD, true, { polar: true });
  line("place", H(placeXZ, carryAll), H(placeXZ, yPlace), y2, y2, JAW_HOLD, true);
  line("leave", H(placeXZ, yPlace), H(placeXZ, yPlace), y2, y2, JAW_OPEN, false, { offA: 0, offB: SHIFT });
  line("leave", H(placeXZ, yPlace), H(placeXZ, carryAll), y2, y2, JAW_OPEN, false, { offA: SHIFT });
  last = { cellXZ: placeXZ, yaw: y2 };
  if (PHOTO_Q && !fail) {                                     // to the photo pose and back (the arm out of the top camera's view)
    const q0 = q.slice();
    jointLine("photo", q0, PHOTO_Q, JAW_OPEN);
    FRAMES[FRAMES.length - 1].shoot = true;
    jointLine("photo", PHOTO_Q, q0, JAW_OPEN);
    q = q0;
  }
  // where the model put the held piece: at the pick it must be the piece on the table, at the end of the place the piece in the
  // cube (20 mm higher for a drop). The largest gap of a cell centre (mm) - a check of this script's turns and offsets
  const dev = (A, B, dy) => A && Math.max(...A.map((h, i) => Math.hypot(h.c[0] - B[i].c[0], h.c[1] - B[i].c[1] - dy, h.c[2] - B[i].c[2]))) * 1000;
  rec.held_at_pick_off_mm = heldFirst && +dev(heldFirst, tableCells[n], 0).toFixed(2);
  rec.held_at_place_off_mm = heldLast && +dev(heldLast, cubeNow[n], st.drop ? 0.02 : 0).toFixed(2);
  rec.fail = fail;
  // round, and the verdict per piece: NG if any gap < 0 (except the held piece meeting its seat), near if < NEAR
  let verdict = fail ? "no pose" : "ok";
  for (const ph of Object.keys(rec.phases)) for (const key of Object.keys(rec.phases[ph])) {
    const v = rec.phases[ph][key]; v.mm = +(v.d * 1000).toFixed(1); delete v.d;
    if (key.includes("(grip)")) continue;                     // the fingers on the cell they hold: by design, not judged
    if (key.includes("(shifted off)")) { if (v.mm < 0) verdict = verdict === "no pose" ? verdict : "NG"; continue; }   // 3 mm off by design; the boxes are fat
    if (key.includes("(seat)")) { if (v.mm < -0.1) verdict = verdict === "no pose" ? verdict : "NG"; continue; }   // into its seat
    if (v.mm < 0) verdict = verdict === "no pose" ? verdict : "NG"; else if (v.mm < NEAR * 1000 && verdict === "ok") verdict = "near";
  }
  rec.verdict = verdict;
  out.steps.push(rec);
  const brief = Object.entries(rec.phases).map(([ph, P]) => ph + ": " + Object.entries(P).map(([k, v]) => `${k} ${v.mm}${v.g ? "(" + v.g + ")" : ""}`).join(", ")).join(" | ");
  console.error(`${n + 1} ${st.piece}: ${verdict}${fail ? " — " + fail : ""}\n   ${brief}`);
  return rec;
}
const r6 = v => +v.toFixed(6);
const frameOut = f => Object.assign({ q: f.q.map(r6), jaw: r6(f.jaw), n: f.n, ph: f.ph, held: f.held }, f.shoot ? { shoot: true } : {});
if (SERVE) {
  // one piece per request (tools/puzzle_run.py --soma-correct): the same path and checks as the batch, from the given start
  const rl = require("readline").createInterface({ input: process.stdin });
  rl.on("line", ln => {
    let ans;
    try {
      const r = JSON.parse(ln);
      q = r.start_q ? r.start_q.slice() : null; last = r.last ? { cellXZ: r.last.cell_mm.map(v => v / 1000), yaw: r.last.yaw } : null;
      FRAMES.length = 0; out.steps.length = 0;
      const mm = v => v ? [v[0] / 1000, v[1] / 1000] : null;
      const rec = buildPiece(r.n, { place: mm(r.place_mm), cubeOff: (r.cube_off_mm || []).map(mm) });
      const mc = SC.motionCheck((r.start_q ? [{ q: r.start_q.slice(), jaw: r.start_jaw, t: 0 }] : []).concat(FRAMES.map((f, i) => ({ q: f.q, jaw: f.jaw, t: i + 1 }))));
      let selfMin = Infinity; FRAMES.forEach(f => { selfMin = Math.min(selfMin, SC.pose(f.q, f.jaw).clearance); });
      let verdict = rec.verdict;
      if (selfMin < 0 || mc.min < 0 || (isFinite(mc.worstBound) && mc.worstBound < 0)) verdict = verdict === "no pose" ? verdict : "NG (self)";
      ans = { ok: true, verdict, fail: rec.fail, self_min_mm: +(Math.min(selfMin, mc.min) * 1000).toFixed(1), held_at_place_off_mm: rec.held_at_place_off_mm,
              phases: rec.phases, last: last && { cell_mm: last.cellXZ.map(v => +(v * 1000).toFixed(3)), yaw: last.yaw }, frames: FRAMES.map(frameOut) };
    } catch (e) { ans = { ok: false, why: String(e && e.stack || e) }; }
    process.stdout.write(JSON.stringify(ans) + "\n");
  });
} else {
for (let n = 0; n < plan.steps.length; n++) buildPiece(n, {});
// the arm against itself along the whole path (sim/self-collision.js motionCheck: between the frames too)
const mc = SC.motionCheck(FRAMES);
out.self = { min_mm: +(mc.min * 1000).toFixed(1), pair: mc.pair, at: FRAMES[mc.t] ? { piece: plan.steps[FRAMES[mc.t].n].piece, phase: FRAMES[mc.t].ph } : null,
             frames: FRAMES.length, worst_bound_mm: isFinite(mc.worstBound) ? +(mc.worstBound * 1000).toFixed(1) : null };
console.error(`self: ${JSON.stringify(out.self)}`);
// per piece and phase: the arm's closest pair to itself at the frames (sim/self-collision.js pose: down to the fine boxes)
FRAMES.forEach(f => {
  const r = SC.pose(f.q, f.jaw), rec = out.steps[f.n], P = rec.self = rec.self || {};
  if (!P[f.ph] || r.clearance < P[f.ph].d) P[f.ph] = { d: r.clearance, pair: r.pair };
});
out.steps.forEach(rec => { for (const ph in rec.self) { const v = rec.self[ph]; v.mm = +(v.d * 1000).toFixed(1); delete v.d; } });
console.error("self per piece (smallest, mm): " + out.steps.map((r, i) => `${i + 1}:` + Object.entries(r.self || {}).map(([ph, v]) => `${ph} ${v.mm}(${v.pair})`).join(",")).join(" | "));
out.steps.forEach(rec => { for (const ph in rec.self || {}) if (rec.self[ph].mm < 0 && rec.verdict !== "no pose") rec.verdict = "NG (self)"; });
if (mc.min < 0 || (isFinite(mc.worstBound) && mc.worstBound < 0)) out.self.verdict = "NG";
console.error("held piece off its table spot at the pick / its cube spot at the place (mm): " + out.steps.map(s => `${s.held_at_pick_off_mm}/${s.held_at_place_off_mm}`).join(" "));
// --path-json OUT (2026-11-24, for tools/puzzle_run.py --soma): the path just measured, as it was measured (URDF degrees, jaw
// degrees, per frame), and the world it was measured in (screen metres: X, Y up, Z; the table at Y = 0)
if (arg("--path-json")) {
  const pieces = plan.steps.map((st, n) => ({ name: st.piece, drop: !!st.drop, grip_index: st.cells.findIndex(c => c[0] === st.grip.cell[0] && c[1] === st.grip.cell[1] && c[2] === st.grip.cell[2]),
    table_cells: tableCells[n].map(b => b.c.map(r6)), table_turn_deg: row.steps[n].pick.table_turn_deg, cube_cells: cubeCells[n].map(b => b.c.map(r6)) }));
  const P = { about: "tools/soma_collide.js --path-json: the collision-checked path of the Soma build (tools/puzzle_run.py --soma)",
    verdicts: out.steps.map(s => s.verdict), self_min_mm: out.self.min_mm, self_verdict: out.self.verdict || "ok",
    raise_mm: RAISE, cube_centre_from_pan_mm: CXMM, robot_y_m: r6(ROBOT_Y), cell_m: C, across_m: r6(C - CLR), clear_mm: CLR * 1000,
    jaw_open_deg: r6(JAW_OPEN), jaw_hold_deg: r6(JAW_HOLD), stand: STAND ? { c: STAND.c.map((v, i) => r6(i === 1 ? v + ROBOT_Y : v)), half: STAND.half.map(r6) } : null,
    reach_file: process.argv[3], plan_file: process.argv[2], photo_q: PHOTO_Q, pieces,
    frames: FRAMES.map(frameOut) };
  fs.writeFileSync(arg("--path-json"), JSON.stringify(P));
  console.error(`path: ${FRAMES.length} frames -> ${arg("--path-json")}`);
}
if (arg("--json")) fs.writeFileSync(arg("--json"), JSON.stringify(out, null, 1));
console.error(`raise ${RAISE} mm, cube ${CXMM} mm: ${out.steps.map(s => s.verdict).join(" ")}${out.self.verdict ? " / self NG between frames" : ""}`);
}
