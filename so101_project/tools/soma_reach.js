// Can the SO-101 (sim/ model) reach every pose of the Soma build plan (tools/soma_plan.py)? (2026-11-23)
// Straight-down tool (sim/motion.js topDown), the jaw closing along the plan's axis, the tool point where the 20 mm grip band
// holds the top of the gripped cell (the tool point is on the fixed finger's face: C/2 off the cell centre). Checked along the straight vertical line from the place (or the release height, for a
// drop) up to the carry height (the piece's bottom 10 mm over the cube top), every 5 mm. The cube centre is scanned along the
// arm's forward axis. The pick side: each piece gets a spot on the table (a 20 mm grid, the piece turned by 0/90/180/270 deg)
// where its vertical line is reachable; the pieces 20 mm apart and 20 mm off the cube, the two fingers (12 mm squares) 5 mm clear
// of the cube and the other pieces, nothing over the base (a search over all 7 together).
// NOT checked here (tools/soma_collide.js does, along the whole path): the arm, the fingers or the held piece hitting the cube,
// the stand or each other, beyond the optional --self-clear-mm / --arm-clear-mm on the pick line;
// joint speed; the camera. Model only.
// usage: node tools/soma_reach.js outputs/soma/plan.json [--cx-mm 150,170,190] [--cz-mm 0] [--raise-mm 0,150,200] [--base-clear-mm 90]
//        [--self-clear-mm 0] [--arm-clear-mm 0]   (> 0: and the arm this far from the whole cube and the stand there)
//        [--self-clear-mm 0]   (> 0: pick spots only where the arm, jaw open, stays this far from itself on the pick line)
const T = "C:/Users/genes/Documents/PROJECTS/lerobot/sim/tools/";
const fs = require("fs");
const { load } = require(T + "_load");
load("kinematics.js", "so101.js", "motion.js", "puzzle.js", "models/so101_visuals.js", "models/so101_points.js", "models/so101_tris.js",
     "collide.js", "chikuwa-collision.js", "self-collision.js");
global.window = { SCENE_SKIP_DEFAULT: 1 }; load("scene-data.js"); delete global.window;
const M = SCENE_MODEL;
const plan = JSON.parse(fs.readFileSync(process.argv[2], "utf-8"));
const arg = (k, d) => { const i = process.argv.indexOf(k); return i > 0 ? process.argv[i + 1] : d; };
const SELF = +arg("--self-clear-mm", "0") / 1000, ARM = +arg("--arm-clear-mm", "0") / 1000;
// the stand under the base (as tools/soma_collide.js): the base's footprint + 5 mm, from the table up to the base (arm frame)
const BB = (() => { const b = [1e9, 1e9, 1e9, -1e9, -1e9, -1e9]; SELF_COLLIDE.groupTris().base.forEach(t => t.forEach(p => { for (let a = 0; a < 3; a++) { b[a] = Math.min(b[a], p[a]); b[a + 3] = Math.max(b[a + 3], p[a]); } })); return b; })();
const standOf = RY => RY - ROBOT_Y0 > 1e-6 ? { type: "box", c: [(BB[0] + BB[3]) / 2, (BB[1] - RY) / 2, (BB[2] + BB[5]) / 2], half: [(BB[3] - BB[0]) / 2 + 0.005, (RY + BB[1]) / 2, (BB[5] - BB[2]) / 2 + 0.005], yaw: 0 } : null;
const CXS = arg("--cx-mm", "120,140,160,180,200").split(",").map(Number);
const CZ = +arg("--cz-mm", "0") / 1000;
const RAISES = arg("--raise-mm", "0").split(",").map(Number);   // the arm base set this much higher above the table (a box under it)
const C = plan.cell_mm / 1000, BAND = plan.grip_band_mm / 1000;
const JAW_OPEN = M.gripFor(plan.cell_mm / 1000, plan.grip_band_mm / 1000).angleFor(plan.cell_mm / 1000 + 0.0061);   // 46.1 mm (soma_collide)
const ROBOT_Y0 = M.robotY(true), PAN_X = SO101.joints[0].origin[0], GY = M.GRASP_Y;
const GAP = +arg("--gap-mm", "20") / 1000;           // free space between two pieces on the table, and from the cube (the fingers)
const FW = 0.012;            // a finger's size on the table (sim/README: about 10 mm thick at the band), as a 12 mm square
const FGAP = 0.005;          // free space around a finger
const BASE_CLEAR = +arg("--base-clear-mm", "90") / 1000;  // no piece cell or finger centre this near the pan axis (the arm's base, or the stand under it)
const DEGR = Math.PI / 180;
const DROP_UP = 0.020;     // a drop: the fingertips (3 mm above the band bottom) go 17 mm under the top; let go 20 mm higher
const out = { about: "tools/soma_reach.js", robot_y_mm: +(ROBOT_Y0 * 1000).toFixed(1), pan_x_mm: +(PAN_X * 1000).toFixed(1), grasp_y_mm: +(GY * 1000).toFixed(1), rows: [] };
for (const raise of RAISES) {
const ROBOT_Y = ROBOT_Y0 + raise / 1000;
for (const cxmm of CXS) {
  const cx = PAN_X + cxmm / 1000;
  let allOk = true, worst = null, q = null;
  const steps = [];
  for (const st of plan.steps) {
    const [i, j, k] = st.grip.cell;
    const X = cx + (i - 1) * C, Z = CZ - (j - 1) * C;          // the plan (x, y, z up) -> screen (X, Y up, Z) = (x, z, -y): right-handed (点検 C67 の 1: was +y, a mirror image)
    const top = (k + 1) * C, minz = Math.min(...st.cells.map(c => c[2]));
    const yPlace = GY + top - BAND + (st.drop ? DROP_UP : 0);
    const yCarry = GY + top - BAND + (3 * C + 0.010 - minz * C);
    // the tool point (sim/scene-data.js gripFor) is on the FIXED finger's inner face, not between the fingers (face = 0): the
    // gripped cell's centre is C/2 from it along the way the jaw opens (yaw). So the tool goes to the cell centre - C/2 u(yaw).
    // Both ways of the axis (the fixed finger on either side) are tried, exactly (点検 C65 の後: the tool was put at the centre)
    const yaw0 = st.grip.axis === "x" ? 0 : 90;
    let ok = false, why = "", n = 0, yaw = yaw0, q0 = q;
    for (const yw of [yaw0, yaw0 + 180]) {
      const ux = Math.cos(yw * DEGR), uz = Math.sin(yw * DEGR);
      let good = true, qq = q0, nn = 0, w = "";
      for (let y = yPlace; y <= yCarry + 1e-9; y += 0.005) {
        const r = MOTION.topDown([X - C / 2 * ux, y - ROBOT_Y, Z - C / 2 * uz], yw, qq, { exact: true });
        nn++;
        if (!r.ok) { good = false; w = `at ${(y * 1000).toFixed(0)} mm: ${r.reason}`; break; }
        qq = r.q;
      }
      if (good) { ok = true; yaw = yw; q = qq; n = nn; why = ""; break; }
      if (!why) why = w;
    }
    // the pick: the piece lies on the table in its cube orientation, turned by a multiple of 90 deg about the vertical (the
    // person sets it out so); the jaw comes down turned the same (exact: not 180 deg off, or the piece arrives turned) and the
    // arm turns it back on the way. Candidate spots (the gripped cell's centre) on a 20 mm grid around the pan axis whose
    // whole vertical line (pick height .. carry height) is reachable. The spots are given out after this loop (below).
    const cands = [];
    if (ok) {
      const jy = yaw;                                         // exact: the jaw points this way at the place
      const hTable = (k - minz + 1) * C;                        // the gripped cell's top when the piece lies on the table
      const yPick = GY + hTable - BAND, yUp = yCarry;
      const minx = Math.min(...st.cells.map(c => c[0])), miny = Math.min(...st.cells.map(c => c[1]));
      const foot = [...new Set(st.cells.map(c => (c[0] - i) + "," + -(c[1] - j)))].map(t => t.split(",").map(Number));   // (dX, dZ)
      for (let gx = -0.26; gx <= 0.30 + 1e-9; gx += 0.02) for (let gz = -0.30; gz <= 0.30 + 1e-9; gz += 0.02) {
        const rr = Math.hypot(gx, gz), ang = Math.atan2(gz, gx) / DEGR;
        if (rr < 0.11 || rr > 0.26 || Math.abs(ang) > 125) continue;
        const sx = PAN_X + gx, sz = gz;
        for (const off of [0, 90, -90, 180]) {
          const co = Math.cos(off * DEGR), si = Math.sin(off * DEGR);
          // squares (centres) of the piece's cells on the table, turned by off (X toward Z positive, as jawYawDeg)
          const sq = foot.map(([dx, dz]) => [sx + (dx * co - dz * si) * C, sz + (dx * si + dz * co) * C]);
          // the two fingers beside the gripped cell, along the plan's axis turned by off (点検 C65 の 1: they were not checked)
          const ax0 = st.grip.axis === "x" ? [1, 0] : [0, 1], ax = [ax0[0] * co - ax0[1] * si, ax0[0] * si + ax0[1] * co];
          const fg = [-1, 1].map(t => [sx + t * ax[0] * (C / 2 + FW / 2), sz + t * ax[1] * (C / 2 + FW / 2)]);
          // cells: their faces at least GAP from the cube's faces (centre 2C + GAP from the cube centre; 点検 C65 の 1: was 1.5C + GAP,
          // which let a cell touch the cube); fingers: FGAP from the cube; neither over the arm's base
          const inCube = (x, z, m) => Math.abs(x - cx) < m && Math.abs(z - CZ) < m;
          if (sq.some(([x, z]) => Math.hypot(x - PAN_X, z) < BASE_CLEAR || inCube(x, z, 2 * C + GAP))) continue;
          if (fg.some(([x, z]) => Math.hypot(x - PAN_X, z) < BASE_CLEAR || inCube(x, z, 1.5 * C + FW / 2 + FGAP))) continue;
          let qq = null, good = true;
          for (let y = yPick; y <= yUp + 1e-9; y += 0.01) {
            const r = MOTION.topDown([sx - C / 2 * Math.cos((jy + off) * DEGR), y - ROBOT_Y, sz - C / 2 * Math.sin((jy + off) * DEGR)], jy + off, qq, { exact: true });
            if (!r.ok) { good = false; break; }
            qq = r.q;
          }
          // --self-clear-mm (optional, tools/soma_collide.js 2026-11-24): the arm clear of itself (sim/self-collision.js pose, down
          // to the fine boxes) at the pick height and every 30 mm up; near the base the camera met the shoulder (j1|cam)
          // --arm-clear-mm (optional, the same day): the arm's parts (coarse boxes, then the fine ones when near) clear of the whole
          // cube (one box: all 27 cells, as if built; the piece itself lies outside it) and of the stand, on the same line
          if (good && (SELF > 0 || ARM > 0)) {
            let q2 = null;
            const cubeBox = { type: "box", c: [cx, 1.5 * C - ROBOT_Y, CZ], half: [1.5 * C, 1.5 * C, 1.5 * C], yaw: 0 };
            for (let y = yPick; y <= yUp + 1e-9 && good; y += 0.03) {
              const r = MOTION.topDown([sx - C / 2 * Math.cos((jy + off) * DEGR), y - ROBOT_Y, sz - C / 2 * Math.sin((jy + off) * DEGR)], jy + off, q2, { exact: true });
              q2 = r.q;
              if (!r.ok || (SELF > 0 && SELF_COLLIDE.pose(r.q, JAW_OPEN).clearance < SELF)) { good = false; break; }
              if (ARM > 0) {
                const W = SELF_COLLIDE.placed(r.q, JAW_OPEN, SELF_COLLIDE.coarseSets());
                let Wf = null;
                for (const g of ["j1", "j2", "j3", "j4", "j5", "jaw", "cam"]) for (const ob of standOf(ROBOT_Y) ? [cubeBox, standOf(ROBOT_Y)] : [cubeBox]) {
                  let d = CK_COLLIDE.partsObstacleClearance(W[g], ob).clearance;
                  if (d < ARM + 0.012) { Wf = Wf || SELF_COLLIDE.placed(r.q, JAW_OPEN, SELF_COLLIDE.fineSets()); d = CK_COLLIDE.partsObstacleClearance(Wf[g], ob).clearance; }
                  if (d < ARM) good = false;
                }
              }
            }
          }
          if (good) cands.push({ r_mm: +(rr * 1000).toFixed(0), angle_deg: +ang.toFixed(0), table_turn_deg: off, jaw_yaw_deg: jy + off, cell_mm: [+(sx * 1000).toFixed(1), +(sz * 1000).toFixed(1)], tool_mm: [+((sx - C / 2 * Math.cos((jy + off) * DEGR)) * 1000).toFixed(1), +(yPick * 1000).toFixed(1), +((sz - C / 2 * Math.sin((jy + off) * DEGR)) * 1000).toFixed(1)], sq, fg });
        }
      }
      if (!cands.length) { ok = false; why = "no reachable pick spot on the table"; }
    }
    steps.push({ piece: st.piece, ok, why, place_yaw_deg: yaw, cell_mm: [+(X * 1000).toFixed(1), +(Z * 1000).toFixed(1)], tool_mm: [+((X - C / 2 * Math.cos(yaw * DEGR)) * 1000).toFixed(1), +(yPlace * 1000).toFixed(1), +((Z - C / 2 * Math.sin(yaw * DEGR)) * 1000).toFixed(1)], carry_y_mm: +(yCarry * 1000).toFixed(1), n, n_spots: cands.length, cands });
    if (!ok) { allOk = false; worst = worst || st.piece; }
  }
  // give out the pick spots: every piece one spot, the squares of two pieces at least GAP apart (finger room), by a search
  // that tries the spots nearest the pan axis first
  if (allOk) {
    const apart = (A, B, m) => A.every(([x1, z1]) => B.every(([x2, z2]) => Math.abs(x1 - x2) >= m || Math.abs(z1 - z2) >= m));
    const far = (a, b) => apart(a.sq, b.sq, C + GAP) && apart(a.fg, b.sq, C / 2 + FW / 2 + FGAP) && apart(b.fg, a.sq, C / 2 + FW / 2 + FGAP);
    for (const s of steps) s.cands.sort((u, v) => u.r_mm - v.r_mm);
    const chosen = [];
    let tries = 0;
    const dfs = (n) => {
      if (n === steps.length) return true;
      if (++tries > 200000) return false;
      for (const c of steps[n].cands) {
        if (chosen.every(o => far(o, c))) { chosen.push(c); if (dfs(n + 1)) return true; chosen.pop(); }
      }
      return false;
    };
    if (dfs(0)) steps.forEach((s, n) => { const { sq, fg, ...p } = chosen[n]; s.pick = p; });
    else { allOk = false; steps.forEach(s => { if (!s.pick) { s.ok = false; s.why = "the pick spots do not all fit on the table together"; } }); }
  }
  for (const s of steps) delete s.cands;
  out.rows.push({ raise_mm: raise, cube_centre_from_pan_mm: cxmm, cz_mm: CZ * 1000, all_ok: allOk, steps });
  console.error(`raise ${raise} mm, cube centre ${cxmm} mm in front of the pan axis: ${allOk ? "all reachable" : "NOT all: " + steps.filter(s => !s.ok).map(s => s.piece + " " + s.why).join(" / ")}`);
}
}
console.log(JSON.stringify(out, null, 1));
