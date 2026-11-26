// Straight-down reach band of the SO-101 (sim/ model), 2026-11-23: for each distance r in front of the pan axis (and to the side),
// the lowest and highest tool height (relative to the arm base origin, mm) that the IK reaches with the tool straight down.
// Model only (no collisions with the table, the stand or the cube). usage: node tools/soma_reachband.js
const T = "C:/Users/genes/Documents/PROJECTS/lerobot/sim/tools/";
const { load } = require(T + "_load");
load("kinematics.js", "so101.js", "motion.js", "puzzle.js", "models/so101_visuals.js", "models/so101_points.js", "models/so101_tris.js",
     "collide.js", "chikuwa-collision.js", "self-collision.js");
global.window = { SCENE_SKIP_DEFAULT: 1 }; load("scene-data.js"); delete global.window;
const PAN_X = SO101.joints[0].origin[0];
function ok(x, y, z) {
  const r = KIN.ikRobust(SO101.axes, SO101.tool, [x, y, z], [0, -1, 0],
    { limitsDeg: SO101.limitsDeg, lockJoints: [4], posTol: 0.0002, dirTolDeg: 0.3, iters: 200 });
  return r.posErr < 0.0005 && r.dirErrDeg < 1.0;
}
console.log("r (mm, in front of the pan axis) : reachable tool heights from the arm base origin (mm, straight down, step 5)");
for (const r of [80, 100, 120, 140, 160, 180, 200, 220, 240, 260]) {
  const hs = [];
  for (let y = -250; y <= 200; y += 5) if (ok(PAN_X + r / 1000, y / 1000, 0)) hs.push(y);
  // print as ranges
  const rg = []; let a = null, b = null;
  for (const h of hs) { if (a === null) { a = b = h; } else if (h === b + 5) b = h; else { rg.push([a, b]); a = b = h; } }
  if (a !== null) rg.push([a, b]);
  console.log(`${r}: ${rg.map(([p, q]) => p + ".." + q).join(", ") || "none"}`);
}
console.log("robot base origin above the table now (mm):", (SCENE_MODEL.robotY(true) * 1000).toFixed(1));
