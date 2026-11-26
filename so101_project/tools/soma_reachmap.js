// Reach map of the SO-101 (sim/ model) for the Soma cube (2026-11-23): at distance r in front of the pan axis and tool height y
// above the table, the smallest tilt of the tool from straight down (tilted in the vertical plane through the pan axis, the tip
// leaning away from the base or toward it) that the IK reaches (position 0.5 mm, direction 1 deg). Model only, no collisions.
// usage: node tools/soma_reachmap.js > outputs/soma/reachmap.txt
const T = "C:/Users/genes/Documents/PROJECTS/lerobot/sim/tools/";
const { load } = require(T + "_load");
load("kinematics.js", "so101.js", "motion.js", "puzzle.js", "models/so101_visuals.js", "models/so101_points.js", "models/so101_tris.js",
     "collide.js", "chikuwa-collision.js", "self-collision.js");
global.window = { SCENE_SKIP_DEFAULT: 1 }; load("scene-data.js"); delete global.window;
const M = SCENE_MODEL;
const ROBOT_Y = M.robotY(true), PAN_X = SO101.joints[0].origin[0];
const D = Math.PI / 180;
function reach(r, y, tilt) {
  const dir = [Math.sin(tilt * D), -Math.cos(tilt * D), 0];     // + = tip leaning away from the base (forward)
  const res = KIN.ikRobust(SO101.axes, SO101.tool, [PAN_X + r, y - ROBOT_Y, 0], dir,
    { limitsDeg: SO101.limitsDeg, lockJoints: [4], posTol: 0.0002, dirTolDeg: 0.3, iters: 200 });
  return res.posErr < 0.0005 && res.dirErrDeg < 1.0;
}
const TILTS = [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, -10, -20, -30, -40];
console.log("rows: tool height above the table (mm); columns: distance in front of the pan axis (mm); cell: smallest |tilt| reached (deg, sign = forward/back), . = none up to 90");
const RS = [100, 120, 140, 160, 180, 200, 220, 240];
console.log("y\r  " + RS.map(r => String(r).padStart(5)).join(""));
for (let y = 300; y >= 20; y -= 20) {
  let line = String(y).padStart(4) + " ";
  for (const r of RS) {
    let got = ".";
    for (const t of [...TILTS].sort((a, b) => Math.abs(a) - Math.abs(b))) { if (reach(r / 1000, y / 1000, t)) { got = String(t); break; } }
    line += got.padStart(5);
  }
  console.log(line);
}
