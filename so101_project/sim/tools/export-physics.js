// 物理エンジン(MuJoCo)で流すための場面を書き出す(physics/run_physics.py が読む)。
//   書くもの: out/physics/<名前>/scene.json
//     ・コマごとの関節角(URDF の角度[度]。MuJoCo の so101_new_calib.xml と同じ角度。2026-11-17 に関節角 3 組で、爪の座標系が 0.002mm 以内で一致するのを確かめた)
//     ・爪の角度、持っているピースの番号(-1 = 持っていない)
//     ・ピースの形(局所のマス)・色・始めと終わりの置き方(画面の座標。Y が上・机 = 0)
//     ・腕の原点の高さ(2.4mm。2026-11-18 までは台ありで 9.6mm)、上のカメラの台の部品の位置(置き方 C の箱 lo・hi。2026-11-19 から)
//   座標の読み替え(physics/run_physics.py が行う): MuJoCo = (x, −z, y)(Z が上。URDF と同じ向き)
//   実行: node tools/export-physics.js [--set 0..6] [--gap mm] [--sol n] [--stand 0|1] [--name 名前]
const fs = require("fs"), path = require("path");
const { load, ROOT } = require("./_load");
load("kinematics.js", "so101.js", "motion.js", "puzzle.js",
     "models/so101_visuals.js", "models/so101_points.js", "models/so101_tris.js", "collide.js",
     "chikuwa-collision.js", "self-collision.js", "scene-data.js");
const M = SCENE_MODEL;
const arg = k => { const i = process.argv.indexOf("--" + k); return i > 0 ? process.argv[i + 1] : undefined; };
const def = M.defaultUiValues();
const ui = { SET: +(arg("set") ?? 0), GAP: +(arg("gap") ?? def.GAP), SOL: +(arg("sol") ?? 0), STAND: +(arg("stand") ?? 1) };
const opt = M.controls.find(c => c.key === "SET").options.find(o => o.value === ui.SET);
if (!opt) { console.log(`種 ${ui.SET} は無い`); process.exit(1); }
const name = arg("name") || `set${ui.SET}_gap${ui.GAP}_stand${ui.STAND}`;
const c = M.buildFromUi(ui)._check;
if (!c.solved) { console.log(`解けない: ${c.why}`); process.exit(1); }
const selfOk = !c.self || (c.self.min >= 0 && c.self.worstBound >= 0);
const doc = {
  what: "SO-101 パズルの場面と動き(sim/scene-data.js から。tools/export-physics.js)",
  made: new Date().toISOString(),
  set: ui.SET, label: opt.label, gap_mm: ui.GAP, sol: ui.SOL, stand: ui.STAND,
  kinematic: { hits: c.hitFrames, self_mm: c.self ? c.self.min * 1000 : null, selfOk, minClear_mm: c.minClear * 1000 },
  units: "長さ m・角度 度。画面の座標(Y が上・机の上の面 = 0)。MuJoCo = (x, -z, y)",
  robotY: c.robotY, cell: c.cell, pitch: c.pitch, T: c.T,
  jaw: { home: c.homeJaw, grip: c.grip },
  joints: ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"],
  motion: c.motion.map(f => ({ t: f.t, q: f.q, jaw: f.jaw, held: f.held })),
  pieces: c.pieces.map((p, i) => ({ idx: p.idx, name: p.name, shape: p.shape, color: p.color,
    cells: PUZZLE.SHAPES[p.shape], start: c.pieceStart[i], end: c.pieceEnd[i] })),
  steps: c.stepTimes,
  // 板は STL、置き方 C の柱・吊る腕・カメラは線分と半径(机 = 0 の画面の座標)。run_physics.py は見た目だけに使う
  stand: c.stand ? M.STAND_PARTS.map(s => s.box ? { id: s.id, lo: [s.box.lo[0], s.box.lo[1] + c.robotY, s.box.lo[2]], hi: [s.box.hi[0], s.box.hi[1] + c.robotY, s.box.hi[2]] }
                                         : s.mesh ? { id: s.id, mesh: s.mesh, pos: M.standPos(s.mesh) }
                                                 : { id: s.id, a: [s.seg.a[0], s.seg.a[1] + c.robotY, s.seg.a[2]], b: [s.seg.b[0], s.seg.b[1] + c.robotY, s.seg.b[2]], r: s.seg.r }) : []
};
const dir = path.join(ROOT, "out", "physics", name);
fs.mkdirSync(dir, { recursive: true });
fs.writeFileSync(path.join(dir, "scene.json"), JSON.stringify(doc));
console.log(`${opt.label}: コマ ${doc.motion.length}・${c.T.toFixed(1)} 秒・ピース ${doc.pieces.length} → ${path.relative(ROOT, dir)}/scene.json`);
