// 関節の並びを書き出す(計画書_完成形まで の S-6)。画面を開かずに、パズルの画面と同じ動きを、
// 時刻・5 つの関節の角度・爪の角度・持っているピース の表にして、JSON と CSV に書く。
//   見ている世界: scene-data.js の buildFromUi(画面と同じファイル・同じ値)。
//   角度は URDF(so101_new_calib)の角度[度]。★LeRobot の角度(shoulder_pan.pos など)への換算は、
//     キャリブレーションのファイルごとに違い、まだ決めていない(計画書の V-4)。爪は URDF の度で、LeRobot の 0〜100 ではない。
//   一定の間隔(--hz)に割り直した表も書く。表の各点は、となりのコマの関節角を線形に結んだ線(当たり判定がコマの間も測っている線)の上にある。
//     ★ただし表の点どうしを結ぶ線は、その線の角を内側に削る(最大 0.3°・道具の点 0.9mm ほど。下の「割り直しの誤差」)。そこは判定していない(点検 C11 の 3)。
//   検算: (1) 書いたファイルを読み戻して、コマの時刻で組み立て直した関節角・道具の点が、元と合うか
//         (2) 負の検算: 読み戻した表の途中を 5° ずらすと、(1) が NG を出すか
//         (3) 関節ごとの最大の速さ・加速度(実機のサーボの上限とはまだ比べていない)
//   実行: node tools/export-motion.js [--set 0..3] [--gap mm] [--sol n] [--stand 0|1] [--hz 30] [--out ファイル名(拡張子なし)]
const fs = require("fs"), path = require("path");
const { load, ROOT } = require("./_load");
load("kinematics.js", "so101.js", "motion.js", "puzzle.js",
     "models/so101_visuals.js", "models/so101_points.js", "models/so101_tris.js", "collide.js", "scene-data.js");
const M = SCENE_MODEL;
const arg = k => { const i = process.argv.indexOf("--" + k); return i > 0 ? process.argv[i + 1] : undefined; };
const def = M.defaultUiValues();
const ui = { SET: +(arg("set") ?? 0), GAP: +(arg("gap") ?? def.GAP), SOL: +(arg("sol") ?? 0), STAND: +(arg("stand") ?? 1) };
const HZ = +(arg("hz") ?? 30);
const setLabel = M.controls.find(c => c.key === "SET").options.find(o => o.value === ui.SET).label;
const out = arg("out") || path.join(ROOT, "out", `motion_set${ui.SET}_gap${ui.GAP}_stand${ui.STAND}`);
const NAMES = SO101.joints.map(j => j.name);   // shoulder_pan … wrist_roll

const c = M.buildFromUi(ui)._check;
if (!c.solved) { console.log(`解けない: ${c.why}`); process.exit(1); }
const fr = c.motion;
// 一定の間隔に割り直す(となりのコマを線形に)
function at(seq, t) {
  let i = 1;
  while (i < seq.length - 1 && seq[i].t < t) i++;
  const a = seq[i - 1], b = seq[i], u = b.t > a.t ? Math.min(1, Math.max(0, (t - a.t) / (b.t - a.t))) : 1;
  return { q: a.q.map((v, k) => v + (b.q[k] - v) * u), jaw: a.jaw + (b.jaw - a.jaw) * u, held: u > 0 ? b.held : a.held };   // 間は次のコマの持ち方(当たり判定と同じ)
}
const samples = [];
for (let k = 0; k * (1 / HZ) <= c.T + 1e-9; k++) { const t = k / HZ, s = at(fr, t); samples.push({ t, q: s.q, jaw: s.jaw, held: s.held }); }
const r3 = v => Math.round(v * 1000) / 1000;
const doc = {
  what: "SO-101 パズルの動き(sim/scene-data.js から書き出し。tools/export-motion.js)",
  made: new Date().toISOString(),
  ui, set: setLabel, solved: c.solved, hits: c.hitFrames, order: c.order, T: r3(c.T),
  units: { t: "秒", q: "URDF so101_new_calib の関節角[度](LeRobot の角度への換算は未定。計画書の V-4)", jaw: "URDF の爪の角度[度](LeRobot の 0〜100 ではない)", held: "持っているピースの番号(-1 は持っていない)" },
  joints: NAMES, hz: HZ,
  frames: fr.map(f => ({ t: Math.round(f.t * 1e6) / 1e6, q: f.q.map(r3), jaw: r3(f.jaw), held: f.held })),
  samples: samples.map(f => ({ t: Math.round(f.t * 1e6) / 1e6, q: f.q.map(r3), jaw: r3(f.jaw), held: f.held }))
};
fs.mkdirSync(path.dirname(out), { recursive: true });
fs.writeFileSync(out + ".json", JSON.stringify(doc));
const csv = ["t," + NAMES.join(",") + ",gripper_deg,held"].concat(doc.samples.map(s => [s.t].concat(s.q, [s.jaw, s.held]).join(","))).join("\n") + "\n";
fs.writeFileSync(out + ".csv", csv);
console.log(`[${setLabel} / すき間 ${ui.GAP}mm / 台 ${ui.STAND ? "あり" : "なし"}] 解けた 順番 ${c.order.join("→")} / 当たり ${c.hitFrames} 回 / ${c.T.toFixed(1)} 秒`);
console.log(`  書いた: ${path.relative(ROOT, out)}.json(コマ ${fr.length}・${HZ}Hz に割り直し ${samples.length})と .csv(${HZ}Hz)`);

// (1) 読み戻して、元のコマの時刻で組み立て直す
const back = JSON.parse(fs.readFileSync(out + ".json", "utf8"));
const tool = q => KIN.fk(SO101.axes, q, SO101.tool).pos;
function compare(seq, label) {
  let dq = 0, dp = 0, dj = 0, at1 = 0;
  fr.forEach(f => {
    const s = at(seq, f.t);
    const e = Math.max(...s.q.map((v, k) => Math.abs(v - f.q[k])));
    if (e > dq) { dq = e; at1 = f.t; }
    dj = Math.max(dj, Math.abs(s.jaw - f.jaw));
    const a = tool(s.q), b = tool(f.q);
    dp = Math.max(dp, Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]));
  });
  return { dq, dp, dj, at1, label };
}
const TOL_Q = 0.01, TOL_P = 0.0005;   // 0.01° と 0.5mm(丸めは 0.001°)
let ng = 0;
const r1 = compare(back.frames, "書いたコマ"), r2 = compare(back.samples, `${HZ}Hz の表`);
[r1].forEach(r => {
  const ok = r.dq < TOL_Q && r.dp < TOL_P;
  if (!ok) ng++;
  console.log(`  (1) ${r.label} を読み戻す: 関節角の差 最大 ${r.dq.toFixed(4)}°・爪 ${r.dj.toFixed(4)}°・道具の点 ${(r.dp * 1000).toFixed(3)}mm ${ok ? "OK" : "★NG"}`);
});
console.log(`      ${r2.label} から元のコマの時刻を組み立てた差(割り直しの誤差。参考): 関節角 最大 ${r2.dq.toFixed(3)}°(${r2.at1.toFixed(2)} 秒)・道具の点 ${(r2.dp * 1000).toFixed(2)}mm`);
// (2) 負の検算: 読み戻した表の真ん中のコマで shoulder_lift を 5° ずらす
const bad = JSON.parse(JSON.stringify(back.frames)), mid = Math.floor(bad.length / 2);
bad[mid].q[1] += 5;
const rn = compare(bad, "5° ずらした表");
const negOk = rn.dq >= TOL_Q && rn.dp >= TOL_P;
if (!negOk) ng++;
console.log(`  (2) 負の検算: ${rn.label} → 関節角の差 ${rn.dq.toFixed(2)}°・道具の点 ${(rn.dp * 1000).toFixed(1)}mm ${negOk ? "OK(ずれを見つけた)" : "★NG(ずれを見逃した)"}`);
// (3) 関節ごとの最大の速さ・加速度(割り直した表の差分)
const dt = 1 / HZ, vmax = NAMES.map(() => 0), vat = NAMES.map(() => 0), amax = NAMES.map(() => 0);
for (let k = 1; k < samples.length; k++) {
  NAMES.forEach((_, j) => {
    const v = (samples[k].q[j] - samples[k - 1].q[j]) / dt;
    if (Math.abs(v) > vmax[j]) { vmax[j] = Math.abs(v); vat[j] = samples[k].t; }
    if (k > 1) amax[j] = Math.max(amax[j], Math.abs(v - (samples[k - 1].q[j] - samples[k - 2].q[j]) / dt) / dt);
  });
}
console.log(`  (3) 最大の速さ[度/秒]: ${NAMES.map((n, j) => n + " " + vmax[j].toFixed(0) + "(" + vat[j].toFixed(2) + " 秒)").join("・")}`);
console.log(`      最大の加速度[度/秒²]: ${NAMES.map((n, j) => n + " " + amax[j].toFixed(0)).join("・")}(実機のサーボの上限とは、まだ比べていない)`);
console.log(ng ? `NG ${ng} 件` : "OK");
process.exit(ng ? 1 : 0);
