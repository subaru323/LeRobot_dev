// S0 の補足（点検 C58 の 3）: ピースを絵から消したとき、detect の各ピースの色の画素の数（段 A の nPix・段 B の nPixB）を出す。
//   欠けたピースの色として、ほかのピースの画素を拾っていないかを見る。sim/plan-cli.js を子のプロセスで使う（render は sim だけ）。
//   node tools/s0_missing_px.js [seed]      （作業フォルダの根で。答えは標準出力。outputs/s0/ に残すなら > でファイルへ）
//   既定盤（set 2: L・J・O）の始めの姿勢の上の絵。消す組: なし / O / J / L / L と O / J と O。detect は全部のピースを探す（消した組の確かめは only で 1 つだけ）。
"use strict";
const { spawnSync } = require("child_process"), path = require("path"), os = require("os"), fs = require("fs");
const SIM = path.join(__dirname, "..", "sim"), seed = +(process.argv[2] || 1);
const init = { cmd: "init", set: 2, k1: "fake", calib: "sim", seed };
function ask(lines) {
  const r = spawnSync("node", ["plan-cli.js"], { cwd: SIM, input: lines.map(l => JSON.stringify(l)).join("\n") + "\n", maxBuffer: 1 << 26 });
  return r.stdout.toString().trim().split("\n").map(l => JSON.parse(l));
}
const a0 = ask([init, { cmd: "truth", seed }]);
const truth = a0[1].layout, L = a0[0].start_lerobot;   // puzzle_run.py は始めの姿勢から 3° 離れた所から動き出すが、最初の上の絵は始めの姿勢で撮る
const tmp = fs.mkdtempSync(path.join(os.tmpdir(), "s0px_"));
const names = a0[0].pieces.map(p => p.name);
const sets = [[[], null], [[2], null], [[1], null], [[0], null], [[0, 2], [2]], [[1, 2], [2]]];
console.log(`seed ${seed}, pieces ${names.join(",")}`);
for (const [hide, only] of sets) {
  const out = path.join(tmp, "top_" + (hide.join("") || "none") + ".rgb");
  const im = ask([init, { cmd: "render", cam: "top", truth, hide, lerobot: L, out }])[1];
  const d = ask([init, { cmd: "detect", img: out, W: im.W, H: im.H, only }])[1];
  const per = (d.per || []).map(p => `${p.name}: A ${p.nPix}` + (p.mode === "B" ? ` B ${p.nPixB}` + (p.sil != null ? ` 輪郭 ${p.sil.toFixed(2)}mm` : "") : ` 輪郭 ${p.sil.toFixed(2)}mm`)).join(" / ");
  console.log(`消す ${hide.map(i => names[i]).join("・") || "なし"}${only ? "（探すのは " + only.map(i => names[i]).join("・") + " だけ）" : ""}: ${d.ok ? "見つけた" : "断る: " + d.why} | ${per}`);
}
fs.rmSync(tmp, { recursive: true, force: true });
