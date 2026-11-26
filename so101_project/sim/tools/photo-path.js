// 写す姿勢と、待ちの姿勢までの経由点を試す(計画書_完成形まで の S-10・S-3)。
//   写す姿勢の候補(--q)と経由点(--via。写す姿勢の側から順に、";" で区切る。"_" はその関節を待ちの姿勢の値にする)を、scene-data.js の 4 種に入れて動きを組み、
//   腕どうし(self-collision.js。コマの間も下限で見る)・台・机・ピースとの当たりを出す。隠れるかは tools/overhead-occlusion.js で見る(そちらは PHOTO_Q を読む)。
//   実行: node tools/photo-path.js --q "-10,-80,-30,45,0" --via "_,-80,-30,45,_;_,-40,40,90,_"
const { load } = require("./_load");
load("kinematics.js", "so101.js", "motion.js", "puzzle.js",
     "models/so101_visuals.js", "models/so101_points.js", "models/so101_tris.js", "collide.js",
     "chikuwa-collision.js", "self-collision.js", "scene-data.js");
const M = SCENE_MODEL;
const arg = k => { const i = process.argv.indexOf("--" + k); return i > 0 ? process.argv[i + 1] : undefined; };
const q = arg("q") ? arg("q").split(",").map(Number) : undefined;
const via = arg("via") === undefined ? undefined : arg("via") === "" ? [] : arg("via").split(";").map(s => s.split(",").map(x => x.trim() === "_" ? null : +x));
const sets = M.controls.find(c => c.key === "SET").options.map(o => o.value), def = M.defaultUiValues();
const mm = v => (v * 1000).toFixed(1);
let bad = 0;
console.log(`写す姿勢 ${q ? q.join(",") : "(既定)"}・経由点 ${via ? JSON.stringify(via) : "(既定)"}`);
for (const set of sets) {
  const t0 = Date.now();
  const c = M.buildFromUi({ SET: set, GAP: def.GAP, SOL: 0, NO_REFINE: true, _PHOTO_Q: q, _PHOTO_VIA: via })._check;
  const s = c.self, ok = c.solved && !c.hitFrames && s && s.min >= SELF_COLLIDE.WARN && s.worstBound >= 0;
  if (!ok) bad++;
  console.log(`[種 ${set}] ${ok ? "OK" : "★NG"} 解けた ${c.solved}・当たり ${c.hitFrames} 回${c.firstHit ? "(最初 " + c.firstHit.t.toFixed(2) + " 秒 " + c.firstHit.bad.join("、") + ")" : ""}・` +
              `腕どうし ${s ? mm(s.min) + "mm(" + s.pair + "・" + s.t.toFixed(2) + " 秒・関節 " + s.q.map(x => x.toFixed(0)).join(",") + ")" + (s.worstBound < 0 ? "・割り切れない下限 " + mm(s.worstBound) : "") : "—"}・` +
              `台 ${c.minStandPair ? mm(c.minStand) + "mm(" + c.minStandPair + ")" : "15mm より遠い"}・いちばん近い ${mm(c.minClear)}mm(${c.minClearPair})・${c.T.toFixed(1)} 秒・${((Date.now() - t0) / 1000).toFixed(1)} 秒で測った`);
}
if (bad) process.exitCode = 1;
