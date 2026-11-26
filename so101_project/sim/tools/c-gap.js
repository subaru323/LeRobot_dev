// C の台(so101.js の OVERHEAD_C.boxesMm)と腕の、本当のすき間を、動きの全コマで測る(2026-11-19。点検 C40 の前)。
//   check-scene.js の台の判定は、面から 15mm より遠い所を測らない(「15mm より遠い」としか出ない)。ここでは箱までの距離をそのまま出す。
//   hardware/C_台の設計.md の「カメラの箱の下の端 520.0 は、腕のいちばん高い所 517.4 と 2.6 しか離れていない」(2026-11-20 から。前は 517.3・2.7。この道具の出す値は 517.4)(入ってはいけない範囲の 20mm を含む箱)を、
//   形そのもの(余裕なし)の箱で確かめる。
//   測る点: 腕の点(scene-data.js の ARM.points。土台を除く j1〜j5・動く爪・手首のカメラ。standClearance と同じ置き方)。点の間は最大 2mm ほど(check-scene の要確認と同じ甘さ)。
//   コマ: scene-data.js の _check.motion(動きの全コマ。コマの間は見ない。check-scene はコマの間も測る)。
//   出すもの: 組(目標の形)ごとに、箱ごとのいちばん近い距離 [mm]・そのときの腕の部品・時刻、腕のいちばん高い点(合板から)。
//     もう 1 つ、hardware/C_台の設計.md の「柱のまわりの入ってはいけない範囲」(余裕 20mm・ケーブル 5mm を含む円柱と箱。KEEP)までの距離も出す(点検 C40 の 5)。
//   見ていないもの: コマの間(check-scene.js の台の判定はコマの間も測るが、15mm より遠い所は「15mm より遠い」としか言わない)・
//     既定のすき間 1 通りだけ(すき間 0・RELOOK・ファイルや絵から読んだ置き場・k1-plan.js の段取り表は見ていない)・
//     持っているピースと土台(腕の点だけ)。ARM.points は 3mm ほどの間隔
//   ★負の検算: カメラの箱を腕のいちばん高い点の真上へ横に動かすと、距離が「540 − いちばん高い点」になり、5mm 下げると 0 より小さくなること。
//   実行: node tools/c-gap.js [--sets 0,1,...](既定は全部の組)
const { load } = require("./_load");
load("kinematics.js", "so101.js", "motion.js", "puzzle.js",
     "models/so101_visuals.js", "models/so101_points.js", "models/so101_tris.js", "collide.js", "scene-data.js");
const M = SCENE_MODEL, A = M.ARM, C = SO101.OVERHEAD_C;
const arg = (k, d) => { const i = process.argv.indexOf("--" + k); return i > 0 ? process.argv[i + 1] : d; };
const SETS = arg("sets", M.controls.find(c => c.key === "SET").options.map(o => o.value).join(",")).split(",").map(Number);
const GROUPS = ["j1", "j2", "j3", "j4", "j5", "jaw", "cam"];
const ry = M.robotY(true);

// 腕の点(机 = 0 の座標・m)。standClearance と同じ置き方
function armPoints(q, jawDeg) {
  const st = KIN.fk(SO101.axes, q, SO101.tool), out = [];
  GROUPS.forEach(g => {
    const fr = st.frames[(g === "jaw" || g === "cam") ? 4 : (+g.slice(1) - 1)];
    A.points[g].forEach(p0 => {
      const p = g === "jaw" ? M.jawRotate(p0, jawDeg || 0) : p0;
      const d = [p[0] - fr.o[0], p[1] - fr.o[1], p[2] - fr.o[2]], w = KIN.matVec(fr.R, d);
      out.push({ g, p: [fr.p[0] + w[0], fr.p[1] + w[1] + ry, fr.p[2] + w[2]] });
    });
  });
  return out;
}
function boxDist(p, lo, hi) {      // 箱の外は正、中は負(いちばん近い面まで)
  const d = [0, 1, 2].map(k => Math.max(lo[k] - p[k], 0, p[k] - hi[k]));
  const out = Math.hypot(d[0], d[1], d[2]);
  if (out > 0) return out;
  return -Math.min(...[0, 1, 2].map(k => Math.min(p[k] - lo[k], hi[k] - p[k])));
}
// 入ってはいけない範囲(hardware/C_台の設計.md の表。高さは合板から。c_mount.py が値の持ち主。ここは写し【仮】)
const KEEP = [
  { id: "足の金具 r62.4", d: p => cylDist(p, 353.3, 19.5, 62.4, 0, 20) },
  { id: "柱 r39.1", d: p => cylDist(p, 353.3, 19.5, 39.1, 20, 575.3) },
  { id: "吊る腕の箱", d: p => boxDist(p, [97.3, 555.3, -15.5].map(v => v / 1000), [418.3, 615.3, 54.5].map(v => v / 1000)) },
  { id: "カメラと吊る板の箱", d: p => boxDist(p, [107.3, 520.0, -30.5].map(v => v / 1000), [207.3, 575.3, 69.5].map(v => v / 1000)) }
];
function cylDist(p, cx, cz, r, y0, y1) {   // [mm] の値を受けて m で返す(縦の円柱)
  const rad = Math.hypot(p[0] - cx / 1000, p[2] - cz / 1000) - r / 1000, dy = Math.max(y0 / 1000 - p[1], 0, p[1] - y1 / 1000);
  if (rad <= 0 && dy === 0) return -Math.min(-rad, p[1] - y0 / 1000, y1 / 1000 - p[1]);
  return Math.hypot(Math.max(rad, 0), dy);
}
function keepGap(motion) {
  const best = KEEP.map(k => ({ id: k.id, d: Infinity, g: "", t: 0 }));
  motion.forEach(f => armPoints(f.q, f.jaw).forEach(({ g, p }) => KEEP.forEach((k, i) => { const d = k.d(p); if (d < best[i].d) best[i] = { id: k.id, d, g, t: f.t }; })));
  return best;
}
function measure(motion, boxes) {
  const best = boxes.map(b => ({ id: b.id, d: Infinity, g: "", t: 0 })); let top = -Infinity, topT = 0, topG = "";
  motion.forEach(f => {
    armPoints(f.q, f.jaw).forEach(({ g, p }) => {
      if (p[1] > top) { top = p[1]; topT = f.t; topG = g; }
      boxes.forEach((b, i) => { const d = boxDist(p, b.lo, b.hi); if (d < best[i].d) best[i] = { id: b.id, d, g, t: f.t }; });
    });
  });
  return { best, top, topT, topG };
}
const boxesMm = (dy) => C.boxesMm.map(b => ({ id: b.id, lo: [b.lo[0] / 1000, (b.lo[1] + (b.id === "c_cam" || b.id === "c_hanger" ? dy : 0)) / 1000, b.lo[2] / 1000],
                                                   hi: [b.hi[0] / 1000, (b.hi[1] + (b.id === "c_cam" || b.id === "c_hanger" ? dy : 0)) / 1000, b.hi[2] / 1000] }));
const opts = M.controls.find(c => c.key === "SET").options;
const def = M.defaultUiValues();
let worst = { d: Infinity }, bad = 0, keepWorst = [], keepIn = 0;
SETS.forEach(set => {
  const r = M.buildFromUi(Object.assign({}, def, { SET: set, STAND: 1 }))._check;
  if (!r.solved) { console.log(`[${opts[set] ? opts[set].label : set}] 解けない: ${r.why}`); return; }
  const m = measure(r.motion, boxesMm(0));
  console.log(`[${opts[set] ? opts[set].label : set}] ${r.motion.length} コマ / 腕のいちばん高い点 ${(m.top * 1000).toFixed(1)}mm(合板から。${m.topG}・${m.topT.toFixed(1)} 秒) / ` +
              m.best.map(b => `${b.id} ${(b.d * 1000).toFixed(1)}mm(${b.g}・${b.t.toFixed(1)} 秒)`).join(" / "));
  m.best.forEach(b => { if (b.d < worst.d) worst = Object.assign({ set }, b); if (b.d < 0) bad++; });
  const kg = keepGap(r.motion);
  console.log(`   入ってはいけない範囲まで: ` + kg.map(b => `${b.id} ${(b.d * 1000).toFixed(1)}mm(${b.g}・${b.t.toFixed(1)} 秒)`).join(" / "));
  kg.forEach((b, i) => { if (!keepWorst[i] || b.d < keepWorst[i].d) keepWorst[i] = Object.assign({ set }, b); if (b.d < 0) keepIn++; });
});
console.log(`入ってはいけない範囲: ${keepIn ? "★入った組がある(" + keepIn + ")" : "どの組も入らない"}。いちばん近い ` + keepWorst.map(b => `${b.id} ${(b.d * 1000).toFixed(1)}mm(組 ${b.set})`).join(" / "));
console.log(`いちばん近い: ${worst.id} ${(worst.d * 1000).toFixed(1)}mm(${worst.g}・組 ${worst.set}・${worst.t.toFixed(1)} 秒)`);
// 負の検算: カメラの箱(32×32)を、腕のいちばん高い点の真上へ横に動かす。高さそのまま(下の端 540mm)なら距離は 540 − いちばん高い点、
//   下の端を いちばん高い点 − 5mm まで下げると負(箱に入る)。写す姿勢のカメラの箱は、横に離れているので近くない(上の行)ことと分けて見る
const r0 = M.buildFromUi(Object.assign({}, def, { SET: SETS[0], STAND: 1 }))._check, m0 = measure(r0.motion, boxesMm(0));
let tp = null;
r0.motion.forEach(f => armPoints(f.q, f.jaw).forEach(({ p }) => { if (!tp || p[1] > tp[1]) tp = p; }));
const camAt = (y0) => [{ id: "c_cam", lo: [tp[0] - 0.016, y0, tp[2] - 0.016], hi: [tp[0] + 0.016, y0 + 0.0323, tp[2] + 0.016] }];
const n1 = measure(r0.motion, camAt(C.lensTipMm[1] / 1000)).best[0], n2 = measure(r0.motion, camAt(tp[1] - 0.005)).best[0];
const negOk = Math.abs(n1.d - (C.lensTipMm[1] / 1000 - m0.top)) < 0.0005 && n2.d < 0;
console.log(`負の検算(カメラの箱を腕のいちばん高い点の真上へ): 下の端 ${C.lensTipMm[1]}mm で ${(n1.d * 1000).toFixed(1)}mm(期待 ${((C.lensTipMm[1] / 1000 - m0.top) * 1000).toFixed(1)}mm)・` +
            `下の端を 5mm 下へ入れると ${(n2.d * 1000).toFixed(1)}mm → ${negOk ? "OK" : "NG(測り方を疑う)"}`);
console.log(bad === 0 && negOk ? "OK" : "NG");
process.exitCode = bad === 0 && negOk ? 0 : 1;
