// 真上を向けて届く範囲を「肩の回転軸からの距離」で測る(高さごと・3 方向)。
//   scene-data.js の R_IN / R_OUT(運ぶ高さ 50mm で 80〜240mm → 余裕 5mm で 85〜235mm)はこの結果から。
//   見ている世界: SO-101(so101_new_calib.urdf)の関節と可動域・爪の向き 0° と 90° の両方が解けるか
//   見ていないもの: 当たり判定・ピースの大きさ(道具の点だけ)
//   ★高さは腕の原点(base_link)から。机からの高さは、これに 2.4mm を足す(2026-11-18 までは台ありで 9.6mm)。
//     運ぶ高さ 50mm(机から)は腕の原点から 47.6mm・40.4mm になり、届く輪は 80〜240mm より広がる(2026-11-16 に確かめた)
//   実行: node tools/reach-ring.js   (2026-11-12 の実測: H=50mm で 0°:80〜240 / 45°:75〜245 / 90°:80〜240)
const { load } = require("./_load");
load("kinematics.js", "so101.js", "motion.js");
const ax = SO101.joints[0].origin[0];
for (const H of [15, 40, 45, 50, 60]) {
  const out = [];
  for (const ang of [0, 45, 90]) {
    let lo = null, hi = null;
    for (let r = 50; r <= 320; r += 5) {
      const x = ax + r / 1000 * Math.cos(ang * Math.PI / 180), z = r / 1000 * Math.sin(ang * Math.PI / 180);
      const ok = MOTION.topDown([x, H / 1000, z], 0, null).ok && MOTION.topDown([x, H / 1000, z], 90, null).ok;
      if (ok) { if (lo === null) lo = r; hi = r; }
    }
    out.push(ang + "°:" + lo + "〜" + hi);
  }
  console.log("道具の点の高さ " + H + "mm  肩の軸からの距離[mm]  " + out.join("  "));
}
