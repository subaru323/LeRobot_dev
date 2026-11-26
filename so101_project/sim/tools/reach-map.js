// 机の上で「真上からつかめる」場所の地図(上から見た図・文字)。
//   見ている世界: SO-101(so101_new_calib.urdf)・机の上面 = base_link の原点の高さ。
//   つかむ高さ: 道具の点を 机の上 H mm(既定 15mm = 25mm 角のブロックの真ん中より少し上)
//   各マスで ヨー 0° と 90° の両方が解けたら「◎」、片方だけ「○」、どちらも駄目なら「・」
//   見ていないもの: 当たり判定(腕と机・ブロック)・爪の実形状(URDF の道具の点だけ)。
const { load } = require("./_load");
load("kinematics.js", "so101.js", "motion.js");
const H = (parseFloat(process.env.H) || 15) / 1000;
const STEP = 20;                         // mm
let rows = [], total = 0, both = 0;
console.log(`[SO-101] 真上からつかめる場所(道具の点の高さ 机+${(H * 1000).toFixed(0)}mm・${STEP}mm 刻み)`);
console.log("  横: X = 前(0〜400mm)  縦: Z = 右(−) 〜 左(+)  ★は土台の位置");
for (let z = 260; z >= -260; z -= STEP) {
  let line = (z >= 0 ? " " : "") + String(z).padStart(4) + " |";
  for (let x = 0; x <= 400; x += STEP) {
    if (Math.hypot(x - 39, z) < 50) { line += "★"; continue; }
    const p = [x / 1000, H, z / 1000];
    const a = MOTION.topDown(p, 0, null).ok, b = MOTION.topDown(p, 90, null).ok;
    total++; if (a && b) both++;
    line += a && b ? "◎" : (a || b) ? "○" : "・";
  }
  console.log(line);
}
console.log(`  ◎ = 両方の向きで解けた ${both} マス / 見たマス ${total}(土台の周り 50mm は除く)`);
