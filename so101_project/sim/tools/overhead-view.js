// 上のカメラ(本家 Overhead_Cam_Mount_32x32_UVC_Module)から、机の上の届く輪がどれだけ写るか(計画書_完成形まで の S-3・V-1・V-5)。
//   見ている世界: so101.js の overheadCamera(置き方 C。2026-11-17 までは OVERHEAD = 台の STL から測った置き方 A)・届く輪(scene-data.js の 85〜235mm、肩の回転軸から)。
//     机の上の面 = base_link の原点より 2.4mm 下(土台の底のリブ。so101.js の overheadTableY。2026-11-18 までは本家の板 7.2mm も足して 9.6mm)。カメラはピンホール(ゆがみなし)、絵は 640×480、縦の画角を変えて見る。
//   見ていないもの: レンズのゆがみ・画角の実際の値(品の資料は普通 対角の画角)・腕に隠れる所・ピントの合う範囲。
//     絵の上下の向きは付け方で 4 通りあるので、既定と 90° 回したものの 2 つを出す(180° 回しても写る範囲は同じ)。
//   出すもの: 絵の真ん中が机に当たる所(前・左右)、光線の角度とピースの上の面のずれ。絵の向き 2 つ × 画角ごとに、輪の前半分(腕の前 ±90°)と輪全体のうち絵に入る点の数、盤の真ん中あたりでの 1 画素の大きさ、
//     14mm のピースが何画素になるか、絵に入らない向き。
//   実行: node tools/overhead-view.js
const { load } = require("./_load");
load("kinematics.js", "so101.js");
const C = SO101.overheadCamera(), TABLE_Y = SO101.overheadTableY();
const W = 640, H = 480;
const R_IN = 0.085, R_OUT = 0.235, PAN_X = SO101.joints[0].origin[0];
function norm(v) { const n = Math.hypot(...v); return v.map(x => x / n); }
function cross(a, b) { return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]; }
function dot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
const z = norm(C.dir);
// 絵の上の向き: 基板のネジ穴は正方形に並ぶので、90° おきの 4 通りに付けられる。既定(so101.js の camUp)と、90° 回したもの
const UPS = [["既定(絵の長い辺が左右)", norm(C.up)], ["90° 回した(絵の長い辺が前後)", norm(cross(z, norm(C.up)))]];
// 画面の点 → 画素 (u: 右, v: 下)。後ろなら null。式は so101.js の overheadImage(画面の three.js のカメラと同じ向き。2026-11-17 の夜に左右の裏返りを直した)
function project(p, fovDeg, up) { return SO101.overheadImage({ tip: C.tip, dir: C.dir, up }, fovDeg, W, H).project(p); }
const inside = q => q && q[0] >= 0 && q[0] < W && q[1] >= 0 && q[1] < H;
const margin = q => Math.min(q[0], W - q[0], q[1], H - q[1]);
console.log(`[上のカメラ] レンズの先 (${C.tip.map(v => (v * 1000).toFixed(1)).join(", ")}) mm(base_link の原点、X 前・Y 上・Z 右)` +
            ` / 机から ${((C.tip[1] - TABLE_Y) * 1000).toFixed(0)} mm / 軸は真下から前へ ${(Math.acos(-z[1]) * 180 / Math.PI).toFixed(1)}° / 机の上の面 Y = ${(TABLE_Y * 1000).toFixed(1)} mm`);
const aim = (TABLE_Y - C.tip[1]) / z[1], ax = C.tip[0] + z[0] * aim, az = C.tip[2] + z[2] * aim;
console.log(`  絵の真ん中が机に当たる所: 肩の軸から 前 ${((ax - PAN_X) * 1000).toFixed(0)} mm・${az < 0 ? "左" : "右"} ${(Math.abs(az) * 1000).toFixed(0)} mm` +
            `(カメラは腕の中心線から ${(Math.abs(C.tip[2]) * 1000).toFixed(0)} mm ${C.tip[2] < 0 ? "左" : "右"}にあり、真っすぐ前へ傾く)`);
const ring = [];
for (let a = -180; a < 180; a += 5) for (let r = R_IN; r <= R_OUT + 1e-9; r += 0.01)
  ring.push({ a, r, p: [PAN_X + r * Math.cos(a * Math.PI / 180), TABLE_Y, r * Math.sin(a * Math.PI / 180)] });
const front = ring.filter(o => Math.abs(o.a) <= 90);
// ピースの上の面(14mm)を机の面として読んだときのずれ = 14 × tan(その点を見る光線の、真下からの角度)
const rayDeg = p => { const d = [p[0] - C.tip[0], p[1] - C.tip[1], p[2] - C.tip[2]]; return Math.acos(-d[1] / Math.hypot(...d)) * 180 / Math.PI; };
const angs = front.map(o => rayDeg(o.p)), lo = Math.min(...angs), hi = Math.max(...angs);
const mid = [PAN_X + (R_IN + R_OUT) / 2, TABLE_Y, 0];        // パズルの盤の真ん中の既定(scene-data.js)
const t14 = a => (14 * Math.tan(a * Math.PI / 180)).toFixed(1);
console.log(`  輪の前半分を見る光線: 真下から ${lo.toFixed(1)}〜${hi.toFixed(1)}°(盤の真ん中 ${rayDeg(mid).toFixed(1)}°)` +
            ` → 14mm のピースの上の面のずれ ${t14(lo)}〜${t14(hi)} mm(盤の真ん中 ${t14(rayDeg(mid))} mm)`);
for (const [label, up] of UPS) {
  const F = up;
  console.log(`  ■ 絵の向き: ${label}`);
  for (const fov of [45, 55, 60, 70, 80, 90]) {
    const diag = 2 * Math.atan(Math.tan(fov * Math.PI / 360) * Math.hypot(W, H) / H) * 180 / Math.PI;
    const qs = front.map(o => project(o.p, fov, F)), inF = qs.filter(inside);
    const inA = ring.filter(o => inside(project(o.p, fov, F)));
    const q0 = project(mid, fov, F), q1 = project([mid[0] + 0.001, mid[1], mid[2]], fov, F), q2 = project([mid[0], mid[1], mid[2] + 0.001], fov, F);
    const pxPerMm = Math.min(Math.hypot(q1[0] - q0[0], q1[1] - q0[1]), Math.hypot(q2[0] - q0[0], q2[1] - q0[1]));
    const missing = [...new Set(front.filter((o, i) => !inside(qs[i])).map(o => o.a))];
    const edge = inF.length === front.length ? ` / 絵の端までの余り 最小 ${Math.min(...inF.map(margin)).toFixed(0)} 画素` : "";
    console.log(`    縦の画角 ${fov}°(対角 ${diag.toFixed(0)}°): 輪の前半分 ${inF.length}/${front.length} 点・輪全体 ${inA.length}/${ring.length} 点が絵に入る` + edge +
                ` / 盤の真ん中で 1 画素 ${(1 / pxPerMm).toFixed(2)} mm・14mm のピース ${(14 * pxPerMm).toFixed(0)} 画素` +
                (missing.length ? ` / 前半分で入らない向き(腕の前 = 0°、+ が右): ${missing.slice(0, 12).join(",")}${missing.length > 12 ? " …" : ""}°` : ""));
  }
}
