// 手首のカメラの絵から、狙う 1 つのピースの置き場を測る(09 の I-5・08 §5 の (6)。W3 の制御プログラム tools/puzzle_run.py が使う)。
//   vision.js の detect は、どれか 1 個でも 20 画素に足りないと失敗にするので、1 個を狙う手首の絵には使えない(09 §5)。ここは 1 個だけを見る。
//   やり方は detect と同じ: 狙うピースの色合い(HUE_TOL 以内・彩度 SAT_MIN 以上)の画素 → まわり 8 画素のうち NEIGH 個以上が同じ色の画素だけ残す
//   → 明るさを 2 つに分けて(大津の方法)明るい方 = 上の面 → 上の面の高さ(辺)の水平の面へ光線を当てる → 形を合わせる(VISION.fitShape)。
//   手首のカメラの置き方は、そのときの関節角から順運動学で出す(cameraAt)。実機では、手首のカメラと爪の位置関係(W4)とカメラの中の値(K5)が要る。
//   見ていないもの: レンズのゆがみ・ぼけ・影・色むら、絵の端で切れたピース(切れていれば why を返す)、置いたピースが絵に入ること(色が違えば分かれる)。
// 読み込み: kinematics.js・so101.js・puzzle.js・vision.js のあと。
var VISION_WRIST = (function () {
  "use strict";
  var DEG = Math.PI / 180;
  var SAT_MIN = 0.35, HUE_TOL = 18, NEIGH = 5, MIN_PIX = 200;   // detect と同じ分け方。MIN_PIX: 手首の絵ではピースが大きく写る(既定の 3×4 で 6000〜7800 画素)
  function hsv(r, g, b) {
    var mx = Math.max(r, g, b), mn = Math.min(r, g, b), d = mx - mn, h = 0;
    if (d > 0) { if (mx === r) h = ((g - b) / d) % 6; else if (mx === g) h = (b - r) / d + 2; else h = (r - g) / d + 4; h *= 60; if (h < 0) h += 360; }
    return [h, mx > 0 ? d / mx : 0, mx];
  }
  function otsu(vals) {
    var hist = new Array(256).fill(0); vals.forEach(function (v) { hist[Math.max(0, Math.min(255, v | 0))]++; });
    var tot = vals.length, sum = 0; for (var i = 0; i < 256; i++) sum += i * hist[i];
    var sB = 0, wB = 0, best = 0, thr = 0;
    for (var t = 0; t < 256; t++) { wB += hist[t]; if (!wB) continue; var wF = tot - wB; if (!wF) break; sB += t * hist[t]; var mB = sB / wB, mF = (sum - sB) / wF, v = wB * wF * (mB - mF) * (mB - mF); if (v > best) { best = v; thr = t; } }
    return thr;
  }
  // 関節角(URDF の度)・爪(度)・腕の原点の高さ(机から m) → 手首のカメラ {tip, dir, up}(机 = 0)。scene-data.js の「手首のカメラから」の視点と同じ
  //   cig(なくてよい): 測った手首のカメラの置き方(tools/k5_wrist.py の cam_in_gripper を m に直した物 {t: [m], R: 9 個・行の順})。
  //     URDF の gripper_link の座標で、t = 光の中心、R の列 = カメラの x(絵の右)・y(絵の下)・z(向き)。渡せば URDF の wrist_camera_link の代わりに使う
  function cameraAt(q, jaw, robotY, cig) {
    var fk = SO101.urdfFK(q.map(function (x) { return x * DEG; }), (jaw || 0) * DEG);
    if (cig) {
      var g = fk.gripper_link, R = g.R, t = cig.t, C = cig.R;
      var mv = function (v) { return [R[0] * v[0] + R[1] * v[1] + R[2] * v[2], R[3] * v[0] + R[4] * v[1] + R[5] * v[2], R[6] * v[0] + R[7] * v[1] + R[8] * v[2]]; };
      var p = mv(t), z = mv([C[2], C[5], C[8]]), y = mv([C[1], C[4], C[7]]);
      return { tip: [g.p[0] + p[0], g.p[1] + p[1] + robotY, g.p[2] + p[2]], dir: z, up: [-y[0], -y[1], -y[2]] };
    }
    var cf = fk.wrist_camera_link;
    return { tip: [cf.p[0], cf.p[1] + robotY, cf.p[2]], dir: [cf.R[2], cf.R[5], cf.R[8]], up: [-cf.R[1], -cf.R[4], -cf.R[7]] };
  }
  // URDF の wrist_camera_link を gripper_link の座標で表した物(cameraAt の cig と同じ形)。puzzle_run.py の sim の較正ファイル作りと検算用
  function urdfCamInGripper() {
    var fk = SO101.urdfFK([0, 0, 0, 0, 0], 0), g = fk.gripper_link, c = fk.wrist_camera_link, R = g.R;
    var tr = function (v) { return [R[0] * v[0] + R[3] * v[1] + R[6] * v[2], R[1] * v[0] + R[4] * v[1] + R[7] * v[2], R[2] * v[0] + R[5] * v[1] + R[8] * v[2]]; };   // Rᵗ·v
    var t = tr([c.p[0] - g.p[0], c.p[1] - g.p[1], c.p[2] - g.p[2]]);
    var cols = [0, 1, 2].map(function (k) { return tr([c.R[k], c.R[3 + k], c.R[6 + k]]); });   // カメラの軸(列)を gripper_link で
    return { t: t, R: [cols[0][0], cols[1][0], cols[2][0], cols[0][1], cols[1][1], cols[2][1], cols[0][2], cols[1][2], cols[2][2]] };
  }
  // 一部が隠れた形を合わせる。手首の絵では、開いた爪の指がピースの一部を隠す(既定の 3×4 の J で、足のマスの半分ほど。2026-11-18)。
  //   VISION.fitShape は点の重心と形の重心を合わせるので、隠れた分だけ位置がずれる(J で 7.9mm 外れた)。ここは、点が形の外へ出る量(2 乗・広さの重み)だけを
  //   小さくする(形の中で点が無い所は罰しない)。点が全部形の中に入る位置が幅を持つ向きにだけ効くよう、見積もり(init)からの距離をごく弱く足す。
  //   init から始めて Nelder–Mead で探す(上の絵の見積もりは 1 マスより十分近い前提。08 §5 の (6))。
  function fitPartial(Q, cells, pitch, cell, init) {
    var h = cell / 2, C = cells.map(function (c) { return [c[0] * pitch, c[1] * pitch]; }), ws = 0, LAMBDA = 1e-4;
    Q.forEach(function (p) { ws += p[2]; });
    function cost(v) {
      var x = v[0], z = v[1], yr = v[2] * DEG, c = Math.cos(yr), s = Math.sin(yr), sum = 0;
      for (var i = 0; i < Q.length; i++) {
        var dx = Q[i][0] - x, dz = Q[i][1] - z, lx = dx * c + dz * s, lz = -dx * s + dz * c, best = Infinity;   // rotY(·, −yaw)
        for (var k = 0; k < C.length; k++) {
          var ex = Math.max(0, Math.abs(lx - C[k][0]) - h), ez = Math.max(0, Math.abs(lz - C[k][1]) - h), d2 = ex * ex + ez * ez;
          if (d2 < best) { best = d2; if (!best) break; }
        }
        sum += best * Q[i][2];
      }
      return sum / ws + LAMBDA * ((x - init[0]) * (x - init[0]) + (z - init[1]) * (z - init[1]));
    }
    // Nelder–Mead(x, z は m、yaw は度。最初の広がり 3mm・3°)
    var P = [init.slice(), [init[0] + 0.003, init[1], init[2]], [init[0], init[1] + 0.003, init[2]], [init[0], init[1], init[2] + 3]];
    var F = P.map(cost);
    for (var it = 0; it < 400; it++) {
      var ord = [0, 1, 2, 3].sort(function (a, b) { return F[a] - F[b]; }); P = ord.map(function (i) { return P[i]; }); F = ord.map(function (i) { return F[i]; });
      var cen = [0, 1, 2].map(function (d) { return (P[0][d] + P[1][d] + P[2][d]) / 3; });
      var at = function (t) { return cen.map(function (v, d) { return v + t * (P[3][d] - v); }); };
      var xr = at(-1), fr = cost(xr);
      if (fr < F[0]) { var xe = at(-2), fe = cost(xe); if (fe < fr) { P[3] = xe; F[3] = fe; } else { P[3] = xr; F[3] = fr; } }
      else if (fr < F[2]) { P[3] = xr; F[3] = fr; }
      else { var xc = at(0.5), fc = cost(xc); if (fc < F[3]) { P[3] = xc; F[3] = fc; } else { for (var j = 1; j < 4; j++) { P[j] = P[j].map(function (v, d) { return P[0][d] + 0.5 * (v - P[0][d]); }); F[j] = cost(P[j]); } } }
      if (Math.abs(P[3][0] - P[0][0]) < 1e-6 && Math.abs(P[3][1] - P[0][1]) < 1e-6 && Math.abs(P[3][2] - P[0][2]) < 1e-3) break;
    }
    var b = 0; for (var q = 1; q < 4; q++) if (F[q] < F[b]) b = q;
    return { x: P[b][0], z: P[b][1], yaw: P[b][2], cost: F[b] };
  }
  //   im: {W, H, rgb}。cam: {tip, dir, up}。piece: {name, shape, color}。known: {pitch, cell}(m)。fov: 縦の画角(度。既定は SO101.CAM_FOV.wrist)
  //   est: 上の絵の見積もり {x, z, yaw}(mm・度)。あれば一部が隠れても合う fitPartial、無ければ VISION.fitShape(全部写っている前提)
  //   戻り: {x, z, yaw(mm・度。scene-data.js の LAYOUT の 1 個と同じ形), nPix, nTop, cost, topFrac, why}
  function detectOne(im, cam, piece, known, fov, est) {
    var W = im.W, H = im.H, img = SO101.overheadImage(cam, fov || SO101.CAM_FOV.wrist, W, H);
    var c0 = VISION.hex(piece.color), hue = hsv(c0[0], c0[1], c0[2])[0];
    var lab = new Uint8Array(W * H), val = new Float32Array(W * H);
    for (var q = 0; q < W * H; q++) {
      var v = hsv(im.rgb[q * 3], im.rgb[q * 3 + 1], im.rgb[q * 3 + 2]);
      if (v[1] < SAT_MIN || v[2] < 40) continue;
      var dh = Math.abs(v[0] - hue); dh = Math.min(dh, 360 - dh);
      if (dh < HUE_TOL) { lab[q] = 1; val[q] = v[2]; }
    }
    var P = [], edge = 0;
    for (var y = 1; y < H - 1; y++) for (var x = 1; x < W - 1; x++) {
      var q3 = y * W + x; if (!lab[q3]) continue;
      var nb = 0;
      for (var dy = -1; dy <= 1; dy++) for (var dx = -1; dx <= 1; dx++) if ((dx || dy) && lab[q3 + dy * W + dx]) nb++;
      if (nb >= NEIGH) { P.push([x, y, val[q3]]); if (x <= 2 || y <= 2 || x >= W - 3 || y >= H - 3) edge++; }
    }
    if (P.length < MIN_PIX) return { why: "手首の絵にピース " + piece.name + " の色が " + P.length + " 画素しかない(" + MIN_PIX + " 未満)", nPix: P.length };
    if (edge > 0) return { why: "ピース " + piece.name + " が手首の絵の端で切れている(端の画素 " + edge + ")", nPix: P.length };
    var thr = otsu(P.map(function (e) { return e[2]; })), top = P.filter(function (e) { return e[2] > thr; });
    var Q = top.map(function (e) {
      var w = img.unproject(e[0] + 0.5, e[1] + 0.5, known.cell), a = img.unproject(e[0], e[1], known.cell), b = img.unproject(e[0] + 1, e[1], known.cell), c = img.unproject(e[0], e[1] + 1, known.cell);
      var area = Math.abs((b[0] - a[0]) * (c[2] - a[2]) - (b[2] - a[2]) * (c[0] - a[0]));
      return [w[0], w[2], area];
    });
    var fit = est ? fitPartial(Q, PUZZLE.SHAPES[piece.shape], known.pitch, known.cell, [est.x / 1000, est.z / 1000, est.yaw])
                  : VISION.fitShape(Q, PUZZLE.SHAPES[piece.shape], known.pitch, known.cell);
    // 上の面の見えた広さ(机の上の面積の和) ÷ 形の上の面の広さ(マスの数 × 辺²)。S2(2026-11-22): 大半が隠れると fitPartial は見積もりの近くの
    //   違う所に合うことがある(画素 390 で見積もり 3mm ずれ → 7.3mm 外した)。読み手が隠れの多さで断れるように返す(ここでは断らない)
    var topArea = 0; Q.forEach(function (e) { topArea += e[2]; });
    var topFrac = topArea / (PUZZLE.SHAPES[piece.shape].length * known.cell * known.cell);
    return { x: +(fit.x * 1000).toFixed(2), z: +(fit.z * 1000).toFixed(2), yaw: +fit.yaw.toFixed(2), nPix: P.length, nTop: top.length, cost: fit.cost, topFrac: +topFrac.toFixed(4), why: "" };
  }
  return { cameraAt: cameraAt, urdfCamInGripper: urdfCamInGripper, detectOne: detectOne, fitPartial: fitPartial, MIN_PIX: MIN_PIX };
})();
if (typeof module !== "undefined") module.exports = VISION_WRIST;
