// 見て動かす(計画書_完成形まで の S-14): 上のカメラの絵を作り、絵だけからピースの置き場を見つける。
//   ■ 絵を作る(render): 机・盤・台・腕(写す姿勢)・ピース(立方体)の三角形を、上のカメラ(so101.js の overheadImage。ピンホール・ゆがみなし)から
//     画素ごとに奥行きを比べて塗る。光は上から(向き LIGHT)。色 = 物の色 × (AMB + DIFF × |面の向き・光|)。画素の雑音は opts.noise(標準偏差・0〜255)。
//   ■ 見つける(detect): 絵と、カメラの置き方(較正で分かるもの)・ピースの形と色(分かっているもの)だけを使う。置き場の正解は見ない。
//     1) 色の濃さ(彩度)が SAT_MIN 以上の画素を、色合いがいちばん近いピースに分ける(HUE_TOL 以内)。
//     2) ピースごとに明るさの山を 2 つに分け(大津の方法)、明るい方 = 上の面とする(上からの光なので、上の面がいちばん明るい)。
//     3) 上の面の画素の真ん中から光線を引き、高さ = 辺(上の面)の水平の面に当てて机の上の点にする。
//     4) 形(マスの並び)を回しながら、点が形の外へ出る量(2 乗の和)がいちばん小さい向きを探す。位置は点の重心と形の重心を合わせて決める。
//     5) 1〜4 の答えがもっともらしくないピースだけ、色合いを合わせ直して拾い直し、明るさを使わず色の画素の輪郭に形を合わせる(段 B。下の「光の乱れに強くする所」。2026-11-18)。
//   ■ 光の乱れ(render の opts.light。K6・2026-11-18): 全体の明るさ・光の向き・影・明るさのむら・暗がり・ピースごとの色のばらつき・色温度・てかり。表は tools/vision-light.js。
//   ■ 見ていないもの: レンズのゆがみ・ぼけ・柔らかい影(光は平行で 1 つ)・照り返し・ピースの表面の模様・自動露出と自動の白の釣り合い・
//     ピースをつなぐ細い棒・ピースどうしで隠れること(写す姿勢では腕に隠れない。overhead-occlusion.js)。
// 読み込み: kinematics.js・so101.js・models/so101_visuals.js・models/so101_tris.js・chikuwa-collision.js・self-collision.js・scene-data.js のあと。
var VISION = (function () {
  "use strict";
  var LIGHT = norm([0.3, 1, 0.2]), AMB = 0.35, DIFF = 0.65;
  var COL = { table: [205, 205, 200], board: [228, 228, 222], arm: [95, 95, 100], stand: [130, 130, 130] };
  var SAT_MIN = 0.35, HUE_TOL = 18, NEIGH = 5;     // 見つけるときの色の分け方【仮】
  function norm(v) { var n = Math.hypot(v[0], v[1], v[2]); return [v[0] / n, v[1] / n, v[2] / n]; }
  function hex(c) { var n = parseInt(c.slice(1), 16); return [(n >> 16) & 255, (n >> 8) & 255, n & 255]; }
  function rotY(v, deg) { var c = Math.cos(deg * Math.PI / 180), s = Math.sin(deg * Math.PI / 180); return [v[0] * c - v[2] * s, v[1], v[0] * s + v[2] * c]; }
  // 乱数(種で決まる)
  function rng(seed) { var s = (seed >>> 0) || 1; return function () { s ^= s << 13; s >>>= 0; s ^= s >>> 17; s ^= s << 5; s >>>= 0; return s / 4294967296; }; }
  function gauss(r) { var u = Math.max(1e-12, r()), v = r(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); }

  // ---- 世界の三角形(机 = 0 の座標) ----
  //   d: SCENE_MODEL.buildFromUi の戻り。layout: {slots:[[x,z]], rots:[deg]}(正解の置き場)。q・jaw: 腕の姿勢
  //   opts.noCam: 手首のカメラと台(cam のまとまり)を描かない(手首のカメラから撮るとき。カメラは自分のレンズの中にある)
  function worldTris(d, layout, q, jaw, opts) {
    opts = opts || {};
    var c = d._check, out = [], RY = c.robotY, M = SCENE_MODEL;
    function add(pts, color, tag) { out.push({ pts: pts, color: color, tag: tag }); }
    // 机と盤
    // 机は 5cm 角に分ける(描く側は、カメラの後ろへはみ出す三角形を丸ごと捨てるので、大きい三角形のままだと手首のカメラから机が消えた)
    var T = 0.6, TS = 0.05, tp = [];
    for (var tx = -T; tx < T - 1e-9; tx += TS) for (var tz = -T; tz < T - 1e-9; tz += TS)
      tp.push([tx, 0, tz], [tx + TS, 0, tz], [tx + TS, 0, tz + TS], [tx, 0, tz], [tx + TS, 0, tz + TS], [tx, 0, tz + TS]);
    add(tp, COL.table, "table");
    var h = c.cell / 2, bp = [];
    c.boardCells.forEach(function (b) { bp.push([b[0] - h, 0.0003, b[1] - h], [b[0] + h, 0.0003, b[1] - h], [b[0] + h, 0.0003, b[1] + h], [b[0] - h, 0.0003, b[1] - h], [b[0] + h, 0.0003, b[1] + h], [b[0] - h, 0.0003, b[1] + h]); });
    add(bp, COL.board, "board");
    // 台
    if (c.stand) M.STAND_PARTS.forEach(function (s) {        // 置き方 C の箱(柱・足の金具・吊る腕・その金具・吊る板・カメラ。2026-11-19 まで本家の板と円柱)(当たり判定と同じ三角形。scene-data.js の standTris)
      var raw = M.standTris(s), pts = [];
      for (var i = 0; i < raw.length; i += 3) pts.push([raw[i], raw[i + 1], raw[i + 2]]);
      add(pts, COL.stand, "stand");
    });
    // 腕(self-collision.js のまとまりの三角形を、姿勢で動かす)
    var st = KIN.fk(SO101.axes, q, SO101.tool), G = SELF_COLLIDE.groupTris();
    SELF_COLLIDE.GROUPS.forEach(function (g) {
      if (g === "cam" && (!c.wrist || opts.noCam)) return;
      var P = SELF_COLLIDE.placeOf(st, g, jaw), pts = [];
      G[g].forEach(function (t) { t.forEach(function (x) {
        var v = [x[0] - P.o[0], x[1] - P.o[1], x[2] - P.o[2]], r = P.R ? KIN.matVec(P.R, v) : v;
        pts.push([r[0] + P.p[0], r[1] + P.p[1] + RY, r[2] + P.p[2]]);
      }); });
      add(pts, COL.arm, "arm");
    });
    // ピース(立方体。マスごと)
    c.pieces.forEach(function (pc) {
      var sl = layout.slots[pc.idx], rot = layout.rots[pc.idx], pts = [], s = c.cell, hh = s / 2;
      PUZZLE.SHAPES[pc.shape].forEach(function (cl) {
        var o = rotY([cl[0] * c.pitch, 0, cl[1] * c.pitch], rot), cx = sl[0] + o[0], cz = sl[1] + o[2];
        var V = [[-hh, 0, -hh], [hh, 0, -hh], [hh, 0, hh], [-hh, 0, hh], [-hh, s, -hh], [hh, s, -hh], [hh, s, hh], [-hh, s, hh]].map(function (v) { var w = rotY(v, rot); return [cx + w[0], w[1], cz + w[2]]; });
        [[0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]].forEach(function (f) { pts.push(V[f[0]], V[f[1]], V[f[2]], V[f[0]], V[f[2]], V[f[3]]); });
      });
      add(pts, hex(pc.color), "piece" + pc.idx);
    });
    return out;
  }

  // ---- 光の乱れ(opts.light。K6・2026-11-18) ----
  //   opts.light を渡さなければ、絵は前と 1 画素も変わらない(既定の陰: AMB + DIFF × |面の向き・光|。両面とも同じ明るさ)。
  //   渡すと、陰の付け方を「カメラに向いた面の向き」で決め、光の当たらない面は AMB だけにする(片面。斜めの光で、光と反対の横の面が暗くなる)。
  //   欄(どれも省いてよい):
  //     gain   全体の明るさ(既定 1。露出の過不足。255 を超えた所は飽和する)
  //     elev, azim  光の向き(度)。elev = 机から見上げる角度(90 = 真上)、azim = 机の上の向き(x 軸から z 軸へ)。省くと LIGHT(上からやや斜め)
  //     amb, diff   まわりの光・直の光の強さ(既定 AMB・DIFF)
  //     shadow 影(true で、腕・台・ピースが光をさえぎった所の直の光を消す。光の向きの平行投影の奥行きの地図で決める)
  //     shadowK  影の濃さ(0〜1、既定 1 = 直の光を全部消す)
  //     grad, gradAzim  明るさのむら: カメラの真下から向き gradAzim(度)に 0.3m 進むと明るさが ×(1 + grad)(直線。電灯からの距離の違い)
  //     blob   机の上の丸い暗がり {x, z, r, k}(m。人や棚の影。直の光を k だけ減らす。ふちは r の 3 割でぼかす)
  //     hue, sat, jseed  ピースごとの色のばらつき: 色合いを ±hue 度、彩度を ×(1 ± sat) の一様な乱数で(種 jseed)
  //     kelvin 光の色温度(K。6500 = 白。3000 = 電球色で赤っぽく、9000 = 青っぽい。黒体の近似の式・明るさは保つ)
  //     spec, shin  ピースのてかり(Blinn。強さ 0〜1・鋭さ。既定 0)
  function hsv2rgb(h, s, v) {
    var c = v * s, hp = ((h % 360) + 360) % 360 / 60, x = c * (1 - Math.abs(hp % 2 - 1)), m = v - c, r = [0, 0, 0];
    if (hp < 1) r = [c, x, 0]; else if (hp < 2) r = [x, c, 0]; else if (hp < 3) r = [0, c, x]; else if (hp < 4) r = [0, x, c]; else if (hp < 5) r = [x, 0, c]; else r = [c, 0, x];
    return [r[0] + m, r[1] + m, r[2] + m];
  }
  function kelvinGain(K) {    // 黒体の色(Tanner Helland の近似)を 6500K で割り、明るさ(0.299R + 0.587G + 0.114B)を 1 にそろえる
    function bb(K) {
      var t = K / 100, r, g, b;
      r = t <= 66 ? 255 : 329.698727446 * Math.pow(t - 60, -0.1332047592);
      g = t <= 66 ? 99.4708025861 * Math.log(t) - 161.1195681661 : 288.1221695283 * Math.pow(t - 60, -0.0755148492);
      b = t >= 66 ? 255 : t <= 19 ? 0 : 138.5177312231 * Math.log(t - 10) - 305.0447927307;
      return [r, g, b].map(function (x) { return Math.max(0, Math.min(255, x)); });
    }
    var a = bb(K), w = bb(6500), g = [a[0] / w[0], a[1] / w[1], a[2] / w[2]], l = 0.299 * g[0] + 0.587 * g[1] + 0.114 * g[2];
    return [g[0] / l, g[1] / l, g[2] / l];
  }
  function lightSetup(L, tris) {
    var o = {
      gain: L.gain == null ? 1 : L.gain, amb: L.amb == null ? AMB : L.amb, diff: L.diff == null ? DIFF : L.diff,
      dir: L.elev == null ? LIGHT : (function () { var e = L.elev * Math.PI / 180, a = (L.azim || 0) * Math.PI / 180; return norm([Math.cos(e) * Math.cos(a), Math.sin(e), Math.cos(e) * Math.sin(a)]); })(),
      wb: L.kelvin ? kelvinGain(L.kelvin) : [1, 1, 1], spec: L.spec || 0, shin: L.shin || 40, shadowK: L.shadowK == null ? 1 : L.shadowK,
      grad: L.grad || 0, gdir: [Math.cos((L.gradAzim || 0) * Math.PI / 180), Math.sin((L.gradAzim || 0) * Math.PI / 180)], blob: L.blob || null
    };
    var jr = rng(L.jseed || 7);
    o.colors = tris.map(function (obj) {
      if (!(L.hue || L.sat) || !/^piece/.test(obj.tag)) return obj.color;
      var h = hsv(obj.color[0], obj.color[1], obj.color[2]), dh = (jr() * 2 - 1) * (L.hue || 0), ds = 1 + (jr() * 2 - 1) * (L.sat || 0);
      return hsv2rgb(h[0] + dh, Math.max(0, Math.min(1, h[1] * ds)), h[2]);
    });
    if (L.shadow) o.smap = shadowMap(o.dir, tris);
    return o;
  }
  // 影の地図: 光の向き Ld に沿った平行投影で、さえぎる物(机と盤でない物)のいちばん光に近い奥行き(p・Ld)を画素ごとに持つ
  function shadowMap(Ld, tris) {
    var U = Math.abs(Ld[2]) < 0.9 ? norm([Ld[1], -Ld[0], 0]) : norm([0, Ld[2], -Ld[1]]);
    var Vv = [Ld[1] * U[2] - Ld[2] * U[1], Ld[2] * U[0] - Ld[0] * U[2], Ld[0] * U[1] - Ld[1] * U[0]];
    var u0 = Infinity, u1 = -Infinity, v0 = Infinity, v1 = -Infinity;
    tris.forEach(function (obj) { obj.pts.forEach(function (p) { var u = p[0] * U[0] + p[1] * U[1] + p[2] * U[2], v = p[0] * Vv[0] + p[1] * Vv[1] + p[2] * Vv[2]; if (u < u0) u0 = u; if (u > u1) u1 = u; if (v < v0) v0 = v; if (v > v1) v1 = v; }); });
    var tex = 0.001, N = Math.min(2048, Math.ceil(Math.max(u1 - u0, v1 - v0) / tex) + 2); tex = Math.max(u1 - u0, v1 - v0) / (N - 2);
    var D = new Float32Array(N * N).fill(-Infinity);
    tris.forEach(function (obj) {
      if (obj.tag === "table" || obj.tag === "board") return;
      var P = obj.pts;
      for (var k = 0; k + 2 < P.length; k += 3) {
        var s = [P[k], P[k + 1], P[k + 2]].map(function (p) { return [(p[0] * U[0] + p[1] * U[1] + p[2] * U[2] - u0) / tex, (p[0] * Vv[0] + p[1] * Vv[1] + p[2] * Vv[2] - v0) / tex, p[0] * Ld[0] + p[1] * Ld[1] + p[2] * Ld[2]]; });
        var ar = (s[1][0] - s[0][0]) * (s[2][1] - s[0][1]) - (s[2][0] - s[0][0]) * (s[1][1] - s[0][1]);
        if (Math.abs(ar) < 1e-12) continue;
        var x0 = Math.max(0, Math.floor(Math.min(s[0][0], s[1][0], s[2][0]))), x1 = Math.min(N - 1, Math.ceil(Math.max(s[0][0], s[1][0], s[2][0])));
        var y0 = Math.max(0, Math.floor(Math.min(s[0][1], s[1][1], s[2][1]))), y1 = Math.min(N - 1, Math.ceil(Math.max(s[0][1], s[1][1], s[2][1])));
        for (var j = y0; j <= y1; j++) for (var i = x0; i <= x1; i++) {
          var px = i + 0.5, py = j + 0.5;
          var w0 = ((s[1][0] - px) * (s[2][1] - py) - (s[2][0] - px) * (s[1][1] - py)) / ar, w1 = ((s[2][0] - px) * (s[0][1] - py) - (s[0][0] - px) * (s[2][1] - py)) / ar, w2 = 1 - w0 - w1;
          if (w0 < -1e-6 || w1 < -1e-6 || w2 < -1e-6) continue;
          var d = w0 * s[0][2] + w1 * s[1][2] + w2 * s[2][2], q = j * N + i;
          if (d > D[q]) D[q] = d;
        }
      }
    });
    // 光の当たる割合(0〜1)。3×3 の地図の画素で数えて、ふちを少しぼかす。ずらし BIAS は、面が自分の影に入らないため
    var BIAS = 0.002;
    return function (p) {
      var u = (p[0] * U[0] + p[1] * U[1] + p[2] * U[2] - u0) / tex, v = (p[0] * Vv[0] + p[1] * Vv[1] + p[2] * Vv[2] - v0) / tex, d = p[0] * Ld[0] + p[1] * Ld[1] + p[2] * Ld[2];
      var ci = Math.floor(u), cj = Math.floor(v), lit = 0, n = 0;
      for (var dj = -1; dj <= 1; dj++) for (var di = -1; di <= 1; di++) {
        var i = ci + di, j = cj + dj; n++;
        if (i < 0 || j < 0 || i >= N || j >= N) { lit++; continue; }
        if (!(D[j * N + i] > d + BIAS)) lit++;
      }
      return lit / n;
    };
  }

  // ---- 絵を作る ----
  //   cam: {tip, dir, up}(机 = 0)。戻り: {W, H, rgb: Uint8ClampedArray(W*H*3), id: Int16Array(W*H)(画素に写った物の番号。検算用)}
  //   opts.light: 光の乱れ(上の lightSetup)。省くと前と同じ絵
  function render(tris, cam, opts) {
    opts = opts || {};
    var W = opts.W || 640, H = opts.H || 480, img = SO101.overheadImage(cam, opts.fov || SO101.CAM_FOV.top, W, H)   // 既定は上のカメラ。手首の絵は opts.fov に SO101.CAM_FOV.wrist を渡す;
    var X = img.right, Y = img.up, Z = img.fwd, C = img.tip, f = img.f;
    var rgb = new Uint8ClampedArray(W * H * 3), zb = new Float32Array(W * H), id = new Int16Array(W * H).fill(-1);
    var NEAR = 0.01;
    var Lt = opts.light ? lightSetup(opts.light, tris) : null, base, lam, spc;
    if (Lt) { base = new Float32Array(W * H * 3); lam = new Float32Array(W * H); spc = new Float32Array(W * H); }
    tris.forEach(function (obj, oi) {
      var P = obj.pts, ocol = Lt ? Lt.colors[oi] : obj.color, isPiece = /^piece/.test(obj.tag);
      for (var k = 0; k + 2 < P.length; k += 3) {
        var a = P[k], b = P[k + 1], cc = P[k + 2], sv = [], ok = true;
        for (var m = 0; m < 3; m++) {
          var p = m === 0 ? a : m === 1 ? b : cc, d = [p[0] - C[0], p[1] - C[1], p[2] - C[2]];
          var zc = d[0] * Z[0] + d[1] * Z[1] + d[2] * Z[2];
          if (zc < NEAR) { ok = false; break; }
          sv.push([W / 2 + f * (d[0] * X[0] + d[1] * X[1] + d[2] * X[2]) / zc, H / 2 - f * (d[0] * Y[0] + d[1] * Y[1] + d[2] * Y[2]) / zc, 1 / zc]);
        }
        if (!ok) continue;
        var e1 = [b[0] - a[0], b[1] - a[1], b[2] - a[2]], e2 = [cc[0] - a[0], cc[1] - a[1], cc[2] - a[2]];
        var n = [e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]], nl = Math.hypot(n[0], n[1], n[2]);
        if (nl < 1e-14) continue;
        var col, la = 0, sp = 0;
        if (!Lt) {
          var sh = AMB + DIFF * Math.abs((n[0] * LIGHT[0] + n[1] * LIGHT[1] + n[2] * LIGHT[2]) / nl);
          col = [obj.color[0] * sh, obj.color[1] * sh, obj.color[2] * sh];
        } else {
          var nn = [n[0] / nl, n[1] / nl, n[2] / nl], tc = [C[0] - a[0], C[1] - a[1], C[2] - a[2]];
          if (nn[0] * tc[0] + nn[1] * tc[1] + nn[2] * tc[2] < 0) nn = [-nn[0], -nn[1], -nn[2]];     // カメラに向いた側を表とする
          la = Math.max(0, nn[0] * Lt.dir[0] + nn[1] * Lt.dir[1] + nn[2] * Lt.dir[2]);
          if (Lt.spec && isPiece && la > 0) {
            var tl = Math.hypot(tc[0], tc[1], tc[2]), hv = norm([Lt.dir[0] + tc[0] / tl, Lt.dir[1] + tc[1] / tl, Lt.dir[2] + tc[2] / tl]);
            sp = Lt.spec * Math.pow(Math.max(0, nn[0] * hv[0] + nn[1] * hv[1] + nn[2] * hv[2]), Lt.shin);
          }
        }
        var x0 = Math.max(0, Math.floor(Math.min(sv[0][0], sv[1][0], sv[2][0]) - 0.5)), x1 = Math.min(W - 1, Math.ceil(Math.max(sv[0][0], sv[1][0], sv[2][0]) - 0.5));
        var y0 = Math.max(0, Math.floor(Math.min(sv[0][1], sv[1][1], sv[2][1]) - 0.5)), y1 = Math.min(H - 1, Math.ceil(Math.max(sv[0][1], sv[1][1], sv[2][1]) - 0.5));
        if (x0 > x1 || y0 > y1) continue;
        var ar = (sv[1][0] - sv[0][0]) * (sv[2][1] - sv[0][1]) - (sv[2][0] - sv[0][0]) * (sv[1][1] - sv[0][1]);
        if (Math.abs(ar) < 1e-12) continue;
        for (var j = y0; j <= y1; j++) {
          var py = j + 0.5;
          for (var i = x0; i <= x1; i++) {
            var px = i + 0.5;
            var w0 = ((sv[1][0] - px) * (sv[2][1] - py) - (sv[2][0] - px) * (sv[1][1] - py)) / ar;
            var w1 = ((sv[2][0] - px) * (sv[0][1] - py) - (sv[0][0] - px) * (sv[2][1] - py)) / ar;
            var w2 = 1 - w0 - w1;
            if (w0 < 0 || w1 < 0 || w2 < 0) continue;
            var iz = w0 * sv[0][2] + w1 * sv[1][2] + w2 * sv[2][2], q = j * W + i;
            if (iz <= zb[q]) continue;
            zb[q] = iz; id[q] = oi;
            if (!Lt) { rgb[q * 3] = col[0]; rgb[q * 3 + 1] = col[1]; rgb[q * 3 + 2] = col[2]; }
            else { base[q * 3] = ocol[0]; base[q * 3 + 1] = ocol[1]; base[q * 3 + 2] = ocol[2]; lam[q] = la; spc[q] = sp; }
          }
        }
      }
    });
    if (Lt) {     // 影・むら・暗がり・色温度は、画素の世界の点(奥行きから戻す)で決める
      for (var j2 = 0; j2 < H; j2++) for (var i2 = 0; i2 < W; i2++) {
        var q1 = j2 * W + i2; if (id[q1] < 0) continue;
        var zc1 = 1 / zb[q1], ua = (i2 + 0.5 - W / 2) / f, vb = (H / 2 - (j2 + 0.5)) / f;
        var p = [C[0] + zc1 * (Z[0] + ua * X[0] + vb * Y[0]), C[1] + zc1 * (Z[1] + ua * X[1] + vb * Y[1]), C[2] + zc1 * (Z[2] + ua * X[2] + vb * Y[2])];
        var lit = Lt.smap && lam[q1] > 0 ? 1 - Lt.shadowK * (1 - Lt.smap(p)) : 1, direct = Lt.diff * lam[q1] * lit;
        if (Lt.blob) { var bd = Math.hypot(p[0] - Lt.blob.x, p[2] - Lt.blob.z), br = Lt.blob.r, e = Math.max(0, Math.min(1, (br - bd) / (0.3 * br) + 0.5)); direct *= 1 - (Lt.blob.k == null ? 0.8 : Lt.blob.k) * e; lit *= 1 - e; }
        var fld = Lt.gain * Math.max(0, 1 + Lt.grad * ((p[0] - C[0]) * Lt.gdir[0] + (p[2] - C[2]) * Lt.gdir[1]) / 0.3), I = Lt.amb + direct, s2 = 255 * spc[q1] * lit;
        for (var ch = 0; ch < 3; ch++) rgb[q1 * 3 + ch] = (base[q1 * 3 + ch] * I + s2) * Lt.wb[ch] * fld;
      }
    }
    if (opts.noise) { var r = rng(opts.seed || 1); for (var q2 = 0; q2 < rgb.length; q2++) rgb[q2] = rgb[q2] + opts.noise * gauss(r); }
    return { W: W, H: H, rgb: rgb, id: id, tags: tris.map(function (o) { return o.tag; }) };
  }

  // ---- 見つける ----
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
  // 点の集まり Q(机の上の [x, z, 重み])に、形 cells(マスの並び)・間隔 pitch・辺 cell を合わせる。戻り {x, z, yaw, cost}
  //   重み = その画素が机の上で占める広さ。遠い所ほど 1 画素が広いので、重みなしの重心は手前(カメラの側)へ寄る
  //   (3×4 の L で 1.8mm。手前のマス 123 画素・奥のマス 105 画素。2026-11-17 に測った)
  //   FIT_BAND_MM: 0〜0.5mm で puzzle_imgbias_iso.js の数がほぼ同じ(640×480 の平均 0.745〜0.746mm。点検 C47 の 2 の後に測った)。FIT_BAND_MAX_DEG: 幅を探す外の端
  var FIT_BAND_MM = 0.2, FIT_BAND_MAX_DEG = 10;
  function fitShape(Q, cells, pitch, cell) {
    var h = cell / 2, n = Q.length, qm = [0, 0], ws = 0;
    Q.forEach(function (p) { var w = p.length > 2 ? p[2] : 1; ws += w; qm[0] += p[0] * w; qm[1] += p[1] * w; });
    qm[0] /= ws; qm[1] /= ws;
    var C = cells.map(function (c) { return [c[0] * pitch, c[1] * pitch]; }), cm = [0, 0];
    C.forEach(function (c) { cm[0] += c[0] / C.length; cm[1] += c[1] / C.length; });
    function cost(yaw) {
      var cs = Math.cos(yaw * Math.PI / 180), sn = Math.sin(yaw * Math.PI / 180);
      var sx = qm[0] - (cm[0] * cs - cm[1] * sn), sz = qm[1] - (cm[0] * sn + cm[1] * cs), s = 0;
      for (var i = 0; i < n; i++) {
        var dx = Q[i][0] - sx, dz = Q[i][1] - sz, a = dx * cs + dz * sn, b = -dx * sn + dz * cs, best = Infinity;
        for (var k = 0; k < C.length; k++) {
          var ex = Math.max(0, Math.abs(a - C[k][0]) - h), ez = Math.max(0, Math.abs(b - C[k][1]) - h), e = ex * ex + ez * ez;
          if (e < best) { best = e; if (!e) break; }
        }
        s += best;
      }
      return { cost: s / n, x: sx, z: sz, yaw: yaw };
    }
    var best = null;
    for (var y = 0; y < 360; y += 2) { var r = cost(y); if (!best || r.cost < best.cost) best = r; }
    var lo = best.yaw - 2, hi = best.yaw + 2;
    for (var it = 0; it < 40; it++) { var m1 = lo + (hi - lo) / 3, m2 = hi - (hi - lo) / 3; if (cost(m1).cost <= cost(m2).cost) hi = m2; else lo = m1; }
    // 罰が同じ(多くは 0: 点が全部形の中)まま続く向きの幅があると、上の `<=` は幅の小さい向きの端へ寄る(隣の会話の列 P の 2 回目:
    //   640×480 で向きの平均 −0.48°)。幅の両端を二分で探し、真ん中を取る(2026-11-20)。幅が無ければ前と同じ所
    //   幅 = 罰が最小 + FIT_BAND_MM² / 点の数 以下の向き(点 1 つが FIT_BAND_MM はみ出す分まで同じと見る。点検 C47 の 2: 罰がちょうど 0 の
    //   ときだけの幅では、画素が 1 つはみ出すと端に寄ったまま)。y0 から外へ 0.25° ずつ歩いて最初に越えた所と、その手前の間を二分で詰める
    //   (点検 C47 の 3: 粗い探しの ±2° の窓で切ると、幅が 2° 近くより広いとき負の側へ寄った。離れた別の谷へは行かない)。外へは FIT_BAND_MAX_DEG まで
    var y0 = (lo + hi) / 2, c0 = cost(y0).cost, lim = c0 + FIT_BAND_MM * FIT_BAND_MM * 1e-6 / n + 1e-15;
    function edge(dir) {
      var a = y0, b = null;
      for (var d = 0.25; d <= FIT_BAND_MAX_DEG + 1e-9; d += 0.25) { var y = y0 + dir * d; if (cost(y).cost <= lim) a = y; else { b = y; break; } }
      if (b === null) return a;
      for (var e = 0; e < 30; e++) { var m = (a + b) / 2; if (cost(m).cost <= lim) a = m; else b = m; }
      return a;
    }
    var yl = edge(-1), yh = edge(+1);
    best = cost((yl + yh) / 2);
    best.yaw = ((best.yaw % 360) + 360) % 360;
    return best;
  }
  // ---- 光の乱れに強くする所(K6・2026-11-18) ----
  //   段 A(前からのやり方): 色の濃さ SAT_MIN・色合い HUE_TOL・明るさ 40 で拾い、明るい方を上の面として形を合わせる(fitTop)。
  //   段 A の答えが「もっともらしい」ピースは、そのまま返す。前からの絵では全部のピースが段 A のままなので、数字は前と 1 文字も変わらない。
  //     もっともらしい = 上の面の広さ(机の上・高さ = 辺)がマスの数 × 辺² の AREA_LO〜AREA_HI 倍 かつ 輪郭のずれ silScore が SIL_OK mm 以下
  //   そうでないピース(斜めの光で横の面のほうが明るい・影で上の面が割れる・てかりで上の面が白く抜ける・色がずれて拾えない)は段 B:
  //     1) 色合いの正規化: 色の濃い画素の色合いの山を探し、ピースの色合いに近い順で山を割り当て直す(電球色の光や色のばらつきで色合いが動いても拾う)
  //     2) 色の濃さの下限を、背景の色の濃さ(95% の所)から決め直す(SAT_MIN より下げてよい。てかり・露出の過多で色が薄くなるため)
  //     3) 明るさを使わずに、色の画素の輪郭(上の面 + 見える横の面)に立方体の形を合わせる(silFit)。光の向き・影・てかりに依らない
  //   段 B では、絵全体の色かぶり(色ごとの中央値。机が絵のほとんどを占める)が CAST_TOL を越えたら、机が灰色(calib.bg があればその色)に戻るよう色ごとに掛け直す(白の釣り合い)。
  //   絵全体が暗い(中央値の明るさ < BG_LUM)ときは、明るさの下限 40 を比で下げ、さらに半分にする。段 A はどちらもしない(段 A は実物の絵でも前と同じ)。
  //   ★机が灰色でないとき(木の机・緑のマット)は calib.bg を渡さないと、段 B の白の釣り合いが色を壊す(点検 C36 の 3。木の色に塗ると J が 6258 画素を拾って止まった)。
  //   per[].nPix は、いつも段 A の数え方の画素の数(live.js の「隠れた」の見張りが比べる数。段 B の数は nPixB。点検 C36 の 6)
  var CAST_TOL = 0.06, BG_LUM = 180, AREA_LO = 0.75, AREA_HI = 1.25, SIL_OK = 1.0, SIL_BAD = 1.4, HUE_SRCH = 35, HUE_TOL_B = 25;
  //   段 B でも輪郭のずれが SIL_BAD mm を越えたピースは、置き場を返さず why に書く(見つけた振りをして外れた所をつかみに行かない。呼ぶ側は止まる)
  //   (値の出どころ: 前からの絵で段 A の輪郭のずれは 0.57〜0.68mm。8 種 × 2 回の表で段 B が受け入れ、ずれの小さかったもの 1.24mm 以下。L の 3 割を腕の色で隠すと 1.55mm でマスのずれ 2.32mm、
  //    負の検算で返したもの 1.54mm でずれ 1.71mm。2.0 から 1.4 に締めた(点検 C36 の 2)。同じ表で決めた値なので、別の乱れでは確かめていない)
  // 輪郭のずれ(mm・二乗平均の根)。S: silPrep の戻り。pose: {x, z, yaw}(fitShape と同じ。形の [0,0] のマスの真ん中・度)
  //   ① 色の画素ごとに、その画素の光線(高さ 辺 〜 0 の間)が、どれかの立方体の足跡(真上から見た正方形)を通るか。通らなければ足跡までの距離²
  //   ② 立方体の上の面と底の面の点(4×4)を絵に写し、色の画素までの距離²(画素 × 画素の大きさ)。底の面は横の面の向こうなので、写った所は色の画素のはず
  //   ①だけだと、色の画素が欠けた向きへずれても分からない。②だけだと、画素がはみ出しても分からない。
  function silScore(S, pose) {
    var cs = Math.cos(pose.yaw * Math.PI / 180), sn = Math.sin(pose.yaw * Math.PI / 180), h = S.cell / 2, C = S.C, s1 = 0, s2 = 0;
    function out2(a, b) { var best = Infinity; for (var k = 0; k < C.length; k++) { var ex = Math.max(0, Math.abs(a - C[k][0]) - h), ez = Math.max(0, Math.abs(b - C[k][1]) - h), e = ex * ex + ez * ez; if (e < best) best = e; } return best; }
    for (var i = 0; i < S.segs.length; i++) {
      var g = S.segs[i], dx0 = g[0] - pose.x, dz0 = g[1] - pose.z, dx1 = g[2] - pose.x, dz1 = g[3] - pose.z;
      var a0 = dx0 * cs + dz0 * sn, b0 = -dx0 * sn + dz0 * cs, a1 = dx1 * cs + dz1 * sn, b1 = -dx1 * sn + dz1 * cs;
      var f0 = out2(a0, b0); if (!f0) continue;
      var f1 = out2(a1, b1); if (!f1) continue;
      var lo = 0, hi = 1;            // 線分の上で、足跡の外への距離² のいちばん小さい所を 3 分割で探す(マス 1 つなら下に凸)
      for (var it = 0; it < 12; it++) { var m1 = lo + (hi - lo) / 3, m2 = hi - (hi - lo) / 3; if (out2(a0 + (a1 - a0) * m1, b0 + (b1 - b0) * m1) <= out2(a0 + (a1 - a0) * m2, b0 + (b1 - b0) * m2)) hi = m2; else lo = m1; }
      var tm = (lo + hi) / 2; s1 += Math.min(f0, f1, out2(a0 + (a1 - a0) * tm, b0 + (b1 - b0) * tm));
    }
    var n2 = 0;
    for (var k = 0; k < C.length; k++) for (var yy = 0; yy < 2; yy++) for (var u = 0; u < 4; u++) for (var w = 0; w < 4; w++) {
      var la = C[k][0] + h * 0.95 * (u * 2 / 3 - 1), lb = C[k][1] + h * 0.95 * (w * 2 / 3 - 1);
      var p = [pose.x + la * cs - lb * sn, yy ? S.cell : 0, pose.z + la * sn + lb * cs], d = S.dt(p);
      s2 += d * d; n2++;
    }
    return Math.sqrt(s1 / S.segs.length + s2 / n2) * 1000;
  }
  function silPrep(P, img, cells, pitch, cell, W, H) {
    var x0 = Infinity, x1 = -1, y0 = Infinity, y1 = -1, M = 12;
    P.forEach(function (e) { if (e[0] < x0) x0 = e[0]; if (e[0] > x1) x1 = e[0]; if (e[1] < y0) y0 = e[1]; if (e[1] > y1) y1 = e[1]; });
    x0 = Math.max(0, x0 - M); y0 = Math.max(0, y0 - M); x1 = Math.min(W - 1, x1 + M); y1 = Math.min(H - 1, y1 + M);
    var ww = x1 - x0 + 1, wh = y1 - y0 + 1, m = new Uint8Array(ww * wh), dt = new Float32Array(ww * wh), bd = [];
    P.forEach(function (e) { m[(e[1] - y0) * ww + e[0] - x0] = 1; });
    for (var j = 0; j < wh; j++) for (var i = 0; i < ww; i++) if (m[j * ww + i] && (i === 0 || j === 0 || i === ww - 1 || j === wh - 1 || !m[j * ww + i - 1] || !m[j * ww + i + 1] || !m[(j - 1) * ww + i] || !m[(j + 1) * ww + i])) bd.push([i, j]);
    for (var j2 = 0; j2 < wh; j2++) for (var i2 = 0; i2 < ww; i2++) {
      if (m[j2 * ww + i2]) continue;
      var b = Infinity; for (var k = 0; k < bd.length; k++) { var dx = bd[k][0] - i2, dy = bd[k][1] - j2, d2 = dx * dx + dy * dy; if (d2 < b) b = d2; }
      dt[j2 * ww + i2] = Math.sqrt(b);
    }
    var fw = img.fwd, tip = img.tip;
    return {
      cell: cell, C: cells.map(function (c) { return [c[0] * pitch, c[1] * pitch]; }),
      segs: P.map(function (e) { var a = img.unproject(e[0] + 0.5, e[1] + 0.5, cell), b = img.unproject(e[0] + 0.5, e[1] + 0.5, 0); return [a[0], a[2], b[0], b[2]]; }),
      dt: function (p) {      // 世界の点 p を写した所から、色の画素までの距離(m。双一次で補う。窓の外は M 画素)
        var uv = img.project(p); if (!uv) return M * 0.002;
        var zc = (p[0] - tip[0]) * fw[0] + (p[1] - tip[1]) * fw[1] + (p[2] - tip[2]) * fw[2], u = uv[0] - 0.5 - x0, v = uv[1] - 0.5 - y0;
        if (u < 0 || v < 0 || u > ww - 1 || v > wh - 1) return M * zc / img.f;
        var i = Math.min(ww - 2, Math.floor(u)), j = Math.min(wh - 2, Math.floor(v)), fu = u - i, fv = v - j;
        var d = (dt[j * ww + i] * (1 - fu) + dt[j * ww + i + 1] * fu) * (1 - fv) + (dt[(j + 1) * ww + i] * (1 - fu) + dt[(j + 1) * ww + i + 1] * fu) * fv;
        return d * zc / img.f;
      }
    };
  }
  // 輪郭に合わせる: 始めの置き場をいくつか受け取り、それぞれから位置と向きを少しずつ動かして(歩幅を半分ずつ)silScore を小さくする
  function silFit(S, inits) {
    var best = null;
    inits.forEach(function (p0) {
      var p = { x: p0.x, z: p0.z, yaw: p0.yaw }, f = silScore(S, p);
      [[0.002, 4], [0.001, 2], [0.0005, 1], [0.00025, 0.5], [0.00012, 0.25], [0.00006, 0.12]].forEach(function (st) {
        for (var it = 0; it < 12; it++) {
          var moved = false;
          [["x", st[0]], ["z", st[0]], ["yaw", st[1]]].forEach(function (d) {
            [1, -1].forEach(function (sg) {
              var q = { x: p.x, z: p.z, yaw: p.yaw }; q[d[0]] += sg * d[1];
              var fq = silScore(S, q); if (fq < f - 1e-9) { p = q; f = fq; moved = true; }
            });
          });
          if (!moved) break;
        }
      });
      if (!best || f < best.sil) best = { x: p.x, z: p.z, yaw: ((p.yaw % 360) + 360) % 360, sil: f };
    });
    return best;
  }
  // 色合いの山を探し、ピースの色合いに割り当てる(段 B の 1)。hist: 1° ごとの数。
  //   戻り: [{hue, k}](山の色合いと、その山を持つピース)。まずピースごとに 1 つずつ近い山を割り当て(山が無いピースはもとの色合い)、
  //   残った山は、割り当てた山のいちばん大きいものの 1/4 以上の数があれば、HUE_SRCH 以内でいちばん近い割り当て済みの山のピースに付ける
  //   (露出の過多で、同じピースの上の面と横の面の色合いが割れるため)
  function hueModes(hist, hues) {
    var sm = new Float64Array(360);
    for (var i = 0; i < 360; i++) { var s = 0; for (var d = -3; d <= 3; d++) s += hist[(i + d + 360) % 360]; sm[i] = s; }
    var peaks = [];
    for (var i2 = 0; i2 < 360; i2++) { var c = sm[i2]; if (c >= 15 && c >= sm[(i2 + 359) % 360] && c > sm[(i2 + 1) % 360]) peaks.push(i2); }
    function cd(a, b) { var d = Math.abs(a - b) % 360; return Math.min(d, 360 - d); }
    var best = null, used = [];
    (function rec(k, got, cost, asg) {         // ピースごとに、近い山(HUE_SRCH 以内)か「無し」を選ぶ。割り当てた数が多い方、同じなら距離² の和が小さい方
      if (k === hues.length) { if (!best || got > best.got || (got === best.got && cost < best.cost)) best = { got: got, cost: cost, asg: asg.slice() }; return; }
      peaks.forEach(function (pk, pi) { if (used[pi]) return; var d = cd(pk, hues[k]); if (d > HUE_SRCH) return; used[pi] = true; asg[k] = pk; rec(k + 1, got + 1, cost + d * d, asg); used[pi] = false; });
      asg[k] = null; rec(k + 1, got, cost, asg);
    })(0, 0, 0, []);
    var res = hues.map(function (h, k) { return { hue: best.asg[k] == null ? h : best.asg[k], k: k }; });
    peaks.forEach(function (pk) {
      if (best.asg.indexOf(pk) >= 0 || sm[pk] < 0.25 * Math.max.apply(null, best.asg.map(function (a) { return a == null ? 0 : sm[a]; }))) return;   // 小さい山(雑音・背景)は付けない
      var bk = -1, bd = HUE_SRCH;
      res.slice(0, hues.length).forEach(function (r) { var d = cd(pk, r.hue); if (d <= bd) { bd = d; bk = r.k; } });
      if (bk >= 0) res.push({ hue: pk, k: bk });
    });
    return res;
  }
  function fitTop(P, img, known, shape) {    // 段 A: 明るい方 = 上の面(大津の方法)に形を合わせる。戻り: fitShape の戻り + nTop・areaRatio
    var thr = otsu(P.map(function (e) { return e[2]; })), top = P.filter(function (e) { return e[2] > thr; }), aSum = 0;
    var Q = top.map(function (e) {
      var w = img.unproject(e[0] + 0.5, e[1] + 0.5, known.cell), a = img.unproject(e[0], e[1], known.cell), b = img.unproject(e[0] + 1, e[1], known.cell), c = img.unproject(e[0], e[1] + 1, known.cell);
      var area = Math.abs((b[0] - a[0]) * (c[2] - a[2]) - (b[2] - a[2]) * (c[0] - a[0]));   // 画素の机の上での広さ
      aSum += area;
      return [w[0], w[2], area];
    });
    var fit = fitShape(Q, PUZZLE.SHAPES[shape], known.pitch, known.cell);
    fit.nTop = top.length; fit.areaRatio = aSum / (PUZZLE.SHAPES[shape].length * known.cell * known.cell);
    return fit;
  }
  //   im: render の戻り(rgb だけを使う)。calib: {cam, fov, bg?}(較正で分かるカメラ。bg = 較正の光での机の色 [R,G,B]。省くと灰色)。known: {pieces:[{idx,name,shape,color}], pitch, cell}
  //   戻り: {layout: {pieces:[{name, x, z, yaw}]}(mm・度。scene-data.js の LAYOUT と同じ形), per: [{name, nPix, nTop, cost, areaRatio, sil, mode}], why, light}
  //     mode: "A"(前からのやり方のまま)/ "B"(輪郭に合わせた)。light: 絵全体の中央値・色かぶり・掛け直した比・明るさの下限・段 B の色の濃さの下限と色合いの山
  function detect(im, calib, known) {
    var W = im.W, H = im.H, N = W * H, img = SO101.overheadImage(calib.cam, calib.fov || SO101.CAM_FOV.top, W, H);
    var hues = known.pieces.map(function (p) { var c = hex(p.color); return hsv(c[0], c[1], c[2])[0]; });
    // 0) 絵全体の色かぶりと明るさ
    var ch = [new Uint32Array(256), new Uint32Array(256), new Uint32Array(256)];
    for (var q0 = 0; q0 < N; q0++) { ch[0][im.rgb[q0 * 3]]++; ch[1][im.rgb[q0 * 3 + 1]]++; ch[2][im.rgb[q0 * 3 + 2]]++; }
    var med = ch.map(function (hh) { var s = 0; for (var i = 0; i < 256; i++) { s += hh[i]; if (s * 2 >= N) return Math.max(1, i); } return 255; });
    var lum = 0.299 * med[0] + 0.587 * med[1] + 0.114 * med[2], ref = calib.bg || [1, 1, 1], refL = 0.299 * ref[0] + 0.587 * ref[1] + 0.114 * ref[2];
    var ratio = [0, 1, 2].map(function (c) { return (med[c] / lum) / (ref[c] / refL); }), cast = Math.max.apply(null, ratio.map(function (r) { return Math.abs(r - 1); }));
    var gain = cast > CAST_TOL ? ratio.map(function (r) { return 1 / r; }) : null, vmin = 40 * Math.min(1, lum / BG_LUM);
    function hsvAll(gn) {     // 画素ごとの色合い・色の濃さ・明るさ。gn: 色ごとに掛ける比(段 B の白の釣り合い。null で掛けない)
      var A = { H: new Float64Array(N), S: new Float64Array(N), V: new Float64Array(N) };
      for (var q = 0; q < N; q++) {
        var r = im.rgb[q * 3], g = im.rgb[q * 3 + 1], b = im.rgb[q * 3 + 2];
        if (gn) { r *= gn[0]; g *= gn[1]; b *= gn[2]; }
        var v = hsv(r, g, b); A.H[q] = v[0]; A.S[q] = v[1]; A.V[q] = v[2];
      }
      return A;
    }
    var AA = hsvAll(null), Hh, Ss, Vv;
    // ラベルを付けて、ばらばらの画素を捨てる: まわり 8 画素のうち NEIGH 個以上が同じピースの色の画素だけ残す(3×4 の 4 回で比べた: 0 → 雑音 3 で 1.03mm・雑音 15 で 4 回とも外れ / 3 → 1.03・2.71mm / 5 → 0.87・1.39mm。2026-11-17)
    //   (雑音で灰色の腕や机の画素が色付きに見え、ピースに混ざった。雑音 15 で置き場が 58mm 外れた。2026-11-17)
    //   grow: 段 B では、捨てずに残った画素に隣り合う同じラベルの画素も戻す(輪郭が 1 画素削れないように)
    function label(A, satMin, vMin, centers, labOf, tolOf, grow) {     // centers[i] の色合いの画素は、ピース labOf[i] の色。A: hsvAll の戻り
      Hh = A.H; Ss = A.S; Vv = A.V;
      var lab = new Int8Array(N).fill(-1), px = known.pieces.map(function () { return []; });
      for (var q = 0; q < N; q++) {
        if (Ss[q] < satMin || Vv[q] < vMin) continue;
        var bi = -1, bd = Infinity;
        for (var k = 0; k < centers.length; k++) { var dh = Math.abs(Hh[q] - centers[k]); dh = Math.min(dh, 360 - dh); if (dh < tolOf(k) && dh < bd) { bd = dh; bi = k; } }
        lab[q] = bi < 0 ? -1 : labOf[bi];
      }
      var keep = new Uint8Array(N);
      for (var y = 1; y < H - 1; y++) for (var x = 1; x < W - 1; x++) {
        var q3 = y * W + x, L = lab[q3]; if (L < 0) continue;
        var nb = 0;
        for (var dy = -1; dy <= 1; dy++) for (var dx = -1; dx <= 1; dx++) if ((dx || dy) && lab[q3 + dy * W + dx] === L) nb++;
        if (nb >= NEIGH) { keep[q3] = 1; px[L].push([x, y, Vv[q3]]); }
      }
      if (grow) for (var y2 = 1; y2 < H - 1; y2++) for (var x2 = 1; x2 < W - 1; x2++) {
        var q4 = y2 * W + x2, L2 = lab[q4]; if (L2 < 0 || keep[q4]) continue;
        var near = false;
        for (var dy2 = -1; dy2 <= 1 && !near; dy2++) for (var dx2 = -1; dx2 <= 1; dx2++) { var qq = q4 + dy2 * W + dx2; if (keep[qq] && lab[qq] === L2) { near = true; break; } }
        if (near) px[L2].push([x2, y2, Vv[q4]]);
      }
      return px;
    }
    // 段 A: 前からのやり方のまま(絵をそのまま。明るさの下限 40・いちばん近い色合いを HUE_TOL 以内で。白の釣り合いも下限の引き下げもしない。点検 C36 の 3)
    var pxA = label(AA, SAT_MIN, 40, hues, hues.map(function (h, k) { return k; }), function () { return HUE_TOL; }, false), pxB = null, satB = null, centersB = null;
    function prepB() {      // 段 B のラベル(要るときだけ作る)。白の釣り合い(gain)と、暗い絵での明るさの下限の引き下げ(vmin)は段 B だけ
      var AB = gain ? hsvAll(gain) : AA; Hh = AB.H; Ss = AB.S; Vv = AB.V;
      var sh = new Uint32Array(101), nv = 0;
      for (var q = 0; q < N; q++) if (Vv[q] >= vmin / 2) { sh[Math.min(100, Math.round(Ss[q] * 100))]++; nv++; }
      var acc = 0, p95 = 1; for (var i = 0; i <= 100; i++) { acc += sh[i]; if (acc >= 0.95 * nv) { p95 = i / 100; break; } }
      satB = Math.max(0.15, Math.min(SAT_MIN, p95 + 0.08));
      var hh = new Float64Array(360);
      for (var q2 = 0; q2 < N; q2++) if (Ss[q2] >= satB && Vv[q2] >= vmin / 2) hh[Math.floor(Hh[q2]) % 360]++;
      centersB = hueModes(hh, hues);      // 色合いの幅は HUE_TOL_B と、ほかのピースの山までの半分の小さい方
      var tol = centersB.map(function (c) { var m = HUE_TOL_B; centersB.forEach(function (o) { if (o.k !== c.k) { var d = Math.abs(c.hue - o.hue) % 360; m = Math.min(m, Math.min(d, 360 - d) / 2); } }); return m; });
      pxB = label(AB, satB, vmin / 2, centersB.map(function (c) { return c.hue; }), centersB.map(function (c) { return c.k; }), function (i) { return tol[i]; }, true);
    }
    // 段 B の画素のうち、ピースの大きさを越えて離れたもの(雑音の色の固まり・背景の色の付いた物)を捨てる:
    //   上の面の高さで机に戻した点の真ん中(x と z の中央値)から、形のマスの真ん中までのいちばん遠い距離 + 辺 × 1.5 より遠い画素
    function near(P, cells) {
      if (P.length < 20) return P;
      var W2 = P.map(function (e) { var w = img.unproject(e[0] + 0.5, e[1] + 0.5, known.cell); return [w[0], w[2]]; });
      function median(a) { var b = a.slice().sort(function (x, y) { return x - y; }); return b[b.length >> 1]; }
      var mx = median(W2.map(function (w) { return w[0]; })), mz = median(W2.map(function (w) { return w[1]; }));
      var cm = [0, 0]; cells.forEach(function (c) { cm[0] += c[0] * known.pitch / cells.length; cm[1] += c[1] * known.pitch / cells.length; });
      var R = 0; cells.forEach(function (c) { R = Math.max(R, Math.hypot(c[0] * known.pitch - cm[0], c[1] * known.pitch - cm[1])); });
      R += known.cell * 1.5;
      return P.filter(function (e, i) { return Math.hypot(W2[i][0] - mx, W2[i][1] - mz) <= R; });
    }
    var per = [], out = [], why = "";
    known.pieces.forEach(function (p, k) {
      var P = pxA[k], cells = PUZZLE.SHAPES[p.shape], fitA = null, silA = null;
      if (P.length >= 20) {
        fitA = fitTop(P, img, known, p.shape);
        if (fitA.areaRatio >= AREA_LO && fitA.areaRatio <= AREA_HI) {
          silA = silScore(silPrep(P, img, cells, known.pitch, known.cell, W, H), fitA);
          if (silA <= SIL_OK) {
            per.push({ name: p.name, nPix: P.length, nTop: fitA.nTop, cost: fitA.cost, areaRatio: +fitA.areaRatio.toFixed(3), sil: +silA.toFixed(3), mode: "A" });
            out[p.idx] = { name: p.name, x: +(fitA.x * 1000).toFixed(2), z: +(fitA.z * 1000).toFixed(2), yaw: +fitA.yaw.toFixed(2) };
            return;
          }
        }
      }
      // 段 B
      if (!pxB) prepB();
      var PB = near(pxB[k], cells);
      if (PB.length < 20) { why = why || ("ピース " + p.name + " の色が絵に見つからない(" + PB.length + " 画素)"); per.push({ name: p.name, nPix: P.length, nPixB: PB.length, mode: "B" }); return; }
      var SB = silPrep(PB, img, cells, known.pitch, known.cell, W, H), fB = fitTop(PB, img, known, p.shape);
      var QA = PB.map(function (e) { var w = img.unproject(e[0] + 0.5, e[1] + 0.5, known.cell / 2); return [w[0], w[2], 1]; }), fAll = fitShape(QA, cells, known.pitch, known.cell);
      var inits = [fB, fAll]; if (fitA) inits.push(fitA);
      var fit = silFit(SB, inits);
      per.push({ name: p.name, nPix: P.length, nPixB: PB.length, nTop: fB.nTop, areaRatio: +fB.areaRatio.toFixed(3), areaRatioA: fitA ? +fitA.areaRatio.toFixed(3) : null, silA: silA == null ? null : +silA.toFixed(3), sil: +fit.sil.toFixed(3), mode: "B" });
      if (fit.sil > SIL_BAD) { why = why || ("ピース " + p.name + " の色の画素が形に合わない(輪郭のずれ " + fit.sil.toFixed(1) + "mm)"); return; }
      out[p.idx] = { name: p.name, x: +(fit.x * 1000).toFixed(2), z: +(fit.z * 1000).toFixed(2), yaw: +fit.yaw.toFixed(2) };
    });
    return { layout: { what: "上のカメラの絵だけから見つけた置き場(vision.js)", pieces: out }, per: per, why: why,
             light: { median: med, cast: +cast.toFixed(3), gain: gain && gain.map(function (x) { return +x.toFixed(3); }), vmin: +vmin.toFixed(1), satB: satB, hueB: centersB } };
  }
  // 置き場どうしの差: マスの真ん中を近い順に組み、2 次元の剛体の重ね合わせ(det → truth)を出す。
  //   回して同じ形になるピース(I・O・S・Z)は、名前の付いたマスが入れ替わっても同じ物なので、マスで比べる
  function cellsOf(shape, pitch, x, z, yaw) { return PUZZLE.SHAPES[shape].map(function (c) { var o = rotY([c[0] * pitch, 0, c[1] * pitch], yaw); return [x + o[0], z + o[2]]; }); }
  function rigid2(A, B) {     // A を B に重ねる 回転 φ(度)・平行移動。A[i] ↔ B[i]
    var n = A.length, ca = [0, 0], cb = [0, 0];
    A.forEach(function (p, i) { ca[0] += p[0] / n; ca[1] += p[1] / n; cb[0] += B[i][0] / n; cb[1] += B[i][1] / n; });
    var sxx = 0, sxy = 0;
    A.forEach(function (p, i) { var a = [p[0] - ca[0], p[1] - ca[1]], b = [B[i][0] - cb[0], B[i][1] - cb[1]]; sxx += a[0] * b[0] + a[1] * b[1]; sxy += a[0] * b[1] - a[1] * b[0]; });
    var phi = Math.atan2(sxy, sxx), c = Math.cos(phi), s = Math.sin(phi);
    return { phi: phi * 180 / Math.PI, apply: function (p) { var a = [p[0] - ca[0], p[1] - ca[1]]; return [cb[0] + a[0] * c - a[1] * s, cb[1] + a[0] * s + a[1] * c]; } };
  }
  //   マスの組は、形の回しの対称(90° ごとに回して同じ形になる回し方。L・J は 0° だけ、O は 4 通り、I・S・Z は 0° と 180°)が決める組を全部試し、
  //   いちばん遠いマスがいちばん近い組を取る(2026-11-20。隣の会話の列 P の頼み:前は近い順に組んだので、ずれが半マスを越えると組を取り違え、
  //   L を 16mm ずらした見積もりで 80.2mm・+104° を返した)。ずれが半マスより小さければ前と同じ組になる
  function symPerms(shape) {
    var S = PUZZLE.SHAPES[shape], key = function (c) { return c[0] + "," + c[1]; }, out = [];
    for (var k = 0; k < 4; k++) {
      var R = S.map(function (c) { var x = c[0], z = c[1]; for (var r = 0; r < k; r++) { var t = x; x = -z; z = t; } return [x, z]; });
      var mx = Math.min.apply(null, R.map(function (c) { return c[0]; })), mz = Math.min.apply(null, R.map(function (c) { return c[1]; }));
      var sx = Math.min.apply(null, S.map(function (c) { return c[0]; })), sz = Math.min.apply(null, S.map(function (c) { return c[1]; }));
      var idx = {}; S.forEach(function (c, i) { idx[key(c)] = i; });
      var perm = R.map(function (c) { return idx[key([c[0] - mx + sx, c[1] - mz + sz])]; });
      if (perm.every(function (v) { return v !== undefined; })) out.push(perm);
    }
    return out;
  }
  function match(shape, pitch, det, tru) {
    return matchPts(shape, cellsOf(shape, pitch, det[0], det[1], det[2]), cellsOf(shape, pitch, tru[0], tru[1], tru[2]));
  }
  // マスの真ん中の点どうし(cellsOf と同じ順。動かした後の点でよい)で match と同じことをする(plan-cli.js の score の置いた所)
  function matchPts(shape, A, B) {
    var best = null;
    symPerms(shape).forEach(function (perm) {
      var Bm = perm.map(function (j) { return B[j]; }), T = rigid2(A, Bm), worst = 0;
      A.forEach(function (a, i) { worst = Math.max(worst, Math.hypot(a[0] - Bm[i][0], a[1] - Bm[i][1])); });
      if (!best || worst < best.worstCell - 1e-12) best = { T: T, phi: T.phi, worstCell: worst };
    });
    return best;
  }
  // ---- 盤を読む(readBoard。2026-11-20・点検 C49 の 2 と C50 の 3 の続き) ----
  //   上の絵で盤のマス(机より明るい灰色の正方形。world の軸にそろう)の縁を読み、較正のカメラで机の上へ戻した点を、本当の盤に合わせる
  //   平面の回しとずらし T を求める: 本当 = R(yaw)(較正で読んだ点 − about) + about + (dx, dz)。about は盤の真ん中。
  //   カメラの置き方の誤りは、机の上ではほぼ平面の回しとずらしに見えるので、ピースの読みに同じ T を掛ければ「盤に対する位置」になる
  //   (ピースの上の面は盤より高いので、カメラの傾きの誤りの分だけ残る。sim/README の盤を読む)。
  //   画素: 色の濃いもの(ピース)を外し、明るさを机 0・盤 1 の比 a にし、a が −0.5〜1.5 の外(腕・影・紙の線)も外す。
  //   cost = Σ(a − s(T(p)))²。s は盤の縁からの符号付きの距離を画素の大きさで 0〜1 にならした物。ずらし ±BOARD_SRCH_MM を 2mm 刻みで見てから、坂を下る
  var BOARD_SRCH_MM = 14, BOARD_SAT_MAX = 0.2, BOARD_LVL_MIN = 8, BOARD_MIN_EDGE = 200, BOARD_COST_MAX = 0.05;   // COST_MAX【仮】: sim の雑音 3 で 0.012〜0.013 の 4 倍ほど(点検 C51 の 6)
  function readBoard(im, calib, board) {
    var W = im.W, H = im.H, img = SO101.overheadImage(calib.cam, calib.fov || SO101.CAM_FOV.top, W, H);
    var Y = board.y == null ? 0.0003 : board.y, h = board.cell / 2, B = board.cells, M = BOARD_SRCH_MM / 1000 + 0.004;
    var bx0 = Infinity, bx1 = -Infinity, bz0 = Infinity, bz1 = -Infinity;
    B.forEach(function (b) { bx0 = Math.min(bx0, b[0] - h); bx1 = Math.max(bx1, b[0] + h); bz0 = Math.min(bz0, b[1] - h); bz1 = Math.max(bz1, b[1] + h); });
    var about = [(bx0 + bx1) / 2, (bz0 + bz1) / 2], u0 = Infinity, u1 = -Infinity, v0 = Infinity, v1 = -Infinity;
    [[bx0 - M, bz0 - M], [bx1 + M, bz0 - M], [bx1 + M, bz1 + M], [bx0 - M, bz1 + M]].forEach(function (c) {
      var pr = img.project([c[0], Y, c[1]]); if (!pr) return;
      u0 = Math.min(u0, pr[0]); u1 = Math.max(u1, pr[0]); v0 = Math.min(v0, pr[1]); v1 = Math.max(v1, pr[1]);
    });
    u0 = Math.max(0, Math.floor(u0)); v0 = Math.max(0, Math.floor(v0)); u1 = Math.min(W - 1, Math.ceil(u1)); v1 = Math.min(H - 1, Math.ceil(v1));
    if (!(u1 > u0 && v1 > v0)) return { why: "盤が絵に入らない" };
    function sd(x, z) {            // 盤の縁からの符号付きの距離(m。中が正)
      var best = -Infinity;
      for (var k = 0; k < B.length; k++) {
        var ex = Math.abs(x - B[k][0]) - h, ez = Math.abs(z - B[k][1]) - h, d;
        if (ex > 0.02 && ez > 0.02) continue;
        d = ex <= 0 && ez <= 0 ? -Math.max(ex, ez) : -Math.hypot(Math.max(ex, 0), Math.max(ez, 0));
        if (d > best) best = d;
      }
      return best === -Infinity ? -0.02 : best;
    }
    var P = [];
    for (var v = v0; v <= v1; v++) for (var u = u0; u <= u1; u++) {
      var q = v * W + u, r = im.rgb[q * 3], g = im.rgb[q * 3 + 1], b = im.rgb[q * 3 + 2], hv = hsv(r, g, b);
      if (hv[1] > BOARD_SAT_MAX) continue;
      var w = img.unproject(u + 0.5, v + 0.5, Y);
      P.push([w[0], w[2], 0.299 * r + 0.587 * g + 0.114 * b]);
    }
    var pc = img.unproject(W / 2, H / 2, Y), pc1 = img.unproject(W / 2 + 1, H / 2, Y), wpx = Math.hypot(pc1[0] - pc[0], pc1[2] - pc[2]);
    function tf(t) { var c = Math.cos(t[2] * Math.PI / 180), s = Math.sin(t[2] * Math.PI / 180);
      return function (x, z) { x -= about[0]; z -= about[1]; return [c * x - s * z + about[0] + t[0], s * x + c * z + about[1] + t[1]]; }; }
    // 大まかに: 明るさと、盤の中 1・外 0 の相関(明るさの水準・むらに左右されない)がいちばん大きいずらし。盤が机より明るい向きだけ
    var best = null, S = BOARD_SRCH_MM / 1000;
    for (var dx = -S; dx <= S + 1e-9; dx += 0.002) for (var dz = -S; dz <= S + 1e-9; dz += 0.002) {
      var f0 = tf([dx, dz, 0]), n = 0, sL = 0, sF = 0, sLL = 0, sFF = 0, sLF = 0;
      for (var i = 0; i < P.length; i += 2) { var o = f0(P[i][0], P[i][1]), F = sd(o[0], o[1]) > 0 ? 1 : 0, L = P[i][2]; n++; sL += L; sF += F; sLL += L * L; sFF += F * F; sLF += L * F; }
      var cv = sLF / n - sL * sF / n / n, den = Math.sqrt(Math.max(1e-12, (sLL / n - sL * sL / n / n) * (sFF / n - sF * sF / n / n))), cr = cv / den;
      if (!best || cr > best.cr) best = { t: [dx, dz, 0], cr: cr };
    }
    var f1 = tf(best.t), Lin = [], Lout = [];
    P.forEach(function (e) { var o = f1(e[0], e[1]), d0 = sd(o[0], o[1]); if (d0 > 0.003) Lin.push(e[2]); else if (d0 < -0.003 && d0 > -0.015) Lout.push(e[2]); });
    function median(a) { if (!a.length) return NaN; var c = a.slice().sort(function (x, y) { return x - y; }); return c[c.length >> 1]; }
    var Lb = median(Lin), Lt = median(Lout);
    if (!(Lb - Lt >= BOARD_LVL_MIN)) return { why: "盤が机より明るく見えない(盤 " + (+Lb).toFixed(0) + "・机 " + (+Lt).toFixed(0) + "。盤が隠れているか、絵に入らない)" };
    var Q = [];
    P.forEach(function (e) { var a = (e[2] - Lt) / (Lb - Lt); if (a >= -0.5 && a <= 1.5) Q.push([e[0], e[1], a]); });
    function cost(t, sub) {
      var c = Math.cos(t[2] * Math.PI / 180), s = Math.sin(t[2] * Math.PI / 180), sum = 0, n = 0, ne = 0;
      for (var i = 0; i < Q.length; i += (sub || 1)) {
        var x = Q[i][0] - about[0], z = Q[i][1] - about[1], X = c * x - s * z + about[0] + t[0], Z = s * x + c * z + about[1] + t[1];
        var f = Math.max(0, Math.min(1, 0.5 + sd(X, Z) / wpx)), e = Q[i][2] - f; sum += e * e; n++; if (f > 0 && f < 1) ne++;
      }
      return { cost: sum / Math.max(1, n), n: n, nEdge: ne };
    }
    var t = best.t, cur = cost(t).cost;
    [[0.001, 0.5], [0.0005, 0.25], [0.00025, 0.12], [0.00012, 0.06], [0.00006, 0.03]].forEach(function (st) {
      for (var it = 0, moved = true; moved && it < 60; it++) {
        moved = false;
        [[0, st[0]], [1, st[0]], [2, st[1]]].forEach(function (dd) {
          [1, -1].forEach(function (sg) { var t2 = t.slice(); t2[dd[0]] += sg * dd[1]; var c2 = cost(t2).cost; if (c2 < cur - 1e-12) { t = t2; cur = c2; moved = true; } });
        });
      }
    });
    var fin = cost(t);
    if (fin.nEdge < BOARD_MIN_EDGE) return { why: "盤の縁が見える画素が少ない(" + fin.nEdge + " < " + BOARD_MIN_EDGE + ")", nEdge: fin.nEdge };
    if (!(fin.cost <= BOARD_COST_MAX)) return { why: "盤の形が合わない(cost " + fin.cost.toFixed(4) + " > " + BOARD_COST_MAX + "。影・盤の上の物・明るさのむら?)", nEdge: fin.nEdge };
    return { dx: t[0], dz: t[1], yaw: t[2], about: about, cost: fin.cost, n: fin.n, nEdge: fin.nEdge, levels: [Lt, Lb], wpx: wpx,
             edge: Math.abs(t[0]) > S - 0.0005 || Math.abs(t[1]) > S - 0.0005 };
  }
  // 盤を読んだ T を、机の上の点(m)と向き(度)に掛ける
  function boardApply(T, x, z, yaw) {
    var c = Math.cos(T.yaw * Math.PI / 180), s = Math.sin(T.yaw * Math.PI / 180), X = x - T.about[0], Z = z - T.about[1];
    return [c * X - s * Z + T.about[0] + T.dx, s * X + c * Z + T.about[1] + T.dz, yaw + T.yaw];
  }
  // K1/K5 の紙 v2(tools\k5_camera.py sheet。隣の会話の W4)を盤の下に敷いた三角形(2026-11-19)。白い紙(A4 横、手前の端 x=80・真ん中 z=0)に、
  //   10mm の線(灰)・50mm の線(濃い灰)・ArUco の印 8 つ(25mm。中の模様は描かず真っ黒の四角 = 黒がいちばん多い側)・P1〜P11 の十字・150mm の定規 2 本。
  //   線は本物(0.2〜0.5mm)より太く、640×480 の上の絵の 1 画素(約 1.3mm)に必ず写る幅にした(厳しい側)。tag は "table"(影を落とさない)
  function sheetTris() {
    var out = [], Y0 = 0.00005, Y1 = 0.0001, Y2 = 0.00015;
    function quad(x0, z0, x1, z1, y) { return [[x0, y, z0], [x1, y, z0], [x1, y, z1], [x0, y, z0], [x1, y, z1], [x0, y, z1]]; }
    var X0 = 0.080, X1 = 0.290, Z0 = -0.1485, Z1 = 0.1485, LW = 0.0014, LW5 = 0.002, lines = [], dark = [], blk = [], x, z, five, w;
    out.push({ pts: quad(X0, Z0, X1, Z1, Y0), color: [245, 245, 242], tag: "table" });
    for (x = Math.ceil(X0 * 100) / 100; x <= X1 + 1e-9; x += 0.01) { five = Math.round(x * 1000) % 50 === 0; w = five ? LW5 : LW; Array.prototype.push.apply(five ? dark : lines, quad(x - w / 2, Z0, x + w / 2, Z1, Y1)); }
    for (z = Math.ceil(Z0 * 100) / 100; z <= Z1 + 1e-9; z += 0.01) { five = Math.round(Math.abs(z) * 1000) % 50 === 0; w = five ? LW5 : LW; Array.prototype.push.apply(five ? dark : lines, quad(X0, z - w / 2, X1, z + w / 2, Y1)); }
    out.push({ pts: lines, color: [170, 170, 170], tag: "table" }, { pts: dark, color: [90, 90, 90], tag: "table" });
    [0.105, 0.255].forEach(function (mx) { [-0.125, -0.060, 0.060, 0.125].forEach(function (mz) { Array.prototype.push.apply(blk, quad(mx - 0.0125, mz - 0.0125, mx + 0.0125, mz + 0.0125, Y2)); }); });
    [[138.4, -83.6], [168.8, 0], [138.4, 83.6], [169.1, -109.3], [208.8, 0], [169.1, 109.3], [199.7, -135], [248.8, 0], [199.7, 135], [179.8, -51.3], [179.8, 51.3]].forEach(function (c) {
      var px = c[0] / 1000, pz = c[1] / 1000;
      Array.prototype.push.apply(blk, quad(px - 0.006, pz - 0.0007, px + 0.006, pz + 0.0007, Y2));
      Array.prototype.push.apply(blk, quad(px - 0.0007, pz - 0.006, px + 0.0007, pz + 0.006, Y2));
    });
    Array.prototype.push.apply(blk, quad(0.100, -0.0207, 0.250, -0.0193, Y2));     // 定規(x の向き、z = −20)
    Array.prototype.push.apply(blk, quad(0.1243, -0.075, 0.1257, 0.075, Y2));      // 定規(z の向き、x = 125)
    out.push({ pts: blk, color: [20, 20, 20], tag: "table" });
    return out;
  }
  return { worldTris: worldTris, sheetTris: sheetTris, render: render, detect: detect, readBoard: readBoard, boardApply: boardApply, BOARD_SRCH_MM: BOARD_SRCH_MM, fitShape: fitShape, match: match, matchPts: matchPts, cellsOf: cellsOf, rng: rng, gauss: gauss, hex: hex,
           LIGHT: LIGHT, AMB: AMB, DIFF: DIFF, COL: COL, SAT_MIN: SAT_MIN, HUE_TOL: HUE_TOL,
           kelvinGain: kelvinGain, silScore: silScore, SIL_OK: SIL_OK, SIL_BAD: SIL_BAD, AREA_LO: AREA_LO, AREA_HI: AREA_HI, CAST_TOL: CAST_TOL };
})();
if (typeof module !== "undefined") module.exports = VISION;
