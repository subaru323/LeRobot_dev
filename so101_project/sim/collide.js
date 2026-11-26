/* =============================================================================
 * collide.js — 当たり判定(形の表面の点 ⇄ 箱・机)
 *
 * ■ 判定体
 *   腕・爪 … 公式の STL の表面に約 3mm 間隔で打った点(tools/build-models.py が絵と同じ STL から作る)
 *   ピース … 立方体の箱(つなぎの棒は見ていない)。持っているピースは箱の表面の点で動かす
 *   机     … 高さ 0 の平面
 *   上のカメラの台 … STL の三角形そのもの(動かない)。腕の点から面までの距離を、面の向きで外を正・中を負にして測る(meshField)
 *
 * ■ 距離の測り方(近似)
 *   動く側の点から、相手の箱への符号付きの距離(中なら負)を測り、いちばん小さい値をその組の余裕とする。
 *   ★点と点の間は見ていない: 余裕を最大で点の間隔の半分ほど(約 2mm)大きめに出しうる
 *     (当たっていないと出る側に倒れる)。
 *   腕どうしの当たり(自分との干渉)はここでは見ない。self-collision.js(ちくわの丸めた箱の組。隙間を小さめに出す側)で見る。
 * ========================================================================== */

var COLLIDE = (function () {
  "use strict";

  function mT(M, v) {                 // Mᵗ v(M は行優先 3x3)
    return [M[0] * v[0] + M[3] * v[1] + M[6] * v[2],
            M[1] * v[0] + M[4] * v[1] + M[7] * v[2],
            M[2] * v[0] + M[5] * v[1] + M[8] * v[2]];
  }

  /** 箱(半分の大きさ h)の表面の点(箱の座標) */
  function surfacePoints(h, step) {
    var pts = [];
    function grid(n) { var a = []; for (var i = 0; i <= n; i++) a.push(-1 + 2 * i / n); return a; }
    var g = h.map(function (x) { return grid(Math.max(1, Math.ceil(2 * x / step))); });
    for (var ax = 0; ax < 3; ax++) {
      var b = (ax + 1) % 3, c = (ax + 2) % 3;
      [-1, 1].forEach(function (s) {
        g[b].forEach(function (u) {
          g[c].forEach(function (w) {
            var p = [0, 0, 0]; p[ax] = s * h[ax]; p[b] = u * h[b]; p[c] = w * h[c];
            pts.push(p);
          });
        });
      });
    }
    return pts;
  }

  /** 点 p から箱(中心 c・回転 M・半分 h)までの符号付きの距離 */
  function pointBox(p, c, M, h) {
    var q = mT(M, [p[0] - c[0], p[1] - c[1], p[2] - c[2]]);
    var dx = Math.abs(q[0]) - h[0], dy = Math.abs(q[1]) - h[1], dz = Math.abs(q[2]) - h[2];
    var ox = Math.max(dx, 0), oy = Math.max(dy, 0), oz = Math.max(dz, 0);
    var out = Math.sqrt(ox * ox + oy * oy + oz * oz);
    return out > 0 ? out : Math.max(dx, dy, dz);
  }

  // 点 p に いちばん近い三角形 abc の上の点(Ericson「Real-Time Collision Detection」5.1.5)
  function closestOnTri(p, a, b, c) {
    var ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]], ac = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    var ap = [p[0] - a[0], p[1] - a[1], p[2] - a[2]];
    function d(u, v) { return u[0] * v[0] + u[1] * v[1] + u[2] * v[2]; }
    function at(u, v) { return [a[0] + ab[0] * u + ac[0] * v, a[1] + ab[1] * u + ac[1] * v, a[2] + ab[2] * u + ac[2] * v]; }
    var d1 = d(ab, ap), d2 = d(ac, ap);
    if (d1 <= 0 && d2 <= 0) return a;
    var bp = [p[0] - b[0], p[1] - b[1], p[2] - b[2]], d3 = d(ab, bp), d4 = d(ac, bp);
    if (d3 >= 0 && d4 <= d3) return b;
    var vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) return at(d1 / (d1 - d3), 0);
    var cp = [p[0] - c[0], p[1] - c[1], p[2] - c[2]], d5 = d(ab, cp), d6 = d(ac, cp);
    if (d6 >= 0 && d5 <= d6) return c;
    var vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) return at(0, d2 / (d2 - d6));
    var va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
      var w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
      return [b[0] + (c[0] - b[0]) * w, b[1] + (c[1] - b[1]) * w, b[2] + (c[2] - b[2]) * w];
    }
    var den = 1 / (va + vb + vc);
    return at(vb * den, vc * den);
  }

  /**
   * 動かない形(三角形の集まり)。tri … 9 個で 1 つの三角形(整数)、scale 倍して offset を足すと世界の座標。
   * query(p) … 点 p から面までの符号付きの距離(外が正。面の向きは STL の巻き順)。外で maxD より遠い所は Infinity。
   *   ★中の点は、面から maxD より奥でも負で出る(本当の深さ。2026-11-17 の K-03 までは Infinity で抜け落ちていた)。
   *   ★面から 5mm 以内の点は、中か外かを巻き数で決める(面の向きだけでは角の近くで取り違える。点検 C10 の 1)。
   */
  var NEAR_WIND = 0.005;                    // 面からこれより近い点は、中か外かを巻き数で決める[m]
  function meshField(tri, scale, offset, maxD) {
    var n = tri.length / 9, V = new Float64Array(tri.length), Nn = new Float64Array(n * 3);
    var lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
    for (var i = 0; i < tri.length; i++) {
      V[i] = tri[i] * scale + offset[i % 3];
      lo[i % 3] = Math.min(lo[i % 3], V[i]); hi[i % 3] = Math.max(hi[i % 3], V[i]);
    }
    var cells = {}, TB = new Float64Array(n * 6);    // TB = 三角形ごとの箱(lo xyz, hi xyz)。dist で遠い三角形を先に飛ばす
    function key(x, y, z) { return x + "," + y + "," + z; }
    for (var t = 0; t < n; t++) {
      var o = t * 9;
      var e1 = [V[o + 3] - V[o], V[o + 4] - V[o + 1], V[o + 5] - V[o + 2]], e2 = [V[o + 6] - V[o], V[o + 7] - V[o + 1], V[o + 8] - V[o + 2]];
      var nx = e1[1] * e2[2] - e1[2] * e2[1], ny = e1[2] * e2[0] - e1[0] * e2[2], nz = e1[0] * e2[1] - e1[1] * e2[0];
      var nl = Math.hypot(nx, ny, nz);
      if (nl < 1e-12) continue;              // 面積 0 の三角形は距離に使わない(法線が無い。巻き数には 0 で効くので残る)
      Nn[t * 3] = nx / nl; Nn[t * 3 + 1] = ny / nl; Nn[t * 3 + 2] = nz / nl;
      var a0 = [], a1 = [];
      for (var k0 = 0; k0 < 3; k0++) {
        TB[t * 6 + k0] = Math.min(V[o + k0], V[o + 3 + k0], V[o + 6 + k0]);
        TB[t * 6 + 3 + k0] = Math.max(V[o + k0], V[o + 3 + k0], V[o + 6 + k0]);
      }
      for (var k = 0; k < 3; k++) {          // 三角形を maxD だけ太らせた箱が掛かる升に登録 → 問い合わせは点の升 1 つだけ見ればよい
        a0[k] = Math.floor((Math.min(V[o + k], V[o + 3 + k], V[o + 6 + k]) - maxD) / maxD);
        a1[k] = Math.floor((Math.max(V[o + k], V[o + 3 + k], V[o + 6 + k]) + maxD) / maxD);
      }
      for (var x = a0[0]; x <= a1[0]; x++) for (var y = a0[1]; y <= a1[1]; y++) for (var z = a0[2]; z <= a1[2]; z++)
        (cells[key(x, y, z)] || (cells[key(x, y, z)] = [])).push(t);
    }
    function query(p) {
      if (p[0] < lo[0] - maxD || p[0] > hi[0] + maxD || p[1] < lo[1] - maxD || p[1] > hi[1] + maxD ||
          p[2] < lo[2] - maxD || p[2] > hi[2] + maxD) return Infinity;
      var L = cells[key(Math.floor(p[0] / maxD), Math.floor(p[1] / maxD), Math.floor(p[2] / maxD))];
      if (!L) return deepOrFar(p);
      var best = Infinity, sBest = 0;
      for (var j = 0; j < L.length; j++) {
        var o = L[j] * 9;
        var q = closestOnTri(p, [V[o], V[o + 1], V[o + 2]], [V[o + 3], V[o + 4], V[o + 5]], [V[o + 6], V[o + 7], V[o + 8]]);
        var dx = p[0] - q[0], dy = p[1] - q[1], dz = p[2] - q[2], d2 = dx * dx + dy * dy + dz * dz;
        var s = dx * Nn[L[j] * 3] + dy * Nn[L[j] * 3 + 1] + dz * Nn[L[j] * 3 + 2];
        // 同じ近さ(辺・角を分け合う三角形)なら、外向きに出る方を採る(とがった角の外の点を中と取り違えない)
        if (d2 < best - 1e-12 || (Math.abs(d2 - best) <= 1e-12 && s > sBest)) { best = d2; sBest = s; }
      }
      if (best > maxD * maxD) return deepOrFar(p);
      var dist = Math.sqrt(best);
      // 面から NEAR_WIND 以内は、面の向きでは角の近くで中と外を取り違える(点検 C10 の 1)ので、巻き数で決める
      if (dist < NEAR_WIND) return inside(p) ? -dist : dist;
      return sBest < 0 ? -dist : dist;
    }
    // 巻き数(三角形ごとの立体角の和 / 4π。Van Oosterom–Strackee)。閉じた形の中なら 1、外なら 0
    function inside(p) {
      var w = 0;
      for (var t = 0; t < n; t++) {
        var o = t * 9;
        var ax = V[o] - p[0], ay = V[o + 1] - p[1], az = V[o + 2] - p[2];
        var bx = V[o + 3] - p[0], by = V[o + 4] - p[1], bz = V[o + 5] - p[2];
        var cx = V[o + 6] - p[0], cy = V[o + 7] - p[1], cz = V[o + 8] - p[2];
        var la = Math.hypot(ax, ay, az), lb = Math.hypot(bx, by, bz), lc = Math.hypot(cx, cy, cz);
        var det = ax * (by * cz - bz * cy) - ay * (bx * cz - bz * cx) + az * (bx * cy - by * cx);
        var den = la * lb * lc + (ax * bx + ay * by + az * bz) * lc + (bx * cx + by * cy + bz * cz) * la + (cx * ax + cy * ay + cz * az) * lb;
        w += 2 * Math.atan2(det, den);
      }
      return Math.abs(w) / (4 * Math.PI) > 0.5;
    }
    // 向きを付けない距離(maxD より遠ければ Infinity)。中か外かを、いちばん近い点だけ inside で決めたいとき用(S-8。巻き数は重い)
    function dist(p) {
      if (p[0] < lo[0] - maxD || p[0] > hi[0] + maxD || p[1] < lo[1] - maxD || p[1] > hi[1] + maxD ||
          p[2] < lo[2] - maxD || p[2] > hi[2] + maxD) return Infinity;
      var L = cells[key(Math.floor(p[0] / maxD), Math.floor(p[1] / maxD), Math.floor(p[2] / maxD))];
      if (!L) return Infinity;
      var best = Infinity;
      for (var j = 0; j < L.length; j++) {
        var o = L[j] * 9;
        if (Nn[L[j] * 3] === 0 && Nn[L[j] * 3 + 1] === 0 && Nn[L[j] * 3 + 2] === 0) continue;   // 面積 0 の三角形
        var b6 = L[j] * 6, ex = Math.max(TB[b6] - p[0], p[0] - TB[b6 + 3], 0), ey = Math.max(TB[b6 + 1] - p[1], p[1] - TB[b6 + 4], 0),
            ez = Math.max(TB[b6 + 2] - p[2], p[2] - TB[b6 + 5], 0);
        if (ex * ex + ey * ey + ez * ez >= best) continue;   // 三角形の箱までが、いま見つけたいちばん近いより遠い
        var q = closestOnTri(p, [V[o], V[o + 1], V[o + 2]], [V[o + 3], V[o + 4], V[o + 5]], [V[o + 6], V[o + 7], V[o + 8]]);
        var dx = p[0] - q[0], dy = p[1] - q[1], dz = p[2] - q[2], d2 = dx * dx + dy * dy + dz * dz;
        if (d2 < best) best = d2;
      }
      return best > maxD * maxD ? Infinity : Math.sqrt(best);
    }
    // 向き付きの距離を、いちばん近い三角形の面の向きだけで出す(巻き数を使わない速い版。角の近くで中と外を取り違えうるので、
    //   負と出た点は inside で確かめ直して使う。腕どうしの当たり S-10 の下見用)
    function queryFast(p) {
      if (p[0] < lo[0] - maxD || p[0] > hi[0] + maxD || p[1] < lo[1] - maxD || p[1] > hi[1] + maxD ||
          p[2] < lo[2] - maxD || p[2] > hi[2] + maxD) return Infinity;
      var L = cells[key(Math.floor(p[0] / maxD), Math.floor(p[1] / maxD), Math.floor(p[2] / maxD))];
      if (!L) return deepOrFar(p);
      var best = Infinity, sBest = 0;
      for (var j = 0; j < L.length; j++) {
        var t = L[j], o = t * 9;
        if (Nn[t * 3] === 0 && Nn[t * 3 + 1] === 0 && Nn[t * 3 + 2] === 0) continue;
        var b6 = t * 6, ex = Math.max(TB[b6] - p[0], p[0] - TB[b6 + 3], 0), ey = Math.max(TB[b6 + 1] - p[1], p[1] - TB[b6 + 4], 0),
            ez = Math.max(TB[b6 + 2] - p[2], p[2] - TB[b6 + 5], 0);
        if (ex * ex + ey * ey + ez * ez > best + 1e-12) continue;
        var q = closestOnTri(p, [V[o], V[o + 1], V[o + 2]], [V[o + 3], V[o + 4], V[o + 5]], [V[o + 6], V[o + 7], V[o + 8]]);
        var dx = p[0] - q[0], dy = p[1] - q[1], dz = p[2] - q[2], d2 = dx * dx + dy * dy + dz * dz;
        var sg = dx * Nn[t * 3] + dy * Nn[t * 3 + 1] + dz * Nn[t * 3 + 2];
        if (d2 < best - 1e-12 || (Math.abs(d2 - best) <= 1e-12 && sg > sBest)) { best = d2; sBest = sg; }
      }
      if (best > maxD * maxD) return deepOrFar(p);
      return sBest < 0 ? -Math.sqrt(best) : Math.sqrt(best);
    }
    // 面から maxD より遠い点(K-03。2026-11-17 まで、ここは中でも Infinity を返し、深くめり込んだ点が抜け落ちていた)。
    //   外なら Infinity。中なら、全部の三角形までのいちばん近い距離を負で返す(本当の深さ。めり込んだ点にしか呼ばれない)。
    //   中か外か: 三角形の登録が無い升(どの三角形からも maxD より遠い)は、升の中が全部同じなので、升の真ん中の巻き数を覚えておく。
    //   登録のある升の点は、全部の三角形からいちばん近い所を探し、その面の向きで外と出ればそこで止め、中と出たら巻き数で確かめる
    //   (巻き数は三角形が多いと重い。面から maxD より遠い点では、面の向きで外と出たものを信じる。query の 5mm より遠い所と同じ扱い)。
    var cellIn = {};
    function deepOrFar(p) {
      if (p[0] < lo[0] || p[0] > hi[0] || p[1] < lo[1] || p[1] > hi[1] || p[2] < lo[2] || p[2] > hi[2]) return Infinity;
      var ix = Math.floor(p[0] / maxD), iy = Math.floor(p[1] / maxD), iz = Math.floor(p[2] / maxD), kk = key(ix, iy, iz);
      if (cells[kk]) {
        var r = nearestAll(p);
        if (r.s >= 0 || !inside(p)) return Infinity;
        return -r.d;
      }
      if (!(kk in cellIn)) cellIn[kk] = inside([(ix + 0.5) * maxD, (iy + 0.5) * maxD, (iz + 0.5) * maxD]);
      return cellIn[kk] ? -nearestAll(p).d : Infinity;
    }
    // 全部の三角形からいちばん近い所(三角形の箱で遠い物を飛ばす)。d = 距離、s = その面の向きで外が正
    function nearestAll(p) {
      var best = Infinity, sBest = 0;
      for (var t = 0; t < n; t++) {
        if (Nn[t * 3] === 0 && Nn[t * 3 + 1] === 0 && Nn[t * 3 + 2] === 0) continue;
        var b6 = t * 6, ex = Math.max(TB[b6] - p[0], p[0] - TB[b6 + 3], 0), ey = Math.max(TB[b6 + 1] - p[1], p[1] - TB[b6 + 4], 0),
            ez = Math.max(TB[b6 + 2] - p[2], p[2] - TB[b6 + 5], 0);
        if (ex * ex + ey * ey + ez * ez > best + 1e-12) continue;
        var o = t * 9, q = closestOnTri(p, [V[o], V[o + 1], V[o + 2]], [V[o + 3], V[o + 4], V[o + 5]], [V[o + 6], V[o + 7], V[o + 8]]);
        var dx = p[0] - q[0], dy = p[1] - q[1], dz = p[2] - q[2], d2 = dx * dx + dy * dy + dz * dz;
        var sg = dx * Nn[t * 3] + dy * Nn[t * 3 + 1] + dz * Nn[t * 3 + 2];
        if (d2 < best - 1e-12 || (Math.abs(d2 - best) <= 1e-12 && sg > sBest)) { best = d2; sBest = sg; }
      }
      return { d: Math.sqrt(best), s: sBest };
    }
    function distAll(p) { return nearestAll(p).d; }
    return { lo: lo, hi: hi, maxD: maxD, tris: n, query: query, dist: dist, inside: inside, queryFast: queryFast, distAll: distAll };
  }

  /**
   * 1 コマの判定。
   *   movers … [{id, group, c, r, pw: function() → 世界の点の配列, ignoreTable}]
   *             c・r は点を囲む球(遠い相手を飛ばすため)。pw は近い相手があるときだけ呼ぶ
   *   others … [{id, group, c, M, h}]   相手の箱
   *   skip(moverGroup, otherGroup) … true なら測らない(つかんでいる相手など)
   *   tableSkip(moverGroup) … true なら机を測らない
   *   statics … [{id, field}] 動かない形(meshField)。mover の ignoreStatic が true なら測らない。
   *             面から field.maxD より遠ければ、組を書かない
   * 戻り値 { pairs: {"a|b": 余裕[m]}, min, minPair }
   */
  function frame(movers, others, skip, tableSkip, statics) {
    var res = {}, min = Infinity, minPair = "";
    function note(a, b, d) {
      var k = a + "|" + b;
      if (!(k in res) || d < res[k]) res[k] = d;
      if (d < min) { min = d; minPair = k; }
    }
    movers.forEach(function (m) {
      var near = others.filter(function (o) {
        if (o.group === m.group || (skip && skip(m.group, o.group))) return false;
        var rO = Math.hypot(o.h[0], o.h[1], o.h[2]);
        return Math.hypot(o.c[0] - m.c[0], o.c[1] - m.c[1], o.c[2] - m.c[2]) - m.r - rO < 0.02;
      });
      var checkTable = !m.ignoreTable && !(tableSkip && tableSkip(m.group)) && (m.c[1] - m.r < 0.02);
      var nearS = (statics && !m.ignoreStatic) ? statics.filter(function (s) {
        var f = s.field, dx = Math.max(f.lo[0] - m.c[0], 0, m.c[0] - f.hi[0]), dy = Math.max(f.lo[1] - m.c[1], 0, m.c[1] - f.hi[1]),
            dz = Math.max(f.lo[2] - m.c[2], 0, m.c[2] - f.hi[2]);
        return Math.hypot(dx, dy, dz) - m.r < f.maxD;
      }) : [];
      if (!near.length && !checkTable && !nearS.length) return;
      var pw = m.pw(), best = near.map(function () { return Infinity; }), bestT = Infinity;
      var bestS = nearS.map(function () { return Infinity; });
      for (var i = 0; i < pw.length; i++) {
        var p = pw[i];
        if (checkTable && p[1] < bestT) bestT = p[1];
        for (var j = 0; j < near.length; j++) {
          var d = pointBox(p, near[j].c, near[j].M, near[j].h);
          if (d < best[j]) best[j] = d;
        }
        for (var k = 0; k < nearS.length; k++) {
          var ds = nearS[k].field.query(p);
          if (ds < bestS[k]) bestS[k] = ds;
        }
      }
      near.forEach(function (o, j) { note(m.id, o.id, best[j]); });
      nearS.forEach(function (s, k) { if (isFinite(bestS[k])) note(m.id, s.id, bestS[k]); });
      if (checkTable) note(m.id, "机", bestT);
    });
    return { pairs: res, min: min, minPair: minPair };
  }

  return { surfacePoints: surfacePoints, pointBox: pointBox, frame: frame, meshField: meshField, closestOnTri: closestOnTri };
})();

if (typeof module !== "undefined") module.exports = COLLIDE;
