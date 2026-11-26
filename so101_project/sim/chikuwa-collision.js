/* [lerobot sim] ちくわ排除機構 wheel-allviews/collision.js(md5 a2197d6663a4a0d9b6a1d9e06bb2fea7・2026-11-17)の写し。
 * 変えたのは 2 か所だけ: (1) 大域の名前 COLLIDE → CK_COLLIDE(sim/collide.js の COLLIDE とぶつかるため)
 * (2) meshTriangles に src.tris(三角形の配列をそのまま渡す口)を足した。kinematics.js はちくわと同じもの(md5 46d5ed5c…)。
 * 元を直したら、ここも写し直す(中身は書き換えない)。 */
/* =============================================================================
 * collision.js — アームと周辺設備(ホイール・受け台・ツールスタンド)の干渉検知
 *
 * ■ 何をしているか
 *   各腕リンクを「線分+半径(カプセル)」で近似し、周辺設備との最短距離を
 *   全工程の全コマで計算する。半径の合計より近ければ干渉。
 *
 * ■ ★障害物を「カプセル」で近似してはいけないものがある(重要)
 *   カプセルは円柱の上下に半球キャップが付いた形。ホイールのような
 *   「平らな円盤」をカプセルで近似すると、上下に半径ぶんのドームが生えて、
 *   ホイールの上を通過するだけの腕まで干渉と判定してしまう。
 *   実際それで誤検知していた:
 *     ・ホイール上端 930mm に対し、y=1003mm(73mm上)を通る腕を干渉と判定
 *     ・ホイール下端 700mm に対し、y=513mm(187mm下 = 設計どおりの台下退避)を干渉と判定
 *   → 縦置きの円柱状のもの(ホイール・受けピン・ツールスタンド)は
 *     type:"cylinderY" の「有限円柱(上下が平ら)」として扱う。
 *   細長い棒(ベース枠)はキャップの誤差が実質効かないのでカプセルのままでよい。
 *
 * ■ ホイールを「中身の詰まった円盤」として扱うことについて
 *   実際のホイールは中空で、爪はセンターボアを通ってハブ裏へ入る。
 *   しかし安全側に倒して「詰まっている」として見る。掴む/置く動作として
 *   意図的にボアへ入る区間だけを sequence.js 側で nearOk:true と印を付け、
 *   判定から除外する。こうすると「意図してボアを通る」以外の場所で
 *   ホイールに触れたら必ず検出できる。
 * ========================================================================== */

var CK_COLLIDE = (function () {
  "use strict";

  function sub(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
  function add(a, b) { return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]; }
  function scale(a, s) { return [a[0] * s, a[1] * s, a[2] * s]; }
  function dot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
  function len(a) { return Math.sqrt(dot(a, a)); }
  function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }

  // ===========================================================================
  // ★実形状(CADメッシュ)からの当たり判定
  //
  //   リンクを1本のカプセルで近似すると、腕のように曲がった形や、ホイールの
  //   ように中空の皿形をまったく表現できない。実際それが誤差の主因だった:
  //     ・ホイールを中実円柱として扱い、開いている中央部でも当たり判定が出る
  //     ・前腕(bbox 417x293x225)を1本のカプセルにして太らせすぎ
  //   → メッシュの頂点を格子でまとめ、各格子の外接球の集合として持つ。
  //     球なら距離計算が単純で、凹んだ形も中空も素直に表現できる。
  // ===========================================================================

  function b64ToBytes(s) {
    var bin = (typeof atob === "function") ? atob(s)
            : Buffer.from(s, "base64").toString("binary");
    var out = new Uint8Array(bin.length);
    for (var i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
    return out;
  }

  /** mesh.js の1モデルから頂点座標(CAD座標)を復元する */
  function meshVertices(src) {
    var all = [];
    for (var gi = 0; gi < src.groups.length; gi++) {
      var g = src.groups[gi];
      var qp = new Uint16Array(b64ToBytes(g.p).buffer);
      var n = qp.length / 3;
      for (var i = 0; i < n; i++) {
        all.push([g.q[0] + qp[i * 3] * g.q[3],
                  g.q[1] + qp[i * 3 + 1] * g.q[4],
                  g.q[2] + qp[i * 3 + 2] * g.q[5]]);
      }
    }
    return all;
  }


  // ===========================================================================
  // ★判定体を「丸め箱(直方体+丸め半径)の集合」で作る    → 絶対条件12 / K-37
  //
  // ■ なぜ球をやめたか(F-71)
  //   sphereSetFromMesh は「メッシュの★頂点を格子でまとめ、格子ごとに外接球」
  //   だった。これは2つの向きに同時に壊れる:
  //     ・過大 … 球を平らな面に外接させると角で大きくはみ出す。実測 96.8mm。
  //              触れてもいないのに「干渉」と出て、余裕の数値も狂う。
  //     ・過小 … 球は★格子内の頂点しか包まない。大きな平面は内側に頂点が
  //              無いので、面の中央がどの球にも入らない。実測 31.0mm。
  //              ★当たっているのに検出しない = 実機が壊れる方向。
  //
  // ■ 直したところは2つ(利用者判断: 球と直方体の組み合わせにする)
  //   1. 形    … 球 → ★丸め箱。ロボットのリンクは平らな面と丸い角でできて
  //               いるので、箱の芯 + 丸め半径がいちばん素直に乗る。
  //   2. 作り元… 頂点 → ★面(三角形)を面積重みで刻んだ表面点。
  //               面の内側から作るので、★大きな平面に穴が空かない。
  //
  // ■ ★過小が 0 になる理屈(ここが肝。安全側の根拠)
  //   格子ごとの箱は、その格子に落ちた表面点すべての AABB なので、
  //   刻んだ点は必ず箱の中にある。刻んだ点と点のあいだの面は、
  //   点の間隔 spacing より遠くへは行けないので、
  //     丸め半径 margin = spacing * 1.6  (最低 1.5mm)
  //   を全体に付ければ、★面の全体が判定体の中に入る。
  //   → 過小 0 は「測ったら 0 だった」ではなく、作り方から出てくる。
  //     それでも測る(絶対条件12)。作り方の思い込みが外れる可能性があるため。
  //
  // ■ 決定的であること(絶対条件9)
  //   表面の刻みは乱数ではなく★ハルトン列。走らせるたびに答えが動くと
  //   「直ったのか、たまたまか」が区別できない。
  // ===========================================================================

  /** mesh.js の1モデルから三角形(CAD座標)を復元する。
   *  ★頂点ではなく index を使う。頂点だけだと面の内側が作れない(F-71) */
  function meshTriangles(src) {
    if (src.tris) return src.tris;   // [lerobot sim の足し] 三角形の配列 [[a,b,c],...] をそのまま渡す口
    var tris = [];
    for (var gi = 0; gi < src.groups.length; gi++) {
      var g = src.groups[gi];
      var qp = new Uint16Array(b64ToBytes(g.p).buffer);
      var raw = b64ToBytes(g.i);
      var idx = g.i32 ? new Uint32Array(raw.buffer) : new Uint16Array(raw.buffer);
      var q = g.q;
      for (var k = 0; k + 2 < idx.length; k += 3) {
        var t = [];
        for (var m = 0; m < 3; m++) {
          var i3 = idx[k + m] * 3;
          t.push([q[0] + qp[i3] * q[3], q[1] + qp[i3 + 1] * q[4], q[2] + qp[i3 + 2] * q[5]]);
        }
        tris.push(t);
      }
    }
    return tris;
  }

  /** 三角形の表面を面積重みで n 点刻む。★ハルトン列なので決定的 */
  function sampleTriangles(tris, n) {
    var area = new Array(tris.length), total = 0, i;
    for (i = 0; i < tris.length; i++) {
      area[i] = 0.5 * len(KIN.cross(sub(tris[i][1], tris[i][0]), sub(tris[i][2], tris[i][0])));
      total += area[i];
    }
    function hal(k, b) { var f = 1, r = 0; while (k > 0) { f /= b; r += f * (k % b); k = Math.floor(k / b); } return r; }
    var out = new Array(n), acc = 0, ti = 0;
    for (var k2 = 0; k2 < n; k2++) {
      var target = total * (k2 + 0.5) / n;
      while (ti < tris.length - 1 && acc + area[ti] < target) { acc += area[ti]; ti++; }
      var u = hal(k2 + 1, 2), v = hal(k2 + 1, 3);
      if (u + v > 1) { u = 1 - u; v = 1 - v; }
      var t2 = tris[ti];
      out[k2] = [t2[0][0] + u * (t2[1][0] - t2[0][0]) + v * (t2[2][0] - t2[0][0]),
                 t2[0][1] + u * (t2[1][1] - t2[0][1]) + v * (t2[2][1] - t2[0][1]),
                 t2[0][2] + u * (t2[1][2] - t2[0][2]) + v * (t2[2][2] - t2[0][2])];
    }
    return { pts: out, area: total };
  }

  /**
   * ★メッシュを「丸め箱の集合」に変換する。
   *   targetCount … 目標の要素数。多いほど実形状に近いが遅くなる
   *   nSample     … 表面を刻む点数(既定 30000)
   *   戻り値 { parts:[{c,half,r,br}], bs:{c,r}, margin, cell }   すべて CAD 座標
   *     c    … 箱の芯の中心 / half … 芯の半寸法 / r … 丸め半径
   *     br   … その要素の外接球半径(枝刈り用。half の対角 + r)
   *     bs   … 全体の外接球(粗い枝刈り用)
   */
  //   ★cellSize を与えたときは個数ではなく「刻みの一辺」で決める。
  //     過大の上限は個数ではなく刻みで決まるから。実測と式は
  //       過大 ≈ cell*√3/2 + margin   (格子の対角の半分 + 丸め半径)
  //     なので、上限 30mm に対して cell=30mm ならどのリンクも通る。
  //     ★個数を固定すると、大きいリンクだけ粗くなって落ちる
  //     (実測: 目標900個で J2 だけ 31.0mm で不合格、他は全部合格)。
  // ===========================================================================
  // ★★三角形そのものから箱を作る(2026-09-20。K-103 / 条件12)
  //   ■ なぜ
  //     いままでは表面を**サンプル点**で刻み、点の AABB を箱にしていた。
  //     ★点と点の間の面を取りこぼさないために ★**丸め半径 margin = 点間隔×1.6** を全部に乗せていた
  //     (ワーク 8mm 刻みで **3.6mm**)。★刻みを細かくしてもこの下駄は消えない = 過大の床(実測 11.9 → 6.7mm で頭打ち)。
  //   ■ どう変えるか
  //     セルに掛かる ★**三角形をセルの箱で切り取り(厳密)**、その切れ端の AABB を箱にする。
  //     ★**面はまるごと箱の中に入る**ので、サンプル誤差のための丸めが要らない = ★過小 0 が作り方から出る。
  //   ■ ★見ていないもの(0-K): 箱の向き(まだ軸に平行 = 斜め面の角の膨らみは残る。★段B で OBB にする)
  /** 凸多角形を軸に平行な箱で切り取る(Sutherland–Hodgman) */
  function clipPolyHalf(poly, axis, sgn, val) {
    var out = [], n = poly.length;
    for (var i = 0; i < n; i++) {
      var P = poly[i], Q = poly[(i + 1) % n];
      var dp = sgn * (P[axis] - val), dq = sgn * (Q[axis] - val);
      var inP = dp >= 0, inQ = dq >= 0;
      if (inP) out.push(P);
      if (inP !== inQ) {
        var t = dp / (dp - dq);
        out.push([P[0] + (Q[0] - P[0]) * t, P[1] + (Q[1] - P[1]) * t, P[2] + (Q[2] - P[2]) * t]);
      }
    }
    return out;
  }
  function clipPolyBox(poly, lo, hi) {
    for (var a = 0; a < 3 && poly.length; a++) {
      poly = clipPolyHalf(poly, a, 1, lo[a]);
      if (!poly.length) break;
      poly = clipPolyHalf(poly, a, -1, hi[a]);
    }
    return poly;
  }
  // ★★セルの箱を「面に沿わせる」(2026-09-20。条件12 段B)
  //   ■ なぜ
  //     軸に平行な箱(AABB)は、★**斜めの面**が入ると角が大きく膨らむ(8mm セルで最大 ~7mm)。
  //     面に沿った薄い板で包めば、膨らみは**板の厚みの半分**まで落ちる。
  //   ■ 向きの決め方(★推定ではなく、三角形そのものから)
  //     セルに掛かった切れ端の ★**面積で重みを付けた法線の和**。平面なら厳密。
  //     ★**和が小さいセル**(表と裏が同じセルに入って打ち消し合う = 薄いリブ等)は
  //     「面が 1 枚ではない」ので ★**傾けない**(AABB のまま)。★自分で効かない場所を避ける
  var ORIENT_MIN = 0.55;        // ★法線の和 ÷ 面積の和。これ未満は「面が 1 枚ではない」とみなす
  function frameFromNormal(n) {
    var a = (Math.abs(n[0]) < 0.9) ? [1, 0, 0] : [0, 1, 0];
    var u1 = [n[1]*a[2] - n[2]*a[1], n[2]*a[0] - n[0]*a[2], n[0]*a[1] - n[1]*a[0]];
    var L1 = Math.sqrt(u1[0]*u1[0] + u1[1]*u1[1] + u1[2]*u1[2]) || 1;
    u1 = [u1[0]/L1, u1[1]/L1, u1[2]/L1];
    var u2 = [n[1]*u1[2] - n[2]*u1[1], n[2]*u1[0] - n[0]*u1[2], n[0]*u1[1] - n[1]*u1[0]];
    // ★列が箱の軸(局所 → リンク)。pointToRBox / clampToBox は列を軸として読む
    return [u1[0], u2[0], n[0],  u1[1], u2[1], n[1],  u1[2], u2[2], n[2]];
  }
  /** ★三角形から作る版。戻りは bodySetFromMesh と同じ形({parts, bs, margin, cell, ...}) */
  function bodySetFromMeshExact(src, cellSize, roundMm) {
    var tris = meshTriangles(src);
    var cell = cellSize > 0 ? cellSize : 0.008;
    var margin = (roundMm === undefined ? 0.5 : roundMm) / 1000;   // ★丸めは数値の安全代だけ
    var cells = {}, keys = [];
    for (var ti = 0; ti < tris.length; ti++) {
      var t = tris[ti];
      var lo = [Math.min(t[0][0], t[1][0], t[2][0]), Math.min(t[0][1], t[1][1], t[2][1]), Math.min(t[0][2], t[1][2], t[2][2])];
      var hi = [Math.max(t[0][0], t[1][0], t[2][0]), Math.max(t[0][1], t[1][1], t[2][1]), Math.max(t[0][2], t[1][2], t[2][2])];
      var i0 = Math.floor(lo[0] / cell), i1 = Math.floor(hi[0] / cell);
      var j0 = Math.floor(lo[1] / cell), j1 = Math.floor(hi[1] / cell);
      var k0 = Math.floor(lo[2] / cell), k1 = Math.floor(hi[2] / cell);
      for (var i = i0; i <= i1; i++) for (var j = j0; j <= j1; j++) for (var k = k0; k <= k1; k++) {
        var cl = [i * cell, j * cell, k * cell], ch = [(i + 1) * cell, (j + 1) * cell, (k + 1) * cell];
        var poly = clipPolyBox([t[0], t[1], t[2]], cl, ch);
        if (!poly.length) continue;
        var key = i + ',' + j + ',' + k, c = cells[key];
        if (!c) { cells[key] = c = { pts: [], nx: 0, ny: 0, nz: 0, area: 0 }; keys.push(key); }
        for (var pi = 0; pi < poly.length; pi++) c.pts.push(poly[pi]);
        // ★面積で重みを付けた法線を貯める(扇形に割って三角形の外積を足す)
        for (var fi = 1; fi + 1 < poly.length; fi++) {
          var e1 = [poly[fi][0]-poly[0][0], poly[fi][1]-poly[0][1], poly[fi][2]-poly[0][2]];
          var e2 = [poly[fi+1][0]-poly[0][0], poly[fi+1][1]-poly[0][1], poly[fi+1][2]-poly[0][2]];
          var cx = e1[1]*e2[2] - e1[2]*e2[1], cy = e1[2]*e2[0] - e1[0]*e2[2], cz = e1[0]*e2[1] - e1[1]*e2[0];
          c.nx += cx / 2; c.ny += cy / 2; c.nz += cz / 2;
          c.area += Math.sqrt(cx*cx + cy*cy + cz*cz) / 2;
        }
      }
    }
    var parts = [], nOriented = 0;
    var orient = (typeof BODY_ORIENT === "undefined") ? true : !!BODY_ORIENT;   // ★戻す口(条件13 の比べ方)
    for (var qi = 0; qi < keys.length; qi++) {
      var c2 = cells[keys[qi]], P = c2.pts;
      var nl = Math.sqrt(c2.nx*c2.nx + c2.ny*c2.ny + c2.nz*c2.nz);
      var R0 = null;
      if (orient && c2.area > 0 && nl / c2.area >= ORIENT_MIN) R0 = frameFromNormal([c2.nx/nl, c2.ny/nl, c2.nz/nl]);
      var plo, phi, ax, ctr, half;   // ★lo/hi は上の三角形の箱で使っている(F-184: 同じ関数で 2 回宣言しない)
      if (R0) {
        // ★局所の軸(R0 の列)で測る
        ax = [[R0[0], R0[3], R0[6]], [R0[1], R0[4], R0[7]], [R0[2], R0[5], R0[8]]];
        plo = [Infinity, Infinity, Infinity]; phi = [-Infinity, -Infinity, -Infinity];
        for (var pj = 0; pj < P.length; pj++) for (var a2 = 0; a2 < 3; a2++) {
          var t2 = P[pj][0]*ax[a2][0] + P[pj][1]*ax[a2][1] + P[pj][2]*ax[a2][2];
          if (t2 < plo[a2]) plo[a2] = t2; if (t2 > phi[a2]) phi[a2] = t2;
        }
        var mid = [(plo[0]+phi[0])/2, (plo[1]+phi[1])/2, (plo[2]+phi[2])/2];
        ctr = [mid[0]*ax[0][0] + mid[1]*ax[1][0] + mid[2]*ax[2][0],
               mid[0]*ax[0][1] + mid[1]*ax[1][1] + mid[2]*ax[2][1],
               mid[0]*ax[0][2] + mid[1]*ax[1][2] + mid[2]*ax[2][2]];
        half = [(phi[0]-plo[0])/2, (phi[1]-plo[1])/2, (phi[2]-plo[2])/2];
        nOriented++;
      } else {
        plo = [Infinity, Infinity, Infinity]; phi = [-Infinity, -Infinity, -Infinity];
        for (var pk = 0; pk < P.length; pk++) for (var a3 = 0; a3 < 3; a3++) {
          var v3 = P[pk][a3]; if (v3 < plo[a3]) plo[a3] = v3; if (v3 > phi[a3]) phi[a3] = v3;
        }
        ctr = [(plo[0]+phi[0])/2, (plo[1]+phi[1])/2, (plo[2]+phi[2])/2];
        half = [(phi[0]-plo[0])/2, (phi[1]-plo[1])/2, (phi[2]-plo[2])/2];
      }
      var pt = { c: ctr, half: half, r: margin,
                 br: Math.sqrt(half[0]*half[0] + half[1]*half[1] + half[2]*half[2]) + margin };
      if (R0) pt.R0 = R0;
      parts.push(pt);
    }
    var bb = src.bbox;
    var bc = [(bb[0] + bb[3]) / 2, (bb[1] + bb[4]) / 2, (bb[2] + bb[5]) / 2];
    var br = 0.5 * Math.sqrt(Math.pow(bb[3]-bb[0], 2) + Math.pow(bb[4]-bb[1], 2) + Math.pow(bb[5]-bb[2], 2)) + margin;
    // ★かたまり(枝刈り用)は従来と同じ作り方・同じ粗さ(条件9: 作り方が場所で違わない)
    return { parts: parts, bs: { c: bc, r: br }, margin: margin, cell: cell, exact: true,
             oriented: nOriented,          // ★面に沿わせた要素の数(残りは AABB のまま)
             clusters: clusterParts(parts, cell * 6) };
  }
  function bodySetFromMesh(src, targetCount, nSample, cellSize) {
    var tris = meshTriangles(src);
    var N = nSample || 30000;
    var sm = sampleTriangles(tris, N);
    var pts = sm.pts;
    var bb = src.bbox;
    var size = Math.max(bb[3] - bb[0], bb[4] - bb[1], bb[5] - bb[2]);
    // ★刻みの粗さ = 表面点どうしの間隔の目安。これを丸め半径の元にすると、
    //   点と点のあいだの面も必ず判定体の中に入る = ★過小が出ない
    var spacing = Math.sqrt(sm.area / N);
    var margin = Math.max(0.0015, spacing * 1.6);
    var cell = (cellSize > 0) ? cellSize : size / 4, best = null;
    var passes = (cellSize > 0) ? 1 : 12;
    for (var pass = 0; pass < passes; pass++) {
      var cells = {}, keys = [];
      for (var j = 0; j < pts.length; j++) {
        var v = pts[j];
        var k = Math.floor(v[0] / cell) + "," + Math.floor(v[1] / cell) + "," + Math.floor(v[2] / cell);
        var c = cells[k];
        if (!c) { cells[k] = [v[0], v[1], v[2], v[0], v[1], v[2]]; keys.push(k); }
        else {
          if (v[0] < c[0]) c[0] = v[0]; if (v[1] < c[1]) c[1] = v[1]; if (v[2] < c[2]) c[2] = v[2];
          if (v[0] > c[3]) c[3] = v[0]; if (v[1] > c[4]) c[4] = v[1]; if (v[2] > c[5]) c[5] = v[2];
        }
      }
      best = { cells: cells, keys: keys };
      if (cellSize > 0) break;
      if (keys.length >= targetCount * 0.7 && keys.length <= targetCount * 1.5) break;
      cell *= (keys.length < targetCount) ? 0.78 : 1.30;
    }
    var parts = [];
    for (var qi = 0; qi < best.keys.length; qi++) {
      var c2 = best.cells[best.keys[qi]];
      var ctr = [(c2[0] + c2[3]) / 2, (c2[1] + c2[4]) / 2, (c2[2] + c2[5]) / 2];
      // ★芯は「その格子に落ちた表面点の AABB」そのまま。縮めない。
      //   一度 margin だけ縮めてみたが、AABB の角の点までの距離が
      //   margin*√3 になるので丸め半径 margin では包みきれず、
      //   ★過小が 3.2mm 出た(実測)。過小は安全側でない方への壊れ方なので、
      //   平らな面で margin ぶん太ることを受け入れて芯を縮めない。
      //   → 刻んだ点は必ず芯の中(距離 0) = ★過小 0 が作り方から出てくる。
      var half = [(c2[3] - c2[0]) / 2, (c2[4] - c2[1]) / 2, (c2[5] - c2[2]) / 2];
      parts.push({ c: ctr, half: half, r: margin,
                   br: Math.sqrt(half[0]*half[0] + half[1]*half[1] + half[2]*half[2]) + margin });
    }
    var bc = [(bb[0] + bb[3]) / 2, (bb[1] + bb[4]) / 2, (bb[2] + bb[5]) / 2];
    var br = 0.5 * Math.sqrt(Math.pow(bb[3]-bb[0], 2) + Math.pow(bb[4]-bb[1], 2) +
                             Math.pow(bb[5]-bb[2], 2)) + margin;
    // ★枝刈り用の「かたまり」をここで作っておく。
    //   判定のたびに作ると遼いし、作り方が場所で違うと結論が場所で違う(絶対条件9)。
    return { parts: parts, bs: { c: bc, r: br }, margin: margin, cell: cell,
             // ★かたまりの粗さは実測で決めた(J2×J4 ・ 3場面の合計[ms]):
             //   x2 22.40 / x3 4.30 / x4 1.90 / ★x6 1.47 / x8 3.15 / x12 6.65
             //   細かすぎると対の数が爆発し、粗すぎると枝刈りが効かない。
             //   ★ここは速さだけの値で、答えは変わらない(parts-selftest で確認)。
             clusters: clusterParts(parts, cell * 6) };
  }

  /**
   * ★探索用の「粗い判定体」を、細かい判定体**から**作る。
   *
   *   ■ なぜメッシュから作り直さないか
   *     作り直すと、粗い版も細かい版も「メッシュを包む」だけで、
   *     ★「粗い版が細かい版を包む」は成り立たない。
   *     格子が入れ子でないので、境界をまたぐ細かい箱がどの粗い箱からも
   *     はみ出し、そこだけ細かい版のほうが太くなる。
   *     実測: 粗いほうが「余裕がある」と言う姿勢が出た(最大 +11.98mm)。
   *     = 探索が「安全」と言って本番が「干渉」と言う = ★見逃しの向き。
   *
   *   ■ ここでは細かい箱をまとめ、**そのグループを丸ごと包む箱**にする。
   *     → 粗い ⊇ 細かい が★構造で保証される。測って確かめる話ではなくなる。
   *
   *   cellSize … まとめる格子の一辺[m]。大きいほど速く、粗い
   */
  function coarsenBodySet(set, cellSize) {
    var map = {}, keys = [], i, a;
    for (i = 0; i < set.parts.length; i++) {
      var e = set.parts[i], c = e.c;
      var k = Math.floor(c[0] / cellSize) + "," +
              Math.floor(c[1] / cellSize) + "," +
              Math.floor(c[2] / cellSize);
      if (!map[k]) { map[k] = []; keys.push(k); }
      map[k].push(i);
    }
    var parts = [];
    for (var q = 0; q < keys.length; q++) {
      var idx = map[keys[q]];
      var lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
      for (i = 0; i < idx.length; i++) {
        var p2 = set.parts[idx[i]];
        // ★丸め半径まで含めて包む。ここを忘れると r のぶんだけ細くなる
        for (a = 0; a < 3; a++) {
          var loA = p2.c[a] - p2.half[a] - p2.r;
          var hiA = p2.c[a] + p2.half[a] + p2.r;
          if (loA < lo[a]) lo[a] = loA;
          if (hiA > hi[a]) hi[a] = hiA;
        }
      }
      var ctr = [(lo[0]+hi[0])/2, (lo[1]+hi[1])/2, (lo[2]+hi[2])/2];
      var half = [(hi[0]-lo[0])/2, (hi[1]-lo[1])/2, (hi[2]-lo[2])/2];
      // ★丸めは 0。半径ぶんはすでに half に含めた(二重に足すと無駄に太る)
      parts.push({ c: ctr, half: half, r: 0,
                   br: Math.sqrt(half[0]*half[0] + half[1]*half[1] + half[2]*half[2]) });
    }
    return { parts: parts, bs: set.bs, margin: 0, cell: cellSize,
             clusters: clusterParts(parts, cellSize * 6) };
  }

  /** 丸め箱の集合を剛体変換する。★向き R は集合で共通なので1つだけ持つ
   *  (箱ごとに持つと、対ごとに行列を引き回すことになって遅い) */
  function transformParts(set, R, p, o, frameOff) {
    var out = new Array(set.parts.length);
    for (var i = 0; i < set.parts.length; i++) {
      var s2 = set.parts[i];
      var v = R ? KIN.matVec(R, sub(s2.c, o)) : sub(s2.c, o);
      out[i] = { c: [v[0]+p[0]+frameOff[0], v[1]+p[1]+frameOff[1], v[2]+p[2]+frameOff[2]],
                 half: s2.half, r: s2.r, br: s2.br };
      // ★★部品が自分の向き R0(リンクの中での向き)を持っていれば、世界の向きを作って渡す(条件12 段B。2026-09-20)
      //   ★持っていない部品は R を付けない = 呼ぶ側はリンクの回転を使う = ★答えは変わらない(条件13)
      if (s2.R0) out[i].R = R ? KIN.matMul(R, s2.R0) : s2.R0;
    }
    var bv = R ? KIN.matVec(R, sub(set.bs.c, o)) : sub(set.bs.c, o);
    return { parts: out, R: R || null,
             clusters: transformClusters(set.clusters, R, p, o, frameOff),
             bs: { c: [bv[0]+p[0]+frameOff[0], bv[1]+p[1]+frameOff[1], bv[2]+p[2]+frameOff[2]],
                   r: set.bs.r } };
  }

  /** ワールド点 p を、箱の芯(丸めを含まない直方体)の上に落とす */
  function clampToBox(p, c, R, h) {
    var dx = p[0]-c[0], dy = p[1]-c[1], dz = p[2]-c[2];
    var lx, ly, lz;
    if (R) { lx = R[0]*dx + R[3]*dy + R[6]*dz;
             ly = R[1]*dx + R[4]*dy + R[7]*dz;
             lz = R[2]*dx + R[5]*dy + R[8]*dz; }
    else   { lx = dx; ly = dy; lz = dz; }
    if (lx >  h[0]) lx =  h[0]; else if (lx < -h[0]) lx = -h[0];
    if (ly >  h[1]) ly =  h[1]; else if (ly < -h[1]) ly = -h[1];
    if (lz >  h[2]) lz =  h[2]; else if (lz < -h[2]) lz = -h[2];
    if (!R) return [c[0]+lx, c[1]+ly, c[2]+lz];
    return [c[0] + R[0]*lx + R[1]*ly + R[2]*lz,
            c[1] + R[3]*lx + R[4]*ly + R[5]*lz,
            c[2] + R[6]*lx + R[7]*ly + R[8]*lz];
  }

  /** 点 p から丸め箱の表面までの距離(内側は負)。これが「点→判定体」の基本形 */
  function pointToRBox(p, e, R) {
    if (e.R) R = e.R;               // ★部品が自分の向きを持っていればそれを使う(条件12 段B)
    var dx = p[0]-e.c[0], dy = p[1]-e.c[1], dz = p[2]-e.c[2];
    var lx, ly, lz;
    if (R) { lx = R[0]*dx + R[3]*dy + R[6]*dz;
             ly = R[1]*dx + R[4]*dy + R[7]*dz;
             lz = R[2]*dx + R[5]*dy + R[8]*dz; }
    else   { lx = dx; ly = dy; lz = dz; }
    var h = e.half;
    var qx = Math.abs(lx) - h[0], qy = Math.abs(ly) - h[1], qz = Math.abs(lz) - h[2];
    var ex = Math.max(0, qx), ey = Math.max(0, qy), ez = Math.max(0, qz);
    var outside = Math.sqrt(ex*ex + ey*ey + ez*ez);
    if (outside > 0) return outside - e.r;
    // 芯の内側。いちばん近い面までの距離を負で返す(F-51 と同じ理由で 0 で切らない)
    return Math.max(qx, Math.max(qy, qz)) - e.r;
  }

  /** 箱の代表点(中心・面心・角)。食い込んだときの深さを測るのに使う */
  function boxProbePoints(c, R, h) {
    var out = [c];
    var L = [[h[0],0,0],[-h[0],0,0],[0,h[1],0],[0,-h[1],0],[0,0,h[2]],[0,0,-h[2]]];
    for (var s0 = -1; s0 <= 1; s0 += 2) for (var s1 = -1; s1 <= 1; s1 += 2) for (var s2 = -1; s2 <= 1; s2 += 2)
      L.push([s0*h[0], s1*h[1], s2*h[2]]);
    for (var i = 0; i < L.length; i++) {
      var l = L[i];
      if (!R) { out.push([c[0]+l[0], c[1]+l[1], c[2]+l[2]]); continue; }
      out.push([c[0] + R[0]*l[0] + R[1]*l[1] + R[2]*l[2],
                c[1] + R[3]*l[0] + R[4]*l[1] + R[5]*l[2],
                c[2] + R[6]*l[0] + R[7]*l[1] + R[8]*l[2]]);
    }
    return out;
  }

  /** 分離軸(15軸)による、箱の芯どうしの食い込み量[m]。0 なら離れている */
  function obbPenetration(ca, Ra, ha, cb, Rb, hb) {
    var I = [1,0,0, 0,1,0, 0,0,1];
    var A = Ra || I, B = Rb || I;
    var d = [cb[0]-ca[0], cb[1]-ca[1], cb[2]-ca[2]];
    var axes = [], i, j;
    for (i = 0; i < 3; i++) axes.push([A[i], A[i+3], A[i+6]]);
    for (j = 0; j < 3; j++) axes.push([B[j], B[j+3], B[j+6]]);
    for (i = 0; i < 3; i++) for (j = 0; j < 3; j++) {
      var cx = KIN.cross([A[i], A[i+3], A[i+6]], [B[j], B[j+3], B[j+6]]);
      if (len(cx) > 1e-6) axes.push(KIN.unit(cx));
    }
    var best = Infinity;
    for (var k = 0; k < axes.length; k++) {
      var L = axes[k], rA = 0, rB = 0;
      for (var m = 0; m < 3; m++) {
        rA += ha[m] * Math.abs(A[m]*L[0] + A[m+3]*L[1] + A[m+6]*L[2]);
        rB += hb[m] * Math.abs(B[m]*L[0] + B[m+3]*L[1] + B[m+6]*L[2]);
      }
      var dd = Math.abs(d[0]*L[0] + d[1]*L[1] + d[2]*L[2]);
      var ov = rA + rB - dd;
      if (ov <= 0) return 0;                 // 1軸でも離れていれば芯は当たっていない
      if (ov < best) best = ov;
    }
    return best;
  }

  /**
   * 丸め箱どうしの隙間。
   *   離れているとき … 交互射影で両方の芯の最近点まで降りる(厳密)
   *   食い込んだとき … 分離軸(芯どうしなら厳密)で深さを出す
   */
  function rbPair(a, Ra, b, Rb) {
    // ★★部品が自分の向き(R)を持っていれば、それを使う(2026-09-20。条件12 段B)
    //   ★`transformParts` が「リンクの回転 × 部品の向き」を掛けて世界座標の R を入れてある。
    //   ★持っていない部品は今までどおりリンクの回転だけ = ★**答えは変わらない**(条件13)
    if (a.R) Ra = a.R;
    if (b.R) Rb = b.R;
    var pa = a.c, pb = b.c, prev = Infinity, dd = 0;
    for (var k = 0; k < 16; k++) {
      pb = clampToBox(pa, b.c, Rb, b.half);
      pa = clampToBox(pb, a.c, Ra, a.half);
      dd = len(sub(pa, pb));
      if (prev - dd < 1e-7) break;
      prev = dd;
    }
    if (dd > 1e-9) return { clearance: dd - a.r - b.r,
                            mid: [(pa[0]+pb[0])/2, (pa[1]+pb[1])/2, (pa[2]+pb[2])/2] };
    var pen = obbPenetration(a.c, Ra, a.half, b.c, Rb, b.half);
    return { clearance: -(pen + a.r + b.r), mid: pa };
  }

  // ===========================================================================
  // ★丸め箱の集合どうしを、実用的な速さで比べる      → K-37 / K-29
  //
  // ■ なぜ要るか
  //   球 40個/リンクを丸め箱 約450個/リンクに増やした(刻み22mm)。
  //   総当たりだと 1リンク対あたり 450x450 = 20万回。
  //   ★1コマあたり 300万回になり、ビルドが数時間になって使えない。
  //
  // ■ どうしたか — 2段の外接球で枝刈りする
  //   1段目 … 集合全体の外接球(bs)
  //   2段目 … 近くの箱を 16個ずつまとめた「かたまり」の外接球(clusters)
  //   3段目 … 箱1個ごとの外接球(br)
  //   どの段も「外接球どうしの距離」は★必ず本当の距離以下なので、
  //   これが今の最小値を超えたらその先は見なくてよい(取りこぼさない)。
  //
  // ■ ★枝刈りは「速くする」だけで、答えを変えてはいけない
  //   外接球で切るのは下限による枝刈りなので、答えは総当たりと一致する。
  //   一致することは obb-selftest / rigid-selftest で確かめる(絶対条件8)。
  // ===========================================================================

  /** 丸め箱を粗い格子でまとめ、かたまりごとの外接球を作る(枝刈り用) */
  function clusterParts(parts, cell) {
    var map = {}, keys = [], i;
    for (i = 0; i < parts.length; i++) {
      var c = parts[i].c;
      var k = Math.floor(c[0] / cell) + "," + Math.floor(c[1] / cell) + "," + Math.floor(c[2] / cell);
      if (!map[k]) { map[k] = []; keys.push(k); }
      map[k].push(i);
    }
    var out = [];
    for (var q = 0; q < keys.length; q++) {
      var idx = map[keys[q]];
      var lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
      for (i = 0; i < idx.length; i++) {
        var e = parts[idx[i]];
        for (var a = 0; a < 3; a++) {
          if (e.c[a] - e.br < lo[a]) lo[a] = e.c[a] - e.br;
          if (e.c[a] + e.br > hi[a]) hi[a] = e.c[a] + e.br;
        }
      }
      var ctr = [(lo[0]+hi[0])/2, (lo[1]+hi[1])/2, (lo[2]+hi[2])/2];
      var rad = 0;
      for (i = 0; i < idx.length; i++) {
        var e2 = parts[idx[i]];
        var d = len(sub(e2.c, ctr)) + e2.br;
        if (d > rad) rad = d;
      }
      out.push({ c: ctr, r: rad, idx: idx });
    }
    return out;
  }

  /** かたまりの外接球を剛体変換する(中身の並びは変わらないので idx はそのまま) */
  function transformClusters(cl, R, p, o, frameOff) {
    var out = new Array(cl.length);
    for (var i = 0; i < cl.length; i++) {
      var v = R ? KIN.matVec(R, sub(cl[i].c, o)) : sub(cl[i].c, o);
      out[i] = { c: [v[0]+p[0]+frameOff[0], v[1]+p[1]+frameOff[1], v[2]+p[2]+frameOff[2]],
                 r: cl[i].r, idx: cl[i].idx };
    }
    return out;
  }
  // ★かたまり対の下限を入れておく作業用の配列。呼ぶたびに作らない。
  //   partsPartsClearance は再帰しないので使い回してよい。
  //   ★以前は毎回 {lb,i,j} を 784個作って sort していた。
  //     答えは同じだが、確保と比較関数呼び出しだけで 1対 3ms かかっていた。
  var _lb = new Float64Array(1024);
  function lbBuf(n) {
    if (_lb.length < n) _lb = new Float64Array(n * 2);
    return _lb;
  }
  // ★Q-11(2026-09-19): 行(A のかたまり)・列(B のかたまり)ごとの下限と、生き残った列の番号。
  var _rowLb = new Float64Array(256), _colLb = new Float64Array(1024), _colIx = new Int32Array(1024);
  function rowBuf(n) { if (_rowLb.length < n) _rowLb = new Float64Array(n * 2); return _rowLb; }
  function colBuf(n) { if (_colLb.length < n) { _colLb = new Float64Array(n * 2); _colIx = new Int32Array(n * 2); } return _colLb; }
  // ★浮動小数の丸めを吸収する余白[m]。1e-12 m = 1 pm。
  //   「この対は見なくてよい」と言うときだけ足す(= 見る側に倒す)。
  //   ★答えを決める比較(lb >= best / lb < mlb)には足さない = 旧実装と同じ式・同じ判断。
  var PP_EPS = 1e-12;
  var _alive = new Uint8Array(1024);
  function aliveBuf(n) { if (_alive.length < n) _alive = new Uint8Array(n * 2); return _alive; }

  /**
   * 丸め箱の集合どうしの最小隙間。★枝刈りしても答えは総当たりと同じ。
   *   A, B … { parts, R, clusters, bs }
   *
   * ■ 手順(すべて「下限による枝刈り」なので取りこぼさない)
   *   1. 集合全体の外接球で落とす
   *   2. かたまり対の下限を出し、★いちばん近い対を先に開く
   *      → 小さい best を早く掴むほど、あとの枝刈りがよく効く
   *   3. 残りは下限が best 以上なら見ない
   *   ★並べ替えはしない。「最小を1つ選んで先に開く」だけで足りる。
   *
   * ■ ★Q-11(2026-09-19): 「かたまり対の下限を全部出す」をやめた
   *   以前は na×nb 対すべてに √ を取っていた(焼きの自己時間の 62%。§8-5 の実測)。
   *   実際に開くのは数対なのに、順番を決めるためだけに 1 万回級の √ を毎回払っていた。
   *   ★いまは 3 段で落とす:
   *     (a) 行の下限  rowLb[i] = |ci − cB| − ri − rcB   (rcB = B のかたまりを全部包む半径)
   *         → 行 i のどの対も lb(i,j) ≥ rowLb[i]。行ごと飛ばせる
   *     (b) 列の下限  colLb[j] = |cj − cA| − rj − rcA    → 列ごと飛ばせる
   *     (c) 対は平方距離で先に比べ、√ は「best を下回りうる対」だけ
   *   ★rcA / rcB を bs.r ではなく「かたまりの球を全部包む半径」にするのは、
   *     かたまりの球が bs から**はみ出しうる**から(かたまりは箱の球の外接、bs は bbox の外接)。
   *     はみ出しを無視すると (a)(b) が下限でなくなる = 取りこぼす。
   *
   *   ■ ★答えが 1 ビットも変わらない理由(条件13。parts-selftest と bake-diff で示す)
   *     旧実装の「開く順」= [いちばん近い対] → [行優先で lb < best の対]。
   *     新実装も**同じ順で同じ対を開く**:
   *       ・飛ばすのは「lb > 境界 + PP_EPS が確実な対」だけ。旧実装もその対は開かない
   *         (lb >= best で飛ばす)か、開いても best を縮められない(真の距離 ≥ lb > best)
   *       ・√ を取る対は旧実装と**同じ式**で lb を出し、**同じ比較**(lb >= best / lb < mlb)をする
   *       ・いちばん近い対(mi)も、同値なら行優先で先の対 = 旧実装の「< で最初に見つけた対」と同じ
   *     開く順と開く対が同じなので、best も mid(同値のときの選ばれ方を含めて)も同じ。
   */
  function partsPartsClearance(A, B) {
    var rough = len(sub(A.bs.c, B.bs.c)) - A.bs.r - B.bs.r;
    if (rough > 0.25) return { clearance: rough, mid: A.bs.c };
    var CA = A.clusters, CB = B.clusters;
    var na = CA.length, nb = CB.length;
    if (na === 0 || nb === 0) return { clearance: rough, mid: A.bs.c };
    var acx = A.bs.c[0], acy = A.bs.c[1], acz = A.bs.c[2];
    var bcx = B.bs.c[0], bcy = B.bs.c[1], bcz = B.bs.c[2];
    var rowLb = rowBuf(na), colLb = colBuf(nb), colIx = _colIx;
    var i, j, k, ca, cb, cx, cy, cz, cr, dx, dy, dz, d2, L, lb;

    // --- (0) かたまりの球を全部包む半径(bs.c を中心に)。★下限を成り立たせるため ---
    var rcA = 0, rcB = 0;
    for (i = 0; i < na; i++) {
      ca = CA[i]; dx = ca.c[0] - acx; dy = ca.c[1] - acy; dz = ca.c[2] - acz;
      L = Math.sqrt(dx*dx + dy*dy + dz*dz) + ca.r; if (L > rcA) rcA = L;
    }
    for (j = 0; j < nb; j++) {
      cb = CB[j]; dx = cb.c[0] - bcx; dy = cb.c[1] - bcy; dz = cb.c[2] - bcz;
      L = Math.sqrt(dx*dx + dy*dy + dz*dz) + cb.r; if (L > rcB) rcB = L;
    }
    // --- (a)(b) 行と列の下限 ---
    var iMin = 0, rowMin = Infinity;
    for (i = 0; i < na; i++) {
      ca = CA[i]; dx = ca.c[0] - bcx; dy = ca.c[1] - bcy; dz = ca.c[2] - bcz;
      lb = Math.sqrt(dx*dx + dy*dy + dz*dz) - ca.r - rcB;
      rowLb[i] = lb; if (lb < rowMin) { rowMin = lb; iMin = i; }
    }
    for (j = 0; j < nb; j++) {
      cb = CB[j]; dx = cb.c[0] - acx; dy = cb.c[1] - acy; dz = cb.c[2] - acz;
      colLb[j] = Math.sqrt(dx*dx + dy*dy + dz*dz) - cb.r - rcA;
    }

    // --- (2) いちばん近いかたまり対 mi を探す(旧実装と同じ対が選ばれる) ---
    //   種: いちばん近い行の対を全部出して「境界」にする。★種は境界にしか使わない
    //   (mi の同値の決め方を旧実装と揃えるため、mlb 自体は Infinity から行優先で更新する)
    var seed = Infinity;
    ca = CA[iMin]; cx = ca.c[0]; cy = ca.c[1]; cz = ca.c[2]; cr = ca.r;
    for (j = 0; j < nb; j++) {
      cb = CB[j]; dx = cb.c[0] - cx; dy = cb.c[1] - cy; dz = cb.c[2] - cz;
      lb = Math.sqrt(dx*dx + dy*dy + dz*dz) - cr - cb.r;
      if (lb < seed) seed = lb;
    }
    var mlb = Infinity, mi = -1, bound = seed + PP_EPS, nc = 0;
    for (j = 0; j < nb; j++) if (colLb[j] <= bound) colIx[nc++] = j;
    for (i = 0; i < na; i++) {
      if (rowLb[i] > bound) continue;
      ca = CA[i]; cx = ca.c[0]; cy = ca.c[1]; cz = ca.c[2]; cr = ca.r;
      for (k = 0; k < nc; k++) {
        j = colIx[k]; cb = CB[j];
        if (colLb[j] > bound) continue;
        dx = cb.c[0] - cx; dy = cb.c[1] - cy; dz = cb.c[2] - cz;
        d2 = dx*dx + dy*dy + dz*dz;
        L = bound + cr + cb.r;
        if (L > 0 && d2 > L * L) continue;              // lb > bound が確実 → 最小ではない
        lb = Math.sqrt(d2) - cr - cb.r;                  // ★旧実装と同じ式
        if (lb < mlb) { mlb = lb; mi = i * nb + j; if (mlb < seed) bound = mlb + PP_EPS; }
      }
    }
    if (mi < 0) return { clearance: rough, mid: A.bs.c };

    var best = Infinity, mid = A.bs.c;
    /** かたまり対の中身を総当たりする。best を縮められたら縮める
     *  ★Q-11: 箱 1 個 ⇄ 相手のかたまりの球 で先に落とす(実測: 箱の行の 82% がここで消える。
     *    §8-5 の「62%」の正体は、この中の箱対の平方距離の走査だった = 1 呼び出しで約 9.7 万対)。
     *    落とすのは「lb > best + PP_EPS が確実」な箱だけ。旧実装もその箱の対は
     *    lim の比較で 1 対ずつ落としていた(= rbPair に届く対は同じ・同じ順)。 */
    function open(ia, ib) {
      var la = CA[ia].idx, lb2 = CB[ib].idx, ny = lb2.length;
      var cA = CA[ia], cB = CB[ib];
      var cAx = cA.c[0], cAy = cA.c[1], cAz = cA.c[2], cAr = cA.r;
      var cBx = cB.c[0], cBy = cB.c[1], cBz = cB.c[2], cBr = cB.r;
      var bnd = best + PP_EPS, alive = aliveBuf(ny), y, b, gx, gy, gz;
      for (y = 0; y < ny; y++) {                       // 相手の箱 ⇄ こちらのかたまりの球
        b = B.parts[lb2[y]];
        gx = b.c[0] - cAx; gy = b.c[1] - cAy; gz = b.c[2] - cAz;
        alive[y] = (Math.sqrt(gx*gx + gy*gy + gz*gz) - b.br - cAr > bnd) ? 0 : 1;
      }
      for (var x = 0; x < la.length; x++) {
        var a = A.parts[la[x]];
        var ax = a.c[0], ay = a.c[1], az = a.c[2], abr = a.br;
        gx = cBx - ax; gy = cBy - ay; gz = cBz - az;   // こちらの箱 ⇄ 相手のかたまりの球
        if (Math.sqrt(gx*gx + gy*gy + gz*gz) - abr - cBr > best + PP_EPS) continue;
        for (y = 0; y < ny; y++) {
          if (alive[y] === 0) continue;
          b = B.parts[lb2[y]];
          // ★平方距離で枝刈りする(√を取らない)。
          //   |ca-cb| >= best + abr + b.br なら、この対は best を縮められない
          var ex = b.c[0] - ax, ey = b.c[1] - ay, ez = b.c[2] - az;
          var lim = best + abr + b.br;
          if (lim > 0 && (ex*ex + ey*ey + ez*ez) >= lim * lim) continue;
          var r = rbPair(a, A.R, b, B.R);
          if (r.clearance < best) { best = r.clearance; mid = r.mid; }
        }
      }
    }

    open((mi / nb) | 0, mi % nb);          // ★いちばん近い対を先に開く。
    //   小さい best を先に掴むほど、このあとの枝刈りがよく効く。
    //   ★「毎回未処理の最小を探して順に開く」も試したが、
    //     n=784 の走査を開くたび繰り返すので★逆に遅かった
    //     (1対 1.38ms → 2.10ms 実測)。一回見すだけで十分。

    // --- (3) 残りを行優先で。lb >= best の対は開かない(旧実装と同じ判断) ---
    //   ★行・列・平方距離で「lb > best + PP_EPS が確実」な対を先に落とす。
    //     落とした対は旧実装でも lb >= best で開かれない(か、開いても縮まらない)。
    bound = best + PP_EPS; nc = 0;
    for (j = 0; j < nb; j++) if (colLb[j] <= bound) colIx[nc++] = j;
    for (i = 0; i < na; i++) {
      if (rowLb[i] > bound) continue;
      ca = CA[i]; cx = ca.c[0]; cy = ca.c[1]; cz = ca.c[2]; cr = ca.r;
      for (k = 0; k < nc; k++) {
        j = colIx[k];
        if (i * nb + j === mi) continue;
        if (colLb[j] > bound) continue;
        cb = CB[j];
        dx = cb.c[0] - cx; dy = cb.c[1] - cy; dz = cb.c[2] - cz;
        d2 = dx*dx + dy*dy + dz*dz;
        L = bound + cr + cb.r;
        if (L > 0 && d2 > L * L) continue;              // lb > best が確実 → 開かない
        lb = Math.sqrt(d2) - cr - cb.r;                  // ★旧実装と同じ式
        if (lb >= best) continue;                        // ★旧実装と同じ比較
        open(i, j);
        bound = best + PP_EPS;
      }
    }
    return { clearance: best, mid: mid };
  }
  /** 丸め箱の集合 ↔ 障害物(planeY / obb / box / cylinder / capsule) */
  function partsObstacleClearance(A, ob) {
    var rough = pointToObstacle(A.bs.c, ob) - A.bs.r;
    if (rough > 0.25) return { clearance: rough, mid: A.bs.c };
    var CA = A.clusters, n = CA.length;
    var buf = lbBuf(n);
    var best = Infinity, mid = A.bs.c, mi = -1, mlb = Infinity, i;
    for (i = 0; i < n; i++) {
      var lb = pointToObstacle(CA[i].c, ob) - CA[i].r;
      buf[i] = lb;
      if (lb < mlb) { mlb = lb; mi = i; }
    }
    if (mi < 0) return { clearance: rough, mid: A.bs.c };
    function open(ci) {
      var idx = CA[ci].idx;
      for (var x = 0; x < idx.length; x++) {
        var a = A.parts[idx[x]];
        if (pointToObstacle(a.c, ob) - a.br >= best) continue;
        var r = rbToObstacle(a, A.R, ob);
        if (r.clearance < best) { best = r.clearance; mid = r.mid; }
      }
    }
    open(mi);
    for (i = 0; i < n; i++) {
      if (i === mi || buf[i] >= best) continue;
      open(i);
    }
    return { clearance: best, mid: mid };
  }

  /** 障害物が「向きつきの直方体」として扱えるなら {c,R,half} を返す。でなければ null。
   *  box は Y まわりに yaw だけ回った直方体なので、その回転行列を作る。 */
  function asBoxLike(ob) {
    if (ob.type === "obb") return { c: ob.c, R: ob.R, half: ob.half };
    if (ob.type === "box") {
      var yaw = ob.yaw || 0, ca = Math.cos(yaw), sa = Math.sin(yaw);
      return { c: ob.c, R: [ca, 0, sa,  0, 1, 0,  -sa, 0, ca], half: ob.half };
    }
    return null;
  }

  /** 丸め箱1個 ↔ 障害物。★刻まずに交互射影で最近点まで降りる */
  function rbToObstacle(a, Ra, ob) {
    if (a.R) Ra = a.R;              // ★部品が自分の向きを持っていればそれを使う(条件12 段B)
    var pa = a.c, k;
    for (k = 0; k < 16; k++) {
      var po = closestOnObstacle(pa, ob);
      var na = clampToBox(po, a.c, Ra, a.half);
      if (len(sub(na, pa)) < 1e-7) { pa = na; break; }
      pa = na;
    }
    var best = pointToObstacle(pa, ob), bp = pa;
    // ★相手も直方体なら、食い込みは**分離軸で厳密に**出せる。
    //   代表点15個の近似は、箱を大きくすると角が外へ出て★逆に浅くなる
    //   (包んでも深さが単調にならない)。実測: 細かい -98.81mm に対し
    //   それを包む粗い箱が -93.65mm と、浅く出た。
    //   rbPair(物体どうし)は前から分離軸を使っている。障害物側も揃える。
    var obBox = asBoxLike(ob);
    if (obBox && best <= 1e-6) {
      var penB = obbPenetration(a.c, Ra, a.half, obBox.c, obBox.R, obBox.half);
      if (penB > 0) return { clearance: -(penB + a.r), mid: a.c };
    }
    // ★食い込んでいるときは交互射影が深さを教えてくれないので、代表点でも測る。
    //
    //   ★ここは長らく `if (best <= 0)` と書いてあった。それだと
    //     **箱が面をまたいでいるとき**に深さを測り直さない:
    //     交互射影は「面の上の点」へ収束するので best は 0(浮動小数で +1e-18)。
    //     0 より大きいので条件を外れ、★どれだけ食い込んでいても ≒0mm と出た。
    //     実測: 角が床下 74.01mm にある箱が 0.00mm と報告された。
    //     (探索用の粗い判定体を検証していて発覚 → tools/coarse-safety.js)
    //   ★離れている場合は交互射影が厳密なので、代表点で測り直す必要は無い
    //     (凸どうしの交互射影は最近点対に収束する)。だから閾値で切る。
    //   収束判定が 1e-7 なので、その1桁上を閾値にする。
    if (best <= 1e-6) {
      var pts = boxProbePoints(a.c, Ra, a.half);
      for (var i = 0; i < pts.length; i++) {
        var d = pointToObstacle(pts[i], ob);
        if (d < best) { best = d; bp = pts[i]; }
      }
    }
    return { clearance: best - a.r, mid: bp };
  }

  /** 障害物の表面のうち、点 p にいちばん近い点 */
  function closestOnObstacle(p, ob) {
    if (ob.type === "planeY") return [p[0], ob.y, p[2]];
    if (ob.type === "obb") return clampToBox(p, ob.c, ob.R, ob.half);
    if (ob.type === "box") {
      var yaw = ob.yaw || 0, ca = Math.cos(yaw), sa = Math.sin(yaw);
      var R = [ca, 0, sa,  0, 1, 0,  -sa, 0, ca];   // yaw の回転行列(列がローカル軸)
      return clampToBox(p, ob.c, R, ob.half);
    }
    if (ob.type === "cylinderY") {
      var y = Math.max(ob.y0, Math.min(ob.y1, p[1]));
      var vx = p[0] - ob.c[0], vz = p[2] - ob.c[1];
      var L = Math.sqrt(vx*vx + vz*vz);
      if (L < 1e-9) return [ob.c[0] + ob.r, y, ob.c[1]];
      var s = Math.min(1, ob.r / L);
      return [ob.c[0] + vx*s, y, ob.c[1] + vz*s];
    }
    if (ob.type === "cylinder") {
      var d = ob.d;
      var t = (p[0]-ob.c[0])*d[0] + (p[1]-ob.c[1])*d[1] + (p[2]-ob.c[2])*d[2];
      if (t >  ob.h) t =  ob.h; else if (t < -ob.h) t = -ob.h;
      var ax = [ob.c[0]+d[0]*t, ob.c[1]+d[1]*t, ob.c[2]+d[2]*t];
      var v = sub(p, ax), L2 = len(v);
      if (L2 < 1e-9) return ax;
      var s2 = Math.min(1, ob.r / L2);
      return [ax[0]+v[0]*s2, ax[1]+v[1]*s2, ax[2]+v[2]*s2];
    }
    if (ob.a && ob.b) {                              // カプセル
      var q = segSegClosest(p, p, ob.a, ob.b).q;
      var v2 = sub(p, q), L3 = len(v2);
      if (L3 < 1e-9) return q;
      var s3 = Math.min(1, ob.r / L3);
      return [q[0]+v2[0]*s3, q[1]+v2[1]*s3, q[2]+v2[2]*s3];
    }
    throw new Error("closestOnObstacle: 未対応の型 '" + (ob.type || "(型なし)") +
                    "' id=" + (ob.id || "?"));
  }

  /** 点 p から、丸め箱の集合までの距離(合体の外側は正・内側は負) */
  function pointToParts(p, A) {
    var rough = len(sub(p, A.bs.c)) - A.bs.r;
    if (rough > 0.25) return rough;
    var best = Infinity, CA = A.clusters;
    for (var i = 0; i < CA.length; i++) {
      if (len(sub(p, CA[i].c)) - CA[i].r >= best) continue;
      var idx = CA[i].idx;
      for (var x = 0; x < idx.length; x++) {
        var a = A.parts[idx[x]];
        if (len(sub(p, a.c)) - a.br >= best) continue;
        var d = pointToRBox(p, a, A.R);
        if (d < best) best = d;
      }
    }
    return best;
  }

  /**
   * ボディを「代表点 + そこに付く半径」の列にする。
   *   丸め箱の集合と、球以外のボディを突き合わせるときに使う。
   *   ★刻みの粗さは nFixed に従う(絶対条件9: 判定の設定は全箇所で同一)。
   */
  function bodySamplePoints(B, nFixed) {
    var out = [], i;
    if (B.type === "spheres") {
      for (i = 0; i < B.spheres.length; i++) out.push({ p: B.spheres[i].c, r: B.spheres[i].r });
      return out;
    }
    if (B.type === "obb") {
      var po = obbSamples(B, (nFixed && nFixed <= 16) ? 3 : 5);
      for (i = 0; i < po.length; i++) out.push({ p: po[i], r: 0 });
      return out;
    }
    if (B.type === "cylinder") {
      var coarse = (nFixed && nFixed <= 16);
      var pc = cylinderSamples(B, coarse ? 3 : 5, coarse ? 8 : 16);
      for (i = 0; i < pc.length; i++) out.push({ p: pc[i], r: 0 });
      return out;
    }
    if (!B.a || !B.b)
      throw new Error("bodySamplePoints: 未対応のボディ型 '" + (B.type || "(型なし)") +
                      "' id=" + (B.id || "?"));
    var n = nFixed || 24;                        // カプセル(工具の各部)
    for (i = 0; i <= n; i++) {
      var u = i / n;
      out.push({ p: [B.a[0] + (B.b[0]-B.a[0])*u,
                     B.a[1] + (B.b[1]-B.a[1])*u,
                     B.a[2] + (B.b[2]-B.a[2])*u], r: B.r });
    }
    return out;
  }

  /**
   * ★メッシュを「球の集合」に変換する。 ★旧方式。F-71 で作り直した。
   *   下互換のために残してある(shape-fidelity が新旧を並べて出せるように)。
   *   格子(cell)で頂点をまとめ、各格子の外接球を1個ずつ作る。
   *   cell を小さくするほど形に忠実だが球が増えて遅くなるので、
   *   目標個数に収まるよう cell を自動で調整する。
   *   戻り値 { spheres:[{c,r}], bs:{c,r} }  bs は全体の外接球(粗い枝刈り用)
   */
  function sphereSetFromMesh(src, targetCount) {
    var verts = meshVertices(src);
    var bb = src.bbox;
    var size = Math.max(bb[3] - bb[0], bb[4] - bb[1], bb[5] - bb[2]);
    var cell = size / 6;
    var best = null;
    for (var pass = 0; pass < 8; pass++) {
      var cells = {};
      for (var i = 0; i < verts.length; i++) {
        var v = verts[i];
        var k = Math.floor(v[0] / cell) + "," + Math.floor(v[1] / cell) + "," + Math.floor(v[2] / cell);
        var c = cells[k];
        if (!c) { cells[k] = { bb: [v[0], v[1], v[2], v[0], v[1], v[2]], vs: [v] }; }
        else {
          var b = c.bb;
          if (v[0] < b[0]) b[0] = v[0]; if (v[1] < b[1]) b[1] = v[1]; if (v[2] < b[2]) b[2] = v[2];
          if (v[0] > b[3]) b[3] = v[0]; if (v[1] > b[4]) b[4] = v[1]; if (v[2] > b[5]) b[5] = v[2];
          c.vs.push(v);
        }
      }
      var keys = Object.keys(cells);
      best = { cells: cells, keys: keys, cell: cell };
      if (keys.length >= targetCount * 0.6 && keys.length <= targetCount * 1.6) break;
      cell *= (keys.length < targetCount) ? 0.75 : 1.35;
    }
    var spheres = [];
    for (var j = 0; j < best.keys.length; j++) {
      var cc = best.cells[best.keys[j]], c2 = cc.bb;
      var ctr = [(c2[0] + c2[3]) / 2, (c2[1] + c2[4]) / 2, (c2[2] + c2[5]) / 2];
      // ★半径は「格子bboxの対角の半分」ではなく「中心から実際の頂点までの
      //   最大距離」にする。前者は必ず実形状より大きく、腕どうしが
      //   触れてもいないのに -1〜-4mm の自己干渉として出ていた。
      //   後者も立派な外接球なので、安全側を崩さずに過大評価だけ消える。
      var rr = 0;
      for (var vi = 0; vi < cc.vs.length; vi++) {
        var vv2 = cc.vs[vi];
        var dd2 = Math.sqrt(Math.pow(vv2[0] - ctr[0], 2) + Math.pow(vv2[1] - ctr[1], 2) +
                            Math.pow(vv2[2] - ctr[2], 2));
        if (dd2 > rr) rr = dd2;
      }
      spheres.push({ c: ctr, r: rr });
    }
    // 全体の外接球
    var bc = [(bb[0] + bb[3]) / 2, (bb[1] + bb[4]) / 2, (bb[2] + bb[5]) / 2];
    var br = 0.5 * Math.sqrt(Math.pow(bb[3] - bb[0], 2) + Math.pow(bb[4] - bb[1], 2) +
                             Math.pow(bb[5] - bb[2], 2));
    return { spheres: spheres, bs: { c: bc, r: br } };
  }

  // ===========================================================================
  // ★球近似で拾った干渉を、CADメッシュの実頂点で測り直す
  //
  //   球は格子ごとの外接球なので、曲面の谷や角では実形状より外へ膨らむ。
  //   そのぶん「当たっている」と過大に出る。実際、前腕とスポークが
  //   球近似で -24mm と報告された箇所を実頂点で測ると 0.0mm(接するだけ)で、
  //   ★貫通していなかった。
  //   球で当たりを拾い、拾った所だけ実頂点で確かめる、の二段構えにすると
  //   速さと正確さが両立する(実頂点は数万点あるので全コマには使えない)。
  // ===========================================================================
  var _vertCache = {};
  function cachedVertices(name, src) {
    if (!_vertCache[name]) _vertCache[name] = meshVertices(src);
    return _vertCache[name];
  }

  /**
   * 姿勢 q における「リンクの実メッシュ」と「障害物」の最短距離。
   *   link … { id, joint }  meshSrc … MESHES[link.id]
   * 戻り値は距離[m]。0以上なら実際には当たっていない。
   */
  function meshMinDistance(q, axes, tool, armFrame, link, meshSrc, ob) {
    var st = KIN.fk(axes, q, tool);
    var f = (link.joint < 0) ? { R: null, p: [0, 0, 0], o: [0, 0, 0] } : st.frames[link.joint];
    var verts = cachedVertices(link.id, meshSrc);
    var best = Infinity;
    for (var i = 0; i < verts.length; i++) {
      var v = verts[i];
      var lv = [v[0] - f.o[0], v[1] - f.o[1], v[2] - f.o[2]];
      var rv = f.R ? KIN.matVec(f.R, lv) : lv;
      var pw = [rv[0] + f.p[0] + armFrame[0],
                rv[1] + f.p[1] + armFrame[1],
                rv[2] + f.p[2] + armFrame[2]];
      var dd = pointToObstacle(pw, ob);
      if (dd < best) { best = dd; if (best <= 0) return 0; }
    }
    return best;
  }

  /** 球の集合を「軸方向 d・中心 c」の座標系へ並べ替えず、そのまま剛体変換する */
  function transformSpheres(set, R, p, o, frameOff) {
    var out = new Array(set.spheres.length);
    for (var i = 0; i < set.spheres.length; i++) {
      var s = set.spheres[i];
      var v = R ? KIN.matVec(R, sub(s.c, o)) : sub(s.c, o);
      out[i] = { c: [v[0] + p[0] + frameOff[0], v[1] + p[1] + frameOff[1], v[2] + p[2] + frameOff[2]],
                 r: s.r };
    }
    var bv = R ? KIN.matVec(R, sub(set.bs.c, o)) : sub(set.bs.c, o);
    return { spheres: out,
             bs: { c: [bv[0] + p[0] + frameOff[0], bv[1] + p[1] + frameOff[1], bv[2] + p[2] + frameOff[2]],
                   r: set.bs.r } };
  }

  /**
   * 2線分 p1-p2 と q1-q2 の最短距離。(Ericson, Real-Time Collision Detection 5.1.9)
   * 戻り値 { d, p, q } … 距離と、それぞれの線分上の最近接点
   */
  function segSegClosest(p1, p2, q1, q2) {
    var d1 = sub(p2, p1), d2 = sub(q2, q1), r = sub(p1, q1);
    var a = dot(d1, d1), e = dot(d2, d2), f = dot(d2, r);
    var s, t;
    if (a < 1e-12 && e < 1e-12) {
      s = 0; t = 0;
    } else if (a < 1e-12) {
      s = 0; t = clamp(f / e, 0, 1);
    } else {
      var c = dot(d1, r);
      if (e < 1e-12) {
        t = 0; s = clamp(-c / a, 0, 1);
      } else {
        var b = dot(d1, d2), denom = a * e - b * b;
        s = (denom > 1e-12) ? clamp((b * f - c * e) / denom, 0, 1) : 0;
        t = (b * s + f) / e;
        if (t < 0) { t = 0; s = clamp(-c / a, 0, 1); }
        else if (t > 1) { t = 1; s = clamp((b - c) / a, 0, 1); }
      }
    }
    var cp1 = add(p1, scale(d1, s)), cp2 = add(q1, scale(d2, t));
    return { d: len(sub(cp1, cp2)), p: cp1, q: cp2 };
  }

  /** カプセル(線分+半径)どうしの隙間。負なら食い込んでいる */
  function capsuleClearance(segA, rA, segB, rB) {
    var c = segSegClosest(segA[0], segA[1], segB[0], segB[1]);
    return { clearance: c.d - rA - rB, mid: scale(add(c.p, c.q), 0.5) };
  }

  /**
   * 点 p から「Y軸に平行な有限円柱」までの距離。円柱の中にあれば 0。
   *   円柱: 中心 (cx, cz) / 半径 R / 高さ y0..y1 (上下は平ら)
   * 半径方向のはみ出しと高さ方向のはみ出しを直交2成分として合成する。
   * これにより上下の角(エッジ)まで正しい距離になり、
   * カプセルのような「上下のドーム」が生えない。
   */
  function pointToCylinderY(p, cx, cz, R, y0, y1) {
    var dx = p[0] - cx, dz = p[2] - cz;
    var dr = Math.sqrt(dx * dx + dz * dz) - R;
    if (dr < 0) dr = 0;
    var dy = 0;
    if (p[1] < y0) dy = y0 - p[1];
    else if (p[1] > y1) dy = p[1] - y1;
    return Math.sqrt(dr * dr + dy * dy);
  }

  /**
   * 線分と有限円柱の最短距離。
   * 解析的に解くと場合分けが多いので、線分を細かく刻んで最小値を採る近似。
   * 既定の刻み幅は約 8mm。モック(レイアウト検討)の用途にはこれで十分で、
   * 誤差は刻み幅の半分(約4mm)程度。
   *   nFixed … 刻み数を固定したいとき(IKの中から何千回も呼ぶので粗くする)
   */
  function segToCylinderY(segA, cx, cz, R, y0, y1, nFixed) {
    var L = len(sub(segA[1], segA[0]));
    var n = nFixed || Math.max(16, Math.min(160, Math.ceil(L / 0.008)));
    var best = Infinity, bestP = segA[0];
    for (var i = 0; i <= n; i++) {
      var u = i / n;
      var p = [segA[0][0] + (segA[1][0] - segA[0][0]) * u,
               segA[0][1] + (segA[1][1] - segA[0][1]) * u,
               segA[0][2] + (segA[1][2] - segA[0][2]) * u];
      var dd = pointToCylinderY(p, cx, cz, R, y0, y1);
      if (dd < best) { best = dd; bestP = p; }
    }
    return { d: best, p: bestP };
  }

  /**
   * 点 p から「任意の向きの有限円柱」までの距離。中にあれば 0。
   *   円柱: 中心 c / 軸方向 d(単位ベクトル) / 半径 R / 半長 h
   * ★把持中のホイールに使う。ホイールは工具軸まわりに付いて回るので、
   *   Y軸固定の cylinderY では表せない。
   */
  function pointToCylinder(p, c, d, R, h) {
    var v = sub(p, c);
    var t = dot(v, d);
    var radial = Math.sqrt(Math.max(0, dot(v, v) - t * t));
    var dr = Math.max(0, radial - R);
    var dt = Math.max(0, Math.abs(t) - h);
    return Math.sqrt(dr * dr + dt * dt);
  }

  /** 線分と任意軸の有限円柱の最短距離(刻んで最小値を採る近似) */
  function segToCylinder(segA, c, d, R, h, nFixed) {
    var L = len(sub(segA[1], segA[0]));
    var n = nFixed || Math.max(16, Math.min(160, Math.ceil(L / 0.008)));
    var best = Infinity, bestP = segA[0];
    for (var i = 0; i <= n; i++) {
      var u = i / n;
      var p = [segA[0][0] + (segA[1][0] - segA[0][0]) * u,
               segA[0][1] + (segA[1][1] - segA[0][1]) * u,
               segA[0][2] + (segA[1][2] - segA[0][2]) * u];
      var dd = pointToCylinder(p, c, d, R, h);
      if (dd < best) { best = dd; bestP = p; }
    }
    return { d: best, p: bestP };
  }

  /**
   * ★点から障害物までの距離。障害物の形ごとに切り替える。
   *   「把持中のホイール」のような円盤状の“動く物体”を、
   *   代表点を並べて全障害物と突き合わせるために使う。
   */
  /**
   * ★任意姿勢の直方体(OBB)への外側距離。
   *   既存の "box" は Y軸まわりの yaw しか回せない(架台・角材が用途)。
   *   ★把持して振り回すワークは3軸とも回るので、それでは表せない。
   *   R は列が箱のローカル軸(three.js と同じ行優先9要素)。
   *   外側は厳密。内側(貫通)は 0 を返す = 深さは出せない。
   *   → 干渉しているかどうかは分かるが、どれだけ食い込んだかは分からない。
   *     余裕の最小化には外側の値しか使わないので、ここでは問題にならない。
   */
  function pointToOBB(p, ob) {
    var d = sub(p, ob.c), R = ob.R, h = ob.half;
    // ローカル座標 = Rᵀ·d (R の列がローカル軸なので、行との内積になる)
    var lx = R[0]*d[0] + R[3]*d[1] + R[6]*d[2];
    var ly = R[1]*d[0] + R[4]*d[1] + R[7]*d[2];
    var lz = R[2]*d[0] + R[5]*d[1] + R[8]*d[2];
    var qx = Math.abs(lx) - h[0], qy = Math.abs(ly) - h[1], qz = Math.abs(lz) - h[2];
    var ex = Math.max(0, qx), ey = Math.max(0, qy), ez = Math.max(0, qz);
    var outside = Math.sqrt(ex*ex + ey*ey + ez*ez);
    if (outside > 0) return outside;
    // ★点が箱の内側にあるときは「最も近い面までの距離」を負で返す。
    //   以前は 0 で下から切っていたので、★食い込んでいる対も 0.0mm と出て
    //   「隙間ゼロで接している」と区別できなかった(自己テストで発覚)。
    //   0 以上を合格条件にしていたので、実際に当たっていても合格になる。
    //   qx,qy,qz はすべて 0 以下なので、最大 = いちばん近い面。
    return Math.max(qx, Math.max(qy, qz));
  }

  /** ★OBB の表面の代表点。面ごとに n×n の格子(端を含むので角と稜も入る) */
  function obbSamples(ob, n) {
    var R = ob.R, h = ob.half, out = [];
    function put(l){
      out.push([ob.c[0] + R[0]*l[0] + R[1]*l[1] + R[2]*l[2],
                ob.c[1] + R[3]*l[0] + R[4]*l[1] + R[5]*l[2],
                ob.c[2] + R[6]*l[0] + R[7]*l[1] + R[8]*l[2]]);
    }
    for (var ax = 0; ax < 3; ax++) {
      var a1 = (ax+1)%3, a2 = (ax+2)%3;
      for (var sgn = -1; sgn <= 1; sgn += 2) {
        for (var i = 0; i < n; i++) for (var j = 0; j < n; j++) {
          var l = [0,0,0];
          l[ax] = sgn * h[ax];
          l[a1] = (n===1?0:(-h[a1] + 2*h[a1]*i/(n-1)));
          l[a2] = (n===1?0:(-h[a2] + 2*h[a2]*j/(n-1)));
          put(l);
        }
      }
    }
    return out;
  }

  function pointToObstacle(p, ob) {
    // ★実形状(丸め箱の集合)の障害物。点→集合は既にある関数で見る。
    if (ob && ob.type === "parts") return pointToParts(p, ob);
    if (ob.type === "planeY") return p[1] - ob.y;
    if (ob.type === "obb") return pointToOBB(p, ob);
    if (ob.type === "cylinderY") return pointToCylinderY(p, ob.c[0], ob.c[1], ob.r, ob.y0, ob.y1);
    if (ob.type === "cylinder") return pointToCylinder(p, ob.c, ob.d, ob.r, ob.h);
    // ★角柱(Y軸まわりに yaw だけ回っている直方体)。
    //   架台やツールスタンドは角柱なのに外接円の円柱で見ていて太すぎ、
    //   スポークは角材なのに内接円の丸棒で見ていて細すぎた。
    //   実際の形で見るためにこの型を足した。
    if (ob.type === "box") {
      var dx = p[0] - ob.c[0], dy = p[1] - ob.c[1], dz = p[2] - ob.c[2];
      var yaw = ob.yaw || 0;
      var ca = Math.cos(-yaw), sa = Math.sin(-yaw);
      var lx = dx * ca + dz * sa, lz = -dx * sa + dz * ca;
      var qx = Math.abs(lx) - ob.half[0];
      var qy = Math.abs(dy) - ob.half[1];
      var qz = Math.abs(lz) - ob.half[2];
      var ex = Math.max(0, qx), ey = Math.max(0, qy), ez = Math.max(0, qz);
      var outside = Math.sqrt(ex * ex + ey * ey + ez * ez);
      if (outside > 0) return outside;
      // ★点が箱の内側にあるときは「最も近い面までの距離」を負で返す。
      //   ここは長らく 0 を返していた = ★どれだけ食い込んでも 0.0mm。
      //   同じ直しは pointToOBB には入っていたのに、box にだけ入っていなかった。
      //   box 型の障害物は アーム架台 / 受け台ポスト / 受け台 天板 /
      //   カメラ支柱 / 灯 / ワーク ── ★セルの主要な物がほぼ全部これ。
      //   つまりこれらへの食い込みの深さは一度も正しく測れていなかった。
      //   (探索用の粗い判定体を検証していて発覚 → tools/coarse-safety.js)
      return Math.max(qx, Math.max(qy, qz));
    }
    return segSegClosest(p, p, ob.a, ob.b).d - ob.r;      // カプセル
  }

  /**
   * ★円盤(円柱)状の物体の代表点。外周面と両端面が効くのでそこを並べる。
   *   把持中のホイールはこれで全障害物・腕と突き合わせる。
   */
  function cylinderSamples(body, nAx, nAng) {
    var d = body.d;
    var t1 = (Math.abs(d[0]) < 0.9) ? [1, 0, 0] : [0, 1, 0];
    var u = KIN.unit(KIN.cross(d, t1));
    var v = KIN.cross(d, u);
    var pts = [];
    for (var i = 0; i < nAx; i++) {
      var t = (nAx === 1) ? 0 : (-body.h + 2 * body.h * i / (nAx - 1));
      for (var k = 0; k < nAng; k++) {
        var a = 2 * Math.PI * k / nAng;
        var cs = Math.cos(a) * body.r, sn = Math.sin(a) * body.r;
        pts.push([body.c[0] + d[0] * t + u[0] * cs + v[0] * sn,
                  body.c[1] + d[1] * t + u[1] * cs + v[1] * sn,
                  body.c[2] + d[2] * t + u[2] * cs + v[2] * sn]);
      }
    }
    // ★端面の内側も入れる。側面(と縁)だけだと、端面の真ん中に当たる形を
    //   見落として隙間を過大に見積もる(= 安全側でない方へ間違える)。
    //   端面の中心と半径半分のリングを足す。
    for (var e = -1; e <= 1; e += 2) {
      var te = e * body.h;
      pts.push([body.c[0] + d[0]*te, body.c[1] + d[1]*te, body.c[2] + d[2]*te]);
      for (var k2 = 0; k2 < nAng; k2++) {
        var a2 = 2 * Math.PI * k2 / nAng;
        var cs2 = Math.cos(a2) * body.r * 0.5, sn2 = Math.sin(a2) * body.r * 0.5;
        pts.push([body.c[0] + d[0]*te + u[0]*cs2 + v[0]*sn2,
                  body.c[1] + d[1]*te + u[1]*cs2 + v[1]*sn2,
                  body.c[2] + d[2]*te + u[2]*cs2 + v[2]*sn2]);
      }
    }
    return pts;
  }

  /** 障害物1つ分の隙間を求める。type に応じて近似を切り替える */
  function clearanceTo(seg, rLink, ob, nFixed) {
    if (ob.type === "cylinderY") {
      var res = segToCylinderY(seg, ob.c[0], ob.c[1], ob.r, ob.y0, ob.y1, nFixed);
      return { clearance: res.d - rLink, mid: res.p };
    }
    if (ob.type === "cylinder") {
      var rc = segToCylinder(seg, ob.c, ob.d, ob.r, ob.h, nFixed);
      return { clearance: rc.d - rLink, mid: rc.p };
    }
    if (ob.type === "obb") {
      // 線分側を刻む。点→OBB が厳密なので、誤差は刻み幅ぶんだけ
      var Lo = len(sub(seg[1], seg[0]));
      var no = nFixed || Math.max(16, Math.min(160, Math.ceil(Lo / 0.008)));
      var bo = Infinity, po = seg[0];
      for (var io = 0; io <= no; io++) {
        var uo = io / no;
        var ptO = [seg[0][0] + (seg[1][0]-seg[0][0])*uo,
                  seg[0][1] + (seg[1][1]-seg[0][1])*uo,
                  seg[0][2] + (seg[1][2]-seg[0][2])*uo];
        var dO = pointToOBB(ptO, ob);
        if (dO < bo) { bo = dO; po = ptO; }
      }
      return { clearance: bo - rLink, mid: po };
    }
    if (ob.type === "box") {
      var Lb = len(sub(seg[1], seg[0]));
      var nb = nFixed || Math.max(16, Math.min(160, Math.ceil(Lb / 0.008)));
      var bb2 = Infinity, bp2 = seg[0];
      for (var ib = 0; ib <= nb; ib++) {
        var ub = ib / nb;
        var pb = [seg[0][0] + (seg[1][0] - seg[0][0]) * ub,
                  seg[0][1] + (seg[1][1] - seg[0][1]) * ub,
                  seg[0][2] + (seg[1][2] - seg[0][2]) * ub];
        var db = pointToObstacle(pb, ob);
        if (db < bb2) { bb2 = db; bp2 = pb; }
      }
      return { clearance: bb2 - rLink, mid: bp2 };
    }
    // ★床。y = ob.y より下へ入ったら干渉。これを入れていなかったので
    //   腕が地面に埋まる姿勢を平気で選んでいた
    if (ob.type === "planeY") {
      var lo = Math.min(seg[0][1], seg[1][1]);
      var pt = (seg[0][1] <= seg[1][1]) ? seg[0] : seg[1];
      return { clearance: lo - ob.y - rLink, mid: [pt[0], ob.y, pt[2]] };
    }
    // 既定はカプセル(線分+半径)。細長い棒向け
    return capsuleClearance(seg, rLink, [ob.a, ob.b], ob.r);
  }

  /**
   * ★リンク形状(CADメッシュの bbox)からカプセルを作る。
   *   bbox の一番長い辺を軸にした「線分+半径」。丸い腕の近似として妥当。
   *   bb … [minx,miny,minz,maxx,maxy,maxz] (CAD座標)
   *   r  … 半径。図面値(腕の外径 ø190 → 95mm)を使う。
   *        bbox の断面から出すと関節ハウジングのぶん太くなりすぎるため
   */
  function capsuleFromBbox(bb, r) {
    var c = [(bb[0] + bb[3]) / 2, (bb[1] + bb[4]) / 2, (bb[2] + bb[5]) / 2];
    var d = [bb[3] - bb[0], bb[4] - bb[1], bb[5] - bb[2]];
    var ax = (d[0] >= d[1] && d[0] >= d[2]) ? 0 : (d[1] >= d[2] ? 1 : 2);
    var half = Math.max(0, d[ax] / 2 - r);
    var a = c.slice(), b = c.slice();
    a[ax] -= half; b[ax] += half;
    return { a: a, b: b, r: r };
  }

  /**
   * ★姿勢 q のときの「腕の実体」を、ワールド座標のカプセル列として返す。
   *   links[j] = { cap:{a,b,r}, joint:i }
   *     joint = -1 … 架台に固定のリンク(CAD座標をそのまま armFrame に置く)
   *     joint = i  … 関節 i を回した後の座標系に乗るリンク
   *   さらに工具(ATC+カメラ/ハンド+爪)をフランジから工具軸方向へ伸ばして足す。
   */
  function armBodies(q, axes, tool, armFrame, links, toolLen, toolR, toolAlign) {
    var st = KIN.fk(axes, q, tool);
    var out = [];
    for (var j = 0; j < links.length; j++) {
      var L = links[j];
      // ★実形状(球の集合)を持っているリンクはそちらを使う。
      //   1本のカプセルでは腕の曲がった形を表現できず、太らせすぎていた
      if (L.set) {
        var fs = (L.joint < 0) ? { R: null, p: [0, 0, 0], o: [0, 0, 0] } : st.frames[L.joint];
        // ★丸め箱の集合(K-37)を持っているならそちら。
        //   球の集合は旧方式(F-71 で廃止)だが、比較のために受け続ける。
        if (L.set.parts) {
          var tp = transformParts(L.set, fs.R, fs.p, fs.o, armFrame);
          out.push({ id: L.id, type: "parts", parts: tp.parts, R: tp.R,
                     clusters: tp.clusters, bs: tp.bs });
          continue;
        }
        var tf = transformSpheres(L.set, fs.R, fs.p, fs.o, armFrame);
        out.push({ id: L.id, type: "spheres", spheres: tf.spheres, bs: tf.bs });
        continue;
      }
      var cap = L.cap, a, b;
      if (L.joint < 0) {
        a = add(cap.a, armFrame); b = add(cap.b, armFrame);
      } else {
        var f = st.frames[L.joint];
        a = add(add(f.p, KIN.matVec(f.R, sub(cap.a, f.o))), armFrame);
        b = add(add(f.p, KIN.matVec(f.R, sub(cap.b, f.o))), armFrame);
      }
      out.push({ id: L.id, a: a, b: b, r: cap.r });
    }
    // ★工具。1本の太いカプセルではなく、実際の部品ごとに分ける。
    //   toolLen が配列(部品列)で渡されたら [開始, 終了, 半径] の並びとして扱う
    var fp = add(st.pos, armFrame);
    if (Array.isArray(toolLen)) {
      // 工具軸に直交する基底(爪や指をリング状に並べるのに使う)
      var uu, vv;
      if (toolAlign) {
        // ★工具のローカル X/Z 軸をそのまま使う = 描画(toolfrm)と同じ向き。
        //   これを渡さないと下の「機械的に作った基底」になり、
        //   ★2本指の並ぶ向きが絵と最大90°ずれる。実際それで、絵では左右に
        //   開いている指が判定では上下に置かれ、下の指が受け台の天板へ
        //   半径ぶん(15.8mm)食い込んでいた。
        //   爪3本のホイール版はほぼ軸対称なので位相のずれが表に出なかった。
        var TR = KIN.matMul(st.R, toolAlign);
        uu = KIN.unit(KIN.matVec(TR, [1, 0, 0]));
        vv = KIN.unit(KIN.matVec(TR, [0, 0, 1]));
      } else {
        uu = KIN.unit(KIN.cross(st.dir, (Math.abs(st.dir[0]) < 0.9) ? [1, 0, 0] : [0, 1, 0]));
        vv = KIN.cross(st.dir, uu);
      }
      for (var t = 0; t < toolLen.length; t++) {
        var pt = toolLen[t];
        if (pt.ring) {
          // ★爪は PCD 上に並ぶ細いピン。1本の太い円柱で見ると実際の3〜4倍太い
          for (var ci = 0; ci < pt.ring.n; ci++) {
            var ang = 2 * Math.PI * ci / pt.ring.n;
            var rdir = add(scale(uu, Math.cos(ang)), scale(vv, Math.sin(ang)));  // 半径方向
            var off = scale(rdir, pt.ring.pcd / 2);
            if (pt.ring.box) {
              // ★板は板のまま見る。外接円の丸棒にすると
              //   ★内側へ (r − 厚み/2) ぶん膨らみ、掛む相手を構造的に貫く。
              //   実測: 指の板(厚10)を r=15.81 の丸棒で見ていたため、
              //   掛んだ瞬間に★姿勢に関係なく -15.81mm の干渉が出ていた。
              //   過大は安全側だが嘘で、★設計上ゼロ距離で接する部品では
              //   「永久に成立しない工程」に化ける。→ F-86 / K-40
              //   pcd は★板の中心の PCD。box = [半径方向の半値, 接線方向の半値]。
              var tdir = add(scale(uu, -Math.sin(ang)), scale(vv, Math.cos(ang))); // 接線方向
              out.push({ id: (pt.id || "爪") + (ci + 1), isTool: true, type: "obb",
                         c: add(add(fp, scale(st.dir, (pt.from + pt.to) / 2)), off),
                         R: [rdir[0], tdir[0], st.dir[0],
                             rdir[1], tdir[1], st.dir[1],
                             rdir[2], tdir[2], st.dir[2]],
                         half: [pt.ring.box[0], pt.ring.box[1], (pt.to - pt.from) / 2] });
              continue;
            }
            out.push({ id: (pt.id || "爪") + (ci + 1), isTool: true,
                       a: add(add(fp, scale(st.dir, pt.from)), off),
                       b: add(add(fp, scale(st.dir, pt.to)), off), r: pt.ring.pinR });
          }
          continue;
        }
        if (pt.bar) {
          // ★バー照明は工具軸に「直交して横へ張り出す」。
          //   from/to の軸方向カプセルでは表せない(半径にすると
          //   バー長の円柱になり、実物の何倍にも太る)。
          //   この走査では光軸が水平・半径方向なので、
          //   バーの長手 = 光軸 × 世界の上 = 接線方向 になる。
          var wup = [0, 1, 0];
          var bd0 = KIN.cross(st.dir, wup);
          if (Math.hypot(bd0[0], bd0[1], bd0[2]) < 1e-6) {
            bd0 = KIN.cross(st.dir, [1, 0, 0]);      // 光軸が真上/真下のときの逃げ
          }
          var bdir = KIN.unit(bd0);
          var sdir = KIN.unit(KIN.cross(bdir, st.dir));   // 画像の上方向
          var bc = add(add(fp, scale(st.dir, pt.bar.at)),
                       scale(sdir, pt.bar.side || 0));
          var bh = Math.max(0, pt.bar.len / 2 - pt.bar.r);
          out.push({ id: pt.id || "バー照明", isTool: true,
                     a: add(bc, scale(bdir, -bh)),
                     b: add(bc, scale(bdir,  bh)), r: pt.bar.r });
          continue;
        }
        // ★flat = 端面が平らな部品(実物の円筒・角ブロック)。
        //   カプセルで見ると★両端に半径ぶんの丸い鼻が付く。ATC(ø90)なら
        //   前後に 45mm ずつ、ハンド(ø60)なら 30mm ずつ、実物より長くなる。
        //   ハンド本体の前 5mm にワークを持っているので、カプセルでは
        //   ★常に -25mm の干渉になり(実測)、それを「必ず接する」と称して
        //   除外していた = 実際には形の誤りを除外で隠していた。
        //   → 端面が平らな部品は OBB(任意姿勢の直方体)で見る。
        //     軸方向は厳密、断面は一辺 2r の正方形 = 円柱の外接 = 過大評価側。
        // ★flat:"cyl" = 丸い部品(ATC・カメラ)。OBB にすると断面が
        //   一辺 2r の正方形になり、半径方向に最大 r(√2-1) ぶん太る
        //   (ø100 のカメラで 20.7mm)。実測でカメラ×腕の余裕が
        //   9.4mm → 1.3mm に落ちたので、丸い部品は円筒のままにする。
        if (pt.flat === "cyl") {
          out.push({ id: pt.id || ("tool" + t), isTool: true, onAxis: true, type: "cylinder",
                     c: add(fp, scale(st.dir, (pt.from + pt.to) / 2)),
                     d: st.dir, r: pt.r, h: (pt.to - pt.from) / 2 });
          continue;
        }
        if (pt.flat) {
          // ★断面は既定で「一辺 2r の正方形」= 円柱の外接(過大側)。
          //   ★2026-09-18(Q-01 / 条件12): 実物が**角ブロック**の部品は、
          //   `halfXY:[hx,hy]` を渡せば**長方形断面**にできる。
          //   ■ なぜ要るか
          //     空圧 2 爪ハンド(SCHUNK JGP-P 100-2)は本体 120×50mm。
          //     外接円柱(ø130)で見ると★**短辺側が 40mm 過大**で、
          //     これは条件12(接触する部品の過大は 30mm 以下)に収まらない。
          //     ★過大な工具の判定体は「工程を永久に成立しなくする」(F-86 で実測 15.81mm)。
          //   ★halfXY を渡さなければ従来どおり(既存のキーの答えは 1 ビットも変わらない)。
          //   ★uu / vv は工具の横方向の 2 軸。halfXY は [uu 方向, vv 方向] の半寸法。
          // ★Q-10(2026-09-18 夜): **toolAlign が無いときは halfXY を使わない。**
          //   ■ なぜ
          //     halfXY は「uu = 工具のローカル X 軸 / vv = 同 Z 軸」を前提にしている。
          //     ★toolAlign を渡さない呼び出しでは uu/vv は**機械的に作った基底**で、
          //     位相が任意 = ★**箱の向きが実物と最大 90° ずれる**([F-03](docs/ノウハウ/失敗と解決策.md) と同じ型。
          //     実際にそれで 2 本指の向きが絵と 96° ずれ、下の指が受け台へ 15.8mm 食い込んでいた)。
          //   ★向きのずれた長方形は**過小(穴)**を作る。過大より危険(実機が壊れる方向)。
          //   → ★向きが保証されないときは、従来どおり**一辺 2r の正方形**(= 円柱の外接 = 過大側)に落とす。
          //   ★黙って落とさない: どちらで見たかは body の `boxFit` に残す(監査が読める)。
          var useBox = !!(pt.halfXY && toolAlign);
          var hx = useBox ? pt.halfXY[0] : pt.r;
          var hy = useBox ? pt.halfXY[1] : pt.r;
          out.push({ id: pt.id || ("tool" + t), isTool: true, onAxis: !useBox, type: "obb",
                     boxFit: useBox ? "実寸" : (pt.halfXY ? "★正方形に落とした(toolAlign が無い)" : "正方形"),
                     c: add(fp, scale(st.dir, (pt.from + pt.to) / 2)),
                     R: [uu[0], vv[0], st.dir[0],
                         uu[1], vv[1], st.dir[1],
                         uu[2], vv[2], st.dir[2]],
                     half: [hx, hy, (pt.to - pt.from) / 2] });
          continue;
        }
        // ★onAxis = 工具軸に乗った軸対称の部品。J6 が回っても占める空間が
        //   変わらないので、手首(arm_J5/J6)との距離は姿勢によらず一定になる。
        //   リング(爪・指)やバー照明は軸外なので false。
        out.push({ id: pt.id || ("tool" + t), isTool: true, onAxis: true,
                   a: add(fp, scale(st.dir, pt.from)),
                   b: add(fp, scale(st.dir, pt.to)), r: pt.r });
      }
    } else if (toolLen > 0) {
      out.push({ id: "tool", isTool: true, onAxis: true,
                 a: fp, b: add(fp, scale(st.dir, toolLen)), r: toolR });
    }
    return out;
  }

  /**
   * ★姿勢1つぶんの最悪クリアランス。IK の解の良し悪しを測るのに使う。
   *   IK の中から何千回も呼ばれるので、刻みを粗く(既定12分割)して速くする。
   *   戻り値 { clearance, link, obstacle, point }
   */
  /** 球1個と障害物1個の隙間 */
  function sphereObstacleClearance(s, ob) {
    return { clearance: pointToObstacle(s.c, ob) - s.r, mid: s.c };
  }

  /** 物体1つと障害物1つの隙間。物体が円盤(把持中のホイール)なら代表点で見る */
  function bodyObstacleClearance(bd, ob, nFixed) {
    // ★障害物が実形状(丸め箱の集合)なら、ボディ同士と同じ道で見る。
    //   これが無いと、静止しているワークを中実の円柱でしか置けない。
    //   ★中央が開いている物(ホイール)では、触れていない姿勢が干渉になる。
    if (ob && ob.type === "parts") return bodyBodyClearance(bd, ob, nFixed);
    // ★直方体のボディ(把持中の角物)。相手が平面や円柱なので、
    //   ここは箱の表面を刻む。粗い呼び出し(nFixed<=16)では 3×3、
    //   それ以外は 5×5(1面あたり)。
    if (bd.type === "obb") {
      // ★まず外接球で粗く枝刈りする(球の集合と同じ手)。
      //   工具の部品を OBB にしたとき、1対あたり 54点(粗いときでも)を
      //   刻むので、遠い障害物まで真面目に見ると★組み立てが倍近くかかった
      //   (実測 13s → 26s)。中心からの距離だけで落とせるものは落とす。
      var rOB = Math.sqrt(bd.half[0]*bd.half[0] + bd.half[1]*bd.half[1] +
                          bd.half[2]*bd.half[2]);
      var roughB = pointToObstacle(bd.c, ob) - rOB;
      // ★枝刈りは粗い呼び出し(IK の中)だけ。返す値は「下限」なので、
      //   報告用の精査(nFixed=0)でこれを返すと★数値そのものが狂う
      //   (自己テストの「床までの距離 450mm」が 413mm になって発覚)。
      //   250mm はバジェット 15mm から充分離れているので、IK の判断は変わらない。
      if (nFixed && nFixed <= 16 && roughB > 0.25) return { clearance: roughB, mid: bd.c };
      var ptsB = obbSamples(bd, (nFixed && nFixed <= 16) ? 3 : 5);
      var bB = Infinity, pB = bd.c;
      for (var iB = 0; iB < ptsB.length; iB++) {
        var dB = pointToObstacle(ptsB[iB], ob);
        if (dB < bB) { bB = dB; pB = ptsB[iB]; }
      }
      return { clearance: bB, mid: pB };
    }
    // ★実形状(丸め箱の集合・K-37)。枝刈りは partsObstacleClearance の中
    if (bd.type === "parts") return partsObstacleClearance(bd, ob);
    // ★実形状(球の集合・旧方式)。まず全体の外接球で粗く枝刈りしてから中を見る
    if (bd.type === "spheres") {
      var rough = pointToObstacle(bd.bs.c, ob) - bd.bs.r;
      if (rough > 0.25) return { clearance: rough, mid: bd.bs.c };
      var bestS = Infinity, bpS = bd.bs.c;
      for (var i = 0; i < bd.spheres.length; i++) {
        var d = pointToObstacle(bd.spheres[i].c, ob) - bd.spheres[i].r;
        if (d < bestS) { bestS = d; bpS = bd.spheres[i].c; }
      }
      return { clearance: bestS, mid: bpS };
    }
    if (bd.type !== "cylinder") return clearanceTo([bd.a, bd.b], bd.r, ob, nFixed);
    var coarse = (nFixed && nFixed <= 16);
    var pts = cylinderSamples(bd, coarse ? 3 : 5, coarse ? 8 : 16);
    var best = Infinity, bp = pts[0];
    for (var i = 0; i < pts.length; i++) {
      var dd = pointToObstacle(pts[i], ob);
      if (dd < best) { best = dd; bp = pts[i]; }
    }
    return { clearance: best, mid: bp };
  }

  /** 点から物体までの距離(物体の形ごと) */
  function pointToBody(p, B) {
    if (B.type === "parts") return pointToParts(p, B);
    if (B.type === "spheres") {
      var rough = len(sub(p, B.bs.c)) - B.bs.r;
      if (rough > 0.25) return rough;
      var best = Infinity;
      for (var i = 0; i < B.spheres.length; i++) {
        var d = len(sub(p, B.spheres[i].c)) - B.spheres[i].r;
        if (d < best) best = d;
      }
      return best;
    }
    if (B.type === "obb") return pointToOBB(p, B);
    if (B.type === "cylinder") return pointToCylinder(p, B.c, B.d, B.r, B.h);
    // ★縦置きの有限円柱もボディとして受ける。
    //   以前は障害物側にしか無く、把持中ワークなどに cylinderY を渡すと
    //   下のカプセル分岐へ落ちて B.a が undefined で例外になっていた
    //   (自己テストで発覚)。
    if (B.type === "cylinderY")
      return pointToCylinderY(p, B.c[0], B.c[1], B.r, B.y0, B.y1);
    // ★型が分からないものを黙ってカプセル扱いしない。
    //   silently 間違うより、その場で落ちて気づけるほうが安全
    if (!B.a || !B.b)
      throw new Error("pointToBody: 未対応のボディ型 '" + (B.type || "(型なし)") +
                      "' id=" + (B.id || "?") +
                      " — spheres/obb/cylinder/cylinderY/カプセル のみ");
    return segSegClosest(p, p, B.a, B.b).d - B.r;      // カプセル
  }

  /** 物体どうしの隙間。片方が円盤なら代表点 vs カプセルで見る */
  /** ★縦置き有限円柱のボディを、任意軸円柱に正規化する。
   *  cylinderY は元は障害物用の型だったので、ボディとして渡されると
   *  カプセル分岐へ落ちて a/b が undefined で例外になっていた(自己テストで発覚)。
   *  入口で1回変換しておけば、以降の経路は全部そのまま動く。 */
  function asCylinderBody(B) {
    if (!B || B.type !== "cylinderY") return B;
    var mid = (B.y0 + B.y1) / 2, hh = (B.y1 - B.y0) / 2;
    return { id: B.id, type: "cylinder", c: [B.c[0], mid, B.c[1]], d: [0, 1, 0],
             r: B.r, h: hh, isTool: B.isTool, skipBodies: B.skipBodies,
             skipObstacles: B.skipObstacles };
  }

  function bodyBodyClearance(A, B, nFixed) {
    A = asCylinderBody(A); B = asCylinderBody(B);
    // ★丸め箱の集合(K-37)。両方が集合なら 2段の外接球で枝刈りする。
    //   片方だけなら、相手の代表点を 点→丸め箱の集合 で見る。
    if (A.type === "parts" && B.type === "parts") return partsPartsClearance(A, B);
    if (A.type === "parts" || B.type === "parts") {
      var PS = (A.type === "parts") ? A : B;
      var OT2 = (A.type === "parts") ? B : A;
      var pts0 = bodySamplePoints(OT2, nFixed);
      var bP = Infinity, pP = PS.bs.c;
      for (var iP = 0; iP < pts0.length; iP++) {
        var dP = pointToParts(pts0[iP].p, PS) - pts0[iP].r;
        if (dP < bP) { bP = dP; pP = pts0[iP].p; }
      }
      return { clearance: bP, mid: pP };
    }
    // ★片方が直方体なら、★相手のほうを刻んで「点→OBB」で見る。
    //   点→OBB は厳密なので、箱を刻むより誤差が小さい
    //   (箱の面は広く、刻みの間に最近点が落ちて隙間を過大に見積もる)。
    if (A.type === "obb" || B.type === "obb") {
      var BX = (A.type === "obb") ? A : B;
      var OT = (A.type === "obb") ? B : A;
      if (OT.type === "obb") {                 // 箱どうしは今は出てこない
        var ptsX = obbSamples(OT, (nFixed && nFixed <= 16) ? 3 : 5);
        var bX = Infinity, pX = OT.c;
        for (var iX = 0; iX < ptsX.length; iX++) {
          var dX = pointToOBB(ptsX[iX], BX);
          if (dX < bX) { bX = dX; pX = ptsX[iX]; }
        }
        return { clearance: bX, mid: pX };
      }
      var pts2, rad2 = 0;
      if (OT.type === "spheres") {
        var bS = Infinity, pS = BX.c;
        for (var iS = 0; iS < OT.spheres.length; iS++) {
          var sS = OT.spheres[iS];
          var dS = pointToOBB(sS.c, BX) - sS.r;
          if (dS < bS) { bS = dS; pS = sS.c; }
        }
        return { clearance: bS, mid: pS };
      }
      if (OT.type === "cylinder") {
        pts2 = cylinderSamples(OT, (nFixed && nFixed <= 16) ? 3 : 5,
                                   (nFixed && nFixed <= 16) ? 8 : 16);
      } else {                                  // カプセル(工具の各部)
        var nC = nFixed || 24, L2 = [OT.a, OT.b];
        pts2 = []; rad2 = OT.r;
        for (var iC = 0; iC <= nC; iC++) {
          var uC = iC / nC;
          pts2.push([L2[0][0] + (L2[1][0]-L2[0][0])*uC,
                     L2[0][1] + (L2[1][1]-L2[0][1])*uC,
                     L2[0][2] + (L2[1][2]-L2[0][2])*uC]);
        }
      }
      var b2 = Infinity, p2 = BX.c;
      for (var i2 = 0; i2 < pts2.length; i2++) {
        var d2 = pointToOBB(pts2[i2], BX) - rad2;
        if (d2 < b2) { b2 = d2; p2 = pts2[i2]; }
      }
      return { clearance: b2, mid: p2 };
    }
    // ★どちらかが実形状(球の集合)なら、球ごとに相手との距離を見る。
    //   まず外接球どうしで粗く枝刈りする(ほとんどの組はここで終わる)
    if (A.type === "spheres" || B.type === "spheres") {
      var SA = (A.type === "spheres") ? A : B;
      var OS = (A.type === "spheres") ? B : A;
      var cA = SA.bs, cB = (OS.type === "spheres") ? OS.bs : null;
      if (cB && len(sub(cA.c, cB.c)) - cA.r - cB.r > 0.25) {
        return { clearance: len(sub(cA.c, cB.c)) - cA.r - cB.r, mid: cA.c };
      }
      var bestS = Infinity, bpS = cA.c;
      for (var i = 0; i < SA.spheres.length; i++) {
        var s = SA.spheres[i];
        var d = pointToBody(s.c, OS) - s.r;
        if (d < bestS) { bestS = d; bpS = s.c; }
      }
      return { clearance: bestS, mid: bpS };
    }
    if (A.type !== "cylinder" && B.type !== "cylinder") {
      return capsuleClearance([A.a, A.b], A.r, [B.a, B.b], B.r);
    }
    var cyl = (A.type === "cylinder") ? A : B;
    var cap = (A.type === "cylinder") ? B : A;
    if (cap.type === "cylinder") {           // 両方円盤になることは今は無い
      return { clearance: Infinity, mid: cyl.c };
    }
    var coarse = (nFixed && nFixed <= 16);
    var pts = cylinderSamples(cyl, coarse ? 3 : 5, coarse ? 8 : 16);
    var best = Infinity, bp = pts[0];
    for (var i = 0; i < pts.length; i++) {
      var dd = segSegClosest(pts[i], pts[i], cap.a, cap.b).d - cap.r;
      if (dd < best) { best = dd; bp = pts[i]; }
    }
    return { clearance: best, mid: bp };
  }

  /**
   *   extraBodies … 腕と一緒に動く「腕以外の物体」。把持中のホイールを渡す。
   *     ★これを物体として扱うことで、ホイール自身が受け台・アーム架台・床・
   *       ツールスタンドに当たるのも検出できる。以前はホイールを「腕に対する
   *       障害物」としてしか見ていなかったので、ホイールが台や架台へ突っ込む
   *       のを素通ししていた。
   */
  // ===========================================================================
  // ★判定の網羅性を記録する仕組み(監査)
  //
  //   「当たり判定が付いていないオブジェクトがあるのでは」という疑いは、
  //   コードを読んでも確かめられない(除外の条件が3種類あり、しかも
  //   skipBodies は完全一致なので、書いたつもりで効いていないことがある)。
  //   ★そこで「実際に比較した対」と「除外した対とその理由」を記録して
  //   表に出す。setAudit(obj) で有効化し、obj にキー別の集計が入る。
  //   既定は null なので、通常の実行に影響しない。
  // ===========================================================================
  var AUDIT = null;
  function setAudit(a) { AUDIT = a; }
  function audit(kind, a, b, skip, clr) {
    if (!AUDIT) return;
    var k = kind + "|" + a + "|" + b;
    var e = AUDIT[k] || (AUDIT[k] = { kind:kind, a:a, b:b, cmp:0, skip:{},
                                      min:Infinity, max:-Infinity });
    if (skip) { e.skip[skip] = (e.skip[skip] || 0) + 1; }
    else { e.cmp++; if (clr < e.min) e.min = clr; if (clr > e.max) e.max = clr; }
  }

  // ===========================================================================
  // ★「剛体内」の対 — 相対位置が姿勢によらず一定な対
  //
  //   工具はフランジにボルトで付いていて、把持中のワークは指でつかまれている。
  //   だから《工具の部品どうし》《工具とフランジ(arm_J6)》《工具と把持中ワーク》
  //   《フランジと把持中ワーク》の距離は、★どんな姿勢でも変わらない。
  //   コマごとに見る意味はない。
  //
  //   ただし「見なくてよい」ではない。見ないままにすると
  //   ★「ハンド本体とワークが当たっている設計」を素通しする(実際にしていた)。
  //   → コマごとの判定からは外し、checkRigid() で★静的に1回だけ確かめる。
  //
  //   以前はここを「工具と手首はすべて除外(wristSide)」で書いていた。
  //   それだとフランジから 130〜190mm 先にある指まで arm_J5 と一括で除外され、
  //   ★姿勢で変わる対まで見なくなっていた(指は光軸まわりに振り回される)。
  // ===========================================================================
  function tipSide(b) { return !!(b.isTool || b.id === "arm_J6" || b.rigidWithTool); }
  // ★手首(J5/J6) × 「工具軸に乗った軸対称の部品」も距離が一定。
  //   J6 の回転はその部品が占める空間を変えないので、J5 がどんな形でも
  //   距離は変わらない(実測 arm_J5×ATC = 15.01〜15.02mm、幅 0.01mm)。
  //   ★軸外の部品(指・爪・バー照明)は振り回されるので、これに含めない
  //   (実測 arm_J5×指 = 160.45〜167.18mm、幅 6.7mm = 姿勢で変わる)。
  var WRIST_IDS = { arm_J5: 1, arm_J6: 1 };
  function rigidPair(A, B) {
    if (tipSide(A) && tipSide(B)) return true;
    if (WRIST_IDS[A.id] && B.isTool && B.onAxis) return true;
    if (WRIST_IDS[B.id] && A.isTool && A.onAxis) return true;
    return false;
  }

  /** 剛体内の対を静的に確かめる。
   *  bodies … armBodies(+把持中ワーク) の結果。どの姿勢のものでもよい
   *  opt.touch … 接して当然の対 ["ハンド|指1", "指1|ワーク(把持中)", ...]
   *  戻り値 { rows:[{a,b,clearance,touch}], worst:{a,b,clearance}|null }
   */
  function checkRigid(bodies, opt) {
    var touch = (opt && opt.touch) || [];
    // ★blind = 「形の近似のせいでこの対は判定できない」と宣言した対。
    //   touch(接して当然)と混ぜてはいけない。混ぜると
    //   「見なくてよい」と「見られない」が区別できなくなり、
    //   ★近似の誤りを除外で隠す、という今回の失敗がまた起きる。
    //   呼び側は理由を必ず info に出すこと。
    var blind = (opt && opt.blind) || [];
    function has(list, a, b) {
      return list.indexOf(a + "|" + b) !== -1 || list.indexOf(b + "|" + a) !== -1;
    }
    var rows = [], worst = null;
    for (var i = 0; i < bodies.length; i++) {
      for (var j = i + 1; j < bodies.length; j++) {
        var A = bodies[i], B = bodies[j];
        if (!rigidPair(A, B)) continue;
        var tch = has(touch, A.id, B.id), bln = has(blind, A.id, B.id);
        var c = bodyBodyClearance(A, B, 0).clearance;
        rows.push({ a: A.id, b: B.id, clearance: c, touch: tch, blind: bln });
        if (!tch && !bln && (!worst || c < worst.clearance)) {
          worst = { a: A.id, b: B.id, clearance: c };
        }
      }
    }
    return { rows: rows, worst: worst };
  }

  // =========================================================================
  // ★静止物どうしの当たり判定(絶対条件5 / 条件1)
  //
  //   checkPose は「動く体 × 障害物」と「動く体どうし」しか見ない。
  //   ★障害物どうしは構造上ループに入らない = 一度も比較していなかった。
  //   ⑦で [F-20] として見つけたが、その案件専用の幾何で塞いだだけだったので
  //   ③④で同じ穴が開いた([F-57])。ここで構造として直す。
  //
  //   静止物は動かないので、姿勢ごとに見る必要はない。**組み立て時に1回**でよい。
  // =========================================================================

  /** 静止物を代表する点。★点で近似するので「細い物が太い物の隙間をすり抜ける」
   *  配置は見落とす。限界として呼び側が出すこと。
   *  ★知らない型は例外。0 や null を返して黙って合格させない。 */
  function obstacleSamples(ob) {
    var out = [], i, j, k, a, th, x, z;
    if (ob.type === "obb") return obbSamples(ob, 3);
    if (ob.type === "cylinder") return cylinderSamples(ob, 3, 8);
    if (ob.type === "box") {
      var yaw = ob.yaw || 0, ca = Math.cos(yaw), sa2 = Math.sin(yaw), h = ob.half;
      for (i = -1; i <= 1; i++) for (j = -1; j <= 1; j++) for (k = -1; k <= 1; k++) {
        var lx = i * h[0], ly = j * h[1], lz = k * h[2];
        // ★2026-10-09(台S・tools/apply-static-samples-yaw.py): 向きの符号を pointToObstacle / asBoxLike / 絵(rotationDeg)と同じ ローカル x = (cos, 0, −sin) に揃えた。
        //   それまで +sin で、yaw が 90° の倍数でない箱(光学装置の fixed_unit・受け台 スポーク)の代表点が鏡写しの位置に置かれ、静止物どうしの隙間が嘘になっていた(白板 × 投光器の台 +70 → −98)
        out.push([ob.c[0] + lx * ca + lz * sa2, ob.c[1] + ly, ob.c[2] - lx * sa2 + lz * ca]);
      }
      return out;
    }
    if (ob.type === "cylinderY") {
      var ym = (ob.y0 + ob.y1) / 2;
      for (a = 0; a < 8; a++) {
        th = a / 8 * Math.PI * 2;
        x = ob.c[0] + ob.r * Math.cos(th); z = ob.c[1] + ob.r * Math.sin(th);
        out.push([x, ob.y0, z]); out.push([x, ym, z]); out.push([x, ob.y1, z]);
      }
      out.push([ob.c[0], ob.y0, ob.c[1]]);
      out.push([ob.c[0], ym, ob.c[1]]);
      out.push([ob.c[0], ob.y1, ob.c[1]]);
      return out;
    }
    if (ob.type === "planeY") return null;          // 無限に広い。点で代表できない
    if (ob.a && ob.b) {                             // カプセル
      for (i = 0; i <= 4; i++) {
        var t = i / 4;
        out.push([ob.a[0] + (ob.b[0] - ob.a[0]) * t,
                  ob.a[1] + (ob.b[1] - ob.a[1]) * t,
                  ob.a[2] + (ob.b[2] - ob.a[2]) * t]);
      }
      return out;
    }
    throw new Error("obstacleSamples: 扱えない静止物 id=" + ob.id +
                    " type=" + ob.type + " keys=" + Object.keys(ob).join(","));
  }

  /** 静止物どうしの余裕[m]。負なら重なっている。
   *  ★box は内側で距離0を返す(pointToObstacle の実装)。めり込み量は
   *    過小に出るが、「0以下 = 接触/重なり」は正しく出る。 */
  function obstacleObstacleClearance(A, B) {
    if (A.type === "planeY" && B.type === "planeY")
      return { clearance: Infinity, blind: "面どうしは比較できない" };
    if (A.type === "planeY" || B.type === "planeY") {
      var pl = (A.type === "planeY") ? A : B, ob = (A.type === "planeY") ? B : A;
      var pts = obstacleSamples(ob);
      if (!pts) return { clearance: Infinity, blind: "点で代表できない" };
      var lo = Infinity;
      for (var s = 0; s < pts.length; s++) lo = Math.min(lo, pts[s][1] - pl.y);
      return { clearance: lo };
    }
    var pa = obstacleSamples(A), pb = obstacleSamples(B);
    if (!pa || !pb) return { clearance: Infinity, blind: "点で代表できない" };
    var best = Infinity, d;
    for (var i = 0; i < pa.length; i++) {
      d = pointToObstacle(pa[i], B); if (d < best) best = d;
    }
    for (var j = 0; j < pb.length; j++) {
      d = pointToObstacle(pb[j], A); if (d < best) best = d;
    }
    return { clearance: best };
  }

  /**
   * 静止物すべての対を1回だけ見る。
   *   opt.touch  … 「接して当然」の対 "a|b"(支柱と機器、床と脚など)
   *   opt.blind  … 「近似のせいで判定できない」対。★touch と混ぜない
   *   静止物自身が `touches:["id",...]` を持っていれば、それも touch として扱う
   *     = 幾何を書いた場所で宣言できるので、離れたリストと食い違わない
   */
  function checkStatic(obstacles, opt) {
    var touch = (opt && opt.touch) || [], blind = (opt && opt.blind) || [];
    function has(list, a, b) {
      return list.indexOf(a + "|" + b) !== -1 || list.indexOf(b + "|" + a) !== -1;
    }
    var rows = [], worst = null, nBlind = 0;
    for (var i = 0; i < obstacles.length; i++) {
      for (var j = i + 1; j < obstacles.length; j++) {
        var A = obstacles[i], B = obstacles[j];
        var declared = (A.touches && A.touches.indexOf(B.id) !== -1) ||
                       (B.touches && B.touches.indexOf(A.id) !== -1);
        var tch = declared || has(touch, A.id, B.id);
        var bln = has(blind, A.id, B.id);
        var r = obstacleObstacleClearance(A, B);
        if (r.blind) { bln = true; }
        if (bln) nBlind++;
        rows.push({ a: A.id, b: B.id, clearance: r.clearance,
                    touch: tch, blind: bln, why: r.blind || null });
        if (!tch && !bln && (!worst || r.clearance < worst.clearance))
          worst = { a: A.id, b: B.id, clearance: r.clearance };
      }
    }
    var bad = rows.filter(function (r) { return !r.touch && !r.blind && r.clearance <= 0; });
    return { rows: rows, worst: worst, overlapping: bad, nPairs: rows.length, nBlind: nBlind };
  }

  // =========================================================================
  // ★コマとコマの「間」を見る(絶対条件4)
  //
  //   checkPose は撮影点と経由点の**姿勢だけ**を見る。コマ間(アニメが補間して
  //   いる最中)は一度も見ていなかった。両端がセーフでも途中で腕が障害物を
  //   通り抜けることがある。
  //
  //   規則(03 §3): 隣り合うコマ間で、いずれかの判定体の点が 10mm 以上動くなら二分する。
  //   ★関節角で刻んではいけない。J2 を 15° 回すと腕の先端は 327mm 動き、
  //     いちばん細い障害物(ø25のピン)を軽く飛び越える。
  //   10mm の根拠: 離散化誤差 d/2 = 5mm。20mm だと物理誤差(2.81mm)を上回る。
  // =========================================================================

  /** 判定体の代表点を、順序が安定した1本の配列で返す(コマ間の移動量を測る用) */
  function bodyPoints(bodies) {
    var out = [], i, k;
    for (i = 0; i < bodies.length; i++) {
      var b = bodies[i];
      if (b.type === "parts" && b.parts) {
        for (k = 0; k < b.parts.length; k++) out.push(b.parts[k].c);
      } else if (b.type === "spheres" && b.spheres) {
        for (k = 0; k < b.spheres.length; k++) out.push(b.spheres[k].c);
      } else if (b.a && b.b) {
        out.push(b.a); out.push(b.b);
      } else if (b.c && b.c.length === 3) {
        out.push(b.c);
      }
    }
    return out;
  }

  /** 2つの点列の、対応する点どうしの最大移動量[m]。長さが違えば比べない */
  function maxPointShift(pa, pb) {
    var n = Math.min(pa.length, pb.length), worst = 0;
    for (var i = 0; i < n; i++) {
      var dx = pb[i][0]-pa[i][0], dy = pb[i][1]-pa[i][1], dz = pb[i][2]-pa[i][2];
      var d = Math.sqrt(dx*dx+dy*dy+dz*dz);
      if (d > worst) worst = d;
    }
    return worst;
  }

  function lerpQ(a, b, t) {
    var o = [];
    for (var i = 0; i < a.length; i++) o.push(a[i] + (b[i]-a[i])*t);
    return o;
  }

  /**
   * qA → qB の**間**を、判定体の移動量が stepM 以下になるまで二分して見る。
   *   ctx = { axes, tool, armFrame, model, obstacles, nFixed, heldAt, toolSpec }
   *     heldAt(q) … その姿勢での把持中ワーク(無ければ null)
   *   opt = { stepM: 0.010, maxSplit: 256 }
   * 返り値: { worst, nChecked, maxShift, truncated }
   *   ★truncated=true は「刻みきれなかった」= **合格ではない**。
   */
  function checkSegment(qA, qB, ctx, opt) {
    var stepM = (opt && opt.stepM) || 0.010;
    var maxSplit = (opt && opt.maxSplit) || 256;
    function bodiesAtQ(q) {
      var tLen = (ctx.toolSpec && ctx.toolSpec.len !== undefined) ? ctx.toolSpec.len : ctx.model.toolLen;
      var tR   = (ctx.toolSpec && ctx.toolSpec.r   !== undefined) ? ctx.toolSpec.r   : ctx.model.toolR;
      var bs = armBodies(q, ctx.axes, ctx.tool, ctx.armFrame, ctx.model.links, tLen, tR,
                         ctx.toolSpec && ctx.toolSpec.align);
      var h = ctx.heldAt ? ctx.heldAt(q) : null;
      return h ? bs.concat([h]) : bs;
    }
    // まず「何分割すれば移動量が stepM 以下になるか」を、両端から見積もる
    var pA = bodyPoints(bodiesAtQ(qA)), pB = bodyPoints(bodiesAtQ(qB));
    var shift = maxPointShift(pA, pB);
    var n = Math.max(1, Math.ceil(shift / stepM));
    var truncated = false;
    if (n > maxSplit) { n = maxSplit; truncated = true; }
    var worst = null, nChecked = 0;
    // 端は呼び側が既に見ているので、中だけ見る
    for (var i = 1; i < n; i++) {
      var q = lerpQ(qA, qB, i / n);
      var held = ctx.heldAt ? ctx.heldAt(q) : null;
      var r = checkPose(q, ctx.axes, ctx.tool, ctx.armFrame, ctx.model,
                        ctx.obstacles, ctx.nFixed || 12, held ? [held] : null, ctx.toolSpec);
      nChecked++;
      if (!worst || r.clearance < worst.clearance)
        worst = { clearance: r.clearance, link: r.link, obstacle: r.obstacle,
                  self: r.self, at: i / n };
    }
    return { worst: worst, nChecked: nChecked, nSplit: n, maxShift: shift, truncated: truncated };
  }

  // =========================================================================
  // ★干渉した区間に中間点を挿入して「避ける」(絶対条件4の後半)
  //
  //   checkSegment は「通り抜けている」ことを教えるだけ。条件4は
  //   「常にチェックし、**避ける**」なので、避ける側が要る。
  //
  //   やり方: 直線補間の中点を関節空間で振り、両半分を刻み直して
  //           **いちばん余裕が増える**中間点を選ぶ。足りなければ再帰。
  //   ★これは経路計画そのものではない。「この区間は中間点1つで避けられるか」を
  //     機械的に確かめるための最小の仕組み。避けられなければ**避けられないと言う**。
  // =========================================================================

  /** qA→qB を刻んで、最小余裕と、干渉した中間姿勢の数を返す */
  function scanSegment(qA, qB, ctx, opt) {
    var stepM = (opt && opt.stepM) || 0.010;
    var cap = (opt && opt.maxSplit) || 128;
    var pA = bodyPoints(ctx.bodiesAt(qA)), pB = bodyPoints(ctx.bodiesAt(qB));
    var shift = maxPointShift(pA, pB);
    var n = Math.max(1, Math.ceil(shift / stepM)), truncated = false;
    if (n > cap) { n = cap; truncated = true; }
    var worst = null, nHit = 0, i;
    for (i = 1; i < n; i++) {
      var q = lerpQ(qA, qB, i / n);
      var r = ctx.checkAt(q);
      if (!r) continue;
      if (r.clearance < 0) nHit++;
      if (!worst || r.clearance < worst.clearance)
        worst = { clearance: r.clearance, link: r.link, obstacle: r.obstacle,
                  self: r.self, at: i / n };
    }
    return { worst: worst, nHit: nHit, nSplit: n, shift: shift, truncated: truncated,
             minClear: worst ? worst.clearance : Infinity };
  }

  /**
   * 干渉している区間に中間点を挿入して避ける。
   *   ctx  = { bodiesAt(q), checkAt(q), limitsDeg }
   *   opt  = { stepM, need(必要な余裕[m]), amps(振り幅[deg]), depth, top }
   * 返り値: { ok, via:[q,...], minClear, tried }
   *   ok=false は「**避けられなかった**」。黙って通さない。
   *
   * ★候補の作り方(この順に強くなる):
   *   ① 動きを軸ごとに分解 … 「先に J1 だけ回す」など。大きな跳びに効く
   *   ② 中点を関節ごとに振る … 局所的な逃げ
   *   ③ ①②で通らなければ、**通せた中間点で分割してさらに探す**(再帰)
   *     1点で足りない区間があるので、これが無いと「避けられない」で終わる
   */
  function repairSegment(qA, qB, ctx, opt) {
    opt = opt || {};
    var need = (opt.need !== undefined) ? opt.need : 0.0;
    var amps = opt.amps || [8, 16, 32, 60];       // 度
    var depth = (opt.depth === undefined) ? 3 : opt.depth;
    var top = opt.top || 3;
    var base = scanSegment(qA, qB, ctx, opt);
    if (base.minClear >= need) return { ok: true, via: [], minClear: base.minClear, tried: 0 };
    if (depth <= 0) return { ok: false, via: [], minClear: base.minClear, tried: 0, base: base };

    var tried = 0, i, j, k;
    function inLimits(q) {
      if (!ctx.limitsDeg) return true;
      for (var z = 0; z < q.length; z++)
        if (ctx.limitsDeg[z] && (q[z] < ctx.limitsDeg[z][0] || q[z] > ctx.limitsDeg[z][1])) return false;
      return true;
    }

    // ---- 候補を作る ----
    var cands = [];
    for (j = 0; j < qA.length; j++) {                    // ① 軸ごとの分解
      var c1 = qA.slice(); c1[j] = qB[j]; cands.push(c1);
      var c2 = qB.slice(); c2[j] = qA[j]; cands.push(c2);
    }
    var mids = [lerpQ(qA, qB, 0.5), lerpQ(qA, qB, 0.3), lerpQ(qA, qB, 0.7)];
    for (var mi = 0; mi < mids.length; mi++)             // ② 中点を振る
      for (var ai = 0; ai < amps.length; ai++)
        for (j = 0; j < qA.length; j++)
          for (var sg = -1; sg <= 1; sg += 2) {
            var cc = mids[mi].slice(); cc[j] += sg * amps[ai]; cands.push(cc);
          }

    // ---- 「その姿勢自体が安全」な候補だけ残し、両半分の余裕で並べる ----
    var scored = [];
    for (i = 0; i < cands.length; i++) {
      if (!inLimits(cands[i])) continue;
      var rc = ctx.checkAt(cands[i]); tried++;
      if (!rc || rc.clearance < need) continue;
      var s1 = scanSegment(qA, cands[i], ctx, opt);
      var s2 = scanSegment(cands[i], qB, ctx, opt);
      scored.push({ q: cands[i], m: Math.min(s1.minClear, s2.minClear),
                    a: s1.minClear, b: s2.minClear });
    }
    scored.sort(function (x, y) { return y.m - x.m; });
    if (scored.length && scored[0].m >= need)
      return { ok: true, via: [scored[0].q], minClear: scored[0].m, tried: tried };

    // ---- ③ 1点で足りないので、良い候補で分割してさらに探す ----
    for (i = 0; i < Math.min(top, scored.length); i++) {
      var v = scored[i];
      var sub = { stepM: opt.stepM, need: need, amps: amps, depth: depth - 1, top: top };
      var r1 = (v.a >= need) ? { ok: true, via: [], minClear: v.a, tried: 0 }
                             : repairSegment(qA, v.q, ctx, sub);
      if (!r1.ok) continue;
      var r2 = (v.b >= need) ? { ok: true, via: [], minClear: v.b, tried: 0 }
                             : repairSegment(v.q, qB, ctx, sub);
      if (!r2.ok) continue;
      return { ok: true, via: r1.via.concat([v.q], r2.via),
               minClear: Math.min(r1.minClear, r2.minClear, v.m),
               tried: tried + r1.tried + r2.tried };
    }
    return { ok: false, via: [], minClear: scored.length ? scored[0].m : base.minClear,
             tried: tried, base: base };
  }

  function checkPose(q, axes, tool, armFrame, model, obstacles, nFixed, extraBodies, toolSpec) {
    // ★装着中の工具で当たり判定の大きさを変える。
    //   常に「一番太くて長い工具(ハンド+爪)」で見ていると、カメラを
    //   付けている区間まで過大に判定してしまう(実際それで -7mm の
    //   誤検知が出ていた。カメラは 150mm/半径49、ハンドは 215mm/半径65)
    var tLen = (toolSpec && toolSpec.len !== undefined) ? toolSpec.len : model.toolLen;
    var tR   = (toolSpec && toolSpec.r   !== undefined) ? toolSpec.r   : model.toolR;
    var bodies = armBodies(q, axes, tool, armFrame, model.links, tLen, tR,
                           toolSpec && toolSpec.align);
    if (extraBodies && extraBodies.length) bodies = bodies.concat(extraBodies);
    var worst = null;
    for (var s = 0; s < bodies.length; s++) {
      var bd = bodies[s];
      for (var o = 0; o < obstacles.length; o++) {
        var ob = obstacles[o];
        if (ob.skipLinks && ob.skipLinks.indexOf(bd.id) !== -1) {
          audit("obs", bd.id, ob.id, "障害物側 skipLinks"); continue; }
        if (bd.skipObstacles && bd.skipObstacles.indexOf(ob.id) !== -1) {
          audit("obs", bd.id, ob.id, "ボディ側 skipObstacles"); continue; }
        var res = bodyObstacleClearance(bd, ob, nFixed || 12);
        audit("obs", bd.id, ob.id, null, res.clearance);
        if (!worst || res.clearance < worst.clearance) {
          worst = { clearance: res.clearance, link: bd.id, obstacle: ob.id, point: res.mid };
        }
      }
    }
    // =======================================================================
    // ★腕どうしの干渉(自己干渉)
    //   これを見ていなかったので、腕が自分自身をすり抜ける姿勢を
    //   IK が平気で選んでいた。
    //   隣り合うリンク(i と i+1)は関節でつながっていて必ず接するので除外。
    //   手首まわり(J5/J6/工具)も互いに密着しているので、その中では見ない。
    // =======================================================================
    // ★工具はフランジに固定で付いているので、手首(J5/J6)とは必ず接する。
    //   工具の部品どうしも同様。名前ではなく isTool の印で判定する
    //   (工具を部品ごとに分けた際、名前で書いた除外が効かなくなって
    //    全コマ干渉と誤判定した)。
    // ★arm_J5 を外した。J5 は「フランジの1つ前のリンク」で、工具は J6 の
    //   回転と一緒に光軸まわりを振り回される = 指のような軸外の部品との
    //   距離は姿勢で変わる。剛体ではないので、ここで一括除外してはいけない。
    var WRIST = { arm_J6: 1 };
    function wristSide(b) { return b.isTool || WRIST[b.id]; }
    // ★間のリンクが短くて構造的に隣接している対は、i+2 離れていても必ず接する。
    //   arm_J1(カプセル長49mm)を挟む arm_J0↔arm_J2 がそれで、
    //   常時 -3mm 程度出ていたが実機では単に隣り合っているだけ
    var ADJ = { "arm_J0|arm_J2": 1 };
    for (var i = 0; i < bodies.length; i++) {
      for (var j = i + 1; j < bodies.length; j++) {
        var A = bodies[i], B = bodies[j];
        // ★隣り合う対(j===i+1)は関節でつながっていて必ず接するので見ない。
        //   以前は j を i+2 から回して「無かったこと」にしていたが、
        //   それだと監査に出てこないので、回して明示的に除外する(挙動は同じ)。
        if (j === i + 1) { audit("self", A.id, B.id, "隣り合うリンク"); continue; }
        // ★剛体内(距離が一定)の対はコマごとに見ない。checkRigid が静的に見る
        if (rigidPair(A, B)) {
          audit("self", A.id, B.id, "剛体内・距離が一定(静的判定 checkRigid で見る)"); continue; }
        if (wristSide(A) && wristSide(B)) {
          audit("self", A.id, B.id, "手首まわり同士(必ず接する)"); continue; }
        if (ADJ[A.id + "|" + B.id] || ADJ[B.id + "|" + A.id]) {
          audit("self", A.id, B.id, "構造的に隣接(ADJ)"); continue; }
        // 把持中のホイールは、掴んでいる工具や手首とは必ず接する
        if (A.skipBodies && A.skipBodies.indexOf(B.id) !== -1) {
          audit("self", A.id, B.id, "skipBodies("+A.id+"側)"); continue; }
        if (B.skipBodies && B.skipBodies.indexOf(A.id) !== -1) {
          audit("self", A.id, B.id, "skipBodies("+B.id+"側)"); continue; }
        var rs = bodyBodyClearance(A, B, nFixed || 12);
        audit("self", A.id, B.id, null, rs.clearance);
        if (!worst || rs.clearance < worst.clearance) {
          worst = { clearance: rs.clearance, link: B.id,
                    obstacle: "自己:" + A.id, point: rs.mid };
        }
      }
    }
    return worst || { clearance: Infinity, link: null, obstacle: null, point: null };
  }

  /**
   * アームの全コマ × 全リンク × 全障害物の最短干渉を調べる。
   *   steps      … sequence.js が解いた S (各 {q}) の配列
   *   axes/tool  … kinematics.js の FK に渡す関節軸・工具定義
   *   armFrame   … アーム原点のワールド座標 [x,y,z]
   *   model      … 腕の実体 { links:[{id, joint, cap}], toolLen, toolR }
   *   obstacles  … 障害物の配列。次の2形式を混在できる
   *                 カプセル      {id, a:[x,y,z], b:[x,y,z], r, skipLinks}
   *                 有限円柱(縦)  {id, type:"cylinderY", c:[x,z], r, y0, y1, skipLinks}
   *                skipLinks があれば、そのリンクIDとは比較しない
   *   perStepObstacles(k) … 省略可。コマ k だけに追加する障害物を返す関数
   * 戻り値: { worst:{clearance,frame,link,obstacle,point}, perStep:[{...}] }
   */
  function checkArm(steps, axes, tool, armFrame, model, obstacles, perStepObstacles, perStepBodies, perStepTool) {
    var worst = null;
    var perStep = [];
    for (var k = 0; k < steps.length; k++) {
      var obsK = perStepObstacles ? obstacles.concat(perStepObstacles(k) || []) : obstacles;
      var bodK = perStepBodies ? (perStepBodies(k) || []) : [];
      var tspK = perStepTool ? perStepTool(k) : null;
      // 最終判定なので刻みは細かく(既定)
      var stepWorst = checkPose(steps[k].q, axes, tool, armFrame, model, obsK, 0, bodK, tspK);
      if (stepWorst && stepWorst.link) {
        // ★clearance だけでなく相手とリンク番号も残す。数値1つだけだと
        //   「どこが当たっているのか」が追えず、原因調査ができない
        stepWorst.frame = k;
        perStep.push(stepWorst);
        if (!worst || stepWorst.clearance < worst.clearance) {
          worst = { clearance: stepWorst.clearance, frame: k, link: stepWorst.link,
                     obstacle: stepWorst.obstacle, point: stepWorst.point };
        }
      } else {
        perStep.push({ clearance: Infinity, frame: k, link: null, obstacle: null, point: null });
      }
    }
    return { worst: worst, perStep: perStep };
  }

  return { segSegClosest: segSegClosest, capsuleClearance: capsuleClearance,
           pointToCylinderY: pointToCylinderY, segToCylinderY: segToCylinderY,
           pointToCylinder: pointToCylinder, segToCylinder: segToCylinder,
           pointToObstacle: pointToObstacle, meshMinDistance: meshMinDistance,
           sphereSetFromMesh: sphereSetFromMesh, meshVertices: meshVertices,
           // ★丸め箱の判定体(絶対条件12 / K-37)
           bodySetFromMesh: bodySetFromMesh, bodySetFromMeshExact: bodySetFromMeshExact,
           coarsenBodySet: coarsenBodySet, meshTriangles: meshTriangles,
           sampleTriangles: sampleTriangles, transformParts: transformParts,
           pointToRBox: pointToRBox, rbPair: rbPair, clampToBox: clampToBox,
           obbPenetration: obbPenetration, pointToParts: pointToParts,
           partsPartsClearance: partsPartsClearance,
           partsObstacleClearance: partsObstacleClearance,
           clusterParts: clusterParts, closestOnObstacle: closestOnObstacle,
           rbToObstacle: rbToObstacle, rbPair: rbPair,
           bodySamplePoints: bodySamplePoints,
           transformSpheres: transformSpheres, pointToBody: pointToBody,
           capsuleFromBbox: capsuleFromBbox, armBodies: armBodies,
           pointToOBB: pointToOBB, obbSamples: obbSamples,
           bodyObstacleClearance: bodyObstacleClearance,
           bodyBodyClearance: bodyBodyClearance,
           checkSegment: checkSegment,
    scanSegment: scanSegment,
    repairSegment: repairSegment,
    bodyPoints: bodyPoints,
    checkStatic: checkStatic,
    obstacleObstacleClearance: obstacleObstacleClearance,
    obstacleSamples: obstacleSamples,
    checkPose: checkPose, checkArm: checkArm, setAudit: setAudit,
           checkRigid: checkRigid, tipSide: tipSide };
})();
