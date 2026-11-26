/* =============================================================================
 * collision-view.js — 当たり判定の形状を「見える部品」に変換する
 *
 * ■ なぜ要るか
 *   干渉判定は数値でしか出てこないので、
 *   「腕が実在しない場所に当たり判定を持っている」「バーが縦向きに描画され
 *   判定は水平だった」といった★絵と判定のズレが起きても気づけない。
 *   実際そのたびに検討結果が無効になった。
 *   → 判定に使っている形状そのものを、同じ変換で画面に出す。
 *
 * ■ 使い方
 *   scene-data が SCENE_MODEL.probe(ui) で
 *     { axes, armFrame, armModel, obstacles, ... }
 *   を返せば、それだけで当たり判定を描ける。★scene-data 側の改造は不要。
 *
 *   var parts = COLLISION_VIEW.parts(probe);   // ビューアの parts に足すだけ
 *
 * ■ 親子関係の約束(ビューアと同じ置き方をすること。ここが違うとまたズレる)
 *     リンク i のメッシュ … 親 "j{i}"、位置 -JOINT_AXES[i-1].origin
 *   なので CAD 座標の点 v は 親 "j{i}"、位置 v - origin に置けば一致する。
 *   工具は "toolfrm" の子で、その +Y が工具軸。
 *
 * ■ 出す部品の id は必ず "cv_" で始める。ビューアはこれで表示を切り替える。
 *
 * ■ ★各部品は hit:"<判定体の名前>" を持つ(絶対条件7)。
 *   ビューアはこれを使って「その時刻に干渉している判定体」だけを赤くする。
 *   ★判定体の名前は checkPose が返す link / obstacle の名前と**同じ文字列**。
 *   ここがずれると、赤くなる場所が嘘になる。
 * ========================================================================== */

var COLLISION_VIEW = (function () {
  "use strict";

  var COL = {
    link:  "#2b6de8",   // 腕のリンク(球の集合)
    tool:  "#e8734a",   // 工具
    work:  "#8e44ad",   // 把持中のワーク
    obs:   "#e74c3c",   // 障害物
    floor: "#7f8c8d"
  };
  var OPA = 0.30;
  // ★リンクの球は1リンクあたり数十個が重なるので、薄くしないと中が見えない。
  //   球近似は谷や角で膨らむ(実測 -24mm と出た箇所が実頂点では 0.0mm だった)。
  //   ここで「実物よりかなり太い」ことが目で見えるのが、この表示の狙い。
  var OPA_LINK = 0.13;

  function sub(a,b){ return [a[0]-b[0],a[1]-b[1],a[2]-b[2]]; }

  /** 円柱の軸(+Y)を向き d に合わせる軸角回転 */
  function alignYTo(d) {
    var n = Math.sqrt(d[0]*d[0]+d[1]*d[1]+d[2]*d[2]) || 1;
    var x=d[0]/n, y=d[1]/n, z=d[2]/n;
    var ax=[z,0,-x], L=Math.sqrt(ax[0]*ax[0]+ax[2]*ax[2]);
    if (L < 1e-9) return { axis:[1,0,0], angleDeg:(y>=0?0:180) };
    return { axis:[ax[0]/L,0,ax[2]/L],
             angleDeg: Math.acos(Math.max(-1,Math.min(1,y)))*180/Math.PI };
  }

  /**
   * probe = { axes, armFrame, armModel:{links, toolParts|handParts}, obstacles, ... }
   * opt   = { tool: "hand"|"cam"|配列, work: 把持中ワークのボディ }
   */
  // ★★判定体が「面に沿った箱」(R0 を持つ)になったので、絵も同じ向きで描く(2026-09-20。条件12 段B / 0-E)
  //   ★ここを直さないと、判定は傾いた薄板・絵は軸に平行な箱 = ★**別の世界**になる。
  //   ★R0 の列が箱の軸(局所 → リンク)。ビューアは three.js の XYZ オイラー(度)で受ける
  function eulerXYZFromR(R) {
    var m11 = R[0], m12 = R[1], m13 = R[2], m23 = R[5], m33 = R[8];
    var y = Math.asin(Math.max(-1, Math.min(1, m13)));
    var x, z;
    if (Math.abs(m13) < 0.9999999) { x = Math.atan2(-m23, m33); z = Math.atan2(-m12, m11); }
    else { x = Math.atan2(R[7], R[4]); z = 0; }
    var d = 180 / Math.PI;
    return [x * d, y * d, z * d];
  }
  /** ★裏返して持つワーク用: F = diag(1,-1,-1)(X まわり 180°)で挟む */
  function flipR(R) {
    var F = [1,0,0, 0,-1,0, 0,0,-1], T = new Array(9), O = new Array(9);
    for (var i = 0; i < 3; i++) for (var j = 0; j < 3; j++)
      T[i*3+j] = F[i*3]*R[j] + F[i*3+1]*R[3+j] + F[i*3+2]*R[6+j];
    for (i = 0; i < 3; i++) for (j = 0; j < 3; j++)
      O[i*3+j] = T[i*3]*F[j] + T[i*3+1]*F[3+j] + T[i*3+2]*F[6+j];
    return O;
  }
  function parts(probe, opt) {
    opt = opt || {};
    var out = [];
    if (!probe) return out;
    var axes = probe.axes || [];

    // ---------------------------------------------------------------------
    // 1. 腕のリンク(球の集合)
    //    ★半径は「外接球」ではなく実頂点までの最大距離。
    //      外接にすると触れてもいない腕どうしが -1〜-4mm の干渉として出る
    // ---------------------------------------------------------------------
    var links = (probe.armModel && probe.armModel.links) || [];
    links.forEach(function (L) {
      if (!L.set) return;
      var j = L.joint;                       // -1 なら架台固定(arm_root の子)
      var parent = (j < 0) ? "arm_root" : ("j" + (j + 1));
      var o = (j < 0) ? [0,0,0] : axes[j].origin;
      // ★丸め箱の集合(K-37)。判定と同じ形をそのまま出す。
      //   丸み(r)は箱の外側に付くので、絵では芯を r だけ太らせた箱で代用する。
      //   ★代用なので、角は実際の判定よりわずかに大きく見える(丸みが描けない)。
      //   形の一致を数字で見るのは tools/shape-fidelity.js の仕事(絶対条件12)。
      if (L.set.parts) {
        L.set.parts.forEach(function (e, i) {
          out.push({ id: "cv_" + L.id + "_" + i, parent: parent, shape: "box",
                     hit: L.id,
                     color: COL.link, opacity: OPA_LINK, noShadow: true,
                     size: [2*(e.half[0]+e.r), 2*(e.half[1]+e.r), 2*(e.half[2]+e.r)],
                     rotationDeg: e.R0 ? eulerXYZFromR(e.R0) : undefined,   // ★判定と同じ向き(0-E)
                     position: sub(e.c, o) });
        });
        return;
      }
      if (!L.set.spheres) return;
      L.set.spheres.forEach(function (s, i) {
        out.push({ id: "cv_" + L.id + "_" + i, parent: parent, shape: "sphere",
                   hit: L.id,
                   color: COL.link, opacity: OPA_LINK, noShadow: true,
                   size: [s.r], position: sub(s.c, o) });
      });
    });

    // ---------------------------------------------------------------------
    // 2. 工具(フランジから先)。toolfrm の +Y が工具軸
    //    ★爪は PCD 上の細いピン3本。1本の太い円柱にすると3〜4倍太くなる
    // ---------------------------------------------------------------------
    var tp = opt.tool;
    if (!tp && probe.armModel) {
      tp = probe.armModel.handParts ||
           (probe.armModel.toolParts && (probe.armModel.toolParts.hand ||
            probe.armModel.toolParts[Object.keys(probe.armModel.toolParts)[0]]));
    }
    if (Array.isArray(tp)) {
      tp.forEach(function (P, i) {
        var mid = ((P.from || 0) + (P.to || 0)) / 2;
        var h = Math.abs((P.to || 0) - (P.from || 0)) || 0.001;
        if (P.ring) {
          for (var k = 0; k < P.ring.n; k++) {
            var a = k * 2 * Math.PI / P.ring.n;
            // ★判定側は (pt.id || "爪") + (ci+1) という名前で1本ずつ持つ
            if (P.ring.box) {
              // ★板のときは絵も箱。判定と違う形を描くと、
              //   ずれていても目で気づけない(条件7 の意味が無くなる)。
              out.push({ id: "cv_tool" + i + "_" + k, parent: "toolfrm", shape: "box",
                         hit: (P.id || "爪") + (k + 1),
                         color: COL.tool, opacity: OPA, noShadow: true,
                         size: [P.ring.box[0]*2, h, P.ring.box[1]*2],
                         rotationDeg: [0, -a * 180 / Math.PI, 0],
                         position: [Math.cos(a)*P.ring.pcd/2, mid, Math.sin(a)*P.ring.pcd/2] });
              continue;
            }
            out.push({ id: "cv_tool" + i + "_" + k, parent: "toolfrm", shape: "cylinder",
                       hit: (P.id || "爪") + (k + 1),
                       color: COL.tool, opacity: OPA, noShadow: true,
                       size: [P.ring.pinR, P.ring.pinR, h, 10],
                       position: [Math.cos(a)*P.ring.pcd/2, mid, Math.sin(a)*P.ring.pcd/2] });
          }
        } else if (P.bar) {
          // 横へ張り出すバー照明。★縦向きに描いて判定は水平、を一度やっている
          out.push({ id: "cv_tool" + i, parent: "toolfrm", shape: "cylinder",
                     hit: P.id || "バー照明",
                     color: COL.tool, opacity: OPA, noShadow: true,
                     size: [P.bar.r, P.bar.r, P.bar.len, 12],
                     position: [P.bar.side || 0, P.bar.at || 0, 0],
                     rotationDeg: [0, 0, 90] });
        } else {
          out.push({ id: "cv_tool" + i, parent: "toolfrm", shape: "cylinder",
                     hit: P.id || ("tool" + i),
                     color: COL.tool, opacity: OPA, noShadow: true,
                     size: [P.r, P.r, h, 14], position: [0, mid, 0] });
        }
      });
    }

    // ---------------------------------------------------------------------
    // 3. 障害物(ワールド固定)
    // ---------------------------------------------------------------------
    (probe.obstacles || []).concat(probe.wheelObs ? [probe.wheelObs] : [])
      .forEach(function (ob, i) {
      if (!ob) return;
      var id = "cv_obs" + i;
      var hid = ob.id;                 // ★判定側の名前。これで赤くする対象を選ぶ
      if (ob.type === "planeY") {
        out.push({ id: id, hit: hid, shape: "box", color: COL.floor, opacity: 0.18, noShadow: true,
                   size: [6, 0.004, 6], position: [0, ob.y || 0, 0] });
      } else if (ob.type === "cylinderY") {
        var h = (ob.y1 - ob.y0);
        out.push({ id: id, hit: hid, shape: "cylinder", color: COL.obs, opacity: OPA, noShadow: true,
                   size: [ob.r, ob.r, h, 24], position: [ob.c[0], ob.y0 + h/2, ob.c[1]] });
      } else if (ob.type === "cylinder") {
        var r2 = alignYTo(ob.d);
        out.push({ id: id, hit: hid, shape: "cylinder", color: COL.obs, opacity: OPA, noShadow: true,
                   size: [ob.r, ob.r, ob.h * 2, 24], position: ob.c,
                   axis: r2.axis, angleDeg: r2.angleDeg });
      } else if ((ob.type === "box" || ob.type === "obb") && (ob.size || ob.half)) {
        // ★half 形式(実際の判定が使っている形)に対応していなかった。
        //   size しか見ていなかったので、受け台・支柱は**一つも描かれない**。
        var sz = ob.size || [ob.half[0]*2, ob.half[1]*2, ob.half[2]*2];
        var pp = { id: id, hit: hid, shape: "box", color: COL.obs, opacity: OPA, noShadow: true,
                   size: sz, position: ob.c || [0,0,0] };
        if (ob.yaw) pp.rotationDeg = [0, ob.yaw * 180 / Math.PI, 0];
        out.push(pp);
      } else if (ob.parts && ob.parts.length) {
        // ★★実形状(丸め箱の集合)で置いてある静止物 — ①の MODE 1 の「ワーク」や、
        //   台の上に置いたワーク(wobs)がこれ(`type:"parts"`。`COLLIDE.transformParts` の出力)。
        //   ★2026-09-20 まで描けておらず、下の `_unknown`(名前を持たない赤い球)に落ちていた。
        //   = ★**その物に当たっても赤くならない**(条件7 の後半が成立しない)。
        //   実測: `hit-view-check` MODE 1 が「時刻表にあるのに絵に無い 1 件: ワーク」を出した(K-114 残り a)。
        //   ★要素の中心 c と向き R は**すでに世界座標**(transformParts が変換済み)。親を付けない。
        //   ★丸み(r)は箱の外側に付くので、芯を r だけ太らせた箱で代用する(把持中ワークと同じ描き方)。
        for (var si = 0; si < ob.parts.length; si++) {
          var se = ob.parts[si];
          out.push({ id: id + "_" + si, hit: hid, shape: "box",
                     color: COL.obs, opacity: OPA, noShadow: true,
                     size: [2*(se.half[0]+se.r), 2*(se.half[1]+se.r), 2*(se.half[2]+se.r)],
                     rotationDeg: se.R ? eulerXYZFromR(se.R) : undefined,   // ★判定と同じ向き(0-E)
                     position: [se.c[0], se.c[1], se.c[2]] });
        }
      } else if (ob.a && ob.b) {
        // ★カプセル(機器を支柱に繋ぐ腕など)。これも描けていなかった。
        var rr = alignYTo([ob.b[0]-ob.a[0], ob.b[1]-ob.a[1], ob.b[2]-ob.a[2]]);
        var ln = Math.sqrt(Math.pow(ob.b[0]-ob.a[0],2)+Math.pow(ob.b[1]-ob.a[1],2)+Math.pow(ob.b[2]-ob.a[2],2));
        out.push({ id: id, hit: hid, shape: "cylinder", color: COL.obs, opacity: OPA, noShadow: true,
                   size: [ob.r, ob.r, ln, 12],
                   position: [(ob.a[0]+ob.b[0])/2, (ob.a[1]+ob.b[1])/2, (ob.a[2]+ob.b[2])/2],
                   axis: rr.axis, angleDeg: rr.angleDeg });
      } else {
        // ★描けない形は**黙って飛ばさない**。目に見える印を出す。
        out.push({ id: id + "_unknown", shape: "sphere", color: [1,0,0], opacity: 0.6,
                   noShadow: true, size: [0.03, 12, 12], position: ob.c || [0,0,0] });
      }
    });

    // ---------------------------------------------------------------------
    // 4. 把持中のワーク(腕と一緒に動く物体。障害物ではない)
    //    ★工具フレームに固定なので toolfrm の子として置く
    // ---------------------------------------------------------------------
    if (opt.work) {
      var w = opt.work;
      if (w.set && w.set.parts) {
        // ★実形状(丸め箱の集合)で持っているワーク。ホイールのように
        //   中央が開いている物は、円柱で描くと**判定と絵が別物**になる。
        //   → 判定が使っている要素をそのまま描く(絶対条件7 / 0-E)。
        //
        //   ★工具フレームでの位置は、判定と同じ式で出す:
        //     flip(把持A) … ローカル +Y が工具の -Y を向く(Rx180)
        //       [x, off - y, -z]
        //     それ以外     … [x, off + y, z]
        //   ★この式を判定側(heldBody)と違う形で書かないこと。ずれても
        //     どちらも単体では正しく見える(0-E)。
        var els = w.set.parts, off = w.off || 0, flip = !!w.flip;
        for (var wi = 0; wi < els.length; wi++) {
          var e = els[wi];
          out.push({ id: "cv_work_" + wi, parent: "toolfrm", shape: "box",
                     hit: w.id || "ワーク(把持中)",
                     color: COL.work, opacity: 0.28, noShadow: true,
                     size: [2*(e.half[0]+e.r), 2*(e.half[1]+e.r), 2*(e.half[2]+e.r)],
                     // ★判定と同じ向き(0-E)。裏返して持つときは向きも裏返す
                     rotationDeg: e.R0 ? eulerXYZFromR(flip ? flipR(e.R0) : e.R0) : undefined,
                     position: flip ? [ e.c[0], off - e.c[1], -e.c[2] ]
                                    : [ e.c[0], off + e.c[1],  e.c[2] ] });
        }
      } else if (w.half) {
        // ★立方体の把持中ワーク。立方体なので向きは結果に影響しない。
        //   ★立方体以外にするなら、向きを渡して回すこと
        out.push({ id: "cv_work", parent: "toolfrm", shape: "box",
                   hit: w.id || "ワーク(把持中)",
                   color: COL.work, opacity: 0.28, noShadow: true,
                   size: [w.half[0]*2, w.half[1]*2, w.half[2]*2], position: [0, w.mid, 0] });
      } else {
        out.push({ id: "cv_work", parent: "toolfrm", shape: "cylinder",
                   hit: w.id || "ワーク(把持中)",
                   color: COL.work, opacity: 0.28, noShadow: true,
                   size: [w.r, w.r, w.h * 2, 28], position: [0, w.mid, 0] });
      }
    }
    return out;
  }

  return { parts: parts, colors: COL };
})();
