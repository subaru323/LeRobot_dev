/* =============================================================================
 * motion.js — SO-101 で「真上からつかむ」ための逆運動学と、動きの組み立て
 *
 * ■ 真上からつかむ姿勢の解き方
 *   SO-101 は腕 5 関節。位置(3)+指先を真下へ(2)= 5 つの条件で、
 *   手首の回転(wrist_roll)の自由が残らない……ように見えるが、指先が真下を向くと
 *   wrist_roll の軸もほぼ真下を向くので、wrist_roll は「ブロックの向き(ヨー)」
 *   だけを変える。そこで
 *     1) wrist_roll を固定して、残り 4 関節で 位置 + 指先の向き を解く(KIN.ik)
 *     2) 爪の開く向き(水平面での角度)を測り、狙いのヨーとの差だけ wrist_roll を回す
 *     3) 道具の点は wrist_roll の軸から 7.9mm ずれているので、1) をやり直す
 *   を 3 回くり返す。
 *
 * ■ 爪の向きは 180° 回しても同じ(2 本の爪で挟むだけ)とみなし、
 *   wrist_roll の可動域の中で前の姿勢に近い方を選ぶ。
 * ========================================================================== */

var MOTION = (function () {
  "use strict";
  var DEG = Math.PI / 180;
  var DOWN = [0, -1, 0];
  var ROLL = 4;                         // wrist_roll の番号

  function wrap180(a) { a = ((a + 180) % 360 + 360) % 360 - 180; return a; }

  /** その姿勢での、爪が開く向きの水平面での角度[度](画面の X から Z へ向かって正) */
  function jawYawDeg(q) {
    var st = KIN.fk(SO101.axes, q, SO101.tool);
    var u = KIN.matVec(st.R, SO101.tool.up);
    return Math.atan2(u[2], u[0]) / DEG;
  }

  /**
   * 真上からつかむ姿勢。
   *   pos … 道具の点を置きたい位置(画面の座標・m)
   *   yawDeg … 爪が開く向き(水平面の角度)。ブロックを挟む向き
   *   q0 … 前の姿勢(近い解を選ぶため)
 *   opt.exact … 爪の向きを 180° 違いで合わせることを許さない(ピースを持っているとき)
   * 戻り値 { q, ok, posErr(m), dirErrDeg, yawErrDeg, reason }
   */
  function topDown(pos, yawDeg, q0, opt) {
    opt = opt || {};
    var lim = SO101.limitsDeg;
    var q = (q0 || [0, 0, 0, 0, 0]).slice();
    var r = null;
    for (var it = 0; it < 4; it++) {
      r = KIN.ikRobust(SO101.axes, SO101.tool, pos, DOWN,
        { q0: q, limitsDeg: lim, lockJoints: [ROLL], posTol: 0.0002, dirTolDeg: 0.3, iters: 200 });
      q = r.q.slice();
      // 爪の向きを合わせる(180° 回しても同じ。可動域の中で今に近い方)
      var d = wrap180(yawDeg - jawYawDeg(q));
      // exact: 爪の向きを 180° 回した解を許さない(持っているピースの向きが変わるため)。
      //        代わりに 1 周(±360°)回した解を候補に入れる
      var cands = opt.exact ? [d, d + 360, d - 360] : [d, d + 180, d - 180];
      var best = null;
      for (var c = 0; c < cands.length; c++) {
        // 指先が下を向いているとき、wrist_roll を +x 回すと爪の水平の向きは
        // ±x 回る(どちらかは姿勢による)。符号は実際に回して測る
        var probe = q.slice(); probe[ROLL] += 1;
        var sgn = wrap180(jawYawDeg(probe) - jawYawDeg(q)) >= 0 ? 1 : -1;
        var nr = q[ROLL] + sgn * cands[c];
        if (nr < lim[ROLL][0] || nr > lim[ROLL][1]) continue;
        var cost = Math.abs(nr - (q0 ? q0[ROLL] : 0));
        if (!best || cost < best.cost) best = { roll: nr, cost: cost };
      }
      if (!best) return { q: q, ok: false, posErr: r.posErr, dirErrDeg: r.dirErrDeg,
                          yawErrDeg: 999, reason: "wrist_roll の可動域で向きを合わせられない" };
      q[ROLL] = best.roll;
    }
    r = KIN.ik(SO101.axes, SO101.tool, pos, DOWN,
      { q0: q, limitsDeg: lim, lockJoints: [ROLL], posTol: 0.0002, dirTolDeg: 0.3, iters: 200 });
    q = r.q.slice();
    var ye = wrap180(yawDeg - jawYawDeg(q));
    ye = opt.exact ? Math.abs(ye) : Math.min(Math.abs(ye), Math.abs(wrap180(ye + 180)));
    var ok = r.posErr < 0.0005 && r.dirErrDeg < 1.0 && ye < 1.0;
    for (var i = 0; i < q.length; i++)
      if (q[i] < lim[i][0] - 1e-6 || q[i] > lim[i][1] + 1e-6) ok = false;
    return { q: q, ok: ok, posErr: r.posErr, dirErrDeg: r.dirErrDeg, yawErrDeg: ye,
             reason: ok ? "" : ("届かない(位置の差 " + (r.posErr * 1000).toFixed(1) +
                                "mm・向きの差 " + r.dirErrDeg.toFixed(1) + "°)") };
  }

  return { topDown: topDown, jawYawDeg: jawYawDeg, wrap180: wrap180 };
})();

if (typeof module !== "undefined") module.exports = MOTION;
