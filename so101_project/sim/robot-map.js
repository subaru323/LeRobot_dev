// sim の URDF の角度 ⇄ 実機(LeRobot 0.6.2 の SOFollower、use_degrees)の角度(計画書_完成形まで の K1 の (2)・V-4)。
//   換算の元は tools/k1_calib.py が書く sim/calib/k1_<id>.json(力を抜いて手で動かして測る)。
//     関節(5 つ): q_urdf = sign × (q_lerobot − offset)。どちらも度。LeRobot の度の 0 は、キャリブレーションで動かした範囲の真ん中。
//     爪: LeRobot は 0〜100。k1_calib.py が測った (LeRobot の値, 爪の間の幅 mm) の組を、sim の爪の形(SCENE_MODEL.gripFor)で
//         URDF の爪の角度に直し、直線で結ぶ(2 組なら 2 点を通る直線、3 組以上なら最小二乗)。
//   範囲: キャリブレーションの範囲(余裕 5°)。手首の回転は ±170(±180 から 10°)、爪は 2〜98。
//   キャリブレーションをやり直すと換算は変わる。calibKey は、キャリブレーションの値を並べた文字列(実機の道具が、いまのモーターの値と比べる)。
var ROBOT_MAP = (function () {
  "use strict";
  var JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"];
  var RES = 4095;                  // sts3215 の 1 周の段の数 − 1(LeRobot の model_resolution_table − 1)

  function calibKey(calib) {
    return JOINTS.concat(["gripper"]).map(function (m) {
      var c = calib[m];
      return m + ":" + [c.id, c.drive_mode, c.homing_offset, c.range_min, c.range_max].join(",");
    }).join(";");
  }

  // k … k1_calib.py の JSON。grip … SCENE_MODEL.gripFor(辺) の返す爪の形(angleFor: 幅[m] → 爪の角度[度])
  function make(k, grip, marginDeg) {
    var margin = marginDeg === undefined ? 5 : marginDeg;
    JOINTS.forEach(function (m) {
      if (!(k.sign[m] === 1 || k.sign[m] === -1)) throw new Error("k1: sign of " + m + " is not ±1");
      if (!isFinite(k.offset[m])) throw new Error("k1: offset of " + m + " is missing");
    });
    // 爪: LeRobot = a + b × 爪の角度
    var gp = (k.gripper || []).map(function (g) { return { L: g.lerobot, ang: grip.angleFor(g.gap_mm / 1000) }; });
    var ga = null, gb = null;
    if (gp.length >= 2) {
      var n = gp.length, sx = 0, sy = 0, sxx = 0, sxy = 0;
      gp.forEach(function (g) { sx += g.ang; sy += g.L; sxx += g.ang * g.ang; sxy += g.ang * g.L; });
      var den = n * sxx - sx * sx;
      if (Math.abs(den) > 1e-9) { gb = (n * sxy - sx * sy) / den; ga = (sy - gb * sx) / n; }
    }
    var lim = {};
    JOINTS.forEach(function (m) {
      var c = k.calibration[m];
      if (m === "wrist_roll") { lim[m] = [-180 + 2 * margin, 180 - 2 * margin]; return; }   // ±180 の近くでは読みが反対の端へ跳ぶ(LeRobot が 0〜4095 に折り返す。点検 C33 の 4)
      var half = (c.range_max - c.range_min) / 2 * 360 / RES;
      lim[m] = [-half + margin, half - margin];
    });
    lim.gripper = [2, 98];
    return {
      joints: JOINTS, limits: lim, calibKey: calibKey(k.calibration), gripperOk: ga !== null,
      // URDF の 5 関節[度] と爪の角度[度] → LeRobot の値 {shoulder_pan: 度, …, gripper: 0〜100}
      toLeRobot: function (q, jawDeg) {
        var o = {};
        JOINTS.forEach(function (m, i) { o[m] = k.offset[m] + q[i] / k.sign[m]; });
        if (jawDeg !== undefined && jawDeg !== null) { if (ga === null) throw new Error("k1: gripper is not measured"); o.gripper = ga + gb * jawDeg; }
        return o;
      },
      // LeRobot の値 → {q: URDF の 5 関節[度], jaw: 爪の角度[度] か null}
      toUrdf: function (L) {
        return { q: JOINTS.map(function (m) { return k.sign[m] * (L[m] - k.offset[m]); }),
                 jaw: L.gripper !== undefined && ga !== null ? (L.gripper - ga) / gb : null };
      },
      // 範囲の外の関節の名前(空なら中)
      outOfRange: function (L) {
        return Object.keys(lim).filter(function (m) { return L[m] !== undefined && (L[m] < lim[m][0] || L[m] > lim[m][1]); });
      }
    };
  }
  return { JOINTS: JOINTS, make: make, calibKey: calibKey };
})();
if (typeof module !== "undefined") module.exports = ROBOT_MAP;
