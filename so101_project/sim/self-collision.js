// 腕どうしの当たり(自分との干渉。計画書_完成形まで の S-10)。作り方はちくわ排除機構の丸め箱をそのまま使う(chikuwa-collision.js = CK_COLLIDE)。
//   ■ 形: 部品のまとまり(base・j1〜j5・jaw・cam)ごとに、STL の三角形(全関節 0 の姿勢の座標)を、一辺 CELL の格子で切り取り、
//     切れ端を包む丸め箱にする(bodySetFromMeshExact。面はまるごと箱の中 = 実物より細くならない。太り方は CELL で決まる)。
//   ■ 測り方: 姿勢 q・爪 jaw で箱を動かし(transformParts)、組ごとに箱どうしの最小の隙間を出す(partsPartsClearance。枝刈りしても総当たりと同じ答え)。
//     隙間が負 = めり込み。箱は実物より太いので、箱で 0 以上なら実物でも当たっていない(安全側)。
//   ■ 見る組: 全部の 2 つの組から、関節でつながる組(ADJ。関節のまわりで触れているのが当たり前)を除く。除いた組は理由つきで AUDIT に出す。
//   ■ コマとコマの間: motionCheck。組ごとに「2 つのコマの隙間の平均 − 縮みうる量の上限」を下限とし、0 未満なら半分に割る(深さ DEPTH まで。割り切れない下限は worstBound)。
//     関節は 2 つのコマの間を直線で動くとする(サーボの追い遅れ・行き過ぎは見ていない)。
//   ■ 箱の太り: まとまり 1 つで ≤ 刻み×√3/2 + 丸め(1.5mm の刻みで 1.8mm)。組の隙間では両側が太るので、実物より最大でその 2 倍(3.6mm)小さく出る。
// 読み込み: kinematics.js・so101.js・models/so101_visuals.js・models/so101_tris.js・chikuwa-collision.js のあと。
var SELF_COLLIDE = (function () {
  "use strict";
  var CELL = 0.004;          // 箱の一辺 [m]。ちくわの「接触する物」の刻み(CELL_TOUCH_MM = 4)と同じ。まず全部の組をこれで見る
  var FINE = 0.0015;         // 近い組(粗い箱で WARN 未満)だけ、この刻みで見直す。箱の太り ≤ FINE×√3/2 + 丸め = 1.8mm
  //   刻みと太りの実測(2026-11-17。爪とカメラ台・全関節 0。点で測った距離 1.45mm は実物より長めに出る): 4mm −0.59 / 3mm −0.78 / 2mm −0.28 / 1.5mm 0.00 / 1mm −0.02
  var ROUND_MM = 0.5;        // 丸め [mm]。数値の安全代だけ(ちくわの既定)
  var WARN = 0.003;          // これより近いと「近い」[m]【仮】(サーボの追い遅れの見込み)
  var GROUPS = ["base", "j1", "j2", "j3", "j4", "j5", "jaw", "cam"];
  var ADJ = { "base|j1": "関節 1 でつながる", "j1|j2": "関節 2 でつながる", "j2|j3": "関節 3 でつながる", "j3|j4": "関節 4 でつながる",
              "j4|j5": "関節 5 でつながる", "j5|jaw": "爪の関節でつながる", "j5|cam": "手首のカメラ台はねじで固定(剛体。距離が一定)",
              // ちくわの checkRigid の考え: 1 つの関節だけで距離が決まる組は、コマごとに見ず、その関節の可動域の全体で 1 度だけ見る(staticCheck)
              "jaw|cam": "爪の角度だけで距離が決まる(どちらも手首にのる)。爪の可動域の全体で staticCheck が見る",
              "j4|cam": "手首の回転(関節 5)だけで距離が決まる。関節 5 の可動域の全体で staticCheck が見る" };
  var STATIC = [{ pair: ["jaw", "cam"], joint: "jaw" }, { pair: ["j4", "cam"], joint: 4 }];
  var PAIRS = [], AUDIT = [];
  for (var i = 0; i < GROUPS.length; i++) for (var j = i + 1; j < GROUPS.length; j++) {
    var k = GROUPS[i] + "|" + GROUPS[j];
    if (ADJ[k]) AUDIT.push({ pair: k, skip: ADJ[k] }); else { PAIRS.push([GROUPS[i], GROUPS[j]]); AUDIT.push({ pair: k, skip: null }); }
  }
  var SETS = null, FSETS = null, TRIS = null;
  function groupOf(link) { return SO101.LINK_GROUP[link] || (SO101.CAMERA_GROUP[link] ? "cam" : null); }
  // まとまりごとの三角形(全関節 0 の姿勢の座標。[[a,b,c],...])
  function groupTris() {
    if (TRIS) return TRIS;
    TRIS = {}; GROUPS.forEach(function (g) { TRIS[g] = []; });
    Object.keys(SO101_VISUALS).forEach(function (link) {
      var g = groupOf(link); if (!g) return;
      SO101_VISUALS[link].forEach(function (v) {
        var T = SO101_TRIS[v.mesh];
        if (!T) throw new Error("三角形が無い: " + v.mesh + "(python tools/build-models.py を流す)");
        var P = SO101.visualPose(link, v.xyz, v.rpy);
        for (var a = 0; a < T.length; a += 9) {
          var t = [];
          for (var m = 0; m < 3; m++) {
            var w = SO101.mv(P.R, [T[a + 3 * m] * 1e-4, T[a + 3 * m + 1] * 1e-4, T[a + 3 * m + 2] * 1e-4]);
            t.push([w[0] + P.p[0], w[1] + P.p[1], w[2] + P.p[2]]);
          }
          TRIS[g].push(t);
        }
      });
    });
    return TRIS;
  }
  function bbox(tris) {
    var b = [Infinity, Infinity, Infinity, -Infinity, -Infinity, -Infinity];
    tris.forEach(function (t) { t.forEach(function (p) { for (var a = 0; a < 3; a++) { if (p[a] < b[a]) b[a] = p[a]; if (p[a] > b[a + 3]) b[a + 3] = p[a]; } }); });
    return b;
  }
  function buildOne(g, cell) { var T = groupTris(); return CK_COLLIDE.bodySetFromMeshExact({ tris: T[g], bbox: bbox(T[g]) }, cell || CELL, ROUND_MM); }
  function build(cell) {
    var out = {};
    GROUPS.forEach(function (g) { out[g] = buildOne(g, cell); });
    return out;
  }
  // まとまりごとに、初めて使うときに作る(作り方は build と同じ。立ち上げを速くするため。2026-11-17)。
  //   既定の画面では、細かい箱(FINE)を要るのは近い組のまとまりだけで、base などは作らずに済む
  function lazySets(make) {
    var out = {}, made = {};
    GROUPS.forEach(function (g) {
      Object.defineProperty(out, g, { enumerable: true, get: function () { return made[g] || (made[g] = make(g)); } });
    });
    return out;
  }
  function sets() { if (!SETS) SETS = lazySets(function (g) { return buildOne(g, CELL); }); return SETS; }
  function fineSets() { if (!FSETS) FSETS = lazySets(function (g) { return buildOne(g, FINE); }); return FSETS; }
  // 枝刈り用のいちばん粗い箱(ちくわの coarsenBodySet: 粗い箱 ⊇ 細かい箱 が作り方で決まる)
  var COARSE = 0.012, CSETS = null;
  function coarseSets() { if (!CSETS) CSETS = lazySets(function (g) { return CK_COLLIDE.coarsenBodySet(sets()[g], COARSE); }); return CSETS; }
  // まとまり g の、姿勢での置き方(世界 = p + R(x - o))
  function placeOf(st, g, jaw) {
    if (g === "base") return { R: null, p: [0, 0, 0], o: [0, 0, 0] };
    var fr = st.frames[(g === "jaw" || g === "cam") ? 4 : (+g.slice(1) - 1)];
    if (g !== "jaw") return { R: fr.R, p: fr.p, o: fr.o };
    // 爪: x0 → jawRotate(x0) = oj + Rj(x0 - oj) → 手首の座標へ。世界 = fr.p + R(oj - fr.o) + R Rj (x0 - oj)
    var oj = SO101.jaw.origin, Rj = KIN.rotAA(SO101.jaw.axis, jaw * Math.PI / 180), d = KIN.matVec(fr.R, [oj[0] - fr.o[0], oj[1] - fr.o[1], oj[2] - fr.o[2]]);
    return { R: KIN.matMul(fr.R, Rj), p: [fr.p[0] + d[0], fr.p[1] + d[1], fr.p[2] + d[2]], o: oj };
  }
  function placed(q, jaw, S) {
    S = S || sets();
    var st = KIN.fk(SO101.axes, q, SO101.tool), W = {};
    GROUPS.forEach(function (g) { var P = placeOf(st, g, jaw); W[g] = CK_COLLIDE.transformParts(S[g], P.R, P.p, P.o, [0, 0, 0]); });
    return W;
  }
  // 姿勢 1 つ: 組ごとの隙間 [m] と、いちばん近い組。S を渡すとその刻みだけで見る(見直しをしない)。
  //   渡さないとき: 3 段で見る。粗い箱(COARSE 12mm)→ 箱(CELL 4mm)→ 細かい箱(FINE 1.5mm)。WARN 未満の組だけ次の段へ進む。
  //   どの段の箱も実物を包むので、どの段の隙間も「実物の隙間の下限」。組の答えは、見た段の下限のいちばん大きいもの。
  // 同じ姿勢を何度も測らないよう、箱の段を渡さないときの答えを覚える(試し方ごとの motionCheck と、動き全体の motionCheck が
  //   同じコマ・同じ割った点を測る。答えは姿勢だけで決まるので、覚えても結果は変わらない。2026-11-17)。戻りは読むだけで使う
  var MEMO = new Map(), MEMO_MAX = 20000, memoHit = 0;   // 既定の画面で 634 姿勢。道具が何度も組むときは、上限で捨てて作り直す(メモリを抑える)
  function pose(q, jaw, S) {
    if (S) return poseRaw(q, jaw, S);
    var key = q[0] + "," + q[1] + "," + q[2] + "," + q[3] + "," + q[4] + "," + jaw, r = MEMO.get(key);
    if (r) { memoHit++; return r; }
    if (MEMO.size >= MEMO_MAX) MEMO.clear();
    r = poseRaw(q, jaw); Object.freeze(r.byPair); Object.freeze(r); MEMO.set(key, r);   // 同じ物を何度も渡すので、書き換えられないように
    return r;
  }
  function poseRaw(q, jaw, S) {
    var byPair = {}, worst = { clearance: Infinity, pair: null };
    var levels = S ? [S] : [coarseSets(), sets(), fineSets()], Wl = levels.map(function () { return {}; });
    var st = KIN.fk(SO101.axes, q, SO101.tool), PL = {};
    function get(L, g) {             // まとまり g を段 L の箱で置く(要るときだけ)
      if (!Wl[L][g]) { var P = PL[g] || (PL[g] = placeOf(st, g, jaw)); Wl[L][g] = CK_COLLIDE.transformParts(levels[L][g], P.R, P.p, P.o, [0, 0, 0]); }
      return Wl[L][g];
    }
    PAIRS.forEach(function (pr) {
      var k = pr[0] + "|" + pr[1], r = null, best = -Infinity;
      for (var L = 0; L < levels.length; L++) {
        var rr = CK_COLLIDE.partsPartsClearance(get(L, pr[0]), get(L, pr[1]));
        if (rr.clearance > best) { best = rr.clearance; r = rr; }
        if (best >= WARN) break;
      }
      byPair[k] = best;
      if (r.clearance < worst.clearance) worst = { clearance: r.clearance, pair: k, mid: r.mid };
    });
    worst.byPair = byPair;
    return worst;
  }
  // 2 つのコマの間で、組 (a, b) の隙間が縮みうる量の上限 [m]。a から見た b の動きは、a と b の間の関節だけで決まる(肩の回転は効かない)。
  //   関節 k が Δq_k 回ると、b の点は (関節 k の原点から b の点までの距離の上限) × |Δq_k| まで動く。距離の上限 = 原点どうしの距離を b の原点まで足し、b の原点から b のいちばん遠い点までを足す
  //   (間の関節がどう曲がっても超えない)。爪は爪の関節も足す。
  var IDX = { base: 0, j1: 1, j2: 2, j3: 3, j4: 4, j5: 5, jaw: 6, cam: 6 }, LEV = null;
  function levers() {
    if (LEV) return LEV;
    var T = groupTris(), st = KIN.fk(SO101.axes, [0, 0, 0, 0, 0], SO101.tool), o = [null].concat(st.frames.map(function (f) { return f.p; }));   // o[k] = 関節 k の原点(全関節 0)
    var oj = SO101.jaw.origin;
    function d(a, b) { return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]); }
    function own(g) { return g === "jaw" ? oj : g === "cam" ? o[5] : o[IDX[g]]; }
    var ext = {}; GROUPS.forEach(function (g) { if (g === "base") return; var c = own(g), m = 0; T[g].forEach(function (t) { t.forEach(function (x) { m = Math.max(m, d(x, c)); }); }); ext[g] = m; });
    // 関節 k の原点から、まとまり g の原点までを鎖でたどった長さ
    function chain(k, g) {
      var s2 = 0, last = Math.min(IDX[g], 5);
      for (var m = k; m < last; m++) s2 += d(o[m], o[m + 1]);
      if (g === "jaw") s2 += d(o[5], oj);
      return s2;
    }
    var R = {};
    PAIRS.forEach(function (pr) {
      var a = pr[0], b = pr[1]; if (IDX[a] > IDX[b]) { var tmp = a; a = b; b = tmp; }
      var r = [0, 0, 0, 0, 0, 0];
      for (var k = IDX[a] + 1; k <= Math.min(IDX[b], 5); k++) r[k - 1] = chain(k, b) + ext[b];
      if (b === "jaw") r[5] = ext.jaw;
      R[pr[0] + "|" + pr[1]] = r;
    });
    LEV = R;
    return LEV;
  }
  function pairBound(key, qA, jA, qB, jB) {
    var r = levers()[key], s2 = r[5] * Math.abs(jB - jA);
    for (var i = 0; i < 5; i++) s2 += r[i] * Math.abs(qB[i] - qA[i]);
    return s2 * Math.PI / 180;
  }
  // 動きの全コマ + コマの間: コマで測り、間は組ごとに「2 つのコマの隙間の平均 − 縮みうる量の上限」で下限を出す。
  //   下限が 0 未満の組があれば真ん中で割って測り直す(深さ DEPTH まで。割り切れずに残った下限は worstBound に出す)。どちらかのコマですでに当たっていれば割らない。
  //   frames: [{q, jaw, t}]。戻り {min, pair, t, q, nPose, worstBound, boundAt}
  var DEPTH = 6;
  function motionCheck(frames, every) {
    every = every || 1;
    var nPose = 0, res = { min: Infinity, pair: null, t: 0, q: null, worstBound: Infinity, boundAt: null, boundPair: null };
    function look(q, jaw, t) { var r = pose(q, jaw); nPose++; if (r.clearance < res.min) { res.min = r.clearance; res.pair = r.pair; res.t = t; res.q = q.slice(); } return r; }
    function seg(qA, jA, tA, rA, qB, jB, tB, rB, depth) {
      if (rA.clearance < 0 || rB.clearance < 0) return;
      var lb = Infinity, lbPair = null;
      PAIRS.forEach(function (pr) { var k = pr[0] + "|" + pr[1], v = (rA.byPair[k] + rB.byPair[k]) / 2 - pairBound(k, qA, jA, qB, jB); if (v < lb) { lb = v; lbPair = k; } });
      if (lb >= 0) return;
      if (depth >= DEPTH) { if (lb < res.worstBound) { res.worstBound = lb; res.boundAt = tA; res.boundPair = lbPair; } return; }
      var qM = qA.map(function (x, i) { return (x + qB[i]) / 2; }), jM = (jA + jB) / 2, tM = (tA + tB) / 2, rM = look(qM, jM, tM);
      seg(qA, jA, tA, rA, qM, jM, tM, rM, depth + 1); seg(qM, jM, tM, rM, qB, jB, tB, rB, depth + 1);
    }
    var prev = null;
    for (var k = 0; k < frames.length; k += every) {
      var f = frames[k], r = look(f.q, f.jaw, f.t);
      if (prev) seg(prev.q, prev.jaw, prev.t, prev.r, f.q, f.jaw, f.t, r, 0);
      prev = { q: f.q, jaw: f.jaw, t: f.t, r: r };
    }
    res.nPose = nPose;
    return res;
  }
  // 1 つの関節だけで距離が決まる組(STATIC)を、その関節の可動域の全体で見る(細かい箱。1° 刻み)
  function staticCheck() {
    var lim = SO101.limitsDeg, out = [];
    STATIC.forEach(function (s) {
      var lo, hi, worst = { clearance: Infinity, at: null };
      if (s.joint === "jaw") { lo = SO101.jaw.limitsDeg[0]; hi = SO101.jaw.limitsDeg[1]; }
      else { lo = lim[s.joint][0]; hi = lim[s.joint][1]; }
      for (var a = Math.ceil(lo); a <= Math.floor(hi); a++) {
        var q = [0, 0, 0, 0, 0], jaw = 0;
        if (s.joint === "jaw") jaw = a; else q[s.joint] = a;
        var W = placed(q, jaw, fineSets()), r = CK_COLLIDE.partsPartsClearance(W[s.pair[0]], W[s.pair[1]]);
        if (r.clearance < worst.clearance) worst = { clearance: r.clearance, at: a };
      }
      out.push({ pair: s.pair.join("|"), joint: s.joint, range: [lo, hi], clearance: worst.clearance, at: worst.at });
    });
    return out;
  }
  return { GROUPS: GROUPS, STATIC: STATIC, FINE: FINE, fineSets: fineSets, staticCheck: staticCheck, PAIRS: PAIRS, ADJ: ADJ, AUDIT: AUDIT, CELL: CELL, WARN: WARN,
           groupTris: groupTris, build: build, sets: sets, coarseSets: coarseSets, COARSE: COARSE, levers: levers, pairBound: pairBound, motionCheck: motionCheck, DEPTH: DEPTH, placeOf: placeOf, placed: placed, pose: pose, poseRaw: poseRaw,
           memoStats: function () { return { size: MEMO.size, hit: memoHit }; },
           clearMemo: function () { MEMO.clear(); } };   // 形(箱・道具・爪)を実行中に変える検算では、変えたあとに呼ぶ(鍵は関節角だけ)
})();
if (typeof module !== "undefined") module.exports = SELF_COLLIDE;
