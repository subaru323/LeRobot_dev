/* =============================================================================
 * scene-data.js — SO-101 がピースを並べて形にする動き(画面のデータ)
 *
 * ■ 作りの決まり(ちくわの 0-E: 絵と判定は同じ世界を見る)
 *   - 腕の関節の位置と軸は so101.js(URDF の値)だけから作る。ここに寸法を写さない。
 *   - 腕の形は公式の STL(models/so101_meshes.js)。当たり判定の点(models/so101_points.js)も
 *     同じ STL から同じ回に作ってある(tools/build-models.py)。
 *   - 爪の寸法(固定の爪の内側の面・指先・指の厚さ・すき間 → 開く角度)は、★その点から測る。
 *     手で決めた爪の数字は持たない。
 *   - 持っているピースの位置は、そのコマの関節角から URDF の行列の積(urdfFK)で作る。
 *
 * ■ 【仮の値】(利用者の確認待ち。計画書の前提 U-3・U-4)
 *   CELL(既定は爪の形から決める 14mm)/ 指の両側の余裕 2mm / つかむ高さ 指先の長さ + 3mm(机から 11.5mm)/ 運ぶ高さ 机 + 50mm /
 *   速さ 関節 90°/s・直線 150mm/s / 上のカメラの台の置き方(so101.js の OVERHEAD)
 *
 * ■ 机と腕の高さ: 机の上の面を高さ 0 にする。腕の原点(base_link)は、土台の底のリブの分 2.4mm 上、
 *   台を敷くとさらに板の 7.2mm 上(ROBOT_Y)。逆運動学には腕の座標(y − ROBOT_Y)で渡す。
 *   (2026-11-16 まで、机を原点の高さに置いていた。点検 C9 で分かって直した)
 *
 * ■ 爪は片方が固定。挟む幅 w のピースの真ん中は、道具の点から (固定の爪の面 + w/2) だけ
 *   動く爪の側にずれる。動きの目標(道具の点)も、持っているピースの位置も、この関係で作る。
 *
 * ■ つかみ方・離し方
 *   つかむ: 爪を「幅 + 余裕 2 × 2mm」に開き、固定の爪をピースから余裕 2mm 離して真下へ下ろす
 *          → 横に 2mm 寄せて固定の爪を当てる → 閉じる → 持ち上げる。
 *   離す:   置いたら「幅 + 余裕 2 × 2mm」まで開く → 横に 2mm 戻す → 持ち上げる。
 *   だから、隣のマスが埋まっていても、すき間 ≥ 指の厚さ + 余裕 2 × 2mm なら指が入る。
 *   すき間 0(詰めて並べる)では、最後のピースのまわりが埋まっていて指が入らない
 *   (真上から 2 本の爪で挟む限り、2×2 以上の詰まった形なら必ず起きる)。
 * ========================================================================== */

var SCENE_MODEL = (function () {
  "use strict";
  var DEG = Math.PI / 180;

  // ---- 【仮の値】 ----
  var CELL = 0.025;               // ブロック 1 個の辺(画面の欄で変える。既定は下の「2 マスを挟める いちばん大きい辺」)
  var FIT_MARGIN_DEFAULT = 0.002; // 指の両側の余裕
  var FIT_MARGIN = FIT_MARGIN_DEFAULT;   // build ごとに ui._FIT_MARGIN(mm)で変えられる(tools/allow-err.js 用。点検 C37 の頼み、2026-11-18)。
                                         //   ★build の外で gripFor を呼ぶ道具は、最後の build の値を使う(既定の build の後なら 2mm)
  var TIP_CLEAR = 0.003;          // つかむとき、指先を机から何 mm 浮かすか(つかむ高さ = 指先の長さ + これ。形から決まる)
  var CARRY_LIFT = 0.020;        // 腕どうしが近すぎる手だけ、運ぶ高さをこれだけ上げる(下の tries)
  var HOVER_Y = 0.050;            // 運ぶ高さ。持ったピースの底が机 + 38.5mm(辺 14mm のピースの上 24.5mm・辺 28.5mm を超えると 10mm を割る)。高いほど届く範囲が狭い(tools/reach-ring.js)
  // 運ぶ高さ 50mm で真上を向ける範囲(肩の回転軸から。tools/reach-ring.js の実測 80〜240mm に 5mm の余裕)
  //   ★高さはどれも机から(机の上の面 = 0)。腕の原点は机より 2.4mm 上(2026-11-19 から台のあり・なしで同じ。それまで台ありは 9.6mm)なので、
  //     腕から見た運ぶ高さは 47.6mm(前の台ありは 40.4mm)で、届く輪は 80〜240mm より少し広い(この値は安全側)
  var R_IN = 0.085, R_OUT = 0.235;
  var REACH_PAD = 0.015;          // 道具の点はマスの真ん中から挟む幅の半分ほどずれるので、その分だけ内側に取る
  var SLOT_CLEAR = 0.020;         // 置き場のピースどうし・盤との間を何 mm 空けるか(指が下りる場所)
  // 置き場の選び直し(build の外側。2026-11-17): 腕どうしが負になったのが、あるピースの 1 手の中(前のピースを置いて上がった後〜このピースを置いて上がるまで)なら、
  //   そのピースの置き場を禁止して組み直す(運ぶ・置くときの手首の向きも、置き場で決まるつかみ方による)。RESLOT_BAN_R 以内には置かない【仮】・RESLOT_MAX 回まで【仮】(1 回 15〜40 秒かかる。画面では 1 回まで)
  var RESLOT_BAN_R = 0.025, RESLOT_MAX = 3;
  // 写す姿勢(上のカメラの台があるとき、最初と最後にここで止まる)。肩を少し左へ回し、上腕を後ろへ 50° 倒して腕をカメラの絵の外へよける【仮】。
  //   4 種のピースと盤のマスが 1 点も隠れない・台から 13mm より離れる・腕どうし(self-collision.js の 3 段)4.2mm(tools/overhead-occlusion.js の格子探し、2026-11-17)。
  //   ★2026-11-16 までの [-90, 0, 90, 90, 0] は、腕を畳みすぎて肩と手首がめり込んでいた(丸め箱で −2.8mm。利用者が画面で見つけた)
  var PHOTO_Q = [-20, -50, -40, 15, 0];
  var JAW_SAFE = 5;               // 爪の可動域の上の端から何度 手前までを使うか[度]
  var JOINT_SPEED = 90;           // [度/秒]
  var LINE_SPEED = 0.15;          // [m/秒]
  var DT = 0.05;                  // コマの刻み[秒]
  var MARGIN_WARN = 0.0025;       // いちばん近い所がこれ未満なら「要確認」(判定の甘さ: 点の間 最大 2mm ほど + コマの割り方 約 0.6mm。点検 C11 の 6)
  var SUB_STEP = 0.004;           // 当たり判定: 腕の点が 1 コマでこれより動く所は、コマの間も割って測る[m]
  var CONTACT_TOL = 0.001;        // 接する組(つかんでいるピースと爪・すき間 0 の隣どうし)の重なりを何 mm まで許すか(点の間隔・置く位置の差ぶん)

  var SETS = [
    { label: "4×5 の長方形・I O L J S", na: 4, nb: 5, pieces: "I O L J S" },
    { label: "4×5 の長方形・I O L J Z", na: 4, nb: 5, pieces: "I O L J Z" },
    { label: "3×4 の長方形・L J O", na: 3, nb: 4, pieces: "L J O" },
    { label: "3×3 の正方形・I3 V3 V3", na: 3, nb: 3, pieces: "I3 V3 V3" },
    // 長方形でない形(2026-11-17。art の "#" が埋めるマス。速い版の敷き詰め solveX で解く)。
    //   ピースの組は、すき間 0・15mm と台なしの 3 通りで腕どうしが 0 以上になる組を選んだ(2026-11-17)。
    //   T 字(2026-11-17 の夕方に足した): O O L J・I O L L は、すき間 0・15mm × 台あり・なしの 4 通りで腕どうし 3.0mm 以上(運ぶ高さを上げる試し方が入った後)。
    //     O L J S は、すき間 0 で J をつかみに下りる所で j2|cam が負。置き場の選び直し(RESLOT)で +1.8mm になる
    //     (2026-11-19 に C の台を本家の板なしにするまでは、台ありは腕が 7.2mm 高く、L を盤に置いた後に j1|cam が −1.9mm で直らなかった)。
    //   入れていないもの: 凹の I O L5 P5・ロの I I3 V5 は、台なし・すき間 0 で j2|cam が負(つかみに下りる所で手首のカメラが上腕に近づく)
    { label: "十字・S V3 P5", art: [".##.", "####", "####", ".##."], pieces: "S V3 P5", solver: "x" },
    { label: "凹の形・I O T I3 V3", art: ["##.##", "##.##", "#####", "#####"], pieces: "I O T I3 V3", solver: "x" },
    { label: "ロの字(真ん中が空き)・I L J", art: ["####", "#..#", "#..#", "####"], pieces: "I L J", solver: "x" },
    { label: "T 字・O O L J", art: ["#####", "#####", ".###.", ".###."], pieces: "O O L J", solver: "x" }
  ];
  var COLORS = ["#e4572e", "#2e86ab", "#f3a712", "#57a773", "#9c6ade", "#d64994"];

  // =========================================================================
  //  盤と置き場の配置(すき間で大きさが変わるので、毎回 届く範囲の輪の中に置き直す)
  // =========================================================================
  var PAN_X = SO101.joints[0].origin[0];
  function inRing(x, z, pad) { var r = Math.hypot(x - PAN_X, z); return r >= R_IN + pad && r <= R_OUT - pad; }
  // 盤の真ん中: 盤のマスの真ん中が全部 輪の中に入る、肩からの距離の範囲の真ん中
  function placeBoard(na, nb, P) {
    var ha = (na - 1) / 2 * P, hb = (nb - 1) / 2 * P, ok = [];
    for (var d = 0.05; d <= R_OUT; d += 0.001) {
      if (d - ha < R_IN + REACH_PAD) continue;
      if (Math.hypot(d + ha, hb) > R_OUT - REACH_PAD) continue;
      if (Math.hypot(d - ha, hb) > R_OUT - REACH_PAD) continue;
      ok.push(d);
    }
    if (!ok.length) return null;
    return [PAN_X + (ok[0] + ok[ok.length - 1]) / 2, 0];
  }
  // 置き場: 大きいピースから順に、盤に近い所へ。マスの真ん中は全部 輪の中・ほかとは SLOT_CLEAR 空ける
  //   avoid … 置いてはいけない所(上のカメラの台の足跡。机の上から見た長方形)。ここからも SLOT_CLEAR 空ける
  //   ban … 選び直しで使わない置き場 [{pid, xz}]。そのピースの置き場は、xz から RESLOT_BAN_R より近い所に取らない(build の選び直し)
  function placeSlots(pieces, P, boardRect, avoid, ban) {
    var placed = [boardRect].concat(avoid || []), slots = [], why = "";
    var order = pieces.slice().sort(function (a, b) { return PUZZLE.SHAPES[b.shape].length - PUZZLE.SHAPES[a.shape].length; });
    var bc = [(boardRect[0] + boardRect[2]) / 2, (boardRect[1] + boardRect[3]) / 2];
    order.forEach(function (p) {
      var cells = PUZZLE.SHAPES[p.shape], best = null;
      for (var x = PAN_X - R_OUT; x <= PAN_X + R_OUT; x += 0.005) {
        for (var z = -R_OUT; z <= R_OUT; z += 0.005) {
          var ok = true, lo = [1, 1], hi = [-1, -1];
          for (var i = 0; i < cells.length && ok; i++) {
            var cx = x + cells[i][0] * P, cz = z + cells[i][1] * P;
            if (!inRing(cx, cz, REACH_PAD)) ok = false;
            lo[0] = Math.min(lo[0], cx); lo[1] = Math.min(lo[1], cz); hi[0] = Math.max(hi[0], cx); hi[1] = Math.max(hi[1], cz);
          }
          if (!ok) continue;
          if (ban && ban.some(function (b) { return b.pid === p.idx && Math.hypot(x - b.xz[0], z - b.xz[1]) < RESLOT_BAN_R; })) continue;
          var rect = [lo[0] - CELL / 2, lo[1] - CELL / 2, hi[0] + CELL / 2, hi[1] + CELL / 2];
          for (var j = 0; j < placed.length && ok; j++) {
            var o = placed[j];
            var dx = Math.max(o[0] - rect[2], rect[0] - o[2], 0), dz = Math.max(o[1] - rect[3], rect[1] - o[3], 0);
            if (Math.hypot(dx, dz) < SLOT_CLEAR) ok = false;
          }
          if (!ok) continue;
          var score = Math.hypot((rect[0] + rect[2]) / 2 - bc[0], (rect[1] + rect[3]) / 2 - bc[1]);
          if (!best || score < best.score) best = { score: score, xz: [x, z], rect: rect };
        }
      }
      if (!best) { why = why || ("ピース " + p.name + " の置き場が届く範囲に取れない"); return; }
      placed.push(best.rect); slots[p.idx] = best.xz;
    });
    return { slots: slots, why: why };
  }

  // 置き場をファイルから読む(計画書_完成形まで の S-5)。layout = { pieces: [{ x, z, yaw }, …] }(ピースの番号順。
  //   机 = 0 の座標で mm・度。x は腕の前、z は腕の右(上から見て)、yaw は上から見て x → z の向きに回す角度で、
  //   puzzle.js の SHAPES の局所の原点 (0,0) を (x, z) に置き、その向きのまま yaw 回す。Z のように (0,0) にマスが無い形もある。
  //   O・I のように回すと同じ形になるものも、局所の (0,0) と向きはこの表のとおりに取る)。届かない・重なるなどの理由を返す
  function rectOf(cells, P, xz, yawDeg) {         // 回したピースの立方体の外側の長方形(上から見て)
    var r = [Infinity, Infinity, -Infinity, -Infinity], c = Math.abs(Math.cos(yawDeg * DEG)), sn = Math.abs(Math.sin(yawDeg * DEG));
    var h = CELL / 2 * (c + sn);
    cells.forEach(function (cl) {
      var o = rotY([cl[0] * P, 0, cl[1] * P], yawDeg), x = xz[0] + o[0], z = xz[1] + o[2];
      r[0] = Math.min(r[0], x - h); r[1] = Math.min(r[1], z - h); r[2] = Math.max(r[2], x + h); r[3] = Math.max(r[3], z + h);
    });
    return r;
  }
  // 180° 回して同じマスの集まりになる形なら、その対称の中心(局所のマスの座標。半端もある)。ならなければ null
  function sym180(shape) {
    var cells = PUZZLE.SHAPES[shape], cx = 0, cz = 0;
    cells.forEach(function (c) { cx += c[0]; cz += c[1]; }); cx /= cells.length; cz /= cells.length;
    var ok = cells.every(function (c) { return cells.some(function (d) { return Math.abs(2 * cx - c[0] - d[0]) < 1e-9 && Math.abs(2 * cz - c[1] - d[1]) < 1e-9; }); });
    return ok ? [cx, cz] : null;
  }
  function squaresOverlap(ca, ra, cb, rb, side) {   // 机の上の正方形 2 つ(真ん中・向き[度]・辺)が重なるか(接するだけは重ならない)
    var h = side / 2, d = [cb[0] - ca[0], cb[1] - ca[1]];
    var ax = [ra, ra + 90, rb, rb + 90].map(function (g) { return [Math.cos(g * DEG), Math.sin(g * DEG)]; });
    function rad(r, u) { var c = Math.cos(r * DEG), s = Math.sin(r * DEG); return h * (Math.abs(c * u[0] + s * u[1]) + Math.abs(-s * u[0] + c * u[1])); }
    return ax.every(function (u) { return Math.abs(d[0] * u[0] + d[1] * u[1]) < rad(ra, u) + rad(rb, u) - 1e-9; });
  }
  function rectGap(a, b) {
    var dx = Math.max(a[0] - b[2], b[0] - a[2], 0), dz = Math.max(a[1] - b[3], b[1] - a[3], 0);
    var inX = a[0] < b[2] && b[0] < a[2], inZ = a[1] < b[3] && b[1] < a[3];
    return inX && inZ ? -1 : Math.hypot(dx, dz);   // 重なれば -1
  }
  function readLayout(layout, pieces, P, boardRect, avoid) {
    var slots = [], rots = [], why = "", notes = [], rects = [];
    var L = (layout && layout.pieces) || [];
    pieces.forEach(function (p) {
      var e = L[p.idx], cells = PUZZLE.SHAPES[p.shape];
      if (!e || !isFinite(e.x) || !isFinite(e.z)) { why = why || ("ピース " + p.name + "(" + p.idx + " 番)の位置がファイルに無い"); return; }
      var xz = [e.x / 1000, e.z / 1000], yaw = +e.yaw || 0;
      slots[p.idx] = xz; rots[p.idx] = yaw;
      var rect = rectOf(cells, P, xz, yaw); rects[p.idx] = rect;
      var far = 0, near = Infinity;
      cells.forEach(function (cl) {
        var o = rotY([cl[0] * P, 0, cl[1] * P], yaw), r = Math.hypot(xz[0] + o[0] - PAN_X, xz[1] + o[2]);
        far = Math.max(far, r); near = Math.min(near, r);
      });
      if (far > R_OUT - REACH_PAD || near < R_IN + REACH_PAD)
        notes.push("ピース " + p.name + ": 届く範囲(肩から " + ((R_IN + REACH_PAD) * 1000).toFixed(0) + "〜" + ((R_OUT - REACH_PAD) * 1000).toFixed(0) + "mm)の外のマスがある(" + (near * 1000).toFixed(0) + "〜" + (far * 1000).toFixed(0) + "mm)。解けるかは逆運動学しだい");
      var gb = rectGap(rect, boardRect);
      //   on_board(2026-11-20、隣の会話の列 P の頼み「置き直し」): 盤の上に置いた後のピースを、そこからつかみ直す。盤に重なっても断らない。
      //   ほかのピースの目標のマスに掛かっていれば、そのピースを置くときの当たり判定が拾う(ここでは見ない)
      //   keep(点検 C48 の 2): 盤の上に置き終えたピース。動かさない(組む手から外す)が、ほかの手の当たりの相手にはなる
      if (gb < 0 && e.keep) notes.push("ピース " + p.name + ": 盤の上に置いたまま動かさない(keep)");
      else if (gb < 0 && e.on_board) notes.push("ピース " + p.name + ": 盤の上からつかみ直す(on_board)");
      else if (gb < 0) why = why || ("ピース " + p.name + " が盤に重なる(盤の上に置いたピースをつかみ直すなら、layout のそのピースに on_board: true。置いたまま動かさないなら keep: true)");
      else if (gb < SLOT_CLEAR) notes.push("ピース " + p.name + ": 盤まで " + (gb * 1000).toFixed(0) + "mm(指が下りるのに " + (SLOT_CLEAR * 1000) + "mm 欲しい。当たりは判定で見る)");
      (avoid || []).forEach(function (o) {
        var g = rectGap(rect, o);
        if (g < 0) why = why || ("ピース " + p.name + " が上のカメラの台に重なる");
        // 足跡まで SLOT_CLEAR 未満は、弾かずに書き置くだけ(当たりは判定で見る。盤までと同じ)。置かれたピースは動かせず、
        //   絵から読んだ置き場は 2mm ほどずれるので、本当は 20mm 空いていても 19mm と読んで組む前に止まった(隣の会話の点検 C29 の 2)。
        //   C の柱では 20mm 空けても当たる所がある(tools/pole-sweep.js)ので、守っているのは当たり判定のほう。自分で選ぶ置き場(placeSlots)は 20mm 空けたまま
        else if (g < SLOT_CLEAR) notes.push("ピース " + p.name + ": 上のカメラの台の足跡まで " + (g * 1000).toFixed(0) + "mm(" + (SLOT_CLEAR * 1000) + "mm 欲しい。当たりは判定で見る)");
      });
    });
    pieces.forEach(function (a) { pieces.forEach(function (b) {
      if (b.idx <= a.idx || !rects[a.idx] || !rects[b.idx]) return;
      var g = rectGap(rects[a.idx], rects[b.idx]);
      if (g < 0) {   // 外側の長方形が重なるときは、立方体どうしで確かめる(回したピースは長方形が大きめに出る)
        //   立方体の足跡(辺 CELL の正方形)どうしを、分ける軸の試し(4 本の辺の向き)で見る。前は真ん中どうしの距離 < CELL で、
        //   向きの違う正方形の重なりを見落とした(点検 C48 の 5: 向きの差 1°・真ん中 14.1mm で重なるのに通った)
        var hit = false;
        PUZZLE.SHAPES[a.shape].forEach(function (ca) { PUZZLE.SHAPES[b.shape].forEach(function (cb) {
          var oa = rotY([ca[0] * P, 0, ca[1] * P], rots[a.idx]), ob = rotY([cb[0] * P, 0, cb[1] * P], rots[b.idx]);
          if (squaresOverlap([slots[a.idx][0] + oa[0], slots[a.idx][1] + oa[2]], rots[a.idx], [slots[b.idx][0] + ob[0], slots[b.idx][1] + ob[2]], rots[b.idx], CELL)) hit = true;
        }); });
        if (hit) why = why || ("ピース " + a.name + " と " + b.name + " が重なる(立方体の足跡が重なる)");
        else notes.push("ピース " + a.name + " と " + b.name + ": 外側の長方形が重なる(立方体は離れている。当たりは判定で見る)");
      } else if (g < SLOT_CLEAR) notes.push("ピース " + a.name + " と " + b.name + ": " + (g * 1000).toFixed(0) + "mm(当たりは判定で見る)");
    }); });
    return { slots: slots, rots: rots, why: why, notes: notes };
  }

  // 判定の名前(画面の部品の名前と同じ)→ 表に出す名前
  var LINK_NAME = { base: "土台", j1: "肩", j2: "上腕", j3: "前腕", j4: "手首", j5: "爪の付け根と固定の爪", jaw: "動く爪", cam: "手首のカメラと台", "机": "机",
                    st_plate: "腕を載せる板", st_bottom: "本家の台の柱(下)", st_middle: "本家の台の柱(中)", st_top: "本家の台の柱(上)",
                    c_pole: "C の柱", c_arm: "C の吊る腕", c_cam: "C のカメラ", c_foot: "C の柱の足の金具", c_hanger: "C の吊る板", c_bracket: "C の吊る腕の金具" };

  // =========================================================================
  //  上のカメラの台(本家 Overhead_Cam_Mount_32x32_UVC_Module。置き方は so101.js の OVERHEAD【仮】)
  //   ★机の上の面を高さ 0 にする。腕(base_link の原点)は ROBOT_Y だけ上にある:
  //     土台の底のリブ 2.4mm(C の台は本家の板を使わないので、台のあり・なしで同じ。2026-11-19 まで台ありは板 7.2mm + 2.4mm)。逆運動学には「腕の座標」(y − ROBOT_Y)で渡す
  // =========================================================================
  // 2026-11-17 の夜から、上のカメラは置き方 C(別の柱で真上から。so101.js の OVERHEAD_C・overheadRig)。本家の台の柱 3 つは外した。
  //   2026-11-19 から C の台の設計(hardware/C_台の設計.md): 本家の腕の板は使わない。柱・吊る腕・吊る板・カメラ・柱の足の金具は、
  //   軸にそろった箱(so101.js の overheadRig)から閉じた箱の三角形を作り、本家の台と同じ道(meshField の距離・足跡・絵)に通す。
  //   (2026-11-17〜11-19 は、本家の板 + 柱・吊る腕・カメラの円柱)
  var STAND_PARTS = SO101.overheadRig().map(function (g) { return { id: g.id, box: g }; });
  var CYL_N = 24;
  function cylTris(a, b, r) {                   // a → b の円柱(両端にふた)。外向きの巻き順。平らな配列 [x,y,z, ...](m)
    var w = subv(b, a), L = Math.hypot(w[0], w[1], w[2]), wn = [w[0] / L, w[1] / L, w[2] / L];
    var t = Math.abs(wn[1]) < 0.9 ? [0, 1, 0] : [1, 0, 0];
    var u = KIN.unit([wn[1] * t[2] - wn[2] * t[1], wn[2] * t[0] - wn[0] * t[2], wn[0] * t[1] - wn[1] * t[0]]);
    var v = [wn[1] * u[2] - wn[2] * u[1], wn[2] * u[0] - wn[0] * u[2], wn[0] * u[1] - wn[1] * u[0]];   // u × v = w
    var R = r / Math.cos(Math.PI / CYL_N), out = [];
    function ring(c, i) { var th = 2 * Math.PI * i / CYL_N; return [c[0] + R * (u[0] * Math.cos(th) + v[0] * Math.sin(th)), c[1] + R * (u[1] * Math.cos(th) + v[1] * Math.sin(th)), c[2] + R * (u[2] * Math.cos(th) + v[2] * Math.sin(th))]; }
    function tri(p, q, s2) { out.push(p[0], p[1], p[2], q[0], q[1], q[2], s2[0], s2[1], s2[2]); }
    for (var i = 0; i < CYL_N; i++) {
      var p0 = ring(a, i), p1 = ring(a, i + 1), q0 = ring(b, i), q1 = ring(b, i + 1);
      tri(p0, p1, q1); tri(p0, q1, q0); tri(a, p1, p0); tri(b, q0, q1);
    }
    return out;
  }
  function boxTris(lo, hi) {                    // 軸にそろった箱(外向きの巻き順)。平らな配列 [x,y,z, ...](m)
    var out = [];
    function v(i) { return [i & 1 ? hi[0] : lo[0], i & 2 ? hi[1] : lo[1], i & 4 ? hi[2] : lo[2]]; }
    function quad(a, b, c, d) { [a, b, c, a, c, d].forEach(function (i) { var p = v(i); out.push(p[0], p[1], p[2]); }); }
    quad(0, 4, 6, 2); quad(1, 3, 7, 5);         // −x・+x
    quad(0, 1, 5, 4); quad(2, 6, 7, 3);         // −y・+y
    quad(0, 2, 3, 1); quad(4, 5, 7, 6);         // −z・+z
    return out;
  }
  function robotY(stand) { return (SO101.OVERHEAD.baseBelowOriginMm + (stand ? SO101.OVERHEAD_C.plateMm : 0)) / 1000; }
  // 台の部品の置き場所(机 = 0 の座標)。off … 検算用に台を机の上でずらす量 [x, z](m)
  function standPos(mesh, off) {
    var p = SO101.overheadPartPos(mesh), o = off || [0, 0];
    return [p[0] + o[0], p[1] + robotY(true), p[2] + o[1]];
  }
  // 部品 s の三角形(机 = 0 の座標・m の平らな配列)。off … 検算用に台を机の上でずらす量 [x, z](m)
  function standTris(s, off) {
    var o = off || [0, 0], ry = robotY(true);
    if (s.box) return boxTris([s.box.lo[0] + o[0], s.box.lo[1] + ry, s.box.lo[2] + o[1]], [s.box.hi[0] + o[0], s.box.hi[1] + ry, s.box.hi[2] + o[1]]);
    if (s.seg) return cylTris([s.seg.a[0] + o[0], s.seg.a[1] + ry, s.seg.a[2] + o[1]], [s.seg.b[0] + o[0], s.seg.b[1] + ry, s.seg.b[2] + o[1]], s.seg.r);
    var p = standPos(s.mesh, off), T = SO101_TRIS[s.mesh], out = new Array(T.length);
    for (var i = 0; i < T.length; i++) out[i] = T[i] * 1e-4 + p[i % 3];
    return out;
  }
  var STAND_FIELDS = {};                        // 台の三角形(机 = 0 の座標)。重いので、ずらす量ごとに 1 回だけ作る
  function standFields(off) {
    var key = off ? off.join(",") : "0,0";
    if (STAND_FIELDS[key]) return STAND_FIELDS[key];
    return (STAND_FIELDS[key] = STAND_PARTS.map(function (s) {
      return { id: s.id, field: COLLIDE.meshField(standTris(s, off), 1, [0, 0, 0], 0.015) };
    }));
  }
  // 台の足跡: 机から maxY までに頂点が 1 つでもある三角形の頂点を、机の上から見た長方形にまとめる(部品ごと)。
  //   当たり判定と同じ三角形から作る(点検 C10 の 7。間引いた点からだと 3mm ほど小さく出うる)
  function standFootprint(maxY, off) {
    var rects = [];
    STAND_PARTS.forEach(function (s) {
      var T = standTris(s, off), r = [Infinity, Infinity, -Infinity, -Infinity];
      for (var o = 0; o < T.length; o += 9) {
        if (Math.min(T[o + 1], T[o + 4], T[o + 7]) > maxY) continue;
        for (var k = 0; k < 9; k += 3) {
          var x = T[o + k], z = T[o + k + 2];
          r[0] = Math.min(r[0], x); r[1] = Math.min(r[1], z); r[2] = Math.max(r[2], x); r[3] = Math.max(r[3], z);
        }
      }
      if (isFinite(r[0])) rects.push(r);
    });
    return rects;
  }

  // =========================================================================
  //  S-8: 爪のまわり(j5・動く爪・手首のカメラ)とピースが近い所は、STL の三角形までの距離で測り直す。
  //   ふだんの判定は腕の表面の点(約 3mm 間隔)⇄ ピースの箱 で、点の間のぶん最大 2mm ほど甘く(遠く)出る。
  //   近い組(REFINE_D 未満)だけ、箱の表面に BOX_STEP 間隔で打った点(粗い判定で近かった所のまわり)から、
  //   腕の三角形までの距離(collide.js の meshField の dist。中か外かは、いちばん近い点だけ巻き数で決める)を測る。
  //   箱の点の間隔のぶん、最大 BOX_STEP/√2 ほど甘い。粗い判定で負(当たり)の組は測り直さない(当たりは当たりのまま)
  // =========================================================================
  var REFINE_D = 0.005, BOX_STEP = 0.0005, PATCH_HALF = 0.003, COARSE_STEP = 0.0015;
  var GRIP_FIELDS = null;                       // group → [{ mesh, field }](全関節 0 の姿勢の画面の座標)
  function gripFields() {
    if (GRIP_FIELDS) return GRIP_FIELDS;
    GRIP_FIELDS = {};
    if (typeof SO101_TRIS === "undefined") return GRIP_FIELDS;
    Object.keys(SO101_VISUALS).forEach(function (link) {
      var g = SO101.LINK_GROUP[link] || (SO101.CAMERA_GROUP[link] ? "cam" : null);
      if (g !== "j5" && g !== "jaw" && g !== "cam") return;
      SO101_VISUALS[link].forEach(function (v) {
        var T = SO101_TRIS[v.mesh];
        if (!T) return;
        var Pv = SO101.visualPose(link, v.xyz, v.rpy), out = new Float64Array(T.length);
        for (var i = 0; i < T.length; i += 3) {
          var w = SO101.mv(Pv.R, [T[i] * 1e-4, T[i + 1] * 1e-4, T[i + 2] * 1e-4]);
          out[i] = w[0] + Pv.p[0]; out[i + 1] = w[1] + Pv.p[1]; out[i + 2] = w[2] + Pv.p[2];
        }
        (GRIP_FIELDS[g] || (GRIP_FIELDS[g] = [])).push({ mesh: v.mesh, field: COLLIDE.meshField(out, 1, [0, 0, 0], REFINE_D + 0.002) });   // 距離は REFINE_D 少し先まで分かれば足りる(升を小さく)
      });
    });
    return GRIP_FIELDS;
  }
  function matTVec(M, v) { return [M[0] * v[0] + M[3] * v[1] + M[6] * v[2], M[1] * v[0] + M[4] * v[1] + M[7] * v[2], M[2] * v[0] + M[5] * v[1] + M[8] * v[2]]; }
  // 組 (腕の部品 gname, 箱の並び cubes) の、三角形までの距離。fr = その部品の座標系(KIN.fk の frames)、ry = 腕の原点の高さ。
  //   armPts = 粗い判定に使った腕の点(机 = 0 の座標)、coarse = 粗い距離。測れないときは null
  function fineDist(gname, fr, jawDeg, ry, cubes, armPts, coarse) {
    var F = gripFields()[gname];
    if (!F || !F.length) return null;
    // 箱の表面で、粗い判定の近い点(coarse + 3mm 以内)に最も近い所を集める
    var seeds = [];
    cubes.forEach(function (b, bi) {
      armPts.forEach(function (w) {
        var q = matTVec(b.M, subv(w, b.c)), c = [0, 0, 0], inside = true, d2 = 0;
        for (var k = 0; k < 3; k++) { c[k] = Math.max(-b.h[k], Math.min(b.h[k], q[k])); if (Math.abs(q[k]) > b.h[k]) inside = false; d2 += (q[k] - c[k]) * (q[k] - c[k]); }
        var d = Math.sqrt(d2);
        if (inside) {                           // 中なら、いちばん近い面へ押し出す
          var kk = 0, m = Infinity;
          for (var k2 = 0; k2 < 3; k2++) { var e = b.h[k2] - Math.abs(q[k2]); if (e < m) { m = e; kk = k2; } }
          c[kk] = q[kk] >= 0 ? b.h[kk] : -b.h[kk]; d = -m;
        }
        if (d <= coarse + 0.003) seeds.push([bi, c]);
      });
    });
    if (!seeds.length) return null;
    // 2 段で探す。1 段目は COARSE_STEP 間隔で当たりを付け、2 段目は候補のまわりだけ BOX_STEP 間隔で。
    //   距離は箱の面の上で 1 ずつしか変わらない(リプシッツ 1)ので、1 段目のいちばん近い値 + COARSE_STEP/√2 より
    //   遠い 1 段目の点のまわりには、本当のいちばん近い所は無い(そこは 2 段目で見なくてよい)
    var Rt = fr.R, memo = {}, best = Infinity, bestP = null, bestF = null;
    function at(bi, k, l) {                     // 箱 bi の面 k の上の点 l(箱の座標)の距離。覚えておく
      var key = bi + ":" + k + ":" + Math.round(l[0] * 1e5) + "," + Math.round(l[1] * 1e5) + "," + Math.round(l[2] * 1e5);
      if (key in memo) return memo[key];
      var b = cubes[bi], w = addv(b.c, KIN.matVec(b.M, l));     // 机 = 0 の座標
      var pa = [w[0], w[1] - ry, w[2]];                          // 腕の座標
      var p0 = addv(fr.o, matTVec(Rt, subv(pa, fr.p)));          // 全関節 0 の姿勢の座標
      if (gname === "jaw") p0 = jawRotate(p0, -jawDeg);
      var m = Infinity;
      for (var j = 0; j < F.length; j++) { var dq = F[j].field.dist(p0); if (dq < m) m = dq; if (dq < best) { best = dq; bestP = p0; bestF = F[j].field; } }
      return (memo[key] = m);
    }
    function patch(bi, k, c, half, step, out) {  // c のまわり ±half を step の格子で(面の外は面の縁に寄せる)
      var b = cubes[bi], a1 = (k + 1) % 3, a2 = (k + 2) % 3;
      for (var u = -half; u <= half + 1e-12; u += step) for (var v = -half; v <= half + 1e-12; v += step) {
        var l = [0, 0, 0];
        l[k] = c[k];
        l[a1] = Math.max(-b.h[a1], Math.min(b.h[a1], Math.round((c[a1] + u) / step) * step));
        l[a2] = Math.max(-b.h[a2], Math.min(b.h[a2], Math.round((c[a2] + v) / step) * step));
        var d = at(bi, k, l);
        if (out) out.push([bi, k, l, d]);
      }
    }
    var lv1 = [];
    seeds.forEach(function (sd) {
      var b = cubes[sd[0]], c = sd[1];
      for (var k = 0; k < 3; k++) {
        if (Math.abs(Math.abs(c[k]) - b.h[k]) > 1e-9) continue;   // c が載っている面ごとに
        patch(sd[0], k, c, PATCH_HALF, COARSE_STEP, lv1);
      }
    });
    var cut = best + COARSE_STEP / Math.SQRT2 + 1e-9;
    lv1.forEach(function (e) { if (e[3] <= cut) patch(e[0], e[1], e[2], COARSE_STEP / 2 + BOX_STEP, BOX_STEP, null); });
    if (!isFinite(best)) return null;
    return bestF.inside(bestP) ? -best : best;    // 中か外かは、いちばん近い点だけ巻き数で決める
  }

  // ---- 小道具 ----
  function subv(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
  function addv(a, b) { return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]; }
  function dotv(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
  function rotY(local, psiDeg) {                // 上から見て X → Z 向きに psi 回す
    var c = Math.cos(psiDeg * DEG), s = Math.sin(psiDeg * DEG);
    return [local[0] * c - local[2] * s, local[1], local[0] * s + local[2] * c];
  }
  function maxAbsDiff(a, b) { var m = 0; for (var i = 0; i < a.length; i++) m = Math.max(m, Math.abs(a[i] - b[i])); return m; }

  // =========================================================================
  //  腕の形(公式の STL)と、当たり判定の点(全関節 0 の姿勢の画面の座標)
  // =========================================================================
  var GROUP_ORIGIN = (function () {
    var o = { base: [0, 0, 0], jaw: SO101.jaw.origin };
    SO101.joints.forEach(function (j, i) { o["j" + (i + 1)] = j.origin; });
    o.cam = SO101.joints[4].origin;             // 手首のカメラと台(爪の付け根 j5 と一緒に動く。爪の寸法を測る j5 の点とは分けておく)
    return o;
  })();
  var ARM = (function () {
    var meshes = [], points = {};               // points[group] = [[x,y,z], ...]
    Object.keys(SO101_VISUALS).forEach(function (link) {
      var g = SO101.LINK_GROUP[link] || (SO101.CAMERA_GROUP[link] ? "cam" : null);   // 手首のカメラは "cam"(公式の so101_new_calib_camera.urdf)
      if (!g) return;
      SO101_VISUALS[link].forEach(function (v) {
        var P = SO101.visualPose(link, v.xyz, v.rpy);
        meshes.push({ group: g, mesh: v.mesh, pos: subv(P.p, GROUP_ORIGIN[g]), rot: SO101.eulerXYZ(P.R) });
        var raw = SO101_POINTS[v.mesh], arr = points[g] || (points[g] = []);
        for (var i = 0; i < raw.length; i += 3) {
          var w = SO101.mv(P.R, [raw[i] * 1e-4, raw[i + 1] * 1e-4, raw[i + 2] * 1e-4]);
          arr.push([w[0] + P.p[0], w[1] + P.p[1], w[2] + P.p[2]]);
        }
      });
    });
    var spheres = {};                           // 点を囲む球(全関節 0 の座標)
    Object.keys(points).forEach(function (g) {
      var lo = [1, 1, 1], hi = [-1, -1, -1];
      points[g].forEach(function (p) { for (var k = 0; k < 3; k++) { lo[k] = Math.min(lo[k], p[k]); hi[k] = Math.max(hi[k], p[k]); } });
      var c = [(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2], r = 0;
      points[g].forEach(function (p) { r = Math.max(r, Math.hypot(p[0] - c[0], p[1] - c[1], p[2] - c[2])); });
      spheres[g] = { c: c, r: r };
    });
    return { meshes: meshes, points: points, spheres: spheres };
  })();

  // 動く爪の点を、爪の角度 deg だけ回す(全関節 0 の座標)
  function jawRotate(p, deg) {
    var o = SO101.jaw.origin, R = KIN.rotAA(SO101.jaw.axis, deg * DEG);
    return addv(o, KIN.matVec(R, subv(p, o)));
  }

  // =========================================================================
  //  爪の寸法を形の点から測る(道具の座標: a = 指先の向き / u = 動く爪の向き / s = 横)
  // =========================================================================
  var TOOLF = (function () {
    var T = SO101.tool.pos, d = KIN.unit(SO101.tool.dir);
    var u0 = SO101.tool.up, u = KIN.unit(subv(u0, KIN.scale(d, dotv(u0, d))));
    return function (p) { var q = subv(p, T); return { a: dotv(q, d), u: dotv(q, u) }; };
  })();
  // つかむ高さ = 指先がいちばん下がる所(爪の角度 −10°〜60° のどこでも)+ TIP_CLEAR
  var GRASP_Y = (function () {
    var tip = -Infinity;
    ARM.points.j5.forEach(function (p) { var q = TOOLF(p); if (q.u > -0.03 && q.u < 0.03) tip = Math.max(tip, q.a); });
    for (var dg = SO101.jaw.limitsDeg[0]; dg <= 60; dg += 2)
      ARM.points.jaw.forEach(function (p) { tip = Math.max(tip, TOOLF(jawRotate(p, dg)).a); });
    return Math.ceil((tip + TIP_CLEAR) * 2000) / 2000;      // 0.5mm に切り上げ
  })();
  var GRIP_CACHE = {};
  function gripFor(cell, hgt) {
    hgt = hgt || cell;                                       // ピースの高さ(既定は辺と同じ立方体。S1 の ui.HEIGHT。2026-11-22)
    var ck = Math.round(cell * 1e5) + ":" + FIT_MARGIN + (hgt === cell ? "" : ":h" + Math.round(hgt * 1e5));   // fitGap は余裕で変わる
    if (GRIP_CACHE[ck]) return GRIP_CACHE[ck];
    var toTool = TOOLF, CELL = cell;
    var aMin = GRASP_Y - hgt;                   // ピースの高さの範囲(ここで指がピースに当たる)
    var fixed = ARM.points.j5.map(toTool).filter(function (q) { return q.a >= aMin; });
    var face = -Infinity, fLo = Infinity, tip = -Infinity;
    fixed.forEach(function (q) { face = Math.max(face, q.u); fLo = Math.min(fLo, q.u); tip = Math.max(tip, q.a); });
    function moving(deg) {
      var lo = Infinity, hi = -Infinity, tp = -Infinity, zone = [];
      ARM.points.jaw.forEach(function (p) {
        var q = toTool(jawRotate(p, deg));
        tp = Math.max(tp, q.a);
        if (q.a < aMin) return;
        zone.push(q);
        lo = Math.min(lo, q.u); hi = Math.max(hi, q.u);
      });
      // 内側の面(ピースに触れる側。いちばん内側から 3mm 以内の点)の いちばん下
      var inLow = -Infinity;
      zone.forEach(function (q) { if (q.u <= lo + 0.003) inLow = Math.max(inLow, q.a); });
      return { inner: lo, outer: hi, tip: tp, innerLow: inLow };
    }
    function gap(deg) { return moving(deg).inner - face; }
    var lim = SO101.jaw.limitsDeg;
    var sign = gap(10) > gap(0) ? 1 : -1;       // 開く向き(角度が増えると開くか)
    // 挟める開きの上限: 動く爪は回って開くので、開くほどピースに触れる内側の面が上がり、
    // ピースの上の角にしか触れなくなる。★内側の面の下端が ピースの高さの半分より下まで届く角度まで
    // (2026-11-12 の点検 C6 の 3: はじめは爪の外側の角(いちばん低い点)で測っていて、甘かった)。
    // この線(「高さの半分」)は Claude の決めた目安で、滑らずに持てるかの物理の裏付けは無い。
    var top = 0, reachA = GRASP_Y - hgt / 2;
    for (var dg = 0; Math.abs(dg) <= Math.abs((sign > 0 ? lim[1] : lim[0])) - JAW_SAFE; dg += sign * 0.5) {
      var mvd = moving(dg);
      if (!isFinite(mvd.inner) || mvd.innerLow < reachA) break;
      top = dg;
    }
    var openTop = (sign > 0 ? lim[1] : lim[0]) - sign * JAW_SAFE;   // 開いて下ろすときは、可動域の端の手前まで開ける
    var cache = {};
    // すき間 w になる爪の角度(二分法)
    function angleFor(w) {
      var key = Math.round(w * 1e5);
      if (key in cache) return cache[key];
      var lo = sign > 0 ? lim[0] : lim[1], hi = openTop;                 // gap(lo) < gap(hi)
      for (var i = 0; i < 40; i++) {
        var mid = (lo + hi) / 2;
        if (gap(mid) < w) lo = mid; else hi = mid;
      }
      return (cache[key] = (lo + hi) / 2);
    }
    var gMax = gap(top);
    var mv25 = moving(angleFor(CELL));
    var g = {
      face: face,                               // 固定の爪の内側の面(u。道具の点から)
      tip: Math.max(tip, mv25.tip),             // 指先(a。道具の点から指先の向きへ)
      thickFixed: face - fLo,                   // 固定の指の厚さ(ピースの高さの範囲で)
      thickMove: mv25.outer - mv25.inner,       // 動く指の厚さ(1 マスを挟んだとき)
      gap0: gap(0), gapMax: gMax, sign: sign, topDeg: top,
      angleFor: angleFor, gap: gap, cell: cell, height: hgt
    };
    var r = GRIP_CACHE[ck] = g;
    r.fingerT = Math.max(r.thickFixed, r.thickMove);
    r.fitGap = r.fingerT + 2 * FIT_MARGIN;      // 隣が埋まっていても指が入るすき間
    r.maxSpan = r.gapMax;                       // 挟める幅の上限(閉じたとき 内側の面が高さの半分より下まで届く)
    r.gapDefault = Math.ceil(r.fitGap * 1000) + 1;
    return r;
  }
  // いまのブロックの辺の爪の寸法(build の最初に setCell で入れ替える)
  var GRIP, FINGER_T, FIT_GAP, MAX_SPAN, HGT;
  function setCell(cell, hgt) {
    CELL = cell; HGT = hgt || cell; GRIP = gripFor(cell, HGT);
    FINGER_T = GRIP.fingerT; FIT_GAP = GRIP.fitGap; MAX_SPAN = GRIP.maxSpan;
  }
  // 既定のブロックの辺: 隣が埋まっていても指が入るすき間で、2 マスを一度に挟める いちばん大きい辺(12〜30mm。2026-11-12 の実測で 14mm)
  //   (2×2 の O のように、どの向きにも隣のマスがあるピースは 2 マスで挟むしかないため)
  var CELL_DEFAULT = (function () {
    for (var c = 30; c >= 12; c--) {
      var g = gripFor(c / 1000);
      if (2 * c + g.gapDefault <= g.maxSpan * 1000 + 1e-9) return c;
    }
    return 25;
  })();
  setCell(CELL_DEFAULT / 1000);
  var GAP_DEFAULT = GRIP.gapDefault;

  var controls = [
    { key: "SET", label: "目標の形とピース", type: "select", default: 2,
      options: SETS.map(function (s, i) { return { value: i, label: s.label }; }) },
    { key: "CELL", label: "ブロックの辺(既定 " + CELL_DEFAULT + "mm = 実物の爪で 2 マスを挟める いちばん大きい辺)",
      min: 12, max: 30, step: 1, unit: "mm", default: CELL_DEFAULT },
    { key: "GAP", label: "ブロックのすき間(0 = すき間なく詰める・指が入るのは約 " + (FIT_GAP * 1000).toFixed(0) + "mm から)",
      min: 0, max: 30, step: 1, unit: "mm", default: GAP_DEFAULT },
    { key: "SOL", label: "敷き詰め方の番号", min: 0, max: 9, step: 1, unit: "番目", default: 0 },
    { key: "STAND", label: "上のカメラ(置き方 C: 別の柱で真上から。腕は合板にじかに置く。寸法は C の台の設計【仮】)", type: "select", default: 1,
      options: [{ value: 0, label: "なし(腕は机にじかに置く)" }, { value: 1, label: "あり(置き方 C の柱・吊る腕・吊る板・カメラ・金具)" }] },
    { key: "WRIST", label: "手首のカメラ(公式の台と 32×32 のカメラ。当たり判定に入る)", type: "select", default: 1,
      options: [{ value: 0, label: "なし" }, { value: 1, label: "あり(完成形 A)" }] },
    { key: "WRIST_CHECK", label: "つかむ場所の上で止まって、手首のカメラで確かめる(整定 + 撮る【仮】。想定動作の 3 段)", type: "select", default: 1,
      options: [{ value: 0, label: "止まらない" }, { value: 1, label: "止まる(決めたこと 6)" }] }
  ];
  function defaultUiValues() {
    var v = {}; controls.forEach(function (c) { v[c.key] = c.default; }); return v;
  }

  function armParts(ry, wrist) {
    // 腕の全体を ROBOT_Y だけ上げる(机の上の面 = 0)
    var parts = [{ id: "robot", shape: "group", position: [0, ry, 0] }, { id: "base", parent: "robot", shape: "group", position: [0, 0, 0] }];
    var J = SO101.joints, prev = [0, 0, 0];
    for (var i = 0; i < J.length; i++) {      // 関節の入れ子(KIN と同じ形: 子の位置 = 自分の軸の点 − 親の軸の点)
      parts.push({ id: "j" + (i + 1), parent: i ? "j" + i : "robot", shape: "group",
                   position: subv(J[i].origin, prev), axis: J[i].axis, angleDeg: 0 });
      prev = J[i].origin;
    }
    parts.push({ id: "jaw", parent: "j5", shape: "group", position: subv(SO101.jaw.origin, J[4].origin), axis: SO101.jaw.axis, angleDeg: 0 });
    ARM.meshes.forEach(function (m, k) {
      if (m.group === "cam" && !wrist) return;
      var par = m.group === "cam" ? "j5" : m.group;
      // 絵(通常)と、判定の点を作った同じ形の半透明(当たり判定を出したとき・干渉で赤くなる)
      parts.push({ id: "vis_" + m.group + "_" + k, parent: par, shape: "mesh", mesh: m.mesh,
                   position: m.pos, rotationDeg: m.rot, collide: m.group });
      parts.push({ id: "cv_" + m.group + "_" + k, parent: par, shape: "mesh", mesh: m.mesh,
                   position: m.pos, rotationDeg: m.rot, color: "#58a6ff", opacity: 0.45, noShadow: true, hit: m.group });
    });
    return parts;
  }

  // =========================================================================
  //  動きの組み立て
  // =========================================================================
  // 組み立ての外側: 置き場を自分で決めたとき(LAYOUT でない)、腕どうしが負なら、置き場を選び直して組み直す(上の RESLOT_*)。
  //   NO_RESLOT … 選び直さない(比べる検算用)。選び直しても負のままなら、腕どうしの隙間がいちばん大きかった回を返す(判定は NG のまま)
  function build(ui) {
    var d = buildOnce(ui);
    if (ui.LAYOUT || ui.LAYOUT_ONLY || ui.NO_RESLOT || ui.SELF === 0 || typeof SELF_COLLIDE === "undefined") return d;
    function selfBad(c) { return c && c.solved && c.self && (c.self.min < 0 || c.self.worstBound < 0); }
    function score(c) { return Math.min(c.self.min, c.self.worstBound); }
    function allOk(c) { return c.solved && !selfBad(c) && !c.hitFrames; }   // 腕どうしだけでなく、ピース・台・手首のカメラとの当たりも無い
    function better(a, b) {        // a のほうが良いか: 総合で OK を先に、次に腕どうしの隙間
      if (allOk(a._check) !== allOk(b._check)) return allOk(a._check);
      if (!a._check.hitFrames !== !b._check.hitFrames) return !a._check.hitFrames;
      return score(a._check) > score(b._check);
    }
    if (!selfBad(d._check)) return d;
    // 画面では 1 回まで(1 回 15〜40 秒、その間は画面が止まる)。node の道具は RESLOT_MAX 回まで
    var max = ui.RESLOT_MAX !== undefined ? ui.RESLOT_MAX : (typeof window !== "undefined" ? 1 : RESLOT_MAX);
    var first = d, best = d, bans = [], log = [];
    for (var n = 0; n < max && selfBad(d._check); n++) {
      var c = d._check, onlyBound = c.self.min >= 0, tBad = onlyBound ? c.self.boundAt : c.self.t, st = null;
      var pair = onlyBound ? c.self.boundPair : c.self.pair;
      c.stepTimes.forEach(function (s, i) {           // 1 手 = 前のピースを置いて上がった後(最初は待ちの姿勢に着いた時刻)〜このピースを置いて上がるまで
        var t0 = s.tBack > 0 ? s.tBack : i ? c.stepTimes[i - 1].tDone : (c.tReady || 0);   // RELOOK の寄り道(置いた所 → 写す姿勢で止まり終わるまで。VIA があるときは待ちの姿勢に戻るまで)は 1 手に入れない(点検 C28 の 3)。写す姿勢(か待ちの姿勢)から このピースの上へは 1 手に入る
        if (!st && tBad >= t0 - 1e-9 && tBad <= s.tDone + 1e-9) st = s;
      });
      if (!st) { log.push("負になったのは " + (+tBad).toFixed(1) + " 秒(" + pair + ")で、どのピースの 1 手でもない(写す姿勢との行き来・RELOOK の寄り道・最後に戻る所。置き場を変えても直らない)"); break; }
      var sIn = (c.slotsIn || c.slots)[st.pid];         // 置き場を決めたときの原点(180° の見直しの前。placeSlots が比べるのはこれ。点検 C28 の 2)
      bans.push({ pid: st.pid, xz: sIn.slice() });
      log.push(st.name + " の置き場 (" + (sIn[0] * 1000).toFixed(0) + ", " + (sIn[1] * 1000).toFixed(0) + ")mm を外した(" +
               pair + " " + (score(c) * 1000).toFixed(1) + "mm" + (onlyBound ? "・コマの間の下限" : "") + "・" + (+tBad).toFixed(1) + " 秒)");
      var prev = d;
      d = buildOnce(Object.assign({}, ui, { _SLOT_BAN: bans.slice() }));
      if (!d._check.solved) { log.push("外したら解けない: " + d._check.why); break; }
      if (better(d, best)) best = d;
      // 0.1mm 以上よくならなければ止める(写す姿勢や盤の位置のような、置き場によらない所で、むだに組み直さない)
      if (selfBad(d._check) && score(d._check) < score(prev._check) + 0.0001) { log.push("外しても 0.1mm 以上よくならない(" + (score(d._check) * 1000).toFixed(1) + "mm)ので止めた"); break; }
    }
    var ok = allOk(best._check), selfFixed = !selfBad(best._check);
    best._check.reslot = { tries: bans.length, bans: bans, log: log, ok: ok, selfFixed: selfFixed, firstSelf: score(first._check) };
    (best.info || []).push({ label: "置き場の選び直し", state: ok ? (score(best._check) < SELF_COLLIDE.WARN ? "warn" : "ok") : "ng",
      value: (ok ? "腕どうしが負だったので、置き場を選び直して通った(" : selfFixed ? "腕どうしは 0 以上になったが、ほかの当たりが出た(" : "選び直したが、腕どうしが負のまま(") + log.join(" → ") + ")" });
    return best;
  }
  // S1(2026-11-22): 1 手ごとに、段(開いて下ろす・寄せて閉じる・上げる・運ぶ・下ろして置く・開く・抜く)の間の、腕と持っているピースから
  //   ほかのピース(その手でつかむピース・触れている・持っているピースを除く)と机までのいちばん近い所。遠くて判定が組を出さないときは Infinity。phaseAt は当たり判定のコマ(間に足した判定を含む)ごとの値で、
  //   爪のまわりは三角形まで測り直した後の値(当たり判定と同じ。_ARM_MARGIN_MM の窓では動かした所の値も混ざる)
  var PHASES = [["open_descend", "tHover", "tDown"], ["approach_close", "tDown", "tPick"], ["lift", "tPick", "tLift"], ["carry", "tLift", "tTop"],
                ["place", "tTop", "tPlace"], ["release", "tPlace", "tRel"], ["withdraw", "tRel", "tDone"]];
  //   段の窓は [始め, 終わり)(境のコマは後の段。最後の「抜く」だけ終わりも含む。点検 C59 の 4)。「移る」= 前の手の終わり(最初は待ちの姿勢に着いた時刻、
  //   RELOOK の寄り道があれば写す姿勢を出た時刻)から、このピースの真上に着くまで。pc = ほかのピース、own = その手のピース(触れている・持っている間は
  //   当たり判定の外なので入らない。開いて下ろす・抜く段で、固定の爪が余裕 2mm の所を通る分が出る。点検 C59 の 1)
  function phaseClearOf(steps, phaseAt, tReady) {
    return steps.map(function (s, i) {
      var out = { pid: s.pid, name: s.name };
      var t0 = s.tBack > 0 ? s.tBack : i ? steps[i - 1].tDone : (tReady || 0);
      [["move", t0, s.tHover]].concat(PHASES.map(function (ph) { return [ph[0], s[ph[1]], s[ph[2]]]; })).forEach(function (ph, k, all) {
        var a = ph[1], b = ph[2], last = k === all.length - 1, r = { pc: Infinity, pcPair: "", pcT: 0, own: Infinity, ownPair: "", ownT: 0, tb: Infinity, tbPair: "", n: 0 };
        phaseAt.forEach(function (e) {
          if (e.t < a - 1e-9 || (last ? e.t > b + 1e-9 : e.t >= b - 1e-9)) return;
          r.n++;
          Object.keys(e.pcs).forEach(function (id) {
            var v = e.pcs[id];
            if (id === "pc" + s.pid) { if (v[0] < r.own) { r.own = v[0]; r.ownPair = v[1]; r.ownT = e.t; } }
            else if (v[0] < r.pc) { r.pc = v[0]; r.pcPair = v[1]; r.pcT = e.t; }
          });
          if (e.tb < r.tb) { r.tb = e.tb; r.tbPair = e.tbPair; }
        });
        out[ph[0]] = r;
      });
      return out;
    });
  }
  function buildOnce(ui) {
    var set = ui._SET_OVERRIDE ? Object.assign({}, ui._SET_OVERRIDE) : (SETS[ui.SET | 0] || SETS[0]);   // _SET_OVERRIDE … 検算用(新しい種を SETS に足す前に試す)
    FIT_MARGIN = ui._FIT_MARGIN === undefined ? FIT_MARGIN_DEFAULT : ui._FIT_MARGIN / 1000;
    setCell((ui.CELL === undefined ? CELL_DEFAULT : ui.CELL) / 1000, ui.HEIGHT === undefined ? undefined : ui.HEIGHT / 1000);   // HEIGHT(mm): S1。既定は辺
    var GAP = (ui.GAP === undefined ? GAP_DEFAULT : ui.GAP) / 1000, P = CELL + GAP;
    // FORCE_FIT … 検算用(tools/check-scene.js)。指が入らないすき間でも入る前提で解かせ、当たり判定が赤を出すかを見る
    var fits = ui.FORCE_FIT ? true : GAP >= FIT_GAP - 1e-9;
    var STAND = ui.STAND === undefined ? 1 : (ui.STAND | 0);
    var WRIST = ui.WRIST === undefined ? 1 : (ui.WRIST | 0);     // 手首のカメラと台(計画書_完成形まで の S-4)
    // 想定動作の 3 段(docs/plan/想定動作_SO101パズル.md・計画書_完成形まで の決めたこと 6): つかむ場所の上で爪を開いたあと止まり、手首のカメラで撮る。
    //   止まる時間 = 整定 + 撮る(【仮】。実物で測ったら入れ替える)。手首のカメラが無いときは止まらない。ずれを直す動きは入れていない(sim では、ずれは 0)
    var WRIST_SETTLE = 0.3, WRIST_SHOT = 0.3;
    // RELOOK … 2 つ目からのピースを取りに行く前に、置いた所 → 写す姿勢 → 次のピースの上 と寄り道し、写す姿勢で RELOOK_SHOT 秒止まる(上のカメラで撮り直す)。
    //   上のカメラが C(真上 540mm)になってから、既定の 3×4 では L を置いて J の上へ行く間(0.5 秒)ずっと、腕が J を隠す(O の前は隠れない時刻がある。
    //   寄り道は 2 つ目からの全部のピースに入るので、要らない所でも 1 回 1.5 秒ほどかかる。点検 C28 の 7)
    //   (隣の会話「カメラ配置とアーム干渉検出」の live.html。利用者「Yes, fix it」2026-11-18)。写す姿勢の前後の経由点は最初の写す姿勢 → 待ちの姿勢と同じ(VIA。いまは無い)。台があるときだけ
    var RELOOK = !!ui.RELOOK && STAND, RELOOK_SHOT = 0.3;
    // _ERR … 許せるずれを測る道具用(tools/allow-err.js。計画書_完成形まで の S-8 の後半)。1 つのピースの 1 段(pick = 真上に着く〜閉じる、
    //   place = 盤の真上から下ろす〜離して上へ戻る)だけ、腕の狙いを {a: 閉じる向き, c: 直角(mm), yaw(度)} ずらす(ピースは置いたまま)。
    //   a・c の向きは tools/vision-loop.js の e_a・e_c と同じ(a = 爪の向き u、c = u を上から見て 90° 回した向き)。
    //   当たり判定はその段の時間だけ測り(速くするため)、_check.err にその段のいちばん近い所と当たりを出す
    //   (2026-11-18、点検 C37 の頼み)ERR.pivot = "tcp": 向きのずれを道具の点の回りで回す(既定は運ぶ道の真ん中の回り)。
    //   ERR.carry(phase = "pick" のとき): つかむ段の c を、持っているピースの爪に対する位置に残して置く段まで運ぶ(a は閉じると固定の爪に
    //   押されて消える見込み【仮】。向きのずれは、前から残る)。当たりを測る時間は、真上に着く〜置いて上へ戻る まで。
    //   _ERR_PRIOR = [{pid, a, c, yaw}]: そのピースを、爪に対してずらして持たせる(置いた所が、置く向きの a・c・yaw だけずれる)。
    //   持っている間も爪に対してずれたまま運ぶ近似(a のずれは本当は爪が挟んで消える)。
    var ERR = ui._ERR || null;   // 撮る間の止まり【仮】
    var PRIOR = {};
    (ui._ERR_PRIOR || []).forEach(function (e) { PRIOR[e.pid] = e; });
    //   _PLACE_OFF = [{pid, x, z, yaw}](mm・mm・度。2026-11-20、隣の会話の列 P の頼み): そのピースの置く所だけを、机の x・z へずらし、
    //   置く爪の向きを yaw だけ回す(挟む所の真ん中の回り)。つかむ所は変えない。関節の誤差を置く姿勢で打ち消す試し用(plan-cli の plan の place_offset)。
    //   当たり判定はずらした動きで見る。「置いた位置の差」(worstPlace)にはずらした分が出る(合否には入らない)
    //   _PLACE_OFF_ARM = true(2026-11-22、plan の offset_is_arm): ずらしは、置く姿勢で腕がくり返す誤りを打ち消す分と見る。腕は本当は
    //   ずらす前の所へ行くので、置く段(盤の真上 tTop から 置いて上へ戻る tDone まで)の当たりは、腕と持っているピースを、ずらした分だけ
    //   戻した所(本当の所)で、ほかのピース・台と比べる。置き直しで、盤に置いたピースが隣の L に当たると断られた(爪を開くと置き場の
    //   ずらし +3.3mm の分だけ隣へ寄る。本当の腕では寄らない)。関節の列・つかむ段・腕どうしの当たりは変えない。
    //   戻すのは爪のまわり(j5・動く爪・手首のカメラ)と持っているピースだけ(点検 C54 の 3: 関節の誤差では根元に近い所はほとんど動かない。
    //   j1〜j4 は言った所のまま)。ずらしには、腕の誤りのほかに、前につかんだときの 爪に直角なずれ(e_c)も入るので、本当の爪は
    //   戻した所から 爪の閉じる向きに直角に 数 mm ずれうる(点検 C54 の 1: sim の置き直しで最大 約 4mm。閉じる向きは 1mm 未満)。
    //   だから窓の中は、戻した所と、そこから直角に ±_PLACE_OFF_PERP_MM(既定 4mm【仮】)動かした所の 3 つで測り、いちばん近い値を使う
    var POFF = {};
    (ui._PLACE_OFF || []).forEach(function (e) { POFF[e.pid] = e; });
    var WRIST_CHECK = WRIST && (ui.WRIST_CHECK === undefined ? 1 : (ui.WRIST_CHECK | 0));
    var ROBOT_Y = robotY(STAND);                                 // 腕の base_link の原点の、机からの高さ
    // 検算用(check-scene.js): _ROBOT_Y_MM … 腕の原点の高さを変える(2026-11-19。本家の板ありの 9.6mm で前の例を作り直すため)。
    //   台の当たり判定(standFields)は robotY(true) のまま置くので、台なし(STAND 0)でだけ使う
    if (ui._ROBOT_Y_MM !== undefined && !STAND) ROBOT_Y = ui._ROBOT_Y_MM / 1000;
    // _AIR[m]: 道具の狙いを全部これだけ上げる(安全に流す道具の決まり「最初は全部の高さに +50mm 足して、空中で流す」。tools/puzzle_run.py の --air)。
    //   当たり判定の世界(机・ピース・台)は動かさないので、腕は机とピースから AIR だけ遠くなる。台(柱・吊る腕)とは、本当の腕の姿勢で測る
    var AIR = +(ui._AIR || 0);
    function toRobot(p) { return [p[0], p[1] + AIR - ROBOT_Y, p[2]]; }  // 机 = 0 の座標 → 腕の座標(逆運動学に渡す)
    // 検算用(tools/check-scene.js): STAND_SHIFT … 台を机の上で [x, z] mm ずらす / NO_AVOID … 置き場に台を避けさせない
    var SHIFT = ui.STAND_SHIFT ? [ui.STAND_SHIFT[0] / 1000, ui.STAND_SHIFT[1] / 1000] : null;
    var STATICS = STAND ? standFields(SHIFT) : null;
    // 台の足跡(運ぶ高さ + 30mm までにある所)。置き場はここから SLOT_CLEAR 空ける
    var FOOT = STAND && !ui.NO_AVOID ? standFootprint(HOVER_Y + 0.03, SHIFT) : [];
    var maxRun = Math.max(1, Math.floor((MAX_SPAN + GAP) / P + 1e-9));
    function span(run) { return run * CELL + (run - 1) * GAP; }
    function uOf(yawDeg) { return [Math.cos(yawDeg * DEG), 0, Math.sin(yawDeg * DEG)]; }
    function offOf(run) { return GRIP.face + span(run) / 2; }   // 道具の点 → 挟む範囲の真ん中(u の向き)
    // 挟む範囲の真ん中 C に対する道具の点(固定の爪がピースに当たる)。shift = u の向きにずらす量
    function tcpFor(C, run, yawDeg, shift) {
      var off = offOf(run) - (shift || 0), u = uOf(yawDeg);
      return [C[0] - off * u[0], GRASP_Y, C[2] - off * u[2]];
    }
    var pieces = set.pieces.split(" ").map(function (s, i) { return { name: s, shape: s, idx: i }; });
    // 目標の形: 長方形(na × nb)か、文字の絵(art。"#" が埋めるマス)。盤の置き場所は、形を囲む長方形で決める
    var target = set.art ? PUZZLE.fromArt(set.art) : PUZZLE.rect(set.na, set.nb);
    if (set.art) { set.na = Math.max.apply(null, target.map(function (c) { return c[0]; })) + 1; set.nb = Math.max.apply(null, target.map(function (c) { return c[1]; })) + 1; }
    var info = [], problems = [];

    var BOARD_CENTER = placeBoard(set.na, set.nb, P), BOARD_IN = !!BOARD_CENTER;
    var layoutWhy = BOARD_CENTER ? "" : "盤(" + set.na + "×" + set.nb + "・ピッチ " + (P * 1000).toFixed(0) + "mm)が届く範囲(肩から " + (R_IN * 1000) + "〜" + (R_OUT * 1000) + "mm)に入らない";
    if (!BOARD_CENTER) BOARD_CENTER = [PAN_X + (R_IN + R_OUT) / 2, 0];
    function boardXZ(c) {
      return [BOARD_CENTER[0] + (c[0] - (set.na - 1) / 2) * P, BOARD_CENTER[1] + (c[1] - (set.nb - 1) / 2) * P];
    }
    var bLo = boardXZ([0, 0]), bHi = boardXZ([set.na - 1, set.nb - 1]);
    var boardRect = [bLo[0] - CELL / 2, bLo[1] - CELL / 2, bHi[0] + CELL / 2, bHi[1] + CELL / 2];
    FOOT.forEach(function (o) {                                  // 盤が台の足跡に近すぎないか(盤は輪の真ん中に置くので動かさない)
      var dx = Math.max(o[0] - boardRect[2], boardRect[0] - o[2], 0), dz = Math.max(o[1] - boardRect[3], boardRect[1] - o[3], 0);
      if (Math.hypot(dx, dz) < SLOT_CLEAR) layoutWhy = layoutWhy || "盤が上のカメラの台に近すぎる(" + (Math.hypot(dx, dz) * 1000).toFixed(0) + "mm)";
    });
    // LAYOUT … 置き場をファイルから読む(S-5)。無ければ、置き場をこちらで決める
    var lay = ui.LAYOUT ? readLayout(ui.LAYOUT, pieces, P, boardRect, FOOT) : placeSlots(pieces, P, boardRect, FOOT, ui._SLOT_BAN);
    var SLOTS = lay.slots, SLOT_ROT = lay.rots || [];
    pieces.forEach(function (p) { SLOT_ROT[p.idx] = SLOT_ROT[p.idx] || 0; });
    pieces.forEach(function (p, i) { if (!SLOTS[i]) SLOTS[i] = [PAN_X - 0.30, (i - 2) * 0.08]; });   // 置けなかったピースは輪の外(絵だけ)
    layoutWhy = layoutWhy || lay.why;
    // 検算用(tools/check-layout.js): 置き場を読んで確かめるだけで、動きは作らない(でたらめに置き直すときに速く選ぶ)
    if (ui.LAYOUT_ONLY) return { _check: { layoutWhy: layoutWhy, layoutNotes: lay.notes || [], foot: FOOT, slots: SLOTS } };

    var sol = (set.solver === "x" ? PUZZLE.solveX : PUZZLE.solve)(target, pieces, 10);   // 新しい種は速い版(答えの集まりは同じ。tools/check-puzzle.js)
    var plan = null, solIdx = -1;
    if (sol.solutions.length) {
      solIdx = Math.min(ui.SOL | 0, sol.solutions.length - 1);
      plan = PUZZLE.planOrder(sol.solutions[solIdx], { fingerFits: fits, maxRun: maxRun, grasp: ui.GRASP });   // GRASP = {名前: つかみ方の番号}(S1)
    }
    info.push({ label: "目標の形", value: (set.art ? "形(囲む長方形 " + set.na + "×" + set.nb + "・" + target.length + " マス)" : set.na + "×" + set.nb + " マス") + "(1 マス " + (CELL * 1000) + "mm・すき間 " + (GAP * 1000).toFixed(0) + "mm)" });
    info.push({ label: "敷き詰め方", value: sol.solutions.length ? (sol.solutions.length + (sol.solutions.length >= 10 ? " 通り以上" : " 通り") + "見つかった(いま " + (solIdx + 1) + " 番目)") : "無い(" + sol.reason + ")",
                state: sol.solutions.length ? "ok" : "ng" });

    // 待ちの姿勢: 盤の手前の上。届く所を手前から順に試す(高いと手首が可動域の端に当たる)
    var home = null;
    [[0.120, 0.090], [0.130, 0.080], [0.140, 0.075], [0.150, HOVER_Y]].some(function (h) {
      home = MOTION.topDown(toRobot([h[0], h[1], 0]), 90, [0, -30, 30, 60, 0], {});
      return home.ok;
    });
    if (!home.ok) problems.push("待ちの姿勢が解けない: " + home.reason);
    var PQ = ui._PHOTO_Q || PHOTO_Q;                    // _PHOTO_Q・_PHOTO_VIA は写す姿勢を探す道具(tools/photo-path.js)用
    var qCur = (STAND ? PQ : home.q).slice();          // 台があれば、写す姿勢から始める
    var frames = [];              // {t, q, jaw, held, touch}
    var t = 0, jawCur = GRIP.angleFor(Math.min(MAX_SPAN, CELL + 2 * FIT_MARGIN));
    var touchNow = -1;            // いま爪が触れているピース(つかむ・離す途中)
    function push(q, jaw, held) { frames.push({ t: t, q: q.slice(), jaw: jaw, held: held, touch: held >= 0 ? held : touchNow }); }
    push(qCur, jawCur, -1);
    function moveJoint(qb, held) {
      var d = maxAbsDiff(qCur, qb), dur = Math.max(0.5, d / JOINT_SPEED), n = Math.ceil(dur / DT);
      var qa = qCur.slice();
      for (var k = 1; k <= n; k++) {
        var u = k / n; u = u < 0.5 ? 2 * u * u : -1 + (4 - 2 * u) * u;
        t += dur / n;
        push(qa.map(function (v, i) { return v + (qb[i] - v) * u; }), jawCur, held);
      }
      qCur = qb.slice();
    }
    function line(pa, pb, yawA, yawB, held, exact) {
      var L = Math.hypot(pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2]);
      // 時間は、道具の点の距離と、爪の向きを回す角度の遅い方で決める(距離だけだと、短い距離で 180° 回すとき
      // 手首の回転が 398 度/秒 になった。tools/export-motion.js の (3)、2026-11-16)
      // さらに、終わりの点を逆運動学で解き、関節角の差の最大 / JOINT_SPEED でも決める(手首の回転は爪の向きと肩の回転の和で回るので、
      // 爪の向きだけでは足りない。点検 C11 の 4)。JOINT_SPEED は平均で、加減速の真ん中はその約 2 倍になる
      var qe = MOTION.topDown(toRobot(pb), yawB, qCur, { exact: exact });
      var dur = Math.max(0.4, L / LINE_SPEED, Math.abs(yawB - yawA) / JOINT_SPEED, qe.ok ? maxAbsDiff(qCur, qe.q) / JOINT_SPEED : 0), n = Math.max(2, Math.ceil(dur / DT));
      for (var k = 1; k <= n; k++) {
        var u = k / n; u = u < 0.5 ? 2 * u * u : -1 + (4 - 2 * u) * u;
        var p = [pa[0] + (pb[0] - pa[0]) * u, pa[1] + (pb[1] - pa[1]) * u, pa[2] + (pb[2] - pa[2]) * u];
        var r = MOTION.topDown(toRobot(p), yawA + (yawB - yawA) * u, qCur, { exact: exact });
        if (!r.ok) return "直線の途中で " + r.reason;
        t += dur / n; qCur = r.q.slice(); push(qCur, jawCur, held);
      }
      return "";
    }
    function jawTo(a, held) {
      var n = Math.ceil(0.4 / DT), a0 = jawCur;
      for (var k = 1; k <= n; k++) { t += 0.4 / n; jawCur = a0 + (a - a0) * k / n; push(qCur, jawCur, held); }
    }
    function hold(sec, held) {                // その姿勢のまま止まる
      var n = Math.max(1, Math.ceil(sec / DT));
      for (var k = 1; k <= n; k++) { t += sec / n; push(qCur, jawCur, held); }
    }

    // 写す姿勢と待ちの姿勢の行き来は、まっすぐ(関節を直線で補う)。いまの写す姿勢では、まっすぐでも台まで 13.5mm・腕どうしは作業の中のほうが近い
    //   (tools/photo-path.js、2026-11-17)。★前の写す姿勢 [-90, 0, 90, 90, 0] のときは、肩を先に回す経由点(photoVia)と、上腕を起こす経由点(photoPre)を挟んでいた
    //   (まっすぐだと手首のカメラが台の足に 2.8mm 入った。tools/check-scene.js の負の検算に残す)
    var photoVia = home.ok ? [home.q[0]].concat(PQ.slice(1)) : null;
    var photoPre = home.ok ? [home.q[0], -30, 70, 90, home.q[4]] : null;
    // 写す姿勢から待ちの姿勢までの経由点(写す姿勢の側から順に)。null の所は待ちの姿勢の値を使う
    var VIA = !home.ok ? [] : ui._PHOTO_VIA ? ui._PHOTO_VIA.map(function (v) { return v.map(function (x, i) { return x === null ? home.q[i] : x; }); }) : ui._OLD_VIA ? [photoVia, photoPre] : [];
    if (STAND && home.ok) { if (!ui.PHOTO_DIRECT) VIA.forEach(function (v) { moveJoint(v, -1); }); moveJoint(home.q, -1); }   // PHOTO_DIRECT は負の検算用   // 写す姿勢 → 待ちの姿勢
    var tReady = t;
    var steps = [];               // 1 ピースごとの記録
    var symUsed = [];             // 180° 回した見方で組んだピース [{pid, name}](下の views。同じ形が 2 つある組もあるので pid も)
    var SLOTS_IN = SLOTS.map(function (x) { return x.slice(); }), ROT_IN = SLOT_ROT.slice();   // 見直す前の置き場(入れた LAYOUT・placeSlots の値)
    // RELOOK の寄り道で待ちの姿勢を通すか。VIA(写す姿勢 ⇄ 待ちの姿勢 の経由点)があるときは通す:
    //   VIA は待ちの姿勢との行き来のために作った点で、置いた所・次のピースの上との間は確かめていない(_OLD_VIA で直に行くと腕どうしの下限 −0.87mm。点検 C29 の 3)
    var RELOOK_HOME = !!ui._RELOOK_HOME || VIA.length > 0;
    var relookAdded = 0;          // RELOOK の寄り道に使った時間の合計[秒](置いた所から写す姿勢へ行って止まるまで。写す姿勢から次のピースの上へは 1 手に入る)
    var solved = false, why = "", failKind = "";
    // S1(2026-11-22): 寸法で決まる断り(拒否)と、探しきれなかった断り(未解決)を分ける。寸法の断りは先に見る(前は 1 マスも挟めない辺でも
    //   maxRun を 1 にして動きを作っていた)。既定の寸法では、どれも当たらない
    var graspBad = ui.GRASP ? Object.keys(ui.GRASP).filter(function (k) { return !(/^#d+$/.test(k) ? pieces[+k.slice(1)] : pieces.some(function (p) { return p.name === k; })); }) : [];
    var dimRule = "";   // 拒否の線が、決めた定数(目安・設計の余裕)から来るときの名前(点検 C59 の 2)。verdict は「拒否(目安)」になる
    var dimWhy = graspBad.length ? "GRASP の " + graspBad.join("・") + " は盤のピースに無い(名前か #番号)" : FIT_MARGIN <= 0 ? "指の両側の余裕 0 では、下ろす指がピースの面に重なる(余裕は正の数)"
               : !isFinite(GRIP.face) ? "高さ " + (HGT * 1000).toFixed(1) + "mm では、指(机から " + (TIP_CLEAR * 1000) + "mm 浮かす)がピースの横に届かない"
               : CELL > MAX_SPAN + 1e-9 ? "辺 " + (CELL * 1000).toFixed(1) + "mm > 挟める幅の上限 " + (MAX_SPAN * 1000).toFixed(1) + "mm(閉じた爪の内側の面が高さ " + (HGT * 1000).toFixed(1) + "mm の半分より下まで届く幅)"
               : "";   // 開く方は上限にならない(挟める幅を越えて開くと、動く指はピースの高さの外へ回って出る)
    if (dimWhy) { why = dimWhy; failKind = "拒否"; dimRule = graspBad.length || FIT_MARGIN <= 0 ? "" : !isFinite(GRIP.face) ? "つかむ高さ(指先を机から TIP_CLEAR " + (TIP_CLEAR * 1000) + "mm 浮かす。机から決めている)" : "挟める幅の線(閉じた爪の内側の面が高さの半分より下まで届く。Claude の目安)"; }
    else if (!sol.solutions.length) { why = sol.reason || "敷き詰め方が無い"; failKind = sol.timedOut ? "未解決" : "拒否"; }   // 打ち切りは未解決(点検 C59 の 3)
    else if (!plan.ok) why = plan.reason + (/番は無い/.test(plan.reason) ? "" : !fits && !/挟める所が無い/.test(plan.reason) ? "(すき間 " + (GAP * 1000).toFixed(0) + "mm < 指の厚さ " + (FINGER_T * 1000).toFixed(1) + "mm + 余裕 " + (2 * FIT_MARGIN * 1000) + "mm)"
                                                   : "(" + (maxRun + 1) + " マスの幅 " + (span(maxRun + 1) * 1000).toFixed(1) + "mm > 挟める幅 " + (MAX_SPAN * 1000).toFixed(1) + "mm)");   // 点検 C37 の頼み(すき間 20・辺 14 で、理由が指の厚さと出ていた)
    else if (layoutWhy) why = layoutWhy;
    if (!dimWhy && sol.solutions.length && !plan.ok) {   // 指が下りるマス・挟める幅・無いつかみ方の番号は、寸法と入力で決まる(順番とつかみ方は全部試している)。
      //   ただし見たのは敷き詰め方 1 通り(SOL)だけなので、ほかの見つけた敷き詰め方でも断るときだけ拒否、どれかで組めれば未解決(点検 C59 の 3)
      var otherSol = -1;
      sol.solutions.forEach(function (sl, k) { if (otherSol < 0 && k !== solIdx && PUZZLE.planOrder(sl, { fingerFits: fits, maxRun: maxRun, grasp: ui.GRASP }).ok) otherSol = k; });
      failKind = otherSol >= 0 || sol.timedOut ? "未解決" : "拒否";
      if (otherSol >= 0) why += "(敷き詰め方 " + otherSol + " 番なら順番が組める)";
      if (failKind === "拒否" && /挟める所が無い/.test(plan.reason)) dimRule = "挟める幅の線(閉じた爪の内側の面が高さの半分より下まで届く。Claude の目安)";
    }
    else if (!failKind && layoutWhy) { failKind = BOARD_IN ? "未解決" : "拒否"; if (!BOARD_IN) dimRule = "届く輪(肩から " + (R_IN * 1000) + "〜" + (R_OUT * 1000) + "mm。実測 80〜240mm に余裕 5mm、盤の向きは固定)"; }   // 盤が届く範囲に入らないのは寸法(目安の輪)、ピースの置き場が取れないのは探し方
    if (!why) {
      solved = true;
      // _PIECES: 先頭の N 手だけ組んで、待ちの姿勢・写す姿勢へ戻る(tools/puzzle_run.py の --pieces。K7 は 1 個。残りのピースは置き場にあるものとして当たりを見る)
      //   keep のピース(盤の上に置き終えた物。readLayout)は手から外す。N は動かす手だけで数える(点検 C48 の 2)
      var KEEP = {};
      if (ui.LAYOUT) ((ui.LAYOUT && ui.LAYOUT.pieces) || []).forEach(function (e, i) { if (e && e.keep) KEEP[i] = 1; });
      var NMOVE = ui._PIECES ? Math.max(1, ui._PIECES | 0) : Infinity, nMoved = 0;
      for (var s = 0; s < plan.steps.length && nMoved < NMOVE && solved; s++) {
        var st = plan.steps[s], pl = st.pl, pid = pl.piece;
        if (KEEP[pid]) continue;
        nMoved++;
        var tLook0 = 0, tLook1 = 0, tBack = 0, seedQ = null;
        if (RELOOK && nMoved > 1 && home.ok) {                         // 写す姿勢へ寄り道して撮り直す(上の RELOOK)
          var tL = t; seedQ = qCur.slice();                       // 次のピースの上の逆運動学は、寄り道の前の姿勢から解く(待ちの姿勢から解くと手首の回転の解が変わり、
                                                                  //   3×4・すき間 0 の J が手首の回転の可動域で組めなかった。2026-11-18)
          if (RELOOK_HOME) moveJoint(home.q, -1);                  // 行きも帰りも待ちの姿勢を通らない(置いた所 → 写す姿勢 → 次のピースの上)。下の帰りのコメント
          if (!ui.PHOTO_DIRECT) VIA.slice().reverse().forEach(function (v) { moveJoint(v, -1); });
          moveJoint(PQ, -1); tLook0 = t; hold(RELOOK_SHOT, -1); tLook1 = t;
          if (!ui.PHOTO_DIRECT) VIA.forEach(function (v) { moveJoint(v, -1); });
          if (RELOOK_HOME) moveJoint(home.q, -1);                 // 帰りは写す姿勢から次のピースの上へ直に行く(下の moveJoint(hp.q)。その動きは次のピースの 1 手に入る)。
                                                                  //   待ちの姿勢を通すと手首の回転が待ちの姿勢の値まで回って戻り、絵から読んだ置き場の 3×4 で O の上へ 231°・2.57 秒、
                                                                  //   腕どうし 2.62mm(隣の会話の点検 C29 の 1)。16 通りで、行きも帰りも直にして当たり 0・腕どうしと机の上の近さは同じ、
                                                                  //   既定の 3×4 で全体 29.2 → 26.2 秒(寄り道なしは 23.3 秒)。_RELOOK_HOME は前の作り(待ちの姿勢を通す)との比べ用。
                                                                  //   手首の回転の大回りは減っただけ(写す姿勢の wrist_roll は 0。既定の 3×4 の J で 写す姿勢 → 上 122°・1.36 秒。点検 C29 の 4)
          tBack = t;
          relookAdded += t - tL;
        }
        var slot = SLOTS[pid];
        var gl = st.grasp.center, run = st.grasp.run;            // 局所の挟む範囲の真ん中
        var bxz = boardXZ(st.board.center);
        var po = POFF[pid] || {};
        var cT = [bxz[0] + (+po.x || 0) / 1000, GRASP_Y, bxz[1] + (+po.z || 0) / 1000];
        var axisDeg = Math.atan2(st.grasp.axis[1], st.grasp.axis[0]) / DEG;
        // 置き場の見方(置き場の位置・向きと、そこから決まる つかむ場所・爪の向き・回す角度)。
        //   v = 0 はふつうの見方。v = 1 は、180° 回して同じ形になるピース(Z・S・I・O・I3 など)を「180° 回った置き場」と見直したもの:
        //   局所の原点を対称の中心の反対側へ移し、向きを +180° にする。机の上の立方体は同じ所のまま、つかむ所は対称の反対のマスになり、
        //   盤のどこへ運ぶかは同じ。ふつうの見方の試し方が全部だめなときだけ使う(2026-11-18。種 1 の Z は、置き場の向きが ±20° ずれると
        //   片方の見方では手首の回転の可動域に入らない。向きを 15° おきに 1 周みると、どの向きでも 2 つの見方のどちらかで組めた)
        function slotView(sl, r) {
          var glw = rotY([gl[0] * P, 0, gl[1] * P], ui._FLIP === "glw" ? -r : r);   // 置き場で回っている分(S-5。_FLIP は負の検算用)
          return { slot: sl, r0: r, cS: [sl[0] + glw[0], GRASP_Y, sl[1] + glw[2]], yawS: axisDeg + (ui._FLIP === "yaw" ? -r : r),
                   dBlock: MOTION.wrap180(pl.k * 90 - r) };                           // ピースを回す角度(置き場の向きから盤の向きへ)
        }
        var views = [slotView(slot, SLOT_ROT[pid])];
        var symC = ui.NO_SYM ? null : sym180(pl.shape);   // NO_SYM は負の検算用(tools/check-scene.js)
        if (symC) { var so = rotY([2 * symC[0] * P, 0, 2 * symC[1] * P], SLOT_ROT[pid]); views.push(slotView([slot[0] + so[0], slot[1] + so[2]], MOTION.wrap180(SLOT_ROT[pid] + 180))); }
        var jClosed = GRIP.angleFor(span(run)), jOpen = GRIP.angleFor(span(run) + 2 * FIT_MARGIN);
        function triesFor(vi) {
          var V = views[vi], out = [];
          [V.yawS, V.yawS + 180].forEach(function (yp) {
            out.push([yp, V.dBlock, null, vi]);
            if (Math.abs(V.dBlock) > 90 + 1e-6) out.push([yp, V.dBlock - 360 * Math.sign(V.dBlock), null, vi]);   // 反対回り(180° は両回り)
          });
          if (typeof SELF_COLLIDE !== "undefined" && ui.SELF !== 0) {
            var base0 = out.slice(), L = CARRY_LIFT;
            [[L, L, L], [0, L, L], [L, L, 0], [0, L, 0]].forEach(function (lf) { base0.forEach(function (x) { out.push([x[0], x[1], lf, vi]); }); });
          }
          return out;
        }
        // 運ぶ高さを CARRY_LIFT 上げた試し方を後ろに足す(triesFor の中。腕どうしが WARN 未満のときだけ使われる)。
        //   台なしの 4×5 で、S を肩の近く(肩から約 89mm)へ運びながら手首を 29°→101° 回すと、手首のカメラ台が上腕に 3mm 以上入った。
        //   20mm 上げると腕が開き、同じ回し方で 3.0mm 空いた(2026-11-17)。40mm 上げると手首の可動域を超えた。
        //   遠い置き場では上げると届かないので、[取る側, 真ん中, 置く側] の上げ方を 4 通り試す(運ぶ間は、真ん中を経由して高さを直線で変える)
        var done = false, lastWhy = "", bestTry = null, whyKeys = [];
        // 爪の向きの試し方(yp と yp+180°・回す向き)を全部組み、腕どうしの隙間(self-collision.js)が WARN 以上の最初のもの、
        // 無ければ隙間のいちばん大きいものを使う(手首を大きく回す解で、手首のカメラが上腕に 0.5mm まで寄った。2026-11-17)
        var SELF_PICK = typeof SELF_COLLIDE !== "undefined" && ui.SELF !== 0;
        for (var pass = 0; pass < views.length && !done; pass++) {   // 見方 0 の試し方が全部だめなときだけ、見方 1(180° 回した見方)を試す
        var tries = triesFor(pass);
        for (var tr = 0; tr < tries.length && !(bestTry && bestTry.self >= (SELF_PICK ? SELF_COLLIDE.WARN : -Infinity)); tr++) {
          var save = { t: t, n: frames.length, q: qCur.slice(), jaw: jawCur };
          var V = views[tries[tr][3]], cS = V.cS, yawS = V.yawS, r0 = V.r0;
          var yp = tries[tr][0], dlt = tries[tr][1], yT = yp + dlt + (+po.yaw || 0), lift = tries[tr][2] || [0, 0, 0], CYs = HOVER_Y + lift[0], CYm = HOVER_Y + lift[1], CYt = HOVER_Y + lift[2];
          var gS = tcpFor(cS, run, yp), gSa = tcpFor(cS, run, yp, -FIT_MARGIN);   // gSa = 固定の爪を 2mm 離した所
          var gT = tcpFor(cT, run, yT), gTa = tcpFor(cT, run, yT, -FIT_MARGIN);
          var ypS = yp, yTT = yT;                                  // 爪の向き(_ERR の yaw を足したもの。ふつうは yp・yT のまま)
          if (ERR && ERR.pid === pid) {
            var eY = (ERR.yaw || 0), uE = uOf(ERR.phase === "pick" ? yp : yT), eA = (ERR.a || 0) / 1000, eC = (ERR.c || 0) / 1000;
            var dE = [eA * uE[0] - eC * uE[2], eA * uE[2] + eC * uE[0]];
            var shE = function (q) { q[0] += dE[0]; q[2] += dE[1]; };
            var pivT = ERR.pivot === "tcp";                       // 道具の点の回り: 位置は名目の向きで作り、向きだけ足す
            if (ERR.phase === "pick") { ypS = yp + eY; if (!pivT) { gS = tcpFor(cS, run, ypS); gSa = tcpFor(cS, run, ypS, -FIT_MARGIN); } [gS, gSa].forEach(shE); }
            else { yTT = yT + eY; if (!pivT) { gT = tcpFor(cT, run, yTT); gTa = tcpFor(cT, run, yTT, -FIT_MARGIN); } [gT, gTa].forEach(shE); }
          }
          var hS = [gSa[0], HOVER_Y, gSa[2]], hT = [gT[0], HOVER_Y, gT[2]], hTa = [gTa[0], HOVER_Y, gTa[2]];
          var hp = MOTION.topDown(toRobot(hS), ypS, seedQ || qCur, { exact: true });
          var err = hp.ok ? "" : ("つかむ場所の上へ " + hp.reason);
          var tPick = 0, tPlace = 0, tHover = 0, tCheck0 = 0, tCheck1 = 0;
          if (!err) { moveJoint(hp.q, -1); tHover = t; jawTo(jOpen, -1); }
          if (!err && WRIST_CHECK) { tCheck0 = t; hold(WRIST_SETTLE + WRIST_SHOT, -1); tCheck1 = t; }   // 手首のカメラで確かめる(想定動作の 3 段)   // tHover = つかむ場所の真上に着いた時刻(手首のカメラで確かめる所。tools/camera-budget.js)
          if (!err) err = line(hS, gSa, ypS, ypS, -1, true);                   // 真下へ下ろす
          var tDown = t;                                          // 下ろし終えた時刻(S1 の段: 開いて下ろす | 寄せて閉じる)
          if (!err) { touchNow = pid; err = line(gSa, gS, ypS, ypS, -1, true); } // 横に寄せて固定の爪を当てる
          if (!err) { jawTo(jClosed, -1); tPick = t; }
          if (!err) err = line(gS, [gS[0], CYs, gS[2]], ypS, ypS, pid, true);
          var tLift = t;                                          // つかんで真上へ上げ終えた時刻(_ARM_MARGIN_MM のつかむ段の終わり)
          var pM = [(gS[0] + hT[0]) / 2, CYm, (gS[2] + hT[2]) / 2], yM = (ypS + yTT) / 2;
          if (!err && !lift[1]) err = line([gS[0], CYs, gS[2]], [hT[0], CYt, hT[2]], ypS, yTT, pid, true);
          else if (!err) { err = line([gS[0], CYs, gS[2]], pM, ypS, yM, pid, true); if (!err) err = line(pM, [hT[0], CYt, hT[2]], yM, yTT, pid, true); }
          var tTop = t;                                           // 盤の真上に着いた時刻(_ERR の place の段の始め)
          if (!err) err = line([hT[0], CYt, hT[2]], gT, yTT, yTT, pid, true);
          if (!err) { tPlace = t; jawTo(jOpen, -1); }
          var tRel = t;                                           // 置いて爪を開き終えた時刻(S1 の段: 開く | 抜く)
          if (!err) err = line(gT, gTa, yTT, yTT, -1, true);                    // 横に戻して離す
          touchNow = -1;
          if (!err) err = line(gTa, hTa, yTT, yTT, -1, true);
          var tDone = t;                                          // 置いて上へ戻った時刻(tools/cycle-tact.js)
          if (err && ui._TRYLOG) ui._TRYLOG.push({ piece: pl.name, view: pass, yp: yp, dlt: dlt, err: err });
          if (err) {                                             // やり直し(この試しを捨てる)
            var wkey = (pass ? "180° 回した見方で " : "") + err;     // 同じ理由は 1 度だけ書く(見方ごとに。点検 C28 の 5)
            if (whyKeys.indexOf(wkey) < 0) { whyKeys.push(wkey); lastWhy += (lastWhy ? " / " : "") + (pass ? "180° 回した見方で " : "") + "爪の向き " + Math.round(MOTION.wrap180(yp)) + "°: " + err; }
            t = save.t; frames.length = save.n; qCur = save.q; jawCur = save.jaw;
            continue;
          }
          done = true;
          var rec = { lift: lift, pid: pid, name: pl.name, k: pl.k, slot: V.slot, view: tries[tr][3], r0: V.r0, gl: gl, run: run, tPick: tPick, tPlace: tPlace, tHover: tHover, tLift: tLift, tDone: tDone, tDown: tDown, tRel: tRel, tCheck0: tCheck0, tCheck1: tCheck1, tLook0: tLook0, tLook1: tLook1, tBack: tBack, tTop: tTop, pl: pl, cS: cS, cT: cT, yp: yp,
                       yWant: axisDeg + r0 + (yp - yawS) };   // 正しい掴む向き(_FLIP に依らない)
          var selfMin = SELF_PICK ? SELF_COLLIDE.motionCheck(frames.slice(Math.max(0, save.n - 1))).min : Infinity;
          if (ui._TRYLOG) ui._TRYLOG.push({ piece: pl.name, view: pass, yp: yp, dlt: dlt, lift: lift, self: selfMin });
          if (!bestTry || selfMin > bestTry.self) bestTry = { self: selfMin, rec: rec, fr: frames.slice(save.n), t: t, q: qCur.slice(), jaw: jawCur };
          t = save.t; frames.length = save.n; qCur = save.q; jawCur = save.jaw;     // いったん戻し、いちばん良いものをあとで入れる
        }
        }
        if (bestTry) { bestTry.fr.forEach(function (f) { frames.push(f); }); t = bestTry.t; qCur = bestTry.q; jawCur = bestTry.jaw; steps.push(bestTry.rec);
          if (bestTry.rec.view === 1) { SLOTS[pid] = bestTry.rec.slot; SLOT_ROT[pid] = bestTry.rec.r0; symUsed.push({ pid: pid, name: pl.name }); } }   // 180° 回した見方で組んだ: 以後の絵・当たり・検算はこの見方(机の上の立方体は同じ所)
        if (!done) { solved = false; failKind = "未解決"; why = pl.name + " のピース: " + lastWhy +
                     (sol.solutions.length > 1 ? "(敷き詰め方の番号を変えると解けることがある)" : ""); }
      }
      if (solved) moveJoint(home.q, -1);
      if (solved && STAND) { if (!ui.PHOTO_DIRECT) VIA.slice().reverse().forEach(function (v) { moveJoint(v, -1); }); moveJoint(PQ, -1); }   // 最後に写す姿勢へ(盤を上のカメラで見る)
    }
    t += 1.0; push(qCur, jawCur, -1);                           // 最後に 1 秒止まる
    var T = t;

    // ---- 当たり判定の相手(ピースの立方体)と、持ったピースの点 ----
    var cubeH = [CELL / 2, HGT / 2, CELL / 2];                  // 高さ HGT(既定は辺)
    var cubePts = COLLIDE.surfacePoints(cubeH, 0.003);
    var poseNow = {};                                            // ピースのいまの位置 {pos, rot}
    pieces.forEach(function (p) { var sl = SLOTS[p.idx] || [0.3, 0.3]; poseNow[p.idx] = { pos: [sl[0], 0, sl[1]], rot: SLOT_ROT[p.idx] }; });
    function cubesOf(pid, pose) {
      pose = pose || poseNow[pid];
      var c = Math.cos(pose.rot * DEG), sn = Math.sin(pose.rot * DEG);
      var M = [c, 0, -sn, 0, 1, 0, sn, 0, c];
      return PUZZLE.SHAPES[pieces[pid].shape].map(function (cl) {
        var o = rotY([cl[0] * P, 0, cl[1] * P], pose.rot);
        return { id: "pc" + pid, group: "pc" + pid, c: [pose.pos[0] + o[0], pose.pos[1] + HGT / 2, pose.pos[2] + o[2]], M: M, h: cubeH };
      });
    }

    // ---- コマごと: 持っているピースの位置(URDF の行列の積)と当たり判定 ----
    var pieceTracks = {};
    pieces.forEach(function (p) { pieceTracks[p.idx] = []; });
    var grasp0 = {};                                             // 掴んだ瞬間の 爪の向き・局所のつかむマス
    var worstPick = 0, worstPickYaw = 0;                          // 掴んだ瞬間の、置き場との位置の差・爪の向きの差
    var refineN = 0, minClearCoarse = Infinity, refineMaxGain = 0, refineMaxLoss = 0;   // S-8: 測り直した回数・粗い判定のいちばん近い所・粗い→細かいで近くなった最大・遠くなった最大
    var hitFrames = [];                                          // [{t, ids, pairs}]
    var minClear = Infinity, minClearPair = "", minClearAt = 0;  // ふつうの組のいちばん近い所
    var minClearFrame = Infinity;   // コマの上だけで測ったいちばん近い所(コマの間に足した判定を除く。tools/check-scene.js の「コマの間」の負の検算が読む)
    var clearAt = [];                                            // [{t, d, pair}] コマ(と間に足した判定)ごとのいちばん近い所
    var phaseAt = [];                                            // [{t, pc, pcPair, tb, tbPair}] S1: コマごとの、ほかのピース(触れている・持っているピースを除く)と机までのいちばん近い所
    var minContact = Infinity, minContactPair = "";              // つかんでいるピースとの接触の組
    var minTip = Infinity, minTipAt = 0;
    var minCam = Infinity, minCamPair = "", minCamAt = 0;        // 手首のカメラと台がいちばん近づいた所(机・ピース・台)
    var minStand = Infinity, minStandPair = "", minStandAt = 0;  // 上のカメラの台にいちばん近づいた所(面から 15mm 以内だけ測る)
    var minStandFrame = Infinity;   // 台との近さを、コマの上だけで(コマの間に足した判定を除く。check-scene.js の「コマの間」の負の検算が読む。2026-11-19)
    var groups = ["base", "j1", "j2", "j3", "j4", "j5", "jaw"].concat(WRIST ? ["cam"] : []);
    // コマの間も測る。速い所では腕の点が 1 コマ(0.05 秒)で 30mm ほど動き、辺 14mm のピースや台の足をすり抜けうる
    // (2026-11-16: 写す姿勢と待ちの姿勢をまっすぐ結ぶと、コマでは台まで 3.6mm、間では −2.8mm だった)。
    // 腕の点(5 つに 1 つ)が 1 コマで SUB_STEP より動く所は、関節角を線形に割った間のコマも、次のコマの持ち方で判定する
    // 持っているピースの代わりに、道具の点から R_HELD 離れた 6 点(爪と一緒に動く)も測る(点検 C11 の 5)
    var R_HELD = Math.max.apply(null, pieces.map(function (p) { return Math.max.apply(null, PUZZLE.SHAPES[p.shape].map(function (c) { return Math.max(Math.abs(c[0]), Math.abs(c[1])); })); })) * P + CELL;
    function samplePts(q, jaw) {
      var st0 = KIN.fk(SO101.axes, q, SO101.tool), out = [], f5 = st0.frames[4];
      [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]].forEach(function (e) {
        out.push(addv(st0.pos, KIN.matVec(f5.R, [e[0] * R_HELD, e[1] * R_HELD, e[2] * R_HELD])));
      });
      groups.forEach(function (gname) {
        if (gname === "base") return;
        var fr = st0.frames[(gname === "jaw" || gname === "cam") ? 4 : (+gname.slice(1) - 1)];
        ARM.points[gname].forEach(function (p0, k) {
          if (k % 5) return;
          var p = gname === "jaw" ? jawRotate(p0, jaw) : p0;
          out.push(addv(fr.p, KIN.matVec(fr.R, subv(p, fr.o))));
        });
      });
      return out;
    }
    var checks = [], prevS = null;
    frames.forEach(function (f, fi) {
      var sp = samplePts(f.q, f.jaw);
      if (prevS) {
        var dmax = 0;
        var heldNow = f.held >= 0 || frames[fi - 1].held >= 0;
        sp.forEach(function (p, k) { if (k < 6 && !heldNow) return; var o = prevS[k]; dmax = Math.max(dmax, Math.hypot(p[0] - o[0], p[1] - o[1], p[2] - o[2])); });
        var m = Math.ceil(dmax / SUB_STEP), a = frames[fi - 1];
        for (var s2 = 1; s2 < m; s2++) {
          var u = s2 / m;
          checks.push({ t: a.t + (f.t - a.t) * u, q: a.q.map(function (v, i) { return v + (f.q[i] - v) * u; }), jaw: a.jaw + (f.jaw - a.jaw) * u,
                        held: f.held, touch: f.touch, fi: fi - 1, sub: true });
        }
      }
      checks.push({ t: f.t, q: f.q, jaw: f.jaw, held: f.held, touch: f.touch, fi: fi, sub: false });
      prevS = sp;
    });
    var subChecks = checks.length - frames.length;
    // _ERR: ずらした段の時間だけ当たりを測る(持っているピースの位置は、続けてほどくので全部のコマで追う)
    // _PLACE_OFF_ARM: 置く段の窓。c1 = ずらした挟む所の真ん中、c0 = ずらす前(机の x・z[m])、yaw = 回した分[度]
    var offWins = ui._PLACE_OFF_ARM ? steps.filter(function (x) { return POFF[x.pid]; }).map(function (x) {
      var po = POFF[x.pid];
      return { t0: x.tTop, t1: x.tDone, c1: [x.cT[0], x.cT[2]], c0: [x.cT[0] - (+po.x || 0) / 1000, x.cT[2] - (+po.z || 0) / 1000], yaw: +po.yaw || 0 };
    }) : [];
    function offWinAt(t) { for (var i = 0; i < offWins.length; i++) if (t >= offWins[i].t0 - 1e-9 && t <= offWins[i].t1 + 1e-9) return offWins[i]; return null; }
    function unOff(w, p) { var d = rotY([p[0] - w.c1[0], 0, p[2] - w.c1[1]], -w.yaw); return [w.c0[0] + d[0], p[1], w.c0[1] + d[2]]; }   // 腕の言った所 → 本当の所
    var OFF_PERP = (ui._PLACE_OFF_PERP_MM === undefined ? 4 : +ui._PLACE_OFF_PERP_MM) / 1000;
    var OFF_GROUPS = { j5: 1, jaw: 1, cam: 1 };
    // _ARM_MARGIN_MM(2026-11-22、plan の arm_margin_mm): 腕は言った所から数 mm ずれて着く(K1 の誤り。sim の --arm-err 1 で、置き直しのずらしは 1.5〜6.2mm)。
    //   置き場のピースは本当の所(上のカメラの読み)にあるので、言った所の爪で比べると、ずれの分だけ近さを見落とす(点検 C54 の 3 の残り)。
    //   つかむ段(つかむ所の真上 tHover から、つかんで上げ終える tLift まで)と置く段(tTop から tDone まで)は、爪のまわり(j5・動く爪・手首のカメラ)と
    //   持っているピースを、机の x・z に 8 つの向きへ _ARM_MARGIN_MM 動かした所でも測り、ほかのピース(いまつかむ・置くピースを除く)との組は
    //   いちばん近い値を使う。_PLACE_OFF_ARM の窓は前のまま(ずらしを戻した所と ±OFF_PERP)。既定(無し)は前と同じ数
    //   _TRUE_BIAS(sim だけ。plan の true_bias_deg): 本当の腕 = 言った角度 + 関節の誤差(URDF の度)で、同じ段の爪のまわり・腕とほかのピースの
    //   いちばん近い所を測って答えに出す(合否には入れない。sim の採点用)
    var ARM_M = (+ui._ARM_MARGIN_MM || 0) / 1000;
    var TRUE_BIAS = Array.isArray(ui._TRUE_BIAS) ? ui._TRUE_BIAS : null;
    var armWins = (ARM_M > 0 || TRUE_BIAS) ? [].concat.apply([], steps.map(function (x) { return [[x.tHover, x.tLift, WRIST_CHECK && x.tCheck1 > 0 ? x.pid : -1], [x.tTop, x.tDone, x.pid]]; })) : [];
    //   つかむ段のピースは、手首のカメラで確かめる手だけ相手にしない: 腕に付いたカメラで測り直すので、爪とそのピースの間には腕のずれが乗らない
    //   (点検 C55 の 4。手首で確かめない手は、上のカメラの読みのままなので相手にする)。置く段のピースは持っている・触れているピースで、もともと外れる
    //   8 つの向きの半径は M / cos 22.5°(八角形の内側の円が半径 M。点検 C55 の 5)。上下と、手首の回りの振れは入れていない
    var RING_R = ARM_M / Math.cos(Math.PI / 8);
    function cubesShift(pid, d) { return cubesOf(pid).map(function (b) { return { id: b.id, group: b.group, c: [b.c[0] + d[0], b.c[1], b.c[2] + d[1]], M: b.M, h: b.h }; }); }
    function inArmWin(t) { for (var i = 0; i < armWins.length; i++) if (t >= armWins[i][0] - 1e-9 && t <= armWins[i][1] + 1e-9) return armWins[i]; return null; }
    var trueClear = { d: Infinity, pair: "", t: 0 }, ringMin = { d: Infinity, pair: "", t: 0 };
    var errStep = ERR ? steps.filter(function (x) { return x.pid === ERR.pid; })[0] : null;
    var errWin = errStep ? (ERR.phase === "pick" ? [errStep.tHover, ERR.carry ? errStep.tDone : errStep.tPick] : [errStep.tTop, errStep.tDone]) : null;
    var errRes = errWin ? { win: errWin, d: Infinity, pair: "", t: 0, hits: 0, hitPairs: [] } : null;
    checks.forEach(function (f) {
      var fi = f.fi, ow = offWins.length ? offWinAt(f.t) : null, heldAt = null;
      var F = SO101.urdfFK(f.q.map(function (v) { return v * DEG; }), 0);
      var st = KIN.fk(SO101.axes, f.q, SO101.tool);
      if (f.held >= 0) {
        var g = F.gripper_frame;
        var up = KIN.matVec(F.gripper_link.R, [1, 0, 0]);
        var jawYaw = Math.atan2(up[2], up[0]) / DEG;
        var gi = grasp0[f.held];
        if (!gi) {
          var stp = steps.filter(function (x) { return x.pid === f.held; })[0];
          gi = grasp0[f.held] = { yaw: jawYaw, last: 0, gl: stp.gl, off: offOf(stp.run), first: true, yWant: stp.yWant };
          // 爪に対するずれ [挟む向き, 直角, 向き](_ERR の carry と _ERR_PRIOR)。s: いまの爪の挟む向き(up)が、名目の爪の向き u(yp)と同じ向きなら +1
          var uh0 = Math.hypot(up[0], up[2]) || 1, s0 = (up[0] * Math.cos(stp.yp * DEG) + up[2] * Math.sin(stp.yp * DEG)) / uh0 >= 0 ? 1 : -1;
          gi.dj = [0, 0, 0];
          if (ERR && ERR.carry && ERR.phase === "pick" && ERR.pid === f.held) gi.dj[1] -= s0 * (ERR.c || 0) / 1000;   // 爪が +c にずれて閉じた → ピースは爪に対して −c
          if (PRIOR[f.held]) { var pe = PRIOR[f.held]; gi.dj[0] += s0 * (pe.a || 0) / 1000; gi.dj[1] += s0 * (pe.c || 0) / 1000; gi.dj[2] += pe.yaw || 0; }
        }
        var psi = gi.last + MOTION.wrap180(jawYaw - gi.yaw - gi.last);  // 続けてほどく
        gi.last = psi;
        var uh = Math.hypot(up[0], up[2]) || 1;                    // 挟む向き(水平)
        var hx = [up[0] / uh, up[2] / uh], hz = [-up[2] / uh, up[0] / uh];   // 挟む向きと、それを上から見て 90° 回した向き(uOf の a・c と同じ回し方)
        var center = [g.p[0] + gi.off * up[0] / uh, g.p[1] + ROBOT_Y - (GRASP_Y - HGT / 2), g.p[2] + gi.off * up[2] / uh];
        if (gi.dj[0] || gi.dj[1]) { center[0] += gi.dj[0] * hx[0] + gi.dj[1] * hz[0]; center[2] += gi.dj[0] * hx[1] + gi.dj[1] * hz[1]; }   // 既定(ずれなし)は前と同じ数
        var rr = SLOT_ROT[f.held] + psi + gi.dj[2];              // ピースの向き = 置き場の向き + 掴んでから回した分(+ _ERR_PRIOR の向き)
        var off = rotY([gi.gl[0] * P, 0, gi.gl[1] * P], rr);
        var pos = [center[0] - off[0], center[1] - HGT / 2, center[2] - off[2]];
        if (gi.first) {                                          // 掴んだ最初の判定: 爪から作ったピースの位置が置き場と合うか(点検 C11 の 2)
          gi.first = false;
          var sl0 = SLOTS[f.held], ePick = Math.hypot(pos[0] - sl0[0], pos[2] - sl0[1]);
          var eYaw = Math.abs(MOTION.wrap180(2 * (jawYaw - gi.yWant)) / 2);   // 爪の向き(180° で同じ)と、ピースの向きから決まる掴む向きの差
          if (ePick > worstPick) worstPick = ePick;
          if (eYaw > worstPickYaw) worstPickYaw = eYaw;
        }
        pieceTracks[f.held].push({ t: f.t, pos: pos, rot: rr });
        poseNow[f.held] = ow ? { pos: unOff(ow, pos), rot: rr - ow.yaw } : { pos: pos, rot: rr };   // 当たりは本当の所で(絵と置いた位置は言った所のまま)
        heldAt = function (Fx) {                                 // _TRUE_BIAS: 別の腕の姿勢 Fx(urdfFK)で、同じつかみ方のピースの位置(上と同じ式。向きは爪の回った分だけ足す)
          var gx = Fx.gripper_frame, upx = KIN.matVec(Fx.gripper_link.R, [1, 0, 0]), uhx = Math.hypot(upx[0], upx[2]) || 1;
          var cx = [gx.p[0] + gi.off * upx[0] / uhx, gx.p[1] + ROBOT_Y - (GRASP_Y - HGT / 2), gx.p[2] + gi.off * upx[2] / uhx];
          if (gi.dj[0] || gi.dj[1]) { cx[0] += gi.dj[0] * upx[0] / uhx - gi.dj[1] * upx[2] / uhx; cx[2] += gi.dj[0] * upx[2] / uhx + gi.dj[1] * upx[0] / uhx; }
          var rx = rr + MOTION.wrap180(Math.atan2(upx[2], upx[0]) / DEG - jawYaw), ox = rotY([gi.gl[0] * P, 0, gi.gl[1] * P], rx);
          return { pos: [cx[0] - ox[0], cx[1] - HGT / 2, cx[2] - ox[2]], rot: rx };
        };
        if (ui._HELD_LOG && !f.sub) ui._HELD_LOG.push({ t: f.t, pid: f.held, pos: pos, rot: rr, check: poseNow[f.held] });   // 検算用(持っているピースの言った所と、当たりに使った所)
      }
      if (errWin && (f.t < errWin[0] - 1e-9 || f.t > errWin[1] + 1e-9)) return;
      var errBad = false;   // _ERR: ふつうの組の当たり(触れてよい組の深さは数えない。つかむ段で固定の爪がピースを押すのは、ピースが滑るだけ)
      // 腕の点を動かす(リンク i の点 = p + R(点 − o)。動く爪は先に爪の角度で回す)
      //   sh: _PLACE_OFF_ARM の窓で、爪のまわりと持っているピースを、戻した所からさらに動かす分(机の x・z[m])。窓の外は null
      //   (_ARM_MARGIN_MM の窓では ow が無く、ずらしを戻さずに sh だけ動かす。stX: 腕の姿勢を差し替える(_TRUE_BIAS))
      function buildMovers(sh, stX) {
        var stU = stX || st;
        var mvs = groups.map(function (gname) {
          var fr = gname === "base" ? null : stU.frames[(gname === "jaw" || gname === "cam") ? 4 : (+gname.slice(1) - 1)];
          function place(p0) {
            var p = gname === "jaw" ? jawRotate(p0, f.jaw) : p0;
            var w = fr ? addv(fr.p, KIN.matVec(fr.R, subv(p, fr.o))) : p;
            var q = [w[0], w[1] + ROBOT_Y, w[2]];           // 腕の座標 → 机 = 0 の座標
            if (sh && OFF_GROUPS[gname]) { if (ow) q = unOff(ow, q); q[0] += sh[0]; q[2] += sh[1]; }
            return q;
          }
          var sp = ARM.spheres[gname];
          return { id: gname, group: gname, c: place(sp.c), r: sp.r + (gname === "jaw" ? 0.01 : 0), ignoreTable: gname === "base", ignoreStatic: gname === "base",
                   pw: function () { return ARM.points[gname].map(place); } };
        });
        var bxs = [];
        pieces.forEach(function (p) {
          var ps = sh && p.idx === f.held ? { pos: [poseNow[p.idx].pos[0] + sh[0], poseNow[p.idx].pos[1], poseNow[p.idx].pos[2] + sh[1]], rot: poseNow[p.idx].rot } : null;
          bxs = bxs.concat(cubesOf(p.idx, ps));
        });
        if (f.held >= 0) {                                       // 持っているピースも動く側
          var hb = bxs.filter(function (b) { return b.id === "pc" + f.held; });
          var c0 = hb.length ? [poseNow[f.held].pos[0] + (sh ? sh[0] : 0), poseNow[f.held].pos[1], poseNow[f.held].pos[2] + (sh ? sh[1] : 0)] : poseNow[f.held].pos;
          mvs.push({ id: "pc" + f.held, group: "pc" + f.held, c: [c0[0], c0[1] + HGT / 2, c0[2]], r: 0.12, ignoreTable: false,
                     pw: function () {
                       var out = [];
                       hb.forEach(function (b) { cubePts.forEach(function (q) { out.push(addv(b.c, KIN.matVec(b.M, q))); }); });
                       return out;
                     } });
        }
        return { movers: mvs, boxes: bxs };
      }
      var built = buildMovers(ow ? [0, 0] : null), movers = built.movers, boxes = built.boxes;
      var res = COLLIDE.frame(movers, boxes, null, null, STATICS);
      if (ow && OFF_PERP > 0) {                                  // 爪の閉じる向きに直角に ±OFF_PERP も測り、組ごとにいちばん近い値(点検 C54 の 1)
        var upW = KIN.matVec(F.gripper_link.R, [1, 0, 0]), uhW = Math.hypot(upW[0], upW[2]) || 1, hzW = [-upW[2] / uhW, upW[0] / uhW];
        [1, -1].forEach(function (sg) {
          var bv = buildMovers([sg * OFF_PERP * hzW[0], sg * OFF_PERP * hzW[1]]), rv = COLLIDE.frame(bv.movers, bv.boxes, null, null, STATICS);
          Object.keys(rv.pairs).forEach(function (k) { if (!(k in res.pairs) || rv.pairs[k] < res.pairs[k]) res.pairs[k] = rv.pairs[k]; });
        });
      }
      var heldId = f.held >= 0 ? "pc" + f.held : "", touchId = f.touch >= 0 ? "pc" + f.touch : "";
      var inAW = armWins.length ? inArmWin(f.t) : null, stepId = inAW ? "pc" + inAW[2] : "";
      function otherPiecePair(ab, arm) {   // 爪のまわり(arm: 腕の全部)か持っているピース ⇄ ほかのピース(いまつかむ・持っている・その段のピースを除く)
        return ((arm ? (ab[0] !== "base" && groups.indexOf(ab[0]) >= 0) : OFF_GROUPS[ab[0]]) || (heldId && ab[0] === heldId)) &&
               ab[1].indexOf("pc") === 0 && ab[1] !== touchId && ab[1] !== heldId && ab[1] !== stepId;
      }
      var ringK = null;                                          // _ARM_MARGIN_MM: 組 → [[動かした分, 粗い距離]](名目の 0 も入れる)
      if (ARM_M > 0 && !ow && inAW) {                            // 8 つの向きへ動かした所でも測る
        ringK = {};
        var nomPairs = Object.assign({}, res.pairs);
        for (var ka = 0; ka < 8; ka++) {
          var an = ka * Math.PI / 4, shK = [RING_R * Math.cos(an), RING_R * Math.sin(an)], bv = buildMovers(shK), rv = COLLIDE.frame(bv.movers, bv.boxes, null, null, STATICS);
          Object.keys(rv.pairs).forEach(function (k) {
            if (!otherPiecePair(k.split("|"), false)) return;
            if (!ringK[k]) ringK[k] = k in nomPairs ? [[[0, 0], nomPairs[k]]] : [];
            ringK[k].push([shK, rv.pairs[k]]);
            if (!(k in res.pairs) || rv.pairs[k] < res.pairs[k]) res.pairs[k] = rv.pairs[k];
          });
        }
      }
      function refineRing(k, dd) {   // 動かした所ごとに、近い組は三角形までの距離で測り直す(箱を逆に動かす。点検 C55 の 2: 粗い判定は最大 2mm 遠く出る)
        var ab = k.split("|"), pid0 = +ab[1].slice(2), mv = movers[groups.indexOf(ab[0])], best = Infinity;
        ringK[k].forEach(function (e) {
          var dc = e[1], fd = null;
          if (dc >= 0 && dc < REFINE_D && !ui.NO_REFINE) fd = fineDist(ab[0], st.frames[4], f.jaw, ROBOT_Y, cubesShift(pid0, [-e[0][0], -e[0][1]]), mv.pw(), dc);
          best = Math.min(best, fd === null ? dc : fd);
        });
        return best;
      }
      if (TRUE_BIAS && inAW) {                                   // _TRUE_BIAS: 本当の腕の姿勢でのほかのピースまで(合否には入れない)
        var qT = f.q.map(function (v, i) { return v + (+TRUE_BIAS[i] || 0); });
        var stT = KIN.fk(SO101.axes, qT, SO101.tool), keepH = heldId ? poseNow[f.held] : null;
        if (heldAt) poseNow[f.held] = heldAt(SO101.urdfFK(qT.map(function (v) { return v * DEG; }), 0));   // 持っているピースも本当の爪から(点検 C55 の 3)
        var bt = buildMovers(null, stT), rt = COLLIDE.frame(bt.movers, bt.boxes, null, null, STATICS);
        Object.keys(rt.pairs).forEach(function (k) {
          var ab = k.split("|"), d = rt.pairs[k];
          if (!otherPiecePair(ab, true)) return;
          if (OFF_GROUPS[ab[0]] && d >= 0 && d < REFINE_D && !ui.NO_REFINE) {   // 爪のまわりは三角形までの距離で測り直す(点検 C55 の 2)
            var fdT = fineDist(ab[0], stT.frames[4], f.jaw, ROBOT_Y, cubesOf(+ab[1].slice(2)), bt.movers[groups.indexOf(ab[0])].pw(), d);
            if (fdT !== null) d = fdT;
          }
          if (d < trueClear.d) { trueClear.d = d; trueClear.pair = k; trueClear.t = f.t; }
        });
        if (keepH) poseNow[f.held] = keepH;
      }
      var ids = {}, bad = [];
      Object.keys(res.pairs).forEach(function (k) {
        var dd = res.pairs[k], ab = k.split("|");
        var touchPc0 = f.touch >= 0 ? "pc" + f.touch : "";
        if (ringK && ringK[k]) {                                 // _ARM_MARGIN_MM で動かして測った組: 動かした所ごとに測り直した、いちばん近い値
          if (OFF_GROUPS[ab[0]] && dd >= 0) dd = refineRing(k, dd);
          if (dd < ringMin.d) { ringMin.d = dd; ringMin.pair = k; ringMin.t = f.t; }
        } else
        if ((ab[0] === "j5" || ab[0] === "jaw" || ab[0] === "cam") && ab[1].indexOf("pc") === 0 && ab[1] !== touchPc0 && dd >= 0 && dd < REFINE_D && !ui.NO_REFINE && !ow) {   // 窓の中は測り直さない(点検 C54 の 6)
          // S-8: 爪のまわりとピースが近い組は、三角形までの距離で測り直す(NO_REFINE は比べる検算用)
          var mv = movers[groups.indexOf(ab[0])], pid0 = +ab[1].slice(2);
          var cbs = cubesOf(pid0);
          var fd = fineDist(ab[0], st.frames[4], f.jaw, ROBOT_Y, cbs, mv.pw(), dd);
          if (ui.REFINE_LOG) {                     // 検算用(tools/check-refine.js): 箱の点を全関節 0 の姿勢へ移す関数ごと渡す
            (function (fr4, jd, g) {
              ui.REFINE_LOG.push({ t: f.t, pair: k, coarse: dd, fine: fd, cubes: cbs, toZero: function (w) {
                var p0 = addv(fr4.o, matTVec(fr4.R, subv([w[0], w[1] - ROBOT_Y, w[2]], fr4.p)));
                return g === "jaw" ? jawRotate(p0, -jd) : p0;
              } });
            })(st.frames[4], f.jaw, ab[0]);
          }
          if (fd !== null) {
            refineN++;
            if (dd < minClearCoarse) { minClearCoarse = dd; }
            if (dd - fd > refineMaxGain) refineMaxGain = dd - fd;
            if (fd - dd > refineMaxLoss) refineMaxLoss = fd - dd;
            dd = fd;
          }
        }
        if ((ab[1].indexOf("st_") === 0 || ab[1].indexOf("c_") === 0) && dd < minStand) { minStand = dd; minStandPair = k; minStandAt = f.t; }
        if ((ab[1].indexOf("st_") === 0 || ab[1].indexOf("c_") === 0) && !f.sub && dd < minStandFrame) minStandFrame = dd;
        if (ab[0] === "cam" && dd < minCam) { minCam = dd; minCamPair = k; minCamAt = f.t; }
        var touchPc = f.touch >= 0 ? "pc" + f.touch : "";
        var contact = (ab[1] === touchPc && (ab[0] === "j5" || ab[0] === "jaw")) || (ab[0] === touchPc && ab[1] === "机")
                   || (GAP < 0.001 && ab[0].indexOf("pc") === 0 && ab[1].indexOf("pc") === 0);   // すき間 0 では隣のピースに接して置く
        if (contact) {
          if (dd < minContact) { minContact = dd; minContactPair = k; }
          if (dd < -CONTACT_TOL) { ids[ab[0]] = ids[ab[1]] = 1; bad.push(k + " " + (dd * 1000).toFixed(1) + "mm"); }
        } else {
          if (dd < minClear) { minClear = dd; minClearPair = k; minClearAt = f.t; }
          if (!f.sub && dd < minClearFrame) minClearFrame = dd;
          // コマごとのいちばん近い所(段ごとに分けて見るため。tools/camera-budget.js が、つかむ段と離す段に分ける。点検 C14 の 1)
          if (!clearAt.length || clearAt[clearAt.length - 1].t !== f.t) clearAt.push({ t: f.t, d: Infinity, pair: "" });
          var ce = clearAt[clearAt.length - 1];
          if (dd < ce.d) { ce.d = dd; ce.pair = k; }
          if (!phaseAt.length || phaseAt[phaseAt.length - 1].t !== f.t) phaseAt.push({ t: f.t, pcs: {}, tb: Infinity, tbPair: "" });
          var pe = phaseAt[phaseAt.length - 1];
          if (ab[1].indexOf("pc") === 0 && ab[1] !== touchPc && ab[1] !== heldId && (!pe.pcs[ab[1]] || dd < pe.pcs[ab[1]][0])) pe.pcs[ab[1]] = [dd, k];   // 相手のピースごと(その手のピースは後で外す)
          if (ab[1] === "机" && dd < pe.tb) { pe.tb = dd; pe.tbPair = k; }
          if (dd < 0) { ids[ab[0]] = ids[ab[1]] = 1; bad.push(k + " " + (dd * 1000).toFixed(1) + "mm"); }
          if (errRes && dd < errRes.d) { errRes.d = dd; errRes.pair = k; errRes.t = f.t; }
          if (errRes && dd < 0) { errBad = true; if (errRes.hitPairs.indexOf(k) < 0) errRes.hitPairs.push(k); }
        }
      });
      if (errBad) errRes.hits++;
      // 指先の高さ(机 = 0)= 爪の点のいちばん低い所
      ["j5", "jaw"].forEach(function (gname) {
        var m = movers[groups.indexOf(gname)];
        if (m.c[1] - m.r > minTip) return;
        m.pw().forEach(function (p) { if (p[1] < minTip) { minTip = p[1]; minTipAt = f.t; } });
      });
      var idl = Object.keys(ids).sort();
      if (idl.length) hitFrames.push({ fi: fi, t: f.t, ids: idl, bad: bad });
      if (fi === 0 && !f.sub) restCheck(0, f.t);                           // 置き場に置いたままのピース ⇄ 台(点検 C10 の 7)
    });
    restCheck(frames.length - 1, T);                             // 盤に置き終えたピース ⇄ 台
    // 動かないピースと台は 1 コマの判定で測っていないので、最初と最後のコマで 1 回ずつ測る
    function restCheck(fi, tAt) {
      if (!STATICS) return;
      pieces.forEach(function (p) {
        var pts = [];
        cubesOf(p.idx).forEach(function (b) { cubePts.forEach(function (q) { pts.push(addv(b.c, KIN.matVec(b.M, q))); }); });
        STATICS.forEach(function (s) {
          var d = Infinity;
          pts.forEach(function (q) { d = Math.min(d, s.field.query(q)); });
          if (!isFinite(d)) return;
          var k = "pc" + p.idx + "|" + s.id;
          if (d < minStand) { minStand = d; minStandPair = k; minStandAt = tAt; }
          if (d < 0) hitFrames.push({ fi: fi, t: tAt, ids: ["pc" + p.idx, s.id], bad: [k + " " + (d * 1000).toFixed(1) + "mm"] });
        });
      });
    }
    hitFrames.sort(function (a, b) { return a.fi - b.fi; });

    // 干渉の時刻表(続いているコマは 1 つの区間にまとめる)
    var hits = [];
    hitFrames.forEach(function (h) {
      var t1 = h.fi + 1 < frames.length ? frames[h.fi + 1].t : T;
      var last = hits[hits.length - 1];
      if (last && last.ids.join("|") === h.ids.join("|") && h.t <= last.t1 + 1e-9) last.t1 = Math.max(last.t1, t1);
      else hits.push({ t0: h.t, t1: t1, ids: h.ids });
    });

    // ---- 置いた位置の差(目標のマス ⇄ 腕が離した位置) ----
    var worstPlace = 0, worstYaw = 0;
    steps.forEach(function (sp) {
      var tr = pieceTracks[sp.pid];
      var last = tr[tr.length - 1];
      var want = boardXZ(sp.pl.origin);                        // 局所 (0,0) の目標
      var e = Math.hypot(last.pos[0] - want[0], last.pos[2] - want[1]);
      var ye = Math.abs(MOTION.wrap180(last.rot - sp.k * 90));
      worstPlace = Math.max(worstPlace, e); worstYaw = Math.max(worstYaw, ye);
      sp.final = last;
    });

    // ---- 部品 ----
    var parts = [];
    parts.push({ id: "table", shape: "box", size: [0.62, 0.02, 0.62], position: [0.13, -0.010, 0], color: "#c9b38f", hit: "机", collide: "机(高さ 0 の平面)" });
    target.forEach(function (c, i) {                             // 盤(目標のマスに薄い印)
      var xz = boardXZ(c);
      parts.push({ id: "cellmark" + i, shape: "box", size: [CELL + 0.002, 0.001, CELL + 0.002], position: [xz[0], 0.0005, xz[1]], color: "#ffffff", opacity: 0.7, noShadow: true, noCollide: "印" });
    });
    // 運ぶ高さで真上を向ける範囲の目安(肩の軸から R_IN〜R_OUT の輪・tools/reach-ring.js の結果から)
    parts.push({ id: "reach", shape: "tube", size: [R_OUT, R_IN, 0.001, 64], position: [PAN_X, 0.0002, 0], color: "#9fc5e8", opacity: 0.25, noShadow: true, noCollide: "目安" });
    pieces.forEach(function (p) {                                // ピース(立方体 = 判定の箱そのもの)
      var id = "pc" + p.idx, cells = PUZZLE.SHAPES[p.shape], col = COLORS[p.idx % COLORS.length];
      var slot = SLOTS[p.idx] || [0.3, 0.3];
      parts.push({ id: id, shape: "group", position: [slot[0], 0, slot[1]], rotationDeg: [0, -SLOT_ROT[p.idx], 0] });
      var has = {};
      cells.forEach(function (c, j) {
        has[c[0] + "," + c[1]] = true;
        parts.push({ id: id + "_c" + j, parent: id, shape: "box", size: [CELL, HGT, CELL], position: [c[0] * P, HGT / 2, c[1] * P], color: col, hit: id, collide: id });
      });
      if (GAP > 0) cells.forEach(function (c, j) {                 // 立方体どうしをつなぐ棒(判定に入れていない)
        [[1, 0], [0, 1]].forEach(function (d, m) {
          if (!has[(c[0] + d[0]) + "," + (c[1] + d[1])]) return;
          parts.push({ id: id + "_b" + j + "_" + m, parent: id, shape: "box",
                       size: [d[0] ? GAP + 0.002 : 0.010, 0.010, d[1] ? GAP + 0.002 : 0.010],
                       position: [(c[0] + d[0] / 2) * P, HGT / 2, (c[1] + d[1] / 2) * P], color: col, noCollide: "細い棒(指は立方体の横に入るので当たらない前提。見ていない)" });
        });
      });
    });
    parts = parts.concat(armParts(ROBOT_Y, WRIST));
    var OHC = null;                                              // 上のカメラ(机 = 0 の座標)
    if (STAND) {
      STAND_PARTS.forEach(function (s) {                         // 台(形そのものが判定の形)
        if (s.box) {
          var sb = SHIFT || [0, 0], lo = [s.box.lo[0] + sb[0], s.box.lo[1] + ROBOT_Y, s.box.lo[2] + sb[1]], hi = [s.box.hi[0] + sb[0], s.box.hi[1] + ROBOT_Y, s.box.hi[2] + sb[1]];
          parts.push({ id: s.id, shape: "box", size: [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]], position: [(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2],
                       rotationDeg: [0, 0, 0], color: "#8a8f99", hit: s.id, collide: s.id });
          return;
        }
        if (!s.seg) { parts.push({ id: s.id, shape: "mesh", mesh: s.mesh, position: standPos(s.mesh, SHIFT), rotationDeg: [0, 0, 0], hit: s.id, collide: s.id }); return; }
        var sh0 = SHIFT || [0, 0], a = [s.seg.a[0] + sh0[0], s.seg.a[1] + ROBOT_Y, s.seg.a[2] + sh0[1]], b = [s.seg.b[0] + sh0[0], s.seg.b[1] + ROBOT_Y, s.seg.b[2] + sh0[1]];
        var w = subv(b, a), L = Math.hypot(w[0], w[1], w[2]), y = [w[0] / L, w[1] / L, w[2] / L];   // three.js の円柱は y が軸
        var t0 = Math.abs(y[1]) < 0.9 ? [0, 1, 0] : [1, 0, 0], x = KIN.unit([y[1] * t0[2] - y[2] * t0[1], y[2] * t0[0] - y[0] * t0[2], y[0] * t0[1] - y[1] * t0[0]]);
        var z = [x[1] * y[2] - x[2] * y[1], x[2] * y[0] - x[0] * y[2], x[0] * y[1] - x[1] * y[0]];
        parts.push({ id: s.id, shape: "cylinder", size: [s.seg.r, s.seg.r, L, CYL_N], position: [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2],
                     rotationDeg: SO101.eulerXYZ([x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2]]), color: "#8a8f99", hit: s.id, collide: s.id });
      });
      var oc = SO101.overheadCamera(), sh = SHIFT || [0, 0];
      OHC = { face: [oc.face[0] + sh[0], oc.face[1] + ROBOT_Y, oc.face[2] + sh[1]], tip: [oc.tip[0] + sh[0], oc.tip[1] + ROBOT_Y, oc.tip[2] + sh[1]], dir: oc.dir, up: oc.up };
      var cz = KIN.unit(OHC.dir), cy = KIN.unit(OHC.up), cx = [cy[1] * cz[2] - cy[2] * cz[1], cy[2] * cz[0] - cy[0] * cz[2], cy[0] * cz[1] - cy[1] * cz[0]];
      parts.push({ id: "st_camera", shape: "mesh", mesh: "wrist_camera_so101_v1", position: OHC.face,      // 32×32 のカメラ(公式の形。z = レンズの向き)
                   rotationDeg: SO101.eulerXYZ([cx[0], cy[0], cz[0], cx[1], cy[1], cz[1], cx[2], cy[2], cz[2]]), noCollide: "高い所にあるので測らない" });
    }

    // ---- アニメーション(全部 frames から) ----
    var anims = [];
    for (var j = 0; j < 5; j++) {
      anims.push({ target: "j" + (j + 1), type: "keys", duration: T,
                   keys: frames.map(function (f) { return { t: f.t, angle: f.q[j] }; }) });
    }
    anims.push({ target: "jaw", type: "keys", duration: T, keys: frames.map(function (f) { return { t: f.t, angle: f.jaw }; }) });
    pieces.forEach(function (p) {
      var tr = pieceTracks[p.idx], slot = SLOTS[p.idx] || [0.3, 0.3];
      var start = { t: 0, pos: [slot[0], 0, slot[1]], rot: [0, -SLOT_ROT[p.idx], 0] };
      var keys = [start];
      if (tr.length) {
        keys.push({ t: Math.max(0, tr[0].t - DT), pos: start.pos, rot: start.rot });
        tr.forEach(function (k) { keys.push({ t: k.t, pos: k.pos, rot: [0, -k.rot, 0] }); });
        var lst = tr[tr.length - 1];
        keys.push({ t: T, pos: lst.pos, rot: [0, -lst.rot, 0] });
      } else keys.push({ t: T, pos: start.pos, rot: start.rot });
      anims.push({ target: "pc" + p.idx, type: "keys", duration: T, keys: keys });
    });

    // ---- 腕どうしの当たり(自分との干渉。S-10)。作り方はちくわ排除機構の丸め箱(self-collision.js)。コマとコマの間も下限で見る ----
    //   SELF_COLLIDE が読まれていない(node の古い道具)か、ui.SELF = 0 のときは見ない(画面の表に「見ていない」と出す)
    var selfFrames = errWin ? frames.filter(function (f) { return f.t >= errWin[0] - 1e-9 && f.t <= errWin[1] + 1e-9; }) : frames;   // _ERR はその段だけ
    var SELF = (typeof SELF_COLLIDE !== "undefined" && ui.SELF !== 0 && selfFrames.length > 1) ? SELF_COLLIDE.motionCheck(selfFrames) : null;
    var selfOk = !SELF || (SELF.min >= 0 && SELF.worstBound >= 0);   // 割り切れずに下限が 0 未満の区間も、当たりうるとして NG

    // ---- 画面の表 ----
    function pairName(k) { return k.split("|").map(function (x) { return LINK_NAME[x] || ("ピース " + pieces[+x.slice(2)].name); }).join(" ⇄ "); }
    var hitOk = hitFrames.length === 0;
    var ok = solved && hitOk && selfOk;
    info.unshift({ label: "判定", big: true, state: ok ? "ok" : "ng",
                   value: !solved ? "解けない: " + why
                        : !selfOk ? (SELF.min < 0 ? "動きは作れたが、腕どうしが当たる(" + pairName(SELF.pair) + " " + (SELF.min * 1000).toFixed(1) + "mm・" + SELF.t.toFixed(1) + " 秒)" : "動きは作れたが、コマの間で腕どうしが当たらないと言い切れない(下限 " + (SELF.worstBound * 1000).toFixed(1) + "mm・" + SELF.boundAt.toFixed(1) + " 秒)")
                        : hitOk ? "解けた(" + steps.length + " ピースを置いた・当たりなし)"
                        : "動きは作れたが当たる(" + hitFrames.length + " 回。コマの間を含む。最初 " + hitFrames[0].t.toFixed(1) + " 秒: " + hitFrames[0].bad.map(function (b) { var s2 = b.split(" "); return pairName(s2[0]) + " " + s2[1]; }).join("、") + ")" });
    if (solved) {
      info.push({ label: "置く順番", value: steps.map(function (s) { return s.name + (s.k ? "(" + s.k * 90 + "°回す)" : ""); }).join(" → ") });
      info.push({ label: "置いた位置の差(最大)", value: (worstPlace * 1000).toFixed(2) + "mm・向き " + worstYaw.toFixed(2) + "°(URDF の行列の積で測った)",
                  state: worstPlace < 0.001 && worstYaw < 1 ? "ok" : "ng" });
    }
    if (frames.length > 1) {
      info.push({ label: "いちばん近づいた所(当たり判定)", state: minClear >= 0 ? "ok" : "ng",
                  value: (minClear * 1000).toFixed(1) + "mm・" + (minClearPair ? pairName(minClearPair) : "—") + "・" + minClearAt.toFixed(1) + " 秒(接する組は除く)" +
                         (minClear >= 0 && minClear < MARGIN_WARN ? "。★要確認: " + (MARGIN_WARN * 1000) + "mm 未満は、点と点の間(最大 2mm ほど甘い)とコマの割り方(約 0.6mm 浅い)より小さい余裕" : "") });
      if (minContactPair) info.push({ label: "接する組(つかんでいるピースと爪" + (GAP < 0.001 ? "・隣どうしのピース" : "") + ")", state: minContact >= -CONTACT_TOL ? "ok" : "ng",
                  value: "いちばん深い重なり " + (minContact * 1000).toFixed(1) + "mm・" + pairName(minContactPair) + "(" + (CONTACT_TOL * 1000) + "mm までは点の間隔・置く位置の差ぶんとして許す)" });
    }
    info.push({ label: "指先の最低の高さ", value: (minTip * 1000).toFixed(1) + "mm(机 = 0・" + minTipAt.toFixed(1) + "秒)", state: minTip > 0 ? "ok" : "ng" });
    info.push({ label: "爪の寸法(形から測った)", value: "固定の爪の面 " + (GRIP.face * 1000).toFixed(1) + "mm・指先 " + (GRIP.tip * 1000).toFixed(1) + "mm・指の厚さ " + (FINGER_T * 1000).toFixed(1) +
                "mm・挟める幅 " + (MAX_SPAN * 1000).toFixed(0) + "mm まで(道具の点から)" });
    if (symUsed.length) info.push({ label: "180° 回した見方", value: symUsed.map(function (x) { return x.name; }).join("・") + " は、置き場を 180° 回った形と見直してつかんだ(180° 回して同じ形のピース。机の上の立方体は同じ所。下の置き場の値は見直した後)" });
    if (ui.LAYOUT) info.push({ label: "置き場", value: "ファイルから読んだ(" + pieces.map(function (p) { return p.name + " " + (SLOTS[p.idx][0] * 1000).toFixed(0) + "," + (SLOTS[p.idx][1] * 1000).toFixed(0) + "mm・" + SLOT_ROT[p.idx].toFixed(0) + "°"; }).join(" / ") + ")" });
    (lay.notes || []).forEach(function (m) { info.push({ label: "置き場の注意", value: m }); });
    problems.forEach(function (m) { info.push({ label: "注意", value: m, state: "ng" }); });
    info.push({ label: "所要時間", value: T.toFixed(1) + " 秒(【仮】の速さ)" });
    info.push({ label: "上のカメラ(置き方 C)", state: !STAND ? "" : (minStand >= 0 ? "ok" : "ng"),
                value: (STAND ? "あり(腕の原点は合板から " + (ROBOT_Y * 1000).toFixed(1) + "mm 上 = 土台の底のリブ。本家の板なし)。いちばん近づいた所: " +
                        (minStandPair ? (minStand * 1000).toFixed(1) + "mm・" + pairName(minStandPair) + "・" + minStandAt.toFixed(1) + " 秒" : "15mm より近づかない") + "。自分で選ぶ置き場は C の柱の足跡から " + (SLOT_CLEAR * 1000) + "mm 空ける(ファイル・絵から読んだ置き場は空いていなくても書き置くだけで、当たりは判定で見る)。置き方は【仮】"
                        : "なし(腕の原点は机から " + (ROBOT_Y * 1000).toFixed(1) + "mm 上 = 土台の底のリブ)") });
    info.push({ label: "手首のカメラ", state: !WRIST ? "" : (minCam >= 0 ? "ok" : "ng"),
                value: WRIST ? "あり(公式の so101_new_calib_camera.urdf の位置)。いちばん近づいた所: " + (minCamPair ? (minCam * 1000).toFixed(1) + "mm・" + pairName(minCamPair) + "・" + minCamAt.toFixed(1) + " 秒" : "近くに相手が無い")
                           : "なし(当たり判定に入れていない)" });
    info.push({ label: "腕どうし(自分との干渉)", state: !SELF ? "" : SELF.min < 0 || SELF.worstBound < 0 ? "ng" : SELF.min < SELF_COLLIDE.WARN ? "warn" : "ok",
                value: !SELF ? "見ていない(self-collision.js が読まれていない・または外した)"
                     : "いちばん近い " + (SELF.min * 1000).toFixed(1) + "mm・" + pairName(SELF.pair) + "・" + SELF.t.toFixed(1) + " 秒(" + SELF.nPose + " 姿勢を測った・コマの間は下限で見た" +
                       (SELF.worstBound < 0 ? "。★割り切れずに下限が " + (SELF.worstBound * 1000).toFixed(1) + "mm の所がある(" + SELF.boundAt.toFixed(1) + " 秒)" : "") +
                       ")。形はちくわ排除機構の丸め箱(実物より太い = 安全側)。関節でつながる組と、1 つの関節で距離が決まる組(爪とカメラ台・手首とカメラ台)は除いた(self-collision.js の ADJ)" });
    info.push({ label: "見ていないもの", value: "手首のカメラのケーブル・ピースをつなぐ細い棒・点と点の間(最大 2mm ほど甘く出る)・物理(滑る・倒れる)・実機の角度の符号(前提 U-2)" });

    var data = {
      title: "SO-101 ブロックパズル(シミュレーション)",
      description: "SO-101 が机の上のピースを真上からつかみ、回して盤に並べ、目標の形を埋めます。\n腕の形と寸法は公式の STL と URDF(so101_new_calib)。【仮】の値は scene-data.js の冒頭。\n「当たり判定: 重ねる」で判定に使っている形(腕の STL・ピースの立方体)を出し、「干渉点のチェック」で当たっている所を赤くします。",
      environment: { backgroundColor: "#e8edf2", showGrid: false, groundColor: "#d5dbe1", shadowExtent: 0.6 },
      parts: parts,
      animations: anims,
      hits: hits,
      cameraShots: [
        { name: "斜め前から", type: "fixed", position: [0.62, 0.46, 0.52], target: [0.12, 0.05, 0] },
        { name: "真上から", type: "fixed", position: [0.13, 0.85, 0.001], target: [0.13, 0, 0] },
        { name: "横から", type: "fixed", position: [0.15, 0.20, 0.75], target: [0.15, 0.12, 0] },
        { name: "ぐるり", type: "orbit", radius: 0.75, height: 0.42, speed: 10, target: [0.12, 0.05, 0] },
        // カメラの位置からの画角(縦は SO101.CAM_FOV【仮】。絵の縦横の比は画面の窓に合わせる。index.html の "view"・"track")
        OHC ? { name: "上のカメラから(画角 " + SO101.CAM_FOV.top + "°)", type: "view", fov: SO101.CAM_FOV.top, position: OHC.tip, up: OHC.up,
                target: [OHC.tip[0] + OHC.dir[0] * 0.3, OHC.tip[1] + OHC.dir[1] * 0.3, OHC.tip[2] + OHC.dir[2] * 0.3] } : null,
        WRIST ? { name: "手首のカメラから(画角 " + SO101.CAM_FOV.wrist + "°)", type: "track", fov: SO101.CAM_FOV.wrist, hidePrefix: "vis_cam_",   // カメラ自身の形は隠す
                  keys: frames.filter(function (f, i) { return i % 2 === 0 || i === frames.length - 1; }).map(function (f) {
                    var cf = SO101.urdfFK(f.q.map(function (x) { return x * DEG; }), f.jaw * DEG).wrist_camera_link;
                    var p = [cf.p[0], cf.p[1] + ROBOT_Y, cf.p[2]], dir = [cf.R[2], cf.R[5], cf.R[8]];   // 光軸 = z・絵の下 = y【仮】(tools/camera-budget.js と同じ)
                    return { t: f.t, p: p, tg: [p[0] + dir[0] * 0.1, p[1] + dir[1] * 0.1, p[2] + dir[2] * 0.1], up: [-cf.R[1], -cf.R[4], -cf.R[7]] };
                  }) } : null,
        (function () {        // 1 つ目のピースをつかむ所を近くから(指とピースの当たり方を見る)
          var c = steps.length ? steps[0].cS : [0.17, 0, 0];
          return { name: "1 つ目をつかむ所(近く)", type: "fixed", position: [c[0] + 0.13, 0.085, c[2] + 0.11], target: [c[0], 0.012, c[2]] };
        })()
      ],   // 上のカメラの位置からの視点は、上の "上のカメラから(画角 …°)"(向きと画角も合わせる。前の「画角は画面のまま」を置き換えた)
      legend: pieces.map(function (p) { return { color: COLORS[p.idx % COLORS.length], label: "ピース " + p.name }; })
              .concat([{ color: "#58a6ff", label: "当たり判定の形(出したとき)" }, { color: "#ff1e1e", label: "当たっている所(干渉点のチェック)" },
                       { color: "#9fc5e8", label: "運ぶ高さで届く目安(肩から " + (R_IN * 1000) + "〜" + (R_OUT * 1000) + "mm)" }]),
      info: info,
      // 上のカメラ(机 = 0 の座標。台なしは null)。index.html の ?_cam=top と tools/overhead-truth.js が使う(計画書_完成形まで の S-9)
      overheadCamera: OHC,
      // 検算の道具(tools/check-scene.js)が読む
      _check: { solved: solved, why: why, worstPlace: worstPlace, worstYaw: worstYaw, minTip: minTip, T: T,
                wrist: WRIST, minCam: minCam, minCamPair: minCamPair,
                // 上のカメラの検算(tools/overhead-occlusion.js)が読む: 待ちの姿勢・ピースのマスと盤のマスの真ん中(机 = 0 の座標)
                homeQ: home.q, homeJaw: frames[0].jaw, lastJaw: frames[frames.length - 1].jaw, photoQ: STAND ? PQ : null, cell: CELL, height: HGT,
                verdict: solved ? (ok ? "ok" : "未解決") : failKind === "拒否" && dimRule ? "拒否(目安)" : (failKind || "未解決"),   // S1: ok / 拒否(寸法と入力で決まる) / 拒否(目安)(決めた定数の線) / 未解決(探しきれない・当たりが残る)
                verdictRule: dimRule,
                minClearAt: minClearAt,   // S1: いちばん近い所の時刻(段の表と並べる)
                dims: { cell: CELL, height: HGT, gap: GAP, pitch: P, fingerT: FINGER_T, fitGap: FIT_GAP, maxSpan: MAX_SPAN, fitMargin: FIT_MARGIN, maxRun: maxRun },
                phaseClear: phaseClearOf(steps, phaseAt, tReady),

                worstPick: worstPick, worstPickYaw: worstPickYaw, marginWarn: MARGIN_WARN,
                armMargin: ARM_M > 0 ? { m: ARM_M, d: ringMin.d, pair: ringMin.pair, t: ringMin.t } : null,   // _ARM_MARGIN_MM: 動かして測った組のいちばん近い所
                trueClear: TRUE_BIAS ? trueClear : null,                                                       // _TRUE_BIAS(sim): 本当の腕でのいちばん近い所
                minClearFrame: minClearFrame, symUsed: symUsed, slotsIn: SLOTS_IN, slotRotIn: ROT_IN, refineN: refineN, minClearCoarse: minClearCoarse, refineMaxGain: refineMaxGain, refineMaxLoss: refineMaxLoss,
                pieceCells: pieces.map(function (p) { var sl = SLOTS[p.idx]; return PUZZLE.SHAPES[p.shape].map(function (c) { var o = rotY([c[0] * P, 0, c[1] * P], SLOT_ROT[p.idx]); return [sl[0] + o[0], sl[1] + o[2]]; }); }),
                slotRot: SLOT_ROT, layoutNotes: lay.notes || [], layoutWhy: layoutWhy,
                boardCells: target.map(function (c) { return boardXZ(c); }),
                self: SELF ? { min: SELF.min, pair: SELF.pair, t: SELF.t, q: SELF.q, worstBound: SELF.worstBound, boundAt: SELF.boundAt, boundPair: SELF.boundPair, nPose: SELF.nPose } : null,
                ok: ok,
                err: errRes ? { win: errRes.win, d: errRes.d, pair: errRes.pair, t: errRes.t, hits: errRes.hits, hitPairs: errRes.hitPairs,
                                yp: errStep.yp, view: errStep.view } : null,   // _ERR の段のいちばん近い所と当たり(どちらも、ふつうの組だけ。触れてよい組は見ない)           // 判定の表と同じ「解けた・当たりなし・腕どうしが 0 以上」。solved は動きが組めたかだけで、当たりは入らない(点検 C29 の 1。動かす側はこちらで止める)
                tReady: tReady,   // 写す姿勢から待ちの姿勢に着いた時刻(台なしは 0。置き場の選び直しで、1 つ目のピースの 1 手の始め)
                stand: STAND, robotY: ROBOT_Y, minStand: minStand, minStandPair: minStandPair, minStandAt: minStandAt, minStandFrame: minStandFrame, slots: SLOTS, foot: FOOT,
                order: steps.map(function (s) { return s.name; }),
                // 見て動かす(tools/vision-loop.js・vision.js)が読む: ピースの形と色・マスの間隔・つかみ方(つかむ場所の真ん中 cS・爪の向き yp・局所のつかむ範囲)
                pieces: pieces.map(function (p) { return { idx: p.idx, name: p.name, shape: p.shape, color: COLORS[p.idx % COLORS.length] }; }), pitch: P, gap: GAP,
                // 物理で流す(tools/export-physics.js・physics/run_physics.py)が読む: ピースの始めと終わりの置き方(pos = 局所のマス (0,0) の底の真ん中・机 = 0 の画面の座標。rot = 上から見て X → Z の向き[度])
                pieceStart: pieces.map(function (p) { var sl = SLOTS[p.idx]; return { pos: [sl[0], 0, sl[1]], rot: SLOT_ROT[p.idx] }; }),
                pieceEnd: pieces.map(function (p) { var tr = pieceTracks[p.idx], l = tr[tr.length - 1]; return l ? { pos: l.pos, rot: l.rot } : null; }),
                grasps: steps.map(function (s) { return { pid: s.pid, name: s.name, gl: s.gl, run: s.run, yp: s.yp, cS: s.cS, cT: s.cT, k: s.k,
                                                         target: boardXZ(s.pl.origin).concat([s.k * 90]) }; }),   // 盤の目標(形の [0,0] のマス x・z[m]・向き[度])。_PLACE_OFF の前
                stepTimes: steps.map(function (s) { return { pid: s.pid, name: s.name, tHover: s.tHover, tPick: s.tPick, tPlace: s.tPlace, tDone: s.tDone, tCheck0: s.tCheck0, tCheck1: s.tCheck1, tLook0: s.tLook0, tLook1: s.tLook1, tBack: s.tBack }; }),
                relook: RELOOK ? { detour: relookAdded, n: steps.filter(function (x) { return x.tLook1 > 0; }).length, shot: RELOOK_SHOT } : null,   // RELOOK の寄り道: detour = 寄り道に使った時間の合計(置いた所 → 写す姿勢 → 止まり終わるまで)。
                //   全体の時間 T の伸びとは一致しない(寄り道しないときの 置いた所 → 次のピースの上 の動きが無くなり、写す姿勢から次のピースの上へ行く動きが足される。
                //   2026-11-18: 3×4・すき間 15mm で T 23.3 → 26.2 秒・detour 3.0 秒、4×5・I O L J S で 37.2 → 42.6 秒・4.8 秒)。T の伸びは、寄り道なしで組んだ T と比べる。tLook0〜tLook1 = 写す姿勢で止まる間(0 は寄り道なし)
                wristCheck: WRIST_CHECK ? { settle: WRIST_SETTLE, shot: WRIST_SHOT } : null, frames: frames.length, subChecks: subChecks, photoVia: STAND ? photoVia : null, via: STAND ? VIA : null,
                // 関節の並び(コマごと。tools/export-motion.js と、コマの間の検算が読む)
                motion: frames.map(function (f) { return { t: f.t, q: f.q, jaw: f.jaw, held: f.held }; }),
                clearAt: clearAt, hitFrames: hitFrames.length, firstHit: hitFrames[0] || null, minClear: minClear, minClearPair: minClearPair,
                minContact: minContact, minContactPair: minContactPair, grip: { face: GRIP.face, tip: GRIP.tip, thickFixed: GRIP.thickFixed,
                thickMove: GRIP.thickMove, gap0: GRIP.gap0, gapMax: GRIP.gapMax, sign: GRIP.sign, fitGap: FIT_GAP, maxSpan: MAX_SPAN, fitMargin: FIT_MARGIN } }
    };
    return data;
  }

  // 関節角 q(度)・爪の角度 jawDeg・腕の原点の高さ ry で、j5・動く爪(と、withCam なら手首のカメラと台)の三角形を、机 = 0 の座標で返す。
  //   [{ group, mesh, tri: Float64Array(9 個で 1 つ) }]。手首のカメラの視線が爪に遮られるか(tools/camera-budget.js)に使う
  function gripTrisWorld(q, jawDeg, ry, withCam) {
    var st = KIN.fk(SO101.axes, q, SO101.tool), fr = st.frames[4], out = [];
    if (typeof SO101_TRIS === "undefined") return out;
    Object.keys(SO101_VISUALS).forEach(function (link) {
      var g = SO101.LINK_GROUP[link] || (SO101.CAMERA_GROUP[link] ? "cam" : null);
      if (g !== "j5" && g !== "jaw" && !(withCam && g === "cam")) return;
      SO101_VISUALS[link].forEach(function (v) {
        var T = SO101_TRIS[v.mesh];
        if (!T) return;
        var Pv = SO101.visualPose(link, v.xyz, v.rpy), tri = new Float64Array(T.length);
        for (var i = 0; i < T.length; i += 3) {
          var p0 = addv(SO101.mv(Pv.R, [T[i] * 1e-4, T[i + 1] * 1e-4, T[i + 2] * 1e-4]), Pv.p);
          if (g === "jaw") p0 = jawRotate(p0, jawDeg || 0);
          var w = addv(fr.p, KIN.matVec(fr.R, subv(p0, fr.o)));
          tri[i] = w[0]; tri[i + 1] = w[1] + ry; tri[i + 2] = w[2];
        }
        out.push({ group: g, mesh: v.mesh, tri: tri });
      });
    });
    return out;
  }

  // 検算用(tools/check-scene.js): 関節角 q(度)・爪の角度で、腕(土台を除く)と台のいちばん近い所。build の当たり判定と同じ点・同じ置き方
  //   shiftMm … 台を机の上で [x, z] mm ずらす(build の STAND_SHIFT と同じ)
  function standClearance(q, jawDeg, shiftMm) {
    var ry = robotY(true), st = KIN.fk(SO101.axes, q, SO101.tool);
    var movers = ["j1", "j2", "j3", "j4", "j5", "jaw", "cam"].map(function (gname) {
      var fr = st.frames[(gname === "jaw" || gname === "cam") ? 4 : (+gname.slice(1) - 1)];
      function place(p0) {
        var p = gname === "jaw" ? jawRotate(p0, jawDeg || 0) : p0, w = addv(fr.p, KIN.matVec(fr.R, subv(p, fr.o)));
        return [w[0], w[1] + ry, w[2]];
      }
      var sp = ARM.spheres[gname];
      return { id: gname, group: gname, c: place(sp.c), r: sp.r + (gname === "jaw" ? 0.01 : 0), ignoreTable: true,
               pw: function () { return ARM.points[gname].map(place); } };
    });
    return COLLIDE.frame(movers, [], null, null, standFields(shiftMm ? [shiftMm[0] / 1000, shiftMm[1] / 1000] : null));
  }

  return { controls: controls, defaultUiValues: defaultUiValues, buildFromUi: build, gripFor: gripFor, GRASP_Y: GRASP_Y, gripFields: gripFields, fineDist: fineDist, gripTrisWorld: gripTrisWorld,
           CELL_DEFAULT: CELL_DEFAULT, ARM: ARM, standClearance: standClearance, robotY: robotY, jawRotate: jawRotate,
           standPos: standPos, STAND_PARTS: STAND_PARTS, standTris: standTris };
})();

// 初期値の場面。画面(index.html・live.html)は先に window.SCENE_SKIP_DEFAULT を立てて、ここでは組まない
//   (読み込みの途中で 20 秒ほど止まり、白い画面のまま待たされるため。画面は「組み立て中」を出してから自分で組む。2026-11-17)。
//   node の道具は window が無いので、これまでどおりここで組む
var SCENE_DATA = (typeof window !== "undefined" && window.SCENE_SKIP_DEFAULT) ? null : SCENE_MODEL.buildFromUi(SCENE_MODEL.defaultUiValues());
