/* =============================================================================
 * puzzle.js — ブロックを並べて形にするパズル(ポリオミノの敷き詰め)
 *
 * ■ パズルの読み方(計画書の決めたこと 3)
 *   立方体をつないだ板状のピースを、机の上に 1 段で並べて目標の形を埋める。
 *   ★裏返さない(真上からつかむだけでは裏返せない)。回すのは 90° おきの 4 通り。
 *
 * ■ マスの座標
 *   盤のマス (a, b): a = 画面の X(前)へ、b = 画面の Z へ。1 マス = CELL[m]。
 *   ピースを ψ 回す(上から見て X から Z へ向かう向きが正)と、
 *   局所のマス (a, b) は (a cosψ − b sinψ, a sinψ + b cosψ) へ動く。
 *   90° なら (a, b) → (−b, a)。
 *
 * ■ 爪が隣に当たる、の決まり(【仮】爪の指 1 本 = 1 マスの幅・厚さ)
 *   真上から下ろす爪は、つかむマスの両隣(挟む向き)の 2 マスに入る。
 *   ★そのマスに置き済みのピースがあれば当たる。だから
 *     「つかむマスの選び方」と「置く順番」を一緒に探す。
 *   見つからなければ「解けない(爪が隣に当たる)」と返す(黙って緩めない)。
 * ========================================================================== */

var PUZZLE = (function () {
  "use strict";

  // ピースの形(局所のマス)。名前は形の通称
  var SHAPES = {
    I: [[0, 0], [1, 0], [2, 0], [3, 0]],
    O: [[0, 0], [1, 0], [0, 1], [1, 1]],
    T: [[0, 0], [1, 0], [2, 0], [1, 1]],
    L: [[0, 0], [1, 0], [2, 0], [2, 1]],
    J: [[0, 0], [1, 0], [2, 0], [0, 1]],
    S: [[0, 0], [1, 0], [1, 1], [2, 1]],
    Z: [[0, 1], [1, 1], [1, 0], [2, 0]],
    I3: [[0, 0], [1, 0], [2, 0]],
    V3: [[0, 0], [1, 0], [0, 1]],
    D2: [[0, 0], [1, 0]],
    // 5 マスのピース(ペントミノ 12 種。名前は通称 + 5。裏返さないので、鏡に映した形は別のピースとして足す必要がある)
    F5: [[1, 0], [2, 0], [0, 1], [1, 1], [1, 2]],
    I5: [[0, 0], [1, 0], [2, 0], [3, 0], [4, 0]],
    L5: [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]],
    N5: [[0, 0], [1, 0], [1, 1], [2, 1], [3, 1]],
    P5: [[0, 0], [1, 0], [0, 1], [1, 1], [2, 0]],
    T5: [[0, 0], [0, 1], [0, 2], [1, 1], [2, 1]],
    U5: [[0, 0], [0, 1], [1, 1], [2, 1], [2, 0]],
    V5: [[0, 0], [1, 0], [2, 0], [2, 1], [2, 2]],
    W5: [[0, 0], [1, 0], [1, 1], [2, 1], [2, 2]],
    X5: [[1, 0], [0, 1], [1, 1], [2, 1], [1, 2]],
    Y5: [[0, 0], [1, 0], [2, 0], [3, 0], [1, 1]],
    Z5: [[0, 0], [0, 1], [1, 1], [2, 1], [2, 2]]
  };

  /** 文字の絵から目標のマスを作る。"#" か "■" が埋めるマス。行 = a(上が前)、列 = b */
  function fromArt(lines) {
    var t = [];
    lines.forEach(function (row, a) { Array.from(row).forEach(function (ch, b) { if (ch === "#" || ch === "■") t.push([a, b]); }); });
    return t;
  }

  function rot90(c) { return [-c[1], c[0]]; }
  function rotK(c, k) { for (var i = 0; i < ((k % 4) + 4) % 4; i++) c = rot90(c); return c; }
  function key(a, b) { return a + "," + b; }

  /** 回した形を、いちばん小さいマスが (0,0) に来るようにずらす。shift も返す */
  function orient(cells, k) {
    var r = cells.map(function (c) { return rotK(c, k); });
    var ma = Infinity, mb = Infinity;
    r.forEach(function (c) { if (c[0] < ma) ma = c[0]; if (c[1] < mb) mb = c[1]; });
    return { cells: r.map(function (c) { return [c[0] - ma, c[1] - mb]; }), shift: [-ma, -mb] };
  }

  /**
   * 敷き詰めを探す(バックトラック。まだ埋まっていないマスのうち最初の 1 つを、
   * どのピースのどの向きで埋めるかを全部試す)。
   *   target … 盤の目標のマス [[a,b],...]
   *   pieces … [{name, shape}] 使うピースの一覧(全部使う)
   *   max … 何通りまで集めるか
   * 戻り値 [[{piece, k, cells(盤のマス), offset}], ...]
   */
  function solve(target, pieces, max) {
    max = max || 50;
    var need = {}; target.forEach(function (c) { need[key(c[0], c[1])] = true; });
    var order = target.slice().sort(function (p, q) { return p[0] - q[0] || p[1] - q[1]; });
    var total = pieces.reduce(function (s, p) { return s + SHAPES[p.shape].length; }, 0);
    if (total !== target.length) return { solutions: [], reason: "ピースのマスの合計(" + total + ")と目標のマス(" + target.length + ")が違う" };
    var filled = {}, used = pieces.map(function () { return false; }), stack = [], sols = [];
    // 同じ形のピースが 2 つ以上あると同じ並べ方が重なるので、同じ形は番号の若い順にだけ使う
    function firstEmpty() {
      for (var i = 0; i < order.length; i++) if (!filled[key(order[i][0], order[i][1])]) return order[i];
      return null;
    }
    function rec() {
      if (sols.length >= max) return;
      var e = firstEmpty();
      if (!e) { sols.push(stack.map(function (s) { return Object.assign({}, s); })); return; }
      for (var p = 0; p < pieces.length; p++) {
        if (used[p]) continue;
        var dup = false;
        for (var q = 0; q < p; q++) if (!used[q] && pieces[q].shape === pieces[p].shape) dup = true;
        if (dup) continue;
        var seen = {};
        for (var k = 0; k < 4; k++) {
          var o = orient(SHAPES[pieces[p].shape], k);
          var sig = o.cells.map(function (c) { return key(c[0], c[1]); }).sort().join(";");
          if (seen[sig]) continue;            // 対称な形は同じ向きを 2 度試さない
          seen[sig] = true;
          // 最初の空きマス e を、形の「いちばん小さいマス」で埋める(探索の決まりで必ずそうなる)
          var first = o.cells.slice().sort(function (u, v) { return u[0] - v[0] || u[1] - v[1]; })[0];
          var off = [e[0] - first[0], e[1] - first[1]];
          var cs = o.cells.map(function (c) { return [c[0] + off[0], c[1] + off[1]]; });
          var okc = cs.every(function (c) { var kk = key(c[0], c[1]); return need[kk] && !filled[kk]; });
          if (!okc) continue;
          cs.forEach(function (c) { filled[key(c[0], c[1])] = true; });
          used[p] = true;
          stack.push({ piece: p, name: pieces[p].name, shape: pieces[p].shape, k: k, cells: cs,
                       origin: [off[0] + o.shift[0], off[1] + o.shift[1]] });
          rec();
          stack.pop(); used[p] = false;
          cs.forEach(function (c) { delete filled[key(c[0], c[1])]; });
        }
      }
    }
    rec();
    return { solutions: sols, reason: sols.length ? "" : "敷き詰め方が無い" };
  }

  /**
   * 敷き詰めを探す(速い版。厳密被覆の Algorithm X):
   *   列 = 目標のマス + ピース 1 つずつ。行 = 「このピースをこの向き・この位置に置く」。
   *   毎回「置き方の候補がいちばん少ないマス(またはピース)」から決める(候補 0 ならすぐ戻る)。
   *   さらに、空きマスが島に分かれたとき、島のマスの数が残りのピースの大きさの和で作れないなら戻る。
   *   戻り値は solve と同じ形。見つける順は solve と違う。同じ形のピースが 2 つ以上あるときは、番号の入れ替えだけ違う並べ方を 1 つにまとめる(solve と同じ集まり。どのピースがどこに行くかの番号は違いうる)。
   *   opts.timeLimit … 打ち切りの秒数(既定 20)。打ち切ったら reason に書く
   */
  function solveX(target, pieces, max, opts) {
    max = max || 50; opts = opts || {};
    var t0 = Date.now(), limit = (opts.timeLimit || 20) * 1000, timedOut = false, nodes = 0;
    var total = pieces.reduce(function (s, p) { return s + SHAPES[p.shape].length; }, 0);
    if (total !== target.length) return { solutions: [], reason: "ピースのマスの合計(" + total + ")と目標のマス(" + target.length + ")が違う" };
    var cellIdx = {}; target.forEach(function (c, i) { cellIdx[key(c[0], c[1])] = i; });
    var NC = target.length, NP = pieces.length;
    // 置き方の行
    var rows = [];
    pieces.forEach(function (pc, p) {
      var seen = {};
      for (var k = 0; k < 4; k++) {
        var o = orient(SHAPES[pc.shape], k);
        var sig = o.cells.map(function (c) { return key(c[0], c[1]); }).sort().join(";");
        if (seen[sig]) continue; seen[sig] = true;
        target.forEach(function (e) {
          var first = o.cells.slice().sort(function (u, v) { return u[0] - v[0] || u[1] - v[1]; })[0];
          var off = [e[0] - first[0], e[1] - first[1]];
          var cs = o.cells.map(function (c) { return [c[0] + off[0], c[1] + off[1]]; });
          var ids = [];
          for (var i = 0; i < cs.length; i++) { var id = cellIdx[key(cs[i][0], cs[i][1])]; if (id === undefined) return; ids.push(id); }
          rows.push({ p: p, k: k, cells: cs, ids: ids, anchor: Math.min.apply(null, ids), origin: [off[0] + o.shift[0], off[1] + o.shift[1]] });
        });
      }
    });
    // 列ごとの行
    var colRows = []; for (var c = 0; c < NC + NP; c++) colRows.push([]);
    rows.forEach(function (r, ri) { r.ids.forEach(function (id) { colRows[id].push(ri); }); colRows[NC + r.p].push(ri); });
    var colDone = new Uint8Array(NC + NP), rowDead = new Int32Array(rows.length), sizes = [];
    var stack = [], sols = [];
    var sizeOfPiece = pieces.map(function (p) { return SHAPES[p.shape].length; });
    // 同じ形のピース: 同じ並べ方が番号の入れ替えで重ならないように、番号の若いピースほど「いちばん小さいマスの番号」が小さい所に置く
    //   (探す順に依らない決まり。重ならない 2 つの置き方は、いちばん小さいマスが必ず違う)
    var same = pieces.map(function (p, i) { var l = []; pieces.forEach(function (q, j) { if (j !== i && q.shape === p.shape) l.push(j); }); return l; });
    var placedAnchor = pieces.map(function () { return -1; });
    function rowCells(ri) { var r = rows[ri]; return r.ids.concat([NC + r.p]); }
    function cover(ri) {   // 行 ri を選ぶ: その列を埋め、ぶつかる行を消す
      var killed = [];
      rowCells(ri).forEach(function (col) {
        colDone[col] = 1;
        colRows[col].forEach(function (rj) { if (rowDead[rj]++ === 0) killed.push(rj); });
      });
      return killed;
    }
    function uncover(ri) { rowCells(ri).forEach(function (col) { colDone[col] = 0; colRows[col].forEach(function (rj) { rowDead[rj]--; }); }); }
    function live(col) { var n = 0, l = colRows[col]; for (var i = 0; i < l.length; i++) if (!rowDead[l[i]] && pieceAllowed(l[i])) n++; return n; }
    function pieceAllowed(ri) {
      var r = rows[ri], l = same[r.p];
      for (var i = 0; i < l.length; i++) {
        var a = placedAnchor[l[i]];
        if (a < 0) continue;
        if (l[i] < r.p ? a > r.anchor : a < r.anchor) return false;
      }
      return true;
    }
    // 島の大きさで戻る: 残りのピースの大きさで作れる和か(部分和の表)
    function islandsOk() {
      var left = []; for (var p = 0; p < NP; p++) if (!colDone[NC + p]) left.push(sizeOfPiece[p]);
      var can = { 0: true }, sum = 0;
      left.forEach(function (sz) { sum += sz; var nx = Object.assign({}, can); Object.keys(can).forEach(function (v) { nx[+v + sz] = true; }); can = nx; });
      var seen = new Uint8Array(NC);
      for (var i = 0; i < NC; i++) {
        if (colDone[i] || seen[i]) continue;
        var q = [i], n = 0; seen[i] = 1;
        while (q.length) {
          var u = q.pop(); n++;
          var cu = target[u];
          [[1, 0], [-1, 0], [0, 1], [0, -1]].forEach(function (d) {
            var v = cellIdx[key(cu[0] + d[0], cu[1] + d[1])];
            if (v !== undefined && !colDone[v] && !seen[v]) { seen[v] = 1; q.push(v); }
          });
        }
        if (!can[n]) return false;
      }
      return true;
    }
    function rec() {
      if (sols.length >= max || timedOut) return;
      if (++nodes % 2048 === 0 && Date.now() - t0 > limit) { timedOut = true; return; }
      var best = -1, bestN = Infinity;
      for (var col = 0; col < NC + NP; col++) {
        if (colDone[col]) continue;
        var n = live(col);
        if (n < bestN) { bestN = n; best = col; if (n === 0) break; }
      }
      if (best < 0) { sols.push(stack.map(function (ri) { var r = rows[ri]; return { piece: r.p, name: pieces[r.p].name, shape: pieces[r.p].shape, k: r.k, cells: r.cells, origin: r.origin }; })); return; }
      if (bestN === 0) return;
      if (!islandsOk()) return;
      var cand = colRows[best].filter(function (ri) { return !rowDead[ri] && pieceAllowed(ri); });
      for (var i = 0; i < cand.length; i++) {
        var ri = cand[i];
        cover(ri); stack.push(ri); placedAnchor[rows[ri].p] = rows[ri].anchor;
        rec();
        placedAnchor[rows[ri].p] = -1; stack.pop(); uncover(ri);
        if (sols.length >= max || timedOut) return;
      }
    }
    rec();
    var why = timedOut ? (sols.length ? "" : "敷き詰め方を " + (limit / 1000) + " 秒で見つけられなかった(打ち切り)") : (sols.length ? "" : "敷き詰め方が無い");
    // 答えが 1 つ以上あって打ち切ったときも、timedOut で分かる(画面は「10 通りで打ち切り」と同じ扱い)
    return { solutions: sols, reason: why, nodes: nodes, timedOut: timedOut, ms: Date.now() - t0 };
  }

  /**
   * つかみ方の候補。挟む向き(局所の [1,0] か [0,1])に並んだマスの「ひと続き」を
   * まるごと挟む(1 マスだけとは限らない。O のような 2×2 は 2 マス分を挟む)。
   *   center … 挟む範囲の真ん中(局所のマス。半端な値もある)
   *   run    … 挟むマスの数(maxRun まで)
   *   ends   … 爪の指が下りるマス 2 つ(ひと続きの両外)
   * 重心に近い順、同じなら run が小さい順(爪を大きく開かない)。
   */
  function graspOptions(shape, maxRun) {
    maxRun = maxRun || 1;
    var cells = SHAPES[shape];
    var has = {}; cells.forEach(function (c) { has[key(c[0], c[1])] = true; });
    var g = [0, 0]; cells.forEach(function (c) { g[0] += c[0] / cells.length; g[1] += c[1] / cells.length; });
    var out = [];
    cells.forEach(function (c) {
      [[1, 0], [0, 1]].forEach(function (ax) {
        // c がひと続きの始まりのときだけ数える(重ねて数えない)
        if (has[key(c[0] - ax[0], c[1] - ax[1])]) return;
        var n = 1;
        while (has[key(c[0] + ax[0] * n, c[1] + ax[1] * n)]) n++;
        if (n > maxRun) return;
        var ctr = [c[0] + ax[0] * (n - 1) / 2, c[1] + ax[1] * (n - 1) / 2];
        out.push({ center: ctr, axis: ax, run: n,
                   ends: [[c[0] - ax[0], c[1] - ax[1]], [c[0] + ax[0] * n, c[1] + ax[1] * n]],
                   d: Math.hypot(ctr[0] - g[0], ctr[1] - g[1]) });
      });
    });
    out.sort(function (p, q) { return (p.d - q.d) || (p.run - q.run); });
    return out;
  }

  /** 置いた後の盤での、挟む範囲の真ん中・挟む向き・指が下りるマス(盤のマス) */
  function graspOnBoard(pl, gopt) {
    function tb(c) { var r = rotK(c, pl.k); return [r[0] + pl.origin[0], r[1] + pl.origin[1]]; }
    return { center: tb(gopt.center), axis: rotK(gopt.axis, pl.k), ends: gopt.ends.map(tb) };
  }

  /**
   * 置く順番とつかみ方を一緒に探す。指が下りるマスに置き済みのピースが無いこと。
   *   opt.fingerFits … 置き済みのピースとのすき間に指が入る(すき間 ≥ 指の厚さ + 余裕)。
   *                    そのときは隣が埋まっていても当たらない
   *   opt.maxRun     … まとめて挟めるマスの数の上限(爪の開きで決まる)
   * 戻り値 { ok, steps:[{pl, grasp(局所), board(盤)}], reason }
   */
  function planOrder(solution, opt) {
    opt = opt || {};
    var fits = !!opt.fingerFits, maxRun = opt.maxRun || 1;
    var n = solution.length, done = [], filled = {}, steps = [];
    for (var z = 0; z < n; z++) if (!graspOptions(solution[z].shape, maxRun).length)
      return { ok: false, steps: [], reason: solution[z].name + " のピースは、爪の開き(" + maxRun + " マスまで)では挟める所が無い" };
    function rec() {
      if (steps.length === n) return true;
      // 前(ロボットから遠い = a が大きい)から置く方を先に試す(腕が置き済みの上を通りにくい)
      var idx = [];
      for (var i = 0; i < n; i++) if (!done[i]) idx.push(i);
      idx.sort(function (p, q) {
        var ap = Math.max.apply(null, solution[p].cells.map(function (c) { return c[0]; }));
        var aq = Math.max.apply(null, solution[q].cells.map(function (c) { return c[0]; }));
        return aq - ap;
      });
      for (var t = 0; t < idx.length; t++) {
        var pl = solution[idx[t]];
        var opts = graspOptions(pl.shape, maxRun);
        // opt.grasp = {ピースの名前 か "#番号": 番号}(S1。2026-11-22): graspOptions の並びのその番号のつかみ方だけを使う(無い番号は断る。既定は並びの順に試す)。
        //   同じ名前のピース(V3 V3・O O)は名前だと両方に効くので、分けるときは "#番号"(ピースの番号)。番号の並びは寸法(maxRun)で変わる
        var gk = opt.grasp ? (opt.grasp["#" + pl.piece] !== undefined ? "#" + pl.piece : opt.grasp[pl.name] !== undefined ? pl.name : null) : null;
        if (gk !== null) {
          var gi = opt.grasp[gk] | 0;
          if (!(gi >= 0 && gi < opts.length)) { badGrasp = pl.name + " のつかみ方 " + gi + " 番は無い(0〜" + (opts.length - 1) + ")"; return false; }
          opts = [opts[gi]];
        }
        for (var o = 0; o < opts.length; o++) {
          var b = graspOnBoard(pl, opts[o]);
          if (!fits && b.ends.some(function (c) { return filled[key(c[0], c[1])]; })) continue;
          done[idx[t]] = true;
          pl.cells.forEach(function (c) { filled[key(c[0], c[1])] = true; });
          steps.push({ pl: pl, grasp: opts[o], board: b });
          if (rec()) return true;
          steps.pop();
          pl.cells.forEach(function (c) { delete filled[key(c[0], c[1])]; });
          done[idx[t]] = false;
          break;                     // 同じピースのほかのつかみ方は、置き済みの集合が同じなので試しても同じ
        }
      }
      return false;
    }
    var badGrasp = "";
    var ok = rec();
    return { ok: ok, steps: steps, reason: ok ? "" : badGrasp || "どの順番でも、指が下りるマスに置き済みのピースがある" };
  }

  /** 目標の形を文字で(■ = 埋める)。a を縦(上が前)、b を横に */
  function rect(na, nb) {
    var t = [];
    for (var a = 0; a < na; a++) for (var b = 0; b < nb; b++) t.push([a, b]);
    return t;
  }

  return { SHAPES: SHAPES, rotK: rotK, orient: orient, solve: solve, solveX: solveX, fromArt: fromArt,
           graspOptions: graspOptions, graspOnBoard: graspOnBoard, planOrder: planOrder, rect: rect };
})();

if (typeof module !== "undefined") module.exports = PUZZLE;
