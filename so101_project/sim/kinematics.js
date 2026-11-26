/* =============================================================================
 * kinematics.js — CRX-20iA/L の順運動学 / 逆運動学
 *
 * ■ 何のためにあるか
 *   関節角を6本のスライダーで手合わせするのは実用にならない。
 *   「カメラをこの位置に、この向きで置きたい」から関節角を逆算できて初めて、
 *   撮影パス(ホイールの周回)を設計できる。
 *
 * ■ 座標系と姿勢の定義
 *   JOINT_AXES[i] = { axis, origin } は tools/extract_axes.py が2姿勢の .obj から
 *   厳密に求めた値で、three.js 系(m)・CAD原点基準。全関節角 0 のとき
 *   CAD の折り畳み姿勢になる。axis は「その姿勢での」軸の向きなので、
 *   FK では親から順に回転を掛けていけばよい。
 *
 * ■ IK の方式
 *   この機種は J4 と J6 が 149mm 離れていて球面手首ではないため、
 *   Pieper の解析解が使えない。減衰最小二乗法(DLS)で数値的に解く:
 *       Δq = Jᵀ (J Jᵀ + λ²I)⁻¹ e
 *   λ を入れるのは特異点付近で Δq が発散するのを防ぐため。
 *
 * ■ 姿勢の拘束は「工具軸の向き」だけ(5自由度)
 *   カメラは光軸さえ合っていればよく、光軸まわりの回転(ロール)は自由。
 *   ロールまで拘束すると解ける範囲が無駄に狭くなるので、姿勢誤差は
 *   「現在の工具軸 × 目標の工具軸」で作る。この量にはロール成分が
 *   原理的に入らないので、自然に5自由度拘束になる。
 * ========================================================================== */

var KIN = (function () {
  "use strict";

  var DEG = Math.PI / 180;

  // ---- ベクトル ----
  function cross(a, b) {
    return [a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]];
  }
  function dot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
  function sub(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
  function add(a, b) { return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]; }
  function scale(a, s) { return [a[0] * s, a[1] * s, a[2] * s]; }
  function norm(a) { return Math.sqrt(dot(a, a)); }
  function unit(a) { var n = norm(a) || 1; return scale(a, 1 / n); }

  // ---- 3x3 行列 (行優先の長さ9配列) ----
  var I3 = [1, 0, 0, 0, 1, 0, 0, 0, 1];
  function matVec(M, v) {
    return [M[0] * v[0] + M[1] * v[1] + M[2] * v[2],
            M[3] * v[0] + M[4] * v[1] + M[5] * v[2],
            M[6] * v[0] + M[7] * v[1] + M[8] * v[2]];
  }
  function matMul(A, B) {
    var C = new Array(9);
    for (var r = 0; r < 3; r++) {
      for (var c = 0; c < 3; c++) {
        C[r * 3 + c] = A[r * 3] * B[c] + A[r * 3 + 1] * B[3 + c] + A[r * 3 + 2] * B[6 + c];
      }
    }
    return C;
  }
  /** 軸 a まわりに th ラジアン回す回転行列 (ロドリゲス) */
  function rotAA(a, th) {
    var x = a[0], y = a[1], z = a[2];
    var c = Math.cos(th), s = Math.sin(th), t = 1 - c;
    return [t * x * x + c,     t * x * y - s * z, t * x * z + s * y,
            t * x * y + s * z, t * y * y + c,     t * y * z - s * x,
            t * x * z - s * y, t * y * z + s * x, t * z * z + c];
  }

  /**
   * 順運動学。
   *   axes … [{axis:[x,y,z], origin:[x,y,z]}, ...] 6個
   *   qDeg … 関節角[度] 6個
   *   tool … {pos:[x,y,z], dir:[x,y,z]} 全関節0のときの工具取付面の位置と外向き
   * 戻り値 { pos, dir, R, joints:[{axis, origin}], frames:[{R, p, o}] }
   *   joints[i] はその姿勢での「関節iの軸の向きと軸上の点」(ヤコビアンに使う)
   *
   *   ★joints[i].origin は「軸上のどこか1点」であって、関節の物理的な位置ではない。
   *     実際 J5→J6 の origin 間距離は 904mm もあり、実機の手首寸法(149mm)とは
   *     全く違う。したがって origin どうしを結んだ線分を「腕の形」だと思っては
   *     いけない(干渉判定をそれで作ってしまい、腕が実在しない場所で
   *     当たり判定を出していた)。
   *
   *   frames[i] は「関節 i を回した後のリンク座標系」。
   *     CAD 座標の点 v は  frames[i].p + frames[i].R ·(v - frames[i].o)  で
   *     その姿勢での位置になる。リンク形状(メッシュ)を置くにはこちらを使う。
   *     scene-data.js のシーングラフ(j{i+1} の子に arm_J{i+1} を -origin だけ
   *     ずらして置く)と同じ変換になっている。
   */
  function fk(axes, qDeg, tool) {
    var R = I3.slice();
    var p = [0, 0, 0];
    var prev = [0, 0, 0];
    var joints = [];
    var frames = [];
    for (var i = 0; i < axes.length; i++) {
      // 親フレームから見た、この関節の位置
      var t = sub(axes[i].origin, prev);
      p = add(p, matVec(R, t));
      joints.push({ axis: matVec(R, axes[i].axis), origin: p.slice() });
      R = matMul(R, rotAA(axes[i].axis, (qDeg[i] || 0) * DEG));
      // 関節 i を回した後の座標系。この系での CAD 原点は axes[i].origin
      frames.push({ R: R.slice(), p: p.slice(), o: axes[i].origin });
      prev = axes[i].origin;
    }
    var last = axes[axes.length - 1].origin;
    return {
      pos: add(p, matVec(R, sub(tool.pos, last))),
      dir: unit(matVec(R, tool.dir)),
      R: R,
      joints: joints,
      frames: frames
    };
  }

  // ---- 連立一次方程式 (部分ピボット付きガウス消去) ----
  function solve(A, b) {
    var n = b.length;
    var M = A.map(function (row, i) { return row.slice().concat([b[i]]); });
    for (var c = 0; c < n; c++) {
      var piv = c;
      for (var r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[piv][c])) piv = r;
      if (Math.abs(M[piv][c]) < 1e-14) return null;
      var tmp = M[c]; M[c] = M[piv]; M[piv] = tmp;
      for (var r2 = 0; r2 < n; r2++) {
        if (r2 === c) continue;
        var f = M[r2][c] / M[c][c];
        for (var k = c; k <= n; k++) M[r2][k] -= f * M[c][k];
      }
    }
    return M.map(function (row, i) { return row[n] / M[i][i]; });
  }

  /**
   * 逆運動学 (減衰最小二乗)。
   *   targetPos … 工具取付面を持っていきたい位置
   *   targetDir … 工具軸(取付面の外向き)を向けたい方向。null なら位置だけ合わせる
   *   opt = { q0, limitsDeg, iters, posTol, dirTolDeg, lambda, stepLimitDeg }
   * 戻り値 { q, ok, posErr, dirErrDeg, iters }
   */
  function ik(axes, tool, targetPos, targetDir, opt) {
    opt = opt || {};
    var n = axes.length;
    var q = (opt.q0 || new Array(n).fill(0)).slice();
    var lim = opt.limitsDeg || null;
    var iters = opt.iters || 120;
    var posTol = opt.posTol !== undefined ? opt.posTol : 0.0005;   // 0.5mm
    var dirTol = (opt.dirTolDeg !== undefined ? opt.dirTolDeg : 0.5) * DEG;
    var lambda = opt.lambda !== undefined ? opt.lambda : 0.02;
    var stepLimit = (opt.stepLimitDeg || 12) * DEG;
    // ★固定する軸(番号の配列)。q0 に入っている値のまま動かさない
    var lock = opt.lockJoints || null;

    var st, posErr = Infinity, dirErr = Infinity, it = 0;
    for (it = 0; it < iters; it++) {
      st = fk(axes, q, tool);

      var ev = sub(targetPos, st.pos);
      posErr = norm(ev);
      var ew = [0, 0, 0];
      dirErr = 0;
      if (targetDir && opt.targetUp && tool.up) {
        // ---- 姿勢を6自由度で拘束する ----
        // ★工具軸の向きだけ(5自由度)にすると、軸まわりのロールが自由に残る。
        //   J4 と J6 はまさにそのロール軸なので値が定まらず、経由点ごとに
        //   180°級で飛んでいた。基準方向(up)まで指定して冗長性を無くす。
        var zc = st.dir;
        var xcRaw = matVec(st.R, tool.up);
        var xc = unit(sub(xcRaw, scale(zc, dot(xcRaw, zc))));
        var yc = cross(zc, xc);
        var zt = unit(targetDir);
        var xtRaw = opt.targetUp;
        var xt = unit(sub(xtRaw, scale(zt, dot(xtRaw, zt))));
        var yt = cross(zt, xt);
        // Rerr = Rt * Rc^T を、列ベクトルの組から直接作る
        var Rerr = [
          xt[0] * xc[0] + yt[0] * yc[0] + zt[0] * zc[0],
          xt[0] * xc[1] + yt[0] * yc[1] + zt[0] * zc[1],
          xt[0] * xc[2] + yt[0] * yc[2] + zt[0] * zc[2],
          xt[1] * xc[0] + yt[1] * yc[0] + zt[1] * zc[0],
          xt[1] * xc[1] + yt[1] * yc[1] + zt[1] * zc[1],
          xt[1] * xc[2] + yt[1] * yc[2] + zt[1] * zc[2],
          xt[2] * xc[0] + yt[2] * yc[0] + zt[2] * zc[0],
          xt[2] * xc[1] + yt[2] * yc[1] + zt[2] * zc[1],
          xt[2] * xc[2] + yt[2] * yc[2] + zt[2] * zc[2]
        ];
        var ang2 = Math.acos(Math.max(-1, Math.min(1, (Rerr[0] + Rerr[4] + Rerr[8] - 1) / 2)));
        dirErr = ang2;
        var vv = [Rerr[7] - Rerr[5], Rerr[2] - Rerr[6], Rerr[3] - Rerr[1]];
        var nv = norm(vv);
        if (nv > 1e-9) ew = scale(vv, ang2 / nv);
        else if (ang2 > 1) ew = [Math.PI, 0, 0];     // 180°回転の縮退
      } else if (targetDir) {
        // 工具軸の向きだけ合わせる(5自由度)。ロールはどうでもよい用途向け
        var d = unit(targetDir);
        var c = cross(st.dir, d);
        var s = norm(c), co = dot(st.dir, d);
        dirErr = Math.atan2(s, co);
        // 真逆を向いている場合は cross が 0 になるので、適当な直交軸で回し始める
        if (s < 1e-9) {
          ew = (co > 0) ? [0, 0, 0]
                        : scale(unit(cross(st.dir, Math.abs(st.dir[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0])), Math.PI);
        } else {
          ew = scale(c, dirErr / s);
        }
      }
      if (posErr < posTol && dirErr < dirTol) break;

      // ヤコビアン: 回転関節 i について
      //   位置の変化 = 軸 × (手先 - 軸上の点) 、 姿勢の変化 = 軸
      var rows = targetDir ? 6 : 3;
      var J = [];
      for (var r = 0; r < rows; r++) J.push(new Array(n).fill(0));
      for (var i = 0; i < n; i++) {
        // ★固定した軸は「動かせない」ので、ヤコビアンの列を 0 にする。
        //   こうすると Δq = Jᵀy がその軸について必ず 0 になり、
        //   残りの軸だけで目標に寄せる解が出る。
        //   台の下へ手を入れる区間で J1 を固定し、腕を鉛直面内に
        //   留めるために使う(面外へ振れると受けピン・スポークに当たる)。
        if (lock && lock.indexOf(i) !== -1) continue;
        var a = st.joints[i].axis;
        var jv = cross(a, sub(st.pos, st.joints[i].origin));
        J[0][i] = jv[0]; J[1][i] = jv[1]; J[2][i] = jv[2];
        if (targetDir) { J[3][i] = a[0]; J[4][i] = a[1]; J[5][i] = a[2]; }
      }
      var e = targetDir ? [ev[0], ev[1], ev[2], ew[0], ew[1], ew[2]]
                        : [ev[0], ev[1], ev[2]];

      // (J Jᵀ + λ²I) y = e  を解いて  Δq = Jᵀ y
      var A = [];
      for (var r3 = 0; r3 < rows; r3++) {
        A.push(new Array(rows).fill(0));
        for (var c3 = 0; c3 < rows; c3++) {
          var s2 = 0;
          for (var k2 = 0; k2 < n; k2++) s2 += J[r3][k2] * J[c3][k2];
          A[r3][c3] = s2 + (r3 === c3 ? lambda * lambda : 0);
        }
      }
      var y = solve(A, e);
      if (!y) break;

      var dq = new Array(n).fill(0);
      for (var i2 = 0; i2 < n; i2++) {
        var s3 = 0;
        for (var r4 = 0; r4 < rows; r4++) s3 += J[r4][i2] * y[r4];
        dq[i2] = s3;
      }

      // ---- 零空間で「基準姿勢に近づける」 ----
      // ★工具軸だけを拘束する(5自由度)ので、光軸まわりのロールが自由に残る。
      //   そのぶん解が漂い、撮影点ごとに J6 が 80〜160° 跳ねていた。
      //   手先を動かさない方向(零空間)に限って基準姿勢へ引き戻すと、
      //   手先姿勢はそのままで関節の動きだけが滑らかになる。
      //     Δq += (I - Jᵀ(JJᵀ+λ²I)⁻¹J) · k · (qRef - q)
      if (opt.qRef && opt.nullGain) {
        var e2 = [];
        for (var i5 = 0; i5 < n; i5++) {
          var dv = opt.qRef[i5] - q[i5];
          while (dv > 180) dv -= 360;
          while (dv < -180) dv += 360;
          e2.push(dv * DEG * opt.nullGain);
        }
        // z = J·e2 、 w = (JJᵀ+λ²I)⁻¹ z 、 Δq += e2 - Jᵀ·w
        var z = [];
        for (var r5 = 0; r5 < rows; r5++) {
          var sz = 0;
          for (var k5 = 0; k5 < n; k5++) sz += J[r5][k5] * e2[k5];
          z.push(sz);
        }
        var w2 = solve(A, z);
        if (w2) {
          for (var i6 = 0; i6 < n; i6++) {
            var sw = 0;
            for (var r6 = 0; r6 < rows; r6++) sw += J[r6][i6] * w2[r6];
            dq[i6] += e2[i6] - sw;
          }
        }
      }

      // ★固定した軸は零空間項でも動かさない(上でヤコビアン列を0にしても、
      //   零空間項の e2 はそのまま残るため、ここで明示的に止める)
      if (lock) for (var iL = 0; iL < lock.length; iL++) dq[lock[iL]] = 0;

      var mx = 0;
      for (var i7 = 0; i7 < n; i7++) if (Math.abs(dq[i7]) > mx) mx = Math.abs(dq[i7]);
      // 1回で飛びすぎると発散するので、最大変化量で頭打ちにする
      var sc = (mx > stepLimit) ? stepLimit / mx : 1;
      for (var i3 = 0; i3 < n; i3++) {
        q[i3] += dq[i3] * sc / DEG;
        if (lim) q[i3] = Math.max(lim[i3][0], Math.min(lim[i3][1], q[i3]));
      }
    }
    st = fk(axes, q, tool);
    var fev = norm(sub(targetPos, st.pos));
    var fdir = dirErr;   // 6自由度拘束のときはロール誤差も含んだ値
    return {
      q: q,
      ok: fev < posTol * 4 && fdir < dirTol * 4,
      posErr: fev,
      dirErrDeg: fdir / DEG,
      iters: it
    };
  }

  /**
   * ik() を初期値を変えて何度か試す版。
   * DLS は最急降下なので初期値が悪いと局所解に落ちる(固定初期値だと 200回中29回失敗した)。
   * 前回解 → 全0 → 乱数、の順に試して最初に収束したものを採る。
   * 乱数は固定シードなので、同じ条件なら毎回同じ答えになる(打合せ中に絵が変わらない)。
   */
  var _seed = 1;
  function rnd() { _seed = (1103515245 * _seed + 12345) & 0x7fffffff; return _seed / 0x7fffffff; }

  function ikRobust(axes, tool, targetPos, targetDir, opt) {
    opt = opt || {};
    var n = axes.length;
    var tries = [];
    if (opt.q0) tries.push(opt.q0.slice());
    // ★外から渡された初期値(前の設計での解など)を必ず試す。
    //   これが無いと、設計を少し変えただけで「それまで見つけていた良い解」が
    //   候補から漏れ、探索運で結果が悪化する。実際、受け台を削って
    //   材料を減らしただけなのに -24mm → -40mm と悪化して見えたのは
    //   幾何が悪くなったのではなく、この漏れが原因だった。
    if (opt.extraSeeds) {
      for (var es = 0; es < opt.extraSeeds.length; es++) {
        if (opt.extraSeeds[es]) tries.push(opt.extraSeeds[es].slice());
      }
    }
    // ★まず「今いる枝の中」を丁寧に探す。
    //   前の姿勢をわずかに揺らした初期値をいくつか入れる。
    //   これが無いと、前の姿勢からの1回の降下で落ちた局所解だけを見て
    //   「この枝は駄目だ」と判断し、遠くの別の枝へ飛んでいた。
    //   実際それでホイールを持ったまま肘を乗り換え、ワークが一回転していた。
    //   (実形状で調べると、同じ枝のまま干渉なく置ける解は存在していた)
    if (opt.q0) {
      _seed = 7;
      for (var np = 0; np < 6; np++) {
        var g0 = opt.q0.slice();
        for (var i0 = 0; i0 < n; i0++) g0[i0] += (rnd() * 2 - 1) * 25;
        tries.push(g0);
      }
    }
    tries.push(new Array(n).fill(0));
    var restarts = opt.restarts !== undefined ? opt.restarts : 6;
    _seed = 1;                                   // 再現性のため毎回リセット
    for (var t = 0; t < restarts; t++) {
      var g = [];
      for (var i = 0; i < n; i++) g.push((rnd() * 2 - 1) * 150);
      tries.push(g);
    }
    // ★固定する軸は、どの初期値でも指定値から始める。
    //   ランダム初期値のまま固定すると、その軸だけデタラメな角度で
    //   固まった解が返ってしまう
    if (opt.lockJoints && opt.q0) {
      for (var tl = 0; tl < tries.length; tl++) {
        for (var jl = 0; jl < opt.lockJoints.length; jl++) {
          tries[tl][opt.lockJoints[jl]] = opt.q0[opt.lockJoints[jl]];
        }
      }
    }
    // ★「最初に収束したもの」を返してはいけない。
    //   6軸機は同じ手先姿勢に対して複数の関節解を持つので、撮影点ごとに
    //   別の解を選ぶと、隣り合う点のあいだで腕が大きく振り回される
    //   (実際 J1 が 23.6° -> -150.4° と 174° 飛ぶ動きになった)。
    //   収束した解を全部集めて、そのなかで「前の姿勢に一番近いもの」を採る。
    //
    // ★opt.penalty(q) があれば、それも加味して選ぶ。
    //   6軸機は同じ手先姿勢に対して複数の解(肘上/肘下など)を持つので、
    //   「近いか」だけで選ぶと障害物を貫く枝を平気で選んでしまう。
    //   実際、台の下へ手を入れる姿勢では肘上と肘下の2解があり、
    //   肘上は前腕がホイールを貫通する。penalty はその貫通量[m]を返す。
    var ref = opt.q0 || null;
    var pen = opt.penalty || null;
    // ★干渉ペナルティの重み。貫通[mm] × PW を距離コストに足す。
    //   距離コストは「関節角の差の2乗和[度²]」なので、1軸が180°動いて 32400。
    //   以前は 95mm の貫通が 36000 にしかならず、距離コストとほぼ拮抗していた。
    //   その結果 IK が「肘を下に切り替えるくらいなら貫通したほうが安い」と
    //   判断して、前腕がホイールを突き抜ける枝を選び続けていた。
    //   1mm = 5000 にすると 95mm で 475,000 となり、どんな大回りより高くなる。
    //   → 干渉しない解があるなら必ずそちらを選ぶ。無いときだけ貫通が最小の解。
    var PW = opt.penaltyWeight !== undefined ? opt.penaltyWeight : 5000;
    var best = null, bestOk = null, bestCost = Infinity;
    for (var k = 0; k < tries.length; k++) {
      var o = Object.assign({}, opt, { q0: tries[k] });
      var r = ik(axes, tool, targetPos, targetDir, o);
      if (r.ok) {
        var dist = 0;
        if (ref) {
          for (var m = 0; m < n; m++) {
            var dd = Math.abs(r.q[m] - ref[m]) % 360;
            if (dd > 180) dd = 360 - dd;
            dist += dd * dd;
          }
        }
        var hit = pen ? Math.max(0, pen(r.q)) : 0;      // 貫通量[m]
        var cost = dist + hit * 1000 * PW;
        // ★ワークを持っている間は「枝の乗り換え」を禁じる。
        //   opt.maxStepDeg を超えて動く解は、多少クリアランスが良くても採らない。
        //   これが無いと、数mmの干渉を避けるために肘を乗り換え、
        //   吊っているホイールが空中で一回転してしまう(安全上よくない)。
        //   同じ枝に解が1つも無いときだけ、乗り換えを許す。
        var far = false;
        if (opt.maxStepDeg && ref) {
          var mv = 0;
          for (var mm = 0; mm < n; mm++) {
            var dm = Math.abs(r.q[mm] - ref[mm]) % 360;
            if (dm > 180) dm = 360 - dm;
            if (dm > mv) mv = dm;
          }
          far = (mv > opt.maxStepDeg);
        }
        if (far) cost += 1e9;                    // 同じ枝の解が無いときの保険
        if (cost < bestCost) { bestCost = cost; bestOk = r; }
        // 干渉が無く、前の姿勢からほとんど動かない解が見つかったら打ち切ってよい
        if (ref && hit <= 0 && bestCost < 400) break;   // 全軸合計で 20°相当
        if (!ref && !pen) break;                        // 初回・干渉考慮なしなら最初の解でよい
      }
      if (!best || (r.posErr + r.dirErrDeg * 0.01) < (best.posErr + best.dirErrDeg * 0.01)) best = r;
    }
    return bestOk || best;
  }

  return { fk: fk, ik: ik, ikRobust: ikRobust, rotAA: rotAA, matVec: matVec, matMul: matMul,
           cross: cross, dot: dot, unit: unit, sub: sub, add: add, scale: scale,
           norm: norm, solve: solve };
})();
