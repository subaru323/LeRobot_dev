// 想定動作の 1 手ずつの時間の内訳と、律速の段(計画書_完成形まで の S-12)。
//   ちくわ排除機構の line-sim の tact.txt(段ごとの内訳と律速)と、光学設計の手順 3rd §3 第 7 段
//   「タクト = 姿勢の数 × (移動 + 整定) + 枚数 × (露光 + 転送 + 推論)」を、SO-101 のパズルに置き直した。
//   想定動作(docs/plan/想定動作_SO101パズル.md): 上で写す → 選ぶ → 近づく → 手首で確かめる → 下ろしてつかむ → 運んで置く → 離して上へ → (次の手)。
//   見ている世界: scene-data.js の 4 種(既定のすき間・台あり)。動きの時間は scene-data.js の動き(速さ 関節 90°/s 平均・直線 150mm/s【仮】)。
//     撮る時間・整定は動きに入っていないので、下の【仮】の値を足す。
//   比べる 2 つの回し方:
//     A「上で写すのは始めと終わりだけ」(いまの動き)… 置いたずれは最後にまとめて見る
//     B「毎手、上で写す」… 置くたびに写す姿勢へ行って写し、待ちの姿勢へ戻る(往復の時間は、いまの動きの始めと終わりの区間から測る)
//   検算: 段の時刻が前から順に並ぶ。つながりの確認: 段の和が動きの全体の時間(_check.T)に一致する(同じ時間の流れの差を足すので、段の足し忘れしか捕まえない。
//     別の出どころとの突き合わせではない。点検 C14 の 6)。
//   手首で測ったずれを直す場合(C): 1 手ごとに、直す動き(関節の最短 0.5 秒)+ 整定 + もう 1 枚 を足す(毎回直すとした上限の目安。点検 C14 の 4)。
//   実行: node tools/cycle-tact.js
const { load } = require("./_load");
load("kinematics.js", "so101.js", "motion.js", "puzzle.js",
     "models/so101_visuals.js", "models/so101_points.js", "models/so101_tris.js", "collide.js", "scene-data.js");
const M = SCENE_MODEL;
// 【仮】の値(実物で測るまで)。ちくわは 1 枚 = 露光 + 転送 + 推論、姿勢ごとに整定を足していた
const T_TOP_SHOT = 0.5;      // 上のカメラで 1 枚撮って、ピースを見つける(露光 + 転送 + 推論)【仮】
// 手首で確かめる止まり(整定 + 撮る)は、scene-data.js の動きに入っている(WRIST_CHECK。決めたこと 6)。値もそこから読む(持ち主を 1 つにする)
const T_SETTLE = 0.3;        // 上で撮る前に腕の揺れが収まるまで【仮】(手首の側は scene-data.js の WRIST_SETTLE)
const T_FIX_MOVE = 0.5;      // 真上の位置を直す動き(scene-data.js の moveJoint の最短)
let bad = 0;
const s1 = x => x.toFixed(1);
for (let set = 0; set < 4; set++) {
  const d = M.buildFromUi({ SET: set, SOL: 0, NO_REFINE: true }), c = d._check;
  if (!c.solved) { console.log(`[種 ${set}] 解けない: ${c.why}`); bad++; continue; }
  const mo = c.motion, same = (a, b) => a.every((v, i) => Math.abs(v - b[i]) < 1e-6);
  const tHome0 = mo.find(f => same(f.q, c.homeQ)).t;                                  // 写す姿勢 → 待ちの姿勢 に着いた時刻
  const S = c.stepTimes, last = S[S.length - 1];
  const tHomeEnd = mo.find(f => f.t > last.tDone + 1e-9 && same(f.q, c.homeQ)).t;    // 最後に待ちの姿勢へ戻った時刻
  const tPhotoEnd = mo.find(f => f.t > tHomeEnd + 1e-9 && same(f.q, c.photoQ)).t;    // 写す姿勢に着いた時刻
  const wc = c.wristCheck;
  if (!wc) { console.log(`[種 ${set}] 手首で確かめる止まりが動きに無い(WRIST_CHECK=0 か手首のカメラなし)`); bad++; continue; }
  const T_WRIST_SHOT = wc.shot, T_WSETTLE = wc.settle;
  const seg = { "写す姿勢→待ち": tHome0, "近づく": 0, "手首で確かめる": 0, "下ろしてつかむ": 0, "運んで置く": 0, "離して上へ": 0, "待ちへ戻る": tHomeEnd - last.tDone,
                "待ち→写す姿勢": tPhotoEnd - tHomeEnd, "終わりに止まる": c.T - tPhotoEnd };
  let prev = tHome0, order = true;
  S.forEach(s => {
    if (!(prev <= s.tHover && s.tHover <= s.tPick && s.tPick <= s.tPlace && s.tPlace <= s.tDone)) order = false;
    seg["近づく"] += s.tHover - prev; seg["手首で確かめる"] += s.tCheck1 - s.tCheck0; seg["下ろしてつかむ"] += (s.tPick - s.tHover) - (s.tCheck1 - s.tCheck0);
    if (!(s.tHover <= s.tCheck0 && s.tCheck0 < s.tCheck1 && s.tCheck1 <= s.tPick) || Math.abs(s.tCheck1 - s.tCheck0 - (wc.settle + wc.shot)) > 1e-6) order = false; seg["運んで置く"] += s.tPlace - s.tPick; seg["離して上へ"] += s.tDone - s.tPlace;
    prev = s.tDone;
  });
  const sum = Object.values(seg).reduce((a, b) => a + b, 0), okSum = Math.abs(sum - c.T) < 1e-6;
  if (!okSum || !order) bad++;
  const n = S.length;
  // 撮る時間(動きに入っていない)
  const shotA = { "上で撮る(始めと終わり)": 2 * (T_SETTLE + T_TOP_SHOT) };
  const A = c.T + Object.values(shotA).reduce((a, b) => a + b, 0);
  const round = seg["写す姿勢→待ち"] + seg["待ち→写す姿勢"];                       // 待ち ⇄ 写す姿勢 の往復
  // 最後の手のあとは A でも写すので n − 1 回足す。置いた所から待ちへ戻る分は、最後の手の「待ちへ戻る」で代える(目安。
  //   待ちから次の真上へ行く分は、A の「近づく」に入っているとみなす。B の動きそのものは組んでいない = 当たり判定を通していない)
  const B = A + (n - 1) * (seg["待ちへ戻る"] + round + T_SETTLE + T_TOP_SHOT);
  const C = A + n * (T_FIX_MOVE + T_WSETTLE + T_WRIST_SHOT);
  const all = Object.assign({}, seg, shotA), tot = A;
  const rows = Object.entries(all).sort((a, b) => b[1] - a[1]);
  console.log(`[種 ${set}] ${n} 手(${c.order.join(" → ")})。A(始めと終わりだけ上で写す) ${s1(A)} 秒・B(毎手写す・目安) ${s1(B)} 秒(待ちへ戻る ${s1(seg["待ちへ戻る"])} + 往復 ${s1(round)} + 整定と撮る ${s1(T_SETTLE + T_TOP_SHOT)} 秒、を ${n - 1} 回)・C(A で毎手ずれを直す) ${s1(C)} 秒`);
  console.log("  A の内訳: " + rows.map(([k, v]) => `${k} ${s1(v)} 秒(${Math.round(v / tot * 100)})`).join("・"));
  console.log(`  律速: ${rows[0][0]}。1 手あたり ${s1((A - seg["写す姿勢→待ち"] - seg["待ち→写す姿勢"] - seg["終わりに止まる"] - shotA["上で撮る(始めと終わり)"]) / n)} 秒` +
    `・時刻の順 ${order ? "OK" : "★NG"}・つながり(段の和 ${s1(sum)} = 動き ${s1(c.T)} 秒) ${okSum ? "OK" : "★NG"}`);
}
console.log(`【仮】上で撮る ${T_TOP_SHOT} 秒・整定 ${T_SETTLE} 秒(上)・手首は scene-data.js の WRIST_SETTLE + WRIST_SHOT・関節 90°/s 平均・直線 150mm/s。括弧の数は全体に対する百分の割合。`);
console.log(bad ? `★NG ${bad}` : "検算は全部 OK");
process.exit(bad ? 1 : 0);
