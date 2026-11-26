// 検算: 画面と同じファイルを読み、目標の形 × すき間 ごとに「解けたか」と数字を出す。
//   見ている世界: sim/scene-data.js の SETS と【仮の値】・SO-101(so101_new_calib.urdf と公式の STL)
//   見ている当たり: 腕・爪・手首のカメラと台(STL の表面の点。手首のカメラは既定であり)⇄ ピースの立方体・机・上のカメラの台(STL の三角形) / 持っているピース ⇄ ほかのピース・机・台
//   台: 既定は「あり」(上のカメラの置き方 C の柱・足の金具・吊る腕・その金具・吊る板・カメラ。2026-11-17 に A から替えた。2026-11-19 から C の台の設計の値で本家の板なし)。机(合板)の上の面 = 0、腕の原点は机から 2.4mm 上(台のあり・なしで同じ。2026-11-18 までは台ありで 9.6mm)。最後に台なしでも回す
//   コマの間: 腕の点が 1 コマで 4mm より動く所は、関節角を割った間も判定する(scene-data.js の SUB_STEP)
//   腕どうし(自分との干渉): self-collision.js(ちくわ排除機構の丸め箱)。隙間が負なら NG、WARN(3mm【仮】)未満なら要確認
//   見ていないもの: 手首のカメラのケーブル・ピースをつなぐ細い棒・点と点の間・物理・実機の角度の符号
//   ★負の検算: 指が入らないすき間なのに「入る」として解かせ、当たり判定が赤を出すかを見る
//     (赤が出なければ、判定が死んでいる)
const { load } = require("./_load");
load("kinematics.js", "so101.js", "motion.js", "puzzle.js",
     "models/so101_visuals.js", "models/so101_points.js", "models/so101_tris.js", "collide.js",
     "chikuwa-collision.js", "self-collision.js", "scene-data.js");
const G = SCENE_MODEL.gripFor(SCENE_MODEL.CELL_DEFAULT / 1000);
const mm = x => (x * 1000).toFixed(1);
console.log(`[既定のブロックの辺 ${SCENE_MODEL.CELL_DEFAULT}mm・つかむ高さ ${mm(SCENE_MODEL.GRASP_Y)}mm] 爪の寸法(形から測った・道具の点から): 固定の爪の面 ${mm(G.face)}mm / 指先 ${mm(G.tip)}mm / ` +
  `指の厚さ 固定 ${mm(G.thickFixed)}・動く ${mm(G.thickMove)}mm / 角度 0 のすき間 ${mm(G.gap0)}mm / ` +
  `使える開きの上限 ${G.topDeg.toFixed(1)}°・そのときのすき間 ${mm(G.gapMax)}mm / 指が入るすき間 ${mm(G.fitGap)}mm / 挟める幅 ${mm(G.maxSpan)}mm / 開く向き ${G.sign > 0 ? "角度が増えると開く" : "角度が減ると開く"}`);
const def = SCENE_MODEL.defaultUiValues();
const sets = SCENE_MODEL.controls.find(c => c.key === "SET").options;
let ng = 0, warn = 0;
function run(ui, label) {
  const t0 = Date.now();
  const d = SCENE_MODEL.buildFromUi(ui);
  const c = d._check;
  const hit = c.hitFrames ? `当たり ${c.hitFrames} 回(最初 ${c.firstHit.t.toFixed(1)}秒: ${c.firstHit.bad.join("、")})` : "当たりなし";
  console.log(`[${label}] ` + (c.solved
    ? `解けた 順番 ${c.order.join("→")} / 置いた位置の差 最大 ${(c.worstPlace * 1000).toFixed(3)}mm・向き ${c.worstYaw.toFixed(3)}° / ` +
      `掴んだ瞬間のずれ ${mm(c.worstPick)}mm・${c.worstPickYaw.toFixed(2)}° / 指先の最低 ${mm(c.minTip)}mm / いちばん近い ${mm(c.minClear)}mm(${c.minClearPair})${c.minClear >= 0 && c.minClear < c.marginWarn ? "★要確認" : ""}/ 接触の最深 ${mm(c.minContact)}mm(${c.minContactPair})/ ${hit} / ${c.T.toFixed(1)}秒 ${c.frames}コマ(間に足した判定 ${c.subChecks})` +
      (c.wrist ? ` / 手首のカメラ ${c.minCamPair ? mm(c.minCam) + "mm(" + c.minCamPair + ")" : "近くに相手なし(囲む球が 20mm より遠い)"}` : " / 手首のカメラなし") +
      (c.stand ? ` / 台 ${c.minStandPair ? mm(c.minStand) + "mm(" + c.minStandPair + ")" : "15mm より遠い"}` : ` / 台なし(腕の原点 ${mm(c.robotY)}mm)`) +
      (c.self ? ` / 腕どうし ${mm(c.self.min)}mm(${c.self.pair}・${c.self.t.toFixed(1)}秒)${c.self.min >= 0 && c.self.min < SELF_COLLIDE.WARN ? "★要確認" : ""}${c.self.worstBound < 0 ? "★間の下限 " + mm(c.self.worstBound) + "mm" : ""}` : " / 腕どうし 見ていない")
    : `解けない: ${c.why}`) + `  (${((Date.now() - t0) / 1000).toFixed(1)}s)`);
  if (c.reslot) console.log(`  置き場の選び直し ${c.reslot.tries} 回・${c.reslot.ok ? "直った" : "負のまま"}: ${c.reslot.log.join(" → ")}`);
  return c;
}
for (const o of sets) for (const gap of [0, def.GAP]) {
  const c = run({ SET: o.value, GAP: gap, SOL: 0 }, `${o.label} / すき間 ${gap}mm`);
  if (!c.solved) {          // 0 番で解けないとき、ほかの敷き詰め方(1〜9 番)も試す(点検 C6 の 1)
    const ok = [];
    for (let s2 = 1; s2 < 10; s2++) { const c2 = SCENE_MODEL.buildFromUi({ SET: o.value, GAP: gap, SOL: s2 })._check; if (c2.solved && !c2.hitFrames) ok.push(s2); }
    console.log(`  ほかの敷き詰め方: ${ok.length ? ok.join("・") + " 番で解けた(当たりなし)" : "1〜9 番も解けない(敷き詰め方は 10 通りで打ち切り)"}`);
  }
  if (c.solved && (c.worstPlace > 0.001 || c.worstYaw > 1 || c.worstPick > 0.001 || c.worstPickYaw > 1 || c.minTip <= 0 || c.hitFrames > 0 || !c.self || c.self.min < 0 || c.self.worstBound < 0)) {
    ng++; console.log("  ★NG: 置いた位置・掴んだ瞬間のずれ・指先の高さ・当たり・腕どうしのどれかが外れている");
  }
  if (c.solved && c.minClear >= 0 && c.minClear < c.marginWarn) warn++;
  if (c.solved && c.self && c.self.min >= 0 && c.self.min < SELF_COLLIDE.WARN) warn++;
}
// 台なし(腕は机にじかに。原点は土台の底のリブの分 2.4mm 上)
for (const o of sets) {
  const c = run({ SET: o.value, GAP: def.GAP, SOL: 0, STAND: 0 }, `台なし / ${o.label} / すき間 ${def.GAP}mm`);
  if (c.solved && (c.worstPlace > 0.001 || c.worstYaw > 1 || c.worstPick > 0.001 || c.worstPickYaw > 1 || c.minTip <= 0 || c.hitFrames > 0 || !c.self || c.self.min < 0 || c.self.worstBound < 0)) { ng++; console.log("  ★NG: 置いた位置・掴んだ瞬間のずれ・指先の高さ・当たり・腕どうしのどれかが外れている"); }
  if (c.solved && c.minClear >= 0 && c.minClear < c.marginWarn) warn++;
  if (c.solved && c.self && c.self.min >= 0 && c.self.min < SELF_COLLIDE.WARN) warn++;
}
// 1 つの関節だけで距離が決まる組(爪とカメラ台・手首とカメラ台。動きの判定から外した組)を、その関節の可動域の全体で 1° おきに見る(細かい箱 1.5mm)。
//   箱は両側で最大 3.6mm 太いので、負でも当たるとは限らない。0 未満は「要確認」(実物で確かめる)として数える。1° の間は見ていない
SELF_COLLIDE.staticCheck().forEach(r => {
  console.log(`[腕どうし・1 つの関節で決まる組] ${r.pair}: 関節 ${r.joint} の ${r.range.map(v => v.toFixed(0)).join("〜")}° で いちばん近い ${mm(r.clearance)}mm(${r.at}°)${r.clearance < 0 ? " ★要確認(箱の太りの内か、本当に触れるか、この道具では分けられない)" : ""}`);
  if (r.clearance < 0) warn++;
});
// 台(上のカメラの置き方 C: 柱・吊る腕・カメラ)の負の検算。C の柱は腕の右奥(肩から約 350mm)に立ち、ふつうの置き場や動きからは遠い。
//   そこで台を机の上でずらして(STAND_SHIFT)作業域に入れ、当たりが出ることを見る(2026-11-17 に C へ替えたとき作り直した。A の柱の検算の置き換え)
const STAND_ID = /(st_|c_)/;
// 手首のカメラの負の検算: 台を [-60, -60]mm ずらし、置き場も避けさせない → 手首のカメラが柱に入るはず(実測 cam|c_pole -8.0mm。2026-11-19 の C の台の値で -5.0mm)
const ref = run({ SET: 2, GAP: def.GAP, SOL: 0, NO_AVOID: true, NO_RESLOT: true, STAND_SHIFT: [-60, -60] }, "手首のカメラの負の検算: 3×4・台を作業域へずらす");
if (!(ref.minCam < 0 && /^cam\|(st_|c_)/.test(ref.minCamPair))) { ng++; console.log("  ★NG: 台を作業域に入れたのに、手首のカメラが台に当たらない(手首のカメラか台が判定に入っていない)"); }
else console.log(`  OK: 台を作業域に入れると、手首のカメラが台に入る(${ref.minCamPair} ${mm(ref.minCam)}mm)`);
// 台の負の検算(動きの中): 台を [-180, 30]mm ずらす → 腕の部品(動く側)が台に入るはず(実測 j3|c_pole -12.0mm。点検 C10 の 2。2026-11-19 の C の台の値で jaw|c_foot -10.0mm)
const negB = run({ SET: 2, GAP: def.GAP, SOL: 0, NO_AVOID: true, NO_RESLOT: true, STAND_SHIFT: [-180, 30] }, "台の負の検算: 3×4・台を作業域へずらす");
if (!(negB.minClear < 0 && /^(j[1-5]|jaw)\|(st_|c_)/.test(negB.minClearPair))) {
  ng++; console.log("  ★NG: 台を動きの中に置いたのに、腕と台の当たりが出ない(動きの判定が台を見ていない)"); }
else console.log(`  OK: 動きの中で腕が台に入る(${negB.minClearPair} ${mm(negB.minClear)}mm)`);
// (台を作業域に入れる負の検算は、選び直しても台が動かないので、置き場の選び直し(RESLOT)を止める。時間だけかかる)
// コマの間の負の検算: 前の写す姿勢 [-90, 0, 90, 90, 0] と待ちの姿勢をまっすぐ結び、台を [-283.6, -152.4]mm ずらす(柱の中心が (69.7, −132.9)mm に来る)
//   → 手首のカメラが柱をかすめるのはコマの間だけ(コマの上だけで測ると 0 以上)で、コマの間に足した判定で負になるはず
//   (実測: コマの上 +1.5mm、コマの間 -5.0mm。コマだけで測っていたときに見逃した型。2026-11-16 に A の柱で見つけ、C で作り直した。
//    2026-11-19 に C の台の値(柱 (353.3, 19.5) の角 20×20・腕が 7.2mm 低い)へ替えて、ずらす量を柱の中心が同じ所に来るよう直した: コマの上 +3.0mm、コマの間 -2.5mm。
//    この前の写す姿勢へまっすぐ行く道は、腕が低くなったので爪が机に -2.3mm 入る。机は見ないよう、台との近さだけ(minStand・minStandFrame)で見る)
const OLD_PHOTO = [-90, 0, 90, 90, 0];
const negC = run({ SET: 2, GAP: def.GAP, SOL: 0, PHOTO_DIRECT: true, _PHOTO_Q: OLD_PHOTO, NO_RESLOT: true, STAND_SHIFT: [-283.6, -152.4] }, "コマの間の負の検算: 3×4・前の写す姿勢へまっすぐ・台をずらす");
if (!(negC.minStand < 0 && /^(j[1-5]|jaw|cam)\|(st_|c_)/.test(negC.minStandPair) && negC.minStandFrame >= 0)) {   // 腕の部品だけ(置いたままのピースと台は restCheck で minStand に入り、コマの間と関係ない。点検 C40 の 7)
  ng++; console.log(`  ★NG: コマの間だけで当たる形にならない(コマの間の判定が見ていないか、コマの刻みが変わった。台とのコマの上 ${mm(negC.minStandFrame)}mm・全体 ${mm(negC.minStand)}mm ${negC.minStandPair})`); }
else console.log(`  OK: 台とはコマの上では ${mm(negC.minStandFrame)}mm でも、コマの間で ${negC.minStandPair} ${mm(negC.minStand)}mm(コマの間を測っている)`);
// 腕どうしの負の検算: 前の写す姿勢(腕を畳みすぎ)では、隣でない組(肩と手首など)がめり込むはず(利用者が画面で見つけた。2026-11-17)
const negS = run({ SET: 2, GAP: def.GAP, SOL: 0, _PHOTO_Q: OLD_PHOTO, _OLD_VIA: true, NO_RESLOT: true }, "腕どうしの負の検算: 3×4・前の写す姿勢と経由点");
if (!(negS.self && negS.self.min < 0 && !SELF_COLLIDE.ADJ[negS.self.pair])) { ng++; console.log("  ★NG: 前の写す姿勢なのに、腕どうしのめり込みが出ない(腕どうしの判定が見ていない)"); }
else console.log(`  OK: 前の写す姿勢では腕どうしがめり込む(${negS.self.pair} ${mm(negS.self.min)}mm・${negS.self.t.toFixed(1)} 秒)`);
// 台の負の検算(姿勢だけ): 肩を柱の向きへ回して腕を伸ばすと、手首と爪が C の柱に入るはず。肩 −60° は 15mm より遠いはず
//   (柱の向き = 肩の軸 x 38.8mm から柱の中心へ。2026-11-19 の C の台の値で 3.5°・j5|c_pole -9.7mm。それまでは 10° と書いていた)
const PAN_TO_POLE = Math.atan2(SO101.OVERHEAD_C.poleXZMm[1], SO101.OVERHEAD_C.poleXZMm[0] - SO101.axes[0].origin[0] * 1000) * 180 / Math.PI;
const sIn = SCENE_MODEL.standClearance([PAN_TO_POLE, 0, 0, 0, 0], 0), sOut = SCENE_MODEL.standClearance([-60, 0, 0, 0, 0], 0);
const pairs = r => Object.keys(r.pairs).map(k => `${k} ${mm(r.pairs[k])}mm`).join("、") || "15mm 以内に無い";
console.log(`[台の負の検算] 肩 ${PAN_TO_POLE.toFixed(1)}°(腕を柱へ伸ばす): ${pairs(sIn)} / 肩 −60°: ${pairs(sOut)}`);
const sInMin = Math.min(...Object.values(sIn.pairs)), sOutMin = Math.min(Infinity, ...Object.values(sOut.pairs).filter(v => v < 0.004));
if (!(sInMin < 0) || isFinite(sOutMin)) { ng++; console.log("  ★NG: 柱に入れた姿勢で負にならない、または柱から離した姿勢で台に当たる(台の判定が見ていない)"); }
else console.log("  OK: 柱に入れると負になり、離すと当たらない(台の判定は生きている)");
// 負の検算: すき間 4mm(指の厚さより狭い)で「指が入る」として解かせる → 当たりが出るはず
const neg = run({ SET: 2, GAP: 4, SOL: 0, FORCE_FIT: true }, "負の検算: 3×4・すき間 4mm を無理に詰める");
if (!neg.solved || neg.hitFrames === 0) { ng++; console.log("  ★NG: 無理に詰めたのに当たりが出ない(判定が見ていない)か、動きが作れなかった"); }
else console.log("  OK: 無理に詰めると当たりが出る(判定は生きている)");
// 置き場の選び直し(scene-data.js の RESLOT): T 字・O L J S・すき間 0・台なしは、選び直さないと J をつかみに下りる所で腕どうしが負(負の検算)、
//   選び直すと 0 以上になるはず(2026-11-17。−2.7mm → +1.8mm)
{
  const T_OLJS = { label: "T 字・O L J S", art: ["#####", "#####", ".###.", ".###."], pieces: "O L J S", solver: "x" };
  const off = run({ _SET_OVERRIDE: T_OLJS, GAP: 0, SOL: 0, STAND: 0, NO_RESLOT: true }, "置き場の選び直しの負の検算: T 字・O L J S・すき間 0・台なし・選び直さない");
  const on = run({ _SET_OVERRIDE: T_OLJS, GAP: 0, SOL: 0, STAND: 0 }, "置き場の選び直し: 同じ条件で選び直す");
  const offBad = off.solved && off.self && off.self.min < 0,
        onOk = on.solved && on.reslot && on.reslot.ok && on.self.min >= 0 && on.self.worstBound >= 0 && !on.hitFrames;
  if (!offBad || !onOk) { ng++; console.log("  ★NG: 選び直さないと負・選び直すと 0 以上(ほかの当たりも無し)、のどちらかが外れた"); }
  else console.log("  OK: 選び直さないと負になり、選び直すと 0 以上になる(選び直しは効いている)");
  if (onOk && on.self.min < SELF_COLLIDE.WARN) warn++;
  // 置き場では直らない例: NG のまま返し、むだに組み直し続けないことを見る。止まる枝は 2 つあり、両方を見る(点検 C40 の 3)。
  //   (1) 0.1mm 以上よくならなければ止める: T 字・O L J S・すき間 0 で、腕の原点を本家の板ありの 9.6mm にする(_ROBOT_Y_MM。台なしで)。
  //       J を外した後に L を盤に置いた所の j1|cam が残る(盤の位置で決まる)。2026-11-19 までは「台あり」でこの例だった(実測 −2.4 → −1.9mm で 2 回で止まる)。
  //       C の台(板なし)では台ありも 2.4mm になり、同じ組は直る(+1.8mm)ので、腕の高さだけを前に戻して同じ例を作る
  //   (2) どのピースの 1 手でもない所で負: 前の写す姿勢と経由点(上の腕どうしの負の検算と同じ)は、負になるのが写す姿勢との行き来(24.0 秒 base|jaw) → 置き場を外さずに 0 回で止まる
  const stOld = run({ _SET_OVERRIDE: T_OLJS, GAP: 0, SOL: 0, STAND: 1 }, "置き場の選び直し: T 字・O L J S・すき間 0・台あり(C の台の値で直るようになった例)");
  console.log(`  (参考)台ありの O L J S: ${stOld.reslot ? (stOld.reslot.ok ? "直った" : "直らない") + "・" + stOld.reslot.tries + " 回" : "選び直し無し"}`);
  const st = run({ _SET_OVERRIDE: T_OLJS, GAP: 0, SOL: 0, STAND: 0, _ROBOT_Y_MM: 9.6 }, "置き場の選び直し(直らない例 1): T 字・O L J S・すき間 0・腕の原点 9.6mm");
  const stOk = st.solved && st.reslot && !st.reslot.ok && st.reslot.tries >= 1 && st.reslot.tries <= 2 && /止めた/.test(st.reslot.log.join(" "));
  if (!stOk) { ng++; console.log("  ★NG: 直らない例 1 で、置き場を外して試し、0.1mm 以上よくならずに 2 回以内に止まる、が外れた"); }
  else console.log(`  OK: 直らない例 1 は NG のまま ${st.reslot.tries} 回で止まる(0.1mm 以上よくならない)`);
  const st2 = run({ SET: 2, GAP: def.GAP, SOL: 0, _PHOTO_Q: OLD_PHOTO, _OLD_VIA: true }, "置き場の選び直し(直らない例 2): 3×4・前の写す姿勢と経由点・選び直す");
  const st2Ok = st2.solved && st2.reslot && !st2.reslot.ok && st2.reslot.tries === 0 && /どのピースの 1 手でもない/.test(st2.reslot.log.join(" "));
  if (!st2Ok) { ng++; console.log("  ★NG: 直らない例 2 で、どのピースの 1 手でもないとして 0 回で止まる、が外れた"); }
  else console.log("  OK: 直らない例 2 は、どのピースの 1 手でもないので置き場を外さずに止まる");
}
// 写す姿勢への寄り道(scene-data.js の RELOOK。2026-11-18。隣の会話の live.html が撮り直しに使う): 既定の 3×4 で、
//   2 つ目からのピースの前に写す姿勢で 0.3 秒止まる(tLook0〜tLook1。次のピースの上に着く前)・当たりなし・腕どうし 0 以上・全体の時間が伸びるはず
//   (止まっている間の関節が写す姿勢そのものかは見ていない。写す姿勢で隠れないことは tools/overhead-occlusion.js)
{
  const R = run({ SET: 2, GAP: def.GAP, SOL: 0, RELOOK: true }, "写す姿勢への寄り道: 3×4・RELOOK");
  const R0 = run({ SET: 2, GAP: def.GAP, SOL: 0 }, "写す姿勢への寄り道の比べ: 3×4・寄り道なし");
  const looks = R.stepTimes.filter(s => s.tLook1 > 0);
  const ok = R.solved && R.hitFrames === 0 && R.self && R.self.min >= 0 && looks.length === R.stepTimes.length - 1 && R.stepTimes[0].tLook1 === 0 &&
    looks.every(s => s.tLook1 - s.tLook0 > 0.29 && s.tBack >= s.tLook1 && s.tBack < s.tHover) && R.T > R0.T && R.relook && R.relook.n === looks.length;
  if (!ok) { ng++; console.log("  ★NG: 寄り道が 2 つ目からのピースの前に入る・当たりなし・全体が伸びる、が外れた"); }
  else console.log(`  OK: 寄り道 ${looks.length} 回(写す姿勢で ${looks.map(s => s.tLook0.toFixed(2) + "〜" + s.tLook1.toFixed(2)).join("・")} 秒)・全体 ${R0.T.toFixed(1)} → ${R.T.toFixed(1)} 秒(寄り道に使った ${R.relook.detour.toFixed(1)} 秒)`);
  if (ok && R.self.min < SELF_COLLIDE.WARN) warn++;
}
// 180° 回して同じ形のピースの見直し(scene-data.js の views。2026-11-18): 画像だけで動かす試し(vision-loop.js の種 1・seed 1)で、
//   絵から見つけた Z が「向き 185.6°」(180° 回った見方)で返り、そのままでは手首の回転の可動域で組めなかった置き場。
//   見直さない(NO_SYM)と解けず(負の検算)、見直すと解けて当たりがなく、Z の立方体は机の上の同じ所のまま(見直しは絵と動きの見方だけ)のはず
{
  const DET = { pieces: [{ x: 149.43, z: -97.41, yaw: 352.32 }, { x: 180.86, z: 90.44, yaw: 343.3 }, { x: 81.86, z: 106.26, yaw: 5.23 },
                         { x: 128.77, z: -152.68, yaw: 358.92 }, { x: 88.24, z: -107.06, yaw: 185.64 }] };
  const off = run({ SET: 1, GAP: def.GAP, SOL: 0, LAYOUT: DET, NO_SYM: true }, "180° の見直しの負の検算: 4×5・I O L J Z・絵から見つけた置き場・見直さない");
  const on = run({ SET: 1, GAP: def.GAP, SOL: 0, LAYOUT: DET }, "180° の見直し: 同じ置き場で見直す");
  const zi = 4, cellsIn = PUZZLE.SHAPES.Z.map(c => { const a = DET.pieces[zi].yaw * Math.PI / 180, x = c[0] * on.pitch, z = c[1] * on.pitch;
    return [DET.pieces[zi].x / 1000 + x * Math.cos(a) - z * Math.sin(a), DET.pieces[zi].z / 1000 + x * Math.sin(a) + z * Math.cos(a)]; });
  const same = on.pieceCells && cellsIn.every(p => on.pieceCells[zi].some(q => Math.hypot(p[0] - q[0], p[1] - q[1]) < 1e-5));
  const ok = !off.solved && on.solved && on.hitFrames === 0 && on.worstPlace < 0.001 && (on.symUsed || []).some(x => x.name === "Z") && same;
  if (!ok) { ng++; console.log(`  ★NG: 見直さないと解けず、見直すと解ける、が外れた(見直さない: ${off.solved ? "解けた" : "解けない"} / 見直す: ${on.solved ? "解けた・見直したピース " + (on.symUsed || []).map(x => x.name).join("・") + "・立方体が同じ所 " + same : on.why})`); }
  else console.log(`  OK: 見直さないと解けず、見直すと解ける(見直したピース ${on.symUsed.map(x => x.name).join("・")}。Z の立方体は同じ所)`);
  if (ok && on.self && on.self.min < SELF_COLLIDE.WARN) warn++;
}
if (warn) console.log(`要確認 ${warn} 行: いちばん近い所が 2.5mm 未満(判定の甘さ: 点の間 最大 2mm ほど + コマの割り方 約 0.6mm より小さい余裕。計画書の S-8 で判定を細かくするまで、当たらないとは言い切れない)`);
console.log(ng ? `NG ${ng} 件` : "OK(解けたものは、置いた位置の差 1mm 未満・向き 1° 未満・掴んだ瞬間のずれ 1mm 未満・1° 未満・指先が机より上・当たりなし。腕どうしの隙間が 0 以上。負の検算 6 つは当たりあり。置き場の選び直しは効く)");
process.exit(ng ? 1 : 0);
