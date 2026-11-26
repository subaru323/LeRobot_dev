/* =============================================================================
 * so101.js — SO-101(フォロワー)の関節の寸法と、そこから作る関節軸
 *
 * ■ 何のためにあるか
 *   絵(scene-data.js)と逆運動学(kinematics.js の KIN)が★同じ値を読むための、
 *   ただ 1 つの持ち主。寸法をほかのファイルに写さない(写すと、ずれても
 *   どちらも単体では正しく見える = ちくわの 0-E)。
 *
 * ■ 値の出どころ
 *   TheRobotStudio/SO-ARM100 の Simulation/SO101/so101_new_calib.urdf
 *   (onshape-to-robot が生成。2026-11-12 に読んだ)。手首のカメラの 2 つは
 *   so101_new_calib_camera.urdf(2026-11-16 に取得。上の URDF に 2 リンクを足しただけ)。
 *   ★取得の経路: ウェブの取得の道具の「要約」経由で数字を写した(生のファイルではない)。
 *     数字は全桁を写させたが、生のファイルとの突き合わせはまだ(計画書の前提 U-1)。
 *   ★"new_calib" は LeRobot の新しいキャリブレーション(各関節の真ん中 = 0)に
 *     合わせた版。実機の角度の向き(符号)・0 の位置との対応はまだ確かめていない(前提 U-2)。
 *
 * ■ 座標
 *   URDF は Z が上・単位 m。画面(three.js)は Y が上・単位 m。
 *   画面 = (x_urdf, z_urdf, -y_urdf)(X 軸まわりに -90°。右手系のまま)。
 *   base_link の原点を画面の原点に置く。机の上面は原点ではない: 土台の底のリブが原点より 2.4mm 下に出て、
 *   上のカメラの台 C は腕を合板に直に置くので、合板(机)は原点の 2.4mm 下(overheadTableY)。パズルの画面(scene-data.js)は机を 0 にして腕を上げる。
 *
 * ■ 関節軸の形(KIN の形)
 *   全関節 0 の姿勢での「関節軸の向き」と「軸上の点」を画面の座標で持つ。
 *   KIN.fk は親から順にその軸まわりに回すだけ(ちくわの CRX と同じ作り)。
 * ========================================================================== */

var SO101 = (function () {
  "use strict";
  var DEG = Math.PI / 180;

  // ---- URDF の値(写し間違いを避けるため、桁はそのまま) ----
  //  origin: [x, y, z], rpy: [r, p, y](rad), axis は全関節とも [0,0,1]
  var URDF = {
    shoulder_pan:  { parent: "base_link",      child: "shoulder_link",
                     xyz: [0.0388353, -8.97657e-09, 0.0624],
                     rpy: [3.14159, 4.18253e-17, -3.14159], lim: [-1.91986, 1.91986] },
    shoulder_lift: { parent: "shoulder_link",  child: "upper_arm_link",
                     xyz: [-0.0303992, -0.0182778, -0.0542],
                     rpy: [-1.5708, -1.5708, 0], lim: [-1.74533, 1.74533] },
    elbow_flex:    { parent: "upper_arm_link", child: "lower_arm_link",
                     xyz: [-0.11257, -0.028, 1.73763e-16],
                     rpy: [-3.63608e-16, 8.74301e-16, 1.5708], lim: [-1.69, 1.69] },
    wrist_flex:    { parent: "lower_arm_link", child: "wrist_link",
                     xyz: [-0.1349, 0.0052, 3.62355e-17],
                     rpy: [4.02456e-15, 8.67362e-16, -1.5708], lim: [-1.65806, 1.65806] },
    wrist_roll:    { parent: "wrist_link",     child: "gripper_link",
                     xyz: [5.55112e-17, -0.0611, 0.0181],
                     rpy: [1.5708, 0.0486795, 3.14159], lim: [-2.74385, 2.84121] },
    // 動く爪(枝分かれ。腕の連鎖には入れない)
    gripper:       { parent: "gripper_link",   child: "moving_jaw_so101_v1_link",
                     xyz: [0.0202, 0.0188, -0.0234],
                     rpy: [1.5708, -5.24284e-08, -1.41553e-15], lim: [-0.174533, 1.74533] },
    // 道具の点(固定)。つかむ点の目安
    gripper_frame: { parent: "gripper_link",   child: "gripper_frame_link",
                     xyz: [-0.0079, -0.000218121, -0.0981274],
                     rpy: [0, 3.14159, 0] },
    // 手首のカメラ(固定)。公式の so101_new_calib_camera.urdf(2026-11-16 取得)の値。
    // 台は Optional/SO101_Wrist_Cam_Hex-Nut_Mount_32x32_UVC_Module と同じ STL(バイト一致を確かめた)
    wrist_camera_mount: { parent: "gripper_link", child: "wrist_camera_mount_link",
                     xyz: [-0.0150, 0.0240, -0.0315],
                     rpy: [-1.5708, -0.0008, -1.5708] },
    wrist_camera:  { parent: "wrist_camera_mount_link", child: "wrist_camera_link",
                     xyz: [-0.04941, -0.03813, 0.0175],
                     rpy: [0, -1.5708, -2.0076] }
  };
  var ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"];

  // ---- 4x4 を {R:3x3 行優先, p:[3]} で持つ ----
  function mm(A, B) {
    var C = new Array(9);
    for (var r = 0; r < 3; r++) for (var c = 0; c < 3; c++)
      C[r * 3 + c] = A[r * 3] * B[c] + A[r * 3 + 1] * B[3 + c] + A[r * 3 + 2] * B[6 + c];
    return C;
  }
  function mv(A, v) {
    return [A[0] * v[0] + A[1] * v[1] + A[2] * v[2],
            A[3] * v[0] + A[4] * v[1] + A[5] * v[2],
            A[6] * v[0] + A[7] * v[1] + A[8] * v[2]];
  }
  function rx(a) { var c = Math.cos(a), s = Math.sin(a); return [1, 0, 0, 0, c, -s, 0, s, c]; }
  function ry(a) { var c = Math.cos(a), s = Math.sin(a); return [c, 0, s, 0, 1, 0, -s, 0, c]; }
  function rz(a) { var c = Math.cos(a), s = Math.sin(a); return [c, -s, 0, s, c, 0, 0, 0, 1]; }
  // URDF の rpy: R = Rz(yaw) * Ry(pitch) * Rx(roll)
  function rpy(v) { return mm(rz(v[2]), mm(ry(v[1]), rx(v[0]))); }
  function compose(T, xyz, R) {      // T * [R | xyz]
    var p = mv(T.R, xyz);
    return { R: mm(T.R, R), p: [T.p[0] + p[0], T.p[1] + p[1], T.p[2] + p[2]] };
  }
  // URDF(Z 上) → 画面(Y 上)
  function toView(v) { return [v[0], v[2], -v[1]]; }
  var C = [1, 0, 0, 0, 0, 1, 0, -1, 0];               // toView の行列
  // 局所の座標系はそのまま(局所 → 画面 = C·R)。C·R·Cᵗ にすると局所の軸まで入れ替わる(一度やった)
  function frameToView(T) { return { R: mm(C, T.R), p: toView(T.p) }; }

  /**
   * URDF の行列の積で、そのまま順運動学を解く(KIN とは独立の作り)。
   *   qRad … 腕 5 関節 [rad]、jawRad … 爪 [rad]
   * 戻り値: 各リンクの座標系(画面の座標)。検算(tools/check-fk.js)と絵の確認に使う。
   */
  function urdfFK(qRad, jawRad) {
    var T = { R: [1, 0, 0, 0, 1, 0, 0, 0, 1], p: [0, 0, 0] };
    var out = { base_link: frameToView(T) };
    for (var i = 0; i < ARM.length; i++) {
      var j = URDF[ARM[i]];
      T = compose(T, j.xyz, mm(rpy(j.rpy), rz(qRad[i] || 0)));
      out[j.child] = frameToView(T);
    }
    var G = URDF.gripper;
    out.moving_jaw = frameToView(compose(T, G.xyz, mm(rpy(G.rpy), rz(jawRad || 0))));
    var F = URDF.gripper_frame;
    out.gripper_frame = frameToView(compose(T, F.xyz, rpy(F.rpy)));
    var CM = URDF.wrist_camera_mount, CA = URDF.wrist_camera;
    var Tm = compose(T, CM.xyz, rpy(CM.rpy));
    out.wrist_camera_mount_link = frameToView(Tm);
    out.wrist_camera_link = frameToView(compose(Tm, CA.xyz, rpy(CA.rpy)));
    return out;
  }

  // ---- 全関節 0 の姿勢での関節軸(KIN の形)を作る ----
  var joints = [];
  (function () {
    var T = { R: [1, 0, 0, 0, 1, 0, 0, 0, 1], p: [0, 0, 0] };
    for (var i = 0; i < ARM.length; i++) {
      var j = URDF[ARM[i]];
      T = compose(T, j.xyz, rpy(j.rpy));            // 関節の座標系(回す前)
      var V = frameToView(T);
      joints.push({
        name: ARM[i],
        axis: [V.R[2], V.R[5], V.R[8]],             // その座標系の z 軸 = 回転軸
        origin: V.p.slice(),
        limitsDeg: [j.lim[0] / DEG, j.lim[1] / DEG]
      });
    }
    var G = URDF.gripper, F = URDF.gripper_frame;
    var Vj = frameToView(compose(T, G.xyz, rpy(G.rpy)));
    joints.jaw = {
      name: "gripper", axis: [Vj.R[2], Vj.R[5], Vj.R[8]], origin: Vj.p.slice(),
      limitsDeg: [G.lim[0] / DEG, G.lim[1] / DEG]
    };
    var Vf = frameToView(compose(T, F.xyz, rpy(F.rpy)));
    var Vg = frameToView(T);                        // gripper_link(爪の付け根)
    joints.tool = {
      pos: Vf.p.slice(),                            // 道具の点(gripper_frame の原点)
      dir: [Vf.R[2], Vf.R[5], Vf.R[8]],             // 指先の向き(gripper_frame の z)
      up:  [Vg.R[0], Vg.R[3], Vg.R[6]]              // 爪が開く向きの目安(gripper_link の x)
    };
  })();

  // ---- 形(STL)の置き場所 ----
  // リンク → 画面の入れ子(scene-data の部品の名前)
  var LINK_GROUP = { base_link: "base", shoulder_link: "j1", upper_arm_link: "j2", lower_arm_link: "j3",
                     wrist_link: "j4", gripper_link: "j5", moving_jaw_so101_v1_link: "jaw" };
  // 手首のカメラの部品(爪の付け根 j5 と一緒に動く)。パズルの画面(scene-data.js)はまだ使わない
  // (当たり判定に入れるのは計画書_完成形まで の S-4)。手で動かす画面(jog.html)が使う
  var CAMERA_GROUP = { wrist_camera_mount_link: "j5", wrist_camera_link: "j5" };
  var Z0 = null;
  /** 全関節 0 の姿勢で、リンク link に置く形(URDF の <visual> の xyz・rpy)の 画面の R・p */
  function visualPose(link, xyz, rpyv) {
    if (!Z0) { Z0 = urdfFK([0, 0, 0, 0, 0], 0); Z0.moving_jaw_so101_v1_link = Z0.moving_jaw; }
    var F = Z0[link];
    var p = mv(F.R, xyz);
    return { R: mm(F.R, rpy(rpyv)), p: [F.p[0] + p[0], F.p[1] + p[1], F.p[2] + p[2]] };
  }
  // ---- 上のカメラの台(本家 Optional/Overhead_Cam_Mount_32x32_UVC_Module。URDF の外) ----
  //  4 個の STL は単位 mm・Y が上。どれも回さず、ずらすだけで組める(2026-11-16 に形から測った)。
  //  ★下の柱(bottom)の座標を「台の座標」とする。数字の出どころ:
  //   - 中・上(middle・top)は同じ座標で組んである(中の上の舌 Y 190〜198 が上の受けに入る)。
  //     下の柱の上の舌(Y 223.1〜231、柱の断面 25.4×36.6 の中心 X 18.7・Z 36.5)が、中の下の受け
  //     (Y 35〜43、断面の中心 0, 0)に入る → 中・上を (18.7, 223.1 − 35.0, 36.5) ずらす。
  //     舌は前後に対称なので 180° 回した組み方もできる。カメラが腕の前(+X)へ曲がる向きを採った(写真と合う)。
  //   - 腕の板(arm_base)は、端の 3 本の舌(幅 6.7〜7.2・すき間 8.4)が、下の柱の足の 2 本の舌(幅 8.2・すき間 7.4)と
  //     かみ合う。X は 93.9 ずらすと合う。板の −Z 側の端(舌の先 −133)を足の +Z 側(本体の端 73)に当てる
  //     → Z を 206 ずらす。厚さはどちらも 7.2(板は Y −5〜2.2、足は 0〜7.2)→ Y を 5 ずらす。
  //     板の両端に舌があるので、腕を足の反対側(−Z)に付けることもできる。本家の写真(腕 1 本)に合わせ、柱が腕の左に来る側を採った。
  //   - 腕は、板の上の面(Y 7.2)に、板の真ん中に置いた。土台(base_so101_v2)の底のリブは base_link の原点より
  //     2.4mm 下まで出る(URDF の見た目の位置 z −0.0024。点検 C9 の 1 で分かった)ので、原点は板の上の面より 2.4mm 上(Y 9.6)。
  //     板の X の長さ 89.6 ⇄ 土台の足跡 87(前ほど細い形が板の形と合う)。板の Z の真ん中 152.55 ⇄ 足跡の真ん中 0.2。
  //     【仮】板にくぼみ(位置決め)は無いので、実物では ±2mm ほどずれうる。
  //   - カメラの板(上の部品の先): 厚さ 3.0mm、下の面の法線 (0.4226, −0.9063, 0) = 真下から腕の前へ 25.0° 傾く。
  //     ネジ穴 4 つが板の真ん中の ±13.5mm にあるので、レンズの軸は上の面の真ん中 (28.24, 364.96, 0)(上の部品の座標)を通る。
  //     【仮】カメラの基板は板の上の面に付き、レンズが穴を通って下へ出る(本家の手順「レンズを穴に押し込む」)。
  //     カメラの形は手首と同じ 32×32 の公式の STL を使う(上のカメラの推奨品の形は確かめていない)。
  //     【仮】絵の上 = 板が上がっていく向き(腕の前の奥)。画角は分からない(品の資料で確かめる)。
  var OVERHEAD = {
    partOffsetMm: { overhead_cam_mount_bottom: [0, 0, 0],
                    overhead_cam_mount_middle: [18.7, 188.1, 36.5],
                    overhead_cam_mount_top:    [18.7, 188.1, 36.5],
                    overhead_arm_base:         [93.9, 5.0, 206.0] },
    robotOriginMm: [23.7, 9.6, 152.35],     // 腕の base_link の原点の、台の座標での位置(板の上の面 7.2 + 土台の底のリブ 2.4)
    plateMm: 7.2,                           // 板の厚さ = 腕が机から上がる量
    baseBelowOriginMm: 2.4,                 // 土台の底のリブが base_link の原点より下に出る量
    camFaceMm: [28.24, 364.96, 0],          // カメラの板の上の面の真ん中(上の部品の座標)
    camDir: [0.4226, -0.9063, 0],           // レンズの向き(台の座標 = 画面の座標と同じ向き)
    camUp: [0.9063, 0.4226, 0],             // 絵の上【仮】
    lensTipMm: 26.3                         // 基板の前の面からレンズの先まで(公式のカメラの STL)
  };
  /** 上のカメラの台(置き方 C)を使うときの机(合板)の上の面の高さ(m、base_link の原点が 0)。
   *  2026-11-19 から C の台の設計(本家の腕の板なし)で −2.4mm。それまでは本家の板を敷いて −9.6mm */
  function overheadTableY() { return -(OVERHEAD_C.plateMm + OVERHEAD.baseBelowOriginMm) / 1000; }
  /** 台の部品 name を置く画面の位置(m、base_link の原点が 0)。回転は無い */
  function overheadPartPos(name) {
    var o = OVERHEAD.partOffsetMm[name], r = OVERHEAD.robotOriginMm;
    return [(o[0] - r[0]) / 1000, (o[1] - r[1]) / 1000, (o[2] - r[2]) / 1000];
  }
  /** カメラの縦の画角 [度]【仮】(品が決まっていない)。2026-11-17 に 1 か所にまとめた(09 の §6-2)。
   *  ここを読む所: scene-data.js(視点の一覧)・vision.js(render と detect の既定。手首の絵は呼ぶ側が wrist を渡す)・index.html(?_cam=top)・
   *  jog.html(スライダーの初期値。手首はそれまで 70 だった)・tools/overhead-truth.js・overhead-occlusion.js・vision-loop.js。
   *  まだ自前で 60 を持つ所: live.js・tools/k02-compare.js(別の会話の持ち物。知らせた) */
  var CAM_FOV = { top: 60, wrist: 60 };
  // ---- 上のカメラの置き方 C(利用者 2026-11-17「Go with C」。docs/Simulation_Logic_Documents/08 の §3-2・現在の課題 K-12) ----
  //  腕から離した別の柱に吊る腕を付け、置き場と盤の真上から真下を向ける。本家の台の柱(bottom・middle・top)は使わない。
  //  2026-11-19 から hardware/C_台の設計.md・hardware/c_mount.py の値(点検 C39 の後の案。何も買っておらず組んでいない)。
  //  腕は本家の板を使わず、合板(12mm)の上に直に置く: 腕の原点は合板の上の面から 2.4mm 上(土台の底のリブ)【仮】。
  //  座標は「合板の上の面が高さ 0、腕の土台(base_link の原点)の真下が x = z = 0」[mm]
  //  (腕の座標(base_link の原点が 0)では、高さが 2.4mm 低い。overheadRig・overheadCamera が直す)。
  //  2026-11-17〜11-19 は tools/k02-compare.js の【仮】(柱 (349.7, 51.1) の丸 r12・上の端 570、本家の板ありで原点 9.6mm)。
  //   - レンズの先: 置き場と盤の角の範囲の真ん中の真上、合板から 540mm(板を外したぶん、腕の原点からは 530.4 → 537.6mm)
  //   - 柱(縦のアルミの角材 20×20): 中心 (353.3, 19.5)、肩の軸から 315.1mm(前の値と同じ距離で、カメラと同じ z に動かした)。合板から 575.3mm
  //   - 吊る腕(横の 20×20): 柱の上に載せる。y 575.3〜595.3、x 117.3〜398.3(まっすぐ −x 向き)
  //   - カメラを吊る板: 基板の前の面の下(y 563.3〜566.3)、耳を含めて ±30mm【仮】。足で吊る腕の下の溝に留める → 箱 y 563.3〜575.3 にまとめた(足のすき間も埋めた厳しい側)
  //   - カメラ: レンズの先 540 から基板の裏(前の面 566.3 + 部品 6mm【仮】)まで、32×32 の箱(レンズの胴も基板の幅にした厳しい側)
  //   - 柱の足の金具(角の金具 20mm を 4 面に): 柱の中心 ±30mm・高さ 20mm の箱(金具の間も埋めた厳しい側)
  //   - 吊る腕を柱に留める角の金具 2 個(柱の ±x の面の外へ 20mm、吊る腕の下 20mm): x 323.3〜383.3・y 555.3〜575.3 の箱(柱と重ねた 1 つの箱)
  //  当たりの形は全部、軸にそろった箱(lo・hi)。柱・吊る腕は形そのもの、カメラ・吊る板・足の金具・上の金具は形より大きめの包み。
  //  入れていない物: ケーブル(c_mount.py の CABLE_R 5mm【仮】)。腕から離す余裕(c_mount.py の入ってはいけない範囲の 20mm)も入れない(当たり判定は形で見る)
  var OVERHEAD_C = {
    lensTipMm: [157.3, 540, 19.5],
    camDir: [0, -1, 0],                     // 真下
    camUp: [1, 0, 0],                       // 絵の上 = 腕の前(+x)。絵の長い辺は置き場の長い向き(z)
    plateMm: 0,                             // 腕の下に敷く板(本家の板は使わない)
    poleXZMm: [353.3, 19.5],                // 柱の中心(表示・説明用。当たりは boxesMm)
    boxesMm: [                              // 合板の上の面が 0 の座標 [mm]。lo = [x, y, z] の小さい方の角、hi = 大きい方の角
      { id: "c_foot",   name: "C の柱の足の金具", lo: [323.3, 0, -10.5],    hi: [383.3, 20, 49.5] },
      { id: "c_pole",   name: "C の柱",           lo: [343.3, 0, 9.5],      hi: [363.3, 575.3, 29.5] },
      { id: "c_arm",    name: "C の吊る腕",       lo: [117.3, 575.3, 9.5],  hi: [398.3, 595.3, 29.5] },
      { id: "c_bracket", name: "C の吊る腕の金具", lo: [323.3, 555.3, 9.5], hi: [383.3, 575.3, 29.5] },
      { id: "c_hanger", name: "C の吊る板",       lo: [127.3, 563.3, -10.5], hi: [187.3, 575.3, 49.5] },
      { id: "c_cam",    name: "C のカメラ",       lo: [141.3, 540, 3.5],    hi: [173.3, 572.3, 35.5] }
    ],
    lensFromFaceMm: 26.3                    // 基板の前の面からレンズの先まで(公式のカメラの STL。A と同じ品を仮に使う)
  };
  /** C の台の部品を、軸にそろった箱 { id, name, lo, hi } で(m、base_link の原点が 0 の座標。overheadPartPos と同じ)。
   *  当たり判定と絵は scene-data.js がこれから作る。2026-11-19 まで(線分と半径 a・b・r の円柱)から形が変わった */
  function overheadRig() {
    var ty = overheadTableY();
    function p(v) { return [v[0] / 1000, v[1] / 1000 + ty, v[2] / 1000]; }
    return OVERHEAD_C.boxesMm.map(function (b) { return { id: b.id, name: b.name, lo: p(b.lo), hi: p(b.hi) }; });
  }
  /** 上のカメラ(置き方 C): 基板の前の面の真ん中(face)、レンズの先(tip)、向き(dir)、絵の上(up)。画面の座標(m、base_link の原点が 0)。
   *  2026-11-17 の夜まで(置き方 A)は、本家の台の上の部品から出していた(OVERHEAD.camFaceMm・camDir・camUp。比べる用に残す) */
  function overheadCamera() {
    var C = OVERHEAD_C, ty = overheadTableY(), t = C.lensTipMm, d = C.camDir, k = C.lensFromFaceMm / 1000;
    var tip = [t[0] / 1000, t[1] / 1000 + ty, t[2] / 1000];
    return { tip: tip, face: [tip[0] - d[0] * k, tip[1] - d[1] * k, tip[2] - d[2] * k], dir: d.slice(), up: C.camUp.slice() };
  }

  /** 上のカメラの絵(ピンホール・ゆがみなし)。cam = { tip, dir, up }(同じ座標)、縦の画角 fovDeg、絵 W×H 画素。
   *  絵の右 = dir × up、絵の上 = 右 × dir(three.js の PerspectiveCamera に up を入れて lookAt したときと同じ向き)。
   *  ★2026-11-17 の夜まで、tools の 2 本は 右 = up × dir で、絵が左右に裏返っていた(絵に入る点の数は変わらない)。
   *  project(p) → [u(右へ), v(下へ)](画素の左上の角が 0。画素 i は u が i〜i+1)。後ろなら null
   *  unproject(u, v, planeY) → その画素の光線が高さ planeY の水平の面に当たる点 */
  function overheadImage(cam, fovDeg, W, H) {
    function norm(v) { var n = Math.hypot(v[0], v[1], v[2]); return [v[0] / n, v[1] / n, v[2] / n]; }
    function cross(a, b) { return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]; }
    function dot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
    var z = norm(cam.dir), x = norm(cross(z, norm(cam.up))), y = cross(x, z), c = cam.tip;
    var f = (H / 2) / Math.tan(fovDeg * Math.PI / 360);
    return {
      f: f, right: x, up: y, fwd: z, tip: c,
      project: function (p) {
        var d = [p[0] - c[0], p[1] - c[1], p[2] - c[2]], zc = dot(d, z);
        return zc > 0 ? [W / 2 + f * dot(d, x) / zc, H / 2 - f * dot(d, y) / zc] : null;
      },
      unproject: function (u, v, planeY) {
        var a = (u - W / 2) / f, b = (H / 2 - v) / f;
        var r = [z[0] + a * x[0] + b * y[0], z[1] + a * x[1] + b * y[1], z[2] + a * x[2] + b * y[2]];
        var s = (planeY - c[1]) / r[1];
        return [c[0] + r[0] * s, planeY, c[2] + r[2] * s];
      }
    };
  }

  /** three.js の XYZ のオイラー角[度](行列 = Rx·Ry·Rz) */
  function eulerXYZ(R) {
    var m13 = Math.max(-1, Math.min(1, R[2])), x, y = Math.asin(m13), z;
    if (Math.abs(m13) < 0.9999999) { x = Math.atan2(-R[5], R[8]); z = Math.atan2(-R[1], R[0]); }
    else { x = Math.atan2(R[7], R[4]); z = 0; }
    return [x / DEG, y / DEG, z / DEG];
  }

  return {
    LINK_GROUP: LINK_GROUP, CAMERA_GROUP: CAMERA_GROUP, OVERHEAD: OVERHEAD, overheadTableY: overheadTableY, overheadPartPos: overheadPartPos, overheadCamera: overheadCamera, overheadImage: overheadImage, OVERHEAD_C: OVERHEAD_C, overheadRig: overheadRig, CAM_FOV: CAM_FOV,
    visualPose: visualPose, eulerXYZ: eulerXYZ, mm: mm, mv: mv, rpy: rpy,
    URDF: URDF, ARM: ARM, DEG: DEG,
    axes: joints.map(function (j) { return { axis: j.axis, origin: j.origin }; }),
    joints: joints.slice(0, ARM.length),
    jaw: joints.jaw,
    tool: joints.tool,
    limitsDeg: joints.map(function (j) { return j.limitsDeg; }),
    urdfFK: urdfFK,
    toView: toView
  };
})();

if (typeof module !== "undefined") module.exports = SO101;
