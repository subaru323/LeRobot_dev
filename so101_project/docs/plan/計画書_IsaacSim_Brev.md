# 計画書：Isaac Sim を NVIDIA Brev の Isaac Launchable で使う段取り

- 作成：2026-11-17。`docs/core/guides/計画書の書き方.md` の欄に合わせた（欄3〜5・7 は条件を比べる実験ではないので飛ばす）
- 状態：**計画だけ。アカウントも作っておらず、マシンも立ち上げていない**（費用がかかる操作は利用者に聞いてから）。段 I0 の前で止まっている。R-001 の点検 [C19](../records/R001_LOG.md) を通した
- 入口は [STATUS.md](../../STATUS.md)。決まりは [docs/Simulation_Logic_Documents/](../Simulation_Logic_Documents/README.md)。ほかの計画書：[計画書_シミュレーション](計画書_シミュレーション.md)（59 行目で Isaac を「重い」として外していた。この計画書で、クラウドで使う方へ改める）、[計画書_完成形まで](計画書_完成形まで.md)（案 A）、[計画書_テレオペ以降](計画書_テレオペ以降.md)（欄6 の費用の上限が未定）

## いまの結論（2026-11-17 の時点。公式の資料を読んだだけで、動かしていない）

1. **Isaac Launchable は、Brev のクラウドの GPU マシンに、ブラウザの VS Code と Isaac Sim の画面の中継をまとめて立ち上げるひな形。** 版は Isaac Sim 6.0.1・Isaac Lab 3.0.0-beta2-post1（Isaac Lab は beta）。公式は「学ぶため。本番用ではない」と書いている。手元のパソコンにはブラウザだけあればよい（下の「調べたこと」の 1）。
2. **SO-101 は、Isaac Lab 3.0 に初めから入っている SO-101 の設定（`SO101_CFG`。実機から測った物理の値の USD）を使い、`sim/` と同じ公式の URDF と突き合わせる道を先に試す。** ただし Launchable の版（3.0.0-beta2-post1）にこの設定があるかは確かめていない（release/3.0.0 の枝にはある）。無ければ、URDF を自分で USD にする。LeIsaac（SO-101 を Isaac Lab に載せた外の道具）は、表の版が Isaac Sim 5.1・Isaac Lab 2.3.0 までで、NVIDIA の Isaac Launchable の版とは合わない。LeIsaac には別の Brev の Launchable がある（「調べたこと」の 2）。
3. **分け方の案：`sim/`（three.js）は「動きを決める・形で当たりを判定する」の正本のまま。`sim/physics/`（MuJoCo）は手元でただで回す物理。Isaac はお金がかかるので、手元でできないこと（本物に近いカメラの絵、たくさん並べて回すこと、物理の 2 つ目の検算）だけに使う。** Isaac は `sim/` が書き出した関節の列（`sim/out/motion_*.csv`）とピースの STL（`sim/out/pieces/`）を読むだけにして、動きを決める所を Isaac の中に作り直さない（欄2）。
4. **費用の見込み：段 I1〜I4 の計算の代が 30〜55 ドル（やり直しを入れて 80 ドルまで）。これに、マシンを残している間のディスクの代（200GB なら月 19 ドルほど）を足す。** Brev の L40S の 1 時間の値段は公式の値段の表に無く、他社の相場からの見込み。Brev の値段がそれより高いこともありうるので、I0 でコンソールの値段に入れ替えるまで上限を決めない（欄6）。
5. **Isaac Launchable は 1.1 より前の版に、重い弱さ（CVE-2025-33223。CVSS 9.8）があった。** 立ち上げる前に、使う版が直したあとのものかを NVIDIA の発表で見る（「確かめていない前提」の 8）。

## 欄1. 利用者が欲しい答え

利用者の言葉（2026-11-17）：「今後、IsaacSim は NVIDIA Brev、Isaac Launchable を使ってやっていきたい」。

この計画が答える問い：

- Brev と Isaac Launchable は何で、どう使うか（料金・GPU・起動・版・つなぎ方・ファイルの出し入れ）。
- SO-101 をそこへ載せる道はどれか。
- いまの `sim/`（three.js）と `sim/physics/`（MuJoCo。別の会話が作っている）と、どう分けるか。
- 段階・終わりの条件・費用・利用者に決めてほしいこと。

答えない部分：

- Isaac で学習（ACT・強化学習）をどう組むか。段 I4 で絵を出せてから考える。
- MuJoCo 側の中身（`sim/physics/` を持つ会話が決める）。
- Brev の値段の正しい数字（コンソールに入らないと見えない。段 I0 で利用者に見てもらう）。

## 調べたこと（出どころと確かめた日。どれも 2026-11-17 に読んだ）

### 1. NVIDIA Brev と Isaac Launchable

| 項目 | 分かったこと | 出どころ |
|---|---|---|
| Brev とは | クラウドの GPU マシンを借りる仕組み。いくつかのクラウド会社のマシンを選べる。ドライバ・CUDA・Python・Docker が入った状態で立ち上がる | [Brev の文書](https://docs.nvidia.com/brev/latest/)・[GPU Instances](https://docs.nvidia.com/brev/latest/concepts/gpu-instances.md) |
| Launchable とは | マシンの種類・ソフト（Docker Compose など）・コード・開く口（Secure Link とポート）をまとめた、リンク 1 つで同じ環境を立ち上げるひな形。TCP/UDP のポートは、作った人が「全部の IP」か「立ち上げた人の IP だけ」かを選ぶ | [Launchables](https://docs.nvidia.com/brev/latest/concepts/launchables.md) |
| Isaac Launchable の中身 | ブラウザの VS Code（code-server）、Isaac Lab のコンテナ（`nvcr.io/nvidia/isaac-lab:3.0.0-beta2-post1` を元にする）、Isaac Sim 6.0.1、Isaac Sim の画面をブラウザに中継する見る側（web viewer）。「学ぶため。本番用ではない」 | [isaac-sim/isaac-launchable](https://github.com/isaac-sim/isaac-launchable)（README と `isaac-lab/vscode/Dockerfile`） |
| Isaac Sim の最新 | 6.2.0（文書の最終更新 2026-10-14）。Launchable はそれより古い 6.0.1 に固定 | [Isaac Sim の Release Notes](https://docs.isaacsim.omniverse.nvidia.com/latest/overview/release_notes.html) |
| 使える GPU（Isaac Sim 全体） | RT コアのある GPU が要る。A100・H100 は Isaac Sim が対応しない。最低は RTX 4080・VRAM 16GB | [要件](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html) |
| 使える GPU（Launchable） | README は GPU を指定せず「選ぶ」とだけ書く。RT コアが要る（画面の中継のため）。試したクラウドは AWS。Crusoe のマシンは使えない。既定が L40S 1 枚というのは第三者の記事（2026-03-01）だけが根拠。I0 で Launchable のページで見る | Launchable の README・[npaka の記事](https://note.com/npaka/n/n8acd18865d8c) |
| 参考：Isaac Sim の Brev の手順（Launchable とは別） | VM の形で `nvcr.io/nvidia/isaac-sim:6.2.0` を Docker で立てる。「L40S を 1 枚」を選ぶ。ポートは 49100・47998・8210、見る側は `http://<IP>:8210`。中継の口には認証も暗号も無いので、自分の IP だけに開ける | [Brev Deployment（6.2.0）](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/installation/install_advanced_cloud_setup_brev.html) |
| 料金のしくみ | 動いている間は GPU の種類ごとに 1 時間いくら。止めると計算の代はかからないが、ディスクの代は残る（$0.0001314 / GB / 時、マシンがある間ずっと）。消すと 0。組織の残高が尽きると、Brev が止められるマシンを止め、ほかは消すことがある（消したデータは戻らない） | [GPU Instances](https://docs.nvidia.com/brev/latest/concepts/gpu-instances.md)・[値段の表](https://www.nvidia.com/en-us/launchables/pricing) |
| 値段の数字 | 公式の値段の表は L4・T4・A10G・A100・H100 などで、**L40S が載っていない**（表は古いカードも並んでいて、いつの値段か書いていない。正しい値段はコンソールで見る、とある）。L40S の他社の相場は、1 枚 1 時間 $1.57（GetDeploying、35 社の中央値、2026-11-12）。AWS の 1 枚の型（g6e.xlarge）は $1.86（Beam のブログ、日付なし）。Brev がこれに上乗せするかは確かめていない | [値段の表](https://www.nvidia.com/en-us/launchables/pricing)・[GetDeploying](https://getdeploying.com/gpus/nvidia-l40s)・[Beam](https://www.beam.cloud/blog/nvidia-l40s-pricing) |
| 起動のしかた | Launchable のページで「Deploy Launchable」→ 立ち上がって setup が終わるまで待つ（初回は時間がかかる。影の計算の溜め込みのため）→ マシンのページの「Using Secure Links」の Shareable URL を開き、Brev のアカウントで入る → VS Code が開く。Isaac Sim は `ACCEPT_EULA=y /isaac-sim/runheadless.sh`、Isaac Lab は `./isaaclab.sh -p <スクリプト> --livestream 2 --viz kit`。画面は同じ URL の末尾に `/viewer/` を付けた別のタブで見る（見るタブは 1 つだけ） | Launchable の README・`isaac-lab/vscode/README.md`・[Launchable Deployment](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_advanced_cloud_setup_launchable.html) |
| 開くポート（Launchable） | TCP/UDP の 1024・47998・49100（中継）。VS Code は Secure Link（Brev のログインで守られる）。中継のポートを自分の IP に絞ってあるかは確かめていない | Launchable の README |
| 手元からのつなぎ方 | (a) ブラウザだけ（上の Secure Link）。(b) Brev の CLI：Windows では WSL の Ubuntu の中に入れる。`brev login` → `brev shell <名前>`（SSH）・`brev open <名前>`（手元の VS Code で開く）・`brev port-forward <名前> --port 8888:8888`（UDP は通らない）・`brev exec <名前> "<コマンド>"`・`brev start`／`brev stop`／`brev delete` | [CLI の始め方](https://docs.nvidia.com/brev/cli/getting-started)・[つなぎ方](https://docs.nvidia.com/brev/cli/connectivity)・[マシンの扱い](https://docs.nvidia.com/brev/cli/instance-management) |
| ファイルの持ち込みと持ち出し | `brev copy`（scp を包んだもの。CLI が要る）：`brev copy -r ./フォルダ <名前>:/home/ubuntu/`、逆向きで持ち出し。`--host` でコンテナでなくマシン本体へ。`scp`（鍵は `~/.brev/brev.pem`）も使える。続けて同期するなら rsync を勧めている | [File Transfer with SCP](https://docs.nvidia.com/brev/latest/guides/development-tools/file-transfer-scp.md) |
| 止めたときに残るもの | Brev の文書では、`/home/ubuntu/workspace`・入れたパッケージ・Docker のイメージとコンテナは止めても残る。`/tmp` は消える。消すと全部消える。止めたマシンを起こすときは、同じ GPU が同じ地域に空いていないと起きない（空くまでデータに触れない）。**これは Brev の標準のコンテナの話で、Launchable の Docker Compose のコンテナ（作業の場所は `/workspace`）にも当てはまるかは I1 で見る** | [GPU Instances](https://docs.nvidia.com/brev/latest/concepts/gpu-instances.md) |

**ファイルを読んで気づいた危うい所（動かして確かめていない）**：Launchable の `docker-compose.yml` で VS Code のコンテナに付く置き場は Omniverse の溜め込み 2 つ（`/root/.cache/ov`・`/root/.local/share/ov`）だけで、作業の場所 `/workspace` は置き場として付いていない（Dockerfile にも `VOLUME` が無い）。そう読むと、`/workspace` に書いたものは、マシンを止めるだけなら残るが、`docker compose down` でコンテナを作り直すと消える。Launchable の README は、困ったときの手として `docker compose down` → `up -d` を勧めているので、**その手順の前には必ず持ち出す**。I1 で確かめるまでは、作ったものは作業の終わりに毎回手元へ持ち出す（I1・I2 ではブラウザの VS Code のダウンロード。`brev copy` は CLI を入れると決めてから）。

### 2. SO-101 を Isaac に載せる道

| 道 | 中身 | 版の合い方 | 出どころ |
|---|---|---|---|
| D. Isaac Lab の SO-101 の設定 | Isaac Lab の `isaaclab_assets/robots/so101.py` に `SO101_CFG` と `SO101_HIGH_PD_CFG`。USD は NVIDIA の置き場（Nucleus）の `Robots_Multiphysics/RobotStudio/so101_new_calib_SysID/so101_new_calib.usda`。駆動の値は USD に書かれ、実機から測った値（SysID）の物理の組を選ぶ。関節の名前は `sim/` と同じ 6 つ（shoulder_pan・shoulder_lift・elbow_flex・wrist_flex・wrist_roll・gripper）。元は TheRobotStudio の SO-ARM100。NVIDIA の講座「Getting started with Isaac Lab」にも SO-101 の章がある | release/3.0.0 の枝にある。**Launchable の 3.0.0-beta2-post1 にあるかは確かめていない**（I1 で見る） | [so101.py（release/3.0.0）](https://github.com/isaac-sim/IsaacLab/blob/release/3.0.0/source/isaaclab_assets/isaaclab_assets/robots/so101.py)・[講座の SO-101 の章](https://docs.nvidia.com/learning/physical-ai/getting-started-with-isaac-lab/latest/train-a-robot-arm/02-so101-robot.html) |
| A. 自分の URDF を取り込む | `sim/` と同じ公式の URDF（TheRobotStudio の `Simulation/SO101`、`sim/models/src/` に取ってある）を、Isaac Sim 6 の URDF の取り込み（6.0 で Python の作りの 3.0 に替わった）か、Isaac Lab の `scripts/tools/convert_urdf.py` で USD にする | Launchable の 6.0.1・3.0 で使える。6.0 では mimic の関節の書き方が替わったが、`so101_new_calib.urdf` に mimic は無い（grep で 0 件。動く関節は 6 つ、固定 1 つ） | [URDF・MJCF の取り込みの変更（6.0）](https://docs.isaacsim.omniverse.nvidia.com/latest/migration_guides/isaac_sim_6_0/urdf_mjcf_importer_exporter_pipeline.html)・[Isaac Lab：Importing a New Asset](https://isaac-sim.github.io/IsaacLab/release/3.0.0/source/how-to/import_new_asset.html) |
| B. LeIsaac | LightwheelAI の道具。SO-101 のフォロワーを Isaac Lab に載せ、リーダーで動かして記録し、LeRobot のデータの形に変える。キーボード・ゲームパッド・遠くからの操作もある。Apache-2.0。**LeIsaac 用の Brev の Launchable が別にある**（ポート 80 の HTTP で VS Code に入り、既定のパスワードが `password`。中の Isaac の版は書かれていない） | 表は Isaac Sim 4.5／5.0／5.1 と Isaac Lab 2.1.1／2.2.1／2.3.0 まで。**NVIDIA の Isaac Launchable の 6.0.1・3.0 は表に無い**。LeRobot の文書の例は `lerobot==0.4.1`（このプロジェクトは 0.6.2） | [LeIsaac](https://github.com/LightwheelAI/leisaac)・[入れ方](https://lightwheelai.github.io/leisaac/docs/getting_started/installation)・[LeIsaac の Brev](https://lightwheelai.github.io/leisaac/docs/cloud_simulation/nvidia_brev)・`MAIN_PG/lerobot/docs/source/envhub_leisaac.mdx`（58 行目・245〜256 行目） |
| C. newton-assets の SO-101 の USD | MuJoCo Menagerie の `robotstudio_so101` を USD に変えたもの（2026-08-12 に取り込まれた。URDF・MJCF は入っていない） | Newton 向け。元のモデルが `sim/` と違う | [newton-assets PR #38](https://github.com/newton-physics/newton-assets/pull/38) |

LeRobot 本体（`MAIN_PG/lerobot`、0.6.2）との関係：

- `examples/isaac_teleop_to_so101/` は **NVIDIA Isaac Teleop**（VR の手持ちか SO-101 のリーダーで、**実機の**フォロワーを動かして記録する道具）で、シミュレーションではない。Linux だけ。ただ、URDF を Hugging Face の `lerobot/robot-urdfs` の `so101` から取ってくる（`common.py` 168〜183 行目の `_ensure_so101_urdf`。中身は `so101_new_calib.urdf`）ので、LeRobot も同じ名前の URDF を使っている。
- `lerobot.envs.make_env` は Hub の環境を読める。`trust_remote_code` は既定が False で、True を明示したときだけ Hub のコードを動かす（`src/lerobot/envs/factory.py` 63・75 行目）。LeIsaac はこの口で LeRobot につながる。IsaacLab Arena は専用の設定 `--env.type=isaaclab_arena` もある（`src/lerobot/envs/configs.py` 650 行目）。Arena の文書は Isaac Sim 5.1.0・Isaac Lab 2.3.0（`docs/source/envhub_isaaclab_arena.mdx`）。
- 本体のソースは書き換えない（CLAUDE.md）。Isaac の側で要るものは、プロジェクトの別の場所に置く。

**選ぶ道**：D を先に試し、無ければ A。理由は、(1) どちらも NVIDIA の Isaac Launchable の版のまま使える、(2) D は実機から測った物理の値が入っていて、自分で値を合わせる手間が省ける、(3) D の USD を `sim/` と同じ URDF と、関節の名前・可動域・道具の点の位置で突き合わせれば、[00 の条件 12](../Simulation_Logic_Documents/00_絶対条件.md)（判定の形が実物と同じ）を Isaac の側でも見られる、(4) 完成形の案 A はリーダーを使わないので、LeIsaac の強み（リーダーで記録）を今は使わない。B は、Isaac で人の手本から学習させると決めたときに、LeIsaac の Launchable を使う道として残す（利用者の言う「Isaac Launchable」に LeIsaac の Launchable が入るかは、利用者に聞く）。

## 欄2. 選択肢の地図

| 軸 | 今回やるもの | 今回やらないもの（理由） |
|---|---|---|
| どこで動かす | NVIDIA の Isaac Launchable（利用者の決定） | 手元のパソコン（GPU の型番が分からない。Isaac は RTX 4080 以上・VRAM 16GB 以上）／Brev の VM の形で自分で入れる／LeIsaac の Launchable（B の道を選ぶまで） |
| GPU | RT コアのある 1 枚（Launchable の既定。第三者の記事では L40S。I0 で見る） | 何枚も（並べて回すのは I4 の後）／A100・H100（RT コアが無く、Isaac Sim が対応しない） |
| SO-101 の載せ方 | D：Isaac Lab の `SO101_CFG`（無ければ A：`sim/models/src/so101_new_calib.urdf` を USD に） | B：LeIsaac（版が合わない）／C：newton-assets の USD（元のモデルが違う） |
| Isaac にさせること | 本物に近いカメラの絵（上・手首）、ピースをつかむ物理の 2 つ目の検算、たくさん並べて回すこと | 動きを決めること（`sim/` の仕事。作り直さない）／学習（I4 の後で決める） |
| つなぎ方 | ブラウザ（Secure Link と `/viewer/`） | 手元の VS Code・Brev の CLI（WSL を入れる必要がある。入れるのは利用者。I1 の後で決める） |
| ファイルの出し入れ | ブラウザの VS Code のアップロードとダウンロード | `brev copy`（CLI を入れると決めてから）／git（このプロジェクトのフォルダは git の置き場になっていない）／Hugging Face の置き場（大きなデータを出すときに考える） |
| 操る人 | 利用者がブラウザで操り、Claude が流すスクリプトと手順を書く | Claude が直接クラウドを操る（CLI と WSL が要る。止めるのは聞かずにしてよい、立ち上げと消すのは毎回聞く。下の「止める決まり」） |

### `sim/`・`sim/physics/`・Isaac の分け方の案

| 置き場 | 持ち主 | 仕事 | 費用 | 正本か |
|---|---|---|---|---|
| `sim/`（three.js・ブラウザ） | 会話「LeRobot プロジェクト実装ガイド」 | 腕の形・逆運動学・置き場と順番を決める・形での当たり判定・上のカメラの絵の正解の画素・関節の列とピースの STL の書き出し | ただ・手元 | **動きと判定の正本** |
| `sim/physics/`（MuJoCo） | MuJoCo を作っている別の会話 | ピースが滑る・倒れる・爪の力。`sim/` の関節の列を流して、物理で崩れないかを見る。公式の MJCF（`sim/models/src/so101_new_calib.xml`）がある | ただ・手元（CPU） | 物理の 1 つ目 |
| Isaac（Brev の Launchable） | この計画書（実装は別の会話に頼む） | (1) 本物に近いカメラの絵（光・影・色を振る）を作り、[09 画像だけで動かす](../Simulation_Logic_Documents/09_画像だけで動かすシミュレーション.md) の見つける道具を試す、(2) MuJoCo と同じ関節の列で、物理の結果が同じ側に出るかの検算、(3) たくさん並べて回すこと（学習を決めたとき） | 時間で払う | どれの正本にもしない。`sim/` の書き出しを読むだけ |

- **入口と出口**：Isaac が読むのは `sim/out/motion_*.csv`（列は `t,shoulder_pan,shoulder_lift,elbow_flex,wrist_flex,wrist_roll,gripper_deg,held`。角度は URDF の度で、爪も URDF の度（LeRobot の 0〜100 ではない）。`held` は持っているピースの番号で、-1 は持っていない。Isaac でつかむ・離すを決めるのに使う）と `sim/out/pieces/*.stl`、置き場は `sim/layouts/`。座標と単位は [02 座標と単位](../Simulation_Logic_Documents/02_座標と単位.md) に合わせる（Isaac は m・ラジアン・Z 上なので、変換を 1 か所に置く）。Isaac が出すのは絵と、物理で動いたあとのピースの位置。これを手元へ持ち帰り、`sim/` の正解と比べる。
- **置き場所**：Isaac 用のスクリプトは、`sim/` の外の新しいフォルダ（案：プロジェクト直下の `isaac/`）に置く。`sim/` の持ち主の会話と同じファイルを書かないため（core 方針11）。作るのは利用者が決めてから。
- 案のまま。MuJoCo 側の中身は読んでいない（2026-11-17 の時点で `sim/physics/` に `run_physics.py` と `.venv`、`sim/out/physics/` に試しの出力が 1 つある）。MuJoCo の会話と役目がぶつかったら、ここを直す。

## 段階（各段の終わりの条件）

| 段 | やること | 終わりの条件（確かめ方） | 費用の見込み | 2026-11-17 |
|---|---|---|---|---|
| I0 | 利用者が Brev のアカウントを作り（会社か個人か）、支払いの手段か残高を入れる。コンソールで、Isaac Launchable のページの GPU・ディスクの大きさ・1 時間いくらを見る。自動で止める機能や予算の知らせがあるか、支払いの形（前払いの残高か、後払いか）を見る。使う Isaac Launchable が CVE-2025-33223 を直したあと（1.1 以降）のものかを、NVIDIA の発表で見る | 値段・ディスク・止める機能・支払いの形・版を、この計画書の「費用」に書き入れた | 0（アカウントだけ） | まだ（利用者） |
| I1 | 試し走り：Launchable を立ち上げ、Isaac Sim と Isaac Lab の例（`create_empty.py`）を `/viewer/` で見る。ブラウザの VS Code でファイルを 1 つ持ち込み、1 つ持ち出す。`/workspace` に書いたものが、止めて起こしたあとと、`docker compose down`・`up -d` のあとに残るかを見る。`ls /workspace/isaaclab/source/isaaclab_assets/isaaclab_assets/robots/so101.py` で D があるかを見る。終わったら止める | 画面の写しと、残る・消えるの結果と、D の有無を [docs/records/](../records/) に書いた。止めた（コンソールで Stopped） | 2〜3 時間：3〜6 ドル | まだ |
| I2 | SO-101 を載せる：D があればその USD を、無ければ `so101_new_calib.urdf` と STL を持ち込んで USD にする。関節の名前・向き・可動域を URDF と比べる。`sim/` の関節の列を 1 本流し、道具の点の位置を `sim/` の順運動学（`sim/` で `node tools/check-fk.js` と同じ式）と比べる | 関節 6 つの名前と可動域が URDF と同じ。同じ角度での道具の点の差を、比べる道具の出力から写した。差の許し（案 1mm）は I2 の前に決める（`sim/` の順運動学は URDF と 3.45e-13mm で合うので、差は取り込みの違いから出る。1mm は Claude の仮の線）。USD を手元へ持ち出した | 4〜6 時間：6〜11 ドル | まだ |
| I3 | 机・ピース・上と手首のカメラを置く。`sim/` の置き場の 1 種を流し、上のカメラの絵を撮って、`sim/out/overhead-truth/` の正解の画素と比べる | ピースの角の画素の差を出し、I3 の前に決めた線と比べた（線は [計画書_完成形まで](計画書_完成形まで.md) の S-12 の要る細かさから決める） | 6〜10 時間：10〜19 ドル | まだ |
| I4 | 物理の検算：MuJoCo の会話と同じ関節の列で、つかんで置いたあとのピースの位置を比べる。光・色を振った絵を何枚か作る | 同じ列で 2 つの物理の結果を並べた表がある。振った絵が手元にある | 6〜10 時間：10〜19 ドル | まだ |
| （後） | 学習（ACT・強化学習）や大量の絵を作るかを決める | 利用者が決める（欄6 の上限が先） | 別に見積もる | — |

- 各段は「立ち上げる → 作業 → 持ち出す → 止める」で 1 回とし、**作業の終わりに必ず止める**。止め忘れると、1 時間 1.6〜1.9 ドルの見込みで 1 日 40 ドル前後かかる。
- I1 は試し走りで、本番に数えない（計画書の書き方の「実物を動かす作業」）。
- 同じ GPU が空いていないと、止めたマシンが起きない。起きなければ、新しく立ち上げ直す（持ち出しておけば失うものは無い）。

### 止める決まり（2026-11-18 の利用者の決定「はい、書いて」）

Claude が Brev の CLI で操れるようになったら（WSL に CLI を入れ、`brev login` を利用者がした後）、次のとおりにする。

| 操作 | コマンド | 利用者に聞くか |
|---|---|---|
| 止める | `brev stop <名前>` | **聞かない**。使い終わったのを確かめたら、Claude が止める。計算の代が止まる（ディスクの代は少し残る） |
| 状態を見る | `brev ls` | 聞かない |
| 立ち上げる・起こす | Launchable の Deploy・`brev start <名前>` | **毎回聞く**（お金がかかり始める） |
| 消す | `brev delete <名前>` | **毎回聞く**（データは戻らない。`brev stop` が使えない型で、消すしかないときも聞く） |

止めるときの順：

1. 利用者に「これで終わりか」を確かめる（会話の中の言葉で。作業の途中で止めない）。
2. 作ったものを手元へ持ち出す（`brev copy`。`/workspace` が残るかは I1 で確かめるまで分からないため）。
3. `brev stop <名前>` を流す。
4. `brev ls` で止まった（Stopped）のを確かめ、止めた時刻と一緒に利用者に知らせる。止まらなければ、そのことを知らせ、コンソールで止めてもらう。

- （案。利用者はまだ決めていない）1 回の作業の上限（欄6 の案 4 時間）を超えたら、使い終わったかを確かめられなくても、持ち出してから止める。
- Claude の会話が閉じている間は見張れない。止め忘れの備えには、コンソールの自動停止か予算の知らせを使う（あるかは I0 で見る）。
- 出どころ（2026-11-18 に読んだ）：[Instance Management](https://docs.nvidia.com/brev/cli/instance-management)（`brev stop`・`--all`・`brev delete`）、[GPU Instances](https://docs.nvidia.com/brev/latest/concepts/gpu-instances)（止めると計算の代は無く、ディスクの代は残る。消すと戻らない）、第三者の記事（型によっては `brev stop` が使えなかった）[Classmethod](https://dev.classmethod.jp/en/articles/nvidia-brev-create-gpu-instance/)。

## 欄6. 費用と止め方（実行前に決める）

**見込み**（I0 でコンソールの値段に入れ替える）：

- 1 時間の値段：1.6〜1.9 ドルと見込む（L40S の他社の相場 $1.57 と AWS の型 $1.86 から。Brev の値段は確かめておらず、上乗せがあればもっと高い）。
- I1〜I4 の計算の代：18〜29 時間 × 1.6〜1.9 ドル → 29〜55 ドル。初回の溜め込みの待ちや、やり直しを入れて 80 ドルまで。
- ディスク：$0.0001314 × 大きさ（GB）× 時間。マシンがある間ずっと（動いていても止めていても）かかる。200GB（仮の大きさ。I0 で見る）なら $0.0001314 × 200 × 24 × 30 ≒ 月 19 ドル。I1〜I4 を 1 か月で回せば、計算の代に 19 ドルほど足す。
- 手元の側：お金はかからない（ブラウザだけ）。

**上限と止め方（案。利用者が決める）**：

- I1〜I4 の上限：計算の代とディスクの代を合わせて 100 ドル（案）。I0 で値段を入れ替えてから決める。
- 1 回の作業：4 時間で必ず止める（案）。I0 で自動で止める機能があれば、それも入れる。
- 段と段の間が 14 日（案）を超えるときは、持ち出してからマシンを消す（ディスクの代を止める。作り直しは初回の立ち上げの待ちがかかる）。I4 が終わったら消す。
- 1 つの段で、見込みの時間の上の端を超えたら止め、計画書を直して利用者に見せる。
- 残高を少なく入れておく手は、使いすぎを止める代わりに、**尽きると Brev がマシンを消すことがある**（消したデータは戻らない）。使うなら、作ったものを毎回持ち出したうえで。
- 削ってよいもの：I4 の振った絵。削ってはいけないもの：I2 の `sim/` との突き合わせ（ここを飛ばすと、Isaac の結果を `sim/` の判定と比べられない）。
- この上限は、[計画書_テレオペ以降](計画書_テレオペ以降.md) の欄6（学習の費用）とは別に数えるか、まとめるかを利用者が決める。

## 確かめていない前提（確かめる順）

1. Brev の値段（Launchable の既定の GPU の 1 時間いくら）、既定のディスクの大きさ、自動で止める機能、支払いの形（I0、コンソール）。
2. 会社のネットワークから、UDP の 47998・49100 で中継が通るか（I1。通らなければ家のネットワークで試す。手元の CLI の port-forward は UDP を通さない）。
3. Launchable の中継のポートが、自分の IP に絞ってあるか（I0・I1。中継の口には認証が無い）。
4. `/workspace` に書いたものが、止めて起こしたあとと、コンテナを作り直したあとに残るか（I1）。
5. Launchable の Isaac Lab（3.0.0-beta2-post1）に `so101.py`（D）があるか（I1）。
6. 取り込んだ SO-101 が、Isaac 6 でつまずかないか（A のときのメッシュの相対パス、慣性の値。I2）。
7. 爪のまわりの当たり（STL の細かい形）が、Isaac の物理でどう扱われるか（I2・I4）。
8. Isaac Launchable の弱さ CVE-2025-33223（公開 2025-12-23。要らない権限で動かせる・CWE-250。CVSS 9.8、ネットワークから認証なしで突ける。1.1 より前の版すべて。NVIDIA の発表は custhelp の a_id 5749）。いまの Launchable が直したあとの版か（I0）。出どころ：[CVE の記録](https://cveawg.mitre.org/api/cve/CVE-2025-33223)・[NVIDIA の発表](https://nvidia.custhelp.com/app/answers/detail/a_id/5749)。
9. Isaac Launchable の利用の約束（「Deploy Launchable」を押すと、NVIDIA Isaac Sim の追加のソフトの使用許諾に同意したことになる、と README にある）を、会社として受けてよいか（I0、利用者）。

## 利用者に決めてほしいこと

1. **アカウント**：Brev のアカウントを会社で作るか、個人で作るか。支払いの手段か残高（I0）。作るのは利用者（Claude はアカウントを作らない）。
2. **上限**：I1〜I4 の上限（案：ディスク込みで 100 ドル。I0 の値段を見てから）、1 回の作業の時間（案 4 時間）、段の間が空いたら消す日数（案 14 日）。学習の費用（計画書_テレオペ以降 の欄6）とまとめるか。
3. **SO-101 の道**：D（Isaac Lab の SO-101 の設定）→ 無ければ A（自分の URDF を取り込む）の順でよいか。
4. **「Isaac Launchable」の範囲**：NVIDIA の isaac-launchable だけを指すか、LeIsaac の Launchable（リーダーで記録して LeRobot の形に変える道具入り。ただし既定のパスワードが `password` で、版が書かれていない）でもよいか。
5. **分け方**：上の案（`sim/` が正本、MuJoCo は手元の物理、Isaac は絵と検算と並べて回すことだけ）でよいか。Isaac 用のスクリプトをプロジェクト直下の `isaac/` に置いてよいか。
6. **操り方**：ブラウザだけで進めるか、Claude が手元からも操れるように WSL と Brev の CLI を入れるか（入れるのは利用者。止めるのは聞かずに Claude がしてよい、立ち上げと消すのは毎回聞く。2026-11-18 に利用者が決めた。上の「止める決まり」）。
7. **約束と危うさ**：Isaac Sim の追加のソフトの使用許諾に会社として同意してよいか。「学ぶため。本番用ではない」ひな形で、過去に重い弱さ（CVE-2025-33223）があったものを使ってよいか。
8. **Isaac で何を一番に得たいか**：本物に近いカメラの絵（K-10 の見つける道具のため）か、物理の検算か、学習か。段 I3 と I4 の順がこれで変わる。

## 欄8・9. 点検

- R-001 の点検は [台帳](../records/R001_LOG.md) の C19（読むだけのサブエージェントで代えた）。挙がった 8 件と対応はそこに書いた。
