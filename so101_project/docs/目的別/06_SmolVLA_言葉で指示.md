# 06 言葉で指示する方策（SmolVLA）を微調整し、ACT と比べる

- 作成：2026-11-16
- 目的：Hugging Face の SmolVLA（画像と言葉を入れると動きを出す、学習済みの方策）を、自分の記録で**微調整**し、文で指示して動かす。同じ記録で ACT も学習し、05 の条件で比べる。
- 動画の場面（17:07〜20:32）：
  - レゴを何個も片付ける課題で SmolVLA を約 3 時間微調整。最初に 2 個つかんだが、その後は動かなかった（動画の人は「課題が難しすぎたかも」）。
  - 課題を易しくした（大きいブロック 1 個を灰色の箱へ、50 回の記録に約 1 時間）。学習を 3 万ステップに増やし、約 4 時間（途中で停電してやり直した）。1 分間、続けて動いた。
  - 動画の人の観察：ACT は先の動きをまとめて長く出すので、つかんだ後に箱を動かすと、もとの場所に落としがち。SmolVLA は箱を動かしても追いついた（回数は数えていない）。
- 確かめた範囲（2026-11-16）：コマンドのオプションの名前は、本体（0.6.2、`53feaef`）のソースと本家の案内で確かめた。**SmolVLA はまだ入れていない**（本体の環境に `transformers` などが無い）。実機でも、学習の場所でも、一度も流していない。

## 0. 先に知っておくこと

| 項目 | 中身 | 出どころ |
|---|---|---|
| 元の方策 | `lerobot/smolvla_base`（約 4.5 億のパラメータ） | 本家の SmolVLA の案内 |
| 記録の数の目安 | 約 50 回。置き方ごとに十分な回数（論文のデータは 5 か所 × 10 回。25 回では足りなかった、とある） | 本家の SmolVLA の案内 |
| 学習の時間の目安 | 2 万ステップで、A100 1 枚で約 4 時間 | 本家の SmolVLA の案内 |
| GPU のメモリの目安 | バッチサイズ 8 で約 10〜16 GB（ACT は約 2〜6 GB） | 本家のハードウェアの案内（[01](01_買い物リスト.md) の K） |
| カメラの名前 | `smolvla_base` は、画像を `observation.images.camera1`・`camera2`・`camera3` の名前で受け取る（`config.json` を 2026-11-16 に読んだ）。このプロジェクトの記録は `front`・`wrist` なので、**名前を付け替える**（`--rename_map`）。足りない 3 台目は `--policy.empty_cameras=1` で空にする | `config.json`、本家の `rename_map` の案内、`configuration_smolvla.py` |
| 一度に出す動きの長さ | SmolVLA は 50 歩、ACT は 100 歩（どちらも既定） | `configuration_smolvla.py`、`configuration_act.py` |
| 実機で動かすコマンド | `lerobot-rollout`。本体の `lerobot-record` は方策を動かせない（動画の版とは違う） | `lerobot_record.py` の説明 |

- 動画の人の「ACT は長くまとめて出すので、箱の移動に遅れる」は、上の「一度に出す動きの長さ」の違いと合います。ACT の `n_action_steps` は「1 回の推論で出した動きのうち、何歩を実際に使うか」です（`configuration_act.py` の説明。`chunk_size` 以下）。小さくする（例：20）と、先頭だけ使って、また画像を見るようになります。学習は同じで、動かすときに `--policy.n_action_steps=20` を足して上書きできるかは未確認です。

## 1. 課題を決める（易しい方から）

| 段 | 課題 | 指示の文（`--dataset.single_task`） | 記録 |
|---|---|---|---|
| 1 | 大きいレゴ 1 個を、灰色の箱に入れる（動画で上手くいった方） | `Grasp a lego block and put it in the bin.` | 50 回。ブロック 5 か所 × 箱 2 か所。組ごとの回数を表に書く |
| 2 | 赤と青のブロックを置き、言われた色だけ入れる | `Put the red block in the bin.` と `Put the blue block in the bin.` を半分ずつ | 段 1 が通ってから。文ごとに 25 回以上 |
| 3 | 何個も片付ける（動画で上手くいかなかった方） | `Put all the lego blocks in the bin.` | 段 2 が通ってから |

- 段 1 の文は、本家の SmolVLA の案内の例と同じ文にしています。
- **記録と評価で、文を一字一句同じにします**（本家の案内「記録で使った文と同じに」）。
- 段 2 は、言葉で区別させる試しです。05 の `two` の条件（2 個あると迷う）への答えになるかを見ます。

## 2. 記録する

[データ収集と学習の手順書](../データ収集と学習の手順書.md) 11（試し記録）と 12（本番の記録）と同じです。違うのは、データの名前と文と時間だけです。

```powershell
python tools\prime_goal.py <フォロワーのポート>
lerobot-record --robot.type=so101_follower --robot.port=<フォロワーのポート> --robot.id=follower1 --robot.cameras="{ front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, wrist: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 30}}" --teleop.type=so101_leader --teleop.port=<リーダーのポート> --teleop.id=leader1 --display_data=true --dataset.repo_id=<HFユーザー名>/lego_bin --dataset.no_stamp=true --dataset.single_task="Grasp a lego block and put it in the bin." --dataset.num_episodes=50 --dataset.episode_time_s=20 --dataset.reset_time_s=15 --dataset.private=true
```

- 1 行目の `prime_goal.py` は毎回（[ルール P-001](../rules/RULES.md)）。
- 箱の位置を変えるのは、片付けの時間（`reset_time_s`）の間に。どの回にどこへ置いたかを表に書きます。
- 動画の人は 50 回に約 1 時間かかりました。途中で止まったら、12 の「途中で止まったときの続き」（`--resume=true`）。

## 3. 学習の場所に SmolVLA を入れる（GPU のパソコン）

準備手順書 A-2 で入れた LeRobot（`53feaef`）に、SmolVLA の分を足します。

```powershell
cd ~\lerobot
.venv\Scripts\activate
uv pip install -e ".[smolvla]"
```

- `smolvla` の中身は `transformers`・`num2words`・`accelerate`（本体の `pyproject.toml`）。
- 入れた後、`python -c "import torch; print(torch.cuda.is_available())"` が `True` のままかを見ます（入れ直しで CPU 版の torch に戻っていないか）。

## 4. 学習する

**学習の場所が Windows なら、先に開発者モードをオンにしてもらいます**（オフだと、保存の所で `WinError 1314` で落ちて、方策も上がらない。[Windows で学習の保存が落ちる](../knowledge/Windowsで学習の保存が落ちる.md)）。

### 4-0. 元の方策を手元に落とす（Windows では必須）

Windows では `--policy.path=lerobot/smolvla_base` のように Hugging Face の名前を渡すと、学習が重みを落とす所で止まります（名前の `/` が `\` に変わる。[Windows でハブの方策名が通らない](../knowledge/Windowsでハブの方策名が通らない.md)）。先にフォルダへ落として、フォルダを渡します。

```powershell
hf download lerobot/smolvla_base --local-dir models\smolvla_base
```

- `smolvla_base` は新しい形式です（Hub のファイルに `policy_preprocessor.json` がある。2026-11-16 に一覧を見た）。08 の 4 で `diffusion_pusht` が止まった「古い形式の変換」は要らない見込みです。

### 4-1. 試し（100 ステップ）

本番の前に、名前の付け替えが通るか、メモリが足りるか、1 秒に何ステップ進むかを見ます。

```powershell
lerobot-train --policy.path=models\smolvla_base --dataset.repo_id=<HFユーザー名>/lego_bin --rename_map="{observation.images.front: observation.images.camera1, observation.images.wrist: observation.images.camera2}" --policy.empty_cameras=1 --batch_size=8 --steps=100 --output_dir=outputs/train/smolvla_lego_try --job_name=smolvla_lego_try --policy.device=cuda --policy.push_to_hub=false
```

- `--rename_map`：記録の名前 → SmolVLA が受け取る名前。`--robot.cameras` と同じく、中に `"` を書かない形にします（PowerShell では `\"` は効かず、名前が付け替わらない）。この形が LeRobot の読み取り（`draccus`）で正しい組に読めることと、Windows PowerShell 5.1 から 1 つの引数で渡ることは確かめた（2026-11-16、学習のコマンドそのものは流していない）。
- `--policy.empty_cameras=1`：3 台目のカメラは空として扱う。
- `--policy.push_to_hub=false`：試しは上げない（これが無く、`--policy.repo_id` も無いと、本家が「repo_id が無い」で止まる。`lerobot_train.py`）。
- メモリが足りない（`CUDA out of memory`）ときは、`--batch_size=4`。
- 止まらずに 100 ステップ進んだら、表示のステップの速さから、本番の時間を見積もります（計画書の欄6 に書く）。

### 4-2. 本番

```powershell
lerobot-train --policy.path=models\smolvla_base --dataset.repo_id=<HFユーザー名>/lego_bin --rename_map="{observation.images.front: observation.images.camera1, observation.images.wrist: observation.images.camera2}" --policy.empty_cameras=1 --batch_size=8 --steps=20000 --save_freq=2000 --output_dir=outputs/train/smolvla_lego --job_name=smolvla_lego --policy.device=cuda --policy.repo_id=<HFユーザー名>/smolvla_lego --policy.private=true
```

- `--steps=20000`：本家の例の数。動画の人は、易しくした課題で 3 万にしました。
- `--save_freq=2000`：途中の学習結果を 2000 ステップごとに残す（停電に備える）。既定は 20,000 ステップごと（`configs\train.py`）なので、2 万ステップの学習では最後にしか残らない。
- **止まったときの続き**（動画の人は停電で最初からやり直した）：
  ```powershell
  lerobot-train --config_path=outputs/train/smolvla_lego/checkpoints/last/pretrained_model/train_config.json --resume=true
  ```
- GPU が借りられないときは、Hugging Face Jobs（データ収集と学習の手順書 13-3。**使った秒数で課金**、`--job.timeout` を必ず付ける）。本家の案内には Google Colab のノートもあります。
- 本家の案内の例にある `--wandb.enable=true` は、学習の様子をグラフで見るためのもので、無くても学習は回ります（v0.7.0 では `--trackio.enable=true` に名前が変わった。08 を参照）。

### 4-3. 比べるための ACT

同じデータで ACT も学習します（データ収集と学習の手順書 13 のコマンドで、名前を `lego_bin`・`act_lego` に読み替える）。ACT はカメラの名前を記録のまま使うので、`--rename_map` は要りません。

## 5. 実機で動かす

始める前の確認は、データ収集と学習の手順書 14-1〜14-3 と同じです（毎回 `prime_goal.py`、腕から 1 m 離れる、5V に手が届く所にいる）。

```powershell
python tools\prime_goal.py <フォロワーのポート>
lerobot-rollout --strategy.type=base --policy.path=<HFユーザー名>/smolvla_lego --rename_map="{observation.images.front: observation.images.camera1, observation.images.wrist: observation.images.camera2}" --robot.type=so101_follower --robot.port=<フォロワーのポート> --robot.id=follower1 --robot.max_relative_target=5 --robot.cameras="{ front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, wrist: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 30}}" --task="Grasp a lego block and put it in the bin." --duration=30
```

- `--rename_map` は学習のときと同じにします。学習で保存された設定が、名前の付け替えを覚えているかは未確認です。動かすときにカメラが見つからないというエラーが出たら、このオプションの有無を入れ替えて試します。
- カメラの名前・番号・解像度、`--robot.id` は記録のときと同じに。
- 初めて動かすときは、14-2 の安全確認（立方体を置かずに `--duration=10`、`prime_goal.py --speed 1000`）を先に通します。
- このパソコンは CPU だけなので、SmolVLA は遅くて間に合わない見込みです（測っていない）。腕をつないだまま方策を GPU のパソコンで動かす方法は [07](07_推論を別のパソコンで.md)。

## 6. ACT と比べる

[05 汎化テスト](05_汎化テスト.md) の道具で、2 つの方策を**同じ条件・同じ回数・ばらばらの順番**で回します。

```powershell
python tools\eval_log.py plan smolvla_lego_eval --conditions seen unseen move_target --repeats 5 --policy smolvla_lego --seed 2
python tools\eval_log.py plan act_lego_eval --conditions seen unseen move_target --repeats 5 --policy act_lego --seed 3
```

- `move_target`：つかんだ後に箱を 10 cm ずらす（動画の人の観察を、回数をそろえて確かめる）。
- 方策を交互に回すと、照明や片付けの慣れが片方にだけ乗りにくくなります（例：SmolVLA の 1 回 → ACT の 1 回 → …）。
- まとめは次で並べて見ます。条件ごとの Fisher の検定の行で、p ≥ 0.05 なら「この回数では見分けられなかった」と書きます（05 の 1-3）。
  ```powershell
  python tools\eval_log.py summary smolvla_lego_eval act_lego_eval
  ```

- [ ] 1 課題と文を決めた
- [ ] 2 記録した（50 回）
- [ ] 3 SmolVLA を入れた
- [ ] 4 学習した（SmolVLA と ACT）
- [ ] 5 実機で動いた
- [ ] 6 比べて、R-001 を通した
