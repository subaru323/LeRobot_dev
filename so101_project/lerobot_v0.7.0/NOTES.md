# LeRobot v0.7.0 検証メモ（Windows / CPU）

検証日: 2026-11-16 ／ マシン: Windows 11, Intel Iris Xe（NVIDIA GPUなし）

## 環境

| 項目 | 値 |
|---|---|
| venv | `.venv`（uv, CPython 3.13.5） ※ lerobot 0.7.0 は `requires_python >=3.12` |
| インストール | `uv pip install --python .venv/Scripts/python.exe "lerobot[dataset,remote]==0.7.0"` |
| torch | 2.15.0+cpu（Windows の PyPI 版は CPU ビルド） |
| remote extra | `eclipse-zenoh>=1.9,<1.11` と `msgpack`（通信は Zenoh） |
| 追加 extra | `pusht`（gym-pusht）, `diffusion`, `training`。diffusers の制約で huggingface-hub が 2.4.0 → 1.33.0 に下がった（ストリーミング学習はその後も動作） |

再作成:

```bash
uv venv .venv --python 3.13
uv pip install --python .venv/Scripts/python.exe "lerobot[dataset,remote]==0.7.0"
```

※ 再作成すると下記のローカルパッチは消えるので当て直すこと。

## 結果

| 確認項目 | 結果 |
|---|---|
| `import lerobot` / CLI 群 | OK（`lerobot-policy-server`, `lerobot-rollout` など22コマンド） |
| Dataset Streaming（`scripts/stream_test.py`, `lerobot/pusht` 1エピソード） | **パッチ3箇所を当てて OK**。初期化 0.8秒、画像込みで約10サンプル/秒 |
| 動画デコード | torchcodec は Windows で動かず、pyav に自動で切り替わる（警告のみ） |
| ストリーミング学習（`lerobot-train --dataset.streaming=true`） | OK（ACT, CPU, 40 steps, 約1.1〜2秒/step） |
| Remote Inference E2E（同一PC、localhost） | **OK**。下記「Remote Inference E2E」参照 |

## Windows で必要だったローカルパッチ（`.venv` 内、`LOCAL PATCH` で検索可）

どれも POSIX 前提のコードが原因。

1. `lerobot/streaming/manifest.py` 約98行目: `str(Path(...))` → `Path(...).as_posix()`
   （動画の Hub URL が `videos%5C...` となり 404 になっていた）
2. `lerobot/streaming/sidecar_utils.py` の `install_sidecar`: ファイルハンドルを閉じてから `os.replace`
   （Windows は開いたままのファイルを rename できず WinError 32）
3. `lerobot/streaming/episode_parquet.py` 約90行目: `str(relative_path)` → `Path(relative_path).as_posix()`
   （parquet のパスが `data\chunk-000\...` となり FileNotFoundError）

→ upstream に issue/PR を出す価値あり（Linux 環境なら不要なはず）。

## 位置保持（FANUC CRX アダプター設計で重要）

ソースで確認済み: **remote rollout は位置保持（hold）が必須**。

- `lerobot/rollout/context.py` の remote 用セットアップで `_prepare_rollout_devices(cfg, require_hold=True, ...)` を呼ぶ
- `RobotWrapper.configure_hold()` は次のどちらかを満たさないと `ValueError` になる
  - `Robot.supports_command_hold = True` かつ自前の `hold()` を実装
  - `Robot.supports_position_hold = True` かつ **全アクションのキーが `.pos` で終わる**（位置指令のみ。速度・トルク混在は不可）
- 標準で対応しているもの: so_follower, koch_follower, omx_follower, yam_follower, hope_jr, rebot_b601（position）、sim_robot（command）

CRX アダプターの方針: 関節位置指令（`joint_1.pos` ...）で `Robot` を実装し、`supports_position_hold = True` にする。
または CRX 側の停止・保持機能を `hold()` に割り当て、`supports_command_hold = True` にする。

## Remote Inference E2E（`remote_e2e/`）

構成: `lerobot-policy-server`（CPU）⇄ Zenoh `tcp/127.0.0.1:7447` ⇄ `lerobot-rollout --inference.type=remote`（PushT の `sim_robot`）

| ファイル | 内容 |
|---|---|
| `server_pusht.yaml` | サーバー設定（features と action_feature は sim の descriptor をコピー） |
| `dump_descriptor.py` | sim robot の features / semantics を表示 |
| `act_pusht_tiny/` | 配線テスト用の ACT（40 steps しか学習していない。性能は期待しない） |
| `disconnect_test.py` | 実行中にサーバーを kill するフォールトテスト |
| `*.log` | 各実行のログ |

起動手順（ターミナル2つ）:

```bash
.venv/Scripts/lerobot-policy-server.exe --config_path remote_e2e/server_pusht.yaml
```

```bash
.venv/Scripts/lerobot-rollout.exe --strategy.type=base --inference.type=remote --inference.endpoint=tcp/127.0.0.1:7447 --inference.deployment=pusht --inference.semantics=pusht-position-v1 --robot.type=sim --robot.env.type=pusht '--rename_map={"observation.images.image": "observation.image"}' --task="Push the T-shaped block onto the T-shaped target." --duration=20 --fps=10
```

### 正常系（20秒）

- 制御ループは 10.0/10 Hz を維持、over budget 0、starved 0
- turnaround（観測送信 → チャンク受信）平均 約0.13秒、最大 0.175秒（CPU、localhost）
- 1チャンク = 20 actions = 2.0秒分。残り 0.5秒（`refill_seconds`）になると次を要求し、バッファ残量は 0.7〜1.5秒で推移
- 終了時はセッションを解放し、サーバーは次のクライアントを待つ状態に戻る

### 切断系（制御開始8秒後にサーバーを kill）

1. kill 後もクライアントはバッファに残った行動を実行し続けた
2. 約4.2秒後に `Remote action starvation: requesting a local hold` が出る（バッファ枯渇 + `action_starvation_grace_s=1.0`）
3. `ConnectionError: Server presence was lost` で推論を停止し、ローカルで hold → teardown → 終了コード 1
4. **自動再接続はしない**。再開するには rollout を起動し直す
5. ログに `a prior hold command does not guarantee pose retention after disconnect` と出る。切断後に姿勢が保たれるかはロボットドライバー次第

CRX では次の3つの判断が要る: hold で何をするか（停止 or 現在位置保持）、バッファ長 × grace の間にどこまで動いてよいか、終了後のブレーキ / サーボの扱い。

## remote 化で分かった制約・ハマりどころ

- **remote の chunk モードは `n_obs_steps=1` が必須**。`lerobot/diffusion_pusht`（n_obs_steps=2）は配信できない。ACT など n_obs_steps=1 のポリシーを使う
- 古い Hub チェックポイント（例: `lerobot/diffusion_pusht`）はプロセッサ形式への移行が必要。同梱の移行スクリプトは `crop_shape` の list→tuple の問題で `config.json` の保存に失敗するので、`remote_e2e/migrate_diffusion_pusht.py`（回避策のラッパー）を使う
- サーバーの features 名はチェックポイントの入力名（`observation.image`）。クライアントは `--rename_map` で sim / ロボットのカメラ名と対応させる
- Windows: `--help` が cp932 で落ちる → `PYTHONIOENCODING=utf-8` を付ける
- Windows: `lerobot-train` の最後で `checkpoints/last` のシンボリックリンク作成が権限不足で失敗する（モデル本体は保存済み）。開発者モードを有効にするか、無視してよい

## SO-101（メインのロボット）

LeRobot 標準対応なのでアダプターは不要。v0.7.0 の `so_follower` は `supports_position_hold = True` なので、Remote Inference もそのまま使える。

- 準備済み: `lerobot[feetech]` をインストールし、`so101_follower` / `so101_leader` の CLI オプションを確認した
- 未確認: 実機を接続していない（2026-11-16 時点で COM ポートなし）
- 関節キー: `shoulder_pan.pos`, `shoulder_lift.pos`, `elbow_flex.pos`, `wrist_flex.pos`, `wrist_roll.pos`, `gripper.pos`（gripper は 0〜100）

### 実機をつないだらやる順番（Windows、COM 番号は環境ごとに置き換える）

1. ポートを調べる（実行中に USB を抜き差しするよう聞かれる）

```bash
.venv/Scripts/lerobot-find-port.exe
```

2. キャリブレーション（follower と leader それぞれ）

```bash
.venv/Scripts/lerobot-calibrate.exe --robot.type=so101_follower --robot.port=COM3 --robot.id=my_follower
```

```bash
.venv/Scripts/lerobot-calibrate.exe --teleop.type=so101_leader --teleop.port=COM4 --teleop.id=my_leader
```

3. テレオペで動作確認

```bash
.venv/Scripts/lerobot-teleoperate.exe --robot.type=so101_follower --robot.port=COM3 --robot.id=my_follower --teleop.type=so101_leader --teleop.port=COM4 --teleop.id=my_leader
```

4. カメラを調べて、データを収集する（`lerobot-find-cameras opencv` → `lerobot-record`）
5. 学習は RTX 2070 機または GPU サーバーで行う（`--dataset.streaming=true` も使える）
6. GPU 機で `lerobot-policy-server` を起動し、この PC で `lerobot-rollout --inference.type=remote --robot.type=so101_follower ...` を実行する。サーバーの設定は `remote_e2e/server_pusht.yaml` と同じ形で、features を SO-101 のキーに置き換える

注意: Windows では `PYTHONIOENCODING=utf-8` を付けないと `--help` が落ちる。

## FANUC CRX アダプター（`lerobot_robot_fanuc_crx/`）— 保留

メインを SO-101 にしたので保留。コードとテストは残してある（`.venv` に editable インストール済み）。

- `--robot.type=fanuc_crx` で LeRobot の CLI から使える（プラグインとして自動で読み込まれる。`lerobot-rollout --help` で確認済み）
- 関節位置指令（`joint_1.pos`〜`joint_6.pos`、度）で、hold は `position` と `command` を選べる
- モックドライバーでテスト 8/8 合格。remote rollout と同じ `ThreadSafeRobot.configure_hold()` / `hold()` の契約も通る
- 実機用の `ExistingSystemCRXDriver` は未実装（既存の接続システムに合わせて実装する）。詳細は `lerobot_robot_fanuc_crx/README.md`

## 次の候補

- RTX 2070 機（Linux / CUDA 13 / ドライバー要件を要確認）で policy server を立て、この PC をクライアントにする（LAN 越しの遅延を測る）
- SO-101 を接続して、キャリブレーション → テレオペ → データ収集まで進める
- （保留）CRX を使うことになったら `ExistingSystemCRXDriver` を実装する
