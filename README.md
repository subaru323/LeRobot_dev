# LeRobot_dev — SO-ARM101 開発用フォーク

このリポジトリは [huggingface/lerobot](https://github.com/huggingface/lerobot)（version **0.6.2**）をベースにした、SO-ARM101（Seeed Studio キット）向けの開発用フォークです。
本家のコードには手を入れず、Windows 開発環境の設定・補助スクリプト・解説資料を追加しています。

> **このセクションの位置づけ**
> このファイルの先頭から「本家 README（ここから下は upstream のまま）」の見出しまでが、このフォーク独自のセットアップ手順です。
> それより下は本家 LeRobot の README をそのまま残しています。
> 手順は **2026-10-15 時点、Windows 11 + uv 0.12.5 + Python 3.12.14** の実機で確認しました。
> 確認できていない手順には **（未検証）** と明記しています。

---

## 目次

1. [このフォークで追加したもの](#1-このフォークで追加したもの)
2. [必要なもの](#2-必要なもの)
3. [リポジトリの取得（clone）](#3-リポジトリの取得clone)
4. [Python 環境の構築（uv）](#4-python-環境の構築uv)
5. [Windows で必ず行う設定](#5-windows-で必ず行う設定)
6. [動作確認（ロボット無し）](#6-動作確認ロボット無し)
7. [GPU を使う場合](#7-gpu-を使う場合)
8. [VS Code の設定](#8-vs-code-の設定)
9. [SO-ARM101 の初回セットアップ（ロボット有り）](#9-so-arm101-の初回セットアップロボット有り)
10. [データ収集・学習・実行の最小コマンド](#10-データ収集学習実行の最小コマンド)
11. [トラブルシューティング](#11-トラブルシューティング)
12. [本家（upstream）の更新を取り込む](#12-本家upstreamの更新を取り込む)
13. [AI エージェント向けの要点](#13-ai-エージェント向けの要点)

---

## 1. このフォークで追加したもの

| パス | 内容 |
| --- | --- |
| `.vscode/settings.json` | `.venv` の Python を既定にし、pytest と ruff を有効にする設定 |
| `.vscode/launch.json` | よく使うコマンドのデバッグ起動設定（info、データセット可視化、CPU での学習スモークテスト、評価、ポート検出など） |
| `.vscode/extensions.json` | 推奨拡張（Python、debugpy、Ruff） |
| `dev_scripts/random_servo.py` | Feetech サーボを、起動時の姿勢の周りでランダムに動かす単体テスト用スクリプト |
| `dev_docs/LeRobot_system_map_detailed.html` | LeRobot × SO-ARM101 のデータの流れを示す対話型の図（ブラウザで開く） |

本家のソースコード（`src/lerobot/`）は変更していません。

---

## 2. 必要なもの

### ソフトウェア

| もの | 確認したバージョン | 備考 |
| --- | --- | --- |
| OS | Windows 11 | Linux / macOS でも動きますが、この手順は Windows 前提です |
| Git | 2.52 | |
| Git LFS | 3.7.1 | テスト用データの取得に必要（下記参照） |
| uv | 0.12.5 | Python とパッケージの管理ツール |
| Python | 3.12.14 | uv が自動で用意します。**3.12 以上が必須**（`pyproject.toml`） |

### ハードウェア（ロボットを動かす場合）

| もの | 内容 |
| --- | --- |
| SO-ARM101 フォロワー腕 | Feetech STS3215 サーボ × 6（全関節 ギア比 1/345） |
| SO-ARM101 リーダー腕 | STS3215 × 6（関節 1・3 が 1/191、関節 2 が 1/345、関節 4〜6 が 1/147） |
| バスサーボ アダプタ基板 | 1 本の腕につき 1 枚。この PC では `USB-Enhanced-SERIAL CH343` として認識されます |
| 電源 | 標準版は 5V。Pro 版はフォロワーのみ 12V。**USB からは給電されません** |
| カメラ（任意） | USB カメラ（UVC）。データ収集と方策の実行に使います |

---

## 3. リポジトリの取得（clone）

### 3-1. Git LFS を有効にする（初回のみ）

```bash
git lfs install
```

### 3-2. clone する

```bash
git clone https://github.com/subaru323/LeRobot_dev.git
cd LeRobot_dev
```

- 本家の履歴を全部含むため、LFS のテスト用データが **約 3.5GB** あります。
- テストを実行しない、または回線が遅い場合は、LFS の実体を取らずに clone できます（ファイルは小さな「ポインタ」のまま残ります）。

```bash
GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/subaru323/LeRobot_dev.git
```

PowerShell の場合は次のとおりです。

```powershell
$env:GIT_LFS_SKIP_SMUDGE = "1"; git clone https://github.com/subaru323/LeRobot_dev.git
```

### 3-3. 本家を upstream として登録する

本家の更新を取り込むために使います（[12 章](#12-本家upstreamの更新を取り込む)）。

```bash
git remote add upstream https://github.com/huggingface/lerobot.git
```

### 3-4. 置き場所について（推奨）

- フォルダのパスに日本語や全角文字を含めない方が無難です（例：`C:\dev\LeRobot_dev`）。
- 開発者の PC（`C:\２年後期-osaka high tech\...`）では、日本語を含むパスでも動作を確認しています。ただし、Windows の文字コード（cp932）との組み合わせで、エラーの表示が文字化けすることがあります。

---

## 4. Python 環境の構築（uv）

### 4-1. uv を入れる（初回のみ）

PowerShell で実行します。

```powershell
powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
```

入れた後、ターミナルを開き直して `uv --version` が表示されることを確認してください。

### 4-2. 依存パッケージを入れる

リポジトリのフォルダ（`pyproject.toml` がある場所）で実行します。

```bash
uv sync --locked --extra core_scripts --extra feetech --extra training --extra pusht --extra test
```

- `.venv/` フォルダに仮想環境が作られます。Python 3.12 が無ければ uv が自動で取得します。
- `--locked` は `uv.lock` のバージョンをそのまま使う指定です。全員が同じ環境になります。
- 開発者の PC の `.venv` は、この指定と完全に一致することを確認しています（`uv sync --dry-run` で「変更なし」）。

| extra | 入るもの | 必要な場面 |
| --- | --- | --- |
| `core_scripts` | データセット、pynput（キー入力）、pyserial、Rerun / Foxglove（可視化） | 記録・再生・校正・遠隔操作 |
| `feetech` | `feetech-servo-sdk`（`import scservo_sdk`） | SO-ARM101 のサーボ通信 |
| `training` | wandb、accelerate | 学習 |
| `pusht` | `gym-pusht` | ロボット無しの学習・評価の練習 |
| `test` | pytest など | テストの実行 |

ほかの方策（SmolVLA や Pi0 など）を使うときは、`--extra smolvla` や `--extra pi` を足します。一覧は `pyproject.toml` の `[project.optional-dependencies]` にあります。

### 4-3. 仮想環境の使い方

次のどちらかで、`.venv` の中のコマンドを使います。

```bash
uv run lerobot-info
```

```powershell
.venv\Scripts\Activate.ps1
lerobot-info
```

以降の手順では、仮想環境を有効にした前提で `lerobot-xxx` と書きます。

---

## 5. Windows で必ず行う設定

### 5-1. UTF-8 モードにする（必須）

設定しないと、`lerobot-record --help` や `lerobot-train --help` が次のエラーで止まります（実際に再現しました）。

```text
UnicodeEncodeError: 'cp932' codec can't encode character '\u2265' ...
```

PowerShell で、ユーザーの環境変数として登録します（一度だけ）。

```powershell
[Environment]::SetEnvironmentVariable("PYTHONUTF8", "1", "User")
```

登録後はターミナル（と VS Code）を開き直してください。その場だけで試すなら次のとおりです。

```powershell
$env:PYTHONUTF8 = "1"
```

### 5-2. FFmpeg（共有 DLL 版）を入れる（推奨・未検証）

動画の読み込みに使う `torchcodec` は、Windows では FFmpeg の **full-shared 版（DLL 付き）** が必要です。無いと次の状態になります。

- `lerobot-info` が `RuntimeError: Could not load libtorchcodec` で**停止**します（実際に再現しました）。
- `lerobot-train` などは警告を出して、遅い方の `pyav` に自動で切り替わり、**動作は続きます**（確認済み）。

対処の候補は次のとおりです（**未検証**）。

```powershell
winget install --id Gyan.FFmpeg.Shared
```

入れた後、FFmpeg の `bin` フォルダ（DLL がある場所）が PATH に入っているか確認し、ターミナルを開き直してから `lerobot-info` を再実行してください。
torchcodec が対応する FFmpeg は 4〜8 です。

FFmpeg を入れない場合は、学習時に `--dataset.video_backend=pyav` を付けると、警告なしで PyAV を使います。

### 5-3. Windows のシンボリックリンク

学習のチェックポイントには、最新を指す `checkpoints/last` というリンクが作られます。Windows で「開発者モード」が無効だと、このリンクが作れません。
そのため、`launch.json` の評価設定では `checkpoints/last` ではなく `checkpoints/000200` のように**ステップ番号を明示**しています。

---

## 6. 動作確認（ロボット無し）

### 6-1. 環境情報

```bash
lerobot-info
```

LeRobot・Python・PyTorch のバージョンと GPU の有無が表示されれば成功です。
torchcodec のエラーで止まる場合は [5-2](#5-2-ffmpeg共有-dll-版を入れる推奨未検証) を参照してください。

### 6-2. CLI が起動するか

```bash
lerobot-calibrate --help
lerobot-record --help
lerobot-train --help
```

開発者の PC では、`PYTHONUTF8=1` の状態で、上の 3 つと `lerobot-setup-motors`、`lerobot-teleoperate` の `--help` がすべて正常に終了することを確認しました。

### 6-3. CPU だけで学習の流れを試す（スモークテスト）

PushT（2D のシミュレーション）のデータで ACT を 200 ステップだけ学習します。VS Code の起動設定「LeRobot: train ACT on pusht (CPU smoke)」と同じ内容です。

```bash
lerobot-train --policy.type=act --policy.device=cpu --policy.push_to_hub=false --dataset.repo_id=lerobot/pusht --env.type=pusht --steps=200 --batch_size=8 --env_eval_freq=0 --num_workers=0 --save_freq=200 --output_dir=outputs/train/act_pusht_smoke --wandb.enable=false
```

- 初回はデータセット `lerobot/pusht` を Hugging Face Hub からダウンロードします。
- 結果は `outputs/train/act_pusht_smoke/checkpoints/000200/pretrained_model/` に保存されます。

続けて評価する場合は次のとおりです。

```bash
lerobot-eval --policy.path=outputs/train/act_pusht_smoke/checkpoints/000200/pretrained_model --policy.device=cpu --env.type=pusht --eval.n_episodes=2 --eval.batch_size=1
```

> この 2 つのコマンドは、README 作成時には実行していません（**未検証**）。

---

## 7. GPU を使う場合

`uv sync` の既定では、**Windows には CPU 版の PyTorch が入ります**。開発者の PC でも `torch 2.11.0+cpu` です。
`pyproject.toml` は、CUDA 版の取得先（cu128）を Linux にだけ指定しているためです。

NVIDIA GPU で学習する場合は、`uv sync` の後に CUDA 版で入れ直します（**未検証**）。

```bash
uv pip install --force-reinstall torch torchvision --index-url https://download.pytorch.org/whl/cu128
```

- GPU ドライバが 570.86 以上である必要があります（本家 docs）。
- **注意**：もう一度 `uv sync` を実行すると、CPU 版に戻ります。その場合は上のコマンドを再実行してください。
- 確認方法は次のとおりです。

```bash
python -c "import torch; print(torch.__version__, torch.cuda.is_available())"
```

`True` と表示されれば GPU が使えます。学習コマンドでは `--policy.device=cuda` を指定します。

---

## 8. VS Code の設定

1. VS Code でリポジトリのフォルダを開きます。
2. 右下の通知、または拡張機能の画面から、推奨拡張（Python、Python Debugger、Ruff）を入れます。
3. Python のインタープリタは自動で `.venv\Scripts\python.exe` になります。
4. 「実行とデバッグ」から、次の起動設定を選べます。

| 起動設定の名前 | 内容 |
| --- | --- |
| LeRobot: info | `lerobot-info` |
| LeRobot: dataset viz (pusht ep0) | PushT データセットの 0 番目のエピソードを Rerun で表示 |
| LeRobot: train ACT on pusht (CPU smoke) | [6-3](#6-3-cpu-だけで学習の流れを試すスモークテスト) の学習 |
| LeRobot: eval smoke checkpoint on pusht | 上の学習結果を評価 |
| LeRobot: find port (robot USB) | `lerobot-find-port` |
| Python: current file | 開いているファイルを実行 |
| Pytest: current file | 開いているテストファイルを実行 |

> `PYTHONUTF8` を環境変数に登録した場合、VS Code を完全に終了してから開き直さないと反映されません。

---

## 9. SO-ARM101 の初回セットアップ（ロボット有り）

ここからは Windows での手順です。公式・Seeed の手順書は Linux（`/dev/ttyACM0`）前提のため、`--robot.port` などは **`COM19` のような COM 番号に読み替えて**ください。
Windows で実機を最後まで動かした確認は、まだ取れていません（**未検証**）。

### 9-1. 配線と電源

1. サーボを数珠つなぎ（デイジーチェーン）にし、関節 1（肩の回転、ID 1）側のケーブルをアダプタ基板に挿します。
2. **電源**を基板につなぎます。USB だけではサーボは動きません。
   - 標準版：リーダー・フォロワーとも 5V
   - Pro 版：リーダー 5V、フォロワー 12V（取り違えるとモーターが壊れます）
3. 基板を USB で PC につなぎます。USB ハブを経由せず、PC に直接挿すことを推奨します。
4. Waveshare 製の基板の場合は、**2 つのジャンパを `B`（USB）側**にします（本家 docs `docs/source/so101.mdx`）。

### 9-2. ポート（COM 番号）を調べる

```bash
lerobot-find-port
```

画面の指示に従い、腕の USB を抜いて Enter を押すと、消えたポートが表示されます。2 本の腕で 1 回ずつ行います。
デバイスマネージャーの「ポート (COM と LPT)」でも確認できます。開発者の PC では `COM19`（CH343）でした。

### 9-3. モーターに ID を書き込む（新品のときだけ）

新品のサーボは ID がすべて 1 なので、1 個ずつつないで ID 1〜6 と通信速度（1,000,000 bps）を書き込みます。

```bash
lerobot-setup-motors --robot.type=so101_follower --robot.port=COM19
lerobot-setup-motors --teleop.type=so101_leader --teleop.port=COM20
```

- 画面の指示に従い、**グリッパー（ID 6）から順に、その 1 個だけ**を基板につないで Enter を押します。
- `COM19` と `COM20` は例です。9-2 で調べた番号にしてください。

### 9-4. 校正（キャリブレーション）

```bash
lerobot-calibrate --robot.type=so101_follower --robot.port=COM19 --robot.id=my_follower
lerobot-calibrate --teleop.type=so101_leader --teleop.port=COM20 --teleop.id=my_leader
```

1. 画面の図のように、**すべての関節を可動域の真ん中**に置いて Enter を押します。
2. 手首の回転（`wrist_roll`）以外の関節を、**端から端まで**ゆっくり動かし、Enter で終了します。
3. 結果は次の場所に保存されます。

```text
%USERPROFILE%\.cache\huggingface\lerobot\calibration\robots\so_follower\my_follower.json
%USERPROFILE%\.cache\huggingface\lerobot\calibration\teleoperators\so_leader\my_leader.json
```

- **`--robot.id` と `--teleop.id` は、以後のすべてのコマンドで同じ値を使います。**
- やり直す場合は、校正コマンドを再実行して、プロンプトで `c` を入力します。

### 9-5. 遠隔操作で動きを確かめる

```bash
lerobot-teleoperate --robot.type=so101_follower --robot.port=COM19 --robot.id=my_follower --teleop.type=so101_leader --teleop.port=COM20 --teleop.id=my_leader
```

リーダーを手で動かすと、フォロワーが同じように動けば成功です。止めるときは `Ctrl + C` を押します。

### 9-6. サーボ単体の動作確認（補助スクリプト）

LeRobot を通さず、サーボだけが通信・動作するかを確かめます。腕が動きますので、周りに物や人が無いことを確認してから実行してください。

```bash
python dev_scripts/random_servo.py --port COM19
python dev_scripts/random_servo.py --port COM19 --ids 1 2 3 --amplitude 300 --cycles 20
```

| 引数 | 既定値 | 意味 |
| --- | --- | --- |
| `--port` | （必須） | COM 番号 |
| `--baudrate` | 1000000 | 通信速度 |
| `--ids` | 0〜`--max-id` を探索 | 動かすサーボの ID |
| `--max-id` | 20 | 探索する ID の上限 |
| `--amplitude` | 200 | 起動時の位置からの最大の振れ幅（4096 = 360°） |
| `--speed` | 600 | 目標速度（ticks/秒） |
| `--acceleration` | 20 | 加速度 |
| `--cycles` | 10 | 動かす回数 |
| `--interval` | 1.0 | 目標を変える間隔（秒） |
| `--seed` | なし | 乱数の種 |

終了時（`Ctrl + C` を含む）は、元の姿勢に戻ってからトルクを切ります。

---

## 10. データ収集・学習・実行の最小コマンド

詳しい流れは `dev_docs/LeRobot_system_map_detailed.html` をブラウザで開いて確認できます。

### 10-1. Hugging Face にログインする（Hub へ上げる場合）

```bash
hf auth login
```

（旧コマンドは `huggingface-cli login` です。）

### 10-2. データ収集

```bash
lerobot-record --robot.type=so101_follower --robot.port=COM19 --robot.id=my_follower --robot.cameras="{front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}}" --teleop.type=so101_leader --teleop.port=COM20 --teleop.id=my_leader --dataset.repo_id=<HFユーザー名>/so101_test --dataset.num_episodes=5 --dataset.single_task="Grab the black cube" --dataset.episode_time_s=30 --dataset.reset_time_s=30 --dataset.push_to_hub=false
```

- キー操作：`→` でエピソードを早く終える、`←` で撮り直し、`Esc` で記録を終了します。
- 保存先：`%USERPROFILE%\.cache\huggingface\lerobot\<repo_id>`
- 既定値：fps 30、1 エピソード 60 秒、リセット 60 秒、50 エピソード、Hub へ自動アップロード（上の例では無効にしています）。

### 10-3. 学習

```bash
lerobot-train --dataset.repo_id=<HFユーザー名>/so101_test --policy.type=act --output_dir=outputs/train/act_so101_test --policy.device=cuda --policy.push_to_hub=false --wandb.enable=false
```

- 既定は 100,000 ステップ、バッチサイズ 8、20,000 ステップごとに保存です。
- GPU が無い場合は `--policy.device=cpu` にします（非常に遅くなります）。

### 10-4. 実機で方策を動かす

```bash
lerobot-rollout --strategy.type=base --policy.path=outputs/train/act_so101_test/checkpoints/100000/pretrained_model --robot.type=so101_follower --robot.port=COM19 --robot.id=my_follower --robot.cameras="{front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}}" --task="Grab the black cube" --duration=60
```

- **カメラの名前（例：`front`）と取り付け位置は、記録したときと同じ**にしてください。
- Seeed Wiki には `lerobot-record --policy.path=...` で評価する手順が載っていますが、このバージョンでは `lerobot-record` に `--teleop` が必須で、方策の実行には `lerobot-rollout` を使います。

---

## 11. トラブルシューティング

| 症状 | 原因と対処 |
| --- | --- |
| `UnicodeEncodeError: 'cp932' codec can't encode ...` | Windows の文字コードの問題です。`PYTHONUTF8=1` を設定します（[5-1](#5-1-utf-8-モードにする必須)）。 |
| `RuntimeError: Could not load libtorchcodec` | FFmpeg の共有 DLL がありません（[5-2](#5-2-ffmpeg共有-dll-版を入れる推奨未検証)）。学習は `pyav` に自動で切り替わって動きます。 |
| `torch.cuda.is_available()` が `False` | Windows には CPU 版の PyTorch が入ります（[7 章](#7-gpu-を使う場合)）。 |
| サーボに ping しても応答が 0 バイト | 電源がつながっているか、基板のジャンパが `B` か、ケーブルが奥まで入っているかを確認します。基板の隣の 1 台だけをつないで試します。 |
| `Motor 'gripper' was not found` | 電源・配線を確認します（Seeed Wiki）。 |
| `Magnitude 30841 exceeds 2047` など | 校正値が範囲外です。電源を入れ直して再校正します（Seeed Wiki）。 |
| 一部のサーボの LED が消えている | その手前のケーブルが緩んでいます（Seeed Wiki）。 |
| キー操作（→ ← Esc）が効かない | 記録中のウィンドウにフォーカスがあるか確認します。 |
| `checkpoints/last` が無い | Windows ではリンクが作れないことがあります。`checkpoints/<ステップ番号>` を直接指定します（[5-3](#5-3-windows-のシンボリックリンク)）。 |
| `git push` で 403 | push するアカウントにリポジトリの書き込み権限がありません。リポジトリの Collaborator に追加し、招待を承認します。 |
| `git push` で `Git LFS upload failed ... (missing)` | LFS の実体がローカルにありません。`git lfs fetch --all upstream` で取得してから push します。 |

---

## 12. 本家（upstream）の更新を取り込む

```bash
git fetch upstream
git merge upstream/main
git push origin main
```

- この README の先頭部分は本家と内容が違うため、本家が README を更新すると**マージ時に衝突**することがあります。その場合は、このフォークの部分（この章まで）を残し、下の本家部分を本家の最新に置き換えてください。
- 本家に新しい LFS ファイルが追加された場合、push の前に `git lfs fetch upstream main` が必要になることがあります。

---

## 13. AI エージェント向けの要点

このリポジトリで作業する AI エージェント（Claude Code など）は、次の事実を前提にしてください。

- **ベース**：huggingface/lerobot 0.6.2。`src/lerobot/` は本家のまま。独自の追加は `.vscode/`、`dev_scripts/`、`dev_docs/` とこの README の先頭部分のみ。
- **開発規約**：本家の `AGENTS.md`（`CLAUDE.md` から参照）に従う。
- **OS**：Windows 11。シェルは PowerShell または Git Bash。ポートは `COMxx`（`/dev/ttyACM*` ではない）。
- **Python**：`.venv\Scripts\python.exe`（3.12.14）。依存は `uv sync --locked --extra core_scripts --extra feetech --extra training --extra pusht --extra test` で再現できる。
- **PyTorch**：`2.11.0+cpu`（CUDA 無し）。
- **環境変数**：`PYTHONUTF8=1` が無いと、CLI の `--help` が cp932 で落ちる。
- **torchcodec**：FFmpeg の DLL が無く読み込めない。`lerobot-info` は停止し、データセットの読み込みは `pyav` へ自動で切り替わる。
- **ロボット**：SO-ARM101。`--robot.type=so101_follower` / `--teleop.type=so101_leader`。サーボは Feetech STS3215 × 6、ID 1〜6、1,000,000 bps、Feetech プロトコル 0。
- **シリアル**：アダプタは CH343（VID 1A86 / PID 55D3）、開発者の PC では `COM19`。
- **校正ファイル**：`%USERPROFILE%\.cache\huggingface\lerobot\calibration\{robots\so_follower|teleoperators\so_leader}\{id}.json`。
- **非同期推論**：本家 0.6.2 には `lerobot.async_inference` が無い。`lerobot-policy-server` と `lerobot-rollout --inference.type=remote`（Zenoh）を使う。Seeed Wiki の async_inference 手順は Seeed のフォーク向け。
- **実機の評価**：`lerobot-rollout` を使う（`lerobot-record` は `--teleop` が必須）。
- **Git**：`origin` = subaru323/LeRobot_dev、`upstream` = huggingface/lerobot。LFS 使用（テスト用データ）。

---
---

# 本家 README（ここから下は upstream のまま）


<p align="center">
  <img alt="LeRobot, Hugging Face Robotics Library" src="./media/readme/lerobot-logo-thumbnail.png" width="100%">
</p>

<div align="center">

[![Tests](https://github.com/huggingface/lerobot/actions/workflows/latest_deps_tests.yml/badge.svg?branch=main)](https://github.com/huggingface/lerobot/actions/workflows/latest_deps_tests.yml?query=branch%3Amain)
[![Tests](https://github.com/huggingface/lerobot/actions/workflows/docker_publish.yml/badge.svg?branch=main)](https://github.com/huggingface/lerobot/actions/workflows/docker_publish.yml?query=branch%3Amain)
[![Python versions](https://img.shields.io/pypi/pyversions/lerobot)](https://www.python.org/downloads/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://github.com/huggingface/lerobot/blob/main/LICENSE)
[![Status](https://img.shields.io/pypi/status/lerobot)](https://pypi.org/project/lerobot/)
[![Version](https://img.shields.io/pypi/v/lerobot)](https://pypi.org/project/lerobot/)
[![Contributor Covenant](https://img.shields.io/badge/Contributor%20Covenant-v2.1-ff69b4.svg)](https://github.com/huggingface/lerobot/blob/main/CODE_OF_CONDUCT.md)
[![Discord](https://img.shields.io/badge/Discord-Join_Us-5865F2?style=flat&logo=discord&logoColor=white)](https://discord.gg/q8Dzzpym3f)

</div>

**LeRobot** aims to provide models, datasets, and tools for real-world robotics in PyTorch. The goal is to lower the barrier to entry so that everyone can contribute to and benefit from shared datasets and pretrained models.

🤗 A hardware-agnostic, Python-native interface that standardizes control across diverse platforms, from low-cost arms (SO-100) to humanoids.

🤗 A standardized, scalable LeRobotDataset format (Parquet + MP4 or images) hosted on the Hugging Face Hub, enabling efficient storage, streaming and visualization of massive robotic datasets.

🤗 State-of-the-art policies that have been shown to transfer to the real-world ready for training and deployment.

🤗 Comprehensive support for the open-source ecosystem to democratize physical AI.

## Quick Start

LeRobot can be installed directly from PyPI.

```bash
pip install lerobot
lerobot-info
```

> [!IMPORTANT]
> For detailed installation guide, please see the [Installation Documentation](https://huggingface.co/docs/lerobot/installation).

## Robots & Control

<div align="center">
  <img src="./media/readme/robots_control_video.webp" width="640px" alt="Reachy 2 Demo">
</div>

LeRobot provides a unified `Robot` class interface that decouples control logic from hardware specifics. It supports a wide range of robots and teleoperation devices.

```python
from lerobot.robots.myrobot import MyRobot

# Connect to a robot
robot = MyRobot(config=...)
robot.connect()

# Read observation and send action
obs = robot.get_observation()
action = model.select_action(obs)
robot.send_action(action)
```

**Supported Hardware:** SO100, LeKiwi, Koch, HopeJR, OMX, EarthRover, Reachy2, Gamepads, Keyboards, Phones, OpenARM, Unitree G1, reBot B601.

While these devices are natively integrated into the LeRobot codebase, the library is designed to be extensible. You can easily implement the Robot interface to utilize LeRobot's data collection, training, and visualization tools for your own custom robot.

For detailed hardware setup guides, see the [Hardware Documentation](https://huggingface.co/docs/lerobot/integrate_hardware).

## LeRobot Dataset

To solve the data fragmentation problem in robotics, we utilize the **LeRobotDataset** format.

- **Structure:** Synchronized MP4 videos (or images) for vision and Parquet files for state/action data.
- **HF Hub Integration:** Explore thousands of robotics datasets on the [Hugging Face Hub](https://huggingface.co/lerobot).
- **Tools:** Seamlessly delete episodes, split by indices/fractions, add/remove features, and merge multiple datasets.

```python
from lerobot.datasets.lerobot_dataset import LeRobotDataset

# Load a dataset from the Hub
dataset = LeRobotDataset("lerobot/aloha_mobile_cabinet")

# Access data (automatically handles video decoding)
episode_index=0
print(f"{dataset[episode_index]['action'].shape=}\n")
```

Learn more about it in the [LeRobotDataset Documentation](https://huggingface.co/docs/lerobot/lerobot-dataset-v3).

## SoTA Models

LeRobot implements state-of-the-art policies in pure PyTorch, covering Imitation Learning, Reinforcement Learning, Vision-Language-Action (VLA) models, World Models, and Reward Models, with more coming soon. It also provides you with the tools to instrument and inspect your training process.

<p align="center">
  <img alt="Gr00t Architecture" src="./media/readme/VLA_architecture.jpg" width="640px">
</p>

Training a policy is as simple as running a script configuration:

```bash
lerobot-train \
  --policy.type=act \
  --dataset.repo_id=lerobot/aloha_mobile_cabinet
```

| Category                   | Models                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| -------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| **Imitation Learning**     | [ACT](./docs/source/policy_act_README.md), [Diffusion](./docs/source/policy_diffusion_README.md), [VQ-BeT](./docs/source/policy_vqbet_README.md), [Multitask DiT Policy](./docs/source/policy_multi_task_dit_README.md)                                                                                                                                                                                                                                                  |
| **Reinforcement Learning** | [HIL-SERL](./docs/source/hilserl.mdx), [TDMPC](./docs/source/policy_tdmpc_README.md) & QC-FQL (coming soon)                                                                                                                                                                                                                                                                                                                                                              |
| **VLAs Models**            | [Pi0](./docs/source/pi0.mdx), [Pi0Fast](./docs/source/pi0fast.mdx), [Pi0.5](./docs/source/pi05.mdx), [FineART-VLA](./docs/source/fineart_vla.mdx), [GR00T N1.7](./docs/source/policy_groot_README.md), [SmolVLA](./docs/source/policy_smolvla_README.md), [XVLA](./docs/source/xvla.mdx), [EO-1](./docs/source/eo1.mdx), [MolmoAct2](./docs/source/molmoact2.mdx), [WALL-OSS](./docs/source/walloss.mdx), [EVO1](./docs/source/evo1.mdx), [DM05](./docs/source/dm05.mdx) |
| **World Models**           | [VLA-JEPA](./docs/source/vla_jepa.mdx), [LingBot-VA](./docs/source/lingbot_va.mdx), [FastWAM](./docs/source/fastwam.mdx), [LaWAM](./docs/source/lawam.mdx), [FLUX 3 Action](./docs/source/flux3.mdx)                                                                                                                                                                                                                                                                     |
| **Reward Models**          | [SARM](./docs/source/sarm.mdx), [TOPReward](./docs/source/topreward.mdx), [Robometer](./docs/source/robometer.mdx)                                                                                                                                                                                                                                                                                                                                                       |

Similarly to the hardware, you can easily implement your own policy & leverage LeRobot's data collection, training, and visualization tools, and share your model to the HF Hub.

For detailed policy setup guides, see the [Policy Documentation](https://huggingface.co/docs/lerobot/bring_your_own_policies). For GPU/RAM requirements and expected training time per policy, see the [Compute Hardware Guide](https://huggingface.co/docs/lerobot/hardware_guide).

## Inference & Evaluation

Evaluate your policies in simulation or on real hardware using the unified evaluation script. LeRobot supports standard benchmarks like **LIBERO**, **MetaWorld** and more to come.

```bash
# Evaluate a policy on the LIBERO benchmark
lerobot-eval \
  --policy.path=lerobot/pi0_libero_finetuned \
  --env.type=libero \
  --env.task=libero_object \
  --eval.n_episodes=10
```

Learn how to implement your own simulation environment or benchmark and distribute it from the HF Hub by following the [EnvHub Documentation](https://huggingface.co/docs/lerobot/envhub).

### Third-Party Hardware

Beyond the natively supported hardware, the community maintains a growing ecosystem of plugins for other robots, teleoperators, cameras, and sensors - UFACTORY xArm, Universal Robots UR5e, Franka, AgileX Piper, Trossen WidowX, ARX5, I2RT YAM, GELLO, SpaceMouse, Meta Quest, ROS 2 bridges, tactile and depth cameras, and more.

Plugins are auto-discovered by package name: LeRobot imports any installed package prefixed with `lerobot_robot_`, `lerobot_teleoperator_`, or `lerobot_camera_`. Install one and use the `type` it registers straight from the CLI:

```bash
pip install lerobot_robot_<name> lerobot_teleoperator_<name>

lerobot-record \
  --robot.type=<robot_name> \
  --teleop.type=<teleoperator_name> \
  --dataset.repo_id=${HF_USER}/my-dataset
```

Browse the full list in the [Third-Party Robots & Teleoperators](https://huggingface.co/docs/lerobot/main/third_party_robots) and [Third-Party Cameras & Sensors](https://huggingface.co/docs/lerobot/main/third_party_sensors) documentation.

## Resources

- **[Documentation](https://huggingface.co/docs/lerobot/index):** The complete guide to tutorials & API.
- **[Chinese Tutorials: LeRobot+SO-ARM101中文教程-同济子豪兄](https://zihao-ai.feishu.cn/wiki/space/7589642043471924447)** Detailed doc for assembling, teleoperate, dataset, train, deploy. Verified by Seed Studio and 5 global hackathon players.
- **[Discord](https://discord.gg/q8Dzzpym3f):** Join the `LeRobot` server to discuss with the community.
- **[X](https://x.com/LeRobotHF):** Follow us on X to stay up-to-date with the latest developments.
- **[Robot Learning Tutorial](https://huggingface.co/spaces/lerobot/robot-learning-tutorial):** A free, hands-on course to learn robot learning using LeRobot.
- **[T-Shirt Folding Experiment](https://huggingface.co/spaces/lerobot/robot-folding):** An end-to-end demonstration of folding t-shirts with LeRobot.
- **[LeLab](https://github.com/huggingface/leLab):** A web interface for LeRobot — teleoperate, calibrate, record datasets, replay, and train your SO arm from the browser, no CLI required.

## Citation

If you use LeRobot in your project, please cite the GitHub repository to acknowledge the ongoing development and contributors:

```bibtex
@misc{cadene2024lerobot,
    author = {Cadene, Remi and Alibert, Simon and Soare, Alexander and Gallouedec, Quentin and Zouitine, Adil and Palma, Steven and Kooijmans, Pepijn and Aractingi, Michel and Shukor, Mustafa and Aubakirova, Dana and Russi, Martino and Capuano, Francesco and Pascal, Caroline and Choghari, Jade and Meftah, Khalil and Ellerbach, Maxime and Moss, Jess and Wolf, Thomas},
    title = {LeRobot: State-of-the-art Machine Learning for Real-World Robotics in Pytorch},
    howpublished = "\url{https://github.com/huggingface/lerobot}",
    year = {2024}
}
```

If you are referencing our research or the academic paper, please also cite our ICLR publication:

<details>
<summary><b>ICLR 2026 Paper</b></summary>

```bibtex
@inproceedings{cadenelerobot,
  title={LeRobot: An Open-Source Library for End-to-End Robot Learning},
  author={Cadene, Remi and Alibert, Simon and Capuano, Francesco and Aractingi, Michel and Zouitine, Adil and Kooijmans, Pepijn and Choghari, Jade and Russi, Martino and Pascal, Caroline and Palma, Steven and Shukor, Mustafa and Moss, Jess and Soare, Alexander and Aubakirova, Dana and Lhoest, Quentin and Gallou\'edec, Quentin and Wolf, Thomas},
  booktitle={The Fourteenth International Conference on Learning Representations},
  year={2026},
  url={https://arxiv.org/abs/2602.22818}
}
```

</details>

## Contribute

We welcome contributions from everyone in the community! To get started, please read our [CONTRIBUTING.md](https://github.com/huggingface/lerobot/blob/main/CONTRIBUTING.md) guide. Whether you're adding a new feature, improving documentation, or fixing a bug, your help and feedback are invaluable. We're incredibly excited about the future of open-source robotics and can't wait to work with you on what's next—thank you for your support!

<p align="center">
  <img alt="SO101 Video" src="./media/readme/so100_video.webp" width="640px">
</p>

<div align="center">
<sub>Built by the <a href="https://huggingface.co/lerobot">LeRobot</a> team at <a href="https://huggingface.co">Hugging Face</a> with ❤️</sub>
</div>
