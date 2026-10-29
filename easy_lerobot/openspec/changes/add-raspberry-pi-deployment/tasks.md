# Tasks

## 1. ラズパイの準備（A 班・E 班）

- [ ] 1.1 ラズパイ 5 に Raspberry Pi OS（64 ビット）を入れ、SSH と mDNS で `ssh <利用者>@<ホスト名>.local` が通ることを確認する。手順を `easy_lerobot/deploy/README.md` に書く
- [ ] 1.2 uv と Python 3.12 を入れ、`easy_lerobot` で `uv sync --locked` を実行する。`uv run python -c "import torch, torchcodec, lerobot"` が通ることで確認する。通らなければ原因を design.md の Risks に追記し、依存の版を変える変更提案を起こす（依存の変更は承認後）
- [ ] 1.3 利用者を `dialout` と `video` のグループに入れ、手打ちで `lerobot-find-port`・`lerobot-calibrate`・`lerobot-teleoperate`・`lerobot-record`（1 エピソード）を実行する。録ったデータセットを `lerobot-dataset-viz` で開けることで確認する

## 2. 推論速度の計測（C 班・D 班）

- [ ] 2.1 学習済みの ACT（実データがなければ pusht のスモーク用チェックポイント）を使い、ラズパイ上で 1 回の推論にかかる時間を 100 回計測するスクリプトを `easy_lerobot/deploy/bench_inference.py` に作る。平均と最大の時間が出力されることで確認する
- [ ] 2.2 SO-101 の実データで学習した ACT で 2.1 を実行し、結果（平均・最大・録画時の fps との比較）を design.md に追記する。ローカル推論を既定のままにするか、リモート推論を既定にするかをその結果で決めて追記する

## 3. LAN での公開と自動起動（E 班）

- [ ] 3.1 `.streamlit/config.toml` に `server.address = "0.0.0.0"`・`server.port = 8501`・`browser.gatherUsageStats = false` を書き、同じ LAN の PC のブラウザで `http://<ホスト名>.local:8501` が開けることを確認する
- [ ] 3.2 `app.py` にパスワードの入口（design D3）を実装し、`.streamlit/secrets.toml` を `.gitignore` に入れる。パスワードなし・誤り・設定ファイルなしの 3 通りでステップの画面に入れないこと、正しいパスワードで入れることをテストで確認する
- [ ] 3.3 `deploy/easy-lerobot.service` を作り、`systemctl enable` 後にラズパイを再起動して画面が開けること、`kill -9` で画面を落とすと自動で戻ること、その間に動かしていたダミーの長い処理が止まらないことを確認する。設定手順を `deploy/README.md` に追記する

## 4. デモの推論の切り替え（D 班）

- [ ] 4.1 `deploy/settings.toml` の読み込みを実装し、ファイルがないときは `inference.mode = "local"` になること、不正な値のときに起動時に分かりやすいエラーになることをテストで確認する
- [ ] 4.2 デモ画面の「モデルを準備する」（design D6）と、local での `lerobot-rollout --policy.path=...` の起動を実装する。モデル未取得のとき `check()` が理由付きで失敗するテストと、ラズパイの実機で LAN の外につながない状態でデモが動くことで確認する
- [ ] 4.3 `remote` extra の追加の承認を得たうえで、remote での `lerobot-rollout --inference.type=remote` の起動を実装する。GPU のある PC で `lerobot-policy-server` を起動し、ラズパイからデモが動くこと、サーバーを止めた状態では画面に接続先付きのエラーが出ることで確認する

## 5. 通し確認

- [ ] 5.1 同じ LAN の PC のブラウザだけを使い、ラズパイの実機で「つなぐ → 合わせる → 教える」を行い、データセットが Hub に上がることを確認する。学習は GPU のある PC で手作業で行ってよい
- [ ] 5.2 学習済みモデルを使い、ブラウザから「動かす」を 3 回行って成功・失敗が記録されることを確認する。途中でブラウザを閉じて開き直しても続きから操作できることも確かめる
