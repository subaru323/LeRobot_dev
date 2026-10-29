# Design

## Context

- 動機は proposal.md の Why、要件は specs/edge-deployment を参照。構成図は `dev_docs/easy_lerobot_uml.drawio` の「配置図」ページ。
- 実装は add-core-contracts の core（Project・Runner・Step）の上に載せる。仮想端末による入力の中継、停止の手順、ファイルロックの Linux 版は同提案の design D2・D4・D5・D9・D10 で決めた。
- ソースで確認した事実:
  - lerobot には、ロボット側の PC と GPU 側の推論サーバーを分ける公式の仕組みがある（`docs/source/remote_inference.mdx`）。ロボット側は `lerobot-rollout --inference.type=remote`、GPU 側は `lerobot-policy-server` を使い、両者は Zenoh（既定 `tcp/<ホスト>:7447`）でつながる。ACT と SmolVLA は実機での動作実績がある（ACT は時間方向の平均化を切る必要がある）。SO-101 フォロワーは対応ロボットに含まれる。
  - リモート推論には lerobot の `remote` extra が要る。現在の `easy_lerobot/pyproject.toml` の extras は `core_scripts,feetech,training` で、`remote` は入っていない。
- 未確認の事実: torch 2.11・torchvision 0.26・torchcodec 0.11.1 の arm64（Linux）版が、現在の `uv.lock` でそのまま入るか。Pi 5 の CPU だけで ACT の推論が実用的な速さで動くか。

## Goals / Non-Goals

**Goals:**
- ラズパイ 1 台と、利用者のブラウザだけで、つなぐ・合わせる・教える・動かすを終えられること。
- デモ当日、学習済みモデルさえ取得してあれば、インターネットなしで動かせること（ローカル推論の場合）。

**Non-Goals:**
- 学習をどこで回すか（学校の PC、HF Jobs、Colab）と、ラズパイの画面から学習を依頼する方法。「学習させる」ページの中に閉じるため、本設計では扱わない。
- インターネット越しの遠隔操作。
- 1 台のラズパイを複数の班が同時に使うこと。

## Decisions

### D1. ハードウェアと OS
ラズパイ 5（8GB）に Raspberry Pi OS（64 ビット）を入れる。Python は uv で 3.12 を入れ、`easy_lerobot` の `uv sync --locked` で環境を作る。
- 理由: lerobot の推論と録画中の動画の変換は CPU とメモリを使うため、現行で最も速い機種とメモリの多い構成にする。OS 付属の Python ではなく uv の Python にすると、開発環境と版をそろえられる。
- 代替案: Jetson などの GPU 付き小型機 → 推論は確実に速いが、価格と環境構築の手間が大きい。ラズパイでの計測（tasks 2）で速度が全く足りなかった場合にだけ再検討する。

### D2. 画面の公開方法
Streamlit を `--server.address 0.0.0.0 --server.port 8501` で起動し、LAN から開けるようにする。ホスト名は mDNS（`<ホスト名>.local`）で引けるようにする。インターネット側への転送（ポート開放やトンネル）は設定しない。利用統計の送信は `.streamlit/config.toml` の `browser.gatherUsageStats = false` で止める。

### D3. パスワードによる入口
`app.py` の最初で、`.streamlit/secrets.toml` の `app_password` と入力値を比べ、一致するまでページの登録（`st.navigation`）を行わない。一致したら `session_state` に記録する。`secrets.toml` は `.gitignore` に入れる。
- 理由: 標準ライブラリと Streamlit だけで済み、依存が増えない。ページを登録しないので、URL を直接打ってもステップの画面には入れない。
- 代替案: Streamlit の `st.login`（OIDC）→ 外部の認証サービスが必要で、オフラインのデモで使えないため不採用。リバースプロキシの Basic 認証 → 構成要素が 1 つ増えるため、今回は見送る。
- 限界: 通信は HTTP のままなので、LAN 内の盗聴は防げない。学内 LAN での利用を前提とする。

### D4. 自動起動
`deploy/easy-lerobot.service`（systemd のユニット）を用意し、`Restart=on-failure` で画面のプロセスを動かす。ユニットはラズパイのログイン利用者の権限で動かし、`dialout`（シリアル）と `video`（カメラ）のグループに入れる。
- 理由: 電源を入れるだけで使え、落ちても自動で戻る。録画や学習は add-core-contracts の見張り役が別セッションで動かしているので、画面の再起動で止まらない。
- Runner の子プロセスが systemd の停止で巻き添えにならないよう、ユニットは `KillMode=process` にする（画面のプロセスだけを止める）。

### D5. 推論の実行場所の設定
ラズパイごとの設定ファイル `deploy/settings.toml` に `inference.mode = "local" | "remote"` と `inference.endpoint`（例 `tcp/192.168.0.10:7447`）を置く。project.json ではなくラズパイごとの設定にするのは、推論サーバーの有無がプロジェクトではなく設置場所で決まるため。
- local: `lerobot-rollout --policy.path=<取得済みモデルのパス>` を Runner で起動する。
- remote: `lerobot-rollout --inference.type=remote --inference.endpoint=<設定値> ...` を Runner で起動する。GPU のある PC では、学習担当が `lerobot-policy-server` を起動しておく。
- 停止・入力の中継は add-core-contracts の Runner のとおり（`--interactive=true` の `/start`・`/stop` も 1 行の送信で届く）。
- 依存: remote を使うには `remote` extra の追加が要る。承認を得るまでは local のみ実装する。

### D6. モデルの事前取得
デモ画面の「モデルを準備する」で、`policy_repo_id` のモデルを Hub から `projects/<名前>/models/` に取得する。デモ開始時の `check()` は、local のときにこのディレクトリがそろっていることを確かめ、なければ理由として返す（add-core-contracts の CheckResult）。

## Risks / Trade-offs

- [torch・torchcodec の arm64 版が `uv.lock` の版で入らない] → 最初のタスクで確かめる。駄目なら版の組み合わせを変える変更提案を起こす（依存の変更は承認が要る）。
- [Pi 5 の CPU では ACT の推論が遅く、アームの動きがぎこちない] → 推論時間を計測するタスクを入れ、足りなければリモート推論を既定にする。
- [録画後の動画の変換に時間がかかり、次のエピソードまで待たされる] → 計測し、遅ければカメラの解像度とフレームレートをタスクの雛形で下げる。
- [USB の給電不足でカメラやサーボ基板が不安定になる] → ラズパイ 5 の純正電源を使い、サーボの電源はアーム側の電源から取る（ラズパイの USB から取らない）。
- [パスワードが HTTP で流れる] → 学内 LAN 限定とし、インターネット側に出さない（D2・D3）。
- [systemd の停止で録画中のプロセスまで止まる] → `KillMode=process` にし、ラズパイの再起動の前には録画を止めるよう画面で案内する。

## Migration Plan

新規の配置のため移行はない。開発中は Windows の PC で core と画面を作り、ダミーのコマンドで動作を確かめる。実機での確認はラズパイで行う。

## Open Questions

- 推論サーバーを置く GPU のある PC の場所と、LAN からの到達性（学内のネットワークでポート 7447 が通るか）。tasks 4 を始めるまでに決めればよい。
