# Proposal

## Why

かんたん LeRobot は、アームの横に置いたラズパイだけで「つなぐ → 合わせる → 教える → 動かす」を行い、利用者は手元の PC やタブレットのブラウザから操作する形を目指す。ただしラズパイの CPU では学習を実用的な時間で終えられない。そのため、どの処理をラズパイで行い、どの処理を GPU のある PC に任せるかを、実装に入る前に仕様として固める。

## What Changes

- 本番の実行環境をラズパイ（Linux、arm64）とし、画面・アームとカメラの接続・録画・デモをラズパイ上で行う。
- 学習はラズパイでは行わず、GPU のある PC で行う。データセットと学習済みモデルは Hugging Face Hub で受け渡す。
- 画面は同じ LAN 内のブラウザから開けるようにし、パスワードを知っている人だけが操作できるようにする。
- ラズパイの電源を入れると画面が自動で立ち上がるようにする。
- デモの推論は、既定ではラズパイ上で行う。速度が足りない場合に備え、GPU のある PC の推論サーバーへ切り替えられるようにする。
- add-core-contracts の Windows 前提の部分（仮想端末による入力の送り方、停止の手順、ファイルロック）は同提案の中で Linux 本番・Windows 開発の前提に改めた。本提案はそれを前提とする。
- UML（配置図・ユースケース図・シーケンス図・状態遷移図）は `dev_docs/easy_lerobot_uml.drawio` を正とする。

## Capabilities

### New Capabilities
- `edge-deployment`: ラズパイと GPU のある PC の役割分担、LAN からの利用と操作の制限、自動起動、推論の実行場所の切り替え、学習済みモデルの受け渡し。

### Modified Capabilities
（なし。add-core-contracts の 3 本はまだアーカイブ前のため、同提案の中で直接改めた）

## Impact

- 対象コード: `easy_lerobot/app.py`（パスワードの入口）、`easy_lerobot/pages/5_demo.py`（推論の実行場所）、`easy_lerobot/deploy/`（自動起動の設定、新規）。
- 実行環境: ラズパイ 5（8GB を想定）に Raspberry Pi OS（64 ビット）、GPU のある PC（場所は未決）。
- 依存: リモート推論を使う場合は lerobot の `remote` extra（Zenoh）が要る。依存の追加は承認を得てから行う。
- 未確認: torch と torchcodec の arm64 版が現在の `uv.lock` でそのまま入るか。ラズパイ上での ACT の推論速度。
