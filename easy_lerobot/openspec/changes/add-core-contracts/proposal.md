# Proposal

## Why

かんたん LeRobot は 5 班（A ハード / B データ収集 / C 学習 / D 評価・推論 / E 基盤・統合）が同時に実装する。各班のページ（pages/1_connect〜5_demo）は E 班の core 部品と project.json を介してつながるため、そのつなぎ目を先に仕様として固めないと、各自の AI アシスタントが別々の解釈で実装し、統合時に食い違う。

## What Changes

- 5 つのステップ（つなぐ・合わせる・教える・学習させる・動かす）が共通で持つ Step の型（`check()` / `run()` / `cancel()` と `status`）を定める。
- `projects/<名前>/project.json` の必須項目と、プロジェクト状態（未接続 → … → デモ中）の遷移規則を定める。
- `lerobot-*` コマンドを別プロセスで動かす Runner の約束（起動・停止、stdin への行送信、`logs/<step>.log` と `run.json`）を定める。
- 既存の `src/lerobot` は改変せず、コマンドを subprocess で呼ぶだけとする方針を仕様上の制約として明記する。
- 画面の文言・配置などの細部は仕様化しない（Non-goal）。

## Capabilities

### New Capabilities
- `step-contract`: 各ステップが実装すべき共通インターフェース（check / run / cancel / status）と、ステップとプロジェクト状態の対応。
- `project-state`: project.json の項目、保存・読み込み、状態の種類と許される遷移、再開の扱い。
- `command-runner`: lerobot-* コマンドの別プロセス実行、stdin 送信、ログと run.json による実行状態の受け渡し、停止。

### Modified Capabilities
（なし。初回の仕様である）

## Impact

- 対象コード: `easy_lerobot/core/project.py`・`core/runner.py`（E 班）、`easy_lerobot/pages/*.py`（A〜D 班）。
- 実行時ファイル: `easy_lerobot/projects/<名前>/`（project.json、logs/、run.json）。git 管理外。
- 依存: Streamlit（画面）。LeRobot 本体の改変はない。
- 実行環境: 本番はラズパイ（Linux）、開発は Windows。OS ごとの差は `core/platform.py` に閉じ込める（配置全体は add-raspberry-pi-deployment）。
- 未決事項（学習を回す GPU の場所、Hub アカウントの持ち方、録画中のキー操作の送り方）は design.md の Open Questions に残す。
