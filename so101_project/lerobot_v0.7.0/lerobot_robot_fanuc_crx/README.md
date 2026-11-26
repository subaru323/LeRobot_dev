# lerobot_robot_fanuc_crx

FANUC CRX を LeRobot v0.7.0 の `Robot` として扱うためのアダプターの**骨組み**です。
制御方式は関節位置指令（度）で、Remote Inference に必須の位置保持（hold）に対応しています。

> 実機用ドライバー（`ExistingSystemCRXDriver`）は未実装です。いま動くのはモックドライバーだけです。

## 構成

```
lerobot-rollout / record / teleoperate
        │  --robot.type=fanuc_crx
        ▼
FanucCRX (fanuc_crx.py)        ← LeRobot の Robot。特徴量、安全クランプ、hold の契約
        │  CRXDriver インターフェース
        ▼
MockCRXDriver                  ← テスト用の模擬アーム
ExistingSystemCRXDriver (TODO) ← 既存の接続システムへのブリッジ
```

| ファイル | 役割 |
|---|---|
| `config_fanuc_crx.py` | `FanucCRXConfig`（`fanuc_crx` として登録） |
| `fanuc_crx.py` | `FanucCRX(Robot)` |
| `driver.py` | `CRXDriver` の抽象、モック、実機用スタブ |
| `tests/test_fanuc_crx.py` | モックでの動作確認（8項目） |

## インストールとテスト

```bash
uv pip install --python .venv/Scripts/python.exe --no-deps -e ./lerobot_robot_fanuc_crx
```

```bash
.venv/Scripts/python.exe lerobot_robot_fanuc_crx/tests/test_fanuc_crx.py
```

パッケージ名が `lerobot_robot_` で始まるため、LeRobot の CLI が自動で読み込みます（`register_third_party_plugins`）。

## 特徴量

| 種類 | キー | 単位 |
|---|---|---|
| action / observation | `joint_1.pos` 〜 `joint_6.pos` | 度 |
| action / observation（任意） | `gripper.pos` | 0〜100 |
| observation（任意） | `--robot.cameras` で指定したカメラ | (H, W, 3) |

全アクションを `.pos` にしているのは、LeRobot の位置保持の条件（`ThreadSafeRobot._configure_position_hold`）を満たすためです。

## 設定

| オプション | 既定値 | 意味 |
|---|---|---|
| `driver` | `mock` | `mock` または `existing` |
| `host` / `port` | `127.0.0.1` / `0` | 既存接続システムのアドレス |
| `joint_limits_deg` | 全関節 ±90° | **仮の値**。CRX の実際の可動範囲ではない。仕様書と DCS 設定に合わせて必ず設定する |
| `max_step_deg` | `1.0` | 1回の指令で動ける最大量。10 Hz なら約10°/s が上限 |
| `hold_mode` | `position` | `position`: 最後の目標を再送する ／ `command`: ドライバーの `hold()`（コントローラー側の停止）を呼ぶ |
| `use_gripper` | `false` | グリッパーを使うか |

## 安全の考え方

- 主な安全はコントローラー側（DCS、速度制限、衝突検知、非常停止）が担う。このアダプターは2段目のチェック
- `send_action` が行うこと:
  - NaN / inf を拒否する
  - キーの過不足を拒否する
  - コントローラーが ready でなければ拒否する
  - 関節リミットと1ステップ量でクランプし、**実際に送った値**を返す
- Remote Inference が切断されると、バッファの行動を使い切ってから hold して停止し、自動では再接続しない（`../NOTES.md` の切断テスト参照）。`max_step_deg` とバッファ長で、切断後に動きうる量の上限が決まる

## 実機をつなぐまでにやること

1. 既存の接続システムで次ができるか確認する
   - J1〜J6 を制御周期（10〜30 Hz）で読めるか
   - 目標を逐次ストリーミングで送れるか（1点ずつ止まりながら動く動作にならないか）
   - 停止要求を出せるか、アラーム / 非常停止 / リモートモードの状態を取れるか
2. `driver.py` の `ExistingSystemCRXDriver` を実装する
3. 実際の可動範囲で `joint_limits_deg` を設定する。最初は `max_step_deg` を小さくする
4. ティーチペンダントと非常停止をすぐ押せる状態で、低速・広い空間から始める
5. Remote Inference 用には、CRX の関節名でデータを収集し、学習したポリシーをサーバーに載せる
