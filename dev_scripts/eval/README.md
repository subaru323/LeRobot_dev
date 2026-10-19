\# D班 評価・推論（dev\_scripts/eval）



実データと学習済み方策がそろう前に、推論の流れと評価の記録方法を先に固めるための置き場です。

`src/lerobot` は変更しません。



\## ファイル



| ファイル | 内容 |

|---|---|

| `mock\_infer.py` | 偽の観測データで ACT の推論ループを回し、1 ステップの処理時間を測る |

| `eval\_records.csv` | 評価記録表。方策・データごとの成功率と推論時間を 1 行ずつ追記する |

| `README.md` | このファイル |



\## 進め方



| 段階 | 内容 | 状態 |

|---|---|---|

| M0 | PushT（シミュ）で学習 200 ステップ → 評価 2 エピソード | 済 |

| M1 | 公開データ `lerobot/svla\_so101\_pickplace` で ACT を 100 ステップ学習（形をそろえるため） | 済 |

| M2 | `mock\_infer.py` で推論ループと処理時間を確認 | 済 |

| M3 | 評価記録表と demo ステップの設計メモ（このファイル） | 済 |

| 本番 | B 班のデータと C 班の方策で、実機の成功率を測る | 待ち |



\## mock\_infer.py の実行



PowerShell で `LeRobot\_dev` から実行します。



```powershell

uv run lerobot-train --policy.type=act --policy.device=cuda --policy.push\_to\_hub=false --dataset.repo\_id=lerobot/svla\_so101\_pickplace --steps=100 --batch\_size=8 --num\_workers=0 --save\_freq=100 --output\_dir=outputs/train/act\_so101\_mock --wandb.enable=false

uv run python dev\_scripts/eval/mock\_infer.py

```



\- GPU が無い PC は `--policy.device=cpu`、`--batch\_size=4` にします。`mock\_infer.py` は CUDA が無いと自動で CPU を使います。

\- 同じ `--output\_dir` で学習をやり直すと止まるか上書きされます。2 回目は名前を変えます。



\## 評価記録表（eval\_records.csv）



1 回の評価を 1 行で追記します。Excel で開く場合の文字化けを避けるため、`notes` は英数字で書きます。



| 列 | 内容 |

|---|---|

| `date` | 実施日 |

| `run\_id` | 実行の名前（例 `real\_cube\_pick\_v1`） |

| `policy` | 方策のパスまたは Hub 名 |

| `policy\_steps` | 学習ステップ数 |

| `dataset` | 学習に使ったデータセット |

| `device` | `cpu` / `cuda` と GPU 名 |

| `mode` | `sim`（シミュ）/ `mock`（偽データ）/ `real`（実機） |

| `episodes` / `successes` / `success\_rate` | 試行回数・成功回数・成功率（%） |

| `mean\_step\_ms` / `max\_step\_ms` | 1 ステップの平均・最大処理時間 |

| `notes` | 初期配置、失敗の種類など |



\## 分かったこと（2026-10-19）



\- ACT は 1 回の推論で 100 動作分（`chunk\_size=100`）を出す。モデルの計算は 100 ステップに 1 回だけ。

\- RTX 4060 Laptop では、温まった状態のモデル計算が 30fps の 1 ステップ（33.3 ms）に収まる。

\- 初回だけ約 0.5 秒かかる。実機では、腕を動かす前に 1 回空の推論をしておく。

\- 方策は学習時と同じカメラ名・台数でしか動かない。M1 の方策は `up`・`side` の 2 台、本番は `front` 1 台の予定。

\- 関節の単位は、このリポジトリでは度数（`use\_degrees=True`）、グリッパーは 0〜100。収録時とロボット設定で揃える。

\- 偽データのため、カメラの読み取りとサーボ通信の時間は含まれていない。実機で測り直す。



\## demo ステップ（5\_demo.py）の設計メモ



E 班の `core/step.py`（openspec タスク 4.1）が公開されてから実装します。以下は `add-core-contracts` の仕様に合わせた案です。



\*\*check()\*\*：次がそろっていなければ不可とし、理由を返す。



| 条件 | 不可のときの理由（画面に出す文） |

|---|---|

| `state` が `trained` | 「先に学習を終えてください」 |

| `policy\_repo\_id` が設定済み | 「学習済みモデルが設定されていません」 |

| `hardware.follower\_port` が設定済み | 「フォロワーのポートが未設定です（1. つなぐ）」 |

| `hardware.cameras` の名前が方策の入力と一致 | 「カメラの名前が学習時と違います」 |



\*\*run()\*\*：Runner で次を起動する（公式の評価手順を元にした案・未検証）。



```text

lerobot-record --robot.type=so101\_follower --robot.port=<follower\_port> --robot.id=<robot\_id> --robot.cameras=<cameras> --dataset.repo\_id=<HF\_USER>/eval\_<task> --dataset.single\_task="<task\_description>" --dataset.num\_episodes=1 --policy.path=<policy\_repo\_id>

```



\*\*on\_finished(run)\*\*：成功なら `trained` へ戻す。失敗・中止なら `fallback\_state` へ。

成功・失敗は利用者が画面のボタンで入力し、`progress.demo\_trials` と `progress.demo\_success` を 1 回分だけ増やす。

D 班独自の値（推論時間など）は `ext.eval` の下に保存する。



\*\*まだ決めていないこと\*\*



\- 1 試行ごとに `lerobot-record` を起動し直すか、1 回の起動で複数エピソードを回すか。

\- 評価用データセット名の付け方。同じ名前で 2 回録るとエラーになるため、試行ごとに名前を変えるか `--resume` を使う。

\- 非常停止。CTRL\_BREAK で止めたときに腕がどの姿勢で止まるかを実機で確認する。

