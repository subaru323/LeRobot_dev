# Windows で、学習の途中保存のときに `lerobot-train` が落ちる

## いまの結論（2026-11-16）

- Windows で `lerobot-train` を流すと、学習の結果を保存する所（`--save_freq` ごとと、最後）で、`checkpoints\last` という**シンボリックリンク**を作ろうとして、`OSError: [WinError 1314] クライアントは要求された特権を保有していません` で止まる。**Windows の「開発者モード」が切ってあり、管理者でもないとき。**
- 起きたのは v0.7.0（練習の環境、CPU、ACT を 20 ステップ）。本体（0.6.2、`53feaef`）にも同じ処理がある（`src\lerobot\common\train_utils.py` の `update_last_checkpoint()` が `symlink_to()` を呼ぶ。`lerobot_train.py` の学習のくり返しの中、1039 行目）。本体で流して確かめてはいない。
- 落ちると：
  - その回の保存のフォルダ（例：`checkpoints\000020\pretrained_model`）は書けている。
  - `checkpoints\last` はできない。手順書の「止まったときの続き」（`checkpoints/last/...`）は、そのままでは使えない。
  - 学習のくり返しの後にある Hugging Face への送り出し（`lerobot_train.py` の 1120 行目から）まで行かないので、**方策は上がらない**。
- 直し方（どちらか。パソコンの持ち主がする）：
  1. **開発者モードを入れる**：設定 → システム → 開発者向け → 「開発者モード」をオン。オンにすると、管理者でなくてもシンボリックリンクを作れる（Windows の仕組み）。パソコンの設定を変えるので、持ち主に頼む。
  2. 管理者として開いた PowerShell で学習を流す。
- 試しの学習（数ステップ）だけなら、`--save_checkpoint=false` を足すと保存しないので落ちない（`configs\train.py` に `save_checkpoint: bool = True` がある。足して流して確かめてはいない）。
- このパソコン（利用者のノート）は、開発者モードが入っていない（2026-11-16、レジストリの `AllowDevelopmentWithoutDevLicense` が無い）。

## 起きたこと

- コマンド（v0.7.0 の練習の環境）：`lerobot-train --dataset.repo_id=lerobot/pusht --dataset.streaming=true --policy.type=act --steps=20 --batch_size=4 --log_freq=5 --save_freq=20 --policy.device=cpu --policy.push_to_hub=false --output_dir=<scratch>\v070_train --job_name=v070_tiny`
- 20 ステップまで学習が進み、`checkpoints\000020` に `pretrained_model`（`config.json`、`model.safetensors` など）と `training_state` が書かれた後、`update_last_checkpoint()` の `symlink_to()` で止まった。終わりのコードは 1。
- 同じ実行のログに、Hugging Face のキャッシュについての「シンボリックリンクを使うには開発者モードか管理者が要る」という警告も出ている（キャッシュの方は、リンクの代わりにコピーするだけで止まらない）。

## 影響する手順書

- [データ収集と学習の手順書](../データ収集と学習の手順書.md) 11-4、13-1、13-2（学習の場所が Windows のとき）。13-3 の Hugging Face Jobs は Linux なので当たらない（未確認）。
- [準備手順書](../準備手順書.md) A（学習用のパソコンの準備）。
- [目的別 06](../目的別/06_SmolVLA_言葉で指示.md)、[目的別 08](../目的別/08_実機なしで練習_v070.md)。
