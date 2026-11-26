# Windows で `--policy.path=<Hub の名前>` が通らない（学習・評価）

## いまの結論（2026-11-16）

- Windows で `lerobot-train --policy.path=lerobot/smolvla_base`（学習済みの方策から微調整）や `lerobot-eval --policy.path=lerobot/diffusion_pusht` のように、**Hugging Face の名前**を渡すと、重みを落とす所で `HFValidationError: Repo id must use alphanumeric chars ... 'lerobot\diffusion_pusht'` で止まる。
- 理由：設定の読み込みで、名前を `Path(...)` に変えている（本体 0.6.2 の `configs\train.py` 218 行目・`configs\eval.py` 54 行目、v0.7.0 の `configs\train.py` 241 行目・`configs\eval.py` 58 行目）。Windows では `Path("lerobot/smolvla_base")` を文字に戻すと `lerobot\smolvla_base` になり、Hugging Face の名前として通らない（`policies\pretrained.py` の `from_pretrained()` が `str()` してから `hf_hub_download()` に渡す）。
- 確かめたこと：
  - v0.7.0 の `lerobot-eval --policy.path=lerobot/diffusion_pusht` で起きた（2026-11-16）。
  - 本体（0.6.2）でも、`ACTPolicy.from_pretrained(Path("lerobot/act_aloha_sim_transfer_cube_human"))` が同じエラーで止まった（2026-11-16）。`lerobot-train` そのものは流していない。
- 当たらない所：本体の `lerobot-rollout --policy.path=<名前>` は、名前を文字のまま渡す（`rollout\configs.py` 417 行目）ので、この道は通らない見込み（流していない）。`--policy.type=act`（一から学習）も当たらない。
- **回り道**：先に手元のフォルダへ落として、そのフォルダを渡す。フォルダなら `from_pretrained()` が中のファイルを直接読む（`os.path.isdir()` の分かれ道）。
  ```powershell
  hf download lerobot/smolvla_base --local-dir models\smolvla_base
  lerobot-train --policy.path=models\smolvla_base ...
  ```
  v0.7.0 の `lerobot-eval` で、フォルダを渡すと重みを読めた（「Loading weights from local directory」、2026-11-16）。ただし `lerobot/diffusion_pusht` は古い形式で、その先の変換で止まった（[目的別 08](../目的別/08_実機なしで練習_v070.md) の 4）。`lerobot-train` でこの回り道を流してはいない。

## 影響する手順書

- [目的別 06](../目的別/06_SmolVLA_言葉で指示.md) の 4（SmolVLA の微調整）。
- [目的別 08](../目的別/08_実機なしで練習_v070.md) の 3（シミュレーションで評価）。
- 関係する、別の Windows の問題：[Windows で学習の保存が落ちる](Windowsで学習の保存が落ちる.md)。
