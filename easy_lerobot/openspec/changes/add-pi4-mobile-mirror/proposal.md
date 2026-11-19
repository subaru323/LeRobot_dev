# Proposal

## Why

add-raspberry-pi-deployment はラズパイ 5 で「つなぐ → 合わせる → 教える → 動かす」をすべて行う最終形を定めている。いま手元にあるのはラズパイ 4（メモリ 4GB）で、当面の目標はノート PC なしでミラー（リーダーの動きをフォロワーに映す）を行うことである。そこで、実機の配線と操作の形を先に決め、Windows 前提で書かれている画面（easy_lerobot）をラズパイ 4 の上で動かせるようにする。

## What Changes

- 実機の構成を決める: ラズパイ 4 に、リーダーとフォロワーの USB（2 本）、ラズパイの電源、USB ハブ経由でカメラ 2 台をつなぐ。カメラはフォロワーに 1 台（リーダーには付けない）と、アームとつかむ物体の距離を見る定点カメラ 1 台。サーボの電源は各アームの電源から取る。
- 画面はラズパイ上で Streamlit のサーバーとして動かし、利用者はスマホのブラウザから無線（同じ Wi-Fi）で開いて操作する。ラズパイには画面もキーボードもつながない。
- 画面の Windows 前提の部分（ポート名 `COM19`・`COM20` の固定、`CTRL_BREAK_EVENT` と `taskkill` によるプロセスの停止、Win32 API によるプロセスの生存確認）を、Linux でも動くように改める。Windows での動作は変えない。
- add-raspberry-pi-deployment の tasks 3（LAN での公開・パスワードの入口・自動起動）を、この構成で実装する。
- 推論と学習（同提案の tasks 2・4）は対象外。ラズパイ 4 では扱わない。

## Capabilities

### New Capabilities
- `pi4-mobile-mirror`: ラズパイ 4 の配線、スマホからの操作、Linux でのアームのプロセス管理、ポートの指定方法。

### Modified Capabilities
（なし。add-raspberry-pi-deployment の edge-deployment はまだアーカイブ前のため、同提案の要件のうち LAN 公開・パスワード・自動起動をそのまま実装する）

## Impact

- 対象コード: `easy_lerobot/core/hardware.py`（ポートの指定）、`easy_lerobot/core/procs.py`（Linux のプロセス管理）、`easy_lerobot/core/key_input.py`・`easy_lerobot/views/record.py`（キー送信が使えない環境の扱い）、`easy_lerobot/app.py`（パスワードの入口）、`easy_lerobot/.streamlit/config.toml`、`easy_lerobot/deploy/`（新規）。
- 実行環境: ラズパイ 4 Model B（4GB）、Raspberry Pi OS Lite 64 ビット（Debian 13 trixie）、OS 付属の Python 3.13。
- 依存: 新しい依存は追加しない。ミラーに要るのは streamlit・pyserial・feetech-servo-sdk だけで、ラズパイ 4 では lerobot 本体と torch を入れない。
- 未確認: カメラ 2 台を同じ USB ハブにつないだときの帯域と給電。録画（lerobot-record）をラズパイ 4 で動かせるか。
