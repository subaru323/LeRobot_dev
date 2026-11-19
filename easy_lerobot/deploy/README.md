# ラズパイ 4 で easy_lerobot を動かす

スマホのブラウザから mirrorMODE を操作するための手順。仕様は `openspec/changes/add-pi4-mobile-mirror/` を参照。

## 前提

- Raspberry Pi 4 Model B、Raspberry Pi OS Lite（64 ビット）
- 利用者 `lerobot`、リポジトリは `~/LeRobot_dev`
- ラズパイとスマホが同じ Wi-Fi にいる

## 1. 依存を入れる

ラズパイ 4 では lerobot 本体と torch を入れない。ミラーに要る分だけを OS 付属の Python の仮想環境に入れる。

```bash
python3 -m venv ~/venv
~/venv/bin/pip install feetech-servo-sdk pyserial "streamlit>=1.40,<2.0"
```

## 2. パスワードとポートを設定する

```bash
cd ~/LeRobot_dev/easy_lerobot
cp .streamlit/secrets.toml.example .streamlit/secrets.toml    # app_password を書き換える（変えたらサーバーを起動し直す）
cp deploy/easy-lerobot.env.example deploy/easy-lerobot.env    # ポートを書き換える
ls -l /dev/serial/by-id/                                      # リーダーとフォロワーをつないでから確認する
```

`deploy/easy-lerobot.env` がなければ `/dev/ttyACM0`（リーダー）・`/dev/ttyACM1`（フォロワー）を使う。この番号は差す順で入れ替わるので、by-id の名前を書いておく。

## 3. 手で起動して確かめる

```bash
cd ~/LeRobot_dev/easy_lerobot
set -a; . deploy/easy-lerobot.env; set +a     # ポートの設定を読む（ファイルがあるとき）
~/venv/bin/python -m streamlit run app.py
```

スマホで `http://<ラズパイの IP アドレス>:8501` を開く。IP アドレスはラズパイで `hostname -I` を実行すると分かる。

## 4. 電源を入れると立ち上がるようにする（sudo が要る）

```bash
sudo usermod -aG dialout,video lerobot
sudo cp ~/LeRobot_dev/easy_lerobot/deploy/easy-lerobot.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now easy-lerobot
systemctl status easy-lerobot            # active (running) なら成功
journalctl -u easy-lerobot -f            # 画面のログを見る
```

止めるときは `sudo systemctl stop easy-lerobot`。自動起動をやめるときは `sudo systemctl disable easy-lerobot`。

## 分かっている限界

- 録画の「この回を終える」「撮り直す」は、キー送信を使うため画面のないラズパイでは押せない。時間が来ると自動で進む
- 通信は HTTP。パスワードは学内の Wi-Fi で使う前提
