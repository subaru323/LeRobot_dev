# Tasks

## 1. tailnet の準備

- [ ] 1.1 Tailscale のアカウント（無料の個人プラン）の名義を決め、作成またはログインする
- [ ] 1.2 アクセス制御の雛形 `deploy/tailscale-acl.example.json` を書く（tag:lerobot-pi に tcp 8501 と tcp 22 だけを通す。D3）
- [ ] 1.3 雛形を管理画面の ACL に反映する

## 2. ラズパイへの導入

- [ ] 2.1 ラズパイに公式リポジトリから `tailscale` を入れる。回線が遅く終わらない場合は、開発用の PC でパッケージを取得して scp で送る
- [ ] 2.2 `sudo tailscale up --hostname=lerobot-pi --advertise-tags=tag:lerobot-pi` でログインし、`tailscaled` が自動起動になっていることを確かめる
- [ ] 2.3 管理画面で lerobot-pi の鍵の期限切れを無効にする（D2）
- [ ] 2.4 `tailscale funnel status` で公開がないことを確かめる

## 3. 端末の参加と確認

- [ ] 3.1 開発用の PC と係の端末を tailnet に参加させる
- [ ] 3.2 係の端末から `http://lerobot-pi:8501` が開け、ミラーの開始・停止が効くことを確かめる
- [ ] 3.3 開発用の PC から `ssh lerobot@lerobot-pi` で入れることを確かめる
- [ ] 3.4 tailnet に参加していない端末からは開けないこと、8501・22 番以外に届かないことを確かめる
- [ ] 3.5 ラズパイの電源を入れ直し、操作なしで 3.2 が通ることを確かめる

## 4. 会場の Wi-Fi での経路の確認

- [ ] 4.1 端末同士が遮断された Wi-Fi で `tailscale ping lerobot-pi` を実行し、直接か DERP かを記録する（D4）
- [ ] 4.2 同じ条件で、画面が開くまでの秒数とミラー開始ボタンの反応を測る
- [ ] 4.3 遅すぎる場合に、係の端末をテザリングにして再度測る

## 5. 文書と VPS の中継の停止

- [ ] 5.1 `deploy/README.md` に Tailscale の節（導入・参加・経路の確認・端末をなくしたときの外し方）を書く
- [ ] 5.2 README の中継の節と `relay-tunnel.service.example` に「Tailscale に置き換えた。使わない」と書き添える（D5）
- [ ] 5.3 4 の確認が済んだら、ラズパイで `sudo systemctl disable --now relay-tunnel` を実行する
