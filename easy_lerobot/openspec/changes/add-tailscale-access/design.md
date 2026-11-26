# Design

## Context

- ラズパイ（lerobot-pi、Raspberry Pi 4、Raspberry Pi OS 64 ビット）は会場や学校の Wi-Fi に入る。この Wi-Fi は次の 2 つの問題を抱えることがある。
  - 端末同士の通信が遮断されている（クライアント分離）。
  - インターネット側が遅い。2026-11-23 には、GitHub から約 9KB/秒、トンネル越しの Streamlit が約 6KB/秒だった。
- ラズパイから外への通信はできるが、外から始まる通信はラズパイに届かない。
- 画面（Streamlit）は `0.0.0.0:8501` で待ち受けている（`.streamlit/config.toml`）。入口にはアプリのパスワードがある（`app.py`）。
- 開発用の PC（dynabook）には、すでに Tailscale が入っている。
- ミラーの制御は、ラズパイの中で USB だけで完結している。ネットを通るのはボタン・状態・画面の表示だけである。

## Goals / Non-Goals

**Goals**
- 費用をかけない（Tailscale の無料の個人プランの範囲で収める）。
- 端末同士が遮断された Wi-Fi やモバイル回線からでも、決められた端末がラズパイの画面と ssh に届く。
- 画面をインターネットに公開しない（edge-deployment の MUST NOT を守る）。
- アプリのコードは変えない。

**Non-Goals**
- 来場者が自分のスマホで操作すること。来場者のスマホを tailnet に参加させるのは現実的でなく、Funnel による公開は上の Goals に反するため。
- 遅い回線そのものを速くすること。D4 のとおり、経路が中継（DERP）になった場合の速さは回線に依存する。
- MQTT による命令の軽量化（add-festival-relay の D3）。必要になれば別の提案で扱う。

## Decisions

### D1. ラズパイと操作する端末を 1 つの tailnet に入れる

```
係の端末・開発用 PC ──(WireGuard、tailnet)──▶ lerobot-pi
   http://lerobot-pi:8501        ssh lerobot@lerobot-pi
```

- ラズパイには公式リポジトリの `tailscale` を入れ、`tailscaled` を systemd で自動起動にする。ホスト名は `lerobot-pi` とし、MagicDNS の名前で開けるようにする。
- 理由：Tailscale は両側から外へ出る通信だけで経路を作る。そのため、ラズパイ側にポート開放や VPS が要らず、端末同士が遮断された Wi-Fi でも届く。無料の個人プランで足りる。
- 画面は `0.0.0.0` で待ち受けたままにする。tailnet のアドレスでも、同じ Wi-Fi のアドレスでも開ける。待ち受けを tailnet に絞ると、端末同士が通じる Wi-Fi での直接の経路を失うため、絞らない。

### D2. ラズパイのノードは鍵の期限を切らない

- Tailscale のノード鍵は既定で 180 日ごとに期限が切れ、切れると再ログインが要る（この情報の信頼度は高くございません）。画面のないラズパイでは再ログインが難しいため、管理画面で lerobot-pi だけ「鍵の期限切れ」を無効にする。
- 操作する端末（人が持つもの）は既定のままにする。

### D3. アクセス制御で 8501 番と 22 番だけを通す

- tailnet のアクセス制御（ACL）で、ラズパイへ通す通信を tcp 8501（画面）と tcp 22（ssh）に限る。ラズパイにはタグ `tag:lerobot-pi` を付け、ルールはタグに対して書く。
- 雛形を `deploy/tailscale-acl.example.json` に置く。管理画面の ACL に貼り付けて使う。
- ssh は Tailscale SSH ではなく、ラズパイの OpenSSH をそのまま使う。今の利用者 `lerobot` とパスワードの運用を変えずに済むためである。
- Funnel と Serve は有効にしない。

### D4. 直接か中継かを確かめてから使う

- Tailscale は、まず端末同士の直接の経路を試す。直接つながらないときは、中継サーバー（DERP、東京にもある）を通す。
- 端末同士が遮断された Wi-Fi では、同じ SSID の 2 台が直接つながらず、DERP 経由になる可能性がある（確度：推測）。DERP 経由では、行きも帰りもその Wi-Fi のインターネット回線を通る。そのため、回線が遅ければ画面も遅い。
- そこで、会場で使う前に、操作する端末から `tailscale ping lerobot-pi` を実行して、経路が `via DERP(...)` か直接かを確かめる。あわせて、画面が開くまでの時間を測り、`deploy/README.md` に結果の書き方を残す。
- DERP 経由で遅すぎる場合の逃げ道は 2 つある。係の端末をテザリングでつなぐ（端末側の回線を変える）か、端末同士が通じる Wi-Fi で直接開く経路に戻すかである。

### D5. VPS の中継をやめる

- 本提案の経路が会場の Wi-Fi で動くことを確かめたら、ラズパイの `relay-tunnel` を `disable --now` する。VPS は不要になる。
- `deploy/relay-tunnel.service.example` と README の中継の節は消さず、「Tailscale に置き換えた。使わない」と書き添える。

## Relationship to other changes

- add-raspberry-pi-deployment（edge-deployment）：「LAN からの利用」の「インターネット側へは公開しない（MUST NOT）」は、本提案でも守る。tailnet は参加を認めた端末だけの私的なネットワークとして扱い、インターネットへの公開とはみなさない。この解釈は、tailnet-access の要件「tailnet の外へ公開しない」で裏付ける。
- add-festival-relay（festival-relay）：「VPS を通した来場者の操作」と「ラズパイ用の鍵の制限」は、D5 によって使われなくなる。同提案を取り下げるか、残してアーカイブするかは本提案では決めない（Open Questions）。「手首カメラの表示」と「Wi-Fi は人が切り替える」は、本提案と独立に残る。
- add-pi4-mobile-mirror：同じ Wi-Fi から直接開く経路は残すため、影響しない。

## Risks / Trade-offs

- [DERP 経由になると遅い] → D4 で会場ごとに確かめる。遅ければ、係の端末側をテザリングにする。
- [Tailscale のアカウントに依存する] → アカウントの持ち主が抜けると管理できなくなる。持ち主と、管理画面に入れる人をチームで決める。
- [係の端末をなくす・盗まれる] → 管理画面からその端末を tailnet から外す。画面にはアプリのパスワードも残る。
- [ラズパイの回線が遅く、導入に時間がかかる] → パッケージを開発用の PC で取得して scp で送る手順を、予備として README に書く。

## Open Questions

- add-festival-relay を取り下げるか、使わなくなった記録として残してアーカイブするか。
- Tailscale のアカウントを誰の名義にするか（個人か、チーム用に作るか）。
- 係の端末を何台にするか（無料の個人プランの端末数の上限に収まるか）。
