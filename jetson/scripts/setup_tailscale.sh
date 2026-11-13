#!/bin/bash
# Tailscale の導入（自宅などから SSH と Web UI に入るため）。Jetson 上で 1 回実行する。
#
#   bash ~/EasyRC_CRAWL/jetson/scripts/setup_tailscale.sh
#
# 行うこと（途中で sudo のパスワードを聞かれる）:
#   1. 公式インストーラ（https://tailscale.com/install.sh。apt リポジトリを追加して入れる）
#   2. sudo tailscale up でログイン（表示された URL をブラウザで開く）
#   3. tailnet のアドレスを表示
#
# Tailscale Funnel（インターネットへの公開）は使わないこと。Web UI には認証がない。
# Web UI の証明書は、次に cugo サービスを起動したときに Tailscale の名前入りで 1 回だけ作り直される。
set -euo pipefail

INSTALL_URL=https://tailscale.com/install.sh
TS_HOSTNAME="${TS_HOSTNAME:-cugo-jetson}"

step() { echo; echo "=== $* ==="; }

step "1. Tailscale のインストール"
if command -v tailscale >/dev/null; then
  echo "インストール済み: $(tailscale version | head -1)"
else
  echo "公式インストーラを実行します: $INSTALL_URL"
  read -r -p "続けますか？ [y/N] " ans
  [ "$ans" = y ] || [ "$ans" = Y ] || { echo "中止しました"; exit 1; }
  curl -fsSL "$INSTALL_URL" | sh
fi

step "2. ログイン（tailscale up）"
if tailscale ip -4 >/dev/null 2>&1; then
  echo "ログイン済み"
else
  echo "表示されたログイン URL をブラウザで開いてください"
  sudo tailscale up --hostname="$TS_HOSTNAME" --ssh=false
fi

step "3. tailnet のアドレス"
tailscale ip -4
tailscale status

echo
echo "完了。"
echo "- 管理画面（https://login.tailscale.com/admin/machines）でこの機械の「Disable key expiry」を押すと、"
echo "  180 日ごとの再ログインが要らなくなります。"
echo "- Web UI の証明書を作り直すため、sudo systemctl restart cugo を実行してください。"
echo "  ブラウザでは証明書が変わったので、一度だけ「詳細設定 → 進む」が必要です。"
echo "- Tailscale Funnel は使わないこと（Web UI には認証がありません）。"
