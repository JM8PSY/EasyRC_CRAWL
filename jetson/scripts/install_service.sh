#!/bin/bash
# cugo.service をインストールして、起動時に自動で立ち上がるようにする（sudo が必要）。
#   bash ~/EasyRC_CRAWL/jetson/scripts/install_service.sh
set -euo pipefail
REPO_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
sed -e "s|__USER__|$USER|" -e "s|__REPO__|$REPO_DIR|" "$REPO_DIR/jetson/systemd/cugo.service" \
  | sudo tee /etc/systemd/system/cugo.service >/dev/null
sudo systemctl daemon-reload
sudo systemctl enable --now cugo.service
sleep 2
systemctl --no-pager status cugo.service | head -5
echo
echo "完了。Web UI: http://$(hostname -I | awk '{print $1}'):8080/"
echo "  ログ:   journalctl -u cugo -f"
echo "  再起動: sudo systemctl restart cugo"
echo "  停止:   sudo systemctl stop cugo   （自動起動をやめる: sudo systemctl disable cugo）"
