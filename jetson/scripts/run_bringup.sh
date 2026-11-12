#!/bin/bash
# systemd（cugo.service）から呼ばれる起動スクリプト。手動でも実行できる。
#   起動引数は ~/.config/cugo.env の CUGO_ARGS で変更できる（例: CUGO_ARGS="mode:=nav map:=$HOME/maps/room.yaml"）
set -eo pipefail

WS="${CUGO_WS:-$HOME/cugo_ws}"
CUGO_ARGS="mode:=slam"
[ -f "$HOME/.config/cugo.env" ] && . "$HOME/.config/cugo.env"

# USB デバイス（ESP32 / LiDAR）が現れるまで最大 30 秒待つ
for i in $(seq 30); do
  [ -e /dev/cugo_esp32 ] && [ -e /dev/cugo_lidar ] && break
  sleep 1
done
ls -l /dev/cugo_esp32 /dev/cugo_lidar || echo "警告: デバイスが見つかりません（そのまま起動します）"

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
echo "ros2 launch cugo_bringup bringup.launch.py $CUGO_ARGS"
# shellcheck disable=SC2086
exec ros2 launch cugo_bringup bringup.launch.py $CUGO_ARGS
