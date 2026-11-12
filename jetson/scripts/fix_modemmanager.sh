#!/bin/bash
# ModemManager が ESP32 / LiDAR のポートを触らないようにする（sudo が必要）。
#   bash ~/EasyRC_CRAWL/jetson/scripts/fix_modemmanager.sh
set -euo pipefail
ESP32_USB_PATH="${ESP32_USB_PATH:-platform-3610000.usb-usb-0:2.1:1.0}"
LIDAR_USB_PATH="${LIDAR_USB_PATH:-platform-3610000.usb-usb-0:2.4:1.0}"
sudo tee /etc/udev/rules.d/99-cugo.rules >/dev/null <<RULES
# CuGo: ESP32 と YDLIDAR X4 PRO は同じ CP2102 シリアル番号のため USB ポート位置で区別する
# ID_MM_DEVICE_IGNORE: ModemManager に触らせない（ESP32 がリセットを繰り返すため）
SUBSYSTEM=="tty", ENV{ID_PATH}=="${ESP32_USB_PATH}", SYMLINK+="cugo_esp32", MODE="0666", ENV{ID_MM_DEVICE_IGNORE}="1"
SUBSYSTEM=="tty", ENV{ID_PATH}=="${LIDAR_USB_PATH}", SYMLINK+="cugo_lidar", MODE="0666", ENV{ID_MM_DEVICE_IGNORE}="1"
RULES
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
sleep 1
udevadm info -q property -n /dev/cugo_esp32 | grep -E "ID_MM_DEVICE_IGNORE|DEVLINKS" || true
sudo systemctl restart ModemManager
sudo systemctl restart cugo
echo "完了。30 秒ほどで http://thisisjetson.local:8080/ が使えます"
