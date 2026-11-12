#!/bin/bash
# Jetson 初期セットアップ（CuGo 自律移動）。Jetson 上で 1 回実行する。
#
#   bash ~/EasyRC_CRAWL/jetson/scripts/setup_jetson.sh
#
# 行うこと:
#   1. apt で ROS 2 Jazzy のパッケージ（Nav2 など）をインストール
#   2. udev ルール /etc/udev/rules.d/99-cugo.rules（/dev/cugo_esp32, /dev/cugo_lidar）
#   3. 外部ソースを固定コミットで取得してビルド
#      - YDLidar-SDK（/usr/local にインストール）
#      - ydlidar_ros2_driver, rf2o_laser_odometry（~/cugo_ws/src/third_party）
#   4. ~/cugo_ws をビルド（このリポジトリの jetson/src も含む）
set -euo pipefail

REPO_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
WS="${CUGO_WS:-$HOME/cugo_ws}"
BUILD_DIR="${CUGO_THIRD_PARTY_BUILD:-$HOME/third_party_build}"

# USB ポート位置（udevadm info -q property -n /dev/ttyUSBx | grep ID_PATH= で確認）
ESP32_USB_PATH="${ESP32_USB_PATH:-platform-3610000.usb-usb-0:2.1:1.0}"
LIDAR_USB_PATH="${LIDAR_USB_PATH:-platform-3610000.usb-usb-0:2.4:1.0}"

YDLIDAR_SDK_URL=https://github.com/YDLIDAR/YDLidar-SDK.git
YDLIDAR_SDK_REV=e008f32
YDLIDAR_DRIVER_URL=https://github.com/YDLIDAR/ydlidar_ros2_driver.git
YDLIDAR_DRIVER_REV=4ef70d3  # humble ブランチ（master は Jazzy でコンパイルできない）
RF2O_URL=https://github.com/MAPIRlab/rf2o_laser_odometry.git
RF2O_REV=b38c68e

step() { echo; echo "=== $* ==="; }

clone_at() {  # clone_at <url> <dir> <rev>
  local url=$1 dir=$2 rev=$3
  [ -d "$dir/.git" ] || git clone -q "$url" "$dir"
  git -C "$dir" fetch -q origin
  git -C "$dir" checkout -q "$rev"
  echo "$dir @ $(git -C "$dir" rev-parse --short HEAD)"
}

step "1. apt パッケージ"
sudo apt-get update -qq
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -qq \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization ros-jazzy-realsense2-camera \
  ros-jazzy-xacro ros-jazzy-rviz2 ros-jazzy-twist-mux ros-jazzy-imu-filter-madgwick \
  ros-jazzy-pointcloud-to-laserscan ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-joint-state-publisher ros-jazzy-robot-state-publisher \
  python3-serial python3-pytest python3-colcon-common-extensions \
  esptool cmake build-essential git swig

step "2. udev ルール"
sudo tee /etc/udev/rules.d/99-cugo.rules >/dev/null <<EOF
# CuGo: ESP32 と YDLIDAR X4 PRO は同じ CP2102 シリアル番号のため USB ポート位置で区別する
# ID_MM_DEVICE_IGNORE: ModemManager がモデムかどうか調べるためにポートを開くと、制御線の操作で
# ESP32 がリセットを繰り返す（起動直後・抜き差し直後に通信できなくなる）ので触らせない
SUBSYSTEM=="tty", ENV{ID_PATH}=="${ESP32_USB_PATH}", SYMLINK+="cugo_esp32", MODE="0666", ENV{ID_MM_DEVICE_IGNORE}="1"
SUBSYSTEM=="tty", ENV{ID_PATH}=="${LIDAR_USB_PATH}", SYMLINK+="cugo_lidar", MODE="0666", ENV{ID_MM_DEVICE_IGNORE}="1"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
sleep 1
ls -l /dev/cugo_esp32 /dev/cugo_lidar || echo "（デバイスが見つかりません。USB の挿し場所を確認してください）"

step "3a. YDLidar-SDK"
mkdir -p "$BUILD_DIR"
clone_at "$YDLIDAR_SDK_URL" "$BUILD_DIR/YDLidar-SDK" "$YDLIDAR_SDK_REV"
cmake -S "$BUILD_DIR/YDLidar-SDK" -B "$BUILD_DIR/YDLidar-SDK/build" -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build "$BUILD_DIR/YDLidar-SDK/build" -j"$(nproc)"
sudo cmake --install "$BUILD_DIR/YDLidar-SDK/build"
sudo ldconfig

step "3b. ROS ドライバ（third_party）"
mkdir -p "$WS/src/third_party"
clone_at "$YDLIDAR_DRIVER_URL" "$WS/src/third_party/ydlidar_ros2_driver" "$YDLIDAR_DRIVER_REV"
clone_at "$RF2O_URL" "$WS/src/third_party/rf2o_laser_odometry" "$RF2O_REV"

step "4. ワークスペースのビルド"
ln -sfn "$REPO_DIR/jetson/src" "$WS/src/cugo"
set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "$WS"
colcon build --symlink-install --packages-select ydlidar_ros2_driver rf2o_laser_odometry \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
if ! colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release; then
  echo "（外部ドライバはビルド済み。CuGo のパッケージのビルドに失敗しました。上のログを確認してください）"
fi

echo
echo "完了。新しいターミナルで: source $WS/install/setup.bash"
