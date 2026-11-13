#!/bin/bash
# librealsense（RSUSB バックエンド + CUDA）と realsense-ros をソースからビルドする。
#
# なぜ: この Jetson のカーネル（6.8 tegra）には HID センサーのドライバが無く、apt 版 librealsense
# （V4L バックエンド）では D455 の IMU が使えない。RSUSB バックエンドは libusb で直接 IMU を読む。
#
# 段階ごとに実行する（sudo が要る段階はユーザーが端末で実行する）:
#   bash build_realsense.sh deps      # sudo: libusb ヘッダ、image_transport_plugins
#   bash build_realsense.sh build     # sudo 不要: librealsense をビルド（40〜90 分。nohup 推奨）
#   bash build_realsense.sh install   # sudo: /usr/local にインストール、udev ルール、apt 版を削除
#   bash build_realsense.sh ros       # sudo 不要: realsense-ros を ~/cugo_ws でビルド
#   bash build_realsense.sh check     # 読み取りのみ: IMU が見えるか
#
# 実行前に cugo サービスを止めておく（カメラを掴んだままにしない）: sudo systemctl stop cugo
#
# 戻し方（apt 版へ）:
#   sudo apt install ros-jazzy-realsense2-camera
#   rm -rf ~/cugo_ws/src/third_party/realsense-ros && (cd ~/cugo_ws && rm -rf build/realsense2_* install/realsense2_*)
#   cd ~/third_party_build/librealsense/build && sudo make uninstall && sudo ldconfig
set -euo pipefail

WS="${CUGO_WS:-$HOME/cugo_ws}"
BUILD_DIR="${CUGO_THIRD_PARTY_BUILD:-$HOME/third_party_build}"
JOBS="${JOBS:-4}"                 # 6 だとリンク時にメモリが足りないことがある
RS_CUDA="${RS_CUDA:-ON}"          # CUDA でのビルドに失敗したら RS_CUDA=OFF で
CUDA_ARCH="${CUDA_ARCH:-87}"      # Orin

# apt 版（2026-11-13 時点で 2.58.4 / 4.58.4）と同じ版に固定する
LIBRS_URL=https://github.com/IntelRealSense/librealsense.git
LIBRS_REV=v2.58.4
RSROS_URL=https://github.com/IntelRealSense/realsense-ros.git
RSROS_REV=4.58.4

LIBRS_DIR="$BUILD_DIR/librealsense"
RSROS_DIR="$WS/src/third_party/realsense-ros"

step() { echo; echo "=== $* ==="; }

clone_at() {  # clone_at <url> <dir> <rev>
  local url=$1 dir=$2 rev=$3
  [ -d "$dir/.git" ] || git clone -q "$url" "$dir"
  git -C "$dir" fetch -q --tags origin
  git -C "$dir" checkout -q "$rev"
  echo "$dir @ $(git -C "$dir" describe --tags --always)"
}

ros_env() {
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  set -u
}

do_deps() {
  step "deps: libusb ヘッダ、image_transport_plugins（映像の JPEG 配信用）"
  sudo apt-get update -qq
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -qq \
    libusb-1.0-0-dev libssl-dev libudev-dev pkg-config cmake build-essential git \
    ros-jazzy-image-transport-plugins
}

do_build() {
  step "build: librealsense $LIBRS_REV（RSUSB, CUDA=$RS_CUDA）"
  # libusb のヘッダが無いと librealsense は同梱版を作るが、v2.58.4 ではそのヘッダが
  # 見つからず「libusb.h: No such file or directory」で止まる。先に deps を実行すること
  if [ ! -f /usr/include/libusb-1.0/libusb.h ]; then
    echo "libusb のヘッダがありません。先に: bash $0 deps"; exit 1
  fi
  mkdir -p "$BUILD_DIR"
  # 同梱 libusb を使う設定が残っていると直らないので、キャッシュは毎回作り直す
  rm -f "$LIBRS_DIR/build/CMakeCache.txt"
  clone_at "$LIBRS_URL" "$LIBRS_DIR" "$LIBRS_REV"
  local cuda_args=(-DBUILD_WITH_CUDA=OFF)
  if [ "$RS_CUDA" = ON ]; then
    cuda_args=(-DBUILD_WITH_CUDA=ON -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc
               -DCMAKE_CUDA_ARCHITECTURES="$CUDA_ARCH")
  fi
  cmake -S "$LIBRS_DIR" -B "$LIBRS_DIR/build" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DFORCE_RSUSB_BACKEND=ON "${cuda_args[@]}" \
    -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF -DBUILD_TOOLS=ON \
    -DBUILD_PYTHON_BINDINGS=OFF -DBUILD_UNIT_TESTS=OFF -DCHECK_FOR_UPDATES=OFF
  cmake --build "$LIBRS_DIR/build" -j "$JOBS"
  echo "librealsense のビルド完了。次: bash $0 install（sudo）"
}

do_install() {
  step "install: librealsense を /usr/local へ"
  [ -f "$LIBRS_DIR/build/Makefile" ] || { echo "先に build を実行してください"; exit 1; }
  sudo make -C "$LIBRS_DIR/build" install
  sudo ldconfig

  step "install: udev ルール（RSUSB は一般ユーザーで USB を開くため必要。99-cugo.rules とは別ファイル）"
  sudo cp "$LIBRS_DIR/config/99-realsense-libusb.rules" /etc/udev/rules.d/
  sudo udevadm control --reload-rules
  sudo udevadm trigger

  step "install: apt 版 RealSense を削除（/opt/ros/jazzy の古い librealsense2.so が先に読まれるのを防ぐ）"
  # autoremove はしない（cv_bridge などの依存を残す）
  sudo apt-get remove -y ros-jazzy-realsense2-camera ros-jazzy-realsense2-camera-msgs \
    ros-jazzy-librealsense2 || true
  echo "次: bash $0 ros（sudo 不要）。D455 を一度抜き差しすると確実"
}

do_ros() {
  step "ros: realsense-ros $RSROS_REV を $WS でビルド"
  mkdir -p "$WS/src/third_party"
  clone_at "$RSROS_URL" "$RSROS_DIR" "$RSROS_REV"
  ros_env
  cd "$WS"
  colcon build --symlink-install --packages-up-to realsense2_camera \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -Drealsense2_DIR=/usr/local/lib/cmake/realsense2
  echo "完了。新しい端末で source $WS/install/setup.bash"
}

do_check() {
  step "check: デバイスと IMU"
  rs-enumerate-devices -s || true
  rs-enumerate-devices | grep -E 'Accel|Gyro' | head -4 || echo "IMU のストリームが見えません"
  ros_env
  set +u
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash"
  set -u
  local node
  node="$(ros2 pkg prefix realsense2_camera)/lib/realsense2_camera/realsense2_camera_node"
  echo "realsense2_camera: $(ros2 pkg prefix realsense2_camera)"
  ldd "$node" | grep realsense2 || true
}

case "${1:-}" in
  deps) do_deps ;;
  build) do_build ;;
  install) do_install ;;
  ros) do_ros ;;
  check) do_check ;;
  *) sed -n '2,20p' "$0"; exit 1 ;;
esac
