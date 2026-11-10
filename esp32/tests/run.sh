#!/bin/sh
# control ライブラリの単体テストを PC 上でビルドして実行する
set -e
cd "$(dirname "$0")/.."
out="${TMPDIR:-/tmp}/cugo_control_test"
c++ -std=c++17 -Wall -Wextra -Werror -Ilib/control/src \
  tests/test_control.cpp lib/control/src/control.cpp -o "$out"
"$out"
