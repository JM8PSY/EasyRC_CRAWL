// 設定。変更したら再ビルドして書き込む（pio run -t upload）
#pragma once

#include <cstdint>

// ---- Wi-Fi ----
// 既存の Wi-Fi（ルーター）に参加する場合だけ SSID を設定する。
// 空のまま、または接続できなかった場合は ESP32 自身がアクセスポイントになる。
#define WIFI_STA_SSID ""
#define WIFI_STA_PASS ""
constexpr uint32_t WIFI_STA_TIMEOUT_MS = 15000;

// アクセスポイントモードの SSID / パスワード（パスワードは 8 文字以上）
#define WIFI_AP_SSID "CuGo-RC"
#define WIFI_AP_PASS "cugo-rc-1234"

// http://cugo.local でも開ける（mDNS 対応の端末のみ）
#define MDNS_NAME "cugo"

// ---- モーター（MD20A: PWM + DIR）----
constexpr int LEFT_PWM_PIN = 25;
constexpr int LEFT_DIR_PIN = 26;
constexpr bool LEFT_INVERT = false;

constexpr int RIGHT_PWM_PIN = 32;
constexpr int RIGHT_DIR_PIN = 33;
constexpr bool RIGHT_INVERT = true;  // 前進で逆転する場合は切り替える

constexpr uint32_t PWM_FREQ_HZ = 20000;  // MD20A の上限 20kHz
constexpr uint8_t PWM_BITS = 10;         // 20kHz では 10bit (0〜1023) まで

constexpr float MAX_SPEED = 1.0f;  // 出力デューティの上限 (0〜1)
constexpr float MAX_ACCEL = 2.0f;  // 1 秒あたりの速度変化量の上限

// ---- 安全 ----
constexpr uint32_t WATCHDOG_MS = 300;   // 指令がこの時間途絶えたら停止
constexpr uint32_t TAKEOVER_MS = 1000;  // 操縦者がこの時間無言なら新しい接続が引き継ぐ
constexpr uint32_t CONTROL_PERIOD_MS = 20;  // 制御周期 (50Hz)

// ---- 動作テスト（BOOT ボタン / シリアルで 't'）----
constexpr float SELFTEST_SPEED = 0.3f;
constexpr uint32_t SELFTEST_DELAY_MS = 5000;  // 開始までの待ち時間
constexpr uint32_t SELFTEST_RUN_MS = 2000;    // 各動作の時間
constexpr uint32_t SELFTEST_PAUSE_MS = 1000;  // 動作の間の停止時間

// ---- ボード ----
constexpr int BOOT_BUTTON_PIN = 0;  // DevKit の BOOT ボタン
constexpr int LED_PIN = 2;          // DevKit の青色 LED
