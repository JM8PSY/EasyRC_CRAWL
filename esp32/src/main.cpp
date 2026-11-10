// CuGo V3 Wi-Fi 遠隔操縦 + Jetson 自律走行（ESP32 + Cytron MD20A ×2）
#include <Arduino.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <WiFi.h>

#include "config.h"
#include "control.h"
#include "web_assets.h"

using namespace cugo;

namespace {

constexpr uint8_t LEFT_CH = 0;
constexpr uint8_t RIGHT_CH = 1;
constexpr uint32_t PWM_MAX = (1u << PWM_BITS) - 1;
constexpr size_t MAX_MESSAGE = 128;

WebServer http(80);
WebSocketsServer ws(81);
Preferences prefs;  // 出力上限の保存先（NVS）

Motion motion(MAX_SPEED, MAX_ACCEL);
DriverArbiter arbiter(TAKEOVER_MS);
ModeArbiter modes(WATCHDOG_MS, JETSON_WATCHDOG_MS, AUTO_MAX_DUTY, MANUAL_TAKEOVER_INPUT);
SelfTest selfTest(SELFTEST_SPEED, SELFTEST_DELAY_MS, SELFTEST_RUN_MS, SELFTEST_PAUSE_MS);
LineBuffer<SERIAL_LINE_MAX> serialLine;

uint32_t lastTick = 0;
uint32_t lastStatus = 0;
uint32_t badLines = 0;  // 起動以降の不正行の数（$T で報告）
int lastReportedStep = -2;
Mode lastSentMode = Mode::Idle;
bool lastSentAuto = true;

void writeSide(uint8_t ch, int dirPin, bool invert, float v) {
  SideOutput o = toSideOutput(v, invert);
  digitalWrite(dirPin, o.reverse ? HIGH : LOW);
  ledcWrite(ch, static_cast<uint32_t>(lroundf(o.duty * PWM_MAX)));
}

void writeOutputs() {
  writeSide(LEFT_CH, LEFT_DIR_PIN, LEFT_INVERT, motion.left());
  writeSide(RIGHT_CH, RIGHT_DIR_PIN, RIGHT_INVERT, motion.right());
}

void stopNow(const char* reason) {
  motion.stop();
  writeOutputs();
  if (reason) Serial.printf("[停止] %s\n", reason);
}

void setupMotors() {
  // 起動直後に暴走しないよう、何より先に PWM=0 / DIR=Low にする
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  digitalWrite(LEFT_DIR_PIN, LOW);
  digitalWrite(RIGHT_DIR_PIN, LOW);
  ledcSetup(LEFT_CH, PWM_FREQ_HZ, PWM_BITS);
  ledcSetup(RIGHT_CH, PWM_FREQ_HZ, PWM_BITS);
  ledcAttachPin(LEFT_PWM_PIN, LEFT_CH);
  ledcAttachPin(RIGHT_PWM_PIN, RIGHT_CH);
  writeOutputs();
}

// ---- 動作テスト ----
void startSelfTest() {
  stopNow(nullptr);
  selfTest.start(millis());
  lastReportedStep = -2;
  Serial.printf("[テスト] %lu 秒後に開始します（BOOT ボタン / 's' で中止）\n",
                static_cast<unsigned long>(SELFTEST_DELAY_MS / 1000));
}

void abortSelfTest(const char* reason) {
  if (!selfTest.active()) return;
  selfTest.abort();
  stopNow(reason);
}

void reportSelfTest(uint32_t now) {
  int step = selfTest.step();
  if (step == -1) {
    int left = static_cast<int>((selfTest.countdownLeft(now) + 999) / 1000);
    if (left != lastReportedStep) Serial.printf("[テスト] 開始まで %d 秒...\n", left);
    lastReportedStep = left;
    return;
  }
  int code = 100 + step;  // カウントダウンの秒数と区別する
  if (code == lastReportedStep) return;
  lastReportedStep = code;
  if (step < SelfTest::kSteps) {
    Serial.printf("[テスト] %d/%d %s\n", step + 1, SelfTest::kSteps, selfTest.stepName(step));
  } else {
    Serial.println("[テスト] 終了（停止しました）");
  }
}

const char* modeName(Mode m) {
  switch (m) {
    case Mode::Auto:
      return "自律";
    case Mode::Manual:
      return "手動";
    case Mode::Test:
      return "テスト";
    default:
      return "待機";
  }
}

// ---- WebSocket ----
// {"mode":"I|A|M|X","auto":true|false}
void formatMode(char* msg, size_t size) {
  snprintf(msg, size, "{\"mode\":\"%c\",\"auto\":%s}", static_cast<char>(modes.mode()),
           modes.autoAllowed() ? "true" : "false");
}

void sendMode(uint8_t id) {
  char msg[40];
  formatMode(msg, sizeof(msg));
  ws.sendTXT(id, msg);
}

// モードか自律許可が変わったら全クライアントへ送る
void broadcastModeIfChanged() {
  if (modes.mode() == lastSentMode && modes.autoAllowed() == lastSentAuto) return;
  if (modes.mode() != lastSentMode) Serial.printf("[モード] %s\n", modeName(modes.mode()));
  if (modes.autoAllowed() != lastSentAuto) {
    Serial.printf("[モード] 自律走行の許可: %s\n", modes.autoAllowed() ? "あり" : "なし");
  }
  lastSentMode = modes.mode();
  lastSentAuto = modes.autoAllowed();
  char msg[40];
  formatMode(msg, sizeof(msg));
  ws.broadcastTXT(msg);
}

void sendHello(uint8_t id, Role role) {
  char msg[64];
  snprintf(msg, sizeof(msg), "{\"role\":\"%s\",\"max\":%.2f}", role == Role::Driver ? "driver" : "viewer",
           motion.maxSpeed());
  ws.sendTXT(id, msg);
}

void broadcastMax() {
  char msg[32];
  snprintf(msg, sizeof(msg), "{\"max\":%.2f}", motion.maxSpeed());
  ws.broadcastTXT(msg);
}

void setMaxSpeed(float v) {
  if (v < MIN_MAX_SPEED || !motion.setMaxSpeed(v)) {
    Serial.printf("[警告] 出力上限 %.3f は範囲外のため無視しました\n", v);
  } else {
    prefs.putFloat("max", motion.maxSpeed());
    Serial.printf("[設定] 出力上限 %.0f%%\n", motion.maxSpeed() * 100);
  }
  broadcastMax();  // 無視した場合も現在値を送り、画面のスライダーを戻す
}

void handleMessage(uint8_t id, const uint8_t* payload, size_t length) {
  if (!arbiter.isDriver(id) || length >= MAX_MESSAGE) return;
  char buf[MAX_MESSAGE];
  memcpy(buf, payload, length);
  buf[length] = '\0';

  Command cmd = parseCommand(buf);
  uint32_t now = millis();
  switch (cmd.type) {
    case CommandType::Stop:
      abortSelfTest("停止ボタンでテストを中止しました");
      stopNow(nullptr);
      modes.webStop(now);  // 自律走行の許可も取り消す
      break;
    case CommandType::Drive:
      // 出力は制御周期で ModeArbiter が決める（テスト中・自律中は使われない）
      modes.webDrive(now, cmd.l, cmd.r);
      break;
    case CommandType::SetMax:
      setMaxSpeed(cmd.value);
      modes.webTouch(now);
      break;
    case CommandType::SetAuto:
      modes.setAutoAllowed(cmd.on);
      modes.webTouch(now);
      break;
    case CommandType::Invalid:
      // ログが $ 行や改行で Jetson 側の解析を乱さないよう、制御文字は置き換える
      for (char* p = buf; *p; p++) {
        if (static_cast<unsigned char>(*p) < 0x20) *p = '?';
      }
      Serial.printf("[警告] 不正な指令を無視しました: %.60s\n", buf);
      return;
  }
  arbiter.touch(id, now);
}

void onWsEvent(uint8_t id, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED: {
      uint8_t kicked;
      Role role = arbiter.onConnect(id, millis(), &kicked);
      if (kicked != kNoClient) {
        stopNow("応答のない操縦者から操縦権を引き継ぎました");
        ws.disconnect(kicked);
      }
      if (role == Role::Driver) {
        modes.driverConnected();  // 操縦者が来たら手動（自律走行は許可されるまで止める）
        Serial.printf("[接続] 操縦者 #%u\n", id);
      } else {
        Serial.printf("[接続] 閲覧のみ #%u\n", id);
      }
      sendHello(id, role);
      sendMode(id);
      break;
    }
    case WStype_DISCONNECTED:
      if (arbiter.onDisconnect(id)) {
        abortSelfTest(nullptr);
        stopNow("操縦者が切断しました");
        modes.driverDisconnected();  // 自律走行の許可に戻る
      }
      break;
    case WStype_TEXT:
      handleMessage(id, payload, length);
      break;
    default:
      break;
  }
}

// ---- HTTP ----
void setupHttp() {
  for (size_t i = 0; i < WEB_ASSET_COUNT; i++) {
    const WebAsset* a = &WEB_ASSETS[i];
    http.on(a->path, HTTP_GET, [a]() {
      http.sendHeader("Content-Encoding", "gzip");
      http.sendHeader("Cache-Control", "no-cache");
      http.send_P(200, a->mime, reinterpret_cast<const char*>(a->data), a->length);
    });
  }
  http.onNotFound([]() { http.send(404, "text/plain", "Not Found"); });
  http.begin();
}

// ---- Wi-Fi ----
void setupWifi() {
  bool connected = false;
  if (strlen(WIFI_STA_SSID) > 0) {
    Serial.printf("[Wi-Fi] %s に接続中...\n", WIFI_STA_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASS);
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_STA_TIMEOUT_MS) delay(100);
    connected = WiFi.status() == WL_CONNECTED;
    if (connected) {
      Serial.printf("[Wi-Fi] 接続しました: http://%s/\n", WiFi.localIP().toString().c_str());
    } else {
      Serial.println("[Wi-Fi] 接続できないためアクセスポイントモードに切り替えます");
      WiFi.disconnect(true);
    }
  }
  if (!connected) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
    Serial.printf("[Wi-Fi] アクセスポイント \"%s\"（パスワード \"%s\"）\n", WIFI_AP_SSID, WIFI_AP_PASS);
    Serial.printf("[Wi-Fi] 接続後 http://%s/ を開いてください\n", WiFi.softAPIP().toString().c_str());
  }
  WiFi.setSleep(false);  // 省電力による遅延を避ける
  if (MDNS.begin(MDNS_NAME)) {
    MDNS.addService("http", "tcp", 80);
    Serial.printf("[Wi-Fi] http://%s.local/ でも開けます\n", MDNS_NAME);
  }
}

// ---- Jetson（USB シリアル）----
void handleSerialLine(const char* line, uint32_t now) {
  SerialCommand c = parseSerialLine(line);
  switch (c.type) {
    case SerialCommandType::Drive:
      modes.jetsonDrive(now, c.l, c.r);
      break;
    case SerialCommandType::Stop:
      modes.jetsonStop(now);
      // 自律で走っているときだけ即時停止する（手動・テスト中は Jetson に権限がない）
      if (modes.mode() == Mode::Auto && (motion.left() != 0.0f || motion.right() != 0.0f)) {
        stopNow("Jetson からの停止指令");
      }
      break;
    case SerialCommandType::Invalid:
      badLines++;
      break;
  }
}

void sendStatus(uint32_t now) {
  char line[80];
  size_t n = formatStatus(line, sizeof(line), modes.mode(), motion.left(), motion.right(), AUTO_MAX_DUTY,
                          modes.flags(now), badLines);
  if (n > 0) Serial.write(reinterpret_cast<const uint8_t*>(line), n);
}

// ---- 入力（BOOT ボタン / シリアル）----
void pollButton(uint32_t now) {
  static bool lastPressed = false;
  static uint32_t lastChange = 0;
  bool pressed = digitalRead(BOOT_BUTTON_PIN) == LOW;
  if (pressed == lastPressed || now - lastChange < 50) return;
  lastPressed = pressed;
  lastChange = now;
  if (!pressed) return;
  if (selfTest.active()) abortSelfTest("BOOT ボタンでテストを中止しました");
  else startSelfTest();
}

void printStatus() {
  uint32_t now = millis();
  Serial.printf("[状態] L %+.2f / R %+.2f  目標 L %+.2f / R %+.2f  上限 %.0f%%  操縦者 %s  テスト %s\n",
                motion.left(), motion.right(), motion.targetLeft(), motion.targetRight(), motion.maxSpeed() * 100,
                arbiter.hasDriver() ? "あり" : "なし", selfTest.active() ? "実行中" : "なし");
  Serial.printf("[状態] モード %s  自律許可 %s  Jetson %s  自律上限 %.0f%%  不正行 %lu\n", modeName(modes.mode()),
                modes.autoAllowed() ? "あり" : "なし", modes.jetsonAlive(now) ? "受信中" : "なし",
                AUTO_MAX_DUTY * 100, static_cast<unsigned long>(badLines));
}

void handleDebugChar(char c) {
  switch (c) {
    case 't':
      if (!selfTest.active()) startSelfTest();
      break;
    case 's':
      abortSelfTest("シリアルからテストを中止しました");
      break;
    case '?':
      printStatus();
      break;
    default:
      break;
  }
}

// '$' で始まる行は Jetson の指令、それ以外の 1 文字はデバッグコマンド（改行なしで打てる）
void pollSerial() {
  while (Serial.available()) {
    switch (serialLine.feed(static_cast<char>(Serial.read()))) {
      case LineEvent::Line:
        handleSerialLine(serialLine.line(), millis());
        break;
      case LineEvent::Overflow:
        badLines++;
        break;
      case LineEvent::DebugChar:
        handleDebugChar(serialLine.debugChar());
        break;
      case LineEvent::None:
        break;
    }
  }
}

void updateLed(uint32_t now) {
  bool on;
  if (selfTest.active()) on = (now / (selfTest.step() == -1 ? 125 : 500)) % 2;  // テスト中は点滅
  else on = arbiter.hasDriver() || modes.mode() == Mode::Auto;                  // 操縦者接続中・自律中は点灯
  digitalWrite(LED_PIN, on ? HIGH : LOW);
}

void controlStep(uint32_t now) {
  float dt = (now - lastTick) / 1000.0f;
  lastTick = now;

  Decision d = modes.update(now, selfTest.active());
  if (d.mode == Mode::Test) {
    float l, r;
    selfTest.update(now, &l, &r);
    reportSelfTest(now);
    if (selfTest.active()) motion.setTarget(l, r);
    else stopNow(nullptr);
  } else {
    if (d.stopNow) stopNow(d.mode == Mode::Manual ? "指令が途絶えました" : "Jetson の指令が途絶えました");
    motion.setTarget(d.l, d.r, d.scaled);  // 自律はスライダーを掛けない（±AUTO_MAX_DUTY は調停でクランプ済み）
  }
  motion.tick(dt);
  writeOutputs();
}

}  // namespace

void setup() {
  setupMotors();
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  Serial.setRxBufferSize(1024);
  Serial.begin(SERIAL_BAUD);
  delay(200);
  Serial.println();
  Serial.println("=== CuGo RC (ESP32) ===");
  Serial.printf("左: PWM=GPIO%d DIR=GPIO%d  右: PWM=GPIO%d DIR=GPIO%d  PWM %luHz\n", LEFT_PWM_PIN, LEFT_DIR_PIN,
                RIGHT_PWM_PIN, RIGHT_DIR_PIN, static_cast<unsigned long>(PWM_FREQ_HZ));

  prefs.begin("cugo", false);
  float savedMax = prefs.isKey("max") ? prefs.getFloat("max", MAX_SPEED) : MAX_SPEED;
  if (savedMax < MIN_MAX_SPEED || !motion.setMaxSpeed(savedMax)) motion.setMaxSpeed(MAX_SPEED);
  Serial.printf("出力上限 %.0f%%（Web 画面で変更できます）\n", motion.maxSpeed() * 100);

  setupWifi();
  setupHttp();
  ws.begin();
  ws.onEvent(onWsEvent);
  ws.enableHeartbeat(2000, 2000, 2);  // 応答のない接続を切る（停止は watchdog が先に行う）

  Serial.println("BOOT ボタン or シリアル 't' で動作テスト、'?' で状態表示");
  Serial.printf("Jetson: USB シリアルで $D / $S を受信、$T を %luHz で送信（自律上限 %.0f%%）\n",
                static_cast<unsigned long>(1000 / STATUS_PERIOD_MS), AUTO_MAX_DUTY * 100);
  // 起動中（Wi-Fi 接続待ち）に溜まった古い指令は使わない
  while (Serial.available()) Serial.read();
  lastTick = lastStatus = millis();
}

void loop() {
  http.handleClient();
  ws.loop();
  pollSerial();

  uint32_t now = millis();
  pollButton(now);
  if (now - lastTick >= CONTROL_PERIOD_MS) controlStep(now);
  broadcastModeIfChanged();
  if (now - lastStatus >= STATUS_PERIOD_MS) {
    lastStatus = now;
    sendStatus(now);
  }
  updateLed(now);
}
