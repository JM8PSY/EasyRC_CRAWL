// control ライブラリの単体テスト（PC 上で実行）: esp32/tests/run.sh
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#include "control.h"

using namespace cugo;

static int failures = 0;
#define CHECK(cond)                                                     \
  do {                                                                  \
    if (!(cond)) {                                                      \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);       \
      failures++;                                                       \
    }                                                                   \
  } while (0)
#define NEAR(a, b) CHECK(std::fabs((a) - (b)) < 1e-4f)

static void test_motion_ramps_with_accel_limit() {
  Motion m(1.0f, 2.0f);
  CHECK(m.setTarget(1.0f, -1.0f));
  m.tick(0.1f);
  NEAR(m.left(), 0.2f);
  NEAR(m.right(), -0.2f);
  for (int i = 0; i < 10; i++) m.tick(0.1f);
  NEAR(m.left(), 1.0f);
  NEAR(m.right(), -1.0f);
}

static void test_motion_caps_dt_and_clamps() {
  Motion m(0.5f, 2.0f);
  m.setTarget(3.0f, -3.0f);  // ±1 にクランプ後 max_speed 0.5 倍
  NEAR(m.targetLeft(), 0.5f);
  NEAR(m.targetRight(), -0.5f);
  m.tick(10.0f);  // 停滞後の大きな dt は 0.1s に制限
  NEAR(m.left(), 0.2f);
}

static void test_motion_rejects_non_finite_and_stop_is_immediate() {
  Motion m(1.0f, 2.0f);
  CHECK(!m.setTarget(NAN, 0.0f));
  CHECK(!m.setTarget(0.0f, INFINITY));
  m.setTarget(1.0f, 1.0f);
  for (int i = 0; i < 10; i++) m.tick(0.1f);
  m.stop();
  NEAR(m.left(), 0.0f);
  NEAR(m.right(), 0.0f);
  NEAR(m.targetLeft(), 0.0f);
}

static void test_motion_set_max_speed_rescales_target() {
  Motion m(1.0f, 2.0f);
  m.setTarget(0.5f, -1.0f);
  CHECK(m.setMaxSpeed(0.4f));
  NEAR(m.maxSpeed(), 0.4f);
  NEAR(m.targetLeft(), 0.2f);  // 走行中に変えても即座に新しい上限が効く
  NEAR(m.targetRight(), -0.4f);
  CHECK(!m.setMaxSpeed(NAN));
  CHECK(!m.setMaxSpeed(-0.1f));
  CHECK(!m.setMaxSpeed(1.1f));
  NEAR(m.maxSpeed(), 0.4f);
}

static void test_side_output() {
  SideOutput o = toSideOutput(0.5f, false);
  NEAR(o.duty, 0.5f);
  CHECK(!o.reverse);
  o = toSideOutput(0.5f, true);
  NEAR(o.duty, 0.5f);
  CHECK(o.reverse);
  o = toSideOutput(-0.25f, false);
  NEAR(o.duty, 0.25f);
  CHECK(o.reverse);
}

static void test_watchdog() {
  Watchdog w(300);
  CHECK(!w.expired(10000));  // feed 前は発火しない
  w.feed(1000);
  CHECK(!w.expired(1300));
  CHECK(w.expired(1301));
  CHECK(!w.expired(1400));  // 1回だけ
  w.feed(0xFFFFFF00u);       // millis() の桁あふれ
  CHECK(!w.expired(0x00000010u));
  CHECK(w.expired(0x00000100u));
}

static void test_arbiter_first_is_driver_others_viewer() {
  DriverArbiter a(1000);
  uint8_t kicked = 0;
  CHECK(a.onConnect(1, 0, &kicked) == Role::Driver);
  CHECK(kicked == kNoClient);
  CHECK(a.onConnect(2, 500, &kicked) == Role::Viewer);
  CHECK(kicked == kNoClient);
  CHECK(a.isDriver(1));
  CHECK(!a.isDriver(2));
  CHECK(!a.onDisconnect(2));
  CHECK(a.onDisconnect(1));
  CHECK(!a.isDriver(1));
  CHECK(a.onConnect(3, 600, &kicked) == Role::Driver);
}

static void test_arbiter_takeover_from_silent_driver() {
  DriverArbiter a(1000);
  uint8_t kicked = 0;
  a.onConnect(1, 0, &kicked);
  a.touch(1, 500);
  CHECK(a.onConnect(2, 1400, &kicked) == Role::Viewer);  // 900ms 無言 → まだ有効
  CHECK(a.onConnect(3, 1501, &kicked) == Role::Driver);  // 1001ms 無言 → 引き継ぎ
  CHECK(kicked == 1);
  CHECK(!a.onDisconnect(1));  // 追い出された側の切断では停止しない
  CHECK(a.isDriver(3));
}

static void test_parse_command() {
  Command c = parseCommand("{\"l\":0.5,\"r\":-0.25}");
  CHECK(c.type == CommandType::Drive);
  NEAR(c.l, 0.5f);
  NEAR(c.r, -0.25f);
  c = parseCommand("{ \"r\" : 1 , \"l\" : -1 }");
  CHECK(c.type == CommandType::Drive);
  NEAR(c.l, -1.0f);
  NEAR(c.r, 1.0f);
  CHECK(parseCommand("{\"stop\":true}").type == CommandType::Stop);
  CHECK(parseCommand("{\"stop\":false}").type == CommandType::Invalid);
  CHECK(parseCommand("{\"l\":0.5}").type == CommandType::Invalid);
  CHECK(parseCommand("{\"l\":\"x\",\"r\":0}").type == CommandType::Invalid);
  CHECK(parseCommand("{\"l\":nan,\"r\":0}").type == CommandType::Invalid);
  CHECK(parseCommand("{\"l\":1e999,\"r\":0}").type == CommandType::Invalid);
  c = parseCommand("{\"max\":0.45}");
  CHECK(c.type == CommandType::SetMax);
  NEAR(c.value, 0.45f);
  CHECK(parseCommand("{\"max\":\"x\"}").type == CommandType::Invalid);
  CHECK(parseCommand("").type == CommandType::Invalid);
  CHECK(parseCommand(nullptr).type == CommandType::Invalid);
}

static void test_self_test_sequence() {
  SelfTest t(0.3f, 5000, 2000, 1000);
  float l = 9, r = 9;
  CHECK(!t.active());
  t.start(100);
  CHECK(t.active());
  t.update(100 + 4999, &l, &r);
  CHECK(t.step() == -1);  // カウントダウン中は停止
  NEAR(l, 0.0f);
  NEAR(r, 0.0f);
  t.update(100 + 5000, &l, &r);
  CHECK(t.step() == 0);  // 左 前進
  NEAR(l, 0.3f);
  NEAR(r, 0.0f);
  t.update(100 + 5000 + 2500, &l, &r);  // 1 つ目の休止
  CHECK(t.step() == 0);
  NEAR(l, 0.0f);
  t.update(100 + 5000 + 3000 * 2, &l, &r);  // 右 前進
  CHECK(t.step() == 2);
  NEAR(l, 0.0f);
  NEAR(r, 0.3f);
  t.update(100 + 5000 + 3000 * 8, &l, &r);  // 全工程終了
  CHECK(!t.active());
  NEAR(l, 0.0f);
  NEAR(r, 0.0f);
  CHECK(t.stepName(0) != nullptr);
  CHECK(t.stepName(SelfTest::kSteps) == nullptr);
}

static void test_self_test_abort() {
  SelfTest t(0.3f, 5000, 2000, 1000);
  float l, r;
  t.start(0);
  t.update(6000, &l, &r);
  t.abort();
  CHECK(!t.active());
}

static void test_parse_command_auto() {
  Command c = parseCommand("{\"auto\":true}");
  CHECK(c.type == CommandType::SetAuto);
  CHECK(c.on);
  c = parseCommand("{ \"auto\" : false }");
  CHECK(c.type == CommandType::SetAuto);
  CHECK(!c.on);
  CHECK(parseCommand("{\"auto\":1}").type == CommandType::Invalid);
  CHECK(parseCommand("{\"auto\":\"yes\"}").type == CommandType::Invalid);
}

static void test_motion_unscaled_target_ignores_max_speed() {
  Motion m(0.4f, 2.0f);
  CHECK(m.setTarget(0.5f, -0.5f, false));  // 自律: スライダーを掛けない
  NEAR(m.targetLeft(), 0.5f);
  NEAR(m.targetRight(), -0.5f);
  CHECK(m.setMaxSpeed(0.2f));  // 上限を変えても自律の目標は変わらない
  NEAR(m.targetLeft(), 0.5f);
  CHECK(m.setTarget(0.5f, -0.5f));  // 手動: 従来通り掛ける
  NEAR(m.targetLeft(), 0.1f);
  CHECK(m.setMaxSpeed(0.4f));
  NEAR(m.targetLeft(), 0.2f);
}

// ---- シリアルプロトコル ----
static void test_proto_checksum() {
  CHECK(protoChecksum("D,0.250,-0.250", 14) == 0x69);
  CHECK(protoChecksum("S", 1) == 0x53);
  CHECK(protoChecksum("", 0) == 0x00);
}

static void test_parse_serial_drive() {
  SerialCommand c = parseSerialLine("$D,0.250,-0.250*69");
  CHECK(c.type == SerialCommandType::Drive);
  NEAR(c.l, 0.25f);
  NEAR(c.r, -0.25f);
  c = parseSerialLine("$D,0.250,-0.250*69\r");  // CRLF も受け付ける
  CHECK(c.type == SerialCommandType::Drive);
  c = parseSerialLine("$D,0.250,-0.250*6a");  // 小文字の 16 進も可（不一致なので不正）
  CHECK(c.type == SerialCommandType::Invalid);
  CHECK(parseSerialLine("$D,0.250,-0.250*68").type == SerialCommandType::Invalid);  // チェックサム不一致
  CHECK(parseSerialLine("$S*53").type == SerialCommandType::Stop);
  CHECK(parseSerialLine("$S*54").type == SerialCommandType::Invalid);
}

// 正しいチェックサムをつけた行を作る（テスト用）
static std::string withSum(const char* body) {
  char tail[8];
  std::snprintf(tail, sizeof(tail), "*%02X", protoChecksum(body, std::strlen(body)));
  return std::string("$") + body + tail;
}

static void test_parse_serial_malformed() {
  CHECK(parseSerialLine(withSum("D,0.1,0.1").c_str()).type == SerialCommandType::Drive);
  CHECK(parseSerialLine(withSum("D,1,-1").c_str()).type == SerialCommandType::Drive);
  CHECK(parseSerialLine(withSum("D,.5,-.5").c_str()).type == SerialCommandType::Drive);
  CHECK(parseSerialLine(withSum("D,+0.5,0").c_str()).type == SerialCommandType::Drive);
  const char* invalidBodies[] = {"D,0.1",        "D,0.1,0.1,0.1", "D,,0.1",     "D,0.1,",     "D,abc,0",
                                 "D,0.1x,0",     "D,nan,0",       "D,inf,0",    "D,0,-inf",   "D,0x1,0",
                                 "D,1e2,0",      "D, 0.1,0",      "D,-,0",      "D,.,0",      "X,0,0",
                                 "S,0",          "",              "D",          "T,A,0,0,0,0,0", "D,1..0,0"};
  for (const char* b : invalidBodies) {
    SerialCommand c = parseSerialLine(withSum(b).c_str());
    if (c.type != SerialCommandType::Invalid) std::printf("  accepted: %s\n", b);
    CHECK(c.type == SerialCommandType::Invalid);
  }
  CHECK(parseSerialLine("D,0.250,-0.250*69").type == SerialCommandType::Invalid);   // $ なし
  CHECK(parseSerialLine("$D,0.250,-0.250").type == SerialCommandType::Invalid);     // * なし
  CHECK(parseSerialLine("$D,0.250,-0.250*6").type == SerialCommandType::Invalid);   // 1 桁
  CHECK(parseSerialLine("$D,0.250,-0.250*699").type == SerialCommandType::Invalid); // 3 桁
  CHECK(parseSerialLine("$D,0.250,-0.250*G9").type == SerialCommandType::Invalid);
  CHECK(parseSerialLine("$D,0.2*50,-0.250*69").type == SerialCommandType::Invalid);
  CHECK(parseSerialLine("").type == SerialCommandType::Invalid);
  CHECK(parseSerialLine(nullptr).type == SerialCommandType::Invalid);
  // 非常に長い数値（オーバーフロー → inf）も拒否
  std::string huge = "D,1";
  huge += std::string(400, '0');
  huge += ",0";
  CHECK(parseSerialLine(withSum(huge.c_str()).c_str()).type == SerialCommandType::Invalid);
}

static void test_parse_serial_clamps() {
  SerialCommand c = parseSerialLine(withSum("D,2.5,-3").c_str());
  CHECK(c.type == SerialCommandType::Drive);
  NEAR(c.l, 1.0f);
  NEAR(c.r, -1.0f);
}

static void test_format_status() {
  char buf[64];
  size_t n = formatStatus(buf, sizeof(buf), Mode::Auto, 0.25f, -0.25f, 0.5f, 5, 0);
  CHECK(std::strcmp(buf, "$T,A,0.250,-0.250,0.50,5,0*26\n") == 0);
  CHECK(n == std::strlen(buf));
  formatStatus(buf, sizeof(buf), Mode::Idle, -0.0001f, 0.0f, 0.5f, 0, 0);  // -0.000 にしない
  CHECK(std::strcmp(buf, "$T,I,0.000,0.000,0.50,0,0*06\n") == 0);
  formatStatus(buf, sizeof(buf), Mode::Manual, 0.1f, 0.0f, 0.5f, 3, 12);
  CHECK(std::strcmp(buf, "$T,M,0.100,0.000,0.50,3,12*33\n") == 0);
  formatStatus(buf, sizeof(buf), Mode::Test, 0.3f, 0.0f, 0.5f, 0, 0);
  CHECK(buf[3] == 'X');
  // 出力した行は自分のパーサーでチェックサムが一致する形式
  CHECK(std::strchr(buf, '*') != nullptr);
  CHECK(formatStatus(buf, 8, Mode::Idle, 0, 0, 0.5f, 0, 0) == 0);  // バッファ不足は空
  CHECK(buf[0] == '\0');
}

static void test_line_buffer() {
  LineBuffer<16> lb;
  // '$' 行以外の文字はデバッグコマンド
  CHECK(lb.feed('t') == LineEvent::DebugChar);
  CHECK(lb.feed('\n') == LineEvent::None);  // 単独の改行は無視
  CHECK(lb.feed('\r') == LineEvent::None);
  const char* line = "$S*53\r\n";
  LineEvent ev = LineEvent::None;
  for (const char* p = line; *p; p++) {
    ev = lb.feed(*p);
    if (*(p + 1)) CHECK(ev == LineEvent::None);  // 行の途中の 't' 's' はコマンドにしない
  }
  CHECK(ev == LineEvent::Line);
  CHECK(std::strcmp(lb.line(), "$S*53") == 0);  // CR は除く
  CHECK(lb.feed('s') == LineEvent::DebugChar);
  // 長すぎる行は捨てて 1 回だけ Overflow
  CHECK(lb.feed('$') == LineEvent::None);
  int overflow = 0;
  for (int i = 0; i < 40; i++) {
    if (lb.feed('t') == LineEvent::Overflow) overflow++;
  }
  CHECK(overflow == 0);
  CHECK(lb.feed('\n') == LineEvent::Overflow);
  CHECK(lb.feed('?') == LineEvent::DebugChar);  // その後は通常に戻る
  // 容量ちょうど（15 文字）は受け付ける
  for (const char* p = "$D,0.1,0.1*0000"; *p; p++) lb.feed(*p);
  CHECK(lb.feed('\n') == LineEvent::Line);
  CHECK(std::strlen(lb.line()) == 15);
}

// ---- モード調停 ----
static ModeArbiter makeArbiter() { return ModeArbiter(300, 300, 0.5f, 0.05f); }

static void test_mode_idle_and_auto_without_driver() {
  ModeArbiter a = makeArbiter();
  Decision d = a.update(0, false);
  CHECK(d.mode == Mode::Idle);
  NEAR(d.l, 0.0f);
  CHECK(a.autoAllowed());  // 操縦者なし → 自律可
  CHECK(a.flags(0) == 1);
  CHECK(a.jetsonDrive(10, 0.3f, -0.2f));
  d = a.update(20, false);
  CHECK(d.mode == Mode::Auto);
  NEAR(d.l, 0.3f);
  NEAR(d.r, -0.2f);
  CHECK(!d.scaled);  // Web スライダーは掛けない
  CHECK(!d.stopNow);
  CHECK(a.flags(20) == (1 | 4));
}

static void test_mode_auto_clamped_to_cap() {
  ModeArbiter a = makeArbiter();
  a.jetsonDrive(0, 1.0f, -0.9f);
  Decision d = a.update(10, false);
  CHECK(d.mode == Mode::Auto);
  NEAR(d.l, 0.5f);
  NEAR(d.r, -0.5f);
  CHECK(!a.jetsonDrive(20, NAN, 0.0f));  // 非有限値は拒否（指令も維持しない）
  CHECK(!a.jetsonDrive(20, 0.0f, INFINITY));
  NEAR(a.cap(), 0.5f);
}

static void test_mode_driver_connect_is_manual() {
  ModeArbiter a = makeArbiter();
  a.jetsonDrive(0, 0.3f, 0.3f);
  a.driverConnected();
  CHECK(!a.autoAllowed());
  Decision d = a.update(10, false);
  CHECK(d.mode == Mode::Manual);
  NEAR(d.l, 0.0f);  // Web 指令はまだない
  CHECK(d.scaled);
  a.webDrive(20, 0.8f, -0.8f);
  d = a.update(30, false);
  CHECK(d.mode == Mode::Manual);
  NEAR(d.l, 0.8f);  // 手動は cap でクランプしない（スライダーを掛ける）
  NEAR(d.r, -0.8f);
  CHECK(d.scaled);
  CHECK(a.flags(30) == (2 | 4));
}

static void test_mode_toggle_auto_and_takeover() {
  ModeArbiter a = makeArbiter();
  a.driverConnected();
  a.setAutoAllowed(true);
  a.jetsonDrive(0, 0.4f, 0.4f);
  a.webDrive(0, 0.0f, 0.0f);  // ブラウザは 0 を送り続ける
  Decision d = a.update(10, false);
  CHECK(d.mode == Mode::Auto);
  NEAR(d.l, 0.4f);
  a.webDrive(20, 0.05f, -0.05f);  // 0.05 以下は奪還しない
  CHECK(a.autoAllowed());
  CHECK(a.update(30, false).mode == Mode::Auto);
  a.webDrive(40, 0.0f, 0.06f);  // 0.05 超 → 手動に奪還
  CHECK(!a.autoAllowed());
  d = a.update(50, false);
  CHECK(d.mode == Mode::Manual);
  NEAR(d.r, 0.06f);
  a.setAutoAllowed(true);
  CHECK(a.update(60, false).mode == Mode::Auto);
  a.setAutoAllowed(false);
  CHECK(a.update(70, false).mode == Mode::Manual);
}

static void test_mode_stop_button_and_disconnect() {
  ModeArbiter a = makeArbiter();
  a.driverConnected();
  a.setAutoAllowed(true);
  a.jetsonDrive(0, 0.4f, 0.4f);
  a.webStop(10);
  CHECK(!a.autoAllowed());
  Decision d = a.update(20, false);
  CHECK(d.mode == Mode::Manual);
  NEAR(d.l, 0.0f);
  a.webDrive(30, 0.5f, 0.5f);
  a.webStop(40);  // 停止ボタンは Web 指令も 0 にする
  d = a.update(50, false);
  NEAR(d.l, 0.0f);
  a.driverDisconnected();
  CHECK(a.autoAllowed());  // 切断で自律可に戻る
  a.jetsonDrive(60, 0.2f, 0.2f);
  d = a.update(70, false);
  CHECK(d.mode == Mode::Auto);
  NEAR(d.l, 0.2f);
  CHECK(a.flags(70) == (1 | 4));
}

static void test_mode_self_test_overrides() {
  ModeArbiter a = makeArbiter();
  a.jetsonDrive(0, 0.4f, 0.4f);
  Decision d = a.update(10, true);
  CHECK(d.mode == Mode::Test);
  NEAR(d.l, 0.0f);
  a.driverConnected();
  a.webDrive(20, 1.0f, 1.0f);
  CHECK(a.update(30, true).mode == Mode::Test);
}

static void test_mode_watchdogs_are_separate() {
  ModeArbiter a = makeArbiter();
  a.driverConnected();
  a.setAutoAllowed(true);
  a.jetsonDrive(0, 0.3f, 0.3f);
  a.webDrive(0, 0.0f, 0.0f);
  CHECK(a.update(300, false).mode == Mode::Auto);  // ちょうど 300ms は有効
  Decision d = a.update(301, false);  // Jetson 途絶 → 即時停止
  CHECK(d.mode == Mode::Idle);
  CHECK(d.stopNow);
  NEAR(d.l, 0.0f);
  d = a.update(302, false);
  CHECK(!d.stopNow);  // 1 回だけ
  // Web が途絶えても自律中なら止めない
  a.jetsonDrive(400, 0.3f, 0.3f);
  d = a.update(410, false);
  CHECK(d.mode == Mode::Auto);
  CHECK(!d.stopNow);
  // 手動中の Web 途絶 → 即時停止。Jetson の途絶は手動に影響しない
  a.setAutoAllowed(false);
  a.webDrive(500, 0.6f, 0.6f);
  d = a.update(800, false);
  CHECK(d.mode == Mode::Manual);
  NEAR(d.l, 0.6f);
  CHECK(!d.stopNow);  // Jetson は 700 で途絶しているが手動なので関係ない
  d = a.update(801, false);
  CHECK(d.mode == Mode::Manual);
  CHECK(d.stopNow);
  NEAR(d.l, 0.0f);
}

static void test_mode_jetson_stop_and_wraparound() {
  ModeArbiter a = makeArbiter();
  a.jetsonDrive(0xFFFFFF00u, 0.3f, 0.3f);
  Decision d = a.update(0x00000010u, false);  // millis() の桁あふれをまたいでも有効
  CHECK(d.mode == Mode::Auto);
  NEAR(d.l, 0.3f);
  a.jetsonStop(0x00000020u);
  d = a.update(0x00000030u, false);
  CHECK(d.mode == Mode::Auto);  // $S も有効な指令（ハートビート）
  NEAR(d.l, 0.0f);
  d = a.update(0x00000200u, false);
  CHECK(d.mode == Mode::Idle);
  CHECK(d.stopNow);
  // flags() を先に呼んでも即時停止のイベントは失われない
  a.jetsonDrive(0x300u, 0.3f, 0.3f);
  CHECK(a.update(0x310u, false).mode == Mode::Auto);
  CHECK(a.flags(0x700u) == 1);  // 途絶済みなので bit2 なし
  d = a.update(0x700u, false);
  CHECK(d.stopNow);
  // 途絶後に millis() が一周して最後の受信時刻に近づいても復活しない
  d = a.update(0x00000020u + 100u, false);
  CHECK(d.mode == Mode::Idle);
}

int main() {
  test_motion_ramps_with_accel_limit();
  test_motion_caps_dt_and_clamps();
  test_motion_rejects_non_finite_and_stop_is_immediate();
  test_motion_set_max_speed_rescales_target();
  test_side_output();
  test_watchdog();
  test_arbiter_first_is_driver_others_viewer();
  test_arbiter_takeover_from_silent_driver();
  test_parse_command();
  test_self_test_sequence();
  test_self_test_abort();
  test_parse_command_auto();
  test_motion_unscaled_target_ignores_max_speed();
  test_proto_checksum();
  test_parse_serial_drive();
  test_parse_serial_malformed();
  test_parse_serial_clamps();
  test_format_status();
  test_line_buffer();
  test_mode_idle_and_auto_without_driver();
  test_mode_auto_clamped_to_cap();
  test_mode_driver_connect_is_manual();
  test_mode_toggle_auto_and_takeover();
  test_mode_stop_button_and_disconnect();
  test_mode_self_test_overrides();
  test_mode_watchdogs_are_separate();
  test_mode_jetson_stop_and_wraparound();
  if (failures) {
    std::printf("%d failure(s)\n", failures);
    return 1;
  }
  std::printf("all tests passed\n");
  return 0;
}
