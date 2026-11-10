// control ライブラリの単体テスト（PC 上で実行）: esp32/tests/run.sh
#include <cmath>
#include <cstdio>

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
  if (failures) {
    std::printf("%d failure(s)\n", failures);
    return 1;
  }
  std::printf("all tests passed\n");
  return 0;
}
