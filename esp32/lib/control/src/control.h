// 操縦ロジック（Arduino 非依存。PC 上で単体テストできる）
#pragma once

#include <cstdint>

namespace cugo {

// 制御が停滞した後でも 1 回の tick で進める時間の上限 [s]（加速度制限の飛び越し防止）
constexpr float kMaxTickDt = 0.1f;

// 左右速度 (-1.0〜1.0) を加速度制限つきで目標へ近づける
class Motion {
 public:
  Motion(float maxSpeed, float maxAccel) : maxSpeed_(maxSpeed), maxAccel_(maxAccel) {}

  // ±1 にクランプして max_speed を掛ける。非有限値は拒否して false
  bool setTarget(float l, float r);
  // 加速度制限を無視して即座に 0
  void stop();
  void tick(float dt);

  float left() const { return curL_; }
  float right() const { return curR_; }
  float targetLeft() const { return tgtL_; }
  float targetRight() const { return tgtR_; }

 private:
  float maxSpeed_;
  float maxAccel_;
  float tgtL_ = 0, tgtR_ = 0;
  float curL_ = 0, curR_ = 0;
};

struct SideOutput {
  float duty;    // 0.0〜1.0
  bool reverse;  // DIR ピンを High にするか
};
SideOutput toSideOutput(float v, bool invert);

// 指令が timeout を超えて途絶えたら 1 回だけ expired() が true を返す
class Watchdog {
 public:
  explicit Watchdog(uint32_t timeoutMs) : timeoutMs_(timeoutMs) {}
  void feed(uint32_t now) {
    last_ = now;
    armed_ = true;
  }
  bool expired(uint32_t now);

 private:
  uint32_t timeoutMs_;
  uint32_t last_ = 0;
  bool armed_ = false;
};

constexpr uint8_t kNoClient = 0xFF;
enum class Role { Driver, Viewer };

// 最初に接続したクライアントだけが操縦できる。
// 操縦者が takeover 時間以上無言なら（Wi-Fi 断で残った半開き接続）、新しい接続が引き継ぐ
class DriverArbiter {
 public:
  explicit DriverArbiter(uint32_t takeoverMs) : takeoverMs_(takeoverMs) {}

  // kicked: 操縦権を奪われたクライアント（なければ kNoClient）
  Role onConnect(uint8_t id, uint32_t now, uint8_t* kicked);
  // 操縦者が切断したら true（呼び出し側で停止する）
  bool onDisconnect(uint8_t id);
  bool isDriver(uint8_t id) const { return driver_ != kNoClient && driver_ == id; }
  bool hasDriver() const { return driver_ != kNoClient; }
  void touch(uint8_t id, uint32_t now) {
    if (isDriver(id)) last_ = now;
  }

 private:
  uint32_t takeoverMs_;
  uint8_t driver_ = kNoClient;
  uint32_t last_ = 0;
};

enum class CommandType { Invalid, Stop, Drive };
struct Command {
  CommandType type;
  float l;
  float r;
};
// {"l":0.5,"r":-0.5} / {"stop":true} を解釈する
Command parseCommand(const char* text);

// 一人で確認するための動作テスト。待ち時間のあと左右を順に動かす
class SelfTest {
 public:
  static constexpr int kSteps = 8;

  SelfTest(float speed, uint32_t delayMs, uint32_t runMs, uint32_t pauseMs)
      : speed_(speed), delayMs_(delayMs), runMs_(runMs), pauseMs_(pauseMs) {}

  void start(uint32_t now) {
    start_ = now;
    active_ = true;
    step_ = -1;
  }
  void abort() { active_ = false; }
  bool active() const { return active_; }
  // -1: カウントダウン中、0〜kSteps-1: 実行中の工程、kSteps: 終了
  int step() const { return step_; }
  // 待ち時間の残り [ms]（カウントダウン中のみ意味がある）
  uint32_t countdownLeft(uint32_t now) const;
  // 現在の目標速度を返す。全工程が終わると active() が false になる
  void update(uint32_t now, float* l, float* r);
  const char* stepName(int i) const;

 private:
  float speed_;
  uint32_t delayMs_, runMs_, pauseMs_;
  uint32_t start_ = 0;
  bool active_ = false;
  int step_ = -1;
};

}  // namespace cugo
