// 操縦ロジック（Arduino 非依存。PC 上で単体テストできる）
#pragma once

#include <cstddef>
#include <cstdint>

namespace cugo {

// 制御が停滞した後でも 1 回の tick で進める時間の上限 [s]（加速度制限の飛び越し防止）
constexpr float kMaxTickDt = 0.1f;

// 左右速度 (-1.0〜1.0) を加速度制限つきで目標へ近づける
class Motion {
 public:
  Motion(float maxSpeed, float maxAccel) : maxSpeed_(maxSpeed), maxAccel_(maxAccel) {}

  // ±1 にクランプして max_speed を掛ける（scaled = false なら掛けない。自律用）。非有限値は拒否して false
  bool setTarget(float l, float r, bool scaled = true);
  // 加速度制限を無視して即座に 0
  void stop();
  void tick(float dt);
  // 出力上限 (0〜1) を変更し、現在の目標にも即座に反映する。範囲外・非有限値は false
  bool setMaxSpeed(float v);

  float maxSpeed() const { return maxSpeed_; }
  float left() const { return curL_; }
  float right() const { return curR_; }
  float targetLeft() const { return tgtL_; }
  float targetRight() const { return tgtR_; }

 private:
  float maxSpeed_;
  float maxAccel_;
  float rawL_ = 0, rawR_ = 0;  // 上限を掛ける前の入力
  bool scaled_ = true;         // 目標に max_speed を掛けているか
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

enum class CommandType { Invalid, Stop, Drive, SetMax, SetAuto };
struct Command {
  CommandType type;
  float l;
  float r;
  float value;  // SetMax の値
  bool on;      // SetAuto の値
};
// {"l":0.5,"r":-0.5} / {"stop":true} / {"max":0.5} / {"auto":true|false} を解釈する
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

// ---- Jetson とのシリアルプロトコル（v1）: $<BODY>*<HH> ----

// BODY の全バイトの XOR
uint8_t protoChecksum(const char* body, size_t len);

enum class SerialCommandType { Invalid, Drive, Stop };
struct SerialCommand {
  SerialCommandType type;
  float l;  // Drive の左右 duty（±1 にクランプ済み）
  float r;
};
// 改行を除いた 1 行（末尾の CR は可）を解釈する。チェックサム不一致・形式不正は Invalid
SerialCommand parseSerialLine(const char* line);

enum class Mode : char { Idle = 'I', Auto = 'A', Manual = 'M', Test = 'X' };

// ステータス行 "$T,<mode>,<l>,<r>,<cap>,<flags>,<bad>*HH\n" を書く。
// 書いた長さを返す（バッファ不足なら 0 と空文字列）
size_t formatStatus(char* buf, size_t size, Mode mode, float l, float r, float cap, unsigned flags,
                    uint32_t bad);

enum class LineEvent { None, DebugChar, Line, Overflow };

// シリアル受信を '$' 行と 1 文字のデバッグコマンドに分ける。
// '$' 行の受信中でないときに届いた文字はデバッグコマンド（改行は無視）。
// N-1 文字を超える行は捨て、行末で 1 回だけ Overflow を返す
template <size_t N>
class LineBuffer {
 public:
  LineEvent feed(char c) {
    if (!inLine_) {
      if (c == '$') {
        inLine_ = true;
        overflow_ = false;
        len_ = 0;
        buf_[len_++] = c;
        return LineEvent::None;
      }
      if (c == '\r' || c == '\n') return LineEvent::None;
      debug_ = c;
      return LineEvent::DebugChar;
    }
    if (c == '\n') {
      inLine_ = false;
      if (overflow_) return LineEvent::Overflow;
      if (len_ > 0 && buf_[len_ - 1] == '\r') len_--;
      buf_[len_] = '\0';
      return LineEvent::Line;
    }
    if (overflow_) return LineEvent::None;
    if (len_ >= N - 1) {
      overflow_ = true;
      return LineEvent::None;
    }
    buf_[len_++] = c;
    return LineEvent::None;
  }
  const char* line() const { return buf_; }
  char debugChar() const { return debug_; }

 private:
  static_assert(N >= 2, "LineBuffer too small");
  char buf_[N] = {};
  size_t len_ = 0;
  bool inLine_ = false;
  bool overflow_ = false;
  char debug_ = 0;
};

// 一定時間内に指令を受けているか（millis() の桁あふれ対応。途絶後は次の feed まで復活しない）
class Liveness {
 public:
  explicit Liveness(uint32_t timeoutMs) : timeoutMs_(timeoutMs) {}
  void feed(uint32_t now) {
    last_ = now;
    armed_ = true;
  }
  void clear() { armed_ = false; }
  // 途絶を検知したら false にして true を返す（1 回だけ）
  bool checkExpired(uint32_t now) {
    if (!armed_ || now - last_ <= timeoutMs_) return false;
    armed_ = false;
    return true;
  }
  // 状態を変えずに確認する（途絶の検知は checkExpired で行う）
  bool alive(uint32_t now) const { return armed_ && now - last_ <= timeoutMs_; }

 private:
  uint32_t timeoutMs_;
  uint32_t last_ = 0;
  bool armed_ = false;
};

// ステータスの flags
constexpr unsigned kFlagAutoAllowed = 1u << 0;
constexpr unsigned kFlagDriver = 1u << 1;
constexpr unsigned kFlagJetson = 1u << 2;

struct Decision {
  Mode mode;
  float l, r;    // 目標（Test のときは 0。呼び出し側でテストの値を使う）
  bool scaled;   // Web の出力上限を掛けるか
  bool stopNow;  // 使用中の指令源が途絶えた（加速度制限を無視して即座に停止する）
};

// どの指令源で走るかを決める。優先順位: 動作テスト > Web 手動 > Jetson 自律
class ModeArbiter {
 public:
  ModeArbiter(uint32_t webTimeoutMs, uint32_t jetsonTimeoutMs, float autoMax, float takeoverInput)
      : web_(webTimeoutMs), jetson_(jetsonTimeoutMs), autoMax_(autoMax), takeoverInput_(takeoverInput) {}

  // Web 操縦者の接続（引き継ぎを含む）: 手動にする
  void driverConnected();
  // Web 操縦者の切断: 自律可に戻す（停止は呼び出し側）
  void driverDisconnected();
  void setAutoAllowed(bool on) { autoAllowed_ = on; }
  // Web の走行指令。自律可のときに 0 でない入力が来たら手動に奪還する
  void webDrive(uint32_t now, float l, float r);
  // 停止ボタン: Web 指令を 0 にして手動にする（即時停止は呼び出し側）
  void webStop(uint32_t now);
  // Web のその他の有効な指令（ウォッチドッグだけ更新）
  void webTouch(uint32_t now) { web_.feed(now); }
  // Jetson の $D。非有限値は拒否して false
  bool jetsonDrive(uint32_t now, float l, float r);
  // Jetson の $S: 指令を 0 にする（即時停止は呼び出し側）
  void jetsonStop(uint32_t now);

  // 制御周期ごとに呼ぶ
  Decision update(uint32_t now, bool selfTestActive);

  Mode mode() const { return mode_; }
  bool autoAllowed() const { return autoAllowed_; }
  bool hasDriver() const { return hasDriver_; }
  bool jetsonAlive(uint32_t now) const { return jetson_.alive(now); }
  float cap() const { return autoMax_; }
  // ステータスの flags（now 時点のウォッチドッグを反映）
  unsigned flags(uint32_t now) const;

 private:
  Liveness web_;
  Liveness jetson_;
  float autoMax_;
  float takeoverInput_;
  bool hasDriver_ = false;
  bool autoAllowed_ = true;
  float webL_ = 0, webR_ = 0;
  float jetL_ = 0, jetR_ = 0;
  Mode mode_ = Mode::Idle;
};

}  // namespace cugo
