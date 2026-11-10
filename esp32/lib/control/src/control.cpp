#include "control.h"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace cugo {

namespace {

float clampUnit(float v) { return v < -1.0f ? -1.0f : (v > 1.0f ? 1.0f : v); }

float approach(float current, float target, float step) {
  if (std::fabs(target - current) <= step) return target;
  return current + (target > current ? step : -step);
}

// "key" の値の先頭を返す（見つからなければ nullptr）
const char* findValue(const char* s, const char* quotedKey) {
  const char* p = std::strstr(s, quotedKey);
  if (!p) return nullptr;
  p += std::strlen(quotedKey);
  while (std::isspace(static_cast<unsigned char>(*p))) p++;
  if (*p != ':') return nullptr;
  p++;
  while (std::isspace(static_cast<unsigned char>(*p))) p++;
  return p;
}

bool parseNumber(const char* p, float* out) {
  if (!p) return false;
  char* end = nullptr;
  double v = std::strtod(p, &end);
  if (end == p || !std::isfinite(v)) return false;
  *out = static_cast<float>(v);
  return std::isfinite(*out);
}

struct StepDef {
  const char* name;
  float l, r;  // speed に掛ける係数
};

const StepDef kStepDefs[SelfTest::kSteps] = {
    {"左 前進", 1, 0},   {"左 後退", -1, 0},          {"右 前進", 0, 1},
    {"右 後退", 0, -1},  {"両方 前進", 1, 1},          {"両方 後退", -1, -1},
    {"その場で左旋回", -1, 1}, {"その場で右旋回", 1, -1},
};

}  // namespace

bool Motion::setTarget(float l, float r, bool scaled) {
  if (!std::isfinite(l) || !std::isfinite(r)) return false;
  rawL_ = clampUnit(l);
  rawR_ = clampUnit(r);
  scaled_ = scaled;
  float k = scaled_ ? maxSpeed_ : 1.0f;
  tgtL_ = rawL_ * k;
  tgtR_ = rawR_ * k;
  return true;
}

bool Motion::setMaxSpeed(float v) {
  if (!std::isfinite(v) || v < 0.0f || v > 1.0f) return false;
  maxSpeed_ = v;
  if (scaled_) {
    tgtL_ = rawL_ * maxSpeed_;
    tgtR_ = rawR_ * maxSpeed_;
  }
  return true;
}

void Motion::stop() { rawL_ = rawR_ = tgtL_ = tgtR_ = curL_ = curR_ = 0.0f; }

void Motion::tick(float dt) {
  if (!(dt > 0.0f)) return;
  if (dt > kMaxTickDt) dt = kMaxTickDt;
  float step = maxAccel_ * dt;
  curL_ = approach(curL_, tgtL_, step);
  curR_ = approach(curR_, tgtR_, step);
}

SideOutput toSideOutput(float v, bool invert) {
  if (invert) v = -v;
  return {std::fabs(v), v < 0.0f};
}

bool Watchdog::expired(uint32_t now) {
  if (!armed_ || now - last_ <= timeoutMs_) return false;
  armed_ = false;
  return true;
}

Role DriverArbiter::onConnect(uint8_t id, uint32_t now, uint8_t* kicked) {
  *kicked = kNoClient;
  if (driver_ != kNoClient && now - last_ <= takeoverMs_) return Role::Viewer;
  if (driver_ != kNoClient) *kicked = driver_;
  driver_ = id;
  last_ = now;
  return Role::Driver;
}

bool DriverArbiter::onDisconnect(uint8_t id) {
  if (!isDriver(id)) return false;
  driver_ = kNoClient;
  return true;
}

Command parseCommand(const char* text) {
  Command c{CommandType::Invalid, 0.0f, 0.0f, 0.0f, false};
  if (!text) return c;
  const char* stop = findValue(text, "\"stop\"");
  if (stop && std::strncmp(stop, "true", 4) == 0) {
    c.type = CommandType::Stop;
    return c;
  }
  if (const char* max = findValue(text, "\"max\"")) {
    if (parseNumber(max, &c.value)) c.type = CommandType::SetMax;
    return c;
  }
  if (const char* on = findValue(text, "\"auto\"")) {
    if (std::strncmp(on, "true", 4) == 0) {
      c.type = CommandType::SetAuto;
      c.on = true;
    } else if (std::strncmp(on, "false", 5) == 0) {
      c.type = CommandType::SetAuto;
      c.on = false;
    }
    return c;
  }
  if (!parseNumber(findValue(text, "\"l\""), &c.l) || !parseNumber(findValue(text, "\"r\""), &c.r)) {
    return c;
  }
  c.type = CommandType::Drive;
  return c;
}

uint32_t SelfTest::countdownLeft(uint32_t now) const {
  uint32_t elapsed = now - start_;
  return elapsed >= delayMs_ ? 0 : delayMs_ - elapsed;
}

void SelfTest::update(uint32_t now, float* l, float* r) {
  *l = *r = 0.0f;
  if (!active_) return;
  uint32_t elapsed = now - start_;
  if (elapsed < delayMs_) {
    step_ = -1;
    return;
  }
  uint32_t t = elapsed - delayMs_;
  uint32_t slot = runMs_ + pauseMs_;
  uint32_t i = t / slot;
  if (i >= static_cast<uint32_t>(kSteps)) {
    step_ = kSteps;
    active_ = false;
    return;
  }
  step_ = static_cast<int>(i);
  if (t % slot < runMs_) {
    *l = kStepDefs[i].l * speed_;
    *r = kStepDefs[i].r * speed_;
  }
}

const char* SelfTest::stepName(int i) const {
  return (i >= 0 && i < kSteps) ? kStepDefs[i].name : nullptr;
}

// ---- シリアルプロトコル ----

namespace {

int hexDigit(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

// [+-]?(digits)?(.digits)? の 10 進数（数字が 1 つ以上）だけを受け付ける。
// nan / inf / 16 進 / 指数表記 / 空白は不正
bool parseDecimal(const char* begin, const char* end, float* out) {
  const char* p = begin;
  if (p < end && (*p == '+' || *p == '-')) p++;
  int digits = 0;
  while (p < end && std::isdigit(static_cast<unsigned char>(*p))) p++, digits++;
  if (p < end && *p == '.') {
    p++;
    while (p < end && std::isdigit(static_cast<unsigned char>(*p))) p++, digits++;
  }
  if (p != end || digits == 0) return false;
  char tmp[32];
  size_t n = static_cast<size_t>(end - begin);
  if (n >= sizeof(tmp)) return false;
  std::memcpy(tmp, begin, n);
  tmp[n] = '\0';
  double v = std::strtod(tmp, nullptr);
  if (!std::isfinite(v)) return false;
  *out = static_cast<float>(v);
  return std::isfinite(*out);
}

// -0.000 にならないよう、丸めて 0 になる値は 0 にする
float noNegZero(float v, float half) { return std::fabs(v) < half ? 0.0f : v; }

}  // namespace

uint8_t protoChecksum(const char* body, size_t len) {
  uint8_t x = 0;
  for (size_t i = 0; i < len; i++) x ^= static_cast<uint8_t>(body[i]);
  return x;
}

SerialCommand parseSerialLine(const char* line) {
  SerialCommand c{SerialCommandType::Invalid, 0.0f, 0.0f};
  if (!line || line[0] != '$') return c;
  size_t len = std::strlen(line);
  if (len > 0 && line[len - 1] == '\r') len--;
  // 末尾は "*HH"
  if (len < 4 || line[len - 3] != '*') return c;
  const char* body = line + 1;
  size_t bodyLen = len - 4;
  if (std::memchr(body, '*', bodyLen) || std::memchr(body, '$', bodyLen)) return c;
  int hi = hexDigit(line[len - 2]);
  int lo = hexDigit(line[len - 1]);
  if (hi < 0 || lo < 0) return c;
  if (protoChecksum(body, bodyLen) != static_cast<uint8_t>(hi * 16 + lo)) return c;

  const char* end = body + bodyLen;
  if (bodyLen == 1 && body[0] == 'S') {
    c.type = SerialCommandType::Stop;
    return c;
  }
  if (bodyLen < 2 || body[0] != 'D' || body[1] != ',') return c;
  const char* a = body + 2;
  const char* comma = static_cast<const char*>(std::memchr(a, ',', static_cast<size_t>(end - a)));
  if (!comma) return c;
  const char* b = comma + 1;
  if (std::memchr(b, ',', static_cast<size_t>(end - b))) return c;
  float l, r;
  if (!parseDecimal(a, comma, &l) || !parseDecimal(b, end, &r)) return c;
  c.type = SerialCommandType::Drive;
  c.l = clampUnit(l);
  c.r = clampUnit(r);
  return c;
}

size_t formatStatus(char* buf, size_t size, Mode mode, float l, float r, float cap, unsigned flags,
                    uint32_t bad) {
  if (!buf || size == 0) return 0;
  char body[96];
  int n = std::snprintf(body, sizeof(body), "T,%c,%.3f,%.3f,%.2f,%u,%lu", static_cast<char>(mode),
                        static_cast<double>(noNegZero(l, 0.0005f)), static_cast<double>(noNegZero(r, 0.0005f)),
                        static_cast<double>(noNegZero(cap, 0.005f)), flags, static_cast<unsigned long>(bad));
  if (n < 0 || static_cast<size_t>(n) >= sizeof(body)) {
    buf[0] = '\0';
    return 0;
  }
  int m = std::snprintf(buf, size, "$%s*%02X\n", body, protoChecksum(body, static_cast<size_t>(n)));
  if (m < 0 || static_cast<size_t>(m) >= size) {
    buf[0] = '\0';
    return 0;
  }
  return static_cast<size_t>(m);
}

// ---- モード調停 ----

void ModeArbiter::driverConnected() {
  hasDriver_ = true;
  autoAllowed_ = false;
  webL_ = webR_ = 0.0f;
  web_.clear();
}

void ModeArbiter::driverDisconnected() {
  hasDriver_ = false;
  autoAllowed_ = true;
  webL_ = webR_ = 0.0f;
  web_.clear();
}

void ModeArbiter::webDrive(uint32_t now, float l, float r) {
  if (!std::isfinite(l) || !std::isfinite(r)) return;
  webL_ = clampUnit(l);
  webR_ = clampUnit(r);
  web_.feed(now);
  if (autoAllowed_ && (std::fabs(webL_) > takeoverInput_ || std::fabs(webR_) > takeoverInput_)) {
    autoAllowed_ = false;
  }
}

void ModeArbiter::webStop(uint32_t now) {
  webL_ = webR_ = 0.0f;
  web_.feed(now);
  autoAllowed_ = false;
}

bool ModeArbiter::jetsonDrive(uint32_t now, float l, float r) {
  if (!std::isfinite(l) || !std::isfinite(r)) return false;
  jetL_ = clampUnit(l);
  jetR_ = clampUnit(r);
  jetson_.feed(now);
  return true;
}

void ModeArbiter::jetsonStop(uint32_t now) {
  jetL_ = jetR_ = 0.0f;
  jetson_.feed(now);
}

Decision ModeArbiter::update(uint32_t now, bool selfTestActive) {
  Mode prev = mode_;
  bool webLost = web_.checkExpired(now);
  bool jetsonLost = jetson_.checkExpired(now);

  Decision d{Mode::Idle, 0.0f, 0.0f, true, false};
  if (selfTestActive) {
    d.mode = Mode::Test;
  } else if (hasDriver_ && !autoAllowed_) {
    d.mode = Mode::Manual;
    if (web_.alive(now)) {
      d.l = webL_;
      d.r = webR_;
    }
    d.stopNow = prev == Mode::Manual && webLost;
  } else if (autoAllowed_ && jetson_.alive(now)) {
    d.mode = Mode::Auto;
    d.l = std::fmax(-autoMax_, std::fmin(autoMax_, jetL_));
    d.r = std::fmax(-autoMax_, std::fmin(autoMax_, jetR_));
    d.scaled = false;
  } else {
    d.mode = Mode::Idle;
    d.stopNow = prev == Mode::Auto && jetsonLost;
  }
  mode_ = d.mode;
  return d;
}

unsigned ModeArbiter::flags(uint32_t now) const {
  unsigned f = 0;
  if (autoAllowed_) f |= kFlagAutoAllowed;
  if (hasDriver_) f |= kFlagDriver;
  if (jetson_.alive(now)) f |= kFlagJetson;
  return f;
}

}  // namespace cugo
