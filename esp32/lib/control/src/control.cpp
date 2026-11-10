#include "control.h"

#include <cctype>
#include <cmath>
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

bool Motion::setTarget(float l, float r) {
  if (!std::isfinite(l) || !std::isfinite(r)) return false;
  rawL_ = clampUnit(l);
  rawR_ = clampUnit(r);
  tgtL_ = rawL_ * maxSpeed_;
  tgtR_ = rawR_ * maxSpeed_;
  return true;
}

bool Motion::setMaxSpeed(float v) {
  if (!std::isfinite(v) || v < 0.0f || v > 1.0f) return false;
  maxSpeed_ = v;
  tgtL_ = rawL_ * maxSpeed_;
  tgtR_ = rawR_ * maxSpeed_;
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
  Command c{CommandType::Invalid, 0.0f, 0.0f, 0.0f};
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

}  // namespace cugo
