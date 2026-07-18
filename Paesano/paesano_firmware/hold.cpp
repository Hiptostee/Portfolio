#include <Arduino.h>
#include <math.h>
#include "config.hpp"
#include "shared.hpp"
#include "hold.hpp"

namespace
{
constexpr float kHoldIntegralLimit = 20000.0f;
constexpr float kHoldMaxVelocity = 800.0f;
constexpr int32_t kHoldDeadbandTicks = 6;

int32_t absTicks(int32_t value)
{
  return value < 0 ? -value : value;
}
}  // namespace

void resetHoldPIDStates()
{
  holdFL = HoldState{};
  holdFR = HoldState{};
  holdBL = HoldState{};
  holdBR = HoldState{};
}

float computeHoldVelocity(int32_t targetPos, int32_t pos, HoldState &s,
                          float kp_hold, float ki_hold, float kd_hold)
{
  int32_t err = targetPos - pos;

  if (s.first)
  {
    s.first = false;
    s.lastErr = err;
    return 0.0f;
  }

  s.iTerm += (float)err * DT;

  if (s.iTerm > kHoldIntegralLimit)
    s.iTerm = kHoldIntegralLimit;
  if (s.iTerm < -kHoldIntegralLimit)
    s.iTerm = -kHoldIntegralLimit;

  float dErr = ((float)(err - s.lastErr)) / DT;
  s.lastErr = err;

  float v_cmd = kp_hold * (float)err + ki_hold * s.iTerm + kd_hold * dErr;
  if (!isfinite(v_cmd))
    v_cmd = 0.0f;

  if (v_cmd > kHoldMaxVelocity)
    v_cmd = kHoldMaxVelocity;
  if (v_cmd < -kHoldMaxVelocity)
    v_cmd = -kHoldMaxVelocity;

  if (absTicks(err) < kHoldDeadbandTicks)
    v_cmd = 0.0f;

  return v_cmd;
}
