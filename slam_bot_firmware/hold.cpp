#include <Arduino.h>
#include <math.h>
#include "config.hpp"
#include "shared.hpp"
#include "hold.hpp"

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
  }

  s.iTerm += (float)err * DT;

  const float ITERM_MAX = 20000.0f;
  if (s.iTerm > ITERM_MAX)
    s.iTerm = ITERM_MAX;
  if (s.iTerm < -ITERM_MAX)
    s.iTerm = -ITERM_MAX;

  float dErr = ((float)(err - s.lastErr)) / DT;
  s.lastErr = err;

  float v_cmd = kp_hold * (float)err + ki_hold * s.iTerm + kd_hold * dErr;

  const float VMAX = 800.0f;
  if (v_cmd > VMAX)
    v_cmd = VMAX;
  if (v_cmd < -VMAX)
    v_cmd = -VMAX;

  if (abs(err) < 6)
    v_cmd = 0.0f;

  return v_cmd;
}