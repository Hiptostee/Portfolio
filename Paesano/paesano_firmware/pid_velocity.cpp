#include <Arduino.h>
#include <math.h>
#include "config.hpp"
#include "shared.hpp"
#include "motor.hpp"
#include "pid_velocity.hpp"

namespace
{
constexpr float kFeedforwardPwmPerTick = 255.0f / MAX_TICKS_S;
constexpr float kStaticFrictionPwm = 25.0f;
constexpr float kIntegralPwmLimit = 80.0f;
constexpr int kCommandLimit = 127;
constexpr float kCommandToTicks = MAX_TICKS_S / (float)kCommandLimit;

float clampIntegralPwm(float value)
{
  if (value > kIntegralPwmLimit)
    return kIntegralPwmLimit;
  if (value < -kIntegralPwmLimit)
    return -kIntegralPwmLimit;
  return value;
}
}  // namespace

void resetVelocityPIDStates()
{
  pidFL = MotorState{};
  pidFR = MotorState{};
  pidBL = MotorState{};
  pidBR = MotorState{};
}

int computeMotorPID(float target, float meas, MotorState &s,
                    float kp, float ki, float kd, float dt,
                    float *u_raw_out)
{
  float e = target - meas;

  float dMeas = 0.0f;
  if (!s.first)
    dMeas = (meas - s.lastMeas) / dt;
  else
    s.first = false;
  s.lastMeas = meas;

  float pTerm = kp * e;
  float dTerm = -kd * dMeas;

  float pwm_ff = kFeedforwardPwmPerTick * target;
  if (fabsf(target) > 1.0f)
  {
    pwm_ff += (target > 0.0f) ? kStaticFrictionPwm : -kStaticFrictionPwm;
  }
  else
  {
    pwm_ff = 0.0f;
  }

  float iTermPWM = 0.0f;

  if (fabsf(ki) < 1e-9f)
  {
    s.iTerm = 0.0f;
  }
  else
  {
    iTermPWM = clampIntegralPwm(ki * s.iTerm);

    float u_pre = pwm_ff + pTerm + iTermPWM + dTerm;

    bool sat_hi = (u_pre >= 255.0f);
    bool sat_lo = (u_pre <= -255.0f);

    bool allow_i = (!sat_hi && !sat_lo) ||
                   (sat_hi && e < 0.0f) ||
                   (sat_lo && e > 0.0f);

    if (allow_i)
      s.iTerm += e * dt;

    float iTerm_max = kIntegralPwmLimit / fabsf(ki);
    if (s.iTerm > iTerm_max)
      s.iTerm = iTerm_max;
    if (s.iTerm < -iTerm_max)
      s.iTerm = -iTerm_max;

    iTermPWM = clampIntegralPwm(ki * s.iTerm);
  }

  float u = pwm_ff + pTerm + iTermPWM + dTerm;

  if (u_raw_out)
    *u_raw_out = u;

  return clamp255((int)lroundf(u));
}

void runVelocityPID(float vFL, float vFR, float vBL, float vBR)
{
  int8_t cFL, cFR, cBL, cBR;
  int16_t kpq, kiq, kdq;

  noInterrupts();
  cFL = cmdFL;
  cFR = cmdFR;
  cBL = cmdBL;
  cBR = cmdBR;
  kpq = kp_q;
  kiq = ki_q;
  kdq = kd_q;
  interrupts();

  if (cFL == 0 && cFR == 0 && cBL == 0 && cBR == 0)
  {
    resetVelocityPIDStates();
    setAll(0, 0, 0, 0);
    return;
  }

  const float kp = ((float)kpq) / 1000.0f;
  const float ki = ((float)kiq) / 100000.0f;
  const float kd = ((float)kdq) / 10.0f;

  float targetFL = cFL * kCommandToTicks;
  float targetFR = cFR * kCommandToTicks;
  float targetBL = cBL * kCommandToTicks;
  float targetBR = cBR * kCommandToTicks;

  float u_raw_fl = 0.0f;
  float u_raw_fr = 0.0f;
  float u_raw_bl = 0.0f;
  float u_raw_br = 0.0f;

  int fl_out = computeMotorPID(targetFL, vFL, pidFL, kp, ki, kd, DT, &u_raw_fl);
  int fr_out = computeMotorPID(targetFR, vFR, pidFR, kp, ki, kd, DT, &u_raw_fr);
  int bl_out = computeMotorPID(targetBL, vBL, pidBL, kp, ki, kd, DT, &u_raw_bl);
  int br_out = computeMotorPID(targetBR, vBR, pidBR, kp, ki, kd, DT, &u_raw_br);

  setAll(fl_out, fr_out, bl_out, br_out);

  noInterrupts();
  tele_pwm_fl = (int32_t)lroundf(u_raw_fl);
  tele_pwm_fr = (int32_t)lroundf(u_raw_fr);
  tele_pwm_bl = (int32_t)lroundf(u_raw_bl);
  tele_pwm_br = (int32_t)lroundf(u_raw_br);
  interrupts();
}
