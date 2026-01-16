#include <Arduino.h>
#include <math.h>
#include "config.hpp"
#include "shared.hpp"
#include "motor.hpp"
#include "pid_velocity.hpp"

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

  const float kF = 255.0f / MAX_TICKS_S;
  const float kS = 25.0f;

  float pwm_ff = kF * target;
  if (fabsf(target) > 1.0f)
  {
    pwm_ff += (target > 0.0f) ? kS : -kS;
  }
  else
  {
    pwm_ff = 0.0f;
  }

  const float I_PWM_MAX = 80.0f;

  if (fabsf(ki) < 1e-9f)
  {
    s.iTerm = 0.0f;
  }
  else
  {
    float iPWM_now = ki * s.iTerm;
    if (iPWM_now > I_PWM_MAX)
      iPWM_now = I_PWM_MAX;
    if (iPWM_now < -I_PWM_MAX)
      iPWM_now = -I_PWM_MAX;

    float u_pre = pwm_ff + pTerm + iPWM_now + dTerm;

    bool sat_hi = (u_pre >= 255.0f);
    bool sat_lo = (u_pre <= -255.0f);

    bool allow_i = (!sat_hi && !sat_lo) ||
                   (sat_hi && e < 0.0f) ||
                   (sat_lo && e > 0.0f);

    if (allow_i)
      s.iTerm += e * dt;

    float iTerm_max = I_PWM_MAX / fabsf(ki);
    if (s.iTerm > iTerm_max)
      s.iTerm = iTerm_max;
    if (s.iTerm < -iTerm_max)
      s.iTerm = -iTerm_max;
  }

  float iTermPWM = ki * s.iTerm;
  if (iTermPWM > I_PWM_MAX)
    iTermPWM = I_PWM_MAX;
  if (iTermPWM < -I_PWM_MAX)
    iTermPWM = -I_PWM_MAX;

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

  const float k_cmd_to_ticks = MAX_TICKS_S / 127.0f;

  float targetFL = cFL * k_cmd_to_ticks;
  float targetFR = cFR * k_cmd_to_ticks;
  float targetBL = cBL * k_cmd_to_ticks;
  float targetBR = cBR * k_cmd_to_ticks;

  float u_raw_fl = 0.0f;

  int fl_out = computeMotorPID(targetFL, vFL, pidFL, kp, ki, kd, DT, &u_raw_fl);
  int fr_out = computeMotorPID(targetFR, vFR, pidFR, kp, ki, kd, DT, nullptr);
  int bl_out = computeMotorPID(targetBL, vBL, pidBL, kp, ki, kd, DT, nullptr);
  int br_out = computeMotorPID(targetBR, vBR, pidBR, kp, ki, kd, DT, nullptr);

  setAll(fl_out, fr_out, bl_out, br_out);

  noInterrupts();
  tele_pwm_fl = (int32_t)fl_out;
  interrupts();
}