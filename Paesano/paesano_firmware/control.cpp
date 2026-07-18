#include <Arduino.h>
#include <math.h>

#include "config.hpp"
#include "shared.hpp"
#include "hold.hpp"
#include "pid_velocity.hpp"
#include "control.hpp"

namespace
{
constexpr int kCommandLimit = 127;

bool velocity_initialized = false;
int32_t last_fl = 0;
int32_t last_fr = 0;
int32_t last_bl = 0;
int32_t last_br = 0;

int clampCommand(int value)
{
  if (value > kCommandLimit)
    return kCommandLimit;
  if (value < -kCommandLimit)
    return -kCommandLimit;
  return value;
}

int velocityToCommand(float velocity)
{
  constexpr float ticks_per_cmd = MAX_TICKS_S / (float)kCommandLimit;
  return clampCommand((int)lroundf(velocity / ticks_per_cmd));
}
}  // namespace

void controlInit()
{
  velocity_initialized = false;
  last_fl = 0;
  last_fr = 0;
  last_bl = 0;
  last_br = 0;
}

void controlTick()
{
  static uint32_t lastMs = 0;
  uint32_t now = millis();
  if (now - lastMs < CONTROL_MS)
    return;
  lastMs = now;

  bool reset_hold = false;
  noInterrupts();
  if (hold_reset_req)
  {
    hold_reset_req = false;
    reset_hold = true;
  }
  interrupts();

  if (reset_hold)
  {
    resetHoldPIDStates();
  }

  // snapshot encoder counts + targets + mode + hold gains
  int32_t eFL, eFR, eBL, eBR;
  int32_t tFL, tFR, tBL, tBR;
  ControlMode m;
  int16_t kp_h_q, ki_h_q, kd_h_q;

  noInterrupts();
  eFL = encFL;
  eFR = encFR;
  eBL = encBL;
  eBR = encBR;
  tFL = target_pos_FL;
  tFR = target_pos_FR;
  tBL = target_pos_BL;
  tBR = target_pos_BR;
  m = mode;
  kp_h_q = kp_q_hold;
  ki_h_q = ki_q_hold;
  kd_h_q = kd_q_hold;
  interrupts();

  // velocity estimate (ticks/sec)
  if (!velocity_initialized)
  {
    last_fl = eFL;
    last_fr = eFR;
    last_bl = eBL;
    last_br = eBR;
    velocity_initialized = true;
    return;
  }

  int32_t dFL = eFL - last_fl;
  int32_t dFR = eFR - last_fr;
  int32_t dBL = eBL - last_bl;
  int32_t dBR = eBR - last_br;

  last_fl = eFL;
  last_fr = eFR;
  last_bl = eBL;
  last_br = eBR;

  float vFL = dFL / DT;
  float vFR = dFR / DT;
  float vBL = dBL / DT;
  float vBR = dBR / DT;

  if (m == MODE_HOLD)
  {
    float kp_hold = ((float)kp_h_q) / 1000.0f;
    float ki_hold = ((float)ki_h_q) / 100000.0f;
    float kd_hold = ((float)kd_h_q) / 10.0f;

    float vtFL = computeHoldVelocity(tFL, eFL, holdFL, kp_hold, ki_hold, kd_hold);
    float vtFR = computeHoldVelocity(tFR, eFR, holdFR, kp_hold, ki_hold, kd_hold);
    float vtBL = computeHoldVelocity(tBL, eBL, holdBL, kp_hold, ki_hold, kd_hold);
    float vtBR = computeHoldVelocity(tBR, eBR, holdBR, kp_hold, ki_hold, kd_hold);

    int cFL = velocityToCommand(vtFL);
    int cFR = velocityToCommand(vtFR);
    int cBL = velocityToCommand(vtBL);
    int cBR = velocityToCommand(vtBR);

    noInterrupts();
    cmdFL = (int8_t)cFL;
    cmdFR = (int8_t)cFR;
    cmdBL = (int8_t)cBL;
    cmdBR = (int8_t)cBR;

    tele_tpos_fl = tFL;
    tele_pos_fl = eFL;
    tele_err_fl = (tFL - eFL);
    tele_tpos_fr = tFR;
    tele_pos_fr = eFR;
    tele_err_fr = (tFR - eFR);
    tele_tpos_bl = tBL;
    tele_pos_bl = eBL;
    tele_err_bl = (tBL - eBL);
    tele_tpos_br = tBR;
    tele_pos_br = eBR;
    tele_err_br = (tBR - eBR);
    interrupts();
  }
  else
  {
    noInterrupts();
    tele_tpos_fl = 0;
    tele_pos_fl = eFL;
    tele_err_fl = 0;
    tele_tpos_fr = 0;
    tele_pos_fr = eFR;
    tele_err_fr = 0;
    tele_tpos_bl = 0;
    tele_pos_bl = eBL;
    tele_err_bl = 0;
    tele_tpos_br = 0;
    tele_pos_br = eBR;
    tele_err_br = 0;
    interrupts();
  }

  runVelocityPID(vFL, vFR, vBL, vBR);
}
