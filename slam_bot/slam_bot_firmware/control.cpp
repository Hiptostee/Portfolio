#include <Arduino.h>
#include <math.h>

#include "config.hpp"
#include "shared.hpp"
#include "hold.hpp"
#include "pid_velocity.hpp"
#include "control.hpp"

void controlInit()
{
}

void controlTick()
{
  static uint32_t lastMs = 0;
  uint32_t now = millis();
  if (now - lastMs < CONTROL_MS)
    return;
  lastMs = now;

  if (hold_reset_req)
  {
    noInterrupts();
    hold_reset_req = false;
    interrupts();
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
  static bool init = false;
  static int32_t lastFL = 0, lastFR = 0, lastBL = 0, lastBR = 0;

  if (!init)
  {
    lastFL = eFL;
    lastFR = eFR;
    lastBL = eBL;
    lastBR = eBR;
    init = true;
    return;
  }

  int32_t dFL = eFL - lastFL;
  int32_t dFR = eFR - lastFR;
  int32_t dBL = eBL - lastBL;
  int32_t dBR = eBR - lastBR;

  lastFL = eFL;
  lastFR = eFR;
  lastBL = eBL;
  lastBR = eBR;

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

    const float ticks_per_cmd = MAX_TICKS_S / 127.0f;

    int cFL = (int)lroundf(vtFL / ticks_per_cmd);
    int cFR = (int)lroundf(vtFR / ticks_per_cmd);
    int cBL = (int)lroundf(vtBL / ticks_per_cmd);
    int cBR = (int)lroundf(vtBR / ticks_per_cmd);

    if (cFL > 127)
      cFL = 127;
    if (cFL < -127)
      cFL = -127;
    if (cFR > 127)
      cFR = 127;
    if (cFR < -127)
      cFR = -127;
    if (cBL > 127)
      cBL = 127;
    if (cBL < -127)
      cBL = -127;
    if (cBR > 127)
      cBR = 127;
    if (cBR < -127)
      cBR = -127;

    noInterrupts();
    cmdFL = (int8_t)cFL;
    cmdFR = (int8_t)cFR;
    cmdBL = (int8_t)cBL;
    cmdBR = (int8_t)cBR;

    tele_tpos_fl = tFL;
    tele_pos_fl = eFL;
    tele_err_fl = (tFL - eFL);
    interrupts();
  }
  else
  {
    noInterrupts();
    tele_tpos_fl = 0;
    tele_pos_fl = eFL;
    tele_err_fl = 0;
    interrupts();
  }

  runVelocityPID(vFL, vFR, vBL, vBR);
}