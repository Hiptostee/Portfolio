#include <Arduino.h>
#include <Wire.h>

#include "config.hpp"
#include "shared.hpp"
#include "i2c_iface.hpp"

static void pack_i32_le(uint8_t *out, int idx, int32_t v)
{
  out[idx + 0] = (uint8_t)(v & 0xFF);
  out[idx + 1] = (uint8_t)((v >> 8) & 0xFF);
  out[idx + 2] = (uint8_t)((v >> 16) & 0xFF);
  out[idx + 3] = (uint8_t)((v >> 24) & 0xFF);
}

static bool writeTelemetry(
  volatile int32_t &target_pos,
  volatile int32_t &position,
  volatile int32_t &error,
  volatile int32_t &pwm)
{
  int32_t tp, p, e, u;
  noInterrupts();
  tp = target_pos;
  p = position;
  e = error;
  u = pwm;
  interrupts();

  uint8_t out[16];
  pack_i32_le(out, 0, tp);
  pack_i32_le(out, 4, p);
  pack_i32_le(out, 8, e);
  pack_i32_le(out, 12, u);
  Wire1.write(out, 16);
  return true;
}

static void drain_extra_bytes(int count, const char *context)
{
  if (count > 0)
  {
    Serial.print("I2C extra bytes for ");
    Serial.print(context);
    Serial.print(": ");
    Serial.println(count);
  }
  while (count-- > 0)
    (void)Wire1.read();
}

static void onReceive(int n)
{
  if (n <= 0)
    return;

  currentReg = (uint8_t)Wire1.read();
  n--;

  if (currentReg == REG_MOTOR_SPEEDS && n >= 4)
  {
    int8_t fl = (int8_t)Wire1.read();
    int8_t fr = (int8_t)Wire1.read();
    int8_t bl = (int8_t)Wire1.read();
    int8_t br = (int8_t)Wire1.read();

    const bool all_zero = (fl == 0 && fr == 0 && bl == 0 && br == 0);

    noInterrupts();
    cmdFL = fl;
    cmdFR = fr;
    cmdBL = bl;
    cmdBR = br;

    if (!all_zero)
    {
      mode = MODE_DRIVE;
      was_moving = true;
      hold_reset_req = true; // leaving hold -> clear hold integrators
    }
    else
    {
      if (was_moving)
      {
        target_pos_FL = encFL;
        target_pos_FR = encFR;
        target_pos_BL = encBL;
        target_pos_BR = encBR;

        mode = MODE_HOLD;
        was_moving = false;
        hold_reset_req = true; // reset hold integrators for clean latch
      }
    }
    interrupts();

    drain_extra_bytes(n - 4, "motor speeds");
    return;
  }

  if (currentReg == REG_PID_GAINS && n >= 12)
  {
    int16_t kp = read_i16_le(Wire1);
    int16_t ki = read_i16_le(Wire1);
    int16_t kd = read_i16_le(Wire1);
    int16_t kp_hold_temp = read_i16_le(Wire1);
    int16_t ki_hold_temp = read_i16_le(Wire1);
    int16_t kd_hold_temp = read_i16_le(Wire1);

    noInterrupts();
    kp_q = kp;
    ki_q = ki;
    kd_q = kd;
    kp_q_hold = kp_hold_temp;
    ki_q_hold = ki_hold_temp;
    kd_q_hold = kd_hold_temp;
    interrupts();

    drain_extra_bytes(n - 12, "PID gains");
    return;
  }

  Serial.print("I2C unknown register write: ");
  Serial.println(currentReg);
  while (n-- > 0)
    (void)Wire1.read();
}

static void onRequest()
{
  if (currentReg == REG_ENCODERS)
  {
    int32_t fl, fr, bl, br;
    noInterrupts();
    fl = encFL;
    fr = encFR;
    bl = encBL;
    br = encBR;
    interrupts();

    uint8_t out[16];
    pack_i32_le(out, 0, fl);
    pack_i32_le(out, 4, fr);
    pack_i32_le(out, 8, bl);
    pack_i32_le(out, 12, br);
    Wire1.write(out, 16);
    return;
  }

  if (currentReg == REG_TELEM_FL) {
    writeTelemetry(tele_tpos_fl, tele_pos_fl, tele_err_fl, tele_pwm_fl);
    return;
  }
  if (currentReg == REG_TELEM_FR) {
    writeTelemetry(tele_tpos_fr, tele_pos_fr, tele_err_fr, tele_pwm_fr);
    return;
  }
  if (currentReg == REG_TELEM_BL) {
    writeTelemetry(tele_tpos_bl, tele_pos_bl, tele_err_bl, tele_pwm_bl);
    return;
  }
  if (currentReg == REG_TELEM_BR) {
    writeTelemetry(tele_tpos_br, tele_pos_br, tele_err_br, tele_pwm_br);
    return;
  }

  Serial.print("I2C unknown register read: ");
  Serial.println(currentReg);
  Wire1.write((uint8_t)0);
}

void i2cInit()
{
  Wire1.setSDA(I2C_SDA_PIN);
  Wire1.setSCL(I2C_SCL_PIN);
  Wire1.begin(I2C_ADDR);
  Wire1.onReceive(onReceive);
  Wire1.onRequest(onRequest);
  Wire1.setClock(100000);
}
