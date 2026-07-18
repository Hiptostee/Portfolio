#include <Wire.h>

#include "shared.hpp"

volatile int32_t encFL = 0;
volatile int32_t encFR = 0;
volatile int32_t encBL = 0;
volatile int32_t encBR = 0;

volatile int8_t cmdFL = 0;
volatile int8_t cmdFR = 0;
volatile int8_t cmdBL = 0;
volatile int8_t cmdBR = 0;

volatile uint8_t currentReg = 0;

volatile int16_t kp_q = 500;
volatile int16_t ki_q = 1;
volatile int16_t kd_q = 0;
volatile int16_t kp_q_hold = 5000;
volatile int16_t ki_q_hold = 0;
volatile int16_t kd_q_hold = 0;

volatile int32_t tele_tpos_fl = 0;
volatile int32_t tele_pos_fl = 0;
volatile int32_t tele_err_fl = 0;
volatile int32_t tele_pwm_fl = 0;
volatile int32_t tele_tpos_fr = 0;
volatile int32_t tele_pos_fr = 0;
volatile int32_t tele_err_fr = 0;
volatile int32_t tele_pwm_fr = 0;
volatile int32_t tele_tpos_bl = 0;
volatile int32_t tele_pos_bl = 0;
volatile int32_t tele_err_bl = 0;
volatile int32_t tele_pwm_bl = 0;
volatile int32_t tele_tpos_br = 0;
volatile int32_t tele_pos_br = 0;
volatile int32_t tele_err_br = 0;
volatile int32_t tele_pwm_br = 0;

volatile int32_t target_pos_FL = 0;
volatile int32_t target_pos_FR = 0;
volatile int32_t target_pos_BL = 0;
volatile int32_t target_pos_BR = 0;

volatile ControlMode mode = MODE_DRIVE;
volatile bool hold_reset_req = false;
volatile bool was_moving = false;

MotorState pidFL;
MotorState pidFR;
MotorState pidBL;
MotorState pidBR;

HoldState holdFL;
HoldState holdFR;
HoldState holdBL;
HoldState holdBR;

int clamp255(int v)
{
  if (v > 255)
    return 255;
  if (v < -255)
    return -255;
  return v;
}

int16_t read_i16_le(TwoWire &w)
{
  const uint8_t lo = (uint8_t)w.read();
  const uint8_t hi = (uint8_t)w.read();
  return (int16_t)((uint16_t)lo | ((uint16_t)hi << 8));
}
